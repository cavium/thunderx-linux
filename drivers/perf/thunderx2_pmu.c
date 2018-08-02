// SPDX-License-Identifier: GPL-2.0
/*
 * CAVIUM THUNDERX2 SoC PMU UNCORE
 * Copyright (C) 2018 Cavium Inc.
 * Author: Ganapatrao Kulkarni <gkulkarni@cavium.com>
 */

#include <linux/acpi.h>
#include <linux/arm-smccc.h>
#include <linux/cpuhotplug.h>
#include <linux/perf_event.h>
#include <linux/platform_device.h>

/* L3C and DMC has 16 and 8 channels per socket respectively.
 * Each Channel supports UNCORE PMU device and consists of
 * 4 independent programmable counters. Counters are 32 bit
 * and do not support overflow interrupt, they need to be
 * sampled before overflow(i.e, at every 2 seconds).
 */

#define UNCORE_MAX_COUNTERS		4
#define UNCORE_DMC_MAX_CHANNELS		8
#define UNCORE_L3_MAX_TILES		16

#define UNCORE_HRTIMER_INTERVAL		(2 * NSEC_PER_SEC)
#define GET_EVENTID(ev)			((ev->hw.config) & 0x1ff)
#define GET_COUNTERID(ev)		((ev->hw.idx) & 0xf)
#define GET_CHANNELID(pmu_uncore)	(pmu_uncore->channel)
#define DMC_EVENT_CFG(idx, val)		((val) << (((idx) * 8) + 1))

#define L3C_COUNTER_CTL			0xA8
#define L3C_COUNTER_DATA		0xAC
#define DMC_COUNTER_CTL			0x234
#define DMC_COUNTER_DATA		0x240

#define THUNDERX2_SMC_CALL_ID		0xC200FF00
#define THUNDERX2_SMC_SET_CHANNEL	0xB010

enum thunderx2_uncore_l3_events {
	L3_EVENT_NONE,
	L3_EVENT_NBU_CANCEL,
	L3_EVENT_DIB_RETRY,
	L3_EVENT_DOB_RETRY,
	L3_EVENT_DIB_CREDIT_RETRY,
	L3_EVENT_DOB_CREDIT_RETRY,
	L3_EVENT_FORCE_RETRY,
	L3_EVENT_IDX_CONFLICT_RETRY,
	L3_EVENT_EVICT_CONFLICT_RETRY,
	L3_EVENT_BANK_CONFLICT_RETRY,
	L3_EVENT_FILL_ENTRY_RETRY,
	L3_EVENT_EVICT_NOT_READY_RETRY,
	L3_EVENT_L3_RETRY,
	L3_EVENT_READ_REQ,
	L3_EVENT_WRITE_BACK_REQ,
	L3_EVENT_INVALIDATE_NWRITE_REQ,
	L3_EVENT_INV_REQ,
	L3_EVENT_SELF_REQ,
	L3_EVENT_REQ,
	L3_EVENT_EVICT_REQ,
	L3_EVENT_INVALIDATE_NWRITE_HIT,
	L3_EVENT_INVALIDATE_HIT,
	L3_EVENT_SELF_HIT,
	L3_EVENT_READ_HIT,
	L3_EVENT_MAX,
};

enum thunderx2_uncore_dmc_events {
	DMC_EVENT_NONE,
	DMC_EVENT_COUNT_CYCLES,
	DMC_EVENT_RES2,
	DMC_EVENT_RES3,
	DMC_EVENT_RES4,
	DMC_EVENT_RES5,
	DMC_EVENT_RES6,
	DMC_EVENT_RES7,
	DMC_EVENT_RES8,
	DMC_EVENT_READ_64B_TXNS,
	DMC_EVENT_READ_BELOW_64B_TXNS,
	DMC_EVENT_WRITE_TXNS,
	DMC_EVENT_TXN_CYCLES,
	DMC_EVENT_DATA_TRANSFERS,
	DMC_EVENT_CANCELLED_READ_TXNS,
	DMC_EVENT_CONSUMED_READ_TXNS,
	DMC_EVENT_MAX,
};

enum thunderx2_uncore_type {
	PMU_TYPE_L3C,
	PMU_TYPE_DMC,
	PMU_TYPE_INVALID,
};

/*
 * pmu on each socket has 2 uncore devices(dmc and l3),
 * each uncore device has up to 16 channels, each channel can sample
 * events independently with counters up to 4.
 */
struct thunderx2_pmu_uncore_channel {
	struct pmu pmu;
	struct hlist_node	node;
	struct thunderx2_pmu_uncore_dev *uncore_dev;
	int channel;
	int cpu;
	DECLARE_BITMAP(active_counters, UNCORE_MAX_COUNTERS);
	struct perf_event *events[UNCORE_MAX_COUNTERS];
	struct hrtimer hrtimer;
	/* to sync counter alloc/release */
	raw_spinlock_t lock;
};

struct thunderx2_pmu_uncore_dev {
	char *name;
	struct device *dev;
	enum thunderx2_uncore_type type;
	void __iomem *base;
	int node;
	u32    max_counters;
	u32    max_channels;
	u32    max_events;
	u64 hrtimer_interval;
	/* this lock synchronizes across channels */
	raw_spinlock_t lock;
	const struct attribute_group **attr_groups;
	void	(*init_cntr_base)(struct perf_event *event,
			struct thunderx2_pmu_uncore_dev *uncore_dev);
	void	(*select_channel)(struct perf_event *event);
	void	(*stop_event)(struct perf_event *event);
	void	(*start_event)(struct perf_event *event, int flags);
};

static inline struct thunderx2_pmu_uncore_channel *
pmu_to_thunderx2_pmu_uncore(struct pmu *pmu)
{
	return container_of(pmu, struct thunderx2_pmu_uncore_channel, pmu);
}

/*
 * sysfs format attributes
 */
static ssize_t thunderx2_pmu_format_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct dev_ext_attribute *eattr;

	eattr = container_of(attr, struct dev_ext_attribute, attr);
	return sprintf(buf, "%s\n", (char *) eattr->var);
}

#define FORMAT_ATTR(_name, _config) \
	(&((struct dev_ext_attribute[]) { \
	   { \
	   .attr = __ATTR(_name, 0444, thunderx2_pmu_format_show, NULL), \
	   .var = (void *) _config, \
	   } \
	})[0].attr.attr)

static struct attribute *l3c_pmu_format_attrs[] = {
	FORMAT_ATTR(event,	"config:0-4"),
	NULL,
};

static struct attribute *dmc_pmu_format_attrs[] = {
	FORMAT_ATTR(event,	"config:0-4"),
	NULL,
};

static const struct attribute_group l3c_pmu_format_attr_group = {
	.name = "format",
	.attrs = l3c_pmu_format_attrs,
};

static const struct attribute_group dmc_pmu_format_attr_group = {
	.name = "format",
	.attrs = dmc_pmu_format_attrs,
};

/*
 * sysfs event attributes
 */
static ssize_t thunderx2_pmu_event_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct dev_ext_attribute *eattr;

	eattr = container_of(attr, struct dev_ext_attribute, attr);
	return sprintf(buf, "config=0x%lx\n", (unsigned long) eattr->var);
}

#define EVENT_ATTR(_name, _config) \
	(&((struct dev_ext_attribute[]) { \
	   { \
	   .attr = __ATTR(_name, 0444, thunderx2_pmu_event_show, NULL), \
	   .var = (void *) _config, \
	   } \
	 })[0].attr.attr)

static struct attribute *l3c_pmu_events_attrs[] = {
	EVENT_ATTR(nbu_cancel,			L3_EVENT_NBU_CANCEL),
	EVENT_ATTR(dib_retry,			L3_EVENT_DIB_RETRY),
	EVENT_ATTR(dob_retry,			L3_EVENT_DOB_RETRY),
	EVENT_ATTR(dib_credit_retry,		L3_EVENT_DIB_CREDIT_RETRY),
	EVENT_ATTR(dob_credit_retry,		L3_EVENT_DOB_CREDIT_RETRY),
	EVENT_ATTR(force_retry,			L3_EVENT_FORCE_RETRY),
	EVENT_ATTR(idx_conflict_retry,		L3_EVENT_IDX_CONFLICT_RETRY),
	EVENT_ATTR(evict_conflict_retry,	L3_EVENT_EVICT_CONFLICT_RETRY),
	EVENT_ATTR(bank_conflict_retry,		L3_EVENT_BANK_CONFLICT_RETRY),
	EVENT_ATTR(fill_entry_retry,		L3_EVENT_FILL_ENTRY_RETRY),
	EVENT_ATTR(evict_not_ready_retry,	L3_EVENT_EVICT_NOT_READY_RETRY),
	EVENT_ATTR(l3_retry,			L3_EVENT_L3_RETRY),
	EVENT_ATTR(read_request,		L3_EVENT_READ_REQ),
	EVENT_ATTR(write_back_request,		L3_EVENT_WRITE_BACK_REQ),
	EVENT_ATTR(inv_nwrite_request,		L3_EVENT_INVALIDATE_NWRITE_REQ),
	EVENT_ATTR(inv_request,			L3_EVENT_INV_REQ),
	EVENT_ATTR(self_request,		L3_EVENT_SELF_REQ),
	EVENT_ATTR(request,			L3_EVENT_REQ),
	EVENT_ATTR(evict_request,		L3_EVENT_EVICT_REQ),
	EVENT_ATTR(inv_nwrite_hit,		L3_EVENT_INVALIDATE_NWRITE_HIT),
	EVENT_ATTR(inv_hit,			L3_EVENT_INVALIDATE_HIT),
	EVENT_ATTR(self_hit,			L3_EVENT_SELF_HIT),
	EVENT_ATTR(read_hit,			L3_EVENT_READ_HIT),
	NULL,
};

static struct attribute *dmc_pmu_events_attrs[] = {
	EVENT_ATTR(cnt_cycles,			DMC_EVENT_COUNT_CYCLES),
	EVENT_ATTR(read_64b_txns,		DMC_EVENT_READ_64B_TXNS),
	EVENT_ATTR(read_below_64b_txns,		DMC_EVENT_READ_BELOW_64B_TXNS),
	EVENT_ATTR(write_txns,			DMC_EVENT_WRITE_TXNS),
	EVENT_ATTR(txn_cycles,			DMC_EVENT_TXN_CYCLES),
	EVENT_ATTR(data_transfers,		DMC_EVENT_DATA_TRANSFERS),
	EVENT_ATTR(cancelled_read_txns,		DMC_EVENT_CANCELLED_READ_TXNS),
	EVENT_ATTR(consumed_read_txns,		DMC_EVENT_CONSUMED_READ_TXNS),
	NULL,
};

static const struct attribute_group l3c_pmu_events_attr_group = {
	.name = "events",
	.attrs = l3c_pmu_events_attrs,
};

static const struct attribute_group dmc_pmu_events_attr_group = {
	.name = "events",
	.attrs = dmc_pmu_events_attrs,
};

/*
 * sysfs cpumask attributes
 */
static ssize_t cpumask_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct cpumask cpu_mask;
	struct thunderx2_pmu_uncore_channel *pmu_uncore =
		pmu_to_thunderx2_pmu_uncore(dev_get_drvdata(dev));

	cpumask_clear(&cpu_mask);
	cpumask_set_cpu(pmu_uncore->cpu, &cpu_mask);
	return cpumap_print_to_pagebuf(true, buf, &cpu_mask);
}
static DEVICE_ATTR_RO(cpumask);

static struct attribute *thunderx2_pmu_cpumask_attrs[] = {
	&dev_attr_cpumask.attr,
	NULL,
};

static const struct attribute_group pmu_cpumask_attr_group = {
	.attrs = thunderx2_pmu_cpumask_attrs,
};

/*
 * Per PMU device attribute groups
 */
static const struct attribute_group *l3c_pmu_attr_groups[] = {
	&l3c_pmu_format_attr_group,
	&pmu_cpumask_attr_group,
	&l3c_pmu_events_attr_group,
	NULL
};

static const struct attribute_group *dmc_pmu_attr_groups[] = {
	&dmc_pmu_format_attr_group,
	&pmu_cpumask_attr_group,
	&dmc_pmu_events_attr_group,
	NULL
};

static inline u32 reg_readl(unsigned long addr)
{
	return readl((void __iomem *)addr);
}

static inline void reg_writel(u32 val, unsigned long addr)
{
	writel(val, (void __iomem *)addr);
}

static int alloc_counter(struct thunderx2_pmu_uncore_channel *pmu_uncore)
{
	int counter;

	raw_spin_lock(&pmu_uncore->lock);
	counter = find_first_zero_bit(pmu_uncore->active_counters,
				pmu_uncore->uncore_dev->max_counters);
	if (counter == pmu_uncore->uncore_dev->max_counters) {
		raw_spin_unlock(&pmu_uncore->lock);
		return -ENOSPC;
	}
	set_bit(counter, pmu_uncore->active_counters);
	raw_spin_unlock(&pmu_uncore->lock);
	return counter;
}

static void free_counter(
		struct thunderx2_pmu_uncore_channel *pmu_uncore, int counter)
{
	raw_spin_lock(&pmu_uncore->lock);
	clear_bit(counter, pmu_uncore->active_counters);
	raw_spin_unlock(&pmu_uncore->lock);
}

/*
 * DMC and L3 counter interface is muxed across all channels.
 * hence we need to select the channel before accessing counter
 * data/control registers.
 *
 *  L3 Tile and DMC channel selection is through SMC call
 *  SMC call arguments,
 *	x0 = THUNDERX2_SMC_CALL_ID	(Vendor SMC call Id)
 *	x1 = THUNDERX2_SMC_SET_CHANNEL	(Id to set DMC/L3C channel)
 *	x2 = Node id
 *	x3 = DMC(1)/L3C(0)
 *	x4 = channel Id
 */
static void uncore_select_channel(struct perf_event *event)
{
	struct arm_smccc_res res;
	struct thunderx2_pmu_uncore_channel *pmu_uncore =
		pmu_to_thunderx2_pmu_uncore(event->pmu);
	struct thunderx2_pmu_uncore_dev *uncore_dev =
		pmu_uncore->uncore_dev;

	arm_smccc_smc(THUNDERX2_SMC_CALL_ID, THUNDERX2_SMC_SET_CHANNEL,
			uncore_dev->node, uncore_dev->type,
			pmu_uncore->channel, 0, 0, 0, &res);
	if (res.a0) {
		dev_err(uncore_dev->dev,
			"SMC to Select channel failed for PMU UNCORE[%s]\n",
				pmu_uncore->uncore_dev->name);
	}
}

/* early probe for firmware support */
static int __init test_uncore_select_channel_early(struct device *dev)
{
	struct arm_smccc_res res;

	arm_smccc_smc(THUNDERX2_SMC_CALL_ID, THUNDERX2_SMC_SET_CHANNEL,
			dev_to_node(dev), 0, 0, 0, 0, 0, &res);
	if (res.a0) {
		dev_err(dev, "No Firmware support for PMU UNCORE(%d)\n",
				dev_to_node(dev));
		return -ENODEV;
	}
	return 0;
}

static void uncore_start_event_l3c(struct perf_event *event, int flags)
{
	u32 val;
	struct hw_perf_event *hwc = &event->hw;

	/* event id encoded in bits [07:03] */
	val = GET_EVENTID(event) << 3;
	reg_writel(val, hwc->config_base);
	local64_set(&hwc->prev_count, 0);
	reg_writel(0, hwc->event_base);
}

static void uncore_stop_event_l3c(struct perf_event *event)
{
	reg_writel(0, event->hw.config_base);
}

static void uncore_start_event_dmc(struct perf_event *event, int flags)
{
	u32 val;
	struct hw_perf_event *hwc = &event->hw;
	int idx = GET_COUNTERID(event);
	int event_type = GET_EVENTID(event);

	/* enable and start counters.
	 * 8 bits for each counter, bits[05:01] of a counter to set event type.
	 */
	val = reg_readl(hwc->config_base);
	val &= ~DMC_EVENT_CFG(idx, 0x1f);
	val |= DMC_EVENT_CFG(idx, event_type);
	reg_writel(val, hwc->config_base);
	local64_set(&hwc->prev_count, 0);
	reg_writel(0, hwc->event_base);
}

static void uncore_stop_event_dmc(struct perf_event *event)
{
	u32 val;
	struct hw_perf_event *hwc = &event->hw;
	int idx = GET_COUNTERID(event);

	/* clear event type(bits[05:01]) to stop counter */
	val = reg_readl(hwc->config_base);
	val &= ~DMC_EVENT_CFG(idx, 0x1f);
	reg_writel(val, hwc->config_base);
}

static void init_cntr_base_l3c(struct perf_event *event,
		struct thunderx2_pmu_uncore_dev *uncore_dev)
{
	struct hw_perf_event *hwc = &event->hw;

	/* counter ctrl/data reg offset at 8 */
	hwc->config_base = (unsigned long)uncore_dev->base
		+ L3C_COUNTER_CTL + (8 * GET_COUNTERID(event));
	hwc->event_base =  (unsigned long)uncore_dev->base
		+ L3C_COUNTER_DATA + (8 * GET_COUNTERID(event));
}

static void init_cntr_base_dmc(struct perf_event *event,
		struct thunderx2_pmu_uncore_dev *uncore_dev)
{
	struct hw_perf_event *hwc = &event->hw;

	hwc->config_base = (unsigned long)uncore_dev->base
		+ DMC_COUNTER_CTL;
	/* counter data reg offset at 0xc */
	hwc->event_base = (unsigned long)uncore_dev->base
		+ DMC_COUNTER_DATA + (0xc * GET_COUNTERID(event));
}

static void thunderx2_uncore_update(struct perf_event *event)
{
	s64 prev, new = 0;
	u64 delta;
	struct hw_perf_event *hwc = &event->hw;
	struct thunderx2_pmu_uncore_channel *pmu_uncore;
	enum thunderx2_uncore_type type;

	pmu_uncore = pmu_to_thunderx2_pmu_uncore(event->pmu);
	type = pmu_uncore->uncore_dev->type;

	pmu_uncore->uncore_dev->select_channel(event);

	new = reg_readl(hwc->event_base);
	prev = local64_xchg(&hwc->prev_count, new);

	/* handles rollover of 32 bit counter */
	delta = (u32)(((1UL << 32) - prev) + new);
	local64_add(delta, &event->count);
}

enum thunderx2_uncore_type get_uncore_device_type(struct acpi_device *adev)
{
	int i = 0;
	struct acpi_uncore_device {
		__u8 id[ACPI_ID_LEN];
		enum thunderx2_uncore_type type;
	} devices[] = {
		{"CAV901D", PMU_TYPE_L3C},
		{"CAV901F", PMU_TYPE_DMC},
		{"", PMU_TYPE_INVALID}
	};

	while (devices[i].type != PMU_TYPE_INVALID) {
		if (!strcmp(acpi_device_hid(adev), devices[i].id))
			return devices[i].type;
		i++;
	}
	return PMU_TYPE_INVALID;
}

/*
 * We must NOT create groups containing events from multiple hardware PMUs,
 * although mixing different software and hardware PMUs is allowed.
 */
static bool thunderx2_uncore_validate_event_group(struct perf_event *event)
{
	struct pmu *pmu = event->pmu;
	struct perf_event *leader = event->group_leader;
	struct perf_event *sibling;
	int counters = 0;

	if (leader->pmu != event->pmu && !is_software_event(leader))
		return false;

	list_for_each_entry(sibling, &event->group_leader->sibling_list,
			group_entry) {
		if (is_software_event(sibling))
			continue;
		if (sibling->pmu != pmu)
			return false;
		counters++;
	}

	/*
	 * If the group requires more counters than the HW has,
	 * it cannot ever be scheduled.
	 */
	return counters < UNCORE_MAX_COUNTERS;
}

static int thunderx2_uncore_event_init(struct perf_event *event)
{
	struct hw_perf_event *hwc = &event->hw;
	struct thunderx2_pmu_uncore_channel *pmu_uncore;

	/* Test the event attr type check for PMU enumeration */
	if (event->attr.type != event->pmu->type)
		return -ENOENT;

	/*
	 * SOC PMU counters are shared across all cores.
	 * Therefore, it does not support per-process mode.
	 * Also, it does not support event sampling mode.
	 */
	if (is_sampling_event(event) || event->attach_state & PERF_ATTACH_TASK)
		return -EINVAL;

	/* SOC counters do not have usr/os/guest/host bits */
	if (event->attr.exclude_user || event->attr.exclude_kernel ||
	    event->attr.exclude_host || event->attr.exclude_guest)
		return -EINVAL;

	if (event->cpu < 0)
		return -EINVAL;

	pmu_uncore = pmu_to_thunderx2_pmu_uncore(event->pmu);
	event->cpu = pmu_uncore->cpu;

	if (event->attr.config >= pmu_uncore->uncore_dev->max_events)
		return -EINVAL;

	/* store event id */
	hwc->config = event->attr.config;

	/* Validate the group */
	if (!thunderx2_uncore_validate_event_group(event))
		return -EINVAL;

	return 0;
}

static void thunderx2_uncore_start(struct perf_event *event, int flags)
{
	struct hw_perf_event *hwc = &event->hw;
	struct thunderx2_pmu_uncore_channel *pmu_uncore;
	struct thunderx2_pmu_uncore_dev *uncore_dev;
	unsigned long irqflags;

	hwc->state = 0;
	pmu_uncore = pmu_to_thunderx2_pmu_uncore(event->pmu);
	uncore_dev = pmu_uncore->uncore_dev;

	raw_spin_lock_irqsave(&uncore_dev->lock, irqflags);
	uncore_dev->select_channel(event);
	uncore_dev->start_event(event, flags);
	raw_spin_unlock_irqrestore(&uncore_dev->lock, irqflags);

	perf_event_update_userpage(event);

	if (!find_last_bit(pmu_uncore->active_counters,
				pmu_uncore->uncore_dev->max_counters)) {
		hrtimer_start(&pmu_uncore->hrtimer,
			ns_to_ktime(uncore_dev->hrtimer_interval),
			HRTIMER_MODE_REL_PINNED);
	}
}

static void thunderx2_uncore_stop(struct perf_event *event, int flags)
{
	struct hw_perf_event *hwc = &event->hw;
	struct thunderx2_pmu_uncore_channel *pmu_uncore;
	struct thunderx2_pmu_uncore_dev *uncore_dev;
	unsigned long irqflags;

	if (hwc->state & PERF_HES_UPTODATE)
		return;

	pmu_uncore = pmu_to_thunderx2_pmu_uncore(event->pmu);
	uncore_dev = pmu_uncore->uncore_dev;

	raw_spin_lock_irqsave(&uncore_dev->lock, irqflags);

	uncore_dev->select_channel(event);
	uncore_dev->stop_event(event);

	WARN_ON_ONCE(hwc->state & PERF_HES_STOPPED);
	hwc->state |= PERF_HES_STOPPED;
	if ((flags & PERF_EF_UPDATE) && !(hwc->state & PERF_HES_UPTODATE)) {
		thunderx2_uncore_update(event);
		hwc->state |= PERF_HES_UPTODATE;
	}
	raw_spin_unlock_irqrestore(&uncore_dev->lock, irqflags);
}

static int thunderx2_uncore_add(struct perf_event *event, int flags)
{
	struct hw_perf_event *hwc = &event->hw;
	struct thunderx2_pmu_uncore_channel *pmu_uncore;
	struct thunderx2_pmu_uncore_dev *uncore_dev;

	pmu_uncore = pmu_to_thunderx2_pmu_uncore(event->pmu);
	uncore_dev = pmu_uncore->uncore_dev;

	/* Allocate a free counter */
	hwc->idx  = alloc_counter(pmu_uncore);
	if (hwc->idx < 0)
		return -EAGAIN;

	pmu_uncore->events[hwc->idx] = event;
	/* set counter control and data registers base address */
	uncore_dev->init_cntr_base(event, uncore_dev);

	hwc->state = PERF_HES_UPTODATE | PERF_HES_STOPPED;
	if (flags & PERF_EF_START)
		thunderx2_uncore_start(event, flags);

	return 0;
}

static void thunderx2_uncore_del(struct perf_event *event, int flags)
{
	struct thunderx2_pmu_uncore_channel *pmu_uncore =
			pmu_to_thunderx2_pmu_uncore(event->pmu);
	struct hw_perf_event *hwc = &event->hw;

	thunderx2_uncore_stop(event, PERF_EF_UPDATE);

	/* clear the assigned counter */
	free_counter(pmu_uncore, GET_COUNTERID(event));

	perf_event_update_userpage(event);
	pmu_uncore->events[hwc->idx] = NULL;
	hwc->idx = -1;
}

static void thunderx2_uncore_read(struct perf_event *event)
{
	unsigned long irqflags;
	struct thunderx2_pmu_uncore_channel *pmu_uncore =
			pmu_to_thunderx2_pmu_uncore(event->pmu);

	raw_spin_lock_irqsave(&pmu_uncore->uncore_dev->lock, irqflags);
	thunderx2_uncore_update(event);
	raw_spin_unlock_irqrestore(&pmu_uncore->uncore_dev->lock, irqflags);
}

static enum hrtimer_restart thunderx2_uncore_hrtimer_callback(
		struct hrtimer *hrt)
{
	struct thunderx2_pmu_uncore_channel *pmu_uncore;
	unsigned long irqflags;
	int idx;
	bool restart_timer = false;

	pmu_uncore = container_of(hrt, struct thunderx2_pmu_uncore_channel,
			hrtimer);

	raw_spin_lock_irqsave(&pmu_uncore->uncore_dev->lock, irqflags);
	for_each_set_bit(idx, pmu_uncore->active_counters,
			pmu_uncore->uncore_dev->max_counters) {
		struct perf_event *event = pmu_uncore->events[idx];

		thunderx2_uncore_update(event);
		restart_timer = true;
	}
	raw_spin_unlock_irqrestore(&pmu_uncore->uncore_dev->lock, irqflags);

	if (restart_timer)
		hrtimer_forward_now(hrt,
			ns_to_ktime(
				pmu_uncore->uncore_dev->hrtimer_interval));

	return restart_timer ? HRTIMER_RESTART : HRTIMER_NORESTART;
}

static int thunderx2_pmu_uncore_register(
		struct thunderx2_pmu_uncore_channel *pmu_uncore)
{
	struct device *dev = pmu_uncore->uncore_dev->dev;
	char *name = pmu_uncore->uncore_dev->name;
	int channel = pmu_uncore->channel;

	/* Perf event registration */
	pmu_uncore->pmu = (struct pmu) {
		.attr_groups	= pmu_uncore->uncore_dev->attr_groups,
		.task_ctx_nr	= perf_invalid_context,
		.event_init	= thunderx2_uncore_event_init,
		.add		= thunderx2_uncore_add,
		.del		= thunderx2_uncore_del,
		.start		= thunderx2_uncore_start,
		.stop		= thunderx2_uncore_stop,
		.read		= thunderx2_uncore_read,
	};

	pmu_uncore->pmu.name = devm_kasprintf(dev, GFP_KERNEL,
			"%s_%d", name, channel);

	return perf_pmu_register(&pmu_uncore->pmu, pmu_uncore->pmu.name, -1);
}

static int thunderx2_pmu_uncore_add(struct thunderx2_pmu_uncore_dev *uncore_dev,
		int channel)
{
	struct thunderx2_pmu_uncore_channel *pmu_uncore;
	int ret, cpu;

	pmu_uncore = devm_kzalloc(uncore_dev->dev, sizeof(*pmu_uncore),
			GFP_KERNEL);
	if (!pmu_uncore)
		return -ENOMEM;

	cpu = cpumask_any_and(cpumask_of_node(uncore_dev->node),
			cpu_online_mask);
	if (cpu >= nr_cpu_ids)
		return -EINVAL;

	pmu_uncore->cpu = cpu;
	pmu_uncore->channel = channel;
	pmu_uncore->uncore_dev = uncore_dev;

	hrtimer_init(&pmu_uncore->hrtimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	pmu_uncore->hrtimer.function = thunderx2_uncore_hrtimer_callback;

	ret = thunderx2_pmu_uncore_register(pmu_uncore);
	if (ret) {
		dev_err(uncore_dev->dev, "%s PMU: Failed to init driver\n",
				uncore_dev->name);
		return -ENODEV;
	}

	/* register hotplug callback for the pmu */
	ret = cpuhp_state_add_instance(
			CPUHP_AP_PERF_ARM_THUNDERX2_UNCORE_ONLINE,
			&pmu_uncore->node);
	if (ret) {
		dev_err(uncore_dev->dev, "Error %d registering hotplug", ret);
		return ret;
	}

	dev_dbg(uncore_dev->dev, "%s PMU UNCORE registered\n",
			pmu_uncore->pmu.name);
	return ret;
}

static struct thunderx2_pmu_uncore_dev *init_pmu_uncore_dev(
		struct device *dev, acpi_handle handle,
		struct acpi_device *adev, u32 type)
{
	struct thunderx2_pmu_uncore_dev *uncore_dev;
	void __iomem *base;
	struct resource res;
	struct resource_entry *rentry;
	struct list_head list;
	int ret;

	INIT_LIST_HEAD(&list);
	ret = acpi_dev_get_resources(adev, &list, NULL, NULL);
	if (ret <= 0) {
		dev_err(dev, "failed to parse _CRS method, error %d\n", ret);
		return NULL;
	}

	list_for_each_entry(rentry, &list, node) {
		if (resource_type(rentry->res) == IORESOURCE_MEM) {
			res = *rentry->res;
			break;
		}
	}

	if (!rentry->res)
		return NULL;

	acpi_dev_free_resource_list(&list);
	base = devm_ioremap_resource(dev, &res);
	if (IS_ERR(base)) {
		dev_err(dev, "PMU type %d: Fail to map resource\n", type);
		return NULL;
	}

	uncore_dev = devm_kzalloc(dev, sizeof(*uncore_dev), GFP_KERNEL);
	if (!uncore_dev)
		return NULL;

	uncore_dev->dev = dev;
	uncore_dev->type = type;
	uncore_dev->base = base;
	uncore_dev->node = dev_to_node(dev);

	raw_spin_lock_init(&uncore_dev->lock);

	switch (uncore_dev->type) {
	case PMU_TYPE_L3C:
		uncore_dev->max_counters = UNCORE_MAX_COUNTERS;
		uncore_dev->max_channels = UNCORE_L3_MAX_TILES;
		uncore_dev->max_events = L3_EVENT_MAX;
		uncore_dev->hrtimer_interval = UNCORE_HRTIMER_INTERVAL;
		uncore_dev->attr_groups = l3c_pmu_attr_groups;
		uncore_dev->name = devm_kasprintf(dev, GFP_KERNEL,
				"uncore_l3c_%d", uncore_dev->node);
		uncore_dev->init_cntr_base = init_cntr_base_l3c;
		uncore_dev->start_event = uncore_start_event_l3c;
		uncore_dev->stop_event = uncore_stop_event_l3c;
		uncore_dev->select_channel = uncore_select_channel;
		break;
	case PMU_TYPE_DMC:
		uncore_dev->max_counters = UNCORE_MAX_COUNTERS;
		uncore_dev->max_channels = UNCORE_DMC_MAX_CHANNELS;
		uncore_dev->max_events = DMC_EVENT_MAX;
		uncore_dev->hrtimer_interval = UNCORE_HRTIMER_INTERVAL;
		uncore_dev->attr_groups = dmc_pmu_attr_groups;
		uncore_dev->name = devm_kasprintf(dev, GFP_KERNEL,
				"uncore_dmc_%d", uncore_dev->node);
		uncore_dev->init_cntr_base = init_cntr_base_dmc;
		uncore_dev->start_event = uncore_start_event_dmc;
		uncore_dev->stop_event = uncore_stop_event_dmc;
		uncore_dev->select_channel = uncore_select_channel;
		break;
	case PMU_TYPE_INVALID:
		devm_kfree(dev, uncore_dev);
		uncore_dev = NULL;
		break;
	}

	return uncore_dev;
}

static acpi_status thunderx2_pmu_uncore_dev_add(acpi_handle handle, u32 level,
				    void *data, void **return_value)
{
	struct thunderx2_pmu_uncore_dev *uncore_dev;
	struct acpi_device *adev;
	enum thunderx2_uncore_type type;
	int channel;

	if (acpi_bus_get_device(handle, &adev))
		return AE_OK;
	if (acpi_bus_get_status(adev) || !adev->status.present)
		return AE_OK;

	type = get_uncore_device_type(adev);
	if (type == PMU_TYPE_INVALID)
		return AE_OK;

	uncore_dev = init_pmu_uncore_dev((struct device *)data, handle,
			adev, type);

	if (!uncore_dev)
		return AE_ERROR;

	for (channel = 0; channel < uncore_dev->max_channels; channel++) {
		if (thunderx2_pmu_uncore_add(uncore_dev, channel)) {
			/* Can't add the PMU device, abort */
			return AE_ERROR;
		}
	}
	return AE_OK;
}

static int thunderx2_uncore_pmu_offline_cpu(unsigned int cpu,
		struct hlist_node *node)
{
	int new_cpu;
	struct thunderx2_pmu_uncore_channel *pmu_uncore;

	pmu_uncore = hlist_entry_safe(node,
			struct thunderx2_pmu_uncore_channel, node);
	if (cpu != pmu_uncore->cpu)
		return 0;

	new_cpu = cpumask_any_and(
			cpumask_of_node(pmu_uncore->uncore_dev->node),
			cpu_online_mask);
	if (new_cpu >= nr_cpu_ids)
		return 0;

	pmu_uncore->cpu = new_cpu;
	perf_pmu_migrate_context(&pmu_uncore->pmu, cpu, new_cpu);
	return 0;
}

static const struct acpi_device_id thunderx2_uncore_acpi_match[] = {
	{"CAV901C", 0},
	{},
};
MODULE_DEVICE_TABLE(acpi, thunderx2_uncore_acpi_match);

static int thunderx2_uncore_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	acpi_handle handle;
	acpi_status status;

	set_dev_node(dev, acpi_get_node(ACPI_HANDLE(dev)));

	/* Make sure firmware supports DMC/L3C set channel smc call */
	if (test_uncore_select_channel_early(dev))
		return -ENODEV;

	if (!has_acpi_companion(dev))
		return -ENODEV;

	handle = ACPI_HANDLE(dev);
	if (!handle)
		return -EINVAL;

	/* Walk through the tree for all PMU UNCORE devices */
	status = acpi_walk_namespace(ACPI_TYPE_DEVICE, handle, 1,
				     thunderx2_pmu_uncore_dev_add,
				     NULL, dev, NULL);
	if (ACPI_FAILURE(status)) {
		dev_err(dev, "failed to probe PMU devices\n");
		return_ACPI_STATUS(status);
	}

	dev_info(dev, "node%d: pmu uncore registered\n", dev_to_node(dev));
	return 0;
}

static struct platform_driver thunderx2_uncore_driver = {
	.probe = thunderx2_uncore_probe,
	.driver = {
		.name		= "thunderx2-uncore-pmu",
		.acpi_match_table = ACPI_PTR(thunderx2_uncore_acpi_match),
	},
};

static int __init register_thunderx2_uncore_driver(void)
{
	int ret;

	ret = cpuhp_setup_state_multi(CPUHP_AP_PERF_ARM_THUNDERX2_UNCORE_ONLINE,
				      "perf/tx2/uncore:online",
				      NULL,
				      thunderx2_uncore_pmu_offline_cpu);
	if (ret)
		return ret;

	return platform_driver_register(&thunderx2_uncore_driver);

}
device_initcall(register_thunderx2_uncore_driver);
