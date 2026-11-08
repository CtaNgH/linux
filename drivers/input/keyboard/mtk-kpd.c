// SPDX-License-Identifier: GPL-2.0-only
/*
 * MediaTek MT6893 keypad (KP) driver.
 *
 * The block at 0x10010000 exposes five 16-bit key-state memories (72 keys)
 * and raises one interrupt when any of them changes.  Bits are active low:
 * a zero bit means the key is down.  The vendor driver is a 4.14 out-of-tree
 * driver; this is the minimal mainline equivalent needed for the volume
 * keys, whose map the board DT already carries:
 *
 *   mediatek,kpd-hw-init-map = <KEY_VOLUMEUP KEY_VOLUMEDOWN ...>
 */
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_wakeup.h>
#include <linux/slab.h>
#include <linux/workqueue.h>

#define KPD_MAX_KEYS	72
#define KPD_NUM_MEMS	5
#define KPD_NAME	"mtk-kpd"

#define KP_STA		0x0000
#define KP_MEM1	0x0004
#define KP_MEM2	0x0008
#define KP_MEM3	0x000c
#define KP_MEM4	0x0010
#define KP_MEM5	0x0014
#define KP_DEBOUNCE	0x0018
#define KP_SEL		0x0020
#define KP_EN		0x0024

#define KPD_DEBOUNCE_MASK	((1U << 14) - 1)
#define KPD_DOUBLE_KEY_MASK	(1U << 0)

struct mtk_kpd {
	void __iomem *base;
	struct input_dev *input;
	struct work_struct work;
	int irq;
	u32 keymap[KPD_MAX_KEYS];
	u32 map_num;
	u16 state[KPD_NUM_MEMS];
};

static void mtk_kpd_work(struct work_struct *work)
{
	struct mtk_kpd *kpd = container_of(work, struct mtk_kpd, work);
	u16 state[KPD_NUM_MEMS];
	int i, bit;

	for (i = 0; i < KPD_NUM_MEMS; i++)
		state[i] = readw(kpd->base + KP_MEM1 + i * 4);

	for (i = 0; i < KPD_NUM_MEMS; i++) {
		u16 change = state[i] ^ kpd->state[i];

		if (!change)
			continue;
		for (bit = 0; bit < 16; bit++) {
			unsigned int hw = i * 16 + bit;
			u16 mask = BIT(bit);
			u32 code;

			if (!(change & mask) || hw >= kpd->map_num)
				continue;
			code = kpd->keymap[hw];
			if (!code)
				continue;
			input_report_key(kpd->input, code,
					 !(state[i] & mask));
		}
	}
	input_sync(kpd->input);
	memcpy(kpd->state, state, sizeof(kpd->state));

	enable_irq(kpd->irq);
}

static irqreturn_t mtk_kpd_irq(int irq, void *data)
{
	struct mtk_kpd *kpd = data;

	disable_irq_nosync(irq);
	schedule_work(&kpd->work);
	return IRQ_HANDLED;
}

static ssize_t keys_show(struct device *dev, struct device_attribute *attr,
			 char *buf)
{
	struct mtk_kpd *kpd = dev_get_drvdata(dev);
	u16 state[KPD_NUM_MEMS];
	int i, n = 0;

	for (i = 0; i < KPD_NUM_MEMS; i++)
		state[i] = readw(kpd->base + KP_MEM1 + i * 4);
	for (i = 0; i < KPD_NUM_MEMS; i++)
		n += scnprintf(buf + n, PAGE_SIZE - n, "%04x%s", state[i],
			       i == KPD_NUM_MEMS - 1 ? "\n" : " ");
	return n;
}
static DEVICE_ATTR_RO(keys);

static int mtk_kpd_probe(struct platform_device *pdev)
{
	struct mtk_kpd *kpd;
	struct device_node *np = pdev->dev.of_node;
	struct resource *res;
	u32 debounce = 0;
	int i, ret;

	kpd = devm_kzalloc(&pdev->dev, sizeof(*kpd), GFP_KERNEL);
	if (!kpd)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	kpd->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(kpd->base))
		return PTR_ERR(kpd->base);

	if (!of_property_read_u32(np, "mediatek,kpd-hw-map-num",
				  &kpd->map_num)) {
		if (!kpd->map_num || kpd->map_num > KPD_MAX_KEYS)
			kpd->map_num = KPD_MAX_KEYS;
	} else {
		kpd->map_num = KPD_MAX_KEYS;
	}
	ret = of_property_read_u32_array(np, "mediatek,kpd-hw-init-map",
					 kpd->keymap, kpd->map_num);
	if (ret)
		memset(kpd->keymap, 0, sizeof(kpd->keymap));

	kpd->input = devm_input_allocate_device(&pdev->dev);
	if (!kpd->input)
		return -ENOMEM;
	kpd->input->name = KPD_NAME;
	kpd->input->id.bustype = BUS_HOST;
	kpd->input->id.vendor = 0x2454;
	kpd->input->id.product = 0x6500;
	kpd->input->id.version = 0x0010;
	__set_bit(EV_KEY, kpd->input->evbit);
	for (i = 0; i < kpd->map_num; i++) {
		if (kpd->keymap[i])
			__set_bit(kpd->keymap[i], kpd->input->keybit);
	}

	INIT_WORK(&kpd->work, mtk_kpd_work);

	if (!of_property_read_u32(np, "mediatek,kpd-key-debounce",
				  &debounce))
		writew(debounce & KPD_DEBOUNCE_MASK,
		       kpd->base + KP_DEBOUNCE);

	writew(1, kpd->base + KP_EN);
	/* flush the posted write and let the scanner fill the memories; a key
	 * held while we probe only shows up after the first scan cycle. */
	readw(kpd->base + KP_EN);
	msleep(50);
	for (i = 0; i < KPD_NUM_MEMS; i++)
		kpd->state[i] = readw(kpd->base + KP_MEM1 + i * 4);

	kpd->irq = platform_get_irq(pdev, 0);
	if (kpd->irq < 0)
		return kpd->irq;

	ret = devm_request_irq(&pdev->dev, kpd->irq, mtk_kpd_irq,
			       IRQF_TRIGGER_RISING, KPD_NAME, kpd);
	if (ret)
		return ret;

	ret = input_register_device(kpd->input);
	if (ret)
		return ret;

	/* A key (typically volume-up for recovery) may already be held while
	 * the driver probes; report it so userspace can see it even though no
	 * state change will ever be generated for it.
	 */
	for (i = 0; i < KPD_NUM_MEMS; i++) {
		int bit;

		for (bit = 0; bit < 16; bit++) {
			unsigned int hw = i * 16 + bit;

			if ((kpd->state[i] & BIT(bit)) || hw >= kpd->map_num)
				continue;
			if (kpd->keymap[hw])
				input_report_key(kpd->input, kpd->keymap[hw], 1);
		}
	}
	input_sync(kpd->input);

	platform_set_drvdata(pdev, kpd);
	device_init_wakeup(&pdev->dev, true);
	device_create_file(&pdev->dev, &dev_attr_keys);

	dev_info(&pdev->dev, "mtk keypad ready (%u keys, irq %d)\n",
		 kpd->map_num, kpd->irq);
	return 0;
}

static int __maybe_unused mtk_kpd_suspend(struct device *dev)
{
	struct mtk_kpd *kpd = dev_get_drvdata(dev);

	if (device_may_wakeup(dev))
		enable_irq_wake(kpd->irq);
	return 0;
}

static int __maybe_unused mtk_kpd_resume(struct device *dev)
{
	struct mtk_kpd *kpd = dev_get_drvdata(dev);

	if (device_may_wakeup(dev))
		disable_irq_wake(kpd->irq);
	return 0;
}

static SIMPLE_DEV_PM_OPS(mtk_kpd_pm_ops, mtk_kpd_suspend, mtk_kpd_resume);

static const struct of_device_id mtk_kpd_of_match[] = {
	{ .compatible = "mediatek,kp" },
	{ .compatible = "mediatek,mt6893-kp" },
	{ }
};
MODULE_DEVICE_TABLE(of, mtk_kpd_of_match);

static struct platform_driver mtk_kpd_driver = {
	.probe = mtk_kpd_probe,
	.driver = {
		.name = KPD_NAME,
		.of_match_table = mtk_kpd_of_match,
		.pm = &mtk_kpd_pm_ops,
	},
};
module_platform_driver(mtk_kpd_driver);

MODULE_DESCRIPTION("MediaTek MT6893 keypad driver");
MODULE_LICENSE("GPL");
