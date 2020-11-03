#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/wait.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/kthread.h>
#include <linux/poll.h>
#include <linux/time.h>
#include <linux/delay.h>
#include <linux/kobject.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/spi/spi.h>

#ifndef CONFIG_MTK_LEGACY
#include <linux/clk.h>
#else
#include <mach/mt_gpio.h>
#include <cust_gpio_usage.h>
#endif

#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_fdt.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#endif

#include <mach/mt_spi.h>
#include "mt_irrx.h"

static struct mt_irrx mt_irrx_dev;
static struct mt_chip_conf irrx_spi_conf;
static atomic_t ir_usage_cnt;

__weak int get_ir_device(void)
{
	if (atomic_cmpxchg(&ir_usage_cnt, 0, 1) != 0)
		return -EBUSY;
	return 0;
}

__weak int put_ir_device(void)
{
	if (atomic_cmpxchg(&ir_usage_cnt, 1, 0) != 1)
		return -EFAULT;
	return 0;
}

void switch_irrx_gpio(IRRX_GPIO_MODE mode)
{
#ifndef CONFIG_MTK_LEGACY
	struct pinctrl_state *pins_irrx = NULL;

	switch (mode) {
	case IRRX_GPIO_DISABLE:
		pins_irrx = pinctrl_lookup_state(mt_irrx_dev.ppinctrl_irrx, "irrx_gpio_disable");
		break;
	case IRRX_GPIO_ENABLE:
		pins_irrx = pinctrl_lookup_state(mt_irrx_dev.ppinctrl_irrx, "irrx_gpio_enable");
		break;
	default:
		break;
	};
	
	if (IS_ERR(pins_irrx)) {
		pr_err("[IRTX]pinctrl_lookup_state fail mode=%d, ptr_err=%ld\n", mode, PTR_ERR(pins_irrx));
		return;
	}
	pinctrl_select_state(mt_irrx_dev.ppinctrl_irrx, pins_irrx);
#else
	switch (mode) {
	case IRRX_GPIO_DISABLE:
	    mt_set_gpio_mode(GPIO_SPI_CS_PIN, GPIO_SPI_CS_PIN_M_GPIO);
	    mt_set_gpio_dir(GPIO_SPI_CS_PIN, GPIO_DIR_OUT);
	    mt_set_gpio_pull_enable(GPIO_SPI_CS_PIN, GPIO_PULL_ENABLE);
    	mt_set_gpio_out(GPIO_SPI_CS_PIN, GPIO_OUT_ZERO);
    	
	    mt_set_gpio_mode(GPIO_SPI_SCK_PIN, GPIO_SPI_SCK_PIN_M_GPIO);
	    mt_set_gpio_dir(GPIO_SPI_SCK_PIN, GPIO_DIR_OUT);
	    mt_set_gpio_pull_enable(GPIO_SPI_SCK_PIN, GPIO_PULL_ENABLE);
    	mt_set_gpio_out(GPIO_SPI_SCK_PIN, GPIO_OUT_ZERO);

	    mt_set_gpio_mode(GPIO_SPI_MISO_PIN, GPIO_SPI_MISO_PIN_M_GPIO);
	    mt_set_gpio_dir(GPIO_SPI_MISO_PIN, GPIO_DIR_IN);
	    mt_set_gpio_pull_enable(GPIO_SPI_MISO_PIN, GPIO_PULL_ENABLE);
    	mt_set_gpio_out(GPIO_SPI_MISO_PIN, GPIO_OUT_ONE);
    	
	    //mt_set_gpio_mode(GPIO_SPI_MOSI_PIN, GPIO_SPI_MOSI_PIN_M_GPIO);
	    //mt_set_gpio_dir(GPIO_SPI_MOSI_PIN, GPIO_DIR_OUT);
	    //mt_set_gpio_pull_enable(GPIO_SPI_MOSI_PIN, GPIO_PULL_ENABLE);
    	//mt_set_gpio_out(GPIO_SPI_MOSI_PIN, GPIO_OUT_ONE);
		break;
	case IRRX_GPIO_ENABLE:
    	mt_set_gpio_out(GPIO_SPI_CS_PIN, GPIO_OUT_ONE);

	    mt_set_gpio_mode(GPIO_SPI_SCK_PIN, GPIO_SPI_SCK_PIN_M_SPI_CKA);
	    mt_set_gpio_mode(GPIO_SPI_MISO_PIN, GPIO_SPI_MISO_PIN_M_SPI_MIA);
		//mt_set_gpio_mode(GPIO_SPI_MOSI_PIN, GPIO_SPI_MOSI_PIN_M_SPI_MOA);
		break;
	default:
		break;
	};
#endif
}

static int dev_char_open(struct inode *inode, struct file *file)
{
	int ret = 0;

	ret = get_ir_device();
	if (ret) {
		pr_err("[IRRX] device busy\n");
		goto exit;
	}

	pr_debug("[IRRX] open by %s\n", current->comm);
	nonseekable_open(inode, file);
exit:
	return ret;
}

static int dev_char_close(struct inode *inode, struct file *file)
{
	int ret = 0;

	ret = put_ir_device();
	if (ret) {
		pr_err("[IRTX] device close without open\n");
		goto exit;
}

	pr_debug("[IRRX] close by %s\n", current->comm);
exit:
	return ret;
}

static ssize_t dev_char_read(struct file *file, char *buf, size_t count, loff_t *ppos)
{
	return 0;
}

static ssize_t dev_char_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{

	return count;
}

static long dev_char_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	int i;
	unsigned char *data_ptr;
	struct spi_message spi_msg;
	struct spi_transfer spi_trf = {0x00};

	if (!mt_irrx_dev.spi_dev || !mt_irrx_dev.spi_buffer)
		return -ENODEV;

	switch (cmd) {
	case SPI_IOC_READ_WAVE:
		pr_debug("[IRRX] ioctl read message\n");
		switch_irrx_gpio(IRRX_GPIO_ENABLE);
		spi_message_init(&spi_msg);
		spi_message_add_tail(&spi_trf, &spi_msg);
		spi_trf.rx_buf = mt_irrx_dev.spi_buffer;
		spi_trf.len = SPI_BUF_LEN;
		spi_trf.tx_buf = mt_irrx_dev.spi_buffer;
		memset(spi_trf.rx_buf, 0, spi_trf.len);
		ret = spi_sync(mt_irrx_dev.spi_dev, &spi_msg);
		pr_debug("[IRRX] spi_sync ret=%d\n", ret);

		/* invert bit */
		if (mt_irrx_dev.spi_data_invert) {
			pr_debug("[IRRX] invert data\n");
		for (i = 0; i < SPI_BUF_LEN; i++) {
			data_ptr = (unsigned char *)mt_irrx_dev.spi_buffer + i;
			*data_ptr = ~(*data_ptr);
		}
		}

		if (copy_to_user((void __user *)arg, spi_trf.rx_buf, spi_trf.len)) {
			pr_err("[IRRX] copy_to_user failed\n");
			ret = -EFAULT;
		}
		switch_irrx_gpio(IRRX_GPIO_DISABLE);
		ret = spi_trf.len;
		break;
	case SPI_IOC_GET_SAMPLE_RATE:
		pr_debug("[IRRX] ioctl get sample rate %d->%d\n", mt_irrx_dev.spi_clock, mt_irrx_dev.spi_hz);
		ret = put_user(mt_irrx_dev.spi_hz, (unsigned int __user *)arg);
		break;
	default:
		pr_err("[IRRX] unknown ioctl cmd 0x%x\n", cmd);
		ret = -ENOTTY;
		break;
	}
	return ret;
}

static int irrx_spi_remove(struct spi_device *spi)
{

	pr_debug("[IRRX] irrx remove\n");
	return 0;
}

static int __init irrx_spi_probe(struct spi_device *spi)
{
	int ret = 0;
	pr_debug("[IRRX] irrx spi probe\n");

	/* update sample rate */
	irrx_spi_conf.high_time = mt_irrx_dev.spi_clock/1000000/2;
	irrx_spi_conf.low_time = mt_irrx_dev.spi_clock/1000000/2;
	mt_irrx_dev.spi_hz = mt_irrx_dev.spi_clock/(irrx_spi_conf.high_time+irrx_spi_conf.low_time);
	/* keep the rest as default */
	irrx_spi_conf.setuptime = 3;
	irrx_spi_conf.holdtime = 3;
	irrx_spi_conf.cs_idletime = 2;
	irrx_spi_conf.ulthgh_thrsh = 0;
	irrx_spi_conf.cpol = 0;
	irrx_spi_conf.cpha = 1;
	irrx_spi_conf.rx_mlsb = 1;
	irrx_spi_conf.tx_mlsb = 1;
	irrx_spi_conf.tx_endian = 0;
	irrx_spi_conf.rx_endian = 0;
	irrx_spi_conf.com_mod = DMA_TRANSFER;
	irrx_spi_conf.pause = 0;
	irrx_spi_conf.finish_intr = 1;
	irrx_spi_conf.deassert = 0;
	irrx_spi_conf.ulthigh = 0;
	irrx_spi_conf.tckdly = 0;
	
	spi->controller_data =(void*)&irrx_spi_conf;
	spi->mode = SPI_MODE_3; // TODO: 
	spi->bits_per_word = 32;
	spi->max_speed_hz = mt_irrx_dev.spi_hz;

	ret = spi_setup(spi);
	if (ret < 0) {
		pr_err("[IRRX] spi_setup fail ret=%d\n", ret);
		goto exit;
	}
	mt_irrx_dev.spi_dev = spi;

exit:
	return ret;
}


static struct spi_device_id spi_id_table = {"spi-irrx", 0};

static struct spi_driver irrx_spi_driver = {
	.driver = {
		.name = "irrx_spi",
		.bus = &spi_bus_type,
		.owner = THIS_MODULE,
	},
	.probe = irrx_spi_probe,
	.remove = irrx_spi_remove,
	.id_table = &spi_id_table,
};

static struct spi_board_info irrx_spi_device[] __initdata = {
	[0] = {
        .modalias="spi-irrx",
		.bus_num = 0,
		.chip_select=1,
		.mode = SPI_MODE_3,
	},
};

static struct file_operations const char_dev_fops = {
	.owner = THIS_MODULE,
	.open = &dev_char_open,
	.read = &dev_char_read,
	.write = &dev_char_write,
	.release = &dev_char_close,
	.unlocked_ioctl = &dev_char_ioctl,
};

static int irrx_probe(struct platform_device *plat_dev)
{
	struct cdev *c_dev;
	dev_t dev_t_irrx;
	struct device *dev = NULL;
	static void *dev_class;
	int ret = 0;

#ifdef CONFIG_OF
	if (plat_dev->dev.of_node == NULL) {
		pr_err("[IRRX] irrx OF node is NULL\n");
		return -ENODEV;
	}
	of_property_read_u32(plat_dev->dev.of_node, "spi_clock", &mt_irrx_dev.spi_clock);
	of_property_read_u32(plat_dev->dev.of_node, "spi_data_invert", &mt_irrx_dev.spi_data_invert);
#endif

#ifndef CONFIG_MTK_LEGACY
	mt_irrx_dev.ppinctrl_irrx = devm_pinctrl_get(&plat_dev->dev);
	if (IS_ERR(mt_irrx_dev.ppinctrl_irrx)) {
		pr_err("[IRRX] devm_pinctrl_get fail ptr_err=%ld\n", PTR_ERR(mt_irrx_dev.ppinctrl_irrx));
		return PTR_ERR(mt_irrx_dev.ppinctrl_irrx);
	}
#endif
	switch_irrx_gpio(IRRX_GPIO_DISABLE);

	/* create char device */
	ret = alloc_chrdev_region(&dev_t_irrx, 0, 1, DEV_NAME);
	if (ret) {
		pr_err("[IRRX] alloc_chrdev_region fail ret=%d\n", ret);
		goto exit;
	}
	c_dev = kmalloc(sizeof(struct cdev), GFP_KERNEL);
	if (!c_dev) {
		pr_err("[IRRX] kmalloc cdev fail\n");
		goto exit;
	}
	cdev_init(c_dev, &char_dev_fops);
	c_dev->owner = THIS_MODULE;
	ret = cdev_add(c_dev, dev_t_irrx, 1);
	if (ret) {
		pr_err("[IRRX] cdev_add fail ret=%d\n", ret);
		goto exit;
	}
	dev_class = class_create(THIS_MODULE, DEV_NAME);
	dev = device_create(dev_class, NULL, dev_t_irrx, NULL, DEV_NAME);
	if (IS_ERR(dev)) {
		ret = PTR_ERR(dev);
		pr_err("[IRRX] device_create fail ret=%d\n", ret);
		goto exit;
	}

	/* create SPI device */
	ret = spi_register_board_info(irrx_spi_device, ARRAY_SIZE(irrx_spi_device));
	if (ret) {
		pr_err("[IRRX] spi_register_board_info fail ret=%d\n", ret);
		goto exit;
	}
	ret = spi_register_driver(&irrx_spi_driver);
	if (ret) {
		pr_err("[IRRX] spi_register_driver fail ret=%d\n", ret);
		goto exit;
	}

	/* alloc buffer */
	mt_irrx_dev.spi_buffer = kzalloc(SPI_BUF_LEN, GFP_KERNEL);
	if (!mt_irrx_dev.spi_buffer) {
		pr_err("[IRRX] kzalloc fail\n");
		ret = -ENOMEM;
		goto exit;
	}

 exit:
	return ret;
}

static struct platform_driver irrx_driver = {
	.driver = {
			.name = DEV_NAME,
		},
	.probe = irrx_probe,
};

#ifdef CONFIG_OF
static const struct of_device_id irrx_of_ids[] = {
	{.compatible = "mediatek,ir-learning",},
	{}
};
#else
static struct platform_device irrx_device = {
	.name = DEV_NAME,
};
#endif

static int __init irrx_init(void)
{
	int ret = 0;

	pr_debug("[IRRX] irrx init\n");

#ifdef CONFIG_OF
	irrx_driver.driver.of_match_table = irrx_of_ids;
#else
	ret = platform_device_register(&irrx_device);
	if (ret) {
		pr_err("[IRRX] irrx platform device register fail %d\n", ret);
		goto exit;
	}
#endif

	ret = platform_driver_register(&irrx_driver);
	if (ret) {
		pr_err("[IRRX] irrx platform driver register fail %d\n", ret);
		goto exit;
	}

exit:
	return ret;
}

module_init(irrx_init);

MODULE_AUTHOR("Xiao Wang <xiao.wang@mediatek.com>");
