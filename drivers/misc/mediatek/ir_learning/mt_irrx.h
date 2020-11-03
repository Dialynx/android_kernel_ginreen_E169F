
#define DEV_NAME "ir-learning"
#define SPI_BUF_LEN (256*1024)

struct mt_irrx {
	unsigned int spi_clock; /* SPI clock source */
	unsigned int spi_hz; /* SPI clock output */
	unsigned int spi_data_invert;
	void *spi_buffer;
	struct spi_device *spi_dev;

#ifndef CONFIG_MTK_LEGACY
	struct pinctrl *ppinctrl_irrx;
#endif
};

typedef enum {
	IRRX_GPIO_DISABLE,
	IRRX_GPIO_ENABLE,
} IRRX_GPIO_MODE;

#define SPI_IOC_MAGIC				'k'
#define SPI_IOC_READ_WAVE			_IOR(SPI_IOC_MAGIC, 1, __u8)
#define SPI_IOC_GET_SAMPLE_RATE		_IOR(SPI_IOC_MAGIC, 2, __u8)
