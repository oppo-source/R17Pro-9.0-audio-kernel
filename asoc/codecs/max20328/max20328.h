#ifndef _MAX20328_H_
#define _MAX20328_H_

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/uaccess.h>
#include <linux/poll.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>

#define MAX20328_I2C_RETRY_DELAY	10
#define MAX20328_I2C_MAX_RETRIES	5

/* Register Map */
#define MAX20328_REG_DEVICE_ID	(0x00)
#define MAX20328_ADC_VAL        (0x01)
#define MAX20328_STATUS1        (0x02)
#define MAX20328_STATUS2        (0x03)
#define MAX20328_REG_INTERRUPT  (0x04)
#define MAX20328_REG_MASK		(0x05)

#define MAX20328_CTL1           (0x06)
#define MAX20328_CTL2           (0x07)
#define MAX20328_CTL3           (0x08)
#define MAX20328_ADC_CTL1           (0x09)
#define MAX20328_ADC_CTL2           (0x0A)
#define MAX20328_HIHS_VAL           (0x0B)
#define MAX20328_OMTP_VAL           (0x0C)
#define MAX20328_DEFAULT1           (0x0D)
#define MAX20328_DEFAULT2           (0x0E)

#define MAX20328_SW_MODE_MASK       (0x7)

struct max20328_data {
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct mutex lock;
	struct mutex i2clock;
	struct mutex activelock;
	s32 int_pin;
	s32 hs_det_pin;
	s32 max_irq;
	u8 irq_state;
	u8 reg_read_buf;
};

#endif
