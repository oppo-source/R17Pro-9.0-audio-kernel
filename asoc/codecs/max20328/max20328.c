/*
* Copyright (c) 2018 JY Kim, jy.kim@maximintegrated.com
* Copyright (c) 2017 Maxim Integrated Products, Inc.
*
* This software is licensed under the terms of the GNU General Public
* License, as published by the Free Software Foundation, and
* may be copied, distributed, and modified under those terms.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
* You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.
* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "max20328.h"

#define REG_CFG_CTL1_SW_EN			(0x13)  			// switch operation is on, default switch mode seletcion
#define REG_CFG_CTL1_SW_AUDIO		(0x16)  			// switch operation is on, default switch mode seletcion
#define REG_CFG_CTL1_SW_DIS			(0x03)  			// switch operation is on, default switch mode seletcion
#define REG_CFG_ADC_CTL1_ADC_OFF	(0x30)  			// ADC is always off, detection is not used in mux switch.
#define REG_CFG_ADC_CTL1_ADC_ON		(0x33)  			// manual ADC detection, detection is not used in mux switch.
#define REG_CFG_ADC_CTL2_ADC_DET	(0x51)  			// manual ADC detection, detection is not used in mux switch.
#define REG_CFG_CTL2_FORCE_MODE		(0xFB)              // force mode for mode switch
#define REG_CFG_CTL2_FORCE_MODE_2	(0xC9)              // force mode for mode switch
#define REG_CFG_CTL2_FORCE_MODE_3	(0xC8)              // force mode for mode switch
#define REG_CFG_CTL3_USB_MODE		(0x46)              // usb data connection
#define REG_CFG_CTL3_AUDIO_MODE		(0xA6)              // audio accessory connection
#define REG_CFG_CTL3_UART_MODE		(0x03)              // UART connection
#define SBU1_MIC_SBU2_AGND			(0x06)              // MIC to SBU1 AGND to SBU2
#define SBU1_AGND_SBU2_MIC			(0x09)              // MIC to SBU2 AGND to SBU1

#define STA1_EBO_BITS_MASK			(0x08)
#define CTL3_MIC_AGND_BITS_MASK		(0x0F)

#ifdef VENDOR_EDIT
/* Huiqun.Han@PSW.MM.AudioDriver.Machine, 2018/07/21, Add for usb type-c audio engineer test */
#define CTL2_MAN_SBU_MASK			(0x20)
#define ADC_CTL1_MG_CHK_DIS_MASK	(0x40)
#define MAX20328_STATUS1_SBU_MASK	(0x04)
#define SBU_CFG_STATE_OPEN			(0)
#define SBU_CFG_STATE_SBU1_MIC		(1)
#define SBU_CFG_STATE_SBU2_MIC		(2)
#endif /* VENDOR_EDIT */

#define MAX20328_USB_MODE			(0)                     // working in usb connection
#define MAX20328_AUDIO_MODE			(1)                     // working in audio accessory connection
#define MAX20328_LOWPOWER_MODE		(2)                     // working in low power mode, minize power consumption
#define MAX20328_DISP_UART_MODE		(3)                     // working in UART mode

// need to be fine tuned
#define MAX20328_INTERVAL_MS		(5)
#define MAX20328_CHK_TIMES			(50)
#ifdef VENDOR_EDIT
/* Huiqun.Han@PSW.MM.AudioDriver.Machine, 2018/08/25, Add for usb type-c audio */
#define MAX20328_HIHS_REF			(0xBE)
#else
#define MAX20328_HIHS_REF			(0x20)
#endif
#ifdef VENDOR_EDIT
/* Huiqun.Han@PSW.MM.AudioDriver.Machine, 2018/08/09, Add for usb type-c audio */
#define MAX20328_OMTP_REF			(0x46)					//300 ohm
#else
#define MAX20328_OMTP_REF			(0x2E)
#endif /* VENDOR_EDIT */

#ifdef VENDOR_EDIT
/* Huiqun.Han@PSW.MM.AudioDriver.Machine, 2018/08/09, Add for usb type-c audio */
#define MAX20328_ADC_DET_DLY_S_1_1MA 20
#define MAX20328_ADC_DET_DLY_S_100UA 10
#define MAX20328_ADC_CTL2_5V_MASK (0xF0)
//#define MAX20328_DEBUG				1
static u8 max20328_aud_adc_val;
static bool max20328_aud_adc_max_curr;
#endif /* VENDOR_EDIT */

static const int delay_msec[] = {5, 10, 15, 20};        // for chip status checking loop
static struct max20328_data *s_max20328 = NULL;         // static variable for copy of driver data

/* I2C function */
static int max20328_write_reg(struct max20328_data *device,
	u8 reg_addr, u8 data)
{
	int err;
	int tries = 0;
	u8 buffer[2] = { reg_addr, data };
	struct i2c_msg msgs[] = {
		{
			.addr = device->client->addr,
			.flags = device->client->flags & I2C_M_TEN,
			.len = 2,
			.buf = buffer,
		},
	};

	do {
		mutex_lock(&device->i2clock);
		err = i2c_transfer(device->client->adapter, msgs, 1);
		mutex_unlock(&device->i2clock);
		if (err != 1)
			msleep_interruptible(MAX20328_I2C_RETRY_DELAY);
	} while ((err != 1) && (++tries < MAX20328_I2C_MAX_RETRIES));

	if (err != 1) {
		pr_err("%s -write transfer error\n", __func__);
		err = -EIO;
		return err;
	}
	return 0;
}

static int max20328_read_reg(struct max20328_data *data,
	u8 *buffer, int length)
{
	int err = -1;
	int tries = 0; /* # of attempts to read the device */
	int addr = 0;
	struct i2c_msg msgs[] = {
		{
			.addr = data->client->addr,
			.flags = data->client->flags & I2C_M_TEN,
			.len = 1,
			.buf = buffer,
		},
		{
			.addr = data->client->addr,
			.flags = (data->client->flags & I2C_M_TEN) | I2C_M_RD,
			.len = length,
			.buf = buffer,
		},
	};
	addr = *buffer;
	do {
		mutex_lock(&data->i2clock);
		err = i2c_transfer(data->client->adapter, msgs, 2);
		mutex_unlock(&data->i2clock);
		if (err != 2)
			msleep_interruptible(MAX20328_I2C_RETRY_DELAY);
	} while ((err != 2) && (++tries < MAX20328_I2C_MAX_RETRIES));

	if (err != 2) {
		pr_err("%s -read transfer error, at %x\n", __func__, addr);
		err = -EIO;
	} else
		err = 0;

	return err;
}

static inline int check_bit_valid(u8 reg, u8 bit_mask, int delay_ms, int try_num, bool invert)
{
	int i= 0;
	u8 val = 0;

	if (invert)
		bit_mask = ~bit_mask;

	/* val = reg; */
	/* max20328_read_reg(s_max20328, &val, 1); */

	for (i = 0; i < try_num; i++) {
		mdelay(delay_ms);
		val = reg;
		max20328_read_reg(s_max20328, &val, 1);
		if (invert){
			if( (val | bit_mask) == bit_mask ){
				pr_info("max20328 -check valid for reg(%x), bit_mask %x try times %d, invert\n", reg, bit_mask, i);
				return 1;
			}
		} else {
			if( (val & bit_mask) == bit_mask ){
				pr_info("max20328 -check valid for reg(%x), bit_mask %x try times %d\n", reg, bit_mask, i);
				return 1;
			}
		}
	}

	pr_info("max20328 %s -read status %x, bit_mask %x\n", __func__, val, bit_mask);
	return 0;
}

#ifdef VENDOR_EDIT
/*Mark.Yao@PSW.MM.Display.LCD.Feature,2018-07-12 add support for displayport*/
#define REG_CFG_CTL2_MAN_TXRX_FORCE_MODE		0x08
#define REG_CFG_CTL2_FORCE_TXRX				0x01
#define REG_CFG_CTL1_CC_POS				0x20
/*Mark.Yao@PSW.MM.Display.LCD.Stable,2018-09-24 add interface to make sure chip ready */
static int max20328_chip_initial = false;
bool max20328_chip_ready(void)
{
	return max20328_chip_initial;
}
EXPORT_SYMBOL(max20328_chip_ready);

int max20328_enable_displayport(bool enable, bool flip)
{
	struct max20328_data *data = s_max20328;
	u8 val = 0;
	int rc;

	if (!data) {
		pr_err("max20328 not ready\n");
		return -ENODEV;
	}

	val = MAX20328_CTL2;
	rc = max20328_read_reg(data, &val, 1);
	if (rc < 0) {
		pr_err("failed to read max20328 CTL2 register\n");
		return rc;
	}

	if (enable) {
		val |= REG_CFG_CTL2_MAN_TXRX_FORCE_MODE;
		val &= ~REG_CFG_CTL2_FORCE_TXRX;
	} else {
		val &= ~REG_CFG_CTL2_MAN_TXRX_FORCE_MODE;
		val |= REG_CFG_CTL2_FORCE_TXRX;
	}

	rc = max20328_write_reg(data, MAX20328_CTL2, val);
	if (rc < 0) {
		pr_err("failed to read max20328 CTL2 register\n");
		return rc;
	}

	val = MAX20328_CTL1;
	rc = max20328_read_reg(data, &val, 1);
	if (rc < 0) {
		pr_err("failed to read max20328 CTL1 register\n");
		return rc;
	}

	if (flip)
		val &= ~REG_CFG_CTL1_CC_POS;
	else
		val |= REG_CFG_CTL1_CC_POS;

	rc = max20328_write_reg(data, MAX20328_CTL1, val);
	if (rc < 0) {
		pr_err("failed to read max20328 CTL1 register\n");
		return rc;
	}

	pr_info("Max20328 switch Success: enable[%d] flip[%d]\n", enable, flip);

	return rc;
}
EXPORT_SYMBOL(max20328_enable_displayport);
#endif /* VENDOR_EDIT */

int max20328_enable_FM(int enable_flag)
{
	int rc = -1;
    u8 val = 0;
    struct max20328_data * data = NULL;

	pr_info("%s\n", __func__);

    if (s_max20328){
		data = s_max20328;
        val = MAX20328_CTL3;
        rc = max20328_read_reg(data, &val, 1);

		if (enable_flag){
			val = ((val & ~0x3) | 0x3);             // force mgs
		}else{
			val = ((val & ~0x3) | ((~(val >> 2)) & 0x3));             // recovery status.
		}
    }
	return rc;
}
EXPORT_SYMBOL(max20328_enable_FM);

int max20328_Device_RDY(void)
{
	return check_bit_valid(0x02, 0x02, 1, 20, false);
}
EXPORT_SYMBOL(max20328_Device_RDY);

int max20328_ADC_val(long *adc_val)
{
	int rc;
	long idet;
	u8 get = 0x02;
	rc = max20328_read_reg(s_max20328, &get, 1);
	if (get & 0x1){                         // EOC checking
		switch (get & 0xC0){
			case 0xC0:
				idet = 55;
				break;
			case 0x80:
				idet = 11;
				break;
			default:
				idet = 1;
				break;
		}
		get = 0x01;
		rc = max20328_read_reg(s_max20328, &get, 1);
		*adc_val = 47460;
        *adc_val = (*adc_val * get)/idet;

		return 0;
	}
	return -1;
}
EXPORT_SYMBOL(max20328_ADC_val);

int max20328_STATUS1_VAL(u8 *val)
{
	*val = 0x02;
	return max20328_read_reg(s_max20328, val, 1);
}
EXPORT_SYMBOL(max20328_STATUS1_VAL);

int max20328_STATUS2_VAL(u8 *val)
{
	*val = 0x03;
	return max20328_read_reg(s_max20328, val, 1);
}
EXPORT_SYMBOL(max20328_STATUS2_VAL);

int max20328_swap_mic_gnd(void)
{
	int rc = -1;
    u8 val = 0;
    struct max20328_data * data = NULL;
#ifdef VENDOR_EDIT
/* Huiqun.Han@PSW.MM.AudioDriver.Machine, 2018/07/31, Add for usb type-c audio */
#ifdef MAX20328_DEBUG
	int i;
#endif
#endif /* VENDOR_EDIT */
	pr_info("%s\n", __func__);

    if (s_max20328){
		data = s_max20328;
        val = MAX20328_CTL2;
        max20328_read_reg(data, &val, 1);
        pr_info("%s: swtich detect type: %s\n", __func__, (val & CTL2_MAN_SBU_MASK)?"mannual":"auto");
        if (val & CTL2_MAN_SBU_MASK) {
            //force_sbu_mg
            val = MAX20328_CTL3;
            rc = max20328_read_reg(data, &val, 1);
            pr_info("%s reg MAX20328_CTL3 - %x \n", __func__, val);
            switch((val & CTL3_MIC_AGND_BITS_MASK)){
                case SBU1_MIC_SBU2_AGND:                 //SBU1 to MIC and SBU2 to AGND, GM_S to GSNS
                    val = ((val & ~CTL3_MIC_AGND_BITS_MASK) | SBU1_AGND_SBU2_MIC);
                    rc = max20328_write_reg(data, MAX20328_CTL3, val);

                    val = MAX20328_CTL3;
                    max20328_read_reg(data, &val, 1);

                    break;
                case SBU1_AGND_SBU2_MIC:                 //SBU2 to MIC and SBU1 to AGND, MG_S to GSNS
                    val = ((val & ~CTL3_MIC_AGND_BITS_MASK) | SBU1_MIC_SBU2_AGND);
                    rc = max20328_write_reg(data, MAX20328_CTL3, val);

                    val = MAX20328_CTL3;
                    max20328_read_reg(data, &val, 1);
                    break;
                default:                  // incorrect pairing, reset to 0x6
                    val = ((val & ~0xC) | ((~val) | 0xC));
                    max20328_write_reg(data, MAX20328_CTL3, val);
                    rc = 2;			// failure case , reset to SBU1_MIC_SBU2_AGND.
                    break;
            }
        } else {
            //auto mode
            val = MAX20328_ADC_CTL1;
            max20328_read_reg(data, &val, 1);
            pr_info("%s MAX20328_ADC_CTL1: %x\n", __func__,(val));
            if (val & ADC_CTL1_MG_CHK_DIS_MASK) {
                //audio sbu config disabled
                rc = -1;
            } else {
                //check sbu status
                val = MAX20328_STATUS1;
                max20328_read_reg(data, &val, 1);
                pr_info("%s MAX20328_STATUS1: %x\n", __func__,(val));

                if (val & MAX20328_STATUS1_SBU_MASK) {
                    //SBU2 to MIC and SBU1 to AGND
					rc = max20328_write_reg(data, 0x06, 0x13);
					mdelay(5);
					rc = max20328_write_reg(data, 0x06, 0x16);

					if ( check_bit_valid (0x02, 0x04, 5, 20, true) )
						rc = max20328_write_reg(data, 0x0A, 0xF1);
                } else {
                    //SBU1 to MIC and SBU2 to AGND
					rc = max20328_write_reg(data, 0x06, 0x33);
					mdelay(5);
					rc = max20328_write_reg(data, 0x06, 0x36);

					if ( check_bit_valid (0x02, 0x04, 5, 20, false) )
						rc = max20328_write_reg(data, 0x0A, 0xF1);
                }
            }
        }
#ifdef VENDOR_EDIT
#ifdef MAX20328_DEBUG
/* Huiqun.Han@PSW.MM.AudioDriver.Machine, 2018/07/31, Add for usb type-c audio */
		for (i=0; i<15; i++) {
			val = i;
			max20328_read_reg(data, &val, 1);
			pr_info("%s reg %x - %x \n", __func__, i, val);
		}
#endif
#endif /* VENDOR_EDIT */
    }
	return rc;
}
EXPORT_SYMBOL(max20328_swap_mic_gnd);

int max20328_set_switch_mode(int mode)
{
	int rc = -1;
	u8 val = 0;
	struct max20328_data * data = NULL;
#ifdef VENDOR_EDIT
	u8 hph_adc_val_side2 = 0;
/* Huiqun.Han@PSW.MM.AudioDriver.Machine, 2018/07/31, Add for usb type-c audio */
#ifdef MAX20328_DEBUG
	int i;
#endif
#endif /* VENDOR_EDIT */

	pr_info("%s -mode(%d)\n", __func__, mode);

	if (s_max20328){
		data = s_max20328;
		switch(mode){
            case MAX20328_USB_MODE:                 //USB mode

                rc = max20328_write_reg(data, 0x0E, 0x00);         //DEF register2 set 00
                rc = max20328_write_reg(data, 0x0D, 0x40);         //DEF register1 set TOP side closed in data connection, bottom side is open
                rc = max20328_write_reg(data, 0x07, 0x00);         //CONTROL2 register, switch state NOT Force mode nor follow MODE[0:2]
                rc = max20328_write_reg(data, 0x08, 0x00);         //CONTROL3 register, force value is not use, anyway default it.
                rc = max20328_write_reg(data, 0x09, 0x30);         //ADC CONTROL1, ADC is always off on USB MODE
                rc = max20328_write_reg(data, 0x06, 0x13);         //CONTROL1 register, switch enable, default programmable with registers 0x0D and 0x0E
				break;
			case MAX20328_AUDIO_MODE:                //audio accessory
				#ifdef VENDOR_EDIT
				/* Huiqun.Han@PSW.MM.AudioDriver.Machine, 2018/07/31, Add for usb type-c audio */
				max20328_aud_adc_val = 0;
				max20328_aud_adc_max_curr = false;
				hph_adc_val_side2 = 0;
				#endif /* VENDOR_EDIT */
				rc = max20328_write_reg(data, 0x0E, 0x00);         //DEF register2
				rc = max20328_write_reg(data, 0x0D, 0x40);         //DEF register
				rc = max20328_write_reg(data, 0x0A, 0xF0);         //ADC CONTROL2
				rc = max20328_write_reg(data, 0x07, 0x02);         //CONTROL2 register
				rc = max20328_write_reg(data, 0x08, 0x00);         //CONTROL3 register
				rc = max20328_write_reg(data, 0x0B, MAX20328_HIHS_REF);         // High Impedance Threashold
				rc = max20328_write_reg(data, 0x0C, MAX20328_OMTP_REF);         // OMTP headset Detection Threshold
				rc = max20328_write_reg(data, 0x09, 0x19);         // ADC CONTROL1, ADC_CTL=01, disable open detection
				rc = max20328_write_reg(data, 0x06, 0x16);         // CONTROL1 register, switch enable, single Audio accessory

				/* if ( check_bit_valid (0x02, 0x04, 1, 5, true) ) {               // orientation is correct. */
					rc = max20328_write_reg(data, 0x0A, 0xF1);         //ADC CONTROL2, manual trigger adc detect

					if ( check_bit_valid(0x02, 0x01, MAX20328_INTERVAL_MS, MAX20328_CHK_TIMES, false) ) {       // EOC checking to max 5ms ~ 100ms delay
						val = 0x1;
						max20328_read_reg(data, &val, 1);

						pr_info("%s -adc1 value(%x)\n", __func__, val);
						#ifdef VENDOR_EDIT
						/* Huiqun.Han@PSW.MM.AudioDriver.Machine, 2018/07/31, Add for usb type-c audio */
						max20328_aud_adc_val = val;
						max20328_aud_adc_max_curr = false;
						#endif /* VENDOR_EDIT */
						if ( val > MAX20328_OMTP_REF ){                                    // overflow and enter special case to detection
							rc = max20328_write_reg(data, 0x06, 0x33);         // switch to default mode for next starting ADC detection
							mdelay(5);
							rc = max20328_write_reg(data, 0x06, 0x36);         // start manual ADC detection again and CC_POS = 1, so MIC and AGND definitly swapping
							/* rc = max20328_write_reg(data, 0x06, 0x16);         // start manual ADC detection again and CC_POS = 1, so MIC and AGND definitly swapping */

							if ( check_bit_valid (0x02, 0x04, 5, 20, false) ) {       // make sure switching is completed
								rc = max20328_write_reg(data, 0x0A, 0xF1);         // start up ADC manually

								if ( check_bit_valid(0x02, 0x01, MAX20328_INTERVAL_MS, MAX20328_CHK_TIMES, false) ) {       // EOC checking to max 5ms ~ 100ms delay
									val = 0x1;
									max20328_read_reg(data, &val, 1);

									pr_info("%s -adc2 value(%x)\n", __func__, val);
									#ifdef VENDOR_EDIT
									/* Huiqun.Han@PSW.MM.AudioDriver.Machine, 2018/07/31, Add for usb type-c audio */
									max20328_aud_adc_val = val;
									max20328_aud_adc_max_curr = false;
									#endif /* VENDOR_EDIT */
									if ( val > MAX20328_OMTP_REF ){
										#ifdef VENDOR_EDIT
										mdelay(MAX20328_ADC_DET_DLY_S_1_1MA);
										#endif /* VENDOR_EDIT */
										rc = max20328_write_reg(data, 0x09, 0x15);         // ADC CONTROL1, ADC_CTL=01, detect current to 100uA
										rc = max20328_write_reg(data, 0x0A, 0xF1);         // CONTROL1 register, switch enable, single Audio accessory

										if ( check_bit_valid (0x02, 0x01, MAX20328_INTERVAL_MS, MAX20328_CHK_TIMES, false) ) {       // EOC checking
											val = 0x1;
											max20328_read_reg(data, &val, 1);              // current ADC value
											pr_info("%s -adc3 value(%x)\n", __func__, val);
											#ifdef VENDOR_EDIT
											mdelay(MAX20328_ADC_DET_DLY_S_100UA);
											/* Huiqun.Han@PSW.MM.AudioDriver.Machine, 2018/07/31, Add for usb type-c audio */
											max20328_aud_adc_val = val;
											max20328_aud_adc_max_curr = true;
											hph_adc_val_side2 = val;
											{
											#else
											if ( val > MAX20328_HIHS_REF) {
											#endif /* VENDOR_EDIT */
												rc = max20328_write_reg(data, 0x06, 0x13);         // switch to default mode for next starting ADC detection
												rc = max20328_write_reg(data, 0x06, 0x16);         // start manual ADC detection again and CC_POS = 1, so MIC and AGND definitly swapping
												rc = max20328_write_reg(data, 0x09, 0x15);         // ADC CONTROL1, ADC_CTL=01, detect current to 100uA

												if ( check_bit_valid (0x02, 0x04, 2, 5, true) ) {               // EOC and MIC/AGND switch position is finalized and device is ready.
													rc = max20328_write_reg(data, 0x0A, 0xF1);         // CONTROL1 register, switch enable, single Audio accessory
													#ifdef VENDOR_EDIT
													/* Huiqun.Han@PSW.MM.AudioDriver.Machine, 2018/07/31, Add for usb type-c audio */
													if ( check_bit_valid(0x02, 0x01, MAX20328_INTERVAL_MS, MAX20328_CHK_TIMES, false) ) {       // EOC checking to max 5ms ~ 100ms delay
														val = 0x1;
														max20328_read_reg(data, &val, 1);
														pr_info("%s -adc4 value(%x)\n", __func__, val);
														max20328_aud_adc_val = val;
														max20328_aud_adc_max_curr = true;
														if (val > hph_adc_val_side2) {
															rc = max20328_write_reg(data, 0x06, 0x33); 
															rc = max20328_write_reg(data, 0x06, 0x36); 

															if ( check_bit_valid (0x02, 0x04, 5, 20, false) ) {       // make sure switching is completed
															#if 0
																rc = max20328_write_reg(data, 0x0A, 0xF1);         // start up ADC manually
																if ( check_bit_valid(0x02, 0x01, MAX20328_INTERVAL_MS, MAX20328_CHK_TIMES, false) ) {       // EOC checking to max 5ms ~ 100ms delay
																	val = 0x1;
																	max20328_read_reg(data, &val, 1);
																	pr_info("%s -adc4 value(%x)\n", __func__, val);
																	max20328_aud_adc_val = val;
																	max20328_aud_adc_max_curr = true;
																}
															#else
																pr_info("%s -adc use value(%x)\n", __func__, hph_adc_val_side2);
																max20328_aud_adc_val = hph_adc_val_side2;
																max20328_aud_adc_max_curr = true;
															#endif
															}
														}
													}
													#endif
												}
											}
										}
									}
								}
							}
						}
					}
				/* } */
				break;
			case MAX20328_LOWPOWER_MODE:							//low power mode
				rc = max20328_write_reg(data, 0x06, 0x03);         // disable switch operation
				break;
			case MAX20328_DISP_UART_MODE:
                rc = max20328_write_reg(data, 0x06, 0x14);         //CONTROL1 register,  UART MODE Top side USB switchs connected.
			break;
			default:
				break;
		}
#ifdef VENDOR_EDIT
/* Huiqun.Han@PSW.MM.AudioDriver.Machine, 2018/07/31, Add for usb type-c audio */
#ifdef MAX20328_DEBUG
		for (i=0; i<15; i++) {
			val = i;
			max20328_read_reg(data, &val, 1);
			pr_info("%s reg %x - %x \n", __func__, i, val);
		}
#endif
#endif /* VENDOR_EDIT */
	}
#ifdef VENDOR_EDIT
/*Mark.Yao@PSW.MM.Display.LCD.Stable,2018-09-24 add interface to make sure chip ready */
	if (!rc && !max20328_chip_initial) {
		pr_info("max20328 switch chip ready");
		max20328_chip_initial = true;
	}
#endif /* VENDOR_EDIT */

	return rc;
}
EXPORT_SYMBOL(max20328_set_switch_mode);

#ifdef VENDOR_EDIT
/* Huiqun.Han@PSW.MM.AudioDriver.Machine, 2018/07/21, Add for usb type-c audio engineer test */
int max20328_get_sbu_cfg_state(void)
{
	int rc = 0;
    u8 val = 0;
    struct max20328_data * data = NULL;

	pr_info("%s\n", __func__);

    if (s_max20328){
		data = s_max20328;

        val = MAX20328_CTL2;
        max20328_read_reg(data, &val, 1);
        pr_info("%s: swtich detect type: %s\n", __func__, (val & CTL2_MAN_SBU_MASK)?"mannual":"auto");
        if (val & CTL2_MAN_SBU_MASK) {
        //force_sbu_mg
            val = MAX20328_CTL3;
            max20328_read_reg(data, &val, 1);
            pr_info("%s MAX20328_CTL3: %x\n", __func__,(val));
            switch((val & CTL3_MIC_AGND_BITS_MASK)){
                case SBU1_MIC_SBU2_AGND:
                //SBU1 to MIC and SBU2 to AGND, GM_S to GSNS
                    rc = SBU_CFG_STATE_SBU1_MIC;
                    break;
                case SBU1_AGND_SBU2_MIC:
                //SBU2 to MIC and SBU1 to AGND, MG_S to GSNS
                    rc = SBU_CFG_STATE_SBU2_MIC;
                    break;
                default:
                // incorrect pairing or open
                    rc = SBU_CFG_STATE_OPEN;
                    break;
            }
        } else {
        //auto mode
            val = MAX20328_ADC_CTL1;
            max20328_read_reg(data, &val, 1);
            pr_info("%s MAX20328_ADC_CTL1: %x\n", __func__,(val));
            if (val & ADC_CTL1_MG_CHK_DIS_MASK) {
                //audio sbu config disabled
                rc = 0;
            } else {
                val = MAX20328_STATUS1;
                max20328_read_reg(data, &val, 1);
                pr_info("%s MAX20328_STATUS1: %x\n", __func__,(val));
                if (val & MAX20328_STATUS1_SBU_MASK) {
                //SBU2 to MIC and SBU1 to AGND
                    rc = SBU_CFG_STATE_SBU2_MIC;
                } else {
                //SBU1 to MIC and SBU2 to AGND
                    rc = SBU_CFG_STATE_SBU1_MIC;
                }
            }
        }
    }
	return rc;
}
EXPORT_SYMBOL(max20328_get_sbu_cfg_state);
bool max20328_ADC_RES_OF(void)
{
#ifdef MAX20328_DEBUG
/* Huiqun.Han@PSW.MM.AudioDriver.Machine, 2018/07/31, Add for usb type-c audio */
    int i;
    u8 val;
#endif /* VENDOR_EDIT */
	if (!s_max20328)
		return false;

#ifdef MAX20328_DEBUG
/* Huiqun.Han@PSW.MM.AudioDriver.Machine, 2018/07/31, Add for usb type-c audio */
	for (i=0; i<15; i++) {
		val = i;
		max20328_read_reg(s_max20328, &val, 1);
		pr_info("%s reg %x - %x \n", __func__, i, val);
	}
#endif
	if ((max20328_aud_adc_val > MAX20328_HIHS_REF) && max20328_aud_adc_max_curr)
		return false;
	else
		return true;
}
EXPORT_SYMBOL(max20328_ADC_RES_OF);
#endif /* VENDOR_EDIT */

irqreturn_t max20328_irq_handler(int max_irq, void *device)
{
	int err;
	u8 interupt_status;
	struct max20328_data *data = device;

	interupt_status = MAX20328_REG_INTERRUPT;
	err = max20328_read_reg(data, &interupt_status, 1);
	if (err == 0) {
		pr_info("%s INTTERUPT : %02x\n", __func__, interupt_status);
		input_report_rel(data->input_dev, REL_X, (s32)interupt_status);
		input_sync(data->input_dev);
	} else {
		pr_err("%s INTERRUPT read fail:%d\n", __func__, err);
	}

	return IRQ_HANDLED;
}

static int max20328_parse_dt(struct max20328_data *data,
	struct device *dev)
{
	struct device_node *dNode = dev->of_node;
	enum of_gpio_flags flags;
#ifndef VENDOR_EDIT
/* Huiqun.Han@PSW.MM.AudioDriver.Machine, 2018/06/29, remove unused variable */
	int ret;
#endif /* VENDOR_EDIT */
	if (dNode == NULL)
		return -ENODEV;

	data->int_pin = of_get_named_gpio_flags(dNode,
		"oppo,max20328-irq", 0, &flags);
	if (data->int_pin < 0) {
		pr_err("%s - get int error\n", __func__);
		return -ENODEV;
	}
#ifndef VENDOR_EDIT
/* Huiqun.Han@PSW.MM.AudioDriver.Machine, 2018/06/29, Add for usb type-c audio */
	data->hs_det_pin = of_get_named_gpio(dNode,
		"max20328,hs-det-gpio", 0);
	if (data->hs_det_pin < 0) {
		pr_err("%s - get int error\n", __func__);
		return -ENODEV;
	}
	ret = gpio_request(data->hs_det_pin, "max20328_hs_det");

	gpio_direction_output(data->hs_det_pin, 1);
#endif /* VENDOR_EDIT */
	return 0;
}

static int max20328_setup_irq(struct max20328_data *data)
{
#ifdef NEED_IRQ
	int errorno = -EIO;

	errorno = request_threaded_irq(data->max_irq, NULL,
		max20328_irq_handler, IRQF_TRIGGER_FALLING|IRQF_ONESHOT,
		"max20328_irq", data);

	if (errorno < 0) {
		pr_err("%s - failed for setup max_irq errono= %d\n",
			   __func__, errorno);
		errorno = -ENODEV;
		return errorno;
	}
	data->irq_state = 0;
	disable_irq(data->max_irq);

	return errorno;
#else
	return 0;
#endif
}

static int max20328_setup_gpio(struct max20328_data *data)
{
	int errorno = -EIO;

	errorno = gpio_request(data->int_pin, "sen_int");

	errorno = gpio_direction_input(data->int_pin);
	if (errorno) {
		pr_err("%s - failed to set int_pin as input\n", __func__);
		goto err_gpio_direction_input;
	}
	data->max_irq = gpio_to_irq(data->int_pin);

	goto done;
err_gpio_direction_input:
	gpio_free(data->int_pin);
done:
	return errorno;
}

static void max20328_irq_set_state(struct max20328_data *data, int irq_enable)
{
	pr_info("%s - irq_enable : %d, irq_state : %d\n",
		__func__, irq_enable, data->irq_state);

	if (irq_enable) {
		if (data->irq_state == 0) {
			data->irq_state = 1;
			enable_irq(data->max_irq);
		}
	} else {
		if (data->irq_state == 0)
			return;
		if (data->irq_state == 1) {
			disable_irq(data->max_irq);
			data->irq_state = 0;
		}
	}
}

/* Register Read Access */
static ssize_t max20328_read_reg_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct max20328_data *data = dev_get_drvdata(dev);

	pr_info("%s - val=0x%02x\n", __func__, data->reg_read_buf);

	return snprintf(buf, PAGE_SIZE, "%02x\n", (u32)data->reg_read_buf);
}

static ssize_t max20328_read_reg_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	int err;
	unsigned int cmd = 0;
	u8 recvData;

	struct max20328_data *data = dev_get_drvdata(dev);

	mutex_lock(&data->activelock);
	err = sscanf(buf, "%2x", &cmd);
	if (err == 0) {
		pr_info("%s - sscanf fail\n", __func__);
		mutex_unlock(&data->activelock);
		return size;
	}

	recvData = (u8)cmd;
	err = max20328_read_reg(data, &recvData, 1);
	if (err != 0) {
		pr_err("%s err=%d, val=0x%02x\n",
			__func__, err, recvData);
		mutex_unlock(&data->activelock);
		return size;
	}
	data->reg_read_buf = recvData;
	mutex_unlock(&data->activelock);

	return size;
}

static ssize_t max20328_write_reg_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	int err;
	unsigned int cmd = 0;
	unsigned int val = 0;

	struct max20328_data *data = dev_get_drvdata(dev);

	mutex_lock(&data->activelock);
	err = sscanf(buf, "%2x %2x", &cmd, &val);

	if (err == 0) {
		pr_info("%s - sscanf fail\n", __func__);
		mutex_unlock(&data->activelock);
		return size;
	}

	pr_info("%s - addr:0x%02x, val:0x%02x\n", __func__, cmd, val);

	/* Enable/Disable IRQ */
	if (cmd == MAX20328_REG_MASK) {
		if ( val == 0x00 )
			max20328_irq_set_state(data, 0);
		else
			max20328_irq_set_state(data, 1);
	}

	err = max20328_write_reg(data, (u8)cmd, (u8)val);
	if (err != 0) {
		pr_err("%s err=%d, addr=0x%02x, val=0x%02x\n",
			__func__, err, cmd, val);
		mutex_unlock(&data->activelock);
		return size;
	}

	mutex_unlock(&data->activelock);

	return size;
}

static DEVICE_ATTR(read_reg, S_IRUGO|S_IWUSR|S_IWGRP,
	max20328_read_reg_show, max20328_read_reg_store);

static DEVICE_ATTR(write_reg, S_IRUGO|S_IWUSR|S_IWGRP,
	NULL, max20328_write_reg_store);

static struct attribute *max20328_sysfs_attrs[] = {
	&dev_attr_read_reg.attr,
	&dev_attr_write_reg.attr,
	NULL
};
static struct attribute_group hrm_attribute_group = {
	.attrs = max20328_sysfs_attrs,
};

static void max20328_initialize(struct max20328_data *data)
{
	data->reg_read_buf = 0;
	return;
}

static int max20328_probe(struct i2c_client *client,
			       const struct i2c_device_id *devid)
{
	int i, err = 0;
	struct max20328_data *data;
	struct input_dev *input_dev;
	u8 recvData;

	pr_info("%s - called\n", __func__);

	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (data == NULL) {
		dev_err(&client->dev,
			"failed to allocate memory for module data\n");
		err = -ENOMEM;
		goto exit;
	}
    s_max20328 = data;
	data->client = client;

	err = max20328_parse_dt(data, &client->dev);
	if (err < 0) {
		pr_err("%s - of_node error\n", __func__);
		err = -ENODEV;
		goto err_of_node;
	}

	err = max20328_setup_gpio(data);
	if (err) {
		pr_err("%s - could not initialize resources\n", __func__);
		goto err_setup_gpio;
	}

	err = max20328_setup_irq(data);
	if (err) {
		pr_err("%s - could not setup irq\n", __func__);
		goto err_setup_irq;
	}

	mutex_init(&data->i2clock);
	mutex_init(&data->lock);
	mutex_init(&data->activelock);

	recvData = MAX20328_REG_DEVICE_ID;
	err = max20328_read_reg(data, &recvData, 1);
	if (err) {
		pr_err("%s WHOAMI read fail\n", __func__);
		err = -ENODEV;
		goto err_read_reg;
	} else {
		pr_info("MAX20328 is detected. ID: %X\n", recvData);
	}

#ifdef VENDOR_EDIT
/* Huiqun.Han@PSW.MM.AudioDriver.Machine, 2018/06/29, Add for usb type-c audio */
    recvData = 0x13;				//default value
    err = max20328_write_reg(data, 0x06, (u8)recvData);
	if (err != 0) {
		pr_err("%s failed to Enable. err=%d, addr=0x%02x, val=0x%02x\n",
			__func__, err, 0x06, recvData);
		goto err_read_reg;
	}
#endif /* VENDOR_EDIT */

	recvData = MAX20328_STATUS2;
	for (i = 0; i < ARRAY_SIZE(delay_msec); i++){
		recvData = MAX20328_STATUS2;
		err = max20328_read_reg(data, &recvData, 1);
		if (!err && (recvData & STA1_EBO_BITS_MASK)){
			break;										// chip is ready now , goto initial setting.
		}
		msleep(delay_msec[i]);
	}
	if (i >= ARRAY_SIZE(delay_msec)){
 		pr_err("%s chip bootup failed\n", __func__);
		err = -ENODEV;
		goto err_read_reg;
	}

#ifdef VENDOR_EDIT
/* Huiqun.Han@PSW.MM.AudioDriver.Machine, 2018/06/29, Add for usb type-c audio */
	recvData = MAX20328_ADC_CTL2;
	err = max20328_read_reg(data, &recvData, 1);
	if (err) {
		pr_err("%s read MAX20328_ADC_CTL2 fail\n", __func__);
		err = -ENODEV;
		goto err_read_reg;
	} else {
		if ((recvData & MAX20328_ADC_CTL2_5V_MASK) != MAX20328_ADC_CTL2_5V_MASK) {
			pr_info("%s Need to update OVP to 5V.\n", __func__);
			recvData = MAX20328_ADC_CTL2_5V_MASK;
			err = max20328_write_reg(data, MAX20328_ADC_CTL2, (u8)recvData);
			if (err != 0) {
				pr_err("%s err=%d, addr=0x%02x, val=0x%02x\n",
				__func__, err, MAX20328_ADC_CTL2, recvData);
				goto err_read_reg;
			}
			msleep(10);
		}
	}
#endif /* VENDOR_EDIT */
    recvData = 0x40;				//default value
 	err = max20328_write_reg(data, 0x0D, (u8)recvData);
	if (err != 0) {
		pr_err("%s err=%d, addr=0x%02x, val=0x%02x\n",
			__func__, err, 0x0D, recvData);
		goto err_read_reg;
	}

    recvData = 0x0;				//default value
 	err = max20328_write_reg(data, 0x0E, (u8)recvData);
	if (err != 0) {
		pr_err("%s err=%d, addr=0x%02x, val=0x%02x\n",
			__func__, err, 0x0E, recvData);
		goto err_read_reg;
	}

    recvData = 0xF0;				//default value
 	err = max20328_write_reg(data, 0x0A, (u8)recvData);
	if (err != 0) {
		pr_err("%s err=%d, addr=0x%02x, val=0x%02x\n",
			__func__, err, 0x0A, recvData);
		goto err_read_reg;
	}

    recvData = 0x0;				//default value
 	err = max20328_write_reg(data, 0x07, (u8)recvData);
	if (err != 0) {
		pr_err("%s err=%d, addr=0x%02x, val=0x%02x\n",
			__func__, err, 0x07, recvData);
		goto err_read_reg;
	}

    recvData = 0x0;				//default value
 	err = max20328_write_reg(data, 0x08, (u8)recvData);
	if (err != 0) {
		pr_err("%s err=%d, addr=0x%02x, val=0x%02x\n",
			__func__, err, 0x08, recvData);
		goto err_read_reg;
	}

    recvData = 0x30;				//ADC is always off and disable all
 	err = max20328_write_reg(data, 0x09, (u8)recvData);
	if (err != 0) {
		pr_err("%s err=%d, addr=0x%02x, val=0x%02x\n",
			__func__, err, 0x09, recvData);
		goto err_read_reg;
	}

    recvData = 0x13;					//default mode
 	err = max20328_write_reg(data, 0x06, (u8)recvData);
	if (err != 0) {
		pr_err("%s err=%d, addr=0x%02x, val=0x%02x\n",
			__func__, err, 0x06, recvData);
		goto err_read_reg;
	}

	input_dev = input_allocate_device();
	if (!input_dev) {
		pr_err("%s: could not allocate input device\n", __func__);
		err = -ENOMEM;
		goto err_input_allocate_device;
	}

	data->input_dev = input_dev;
	input_dev->name = "max20328";
	input_set_drvdata(input_dev, data);
	input_set_capability(input_dev, EV_REL, REL_X);

	err = input_register_device(input_dev);
	if (err < 0) {
		pr_err("%s: could not register input device\n", __func__);
		input_free_device(data->input_dev);
		goto err_input_register_device;
	}

	err = sysfs_create_group(&data->input_dev->dev.kobj,
				 &hrm_attribute_group);


	i2c_set_clientdata(client, data);
	dev_set_drvdata(&input_dev->dev, data);

	/* INIT VAR */
	max20328_initialize(data);

	return 0;

err_input_register_device:
err_input_allocate_device:
err_read_reg:
	mutex_destroy(&data->lock);
	mutex_destroy(&data->i2clock);
	mutex_destroy(&data->activelock);
err_setup_irq:
err_setup_gpio:
err_of_node:
	kfree(data);
#ifdef VENDOR_EDIT
/* Huiqun.Han@PSW.MM.AudioDriver.Machine.1564256, 2018/09/18, Add for usb type-c audio */
	s_max20328 = NULL;
#endif /* VENDOR_EDIT */
exit:
	return err;
}

static int max20328_remove(struct i2c_client *client)
{
	int err = 0;
	struct max20328_data *max20328_data = i2c_get_clientdata(client);

	input_unregister_device(max20328_data->input_dev);
	mutex_destroy(&max20328_data->lock);
	kfree(max20328_data);
	s_max20328 = NULL;
	pr_info("%s: \n", __func__);
	return err;
}

static int max20328_suspend(struct device *dev)
{
	struct max20328_data *data = dev_get_drvdata(dev);
	u8 val = REG_CFG_CTL1_SW_AUDIO;
	int err = 0;

	err = max20328_write_reg(data, MAX20328_CTL1, val);         // make sure enable switch operation
	return err;
}

static int max20328_resume(struct device *dev)
{
	struct max20328_data *data = dev_get_drvdata(dev);
	u8 val = REG_CFG_CTL1_SW_AUDIO;
	int err = 0;

	err = max20328_write_reg(data, MAX20328_CTL1, val);         // make sure enable switch operation
	return err;
}

static const struct dev_pm_ops max20328_pm_ops = {
	.suspend = max20328_suspend,
	.resume = max20328_resume
};

static const struct i2c_device_id max20328_id[] = {
	{ "max20328", 0 },
	{ }
};

static struct of_device_id max20328_match_table[] = {
	{ .compatible = "max20328",},
	{ },
};

MODULE_DEVICE_TABLE(i2c, max20328_id);

static struct i2c_driver max20328_driver = {
	.driver = {
		.name		= "max20328",
		.of_match_table = max20328_match_table,
	},
	.probe = max20328_probe,
	.remove = max20328_remove,
	.id_table = max20328_id,
};

static int __init max20328_init(void)
{
	return i2c_add_driver(&max20328_driver);
}

static void __exit max20328_exit(void)
{
	i2c_del_driver(&max20328_driver);
}

module_init(max20328_init);
module_exit(max20328_exit);

MODULE_AUTHOR("JY Kim <jy.kim@maximintegrated.com>");
MODULE_DESCRIPTION("max20328 Module driver");
MODULE_LICENSE("GPL");
