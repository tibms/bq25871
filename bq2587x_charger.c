/*
 * BQ2587x battery charging driver
 *
 * Copyright (C) 2013 Texas Instruments
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.

 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/power_supply.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/of_gpio.h>
#include "bq2587x_reg.h"



#define	ADC_REG_BASE	0x13

typedef enum e_adc_channel {
	ADC_VBUS,
	ADC_IBUS,
	ADC_VOUT,
	ADC_VDROP,
	ADC_VBAT,
	ADC_IBAT,
	ADC_TBUS,
	ADC_TBAT,
	ADC_DIE_TEMP,
	ADC_VUSB,
	ADC_MAX_CHANNEL
}tADCChannel;


enum bq2587x_part_no {
	BQ25870	= 0x01,
	BQ25871 = 0x02,
	BQ25872 = 0x03,
};

const char *part_name[] = {
	"null",
	"bq25870",
	"bq25871",
	"bq25872",
};

struct bq2587x_config {
	int		vbus_ovp_threshold;
	int		ibus_ocp_threshold;

	int		ibus_reg_threshold;
	int		vbat_reg_threshold;
	int		ibat_reg_threshold;
	int		vout_reg_threshold;

	int		tbus_ot_threshold;
	int		tbat_ot_threshold;

	int		ibus_irev_threshold;

	int		vdrop_alarm_threshold;
	int		vdrop_ovp_threshold;
};


struct bq2587x {
	struct device *dev;
	struct i2c_client *client;

	enum   bq2587x_part_no part_no;
	int    revision;

	bool	vbus_inserted;
	bool	bat_inserted;

	bool	vbus_ovp;
	bool	ibus_ocp;
	bool	ldo_active;
	bool	tbus_ot;
	bool	tbat_ot;
	bool	ibus_irev;
	bool	vdrop_alm;
	bool	vdrop_ovp;
	bool	tshutdown;
	bool	scp;     /* high current fault from vbus to vout */
	bool	vbat_ovp;
	bool	ibat_ocp;
	bool	vout_ovp;

	int		vusb_volt;
	int		vbus_volt;
	int		vout_volt;
	int		vbat_volt;
	int		vdrop_volt;
	int		ibus_current;
	int		ibat_current;
	int		tbus_temp;
	int		tbat_temp;
	int		die_temp;


	struct bq2587x_config	cfg;
	struct work_struct irq_work;
	struct work_struct adapter_in_work;
	struct work_struct adapter_out_work;
	struct delayed_work monitor_work;

	struct power_supply wall;
};


static struct bq2587x *g_bq;

static DEFINE_MUTEX(bq2587x_i2c_lock);

static int bq2587x_read_byte(struct bq2587x *bq, u8 *data, u8 reg)
{
	int ret;

	mutex_lock(&bq2587x_i2c_lock);
	ret = i2c_smbus_read_byte_data(bq->client, reg);
	if (ret < 0) {
		dev_err(bq->dev, "failed to read 0x%.2x\n", reg);
		mutex_unlock(&bq2587x_i2c_lock);
		return ret;
	}

	*data = (u8)ret;
	mutex_unlock(&bq2587x_i2c_lock);

	return 0;
}

static int bq2587x_write_byte(struct bq2587x *bq, u8 reg, u8 data)
{
	int ret;
	mutex_lock(&bq2587x_i2c_lock);
	ret = i2c_smbus_write_byte_data(bq->client, reg, data);
	mutex_unlock(&bq2587x_i2c_lock);
	return ret;
}


static int bq2587x_read_word(struct bq2587x *bq, u16 *data, u8 reg)
{
	int ret;

	mutex_lock(&bq2587x_i2c_lock);
	ret = i2c_smbus_read_word_data(bq->client,reg);
	if (ret < 0) {
		dev_err(bq->dev, "failed to read 0x%.2x\n", reg);
		mutex_unlock(&bq2587x_i2c_lock);
		return ret;
	}

	*data = (u16)ret;
	mutex_unlock(&bq2587x_i2c_lock);
	
	return 0;

}

static int bq2587x_update_bits(struct bq2587x *bq, u8 reg, u8 mask, u8 data)
{
	int ret;
	u8 tmp;

	ret = bq2587x_read_byte(bq, &tmp, reg);

	if (ret)
		return ret;

	tmp &= ~mask;
	tmp |= data & mask;

	return bq2587x_write_byte(bq, reg, tmp);
}


int bq2587x_set_evt_int_mask(u8 reg, u8 mask)
{
	int ret;
	u8 val;

	ret = bq2587x_read_byte(g_bq, &val, reg);
	if (ret)
		return ret;
	
	val |= mask;

	return bq2587x_write_byte(g_bq, reg, val);
}
EXPORT_SYMBOL_GPL(bq2587x_set_evt_int_mask);

int bq2587x_clear_evt_int_mask(u8 reg, u8 mask)
{
	int ret;
	u8 val;

	ret = bq2587x_read_byte(g_bq, &val, reg);
	if (ret)
		return ret;
	
	val &= ~mask;

	return bq2587x_write_byte(g_bq, reg, val);
}
EXPORT_SYMBOL_GPL(bq2587x_clear_evt_int_mask);


/*
 * enable protection in EVENT1 register
 */
int bq2587x_evt_enable(u8 reg, u8 mask)
{
	int ret;
	u8 val;

	ret = bq2587x_read_byte(g_bq, &val, reg);
	if (ret)
		return ret;
	
	val |= mask;

	return bq2587x_write_byte(g_bq, reg, val);

}
EXPORT_SYMBOL_GPL(bq2587x_evt_enable);


/*
 * disable protection in EVENT1 register
 */
int bq2587x_evt_disable(u8 reg, u8 mask)
{
	int ret;
	u8 val;

	ret = bq2587x_read_byte(g_bq, &val, reg);
	if (ret)
		return ret;
	
	val &= ~mask;

	return bq2587x_write_byte(g_bq, reg, val);

}
EXPORT_SYMBOL_GPL(bq2587x_evt_disable);

/*
 * read event register state, read two times to get current state
 */

int bq2587x_read_evt_status(u8 reg, u8* state)
{
	int ret;
	u8 val;

	if (reg < BQ2587X_REG_04 || reg > BQ2587X_REG_06)
		return -1;

	ret = bq2587x_read_byte(g_bq, &val,reg);
	if (ret)
		return ret;

	ret = bq2587x_read_byte(g_bq, &val,reg);
	if (ret)
		return ret;

	*state = val;

	return 0;
}
EXPORT_SYMBOL_GPL(bq2587x_read_evt_status);

int bq2587x_set_vbus_pulldown(bool enable)
{
	int ret;
	u8 val;
	if (enable)
		val = REG05_VBUS_PD_ENABLE;
	else
		val = REG05_VBUS_PD_DISABLE;

	val <<= REG05_VBUS_PD_EN_SHIFT;
	ret = bq2587x_update_bits(g_bq, BQ2587X_REG_05, REG05_VBUS_PD_EN, val);
	return ret;

}
EXPORT_SYMBOL_GPL(bq2587x_set_vbus_pulldown);


int bq2587x_set_sense_R(u8 moh)
{
	int ret;
	u8 val;
	if (moh == 5)
		val = REG06_SENSE_R_5MOH;
	else
		val = REG06_SENSE_R_10MOH;

	val <<= REG06_SENSE_R_SHIFT;
	ret = bq2587x_update_bits(g_bq, BQ2587X_REG_06, REG06_SENSE_R, val);
	return ret;
}
EXPORT_SYMBOL_GPL(bq2587x_set_sense_R);

int bq2587x_enable_charge(void)
{
	int ret;
	u8 val;
	val = REG06_CHG_EN_ENABLE << REG06_CHG_EN_SHIFT;
	ret = bq2587x_update_bits(g_bq, BQ2587X_REG_06, REG06_CHG_EN, val);
	return ret;
}
EXPORT_SYMBOL_GPL(bq2587x_enable_charge);

int bq2587x_disable_charge(void)
{
	int ret;
	u8 val;

	val = REG06_CHG_EN_DISABLE << REG06_CHG_EN_SHIFT;
	ret = bq2587x_update_bits(g_bq, BQ2587X_REG_06, REG06_CHG_EN, val);
	return ret;
}
EXPORT_SYMBOL_GPL(bq2587x_disable_charge);

/*
 * set watchdog disable or timeout, options are:
 *	REG06_WATCHDOG_DISABLE
 *	REG06_WATCHDOG_1S
 *	REG06_WATCHDOG_1P5S
 *	REG06_WATCHDOG_5S
 */
int bq2587x_set_watchdog(u8 timeout)
{
	int ret;
	u8 val;

	val = timeout << REG06_WATCHDOG_SHIFT;
	ret = bq2587x_update_bits(g_bq, BQ2587X_REG_06, REG06_WATCHDOG, val);
	return ret;
}
EXPORT_SYMBOL_GPL(bq2587x_set_watchdog);

/*
 * set reverse current protection threhsold, optionals are:
 *	REG06_RCP_0A or REG06_RCP_3A
 */
int bq2587x_set_rcp(u8 threshold)
{
	int ret;
	u8 val;

	val = threshold << REG06_RCP_SET_SHIFT;

	ret = bq2587x_update_bits(g_bq, BQ2587X_REG_06, REG06_RCP_SET, val);
	return ret;
}
EXPORT_SYMBOL_GPL(bq2587x_set_rcp);

/*
 * reset register
 */
int bq2587x_reg_reset(void)
{
	int ret;
	u8 val;

	val =  1 << REG06_REG_RST_SHIFT;

	ret = bq2587x_update_bits(g_bq, BQ2587X_REG_06, REG06_REG_RST, val);
	return ret;
}
EXPORT_SYMBOL_GPL(bq2587x_reg_reset);


/*
 * enable adc scanning
 */
int bq2587x_adc_start(void)
{
	int ret;
	u8 val;

	val =  REG07_ADC_ENABLE << REG07_ADC_EN_SHIFT;

	ret = bq2587x_update_bits(g_bq, BQ2587X_REG_07, REG07_ADC_EN, val);
	return ret;

}
EXPORT_SYMBOL_GPL(bq2587x_adc_start);

/*
 * disable adc scanning
 */

int bq2587x_adc_stop(void)
{
	int ret;
	u8 val;

	val =  REG07_ADC_DISABLE << REG07_ADC_EN_SHIFT;

	ret = bq2587x_update_bits(g_bq, BQ2587X_REG_07, REG07_ADC_EN, val);
	return ret;

}
EXPORT_SYMBOL_GPL(bq2587x_adc_stop);

int bq2587x_adc_set_rate(bool oneshot)
{
	int ret;
	u8 val;

	if (oneshot)
		val = REG07_ADC_RATE_ONESHOT;
	else
		val = REG07_ADC_RATE_CONTINOUS;
		
	val <<= REG07_ADC_RATE_SHIFT;

	ret = bq2587x_update_bits(g_bq, BQ2587X_REG_07, REG07_ADC_RATE, val);
	return ret;
}
EXPORT_SYMBOL_GPL(bq2587x_adc_set_rate);

int bq2587x_adc_set_average(bool enable)
{
	int ret;
	u8 val;

	if (enable)
		val = REG07_ADC_AVG_ENABLE;
	else
		val = REG07_ADC_AVG_DISABLE;
	
	val <<= REG07_ADC_AVG_SHIFT;

	ret = bq2587x_update_bits(g_bq, BQ2587X_REG_07, REG07_ADC_AVG_EN, val);
	return ret;
}
EXPORT_SYMBOL_GPL(bq2587x_adc_set_average);

int bq2587x_adc_set_samples(u8 samples)
{
	int ret;
	u8 val;

	if (samples == 8)
		val = REG07_ADC_SAMPLES_8;
	else
		val = REG07_ADC_SAMPLES_16;
	
	val <<= REG07_ADC_SAMPLES_SHIFT;

	ret = bq2587x_update_bits(g_bq, BQ2587X_REG_07, REG07_ADC_SAMPLES, val);
	return ret;
}
EXPORT_SYMBOL_GPL(bq2587x_adc_set_samples);

int bq2587x_adc_init(bool oneshot, bool average, u8 samples)
{
	int ret;
	u8 val = 0;
	
	if (oneshot)
		val |= REG07_ADC_RATE_ONESHOT << REG07_ADC_RATE_SHIFT;
	else
		val |= REG07_ADC_RATE_CONTINOUS << REG07_ADC_RATE_SHIFT;

	if (average)
		val |= REG07_ADC_AVG_ENABLE << REG07_ADC_AVG_SHIFT;
	else
		val |= REG07_ADC_AVG_DISABLE << REG07_ADC_AVG_SHIFT;

	if (samples == 8)
		val |= REG07_ADC_SAMPLES_8 << REG07_ADC_SAMPLES_SHIFT;
	else
		val |= REG07_ADC_SAMPLES_16 << REG07_ADC_SAMPLES_SHIFT;

	ret = bq2587x_write_byte(g_bq, BQ2587X_REG_07, val);

	return ret;

}
EXPORT_SYMBOL_GPL(bq2587x_adc_init);


int bq2587x_adc_set_scan_enable(u8 reg, u8 mask)
{
	int ret;
	u8 val;

	ret = bq2587x_read_byte(g_bq, &val, reg);
	if (ret)
		return ret;

	val |= mask;

	return bq2587x_write_byte(g_bq, reg, val);
}
EXPORT_SYMBOL_GPL(bq2587x_adc_set_scan_enable);

int bq2587x_adc_clear_scan_enable(u8 reg, u8 mask)
{
	int ret;
	u8 val;

	ret = bq2587x_read_byte(g_bq, &val, reg);
	if (ret)
		return ret;
	
	val &= ~mask;

	return bq2587x_write_byte(g_bq, reg, val);
}
EXPORT_SYMBOL_GPL(bq2587x_adc_clear_scan_enable);

int bq2587x_set_ocp_response_mode(bool blank)
{
	int ret;
	u8 val;

	if (blank)
		val = REG09_OCP_RES_BLANK;
	else
		val = REG09_OCP_RES_HICCUP;
	
	val <<= REG09_OCP_RES_SHIFT;

	ret = bq2587x_update_bits(g_bq, BQ2587X_REG_09, REG09_OCP_RES, val);
	return ret;
}
EXPORT_SYMBOL_GPL(bq2587x_set_ocp_response_mode);

int bq2587x_set_vbus_ovp_delay(u8 us)
{
	int ret;
	u8 val;

	if (us == 8)
		val = REG09_VBUS_OVP_DLY_8US;
	else
		val = REG09_VBUS_OVP_DLY_128US;
	
	val <<= REG09_VBUS_OVP_DLY_SHIFT;

	ret = bq2587x_update_bits(g_bq, BQ2587X_REG_09, REG09_VBUS_OVP_DLY, val);
	return ret;
}
EXPORT_SYMBOL_GPL(bq2587x_set_vbus_ovp_delay);


int bq2587x_set_ibus_ocp_threshold(int ocp_th)
{
	int ret;
	u8 val;

	val = (u8)((ocp_th - REG09_IBUS_OCP_BASE)/REG09_IBUS_OCP_LSB);
	val <<= REG09_IBUS_OCP_SHIFT;

	ret = bq2587x_update_bits(g_bq, BQ2587X_REG_09, REG09_IBUS_OCP, val);
	return ret;
}
EXPORT_SYMBOL_GPL(bq2587x_set_ibus_ocp_threshold);

int bq2587x_set_vbus_ovp_threshold(int ovp_th)
{
	int ret;
	u8 val;

	val = (u8)((ovp_th - REG0A_VBUS_OVP_BASE)/REG0A_VBUS_OVP_LSB);
	val <<= REG0A_VBUS_OVP_SHIFT;

	ret = bq2587x_update_bits(g_bq, BQ2587X_REG_0A, REG0A_VBUS_OVP, val);
	return ret;
}
EXPORT_SYMBOL_GPL(bq2587x_set_vbus_ovp_threshold);

int bq2587x_set_vout_reg_threshold(int vout_th)
{
	int ret;
	u8 val;

	val = (u8)((vout_th - REG0B_VOUT_BASE)/REG0B_VOUT_LSB);
	val <<= REG0B_VOUT_SHIFT;

	ret = bq2587x_update_bits(g_bq, BQ2587X_REG_0B, REG0B_VOUT_REG, val);
	return ret;
}
EXPORT_SYMBOL_GPL(bq2587x_set_vout_reg_threshold);


int bq2587x_set_vdrop_ovp_threshold(int ovp_th)
{
	int ret;
	u8 val;

	val = (u8)((ovp_th - REG0C_VDROP_OVP_BASE)/REG0C_VDROP_OVP_LSB);
	val <<= REG0C_VDROP_OVP_SHIFT;

	ret = bq2587x_update_bits(g_bq, BQ2587X_REG_0C, REG0C_VDROP_OVP, val);
	return ret;
}
EXPORT_SYMBOL_GPL(bq2587x_set_vdrop_ovp_threshold);

int bq2587x_set_vdrop_alarm_threshold(int alm_th)
{
	int ret;
	u8 val;

	val = (u8)((alm_th - REG0D_VDROP_ALM_BASE)/REG0D_VDROP_ALM_LSB);
	val <<= REG0D_VDROP_ALM_SHIFT;

	ret = bq2587x_update_bits(g_bq, BQ2587X_REG_0D, REG0D_VDROP_ALM, val);
	return ret;
}
EXPORT_SYMBOL_GPL(bq2587x_set_vdrop_alarm_threshold);


int bq2587x_set_vbat_reg_threshold(int vbat_th)
{
	int ret;
	u8 val;

	//val = (u8)((vbat_th - REG11_VBAT_REG_BASE)/(REG11_VBAT_REG_LSB);
	val = (u8)((vbat_th - REG0E_VBAT_REG_BASE) * 10 / 125);
	val <<= REG0E_VBAT_REG_SHIFT;

	ret = bq2587x_update_bits(g_bq, BQ2587X_REG_0E, REG0E_VBAT_REG, val);
	return ret;
}
EXPORT_SYMBOL_GPL(bq2587x_set_vbat_reg_threshold);


int bq2587x_set_ibat_reg_threshold(int ibat_th)
{
	int ret;
	u8 val;

	val = (u8)((ibat_th - REG0F_IBAT_REG_BASE)/REG0F_IBAT_REG_LSB);
	val <<= REG0F_IBAT_REG_SHIFT;

	ret = bq2587x_update_bits(g_bq, BQ2587X_REG_0F, REG0F_IBAT_REG, val);
	return ret;
}
EXPORT_SYMBOL_GPL(bq2587x_set_ibat_reg_threshold);


int bq2587x_set_ibus_reg_threshold(int ibus_reg)
{
	int ret;
	u8 val;

	val = (u8)((ibus_reg - REG10_IBUS_REG_BASE)/REG10_IBUS_REG_LSB);
	val <<= REG10_IBUS_REG_SHIFT;

	ret = bq2587x_update_bits(g_bq, BQ2587X_REG_10, REG10_IBUS_REG, val);
	return ret;
}
EXPORT_SYMBOL_GPL(bq2587x_set_ibus_reg_threshold);


int bq2587x_set_ts_bus_threshold(int ts_th)
{
	int ret;
	u8 val;

	val = (u8)((ts_th - REG11_TS_BUS_FLT_BASE)/REG11_TS_BUS_FLT_LSB);
	val <<= REG11_TS_BUS_FLT_SHIFT;

	ret = bq2587x_update_bits(g_bq, BQ2587X_REG_11, REG11_TS_BUS_FLT, val);
	return ret;
}
EXPORT_SYMBOL_GPL(bq2587x_set_ts_bus_threshold);


int bq2587x_set_ts_bat_threshold(int ts_th)
{
	int ret;
	u8 val;

	val = (u8)((ts_th - REG12_TS_BAT_FLT_BASE)/REG12_TS_BAT_FLT_LSB);
	val <<= REG12_TS_BAT_FLT_SHIFT;

	ret = bq2587x_update_bits(g_bq, BQ2587X_REG_12, REG12_TS_BAT_FLT, val);
	return ret;
}
EXPORT_SYMBOL_GPL(bq2587x_set_ts_bat_threshold);

/*
 *	options are REG29_VUSBOVP_55V/65V/105V/140V *
 */
int bq2587x_set_vusb_ovp_threshold(int ovp_th)
{
	int ret;
	u8 val;

	val = ovp_th;
	val <<= REG29_VUSBOVP_SHIFT;

	ret = bq2587x_update_bits(g_bq, BQ2587X_REG_29, REG29_VUSBOVP_TH, val);
	return ret;

}
EXPORT_SYMBOL_GPL(bq2587x_set_vusb_ovp_threshold);

int bq2587x_set_ovpset(bool enable)
{
	int ret;
	u8 val;

	if (enable)
		val = REG29_OVPSET_ENABLE;
	else
		val = REG29_OVPSET_DISABLE;
	val <<= REG29_OVPSET_SHIFT;

	ret = bq2587x_update_bits(g_bq, BQ2587X_REG_29, REG29_OVPSET, val);
	return ret;

}
EXPORT_SYMBOL_GPL(bq2587x_set_ovpset);


/*
 *	options are REG29_VUSBOVP_55V/65V/105V/140V *
 */
int bq2587x_set_vusb_ovp_i2c_threshold(int ovp_th)
{
	int ret;
	u8 val;

	val = ovp_th;
	val <<= REG29_VUSBOVP_I2C_SHIFT;

	ret = bq2587x_update_bits(g_bq, BQ2587X_REG_29, REG29_VUSBOVP_I2C, val);
	return ret;

}
EXPORT_SYMBOL_GPL(bq2587x_set_vusb_ovp_i2c_threshold);


int bq2587x_set_R_place(bool high_side)
{
	int ret;
	u8 val;

	if (high_side)
		val = REG29_R_PLACE_HIGHSIDE;
	else
		val = REG29_R_PLACE_LOWSIDE;
	val <<= REG29_R_PLACE_SHIFT;

	ret = bq2587x_update_bits(g_bq, BQ2587X_REG_29, REG29_R_PLACE, val);
	return ret;

}
EXPORT_SYMBOL_GPL(bq2587x_set_R_place);

/*
 * read adc measurement result
 */
int bq2587x_read_adc_data(tADCChannel chan, int *result)
{
	int ret;
	u16 val = 0;
	u8 tdie;

	if (chan >= ADC_MAX_CHANNEL) 
		return -1;

	if (chan == ADC_DIE_TEMP){
		ret = bq2587x_read_byte(g_bq,&tdie, BQ2587X_REG_23);
		val = tdie;
	}
	else if (chan == ADC_VUSB){
		ret = bq2587x_read_word(g_bq, &val, BQ2587X_REG_27);
	}
	else{
		ret = bq2587x_read_word(g_bq, &val, ADC_REG_BASE + (chan << 1));
	}
	if (ret) return ret;
	*result = (int) val;
	return 0;
}
EXPORT_SYMBOL_GPL(bq2587x_read_adc_data);

void bq2587x_update_evt_status(void)
{
	int ret;
	u8 status;

	ret = bq2587x_read_evt_status(BQ2587X_REG_03,&status);
	if (ret) return ;

	g_bq->vbus_ovp 		= !!(status & REG03_VBUS_OVP_FLT_EVT);
	g_bq->ldo_active 	= !!(status & REG03_LDO_ACTIVE_EVT);
	g_bq->tbus_ot 		= !!(status & REG03_TBUS_FLT_EVT);
	g_bq->tbat_ot 		= !!(status & REG03_TBAT_FLT_EVT);
	g_bq->ibus_irev 	= !!(status & REG03_IBUS_IREV_EVT);

	ret = bq2587x_read_evt_status(BQ2587X_REG_04,&status);
	if (ret) return ;

	g_bq->vdrop_alm 	= !!(status & REG04_VDROP_ALM_EVT);
	g_bq->vdrop_ovp 	= !!(status & REG04_VDROP_OVP_FLT_EVT);
	g_bq->vbus_inserted = !!(status & REG04_VBUS_INSERT_EVT);
	g_bq->bat_inserted 	= !!(status & REG04_BAT_INSERT_EVT);
	g_bq->tshutdown 	= !!(status & REG04_TSHUT_FLT_EVT);
	g_bq->ibus_ocp		= !!(status & REG04_IBUS_OCP_FLT_EVT);


	ret = bq2587x_read_evt_status(BQ2587X_REG_26,&status);
	if (ret) return ;

	g_bq->vbat_ovp 		= !!(status & REG26_VBAT_OVP_FLT_EVT);
	g_bq->ibat_ocp 		= !!(status & REG26_IBAT_OCP_FLT_EVT);
	g_bq->vout_ovp  	= !!(status & REG26_VOUT_OVP_FLT_EVT);
	g_bq->scp			= !!(status & REG26_SCP_FLT_EVT);
}
EXPORT_SYMBOL_GPL(bq2587x_update_evt_status);

void bq2587x_update_adc_data(void)
{
	int ret;
	int val;

	ret = bq2587x_read_adc_data(ADC_VUSB, &val);
	if (ret) 
		dev_err(g_bq->dev,"Failed to read ADC_VUSB.");
	else 
		g_bq->vusb_volt = val;

	ret = bq2587x_read_adc_data(ADC_VBUS, &val);
	if (ret) 
		dev_err(g_bq->dev,"Failed to read ADC_VBUS.");
	else	
		g_bq->vbus_volt = val;

	ret = bq2587x_read_adc_data(ADC_IBUS, &val);
	if (ret)
		dev_err(g_bq->dev,"Failed to read ADC_IBUS.");
	else
		g_bq->ibus_current = val;

	ret = bq2587x_read_adc_data(ADC_VOUT, &val);
	if (ret)
		dev_err(g_bq->dev,"Failed to read ADC_VOUT.");
	else
		g_bq->vout_volt = val;

	ret = bq2587x_read_adc_data(ADC_VDROP, &val);
	if (ret)
		dev_err(g_bq->dev,"Failed to read ADC_VDROP.");
	else
		g_bq->vdrop_volt = val;

	ret = bq2587x_read_adc_data(ADC_VBAT, &val);
	if (ret)
		dev_err(g_bq->dev,"Failed to read ADC_VBAT.");
	else
		g_bq->vbat_volt = val;

	ret = bq2587x_read_adc_data(ADC_IBAT, &val);
	if (ret)
		dev_err(g_bq->dev,"Failed to read ADC_IBAT.");
	else
		g_bq->ibat_current = val;

	ret = bq2587x_read_adc_data(ADC_TBUS, &val);
	if (ret)
		dev_err(g_bq->dev,"Failed to read ADC_TBUS.");
	else
		g_bq->tbus_temp = val;

	ret = bq2587x_read_adc_data(ADC_TBAT, &val);
	if (ret)
		dev_err(g_bq->dev,"Failed to read ADC_TBAT.");
	else
		g_bq->tbat_temp = val;

	ret = bq2587x_read_adc_data(ADC_DIE_TEMP, &val);
	if (ret)
		dev_err(g_bq->dev,"Failed to read ADC_DIE_TEMP.");
	else
		g_bq->die_temp = val;

}
EXPORT_SYMBOL_GPL(bq2587x_update_adc_data);


static int bq2587x_init_device(struct bq2587x *bq)
{

	bq2587x_set_watchdog(REG06_WATCHDOG_DISABLE);
	bq2587x_disable_charge();

	bq2587x_adc_stop();
	bq2587x_adc_init(false, true, 8);
	bq2587x_set_sense_R(5);

	/* configure source to trigger interrupt */
	bq2587x_clear_evt_int_mask(BQ2587X_REG_01, REG01_VBUS_OVP_MASK
											| REG01_LDO_ACTIVE_MASK
											);

	bq2587x_clear_evt_int_mask(BQ2587X_REG_02, REG02_VBUS_INSERT_MASK
											| REG02_BAT_INSERT_MASK
											| REG02_VDROP_OVP_MASK
											| REG02_IBUS_OCP_MASK
											);
	bq2587x_clear_evt_int_mask(BQ2587X_REG_25, REG25_VBAT_OVP_MASK
											| REG25_VOUT_OVP_MASK
											| REG25_IBAT_OCP_MASK
											);

	bq2587x_evt_enable(BQ2587X_REG_05, REG05_VBUS_OVP_EN 
					| REG05_IBUS_REG_EN 
					| REG05_VBAT_REG_EN 
					| REG05_IBAT_REG_EN
					| REG05_VOUT_REG_EN
					);


	bq2587x_evt_enable(BQ2587X_REG_24, REG24_IBUS_OCP_EN
					| REG24_VBAT_OVP_EN
					| REG24_IBAT_OCP_EN
					| REG24_VOUT_OVP_EN
					);

	bq2587x_evt_enable(BQ2587X_REG_06, REG06_VDROP_OVP_EN | REG06_VDROP_ALM_EN);

	bq2587x_evt_disable(BQ2587X_REG_05,REG05_TS_BUS_FLT_EN
					| REG05_TS_BAT_FLT_EN
					);



	/*configure adc channel to be sampled */
	bq2587x_adc_set_scan_enable(BQ2587X_REG_08, REG08_VBUS_ADC_EN
											| REG08_IBUS_ADC_EN
											| REG08_VOUT_ADC_EN
											| REG08_VBAT_ADC_EN
											| REG08_VDROP_ADC_EN
											| REG08_IBAT_ADC_EN
											);

	bq2587x_adc_set_scan_enable(BQ2587X_REG_07, REG07_TDIE_ADC_EN);

	return 0;
}


static enum power_supply_property bq2587x_charger_props[] = {
	POWER_SUPPLY_PROP_ONLINE, /* External power source */
};


static int bq2587x_wall_get_property(struct power_supply *psy,
				enum power_supply_property psp,
				union power_supply_propval *val)
{

	struct bq2587x *bq = container_of(psy, struct bq2587x, wall);

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		if (bq->vbus_inserted && bq->bat_inserted)
			val->intval = 1;
		else
			val->intval = 0;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int bq2587x_psy_register(struct bq2587x *bq)
{
	int ret;

	bq->wall.name = "bq2587x-Wall";
	bq->wall.type = POWER_SUPPLY_TYPE_MAINS;
	bq->wall.properties = bq2587x_charger_props;
	bq->wall.num_properties = ARRAY_SIZE(bq2587x_charger_props);
	bq->wall.get_property = bq2587x_wall_get_property;
	bq->wall.external_power_changed = NULL;

	ret = power_supply_register(bq->dev, &bq->wall);
	if (ret < 0) {
		dev_err(bq->dev, "%s:failed to register wall psy:%d\n", __func__, ret);
		goto fail_1;
	}

	return 0;

fail_1:

	return ret;
}

static void bq2587x_psy_unregister(struct bq2587x *bq)
{
	power_supply_unregister(&bq->wall);
}

static ssize_t bq2587x_show_registers(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	u8 addr;
	u8 val;
	u8 tmpbuf[300];
	int len;
	int idx = 0;
	int ret ;

	idx = snprintf(buf, PAGE_SIZE, "%s:\n", "Flash Charger");
	for (addr = 0x0; addr <= 0x29; addr++) {
		ret = bq2587x_read_byte(g_bq, &val, addr);
		if (ret == 0) {
			len = snprintf(tmpbuf, PAGE_SIZE - idx,"Reg[0x%.2x] = 0x%.2x\n", addr, val);
			memcpy(&buf[idx], tmpbuf, len);
			idx += len;
		}
	}

	return idx;
}

static ssize_t bq2587x_store_register(struct device *dev,
				struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;
	unsigned int reg;
	unsigned int val;

	ret = sscanf(buf,"%x %x",&reg, &val);
	if (ret == 2 && (reg == 0x01 || reg == 0x02 || (reg >= 0x05 && reg <= 0x12) 
					|| reg == 0x24 || reg == 0x25 || reg == 0x29)) {
		bq2587x_write_byte(g_bq,(unsigned char)reg,(unsigned char)val);	
	}

	return count;
}

static DEVICE_ATTR(registers, S_IRUGO, bq2587x_show_registers, bq2587x_store_register);

static struct attribute *bq2587x_attributes[] = {
	&dev_attr_registers.attr,
	NULL,
};

static const struct attribute_group bq2587x_attr_group = {
	.attrs = bq2587x_attributes,
};


static int bq2587x_parse_dt(struct device *dev, struct bq2587x *bq)
{
	int ret;
	struct device_node *np = dev->of_node;


	ret = of_property_read_u32(np, "ti,bq2587x,vbus-ovp-threshold",&bq->cfg.vbus_ovp_threshold);
	if (ret)
		return ret;

	ret = of_property_read_u32(np, "ti,bq2587x,ibus-ocp-threshold",&bq->cfg.ibus_ocp_threshold);
	if (ret)
		return ret;

	ret = of_property_read_u32(np, "ti,bq2587x,tbus-ot-threshold",&bq->cfg.tbus_ot_threshold);
	if (ret)
		return ret;

	ret = of_property_read_u32(np, "ti,bq2587x,tbat-ot-threshold",&bq->cfg.tbat_ot_threshold);
	if (ret)
		return ret;

	ret = of_property_read_u32(np, "ti,bq2587x,ibus-irev-threshold",&bq->cfg.ibus_irev_threshold);
	if (ret)
		return ret;

	ret = of_property_read_u32(np, "ti,bq2587x,vdrop-alarm-threshold",&bq->cfg.vdrop_alarm_threshold);
	if (ret)
		return ret;

	ret = of_property_read_u32(np, "ti,bq2587x,vdrop-ovp-threshold",&bq->cfg.vdrop_ovp_threshold);
	if (ret)
		return ret;

	ret = of_property_read_u32(np, "ti,bq2587x,vbat-reg-threshold",&bq->cfg.vbat_reg_threshold);
	if (ret)
		return ret;

	ret = of_property_read_u32(np, "ti,bq2587x,ibat-reg-threshold",&bq->cfg.ibat_reg_threshold);
	if (ret)
		return ret;

	ret = of_property_read_u32(np, "ti,bq2587x,vout-reg-threshold",&bq->cfg.vout_reg_threshold);
	if (ret)
		return ret;

	ret = of_property_read_u32(np, "ti,bq2587x,ibus-reg-threshold",&bq->cfg.ibus_reg_threshold);
	if (ret)
		return ret;




	return 0;
}

static int bq2587x_detect_device(struct bq2587x *bq)
{
	int ret;
	u8 data;

	ret = bq2587x_read_byte(bq, &data, BQ2587X_REG_00);
	if (ret == 0) {
		bq->part_no = (data & REG00_DEV_ID) >> REG00_DEV_ID_SHIFT;
		bq->revision = (data & REG00_DEV_REV) >> REG00_DEV_REV_SHIFT;
	}

	return ret;
}


static void bq2587x_adapter_in_workfunc(struct work_struct *work)
{
	struct bq2587x *bq = container_of(work, struct bq2587x, adapter_in_work);
	int ret;

	bq2587x_set_vbus_ovp_threshold(bq->cfg.vbus_ovp_threshold);
	bq2587x_set_ibus_ocp_threshold(bq->cfg.ibus_ocp_threshold);

	bq2587x_set_ibus_reg_threshold(bq->cfg.ibus_reg_threshold);
	bq2587x_set_vout_reg_threshold(bq->cfg.vout_reg_threshold);
	bq2587x_set_vbat_reg_threshold(bq->cfg.vbat_reg_threshold);
	bq2587x_set_ibat_reg_threshold(bq->cfg.ibat_reg_threshold);

	bq2587x_set_vdrop_ovp_threshold(bq->cfg.vdrop_ovp_threshold);
	bq2587x_set_vdrop_alarm_threshold(bq->cfg.vdrop_alarm_threshold);

	bq2587x_set_ts_bus_threshold(bq->cfg.tbus_ot_threshold);
	bq2587x_set_ts_bat_threshold(bq->cfg.tbat_ot_threshold);

	bq2587x_adc_start();

	ret = bq2587x_enable_charge();
	if (!ret){
		dev_info(bq->dev,"%s:start charging",__func__);
		schedule_delayed_work(&bq->monitor_work, 0);
	} else {
		dev_err(bq->dev, "%s:failed to start charging,%d",__func__,ret);
	}
}

static void bq2587x_adapter_out_workfunc(struct work_struct *work)
{
	struct bq2587x *bq = container_of(work, struct bq2587x, adapter_out_work);

	bq->vbus_inserted = false;

	bq2587x_adc_stop();
	bq2587x_disable_charge();

	cancel_delayed_work_sync(&bq->monitor_work);
}


static void bq2587x_monitor_workfunc(struct work_struct *work)
{
	struct bq2587x *bq = container_of(work, struct bq2587x, monitor_work.work);

//	bq2587x_update_evt_status();
	bq2587x_update_adc_data();
	dev_info(bq->dev,"vbus_volt:%d,vbat_volt:%d,vusb_volt:%d,vout_volt:%d, vdrop_volt:%d, ibus_current:%d, ibat_current:%d, tbus_temp:%d, tbat_temp:%d, die_temp:%d",
					bq->vbus_volt, bq->vbat_volt, bq->vusb_volt, bq->vout_volt, bq->vdrop_volt, bq->ibus_current, bq->ibat_current, bq->tbus_temp, bq->tbat_temp, bq->die_temp);

	schedule_delayed_work(&bq->monitor_work, 5 * HZ);
}



static void bq2587x_charger_irq_workfunc(struct work_struct *work)
{
	struct bq2587x *bq = container_of(work, struct bq2587x, irq_work);
	u8 status = 0;
	int ret;

	/*
	 * check interrupt source and dispatch to handler
	 */

	ret = bq2587x_read_evt_status(BQ2587X_REG_03, &status);
	if (ret) return;

	if (!bq->vbus_ovp && (status & REG03_VBUS_OVP_FLT_EVT)) {
		bq->vbus_ovp = true;
		//TODO: vbus ovp handler
	}
	else
		bq->vbus_ovp = !!(status & REG03_VBUS_OVP_FLT_EVT);

	if (!bq->ldo_active && (status & REG03_LDO_ACTIVE_EVT)) {
		bq->ldo_active = true;
		//TODO: ldo active handler
	}
	else
		bq->ldo_active = !!(status & REG03_LDO_ACTIVE_EVT);

	if (!bq->tbus_ot && (status & REG03_TBUS_FLT_EVT)) {
		bq->tbus_ot = true;
		//TODO: bus over temperature handler
	}
	else
		bq->tbus_ot = !!(status & REG03_TBUS_FLT_EVT);

	if (!bq->tbat_ot && (status & REG03_TBAT_FLT_EVT)) {
		bq->tbat_ot = true;
		//TODO: battery over temperature handelr
	}
	else
		bq->tbat_ot = !!(status & REG03_TBAT_FLT_EVT);

	if (!bq->ibus_irev && (status & REG03_IBUS_IREV_EVT)) {
		bq->ibus_irev = true;
		//TODO: ibus current reverse handler
	}
	else
		bq->ibus_irev = !!(status & REG03_IBUS_IREV_EVT);

	ret = bq2587x_read_evt_status(BQ2587X_REG_04, &status);
	if (ret)
		return;

	if (!bq->vdrop_alm && (status & REG04_VDROP_ALM_EVT)) {
		bq->vdrop_alm = true;
		//TODO: vdrop alarm handler
	}
	else
		bq->vdrop_alm = !!(status & REG04_VDROP_ALM_EVT);

	if (!bq->vdrop_ovp && (status & REG04_VDROP_OVP_FLT_EVT)) {
		bq->vdrop_ovp = true;
		//TODO: vdrop ovp handler
	}
	else
		bq->vdrop_ovp = !!(status & REG04_VDROP_OVP_FLT_EVT);

	if (!bq->vbus_inserted && (status & REG04_VBUS_INSERT_EVT)) {
		bq->vbus_inserted = true;
		schedule_work(&bq->adapter_in_work);
	}
	else
		bq->vbus_inserted = !!(status & REG04_VBUS_INSERT_EVT);

	if (!bq->bat_inserted && (status & REG04_BAT_INSERT_EVT)) {
		//TODO: battery insert handelr
		bq->bat_inserted = true;
	}
	else
		bq->bat_inserted = !!(status & REG04_BAT_INSERT_EVT);

	if (!bq->tshutdown && (status & REG04_TSHUT_FLT_EVT)) {
		bq->tshutdown = true;
		//TODO:charger ic thermal shutdown handler
	}
	else
		bq->tshutdown = !!(status & REG04_TSHUT_FLT_EVT);

	if (!bq->ibus_ocp && (status & REG04_IBUS_OCP_FLT_EVT)) {
		bq->ibus_ocp = true;
		//TODO: ibus ocp handler
	}
	else
		bq->ibus_ocp = !!(status & REG04_IBUS_OCP_FLT_EVT);

	ret = bq2587x_read_evt_status(BQ2587X_REG_26, &status);
	if (ret)
		return;

	if (!bq->vbat_ovp && (status & REG26_VBAT_OVP_FLT_EVT)) {
		bq->vbat_ovp = true;
		//TODO: battery over voltage handler
	}
	else
		bq->vbat_ovp = !!(status & REG26_VBAT_OVP_FLT_EVT);

	if (!bq->ibat_ocp && (status & REG26_IBAT_OCP_FLT_EVT)) {
		bq->ibat_ocp = true;
		//TODO: charge current over current handler
	}
	else
		bq->ibat_ocp = !!(status & REG26_IBAT_OCP_FLT_EVT);

	if (!bq->vout_ovp && (status & REG26_VOUT_OVP_FLT_EVT)) {
		bq->vout_ovp = true;
		//TODO: vout ovp handler
	}
	else
		bq->vout_ovp = !!(status & REG26_VOUT_OVP_FLT_EVT);

	if (!bq->scp && (status & REG26_SCP_FLT_EVT)) {
		bq->scp = true;
		//TODO:short circuit fault handler
	}
	else
		bq->scp = !!(status & REG26_SCP_FLT_EVT); 

	msleep(5);

}


static irqreturn_t bq2587x_charger_interrupt(int irq, void *data)
{
	struct bq2587x *bq = data;

	schedule_work(&bq->irq_work);
	return IRQ_HANDLED;
}

#define GPIO_IRQ    80
static int bq2587x_charger_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	struct bq2587x *bq;
	int irqn;

	int ret;

	g_bq = NULL;

	bq = devm_kzalloc(&client->dev, sizeof(struct bq2587x), GFP_KERNEL);
	if (!bq) {
		dev_err(&client->dev, "%s: out of memory\n", __func__);
		return -ENOMEM;
	}

	bq->dev = &client->dev;
	bq->client = client;
	i2c_set_clientdata(client, bq);

	ret = bq2587x_detect_device(bq);
	if (!ret) {
		dev_info(bq->dev, "%s: charger device %s detected, revision:%d\n", __func__, part_name[bq->part_no],bq->revision);
	} else {
		dev_info(bq->dev, "%s: no bq2587x charger device found:%d\n", __func__, ret);
		return -ENODEV;
	}

	g_bq = bq;

	if (client->dev.of_node)
		bq2587x_parse_dt(&client->dev, bq);

	ret = bq2587x_init_device(bq);
	if (ret) {
		dev_err(bq->dev, "device init failure: %d\n", ret);
		goto err_0;
	}

	ret = gpio_request(GPIO_IRQ, "bq2587x irq pin");
	if (ret) {
		dev_err(bq->dev, "%s: %d gpio request failed\n", __func__, GPIO_IRQ);
		goto err_0;
	}
	gpio_direction_input(GPIO_IRQ);

	irqn = gpio_to_irq(GPIO_IRQ);
	if (irqn < 0) {
		dev_err(bq->dev, "%s:%d gpio_to_irq failed\n", __func__, irqn);
		ret = irqn;
		goto err_1;
	}
	client->irq = irqn;

	ret = bq2587x_psy_register(bq);
	if (ret)
		goto err_0;

	INIT_WORK(&bq->irq_work, bq2587x_charger_irq_workfunc);
	INIT_WORK(&bq->adapter_in_work, bq2587x_adapter_in_workfunc);
	INIT_WORK(&bq->adapter_out_work, bq2587x_adapter_out_workfunc);
	INIT_DELAYED_WORK(&bq->monitor_work, bq2587x_monitor_workfunc);


	ret = sysfs_create_group(&bq->dev->kobj, &bq2587x_attr_group);
	if (ret) {
		dev_err(bq->dev, "failed to register sysfs. err: %d\n", ret);
		goto err_irq;
	}

	ret = request_irq(client->irq, bq2587x_charger_interrupt, IRQF_TRIGGER_FALLING | IRQF_ONESHOT, "bq2587x_charger_irq", bq);
	if (ret) {
		dev_err(bq->dev, "%s:Request IRQ %d failed: %d\n", __func__, client->irq, ret);
		goto err_irq;
	} else {
		dev_info(bq->dev, "%s:irq = %d\n", __func__, client->irq);
	}

	return 0;

err_irq:
	cancel_work_sync(&bq->irq_work);
	cancel_work_sync(&bq->adapter_in_work);
	cancel_work_sync(&bq->adapter_out_work);
	cancel_delayed_work_sync(&bq->monitor_work);
err_1:
	gpio_free(GPIO_IRQ);
err_0:
	g_bq = NULL;
	return ret;
}

static void bq2587x_charger_shutdown(struct i2c_client *client)
{
	struct bq2587x *bq = i2c_get_clientdata(client);

	dev_info(bq->dev, "%s: shutdown\n", __func__);

	bq2587x_psy_unregister(bq);

	sysfs_remove_group(&bq->dev->kobj, &bq2587x_attr_group);
	cancel_work_sync(&bq->irq_work);
	cancel_work_sync(&bq->adapter_in_work);
	cancel_work_sync(&bq->adapter_out_work);
	cancel_delayed_work_sync(&bq->monitor_work);

	free_irq(bq->client->irq, NULL);
	gpio_free(GPIO_IRQ);
	g_bq = NULL;
}

static struct of_device_id bq2587x_charger_match_table[] = {
	{.compatible = "ti,bq2587x",},
	{},
};


static const struct i2c_device_id bq2587x_charger_id[] = {
	{ "bq25870", BQ25870 },
	{ "bq25871", BQ25871 },
	{ "bq25872", BQ25872 },
	{},
};

MODULE_DEVICE_TABLE(i2c, bq2587x_charger_id);

static struct i2c_driver bq2587x_charger_driver = {
	.driver		= {
		.name	= "bq2587x",
		.of_match_table = bq2587x_charger_match_table,
	},
	.id_table	= bq2587x_charger_id,

	.probe		= bq2587x_charger_probe,
	.shutdown   = bq2587x_charger_shutdown,
};

module_i2c_driver(bq2587x_charger_driver);

MODULE_DESCRIPTION("TI BQ2587x Charger Driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Texas Instruments");
