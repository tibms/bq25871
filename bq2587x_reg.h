
#ifndef __BQ2587X_HEADER__
#define __BQ2587X_HEADER__

/* Register 00h, DeviceInfo */
#define BQ2587X_REG_00      		0x00
#define REG00_DEV_REV				0x38
#define	REG00_DEV_REV_SHIFT			3
#define REG00_DEV_ID				0x07
#define REG00_DEV_ID_SHIFT			0

/* Register 01h, Event 1 mask */
#define BQ2587X_REG_01		    	0x01
#define REG01_VBUS_OVP_MASK       	0x80
#define REG01_LDO_ACTIVE_MASK		0x78
#define REG01_TS_BUS_FLT_MASK		0x04
#define	REG01_TS_BAT_FLT_MASK		0x02
#define REG01_IBUS_IREV_MASK		0x01

/* Register 0x02, Event 2 mask */
#define BQ2587X_REG_02              0x02
#define REG02_ADC_DONE_MASK	    	0x40
#define REG02_VDROP_ALM_MASK		0x20
#define REG02_VDROP_OVP_MASK		0x10
#define REG02_VBUS_INSERT_MASK    	0x08
#define REG02_BAT_INSERT_MASK		0x04
#define	REG02_IBUS_OCP_MASK			0x01


/* Register 0x03, Event 1 */
#define BQ2587X_REG_03              0x03
#define REG03_VBUS_OVP_FLT_EVT	    0x80
#define REG03_LDO_ACTIVE_EVT		0x78
#define REG03_TBUS_FLT_EVT			0x04
#define REG03_TBAT_FLT_EVT     		0x02
#define REG03_IBUS_IREV_EVT			0x01

/* Register 0x04, Event 2 */
#define BQ2587X_REG_04              0x04
#define REG04_ADC_DONE_EVT       	0x40
#define REG04_VDROP_ALM_EVT    		0x20
#define REG04_VDROP_OVP_FLT_EVT		0x10
#define	REG04_VBUS_INSERT_EVT		0x08
#define REG04_BAT_INSERT_EVT		0x04
#define REG04_TSHUT_FLT_EVT			0x02
#define	REG04_IBUS_OCP_FLT_EVT		0x01

/* Register 0x05, Event 1 enable*/
#define BQ2587X_REG_05              0x05
#define REG05_VBUS_OVP_EN	        0x80
#define REG05_IBUS_REG_EN	        0x40
#define REG05_VBAT_REG_EN        	0x20
#define REG05_IBAT_REG_EN        	0x10
#define REG05_VOUT_REG_EN        	0x08
#define REG05_TS_BUS_FLT_EN        	0x04
#define REG05_TS_BAT_FLT_EN        	0x02
#define REG05_VBUS_PD_EN        	0x01
#define	REG05_VBUS_PD_EN_SHIFT		0
#define	REG05_VBUS_PD_ENABLE		1
#define	REG05_VBUS_PD_DISABLE		0

/* Register 0x06, Control*/
#define BQ2587X_REG_06              0x06
#define	REG06_VDROP_OVP_EN			0x80
#define REG06_VDROP_OVP_EN_SHIFT	7
#define	REG06_VDROP_OVP_ENABLE		1
#define	REG06_VDROP_OVP_DISABLE		0
#define	REG06_VDROP_ALM_EN			0x40
#define REG06_VDROP_ALM_EN_SHIFT	6
#define	REG06_VDROP_ALM_ENABLE		1
#define	REG06_VDROP_ALM_DISABLE		0
#define	REG06_SENSE_R				0x20
#define	REG06_SENSE_R_SHIFT			5
#define	REG06_SENSE_R_5MOH			0x00
#define	REG06_SENSE_R_10MOH			0x01
#define	REG06_CHG_EN				0x10
#define	REG06_CHG_EN_SHIFT			4
#define	REG06_CHG_EN_ENABLE			1
#define	REG06_CHG_EN_DISABLE		0
#define	REG06_WATCHDOG				0x0C
#define	REG06_WATCHDOG_SHIFT		2
#define	REG06_WATCHDOG_DISABLE		0
#define	REG06_WATCHDOG_1S			1
#define	REG06_WATCHDOG_1P5S			2
#define	REG06_WATCHDOG_5S			3
#define	REG06_RCP_SET				0x02
#define	REG06_RCP_SET_SHIFT			1
#define	REG06_RCP_0A				0
#define	REG06_RCP_3A				1
#define	REG06_REG_RST				0x01
#define	REG06_REG_RST_SHIFT			0

/* Register 0x07, ADC Control*/
#define BQ2587X_REG_07              0x07
#define	REG07_TDIE_ADC_EN			0x80
#define	REG07_TDIE_ADC_EN_SHIFT		7
#define	REG07_TDIE_ADC_ENABLE		1
#define	REG07_TDIE_ADC_DISABLE		0
#define	REG07_ADC_EN				0x08
#define	REG07_ADC_EN_SHIFT			3
#define	REG07_ADC_ENABLE			1
#define	REG07_ADC_DISABLE			0
#define	REG07_ADC_RATE				0x04
#define	REG07_ADC_RATE_SHIFT		2
#define	REG07_ADC_RATE_ONESHOT		0
#define	REG07_ADC_RATE_CONTINOUS	1
#define	REG07_ADC_AVG_EN			0x02
#define	REG07_ADC_AVG_SHIFT			1
#define	REG07_ADC_AVG_ENABLE		1
#define	REG07_ADC_AVG_DISABLE		0
#define	REG07_ADC_SAMPLES			0x01
#define	REG07_ADC_SAMPLES_SHIFT		0
#define	REG07_ADC_SAMPLES_8			0
#define	REG07_ADC_SAMPLES_16		1

/* Register 0x08, ADC mask 1*/
#define BQ2587X_REG_08              0x08
#define	REG08_VBUS_ADC_EN			0x80
#define	REG08_IBUS_ADC_EN			0x40
#define	REG08_VOUT_ADC_EN			0x20
#define	REG08_VDROP_ADC_EN			0x10
#define	REG08_VBAT_ADC_EN			0x08
#define	REG08_IBAT_ADC_EN			0x04
#define	REG08_TBUS_ADC_EN			0x02
#define	REG08_TBAT_ADC_EN			0x01


/* Register 0x09, Protection*/
#define BQ2587X_REG_09              0x09
#define	REG09_IBUS_OCP				0xF0
#define	REG09_IBUS_OCP_SHIFT		4
#define	REG09_IBUS_OCP_BASE			0
#define	REG09_IBUS_OCP_LSB			500

#define	REG09_OCP_RES				0x02
#define	REG09_OCP_RES_SHIFT			1
#define	REG09_OCP_RES_BLANK			0
#define	REG09_OCP_RES_HICCUP		1
#define	REG09_VBUS_OVP_DLY			0x01
#define	REG09_VBUS_OVP_DLY_SHIFT	0
#define	REG09_VBUS_OVP_DLY_8US		0
#define	REG09_VBUS_OVP_DLY_128US	1


/* Register 0x0A, VBUS ovp*/
#define BQ2587X_REG_0A              0x0A
#define	REG0A_VBUS_OVP				0x7F
#define	REG0A_VBUS_OVP_SHIFT		0
#define	REG0A_VBUS_OVP_BASE			4200
#define	REG0A_VBUS_OVP_LSB			30


/* Register 0x0B, Vout regulation*/
#define BQ2587X_REG_0B              0x0B
#define	REG0B_VOUT_REG				0x7E
#define	REG0B_VOUT_SHIFT			1
#define	REG0B_VOUT_BASE				4200 
#define	REG0B_VOUT_LSB				25

/* Register 0x0C, Vdrop ovp*/
#define BQ2587X_REG_0C              0x0C
#define	REG0C_VDROP_OVP				0xFE
#define	REG0C_VDROP_OVP_SHIFT		1
#define	REG0C_VDROP_OVP_BASE		0
#define	REG0C_VDROP_OVP_LSB			10

/* Register 0x0D, Vdrop alarm*/
#define BQ2587X_REG_0D              0x0D
#define	REG0D_VDROP_ALM				0xFE
#define	REG0D_VDROP_ALM_SHIFT		1
#define	REG0D_VDROP_ALM_BASE		0
#define	REG0D_VDROP_ALM_LSB			10

/* Register 0x0E, Vbat regulation*/
#define BQ2587X_REG_0E              0x0E
#define	REG0E_VBAT_REG				0x7F
#define	REG0E_VBAT_REG_SHIFT			0
#define	REG0E_VBAT_REG_BASE			4200
#define	REG0E_VBAT_REG_LSB			(125/10)

/* Register 0x0F, Ibat regulation*/
#define BQ2587X_REG_0F              0x0F
#define	REG0F_IBAT_REG				0x7F
#define	REG0F_IBAT_REG_SHIFT		0
#define	REG0F_IBAT_REG_BASE			0
#define	REG0F_IBAT_REG_LSB			50

/* Register 0x10, Ibus REG*/
#define BQ2587X_REG_10              0x10
#define	REG10_IBUS_REG				0x7E
#define	REG10_IBUS_REG_SHIFT		1
#define	REG10_IBUS_REG_BASE			0
#define	REG10_IBUS_REG_LSB			100

/* Register 0x11, Ts bus fault threshold*/
#define BQ2587X_REG_11              0x11
#define	REG11_TS_BUS_FLT			0x7F
#define	REG11_TS_BUS_FLT_SHIFT		0
#define	REG11_TS_BUS_FLT_BASE		0
#define	REG11_TS_BUS_FLT_LSB		25


/* Register 0x12, Ts bat fault threshold*/
#define	BQ2587X_REG_12				0x12
#define	REG12_TS_BAT_FLT			0x7F
#define	REG12_TS_BAT_FLT_SHIFT		0
#define	REG12_TS_BAT_FLT_BASE		0
#define	REG12_TS_BAT_FLT_LSB		25

/* Register 0x13, Vbus adc output, high byte */
#define	BQ2587X_REG_13				0x13
#define	REG13_VBUS_ADC				0x7F
#define	REG13_VBUS_ADC_SHIFT		0
#define	REG13_VBUS_ADC_BASE			0
#define	REG13_VBUS_ADC_LSB			256

/* Register 0x14, Vbus adc output, low byte */
#define	BQ2587X_REG_14				0x14
#define	REG14_VBUS_ADC				0xFF
#define	REG14_VBUS_ADC_SHIFT		0
#define	REG14_VBUS_ADC_BASE			0
#define	REG14_VBUS_ADC_LSB			1


/* Register 0x15, Ibus adc output, high byte */
#define	BQ2587X_REG_15				0x15
#define	REG15_IBUS_ADC				0x7F
#define	REG15_IBUS_ADC_SHIFT		0
#define	REG15_IBUS_ADC_BASE			0
#define	REG15_IBUS_ADC_LSB			256

/* Register 0x16, Ibus adc output, low byte */
#define	BQ2587X_REG_16				0x16
#define	REG16_IBUS_ADC				0xFF
#define	REG16_IBUS_ADC_SHIFT		0
#define	REG16_IBUS_ADC_BASE			0
#define	REG16_IBUS_ADC_LSB			1

/* Register 0x17, Vout adc output, high byte */
#define	BQ2587X_REG_17				0x17
#define	REG17_VOUT_ADC				0x7F
#define	REG17_VOUT_ADC_SHIFT		0
#define	REG17_VOUT_ADC_BASE			0
#define	REG17_VOUT_ADC_LSB			256

/* Register 0x18, Vout adc output, low byte */
#define	BQ2587X_REG_18				0x18
#define	REG18_VOUT_ADC				0xFF
#define	REG18_VOUT_ADC_SHIFT		0
#define	REG18_VOUT_ADC_BASE			0
#define	REG18_VOUT_ADC_LSB			1

/* Register 0x19, Vdrop adc output, high byte */
#define	BQ2587X_REG_19				0x19
#define	REG19_VDROP_ADC				0x7F
#define	REG19_VDROP_ADC_SHIFT		0
#define	REG19_VDROP_ADC_BASE		0
#define	REG19_VDROP_ADC_LSB			256

/* Register 0x1A, Vdrop adc output, low byte */
#define	BQ2587X_REG_1A				0x1A
#define	REG1A_VDROP_ADC				0xFF
#define	REG1A_VDROP_ADC_SHIFT		0
#define	REG1A_VDROP_ADC_BASE		0
#define	REG1A_VDROP_ADC_LSB			1

/* Register 0x1B, Vbat adc output, high byte */
#define	BQ2587X_REG_1B				0x1B
#define	REG1B_VBAT_ADC				0x7F
#define	REG1B_VBAT_ADC_SHIFT		0
#define	REG1B_VBAT_ADC_BASE			0
#define	REG1B_VBAT_ADC_LSB			256

/* Register 0x1C, Vbat adc output, low byte */
#define	BQ2587X_REG_1C				0x1C
#define	REG1C_VBAT_ADC				0xFF
#define	REG1C_VBAT_ADC_SHIFT		0
#define	REG1C_VBAT_ADC_BASE			0
#define	REG1C_VBAT_ADC_LSB			1

/* Register 0x1D, Ibat adc output, high byte */
#define	BQ2587X_REG_1D				0x1D
#define	REG1D_IBAT_ADC				0x7F
#define	REG1D_IBAT_ADC_SHIFT		0
#define	REG1D_IBAT_ADC_BASE			0
#define	REG1D_IBAT_ADC_LSB			256

/* Register 0x1E, Ibat adc output, low byte */
#define	BQ2587X_REG_1E				0x1E
#define	REG1E_IBAT_ADC				0xFF
#define	REG1E_IBAT_ADC_SHIFT		0
#define	REG1E_IBAT_ADC_BASE			0
#define	REG1E_IBAT_ADC_LSB			1

/* Register 0x1F, Tbus adc output, high byte */
#define	BQ2587X_REG_1F				0x1F
#define	REG1F_TBUS_ADC				0x7F
#define	REG1F_TBUS_ADC_SHIFT		0
#define	REG1F_TBUS_ADC_BASE			0
#define	REG1F_TBUS_ADC_LSB			256

/* Register 0x20, Tbus adc output, low byte */
#define	BQ2587X_REG_20				0x20
#define	REG20_TBUS_ADC				0xFF
#define	REG20_TBUS_ADC_SHIFT		0
#define	REG20_TBUS_ADC_BASE			0
#define	REG20_TBUS_ADC_LSB			1

/* Register 0x21, Tbat adc output, high byte */
#define	BQ2587X_REG_21				0x21
#define	REG21_TBAT_ADC				0x7F
#define	REG21_TBAT_ADC_SHIFT		0
#define	REG21_TBAT_ADC_BASE			0
#define	REG21_TBAT_ADC_LSB			256

/* Register 0x22, Tbat adc output, low byte */
#define	BQ2587X_REG_22				0x22
#define	REG22_TBAT_ADC				0xFF
#define	REG22_TBAT_ADC_SHIFT		0
#define	REG22_TBAT_ADC_BASE			0
#define	REG22_TBAT_ADC_LSB			1

/* Register 0x23, Die temp adc output */
#define	BQ2587X_REG_23				0x23
#define	REG23_DIE_TEMP_ADC			0xFF
#define	REG23_DIE_TEMP_ADC_SHIFT	0
#define	REG23_DIE_TEMP_ADC_BASE		0
#define	REG23_DIE_TEMP_ADC_LSB		1

/* Register 0x24, Protection enable*/
#define BQ2587X_REG_24              0x24
#define	REG24_VDROP_AMP				0x80
#define	REG24_VDROP_AMP_SHIFT		7
#define	REG24_VDROP_AMP_ENABLE		0
#define	REG24_VDROP_AMP_DISABLE		1

#define REG24_IBUS_OCP_EN	        0x80
#define REG24_VBAT_OVP_EN        	0x40
#define REG24_IBAT_OCP_EN        	0x20
#define REG24_VOUT_OVP_EN        	0x10

/* Register 0x25, Event 3 mask */
#define BQ2587X_REG_25              0x25
#define REG25_VBAT_OVP_MASK     	0x04
#define REG25_IBAT_OCP_MASK     	0x02
#define REG25_VOUT_OVP_MASK      	0x01

/* Register 0x26, Event 3*/
#define BQ2587X_REG_26              0x26
#define	REG26_SCP_FLT_EVT			0x80
#define	REG26_VUSB_OVP_FLT_EVT		0x08	/*bq25872 only*/
#define REG26_VBAT_OVP_FLT_EVT 		0x04
#define REG26_IBAT_OCP_FLT_EVT		0x02
#define	REG26_VOUT_OVP_FLT_EVT		0x01

/* Register 0x27, Vusb adc output, high byte, bq25872 only*/ 
#define	BQ2587X_REG_27				0x27
#define	REG27_VUSB_ADC				0x7F
#define	REG27_VUSB_ADC_SHIFT		0
#define	REG27_VUSB_ADC_BASE			0
#define	REG27_VUSB_ADC_LSB			256

/* Register 0x28, Vbus adc output, low byte, bq25872 only*/
#define	BQ2587X_REG_28				0x28
#define	REG28_VUSB_ADC				0xFF
#define	REG28_VUSB_ADC_SHIFT		0
#define	REG28_VUSB_ADC_BASE			0
#define	REG28_VUSB_ADC_LSB			1

/* Register 0x29, Control 2, bit2-7 for bq25872 only*/
#define BQ2587X_REG_29      	    0x29
#define	REG29_VUSBOVP_TH			0x60
#define	REG29_VUSBOVP_SHIFT			5
#define	REG29_VUSBOVP_55V			0   /*5.5v*/
#define	REG29_VUSBOVP_65V			1	/*6.5V*/
#define	REG29_VUSBOVP_105V			2	/*10.5v*/
#define	REG29_VUSBOVP_140V			3	/*14.0v*/
#define	REG29_OVPSET				0x10
#define	REG29_OVPSET_SHIFT			4
#define	REG29_OVPSET_ENABLE			0
#define	REG29_OVPSET_DISABLE		1
#define	REG29_VUSBOVP_I2C			0xC0
#define	REG29_VUSBOVP_I2C_SHIFT		2

#define	REG29_R_PLACE				0x01
#define	REG29_R_PLACE_SHIFT			0
#define	REG29_R_PLACE_LOWSIDE		0
#define	REG29_R_PLACE_HIGHSIDE		1


/* Register 0x40, Die temp threshold*/
#define	BQ2587X_REG_40				0x40
#define	REG40_DIE_TEMP_FLT_TH		0x03
#define	REG40_DIE_TEMP_FLT_SHIFT	0
#define	REG40_DIE_TEMP_FLT_BASE		105
#define	REG40_DIE_TEMP_FLT_LSB		15



#endif


