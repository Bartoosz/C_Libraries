/*
 * LPS331AP.h
 *
 *  Created on: Apr 3, 2023
 *      Author: barte
 */

#ifndef INC_LPS331AP_H_
#define INC_LPS331AP_H_


/* ------------- DEFINE LPS331AP REGISTER  ------------*/

/*------------------------------------------------*/
#define LPS331AP_REF_P_XL         (0x08)
/* The lower part of the reference pressure , def:0x0
   The full value is   REF_P_XL   +   REF_P_H   +   REF_P_L   and is represented as 2・s complement.*/
/*------------------------------------------------*/
#define LPS331AP_REF_P_L          (0x09)
/* This register contains the middle part of the reference pressure
   The full value is   REF_P_XL   +   REF_P_H   +   REF_P_L   and is represented as 2・s complement.*/
/*------------------------------------------------*/
#define LPS331AP_REF_P_H          (0x0A)
/* This register contains the higher part of the reference pressure
   The full value is   REF_P_XL   +   REF_P_H   +   REF_P_L   and is represented as 2・s complement.*/
/*------------------------------------------------*/
#define LPS331AP_RES_CONF         (0x10)
/* AVGP3-AVGP0 allow to select the pressure internal average. AVGT2-AVGT0 allow to
   select the temperature internal average.*/
/*------------------------------------------------*/
#define LPS331AP_WHO_AM_I         (0x0F)
/* This read-only register contains the device identifier that
   for LPS331AP, is set to 0xBB*/
/*------------------------------------------------*/
#define LPS331AP_CTRL_REG1        (0x20)
/* bit 7   [PD] - power down control. "0" -> power down "1" -> active
   bit 6-4 [ODR2, ODR1, ODR0] -bits allow to change the output data rates of pressure and temperature samples.
   	   	   	   Before changing the ODR it is necessary to power down the device!
   bit 3   [DIFF_EN] - bit is used to enable/disable the mechanism computing of differential pressure output
   	   	   	   It is suggested to turn on the circuitry only after the configuration of REF_P_x and THS_P_x
   bit 2   [BDU] - bit blocks writing to the bottom and top of the register to prevent reading from two different samples
   bit 1   [DELTA_EN] - 1: delta pressure registers enabled. 0: disable
   bit 0   [SIM] - bit is used to select SPI serial interface*/
/*------------------------------------------------*/
#define LPS331AP_CTRL_REG2        (0x21)
/* bit 7   [BOOT] def:0   Reboot memory content.   "0" -> normal mode ;  "1" -> reboot memory content.
   bit 6~3 [RESERVED]
   bit 2   [SWRESET] def: 0  , Software reset.   "0" -> normal mode ;   "1" -> software reset
			The device is reset to the power on configuration if the SWRESET bit is set to ．1・ and BOOT is set to ．1・.
   bit 1   [AUTO_ZERO] def: 0  , Autozero enable.   "0" -> normal mode;   "1" -> autozero enable
   bit 0   [ONE_SHOT] def: 0  , One shot enable.   "0" ->waiting for start of conversion ;   "1" ->start for a new dataset
			start a new conversion when ODR1-ODR0 bits in CTRL_REG1 are set to "000"
			when ONE_SHOT bit is set to ．1・. At the end of conversion the new data are available in the output registers,
			the REG_STAUS[0] and REG_STAUS[1] bits are set to ．1・ and the ONE_SHOT bit comes back to ．0・ by hardware.*/
/*------------------------------------------------*/
#define LPS331AP_CTRL_REG3        (0x22)
/* Interrupt configuration, for more information see spec!
   bit 7   [INT_H_L] def:0 Interrupt active high, low.    "0" ->  active high ;   "1" -> active low
   bit 6   [PP_OD] def:0  Push-pull/open drain selection on interrupt pads.  "0" -> push-pull ;   "1" -> open drain
   bit 5-3 [INT2_S3, INT2_S2, INT2_S1] def:0   data signal on INT2 pad control bits. see below table!
   bit 2-0 [INT1_S3, INT1_S2, INT1_S1] def:0   data signal on INT1 pad control bits. see below table!*/
/*------------------------------------------------*/
#define LPS331AP_INTERRUPT_CFG    (0x23)
/* bit 7-3  [RESERVED]
 * bit 2    [LIR] - Latch Interrupt request into INT_SOURCE register. Default value: 0.
			(0: interrupt request not latched; 1: interrupt request latched)
   bit 1    [PL_E] -Enable interrupt generation on differential pressure low event. Default value: 0.
			(0: disable interrupt request;
			1: enable interrupt request on measured differential pressure value lower than preset threshold)
   bit 0    [PH_E] - : Enable interrupt generation on differential pressure high event. Default value: 0
			(0: disable interrupt request;
			1:enable interrupt request on measured differential pressure value higher than preset
			threshold*/
/*------------------------------------------------*/
#define LPS331AP_INT_SOURCE       (0x24)
/* INT_SOURCE register is cleared by reading INT_ACK register.
  bit 7-3  [RESERVED]
  bit 2    [IA] - Interrupt Active
  bit 1    [PL] - Differential pressure Low
  bit 0    [PH] - Differential pressure High
  value "1" event has occured*/
/*------------------------------------------------*/
#define LPS331AP_THS_P_L          (0x25)
/* This register contains the low part of threshold value for pressure interrupt
   generation. The complete threshold value is given by THS_P_H & THS_P_L and is
   expressed as unsigned number*/
/*------------------------------------------------*/
#define LPS331AP_THS_P_H          (0x26)
/* This register contains the high part of the threshold value for pressure interrupt
   generation.
   The complete threshold value is given by THS_P_H & THS_P_L and is expressed as
   unsigned number. P_ths(mbar)=(THS_P_H & THS_P_L)[dec]/16.*/
/*------------------------------------------------*/
#define LPS331AP_STATUS_REG       (0x27)
/*The content of this register is updated every ODR cycle, regardless of BDU value in
CTRL_REG1.
 bit 7-6 [0]
 bit 5   [P_OR] - pressure data overrun "1" - new data for pressure has overwritten the previos one
 bit 4   [T_OR] - Temperature data overwritten  a new data for temperature has overwritten the previous one
 bit 3-2 [0]
 bit 1   [P_DA] - pressure data available. "1" new data for pressure is available
 bit 0   [T_DA] - temperature data available. "1" new data for temperature is available*/
/*------------------------------------------------*/
#define LPS331AP_PRESS_OUT_XL     (0x28)
/* Pressure data LSB */
/*------------------------------------------------*/
#define LPS331AP_PRESS_OUT_L      (0x29)
/* Pressure data */
/*------------------------------------------------*/
#define LPS331AP_PRESS_OUT_H      (0x2A)
/* Pressure data are expressed as PRESS_OUT_H & PRESS_OUT_L &
   PRESS_OUT_XL in 2’s complement. Values exceeding the operating pressure
   Range (see Table 3) are clipped.
   Pressure output data: Pout(mbar)=(PRESS_OUT_H & PRESS_OUT_L &
   PRESS_OUT_XL)[dec]/409*/
/*------------------------------------------------*/
#define LPS331AP_TEMP_OUT_L       (0x2B)
/* temperature data LSB */
/*------------------------------------------------*/
#define LPS331AP_TEMP_OUT_H       0x2C
/* temperature data MSB
   Temperature data are expressed as TEMP_OUT_H & TEMP_OUT_L as 2’s complement
   numbers.
   Temperature output data:
   T(degC) = 42.5 + (Temp_OUTH & TEMP_OUT_L)[dec]/480*/
/*------------------------------------------------*/
#define LPS331AP_AMP_CTRL         0x2D
/* bit 7-1  [RESERVED]
   bit 0    [SELMAIN] - SELMAIN: Current of operational amplifier selector
‘1’ always high current
‘0’ high current during pressure acquisition and low current during temperature acquisition*/
/*------------------------------------------------*/
#define LPS331AP_DELTA_PRESS_XL   0x3C
/* Delta pressure register for One Point calibration */
/*------------------------------------------------*/
#define LPS331AP_DELTA_PRESS_L    0X3D
/* Delta pressure register for One Point calibration*/
/*------------------------------------------------*/
#define LPS331AP_DELTA_PRESS_L   0x3E
/* Delta pressure register for One Point calibration
   DELTA_PRESS registers are used to store the one point calibration value to eliminate
   accuracy shift after soldering. DELTA_PRESS acts on the output pressure when
   CTRL_REG1[1] DELTA_EN is set to ‘1’.*/
/*------------------------------------------------*/

/* ------------- DEFINE LPS331AP Parameter -------------*/
#define LPS331AP_MODE_ON		 				(0x80)
#define LPS331AP_MODE_OFF		  				(0x00)
#define LPS331AP_I2C_TIMEOUT  					 1000
#define LPS331AP_OUTPUT_RATE_ONE_SHOT	 		(0x00)
#define LPS331AP_OUTPUT_RATE_1HZ_1HZ			(0x10)	/*pressure 1HZ,  temperature1HZ  */
#define LPS331AP_OUTPUT_RATE_7HZ_1HZ			(0x20)	/*pressure 7HZ,  temperature1HZ  */
#define LPS331AP_OUTPUT_RATE_12_5HZ_1HZ		    (0x30)	/*pressure 12.5HZ,  temperature1HZ  */
#define LPS331AP_OUTPUT_RATE_25HZ_1HZ			(0x40)	/*pressure 25HZ,  temperature1HZ  */
#define LPS331AP_OUTPUT_RATE_7HZ_7HZ			(0x50)	/*pressure 7HZ,  temperature7HZ  */
#define LPS331AP_OUTPUT_RATE_12_5HZ_12_5HZ	    (0x60)	/*pressure 12.5HZ,  temperature12.5HZ  */
#define LPS331AP_OUTPUT_RATE_25HZ_25HZ		    (0x70)  /*pressure 25HZ,  temperature25HZ  */


/* ------------- DEFINE LPS331AP Parameter -------------*/
/*SET CONFIG Parameter*/
/*
	set  REG_CTRL1f
*/
#define LPS331AP_SET_REGCTRL1	(LPS331AP_MODE_ON | LPS331AP_OUTPUT_RATE_25HZ_25HZ)
typedef struct
{
	I2C_HandleTypeDef  *lps_i2c;
	uint8_t             Address;

}LPS331AP_t;

uint8_t LPS331AP_Init(LPS331AP_t *lps, I2C_HandleTypeDef *i2c, uint8_t address);
void Write8(LPS331AP_t *lps, uint8_t Register, uint8_t Value);
void BMP280_SetREG_CTRL1(LPS331AP_t *lps, uint8_t Mode);
#endif /* INC_LPS331AP_H_ */
