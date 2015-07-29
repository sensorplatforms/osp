/*
 ****************************************************************************
 *
 * (C) All rights reserved by ROBERT BOSCH GMBH
 *
 ****************************************************************************/
/*	Date: 2013/08/07
 *	Revision: 1.1
 *
 */

/*****************************************************************************
 * Copyright (C) 2013 Bosch Sensortec GmbH
 *
 * bmm050.c
 *
 * Usage:        Sensor Driver for BMM050
 *
 ******************************************************************************/
/*****************************************************************************/
/*  Disclaimer
 *
 * Common:
 * Bosch Sensortec products are developed for the consumer goods industry. They
 * may only be used within the parameters of the respective valid product data
 * sheet.  Bosch Sensortec products are provided with the express understanding
 * that there is no warranty of fitness for a particular purpose.They are not
 * fit for use in life-sustaining, safety or security sensitive systems or any
 * system or device that may lead to bodily harm or property damage if the
 * system or device malfunctions. In addition, Bosch Sensortec products are not
 * fit for use in products which interact with motor vehicle systems.The resale
 * and or use of products are at the purchasers own risk and his own
 * responsibility. The examination of fitness for the intended use is the sole
 * responsibility of the Purchaser.
 *
 * The purchaser shall indemnify Bosch Sensortec from all third party claims,
 * including any claims for incidental, or consequential damages, arising from
 * any product use not covered by the parameters of the respective valid product
 * data sheet or not approved by Bosch Sensortec and reimburse Bosch
 * Sensortec for all costs in connection with such claims.
 *
 * The purchaser must monitor the market for the purchased products,
 * particularly with regard to product safety and inform Bosch Sensortec without
 * delay of all security relevant incidents.
 *
 * Engineering Samples are marked with an asterisk (*) or (e). Samples may vary
 * from the valid technical specifications of the product series. They are
 * therefore not intended or fit for resale to third parties or for use in end
 * products. Their sole purpose is internal client testing. The testing of an
 * engineering sample may in no way replace the testing of a product series.
 * Bosch Sensortec assumes no liability for the use of engineering samples. By
 * accepting the engineering samples, the Purchaser agrees to indemnify Bosch
 * Sensortec from all claims arising from the use of engineering samples.
 *
 * Special:
 * This software module (hereinafter called "Software") and any information on
 * application-sheets (hereinafter called "Information") is provided free of
 * charge for the sole purpose to support your application work. The Software
 * and Information is subject to the following terms and conditions:
 *
 * The Software is specifically designed for the exclusive use for Bosch
 * Sensortec products by personnel who have special experience and training.
 * Do not use this Software if you do not have the proper experience or
 * training.
 *
 * This Software package is provided `` as is `` and without any expressed or
 * implied warranties, including without limitation, the implied warranties of
 * merchantability and fitness for a particular purpose.
 *
 * Bosch Sensortec and their representatives and agents deny any liability for
 * the functional impairment of this Software in terms of fitness, performance
 * and safety. Bosch Sensortec and their representatives and agents shall not be
 * liable for any direct or indirect damages or injury, except as otherwise
 * stipulated in mandatory applicable law.
 *
 * The Information provided is believed to be accurate and reliable. Bosch
 * Sensortec assumes no responsibility for the consequences of use of such
 * Information nor for any infringement of patents or other rights of third
 * parties which may result from its use. No license is granted by implication
 * or otherwise under any patent or patent rights of Bosch. Specifications
 * mentioned in the Information are subject to change without notice.
 *
 * It is not allowed to deliver the source code of the Software to any third
 * party without permission of Bosch Sensortec.
 */
/****************************************************************************/
/*! \file <BMM050 >
    \brief <Sensor driver for BMM050> */

#include "bmm050.h"
#include "sensorhub.h"
#include "common.h"
#include "mag_common.h"
#include "osp-sensors.h"
#include "sensacq_i2c.h"
#include "gpio_api.h"
#include "gpio_irq_api.h"

static struct bmm050 *p_bmm050;
static struct bmm050 bmm050;
#if 0
static void mag_activate(bool enable)
{
	if (enable) {
		bmm050_set_functional_state(BMM050_NORMAL_MODE);
		/* Update the last set data rate */
		bmm050_set_datarate(BMM050_DR_25HZ);
		/* Update repititions when activated */
		bmm050_set_repetitions_XY(BMM050_REGULAR_REPXY);
		bmm050_set_repetitions_Z(BMM050_REGULAR_REPZ);
		/* Enable Data ready interrupt */
		bmm050_set_mag_drdy_interrupt(enable, 1);
		/* Read to clear any pending interrupt */
		Mag_ReadData(NULL);

		/* Enable interrupt in the NVIC */
		NVIC_EnableIRQ(MAG_PINT_IRQn);
		NVIC_ClearPendingIRQ(MAG_PINT_IRQn);
	}
	else {
		bmm050_set_mag_drdy_interrupt(enable, 1);
		bmm050_set_functional_state(BMM050_SLEEP_MODE);
		NVIC_DisableIRQ(MAG_PINT_IRQn);
	}
}
#endif

void Mag_HardwareSetup(osp_bool_t enable)
{
    gpio_t hostifIrq;
	NVIC_SetPriority(MAG_PINT_IRQn, SENSOR_IRQ_PRIORITY);
	NVIC_DisableIRQ(MAG_PINT_IRQn);

	/* MAG INT2 irq setup */
	//Chip_GPIO_SetPinDIRInput(LPC_GPIO, MAG_INT_PORT, MAG_INT_PIN);
    hostifIrq.pin = ENCODE_PORT_PIN(MAG_INT_PORT, MAG_INT_PIN);
    gpio_dir(&hostifIrq,PIN_INPUT);
    
	Chip_INMUX_PinIntSel(MAG_PINT_SEL, MAG_INT_PORT, MAG_INT_PIN);	/* Configure INMUX block */
//	Chip_PININT_SetPinModeEdge(LPC_PININT, MAG_PINT_CH);/* edge sensitive and rising edge interrupt */
//	Chip_PININT_EnableIntHigh(LPC_PININT, MAG_PINT_CH);
    {
        gpio_irq_t gpioIrq;
        gpioIrq.irq_index = MAG_PINT_CH;
        gpioIrq.event = IRQ_EDGE_RISE;
        gpio_irq_enable(&gpioIrq);
    }
	Chip_SYSCON_EnableWakeup(MAG_WAKE);	/* enable to wake from sleep */
	Chip_SYSCON_EnableWakeup(SYSCON_STARTER_WWDT);	/* enable to wake from sleep */

	//Chip_GPIO_SetPinDIRInput(LPC_GPIO, MAG_INT3_PORT, MAG_INT3_PIN);
    hostifIrq.pin = ENCODE_PORT_PIN(MAG_INT3_PORT, MAG_INT3_PIN);
    gpio_dir(&hostifIrq,PIN_INPUT);

}


void Mag_Initialize(void)
{
	bmm050.bus_write 	= dev_i2c_write;
	bmm050.bus_read		= dev_i2c_read;
	bmm050.delay_msec	= dev_i2c_delay;

	D0_printf("MAG INIT  0\r\n");
	bmm050_init(&bmm050);
	/* reset the mag */
	D0_printf("MAG INIT  1\r\n");
	dev_i2c_delay(10);
	/* mag_setDelay(pSens, 50); */
	bmm050_set_datarate(BMM050_DR_25HZ);
	dev_i2c_delay(10);
	bmm050_set_mag_drdy_interrupt(true, 1);
	bmm050_set_functional_state(BMM050_SLEEP_MODE);

    Mag_ReadData(NULL);
 

    /* Note: The mag sensor is configured to SLEEP mode so in order to get sensor data you must set the OpMode to either normal (0x00) or forced mode (0x01) */

//	mag_activate(true);
}

void Mag_SetLowPowerMode(void)
{
	bmm050_set_functional_state(BMM050_SLEEP_MODE);
}

void Mag_ConfigDataInt(osp_bool_t enable)
{
	/* Need to implement: */
	if (enable) {
		bmm050_set_functional_state(BMM050_NORMAL_MODE);
		/* Update the last set data rate */
		bmm050_set_datarate(BMM050_DR_25HZ);
		/* Update repititions when activated */
		bmm050_set_repetitions_XY(BMM050_REGULAR_REPXY);
		bmm050_set_repetitions_Z(BMM050_REGULAR_REPZ);
		/* Enable Data ready interrupt */
		bmm050_set_mag_drdy_interrupt(enable, 1);
		
		/* Enable interrupt in the NVIC */
		NVIC_EnableIRQ(MAG_PINT_IRQn);
		dev_i2c_delay(20);
	}
	else {
		bmm050_set_mag_drdy_interrupt(enable, 1);
		bmm050_set_functional_state(BMM050_SLEEP_MODE);
		NVIC_DisableIRQ(MAG_PINT_IRQn);
	}
}

void Mag_ClearDataInt()
{
	/* Read to clear any pending interrupt */
	Mag_ReadData(NULL);
}

void Mag_TriggerDataAcq(void)
{
	Mag_ReadData(NULL);
	bmm050_set_mag_drdy_interrupt(true, 1);
	bmm050_set_functional_state(BMM050_FORCED_MODE);
}

void Mag_ReadData(MsgMagData *magData)
{
	struct bmm050_mdata mdata;

	bmm050_get_raw_xyz(&mdata);
	if (magData) {
		magData->X = mdata.datax;
		magData->Y = mdata.datay;
		magData->Z = mdata.dataz;
	}
}
void MAG_IRQHandler(void)
{
    gpio_irq_t gpioIrq;
	uint32_t currTime = GetCurrentTime();
    gpioIrq.irq_index = MAG_PINT_CH;
#if 0
	uint32_t currTime = g_Timer.GetCurrent();
	PhysicalSensor_t* pSens = g_phySensors[PHYS_MAG_ID];
	pSens->ts_nextSample = currTime + ((pSens->period + (pSens->ts_nextSample - pSens->ts_lastSample)) >> 1) ;
	pSens->ts_lastSample = currTime;
	
	pSens->irq_pending++;
//	Chip_PININT_ClearIntStatus(LPC_PININT, MAG_PINT_CH);
    gpio_irq_disable(&gpioIrq);
	ResMgr_IRQDone();
#else
//	Chip_PININT_ClearIntStatus(LPC_PININT, MAG_PINT_CH);
    gpio_irq_disable(&gpioIrq);
	SendDataReadyIndication(MAG_INPUT_SENSOR, currTime);
#endif
}


BMM050_RETURN_FUNCTION_TYPE bmm050_init(struct bmm050 *bmm050)
{
	BMM050_RETURN_FUNCTION_TYPE comres = 0;
	unsigned char a_data_u8r[2];
	p_bmm050 = bmm050;

	p_bmm050->dev_addr = BMM050_I2C_ADDRESS;

	/* set device from suspend into sleep mode */
	bmm050_set_powermode(BMM050_ON);
    
    /* Perform a software to reset all the registers except the trim registers. 
     * Soft reset only executes when device is in sleep mode. No execution in suspended mode 
     */
    bmm050_soft_reset();

	/*Read CHIP_ID and REv. info */
	comres = p_bmm050->BMM050_BUS_READ_FUNC(p_bmm050->dev_addr,
											BMM050_CHIP_ID, a_data_u8r, 1);
	p_bmm050->company_id = a_data_u8r[0];
	//D0_printf("Returned: 0x%x\r\n", (uint32_t)(a_data_u8r[0]));
	/* Function to initialise trim values */
	bmm050_init_trim_registers();
	bmm050_set_presetmode(BMM050_PRESETMODE_REGULAR);
	return comres;
}

BMM050_RETURN_FUNCTION_TYPE bmm050_set_presetmode(unsigned char mode)
{
	BMM050_RETURN_FUNCTION_TYPE comres = 0;
	switch (mode) {
	case BMM050_PRESETMODE_LOWPOWER:
		/* Set the data rate for Low Power mode */
		comres = bmm050_set_datarate(BMM050_LOWPOWER_DR);
		/* Set the XY-repetitions number for Low Power mode */
		comres |= bmm050_set_repetitions_XY(BMM050_LOWPOWER_REPXY);
		/* Set the Z-repetitions number  for Low Power mode */
		comres |= bmm050_set_repetitions_Z(BMM050_LOWPOWER_REPZ);
		break;

	case BMM050_PRESETMODE_REGULAR:
		/* Set the data rate for Regular mode */
		comres = bmm050_set_datarate(BMM050_REGULAR_DR);
		/* Set the XY-repetitions number for Regular mode */
		comres |= bmm050_set_repetitions_XY(BMM050_REGULAR_REPXY);
		/* Set the Z-repetitions number  for Regular mode */
		comres |= bmm050_set_repetitions_Z(BMM050_REGULAR_REPZ);
		break;

	case BMM050_PRESETMODE_HIGHACCURACY:
		/* Set the data rate for High Accuracy mode */
		comres = bmm050_set_datarate(BMM050_HIGHACCURACY_DR);
		/* Set the XY-repetitions number for High Accuracy mode */
		comres |= bmm050_set_repetitions_XY(BMM050_HIGHACCURACY_REPXY);
		/* Set the Z-repetitions number  for High Accuracy mode */
		comres |= bmm050_set_repetitions_Z(BMM050_HIGHACCURACY_REPZ);
		break;

	case BMM050_PRESETMODE_ENHANCED:
		/* Set the data rate for Enhanced Accuracy mode */
		comres = bmm050_set_datarate(BMM050_ENHANCED_DR);
		/* Set the XY-repetitions number for High Enhanced mode */
		comres |= bmm050_set_repetitions_XY(BMM050_ENHANCED_REPXY);
		/* Set the Z-repetitions number  for High Enhanced mode */
		comres |= bmm050_set_repetitions_Z(BMM050_ENHANCED_REPZ);
		break;

	default:
		comres = E_BMM050_OUT_OF_RANGE;
		break;
	}
	return comres;
}

BMM050_RETURN_FUNCTION_TYPE bmm050_set_functional_state	\
	(unsigned char functional_state)
{
	BMM050_RETURN_FUNCTION_TYPE comres = 0;
	unsigned char v_data1_u8r;
	if (p_bmm050 == BMM050_NULL) {
		return E_BMM050_NULL_PTR;
	}
	else {
		switch (functional_state) {
		case BMM050_NORMAL_MODE:
			comres = bmm050_get_powermode(&v_data1_u8r);
			if (v_data1_u8r == BMM050_OFF) {
				comres |= bmm050_set_powermode(BMM050_ON);
				p_bmm050->delay_msec(
					BMM050_DELAY_SUSPEND_SLEEP);
			}
			{
				comres |= p_bmm050->BMM050_BUS_READ_FUNC(
					p_bmm050->dev_addr,
					BMM050_CNTL_OPMODE__REG,
					&v_data1_u8r, 1);
				v_data1_u8r = BMM050_SET_BITSLICE(
					v_data1_u8r,
					BMM050_CNTL_OPMODE,
					BMM050_NORMAL_MODE);
				comres |= p_bmm050->BMM050_BUS_WRITE_FUNC(
					p_bmm050->dev_addr,
					BMM050_CNTL_OPMODE__REG,
					&v_data1_u8r, 1);
			}
			break;

		case BMM050_SUSPEND_MODE:
			comres = bmm050_set_powermode(BMM050_OFF);
			break;

		case BMM050_FORCED_MODE:
			comres = bmm050_get_powermode(&v_data1_u8r);
			if (v_data1_u8r == BMM050_OFF) {
				comres = bmm050_set_powermode(BMM050_ON);
				p_bmm050->delay_msec(
					BMM050_DELAY_SUSPEND_SLEEP);
			}
			comres |= p_bmm050->BMM050_BUS_READ_FUNC(
				p_bmm050->dev_addr,
				BMM050_CNTL_OPMODE__REG,
				&v_data1_u8r, 1);
			v_data1_u8r = BMM050_SET_BITSLICE(
				v_data1_u8r,
				BMM050_CNTL_OPMODE, BMM050_ON);
			comres |= p_bmm050->BMM050_BUS_WRITE_FUNC(
				p_bmm050->dev_addr,
				BMM050_CNTL_OPMODE__REG,
				&v_data1_u8r, 1);
			break;

		case BMM050_SLEEP_MODE:
			bmm050_get_powermode(&v_data1_u8r);
			if (v_data1_u8r == BMM050_OFF) {
				comres = bmm050_set_powermode(BMM050_ON);
				p_bmm050->delay_msec(
					BMM050_DELAY_SUSPEND_SLEEP);
			}
			comres |= p_bmm050->BMM050_BUS_READ_FUNC(
				p_bmm050->dev_addr,
				BMM050_CNTL_OPMODE__REG,
				&v_data1_u8r, 1);
			v_data1_u8r = BMM050_SET_BITSLICE(
				v_data1_u8r,
				BMM050_CNTL_OPMODE,
				BMM050_SLEEP_MODE);
			comres |= p_bmm050->BMM050_BUS_WRITE_FUNC(
				p_bmm050->dev_addr,
				BMM050_CNTL_OPMODE__REG,
				&v_data1_u8r, 1);
			break;

		default:
			comres = E_BMM050_OUT_OF_RANGE;
			break;
		}
	}
	return comres;
}

BMM050_RETURN_FUNCTION_TYPE bmm050_get_functional_state	\
	(unsigned char *functional_state)
{
	BMM050_RETURN_FUNCTION_TYPE comres = 0;
	unsigned char v_data_u8r;
	if (p_bmm050 == BMM050_NULL) {
		return E_BMM050_NULL_PTR;
	}
	else {
		comres = p_bmm050->BMM050_BUS_READ_FUNC(
			p_bmm050->dev_addr,
			BMM050_CNTL_OPMODE__REG,
			&v_data_u8r, 1);
		*functional_state = BMM050_GET_BITSLICE(
			v_data_u8r, BMM050_CNTL_OPMODE);
	}
	return comres;
}

BMM050_RETURN_FUNCTION_TYPE bmm050_read_mdataXYZ(struct bmm050_mdata *mdata)
{
	BMM050_RETURN_FUNCTION_TYPE comres;

	unsigned char a_data_u8r[8];

	struct {
		BMM050_S16 raw_dataX;
		BMM050_S16 raw_dataY;
		BMM050_S16 raw_dataZ;
		BMM050_U16 raw_dataR;
	} raw_dataXYZ;

	if (p_bmm050 == BMM050_NULL) {
		return E_BMM050_NULL_PTR;
	}
	else {
		comres = p_bmm050->BMM050_BUS_READ_FUNC(p_bmm050->dev_addr,
												BMM050_DATAX_LSB, a_data_u8r, 8);

		/* Reading data for X axis */
		a_data_u8r[0] = BMM050_GET_BITSLICE(a_data_u8r[0],
											BMM050_DATAX_LSB_VALUEX);
		raw_dataXYZ.raw_dataX = (BMM050_S16) ((((BMM050_S16)
												((signed char) a_data_u8r[1])) <<
											   SHIFT_LEFT_5_POSITION) | a_data_u8r[0]);

		/* Reading data for Y axis */
		a_data_u8r[2] = BMM050_GET_BITSLICE(a_data_u8r[2],
											BMM050_DATAY_LSB_VALUEY);
		raw_dataXYZ.raw_dataY = (BMM050_S16) ((((BMM050_S16)
												((signed char) a_data_u8r[3])) <<
											   SHIFT_LEFT_5_POSITION) | a_data_u8r[2]);

		/* Reading data for Z axis */
		a_data_u8r[4] = BMM050_GET_BITSLICE(a_data_u8r[4],
											BMM050_DATAZ_LSB_VALUEZ);
		raw_dataXYZ.raw_dataZ = (BMM050_S16) ((((BMM050_S16)
												((signed char) a_data_u8r[5])) <<
											   SHIFT_LEFT_7_POSITION) | a_data_u8r[4]);

		/* Reading data for Resistance*/
		a_data_u8r[6] = BMM050_GET_BITSLICE(a_data_u8r[6],
											BMM050_R_LSB_VALUE);
		raw_dataXYZ.raw_dataR = (BMM050_U16) ((((BMM050_U16)
												a_data_u8r[7]) <<
											   SHIFT_LEFT_6_POSITION) | a_data_u8r[6]);

		/* Compensation for X axis */
		mdata->datax = bmm050_compensate_X(raw_dataXYZ.raw_dataX,
										   raw_dataXYZ.raw_dataR);

		/* Compensation for Y axis */
		mdata->datay = bmm050_compensate_Y(raw_dataXYZ.raw_dataY,
										   raw_dataXYZ.raw_dataR);

		/* Compensation for Z axis */
		mdata->dataz = bmm050_compensate_Z(raw_dataXYZ.raw_dataZ,
										   raw_dataXYZ.raw_dataR);

		/* Output raw resistance value */
		mdata->resistance = raw_dataXYZ.raw_dataR;
	}
	return comres;
}

/* In this function X and Y axis is remapped,
 * this API is only applicable for BMX055*/
BMM050_RETURN_FUNCTION_TYPE bmm050_read_bmx055_remapped_mdataXYZ \
	(struct bmm050_remapped_mdata *mdata)
{
	BMM050_RETURN_FUNCTION_TYPE comres;

	unsigned char a_data_u8r[8];

	struct {
		BMM050_S16 raw_dataX;
		BMM050_S16 raw_dataY;
		BMM050_S16 raw_dataZ;
		BMM050_U16 raw_dataR;
	} raw_dataXYZ;

	if (p_bmm050 == BMM050_NULL) {
		return E_BMM050_NULL_PTR;
	}
	else {
		comres = p_bmm050->BMM050_BUS_READ_FUNC(p_bmm050->dev_addr,	\
												BMM050_BMX055_REMAPPED_DATAY_LSB, a_data_u8r, 8);

		/* Reading data for Y axis */
		a_data_u8r[0] = BMM050_GET_BITSLICE(a_data_u8r[0],
											BMM050_BMX055_REMAPPED_DATAY_LSB_VALUEY);
		raw_dataXYZ.raw_dataY = (BMM050_S16) ((((BMM050_S16)
												((signed char) a_data_u8r[1])) <<
											   SHIFT_LEFT_5_POSITION) | a_data_u8r[0]);

		/* Reading data for X axis */
		a_data_u8r[2] = BMM050_GET_BITSLICE(a_data_u8r[2],
											BMM050_BMX055_REMAPPED_DATAX_LSB_VALUEX);
		raw_dataXYZ.raw_dataX = (BMM050_S16) ((((BMM050_S16)
												((signed char) a_data_u8r[3])) <<
											   SHIFT_LEFT_5_POSITION) | a_data_u8r[2]);
		raw_dataXYZ.raw_dataX = -raw_dataXYZ.raw_dataX;

		/* Reading data for Z axis */
		a_data_u8r[4] = BMM050_GET_BITSLICE(a_data_u8r[4],
											BMM050_DATAZ_LSB_VALUEZ);
		raw_dataXYZ.raw_dataZ = (BMM050_S16) ((((BMM050_S16)
												((signed char) a_data_u8r[5])) <<
											   SHIFT_LEFT_7_POSITION) | a_data_u8r[4]);

		/* Reading data for Resistance*/
		a_data_u8r[6] = BMM050_GET_BITSLICE(a_data_u8r[6],
											BMM050_R_LSB_VALUE);
		raw_dataXYZ.raw_dataR = (BMM050_U16) ((((BMM050_U16)
												a_data_u8r[7]) <<
											   SHIFT_LEFT_6_POSITION) | a_data_u8r[6]);

		/* Compensation for X axis */
		mdata->datax = bmm050_compensate_X(raw_dataXYZ.raw_dataX,
										   raw_dataXYZ.raw_dataR);

		/* Compensation for Y axis */
		mdata->datay = bmm050_compensate_Y(raw_dataXYZ.raw_dataY,
										   raw_dataXYZ.raw_dataR);

		/* Compensation for Z axis */
		mdata->dataz = bmm050_compensate_Z(raw_dataXYZ.raw_dataZ,
										   raw_dataXYZ.raw_dataR);

		/* Output raw resistance value */
		mdata->resistance = raw_dataXYZ.raw_dataR;
	}
	return comres;
}

BMM050_RETURN_FUNCTION_TYPE bmm050_read_mdataXYZ_s32 \
	(struct bmm050_mdata_s32 *mdata)
{
	BMM050_RETURN_FUNCTION_TYPE comres;

	unsigned char a_data_u8r[8];

	struct {
		BMM050_S16 raw_dataX;
		BMM050_S16 raw_dataY;
		BMM050_S16 raw_dataZ;
		BMM050_U16 raw_dataR;
	} raw_dataXYZ;

	if (p_bmm050 == BMM050_NULL) {
		return E_BMM050_NULL_PTR;
	}
	else {
		comres = p_bmm050->BMM050_BUS_READ_FUNC(p_bmm050->dev_addr,
												BMM050_DATAX_LSB, a_data_u8r, 8);

		a_data_u8r[0] = BMM050_GET_BITSLICE(a_data_u8r[0],
											BMM050_DATAX_LSB_VALUEX);

		raw_dataXYZ.raw_dataX = (BMM050_S16) (
			(((BMM050_U16) a_data_u8r[1]) << BMM050_DATAX_LSB_VALUEX__LEN) |
			a_data_u8r[0]);
		if (a_data_u8r[1] & 0x80) {
			raw_dataXYZ.raw_dataX |= 0xe000;
		}

		a_data_u8r[2] = BMM050_GET_BITSLICE(a_data_u8r[2],
											BMM050_DATAY_LSB_VALUEY);

		raw_dataXYZ.raw_dataY = (BMM050_S16) (
			(((BMM050_U16) a_data_u8r[3]) << BMM050_DATAY_LSB_VALUEY__LEN) |
			a_data_u8r[2]);
		if (a_data_u8r[3] & 0x80) {
			raw_dataXYZ.raw_dataY |= 0xe000;
		}

		a_data_u8r[4] = BMM050_GET_BITSLICE(a_data_u8r[4],
											BMM050_DATAZ_LSB_VALUEZ);

		raw_dataXYZ.raw_dataZ = (BMM050_S16) (
			(((BMM050_U16) a_data_u8r[5]) << BMM050_DATAZ_LSB_VALUEZ__LEN) |
			a_data_u8r[4]);
		if (a_data_u8r[5] & 0x80) {
			raw_dataXYZ.raw_dataZ |= 0x8000;
		}

		/* Reading data for Resistance*/
		a_data_u8r[6] = BMM050_GET_BITSLICE(a_data_u8r[6],
											BMM050_R_LSB_VALUE);

		raw_dataXYZ.raw_dataR = (BMM050_S16) ((((BMM050_U16) a_data_u8r[7]) << BMM050_R_LSB_VALUE__LEN) |
											  a_data_u8r[6]);
		if (a_data_u8r[7] & 0x80) {
			raw_dataXYZ.raw_dataR |= 0xc000;
		}

#if 1
		mdata->datax = raw_dataXYZ.raw_dataX;
		mdata->datay = raw_dataXYZ.raw_dataY;
		mdata->dataz = raw_dataXYZ.raw_dataZ;

#else

		/* Compensation for X axis */
		mdata->datax = bmm050_compensate_X_s32(raw_dataXYZ.raw_dataX,
											   raw_dataXYZ.raw_dataR);

		/* Compensation for Y axis */
		mdata->datay = bmm050_compensate_Y_s32(raw_dataXYZ.raw_dataY,
											   raw_dataXYZ.raw_dataR);

		/* Compensation for Z axis */
		mdata->dataz = bmm050_compensate_Z_s32(raw_dataXYZ.raw_dataZ,
											   raw_dataXYZ.raw_dataR);
#endif
#if 0
		comres = p_bmm050->BMM050_BUS_READ_FUNC(p_bmm050->dev_addr,
												BMM050_DATAX_LSB, a_data_u8r, 8);

		/* Reading data for X axis */
		a_data_u8r[0] = BMM050_GET_BITSLICE(a_data_u8r[0],
											BMM050_DATAX_LSB_VALUEX);
		raw_dataXYZ.raw_dataX = (BMM050_S16) ((((BMM050_S16)
												((signed char) a_data_u8r[1])) <<
											   SHIFT_LEFT_5_POSITION) | a_data_u8r[0]);

		/* Reading data for Y axis */
		a_data_u8r[2] = BMM050_GET_BITSLICE(a_data_u8r[2],
											BMM050_DATAY_LSB_VALUEY);
		raw_dataXYZ.raw_dataY = (BMM050_S16) ((((BMM050_S16)
												((signed char) a_data_u8r[3])) <<
											   SHIFT_LEFT_5_POSITION) | a_data_u8r[2]);

		/* Reading data for Z axis */
		a_data_u8r[4] = BMM050_GET_BITSLICE(a_data_u8r[4],
											BMM050_DATAZ_LSB_VALUEZ);
		raw_dataXYZ.raw_dataZ = (BMM050_S16) ((((BMM050_S16)
												((signed char) a_data_u8r[5])) <<
											   SHIFT_LEFT_7_POSITION) | a_data_u8r[4]);

		/* Reading data for Resistance*/
		a_data_u8r[6] = BMM050_GET_BITSLICE(a_data_u8r[6],
											BMM050_R_LSB_VALUE);
		raw_dataXYZ.raw_dataR = (BMM050_U16) ((((BMM050_U16)
												a_data_u8r[7]) <<
											   SHIFT_LEFT_6_POSITION) | a_data_u8r[6]);

		/* Compensation for X axis */
		mdata->datax = bmm050_compensate_X_s32(raw_dataXYZ.raw_dataX,
											   raw_dataXYZ.raw_dataR);

		/* Compensation for Y axis */
		mdata->datay = bmm050_compensate_Y_s32(raw_dataXYZ.raw_dataY,
											   raw_dataXYZ.raw_dataR);

		/* Compensation for Z axis */
		mdata->dataz = bmm050_compensate_Z_s32(raw_dataXYZ.raw_dataZ,
											   raw_dataXYZ.raw_dataR);

		/* Output raw resistance value */
		mdata->resistance = raw_dataXYZ.raw_dataR;
#endif
	}
	return comres;
}

/* In this function X and Y axis is remapped,
 * this API is only applicable for BMX055*/
BMM050_RETURN_FUNCTION_TYPE bmm050_read_bmx055_remapped_mdataXYZ_s32 \
	(struct bmm050_remapped_mdata_s32 *mdata)
{
	BMM050_RETURN_FUNCTION_TYPE comres;

	unsigned char a_data_u8r[8];

	struct {
		BMM050_S16 raw_dataX;
		BMM050_S16 raw_dataY;
		BMM050_S16 raw_dataZ;
		BMM050_U16 raw_dataR;
	} raw_dataXYZ;

	if (p_bmm050 == BMM050_NULL) {
		return E_BMM050_NULL_PTR;
	}
	else {
		comres = p_bmm050->BMM050_BUS_READ_FUNC(p_bmm050->dev_addr,	\
												BMM050_BMX055_REMAPPED_DATAY_LSB, a_data_u8r, 8);

		/* Reading data for Y axis */
		a_data_u8r[0] = BMM050_GET_BITSLICE(a_data_u8r[0],
											BMM050_BMX055_REMAPPED_DATAY_LSB_VALUEY);
		raw_dataXYZ.raw_dataY = (BMM050_S16) ((((BMM050_S16)
												((signed char) a_data_u8r[1])) <<
											   SHIFT_LEFT_5_POSITION) | a_data_u8r[0]);

		/* Reading data for X axis */
		a_data_u8r[2] = BMM050_GET_BITSLICE(a_data_u8r[2],
											BMM050_BMX055_REMAPPED_DATAX_LSB_VALUEX);
		raw_dataXYZ.raw_dataX = (BMM050_S16) ((((BMM050_S16)
												((signed char) a_data_u8r[3])) <<
											   SHIFT_LEFT_5_POSITION) | a_data_u8r[2]);
		raw_dataXYZ.raw_dataX = -raw_dataXYZ.raw_dataX;

		/* Reading data for Z axis */
		a_data_u8r[4] = BMM050_GET_BITSLICE(a_data_u8r[4],
											BMM050_DATAZ_LSB_VALUEZ);
		raw_dataXYZ.raw_dataZ = (BMM050_S16) ((((BMM050_S16)
												((signed char) a_data_u8r[5])) <<
											   SHIFT_LEFT_7_POSITION) | a_data_u8r[4]);

		/* Reading data for Resistance*/
		a_data_u8r[6] = BMM050_GET_BITSLICE(a_data_u8r[6],
											BMM050_R_LSB_VALUE);
		raw_dataXYZ.raw_dataR = (BMM050_U16) ((((BMM050_U16)
												a_data_u8r[7]) <<
											   SHIFT_LEFT_6_POSITION) | a_data_u8r[6]);

		/* Compensation for X axis */
		mdata->datax = bmm050_compensate_X_s32(raw_dataXYZ.raw_dataX,
											   raw_dataXYZ.raw_dataR);

		/* Compensation for Y axis */
		mdata->datay = bmm050_compensate_Y_s32(raw_dataXYZ.raw_dataY,
											   raw_dataXYZ.raw_dataR);

		/* Compensation for Z axis */
		mdata->dataz = bmm050_compensate_Z_s32(raw_dataXYZ.raw_dataZ,
											   raw_dataXYZ.raw_dataR);

		/* Output raw resistance value */
		mdata->resistance = raw_dataXYZ.raw_dataR;
	}
	return comres;
}

#ifdef ENABLE_FLOAT
BMM050_RETURN_FUNCTION_TYPE bmm050_read_mdataXYZ_float \
	(struct bmm050_mdata_float *mdata)
{
	BMM050_RETURN_FUNCTION_TYPE comres;

	unsigned char a_data_u8r[8];

	struct {
		BMM050_S16 raw_dataX;
		BMM050_S16 raw_dataY;
		BMM050_S16 raw_dataZ;
		BMM050_U16 raw_dataR;
	} raw_dataXYZ;

	if (p_bmm050 == BMM050_NULL) {
		return E_BMM050_NULL_PTR;
	}
	else {
		comres = p_bmm050->BMM050_BUS_READ_FUNC(p_bmm050->dev_addr,
												BMM050_DATAX_LSB, a_data_u8r, 8);

		/* Reading data for X axis */
		a_data_u8r[0] = BMM050_GET_BITSLICE(a_data_u8r[0],
											BMM050_DATAX_LSB_VALUEX);
		raw_dataXYZ.raw_dataX = (BMM050_S16) ((((BMM050_S16)
												((signed char) a_data_u8r[1])) <<
											   SHIFT_LEFT_5_POSITION) | a_data_u8r[0]);

		/* Reading data for Y axis */
		a_data_u8r[2] = BMM050_GET_BITSLICE(a_data_u8r[2],
											BMM050_DATAY_LSB_VALUEY);
		raw_dataXYZ.raw_dataY = (BMM050_S16) ((((BMM050_S16)
												((signed char) a_data_u8r[3])) <<
											   SHIFT_LEFT_5_POSITION) | a_data_u8r[2]);

		/* Reading data for Z axis */
		a_data_u8r[4] = BMM050_GET_BITSLICE(a_data_u8r[4],
											BMM050_DATAZ_LSB_VALUEZ);
		raw_dataXYZ.raw_dataZ = (BMM050_S16) ((((BMM050_S16)
												((signed char) a_data_u8r[5])) <<
											   SHIFT_LEFT_7_POSITION) | a_data_u8r[4]);

		/* Reading data for Resistance*/
		a_data_u8r[6] = BMM050_GET_BITSLICE(a_data_u8r[6],
											BMM050_R_LSB_VALUE);
		raw_dataXYZ.raw_dataR = (BMM050_U16) ((((BMM050_U16)
												a_data_u8r[7]) <<
											   SHIFT_LEFT_6_POSITION) | a_data_u8r[6]);

		/* Compensation for X axis */
		mdata->datax = bmm050_compensate_X_float(raw_dataXYZ.raw_dataX,
												 raw_dataXYZ.raw_dataR);

		/* Compensation for Y axis */
		mdata->datay = bmm050_compensate_Y_float(raw_dataXYZ.raw_dataY,
												 raw_dataXYZ.raw_dataR);

		/* Compensation for Z axis */
		mdata->dataz = bmm050_compensate_Z_float(raw_dataXYZ.raw_dataZ,
												 raw_dataXYZ.raw_dataR);

		/* Output raw resistance value */
		mdata->resistance = raw_dataXYZ.raw_dataR;
	}
	return comres;
}

#endif

/* In this function X and Y axis is remapped,
 * this API is only applicable for BMX055*/
#ifdef ENABLE_FLOAT
BMM050_RETURN_FUNCTION_TYPE bmm050_read_bmx055_remapped_mdataXYZ_float \
	(struct bmm050_remapped_mdata_float *mdata)
{
	BMM050_RETURN_FUNCTION_TYPE comres;

	unsigned char a_data_u8r[8];

	struct {
		BMM050_S16 raw_dataX;
		BMM050_S16 raw_dataY;
		BMM050_S16 raw_dataZ;
		BMM050_U16 raw_dataR;
	} raw_dataXYZ;

	if (p_bmm050 == BMM050_NULL) {
		return E_BMM050_NULL_PTR;
	}
	else {
		comres = p_bmm050->BMM050_BUS_READ_FUNC(p_bmm050->dev_addr,	\
												BMM050_BMX055_REMAPPED_DATAY_LSB, a_data_u8r, 8);

		/* Reading data for Y axis */
		a_data_u8r[0] = BMM050_GET_BITSLICE(a_data_u8r[0],
											BMM050_BMX055_REMAPPED_DATAY_LSB_VALUEY);
		raw_dataXYZ.raw_dataY = (BMM050_S16) ((((BMM050_S16)
												((signed char) a_data_u8r[1])) <<
											   SHIFT_LEFT_5_POSITION) | a_data_u8r[0]);

		/* Reading data for X axis */
		a_data_u8r[2] = BMM050_GET_BITSLICE(a_data_u8r[2],
											BMM050_BMX055_REMAPPED_DATAX_LSB_VALUEX);
		raw_dataXYZ.raw_dataX = (BMM050_S16) ((((BMM050_S16)
												((signed char) a_data_u8r[3])) <<
											   SHIFT_LEFT_5_POSITION) | a_data_u8r[2]);
		raw_dataXYZ.raw_dataX = -raw_dataXYZ.raw_dataX;

		/* Reading data for Z axis */
		a_data_u8r[4] = BMM050_GET_BITSLICE(a_data_u8r[4],
											BMM050_DATAZ_LSB_VALUEZ);
		raw_dataXYZ.raw_dataZ = (BMM050_S16) ((((BMM050_S16)
												((signed char) a_data_u8r[5])) <<
											   SHIFT_LEFT_7_POSITION) | a_data_u8r[4]);

		/* Reading data for Resistance*/
		a_data_u8r[6] = BMM050_GET_BITSLICE(a_data_u8r[6],
											BMM050_R_LSB_VALUE);
		raw_dataXYZ.raw_dataR = (BMM050_U16) ((((BMM050_U16)
												a_data_u8r[7]) <<
											   SHIFT_LEFT_6_POSITION) | a_data_u8r[6]);

		/* Compensation for X axis */
		mdata->datax = bmm050_compensate_X_float(raw_dataXYZ.raw_dataX,	\
												 raw_dataXYZ.raw_dataR);

		/* Compensation for Y axis */
		mdata->datay = bmm050_compensate_Y_float(raw_dataXYZ.raw_dataY,	\
												 raw_dataXYZ.raw_dataR);

		/* Compensation for Z axis */
		mdata->dataz = bmm050_compensate_Z_float(raw_dataXYZ.raw_dataZ,	\
												 raw_dataXYZ.raw_dataR);

		/* Output raw resistance value */
		mdata->resistance = raw_dataXYZ.raw_dataR;
	}
	return comres;
}

#endif

BMM050_RETURN_FUNCTION_TYPE bmm050_read_register(unsigned char addr, \
												 unsigned char *data, unsigned char len)
{
	BMM050_RETURN_FUNCTION_TYPE comres = 0;
	if (p_bmm050 == BMM050_NULL) {
		return E_BMM050_NULL_PTR;
	}
	else {
		comres += p_bmm050->BMM050_BUS_READ_FUNC(p_bmm050->dev_addr, \
												 addr, data, len);
	}
	return comres;
}

BMM050_RETURN_FUNCTION_TYPE bmm050_write_register(unsigned char addr, \
												  unsigned char *data, unsigned char len)
{
	BMM050_RETURN_FUNCTION_TYPE comres = 0;
	if (p_bmm050 == BMM050_NULL) {
		return E_BMM050_NULL_PTR;
	}
	else {
		comres = p_bmm050->BMM050_BUS_WRITE_FUNC(p_bmm050->dev_addr, \
												 addr, data, len);
	}
	return comres;
}

BMM050_RETURN_FUNCTION_TYPE bmm050_set_selftest(unsigned char selftest)
{
	BMM050_RETURN_FUNCTION_TYPE comres = 0;
	unsigned char v_data1_u8r;
	if (p_bmm050 == BMM050_NULL) {
		return E_BMM050_NULL_PTR;
	}
	else {
		comres = p_bmm050->BMM050_BUS_READ_FUNC(
			p_bmm050->dev_addr, BMM050_CNTL_S_TEST__REG,
			&v_data1_u8r, 1);
		v_data1_u8r = BMM050_SET_BITSLICE(
			v_data1_u8r, BMM050_CNTL_S_TEST, selftest);
		comres |= p_bmm050->BMM050_BUS_WRITE_FUNC(
			p_bmm050->dev_addr, BMM050_CNTL_S_TEST__REG,
			&v_data1_u8r, 1);
	}
	return comres;
}

BMM050_RETURN_FUNCTION_TYPE bmm050_get_self_test_XYZ \
	(unsigned char *self_testxyz)
{
	BMM050_RETURN_FUNCTION_TYPE comres = 0;
	unsigned char a_data_u8r[5], v_result_u8r = 0x00;
	if (p_bmm050 == BMM050_NULL) {
		return E_BMM050_NULL_PTR;
	}
	else {
		comres = p_bmm050->BMM050_BUS_READ_FUNC(
			p_bmm050->dev_addr, BMM050_DATAX_LSB_TESTX__REG,
			a_data_u8r, 5);

		v_result_u8r = BMM050_GET_BITSLICE(a_data_u8r[4],
										   BMM050_DATAZ_LSB_TESTZ);

		v_result_u8r = (v_result_u8r << 1);
		v_result_u8r = (v_result_u8r | BMM050_GET_BITSLICE(
							a_data_u8r[2], BMM050_DATAY_LSB_TESTY));

		v_result_u8r = (v_result_u8r << 1);
		v_result_u8r = (v_result_u8r | BMM050_GET_BITSLICE(
							a_data_u8r[0], BMM050_DATAX_LSB_TESTX));

		*self_testxyz = v_result_u8r;
	}
	return comres;
}

BMM050_RETURN_FUNCTION_TYPE bmm050_set_spi3(unsigned char value)
{
	BMM050_RETURN_FUNCTION_TYPE comres = 0;
	unsigned char v_data1_u8r;
	if (p_bmm050 == BMM050_NULL) {
		return E_BMM050_NULL_PTR;
	}
	else {
		comres = p_bmm050->BMM050_BUS_READ_FUNC(p_bmm050->dev_addr,
												BMM050_POWER_CNTL_SPI3_EN__REG, &v_data1_u8r, 1);
		v_data1_u8r = BMM050_SET_BITSLICE(v_data1_u8r,
										  BMM050_POWER_CNTL_SPI3_EN, value);
		comres |= p_bmm050->BMM050_BUS_WRITE_FUNC(p_bmm050->dev_addr,
												  BMM050_POWER_CNTL_SPI3_EN__REG, &v_data1_u8r, 1);
	}
	return comres;
}

BMM050_RETURN_FUNCTION_TYPE bmm050_set_datarate(unsigned char data_rate)
{
	BMM050_RETURN_FUNCTION_TYPE comres = 0;
	unsigned char v_data1_u8r;
	if (p_bmm050 == BMM050_NULL) {
		return E_BMM050_NULL_PTR;
	}
	else {
		comres = p_bmm050->BMM050_BUS_READ_FUNC(
			p_bmm050->dev_addr,
			BMM050_CNTL_DR__REG,
			&v_data1_u8r, 1);
		v_data1_u8r = BMM050_SET_BITSLICE(v_data1_u8r,
										  BMM050_CNTL_DR, data_rate);
		comres |= p_bmm050->BMM050_BUS_WRITE_FUNC(
			p_bmm050->dev_addr,
			BMM050_CNTL_DR__REG,
			&v_data1_u8r, 1);
	}
	return comres;
}

BMM050_RETURN_FUNCTION_TYPE bmm050_get_datarate(unsigned char *data_rate)
{
	BMM050_RETURN_FUNCTION_TYPE comres = 0;
	unsigned char v_data_u8r;
	if (p_bmm050 == BMM050_NULL) {
		return E_BMM050_NULL_PTR;
	}
	else {
		comres = p_bmm050->BMM050_BUS_READ_FUNC(
			p_bmm050->dev_addr,
			BMM050_CNTL_DR__REG,
			&v_data_u8r, 1);
		*data_rate = BMM050_GET_BITSLICE(v_data_u8r,
										 BMM050_CNTL_DR);
	}
	return comres;
}

BMM050_RETURN_FUNCTION_TYPE bmm050_perform_advanced_selftest \
	(BMM050_S16 *diff_z)
{
	BMM050_RETURN_FUNCTION_TYPE comres;
	BMM050_S16 result_positive, result_negative;
	struct bmm050_mdata mdata;

	if (p_bmm050 == BMM050_NULL) {
		return E_BMM050_NULL_PTR;
	}
	else {
		/* set sleep mode to prepare for forced measurement.
		 * If sensor is off, this will turn it on
		 * and respect needed delays. */
		comres = bmm050_set_functional_state(BMM050_SLEEP_MODE);

		/* set normal accuracy mode */
		comres |= bmm050_set_repetitions_Z(BMM050_LOWPOWER_REPZ);
		/* 14 repetitions Z in normal accuracy mode */

		/* disable X, Y channel */
		comres |= bmm050_set_control_measurement_x(
			BMM050_CHANNEL_DISABLE);
		comres |= bmm050_set_control_measurement_y(
			BMM050_CHANNEL_DISABLE);

		/* enable positive current and force a
		 * measurement with positive field */
		comres |= bmm050_set_adv_selftest(
			BMM050_ADVANCED_SELFTEST_POSITIVE);
		comres |= bmm050_set_functional_state(BMM050_FORCED_MODE);
		/* wait for measurement to complete */
		p_bmm050->delay_msec(4);

		/* read result from positive field measurement */
		comres |= bmm050_read_mdataXYZ(&mdata);
		result_positive = mdata.dataz;

		/* enable negative current and force a
		 * measurement with negative field */
		comres |= bmm050_set_adv_selftest(
			BMM050_ADVANCED_SELFTEST_NEGATIVE);
		comres |= bmm050_set_functional_state(BMM050_FORCED_MODE);
		p_bmm050->delay_msec(4);/* wait for measurement to complete */

		/* read result from negative field measurement */
		comres |= bmm050_read_mdataXYZ(&mdata);
		result_negative = mdata.dataz;

		/* turn off self test current */
		comres |= bmm050_set_adv_selftest(
			BMM050_ADVANCED_SELFTEST_OFF);

		/* enable X, Y channel */
		comres |= bmm050_set_control_measurement_x(
			BMM050_CHANNEL_ENABLE);
		comres |= bmm050_set_control_measurement_y(
			BMM050_CHANNEL_ENABLE);

		/* write out difference in positive and negative field.
		 * This should be ~ 200 mT = 3200 LSB */
		*diff_z = (result_positive - result_negative);
	}
	return comres;
}

BMM050_RETURN_FUNCTION_TYPE bmm050_init_trim_registers(void)
{
	BMM050_RETURN_FUNCTION_TYPE comres = 0;
	unsigned char a_data_u8r[2];
	comres = p_bmm050->BMM050_BUS_READ_FUNC(p_bmm050->dev_addr,
											BMM050_DIG_X1, (unsigned char *) &p_bmm050->dig_x1, 1);
	comres |= p_bmm050->BMM050_BUS_READ_FUNC(p_bmm050->dev_addr,
											 BMM050_DIG_Y1, (unsigned char *) &p_bmm050->dig_y1, 1);
	comres |= p_bmm050->BMM050_BUS_READ_FUNC(p_bmm050->dev_addr,
											 BMM050_DIG_X2, (unsigned char *) &p_bmm050->dig_x2, 1);
	comres |= p_bmm050->BMM050_BUS_READ_FUNC(p_bmm050->dev_addr,
											 BMM050_DIG_Y2, (unsigned char *) &p_bmm050->dig_y2, 1);
	comres |= p_bmm050->BMM050_BUS_READ_FUNC(p_bmm050->dev_addr,
											 BMM050_DIG_XY1, (unsigned char *) &p_bmm050->dig_xy1, 1);
	comres |= p_bmm050->BMM050_BUS_READ_FUNC(p_bmm050->dev_addr,
											 BMM050_DIG_XY2, (unsigned char *) &p_bmm050->dig_xy2, 1);

	/* shorts can not be recast into (unsigned char*)
	 * due to possible mix up between trim data
	 * arrangement and memory arrangement */

	comres |= p_bmm050->BMM050_BUS_READ_FUNC(p_bmm050->dev_addr,
											 BMM050_DIG_Z1_LSB, a_data_u8r, 2);
	p_bmm050->dig_z1 = (BMM050_U16) ((((BMM050_U16) ((unsigned char)
													 a_data_u8r[1])) <<
									  SHIFT_LEFT_8_POSITION) | a_data_u8r[0]);

	comres |= p_bmm050->BMM050_BUS_READ_FUNC(p_bmm050->dev_addr,
											 BMM050_DIG_Z2_LSB, a_data_u8r, 2);
	p_bmm050->dig_z2 = (BMM050_S16) ((((BMM050_S16) (
										   (signed char) a_data_u8r[1])) <<
									  SHIFT_LEFT_8_POSITION) | a_data_u8r[0]);

	comres |= p_bmm050->BMM050_BUS_READ_FUNC(p_bmm050->dev_addr,
											 BMM050_DIG_Z3_LSB, a_data_u8r, 2);
	p_bmm050->dig_z3 = (BMM050_S16) ((((BMM050_S16) (
										   (signed char) a_data_u8r[1])) <<
									  SHIFT_LEFT_8_POSITION) | a_data_u8r[0]);

	comres |= p_bmm050->BMM050_BUS_READ_FUNC(p_bmm050->dev_addr,
											 BMM050_DIG_Z4_LSB, a_data_u8r, 2);
	p_bmm050->dig_z4 = (BMM050_S16) ((((BMM050_S16) (
										   (signed char) a_data_u8r[1])) <<
									  SHIFT_LEFT_8_POSITION) | a_data_u8r[0]);

	comres |= p_bmm050->BMM050_BUS_READ_FUNC(p_bmm050->dev_addr,
											 BMM050_DIG_XYZ1_LSB, a_data_u8r, 2);
	a_data_u8r[1] = BMM050_GET_BITSLICE(a_data_u8r[1], BMM050_DIG_XYZ1_MSB);
	p_bmm050->dig_xyz1 = (BMM050_U16) ((((BMM050_U16)
										 ((unsigned char) a_data_u8r[1])) <<
										SHIFT_LEFT_8_POSITION) | a_data_u8r[0]);
	return comres;
}

BMM050_RETURN_FUNCTION_TYPE bmm050_set_adv_selftest	\
	(unsigned char adv_selftest)
{
	BMM050_RETURN_FUNCTION_TYPE comres = 0;
	unsigned char v_data1_u8r;
	if (p_bmm050 == BMM050_NULL) {
		return E_BMM050_NULL_PTR;
	}
	else {
		switch (adv_selftest) {
		case BMM050_ADVANCED_SELFTEST_OFF:
			comres = p_bmm050->BMM050_BUS_READ_FUNC(
				p_bmm050->dev_addr,
				BMM050_CNTL_ADV_ST__REG,
				&v_data1_u8r, 1);
			v_data1_u8r = BMM050_SET_BITSLICE(
				v_data1_u8r,
				BMM050_CNTL_ADV_ST,
				BMM050_ADVANCED_SELFTEST_OFF);
			comres |= p_bmm050->BMM050_BUS_WRITE_FUNC(
				p_bmm050->dev_addr,
				BMM050_CNTL_ADV_ST__REG,
				&v_data1_u8r, 1);
			break;

		case BMM050_ADVANCED_SELFTEST_POSITIVE:
			comres = p_bmm050->BMM050_BUS_READ_FUNC(
				p_bmm050->dev_addr,
				BMM050_CNTL_ADV_ST__REG,
				&v_data1_u8r, 1);
			v_data1_u8r = BMM050_SET_BITSLICE(
				v_data1_u8r,
				BMM050_CNTL_ADV_ST,
				BMM050_ADVANCED_SELFTEST_POSITIVE);
			comres |= p_bmm050->BMM050_BUS_WRITE_FUNC(
				p_bmm050->dev_addr,
				BMM050_CNTL_ADV_ST__REG,
				&v_data1_u8r, 1);
			break;

		case BMM050_ADVANCED_SELFTEST_NEGATIVE:
			comres = p_bmm050->BMM050_BUS_READ_FUNC(
				p_bmm050->dev_addr,
				BMM050_CNTL_ADV_ST__REG,
				&v_data1_u8r, 1);
			v_data1_u8r = BMM050_SET_BITSLICE(
				v_data1_u8r,
				BMM050_CNTL_ADV_ST,
				BMM050_ADVANCED_SELFTEST_NEGATIVE);
			comres |= p_bmm050->BMM050_BUS_WRITE_FUNC(
				p_bmm050->dev_addr,
				BMM050_CNTL_ADV_ST__REG,
				&v_data1_u8r, 1);
			break;

		default:
			break;
		}
	}
	return comres;
}

BMM050_RETURN_FUNCTION_TYPE bmm050_get_adv_selftest	\
	(unsigned char *adv_selftest)
{
	BMM050_RETURN_FUNCTION_TYPE comres = 0;
	unsigned char v_data_u8r;
	if (p_bmm050 == BMM050_NULL) {
		return E_BMM050_NULL_PTR;
	}
	else {
		comres = p_bmm050->BMM050_BUS_READ_FUNC(p_bmm050->dev_addr,
												BMM050_CNTL_ADV_ST__REG, &v_data_u8r, 1);
		*adv_selftest = BMM050_GET_BITSLICE(v_data_u8r,
											BMM050_CNTL_ADV_ST);
	}
	return comres;
}

BMM050_RETURN_FUNCTION_TYPE bmm050_get_presetmode \
	(unsigned char *mode)
{
	BMM050_RETURN_FUNCTION_TYPE comres = 0;
	unsigned char data_rate = 0;
	unsigned char repetitionsxy = 0;
	unsigned char repetitionsz = 0;

	/* Get the current data rate */
	comres = bmm050_get_datarate(&data_rate);
	/* Get the preset number of XY Repetitions */
	comres |= bmm050_get_repetitions_XY(&repetitionsxy);
	/* Get the preset number of Z Repetitions */
	comres |= bmm050_get_repetitions_Z(&repetitionsz);
	if ((data_rate == BMM050_LOWPOWER_DR) && (
			repetitionsxy == BMM050_LOWPOWER_REPXY) && (
			repetitionsz == BMM050_LOWPOWER_REPZ)) {
		*mode = BMM050_PRESETMODE_LOWPOWER;
	}
	else {
		if ((data_rate == BMM050_REGULAR_DR) && (
				repetitionsxy == BMM050_REGULAR_REPXY) && (
				repetitionsz == BMM050_REGULAR_REPZ)) {
			*mode = BMM050_PRESETMODE_REGULAR;
		}
		else {
			if ((data_rate == BMM050_HIGHACCURACY_DR) && (
					repetitionsxy == BMM050_HIGHACCURACY_REPXY) && (
					repetitionsz == BMM050_HIGHACCURACY_REPZ)) {
				*mode = BMM050_PRESETMODE_HIGHACCURACY;
			}
			else {
				if ((data_rate == BMM050_ENHANCED_DR) && (
						repetitionsxy == BMM050_ENHANCED_REPXY) && (
						repetitionsz == BMM050_ENHANCED_REPZ)) {
					*mode = BMM050_PRESETMODE_ENHANCED;
				}
				else {
					*mode = E_BMM050_UNDEFINED_MODE;
				}
			}
		}
	}
	return comres;
}

BMM050_RETURN_FUNCTION_TYPE bmm050_get_powermode(unsigned char *mode)
{
	BMM050_RETURN_FUNCTION_TYPE comres = 0;
	unsigned char v_data_u8r;
	if (p_bmm050 == BMM050_NULL) {
		return E_BMM050_NULL_PTR;
	}
	else {
		comres = p_bmm050->BMM050_BUS_READ_FUNC(
			p_bmm050->dev_addr,
			BMM050_POWER_CNTL_PCB__REG,
			&v_data_u8r, 1);
		*mode = BMM050_GET_BITSLICE(v_data_u8r,
									BMM050_POWER_CNTL_PCB);
	}
	return comres;
}

BMM050_RETURN_FUNCTION_TYPE bmm050_set_powermode(unsigned char mode)
{
	BMM050_RETURN_FUNCTION_TYPE comres = 0;
	unsigned char v_data_u8r;
	if (p_bmm050 == BMM050_NULL) {
		return E_BMM050_NULL_PTR;
	}
	else {
		comres = p_bmm050->BMM050_BUS_READ_FUNC(
			p_bmm050->dev_addr,
			BMM050_POWER_CNTL_PCB__REG,
			&v_data_u8r, 1);

		v_data_u8r = BMM050_SET_BITSLICE(v_data_u8r,
										 BMM050_POWER_CNTL_PCB, mode);

		comres |= p_bmm050->BMM050_BUS_WRITE_FUNC(
			p_bmm050->dev_addr,
			BMM050_POWER_CNTL_PCB__REG,
			&v_data_u8r, 1);

		/* wait four millisecond for bmc to settle */

		p_bmm050->delay_msec(BMM050_DELAY_SETTLING_TIME * 4);

		comres = p_bmm050->BMM050_BUS_READ_FUNC(
			p_bmm050->dev_addr,
			BMM050_POWER_CNTL_PCB__REG,
			&v_data_u8r, 1);

	}
	return comres;
}

BMM050_RETURN_FUNCTION_TYPE bmm050_get_repetitions_XY(
	unsigned char *no_repetitions_xy)
{
	BMM050_RETURN_FUNCTION_TYPE comres = 0;
	unsigned char v_data_u8r;
	if (p_bmm050 == BMM050_NULL) {
		return E_BMM050_NULL_PTR;
	}
	else {
		comres = p_bmm050->BMM050_BUS_READ_FUNC(
			p_bmm050->dev_addr,
			BMM050_NO_REPETITIONS_XY,
			&v_data_u8r, 1);
		*no_repetitions_xy = v_data_u8r;
	}
	return comres;
}

BMM050_RETURN_FUNCTION_TYPE bmm050_set_repetitions_XY(
	unsigned char no_repetitions_xy)
{
	BMM050_RETURN_FUNCTION_TYPE comres = 0;
	unsigned char v_data_u8r;
	if (p_bmm050 == BMM050_NULL) {
		return E_BMM050_NULL_PTR;
	}
	else {
		v_data_u8r = no_repetitions_xy;
		comres = p_bmm050->BMM050_BUS_WRITE_FUNC(
			p_bmm050->dev_addr,
			BMM050_NO_REPETITIONS_XY,
			&v_data_u8r, 1);
	}
	return comres;
}

BMM050_RETURN_FUNCTION_TYPE bmm050_get_repetitions_Z(
	unsigned char *no_repetitions_z)
{
	BMM050_RETURN_FUNCTION_TYPE comres = 0;
	unsigned char v_data_u8r;
	if (p_bmm050 == BMM050_NULL) {
		return E_BMM050_NULL_PTR;
	}
	else {
		comres = p_bmm050->BMM050_BUS_READ_FUNC(
			p_bmm050->dev_addr,
			BMM050_NO_REPETITIONS_Z,
			&v_data_u8r, 1);
		*no_repetitions_z = v_data_u8r;
	}
	return comres;
}

BMM050_RETURN_FUNCTION_TYPE bmm050_set_repetitions_Z(
	unsigned char no_repetitions_z)
{
	BMM050_RETURN_FUNCTION_TYPE comres = 0;
	unsigned char v_data_u8r;
	if (p_bmm050 == BMM050_NULL) {
		return E_BMM050_NULL_PTR;
	}
	else {
		v_data_u8r = no_repetitions_z;
		comres = p_bmm050->BMM050_BUS_WRITE_FUNC(p_bmm050->dev_addr,
												 BMM050_NO_REPETITIONS_Z, &v_data_u8r, 1);
	}
	return comres;
}

BMM050_S16 bmm050_compensate_X(BMM050_S16 mdata_x, BMM050_U16 data_R)
{
	BMM050_S16 inter_retval;
	if (mdata_x != BMM050_FLIP_OVERFLOW_ADCVAL	/* no overflow */
		) {
		inter_retval = ((BMM050_S16) (((BMM050_U16)
									   ((((BMM050_S32) p_bmm050->dig_xyz1) << 14) /
										(data_R != 0 ? data_R : p_bmm050->dig_xyz1))) -
									  ((BMM050_U16) 0x4000)));
		inter_retval = ((BMM050_S16) ((((BMM050_S32) mdata_x) *
									   ((((((((BMM050_S32) p_bmm050->dig_xy2) *
											 ((((BMM050_S32) inter_retval) *
											   ((BMM050_S32) inter_retval)) >> 7)) +
											(((BMM050_S32) inter_retval) *
											 ((BMM050_S32) (((BMM050_S16) p_bmm050->dig_xy1)
															<< 7)))) >> 9) +
										  ((BMM050_S32) 0x100000)) *
										 ((BMM050_S32) (((BMM050_S16) p_bmm050->dig_x2) +
														((BMM050_S16) 0xA0)))) >> 12)) >> 13)) +
					   (((BMM050_S16) p_bmm050->dig_x1) << 3);
	}
	else {
		/* overflow */
		inter_retval = BMM050_OVERFLOW_OUTPUT;
	}
	return inter_retval;
}

BMM050_S32 bmm050_compensate_X_s32(BMM050_S16 mdata_x, BMM050_U16 data_R)
{
	BMM050_S32 retval;

	retval = bmm050_compensate_X(mdata_x, data_R);
	if (retval == (BMM050_S32) BMM050_OVERFLOW_OUTPUT) {
		retval = BMM050_OVERFLOW_OUTPUT_S32;
	}
	return retval;
}

#ifdef ENABLE_FLOAT
float bmm050_compensate_X_float(BMM050_S16 mdata_x, BMM050_U16 data_R)
{
	float inter_retval;
	if (mdata_x != BMM050_FLIP_OVERFLOW_ADCVAL	/* no overflow */
		) {
		if (data_R != 0) {
			inter_retval = ((((float) p_bmm050->dig_xyz1) * 16384.0f
							 / data_R) - 16384.0f);
		}
		else {
			inter_retval = 0;
		}
		inter_retval = (((mdata_x * ((((((float) p_bmm050->dig_xy2) *
										(inter_retval * inter_retval / 268435456.0f) +
										inter_retval * ((float) p_bmm050->dig_xy1) / 16384.0f))
									  + 256.0f) * (((float) p_bmm050->dig_x2) + 160.0f)))
						 / 8192.0f) + (((float) p_bmm050->dig_x1) * 8.0f)) / 16.0f;
	}
	else {
		inter_retval = BMM050_OVERFLOW_OUTPUT_FLOAT;
	}
	return inter_retval;
}

#endif

BMM050_S16 bmm050_compensate_Y(BMM050_S16 mdata_y, BMM050_U16 data_R)
{
	BMM050_S16 inter_retval;
	if (mdata_y != BMM050_FLIP_OVERFLOW_ADCVAL	/* no overflow */
		) {
		inter_retval = ((BMM050_S16) (((BMM050_U16) (((
														  (BMM050_S32) p_bmm050->dig_xyz1) << 14) /
													 (data_R != 0 ?
													  data_R : p_bmm050->dig_xyz1))) -
									  ((BMM050_U16) 0x4000)));
		inter_retval = ((BMM050_S16) ((((BMM050_S32) mdata_y) *
									   ((((((((BMM050_S32)
											  p_bmm050->dig_xy2) *
											 ((((BMM050_S32) inter_retval) *
											   ((BMM050_S32) inter_retval)) >> 7)) +
											(((BMM050_S32) inter_retval) *
											 ((BMM050_S32) (((BMM050_S16)
															 p_bmm050->dig_xy1) << 7)))) >> 9) +
										  ((BMM050_S32) 0x100000)) *
										 ((BMM050_S32) (((BMM050_S16) p_bmm050->dig_y2)
														+ ((BMM050_S16) 0xA0))))
										>> 12)) >> 13)) +
					   (((BMM050_S16) p_bmm050->dig_y1) << 3);
	}
	else {
		/* overflow */
		inter_retval = BMM050_OVERFLOW_OUTPUT;
	}
	return inter_retval;
}

BMM050_S32 bmm050_compensate_Y_s32(BMM050_S16 mdata_y, BMM050_U16 data_R)
{
	BMM050_S32 retval;

	retval = bmm050_compensate_Y(mdata_y, data_R);
	if (retval == BMM050_OVERFLOW_OUTPUT) {
		retval = BMM050_OVERFLOW_OUTPUT_S32;
	}
	return retval;
}

#ifdef ENABLE_FLOAT
float bmm050_compensate_Y_float(BMM050_S16 mdata_y, BMM050_U16 data_R)
{
	float inter_retval;
	if (mdata_y != BMM050_FLIP_OVERFLOW_ADCVAL	/* no overflow */
		) {
		if (data_R != 0) {
			inter_retval = ((((float) p_bmm050->dig_xyz1) * 16384.0f
							 / data_R) - 16384.0f);
		}
		else {
			inter_retval = 0;
		}
		inter_retval = (((mdata_y * ((((((float) p_bmm050->dig_xy2) *
										(inter_retval * inter_retval / 268435456.0f) +
										inter_retval * ((float) p_bmm050->dig_xy1) / 16384.0f)) +
									  256.0f) * (((float) p_bmm050->dig_y2) + 160.0f)))
						 / 8192.0f) + (((float) p_bmm050->dig_y1) * 8.0f)) / 16.0f;
	}
	else {
		/* overflow, set output to 0.0f */
		inter_retval = BMM050_OVERFLOW_OUTPUT_FLOAT;
	}
	return inter_retval;
}

#endif

BMM050_S16 bmm050_compensate_Z(BMM050_S16 mdata_z, BMM050_U16 data_R)
{
	BMM050_S32 retval;
	if ((mdata_z != BMM050_HALL_OVERFLOW_ADCVAL)	/* no overflow */
		) {
		retval = (((((BMM050_S32) (mdata_z - p_bmm050->dig_z4)) << 15) -
				   ((((BMM050_S32) p_bmm050->dig_z3) *
					 ((BMM050_S32) (((BMM050_S16) data_R) -
									((BMM050_S16)
									 p_bmm050->dig_xyz1)))) >> 2)) /
				  (p_bmm050->dig_z2 +
				   ((BMM050_S16) (((((BMM050_S32)
									 p_bmm050->dig_z1) *
									((((BMM050_S16) data_R) << 1))) +
								   (1 << 15)) >> 16))));
		/* saturate result to +/- 2 mT */
		if (retval > BMM050_POSITIVE_SATURATION_Z) {
			retval =  BMM050_POSITIVE_SATURATION_Z;
		}
		else {
			if (retval < BMM050_NEGATIVE_SATURATION_Z) {
				retval = BMM050_NEGATIVE_SATURATION_Z;
			}
		}
	}
	else {
		/* overflow */
		retval = BMM050_OVERFLOW_OUTPUT;
	}
	return (BMM050_S16) retval;
}

BMM050_S32 bmm050_compensate_Z_s32(BMM050_S16 mdata_z, BMM050_U16 data_R)
{
	BMM050_S32 retval;
	if (mdata_z != BMM050_HALL_OVERFLOW_ADCVAL) {
		retval = (((((BMM050_S32) (mdata_z - p_bmm050->dig_z4)) << 15) -
				   ((((BMM050_S32) p_bmm050->dig_z3) *
					 ((BMM050_S32) (((BMM050_S16) data_R) -
									((BMM050_S16) p_bmm050->dig_xyz1)))) >> 2)) /
				  (p_bmm050->dig_z2 +
				   ((BMM050_S16) (((((BMM050_S32) p_bmm050->dig_z1) *
									((((BMM050_S16) data_R) << 1))) + (1 << 15)) >> 16))));
	}
	else {
		retval = BMM050_OVERFLOW_OUTPUT_S32;
	}
	return retval;
}

#ifdef ENABLE_FLOAT
float bmm050_compensate_Z_float(BMM050_S16 mdata_z, BMM050_U16 data_R)
{
	float inter_retval;
	if (mdata_z != BMM050_HALL_OVERFLOW_ADCVAL	/* no overflow */
		) {
		inter_retval = ((((((float) mdata_z) - ((float) p_bmm050->dig_z4)) *
						  131072.0f) - (((float) p_bmm050->dig_z3) * (((float) data_R) -
																	  ((float) p_bmm050->dig_xyz1)))) /
						((((float) p_bmm050->dig_z2) +
						  ((float)
						   p_bmm050->
						   dig_z1) * ((float) data_R) / 32768.0f) * 4.0f)) / 16.0f;
	}
	else {
		/* overflow, set output to 0.0f */
		inter_retval = BMM050_OVERFLOW_OUTPUT_FLOAT;
	}
	return inter_retval;
}

#endif

BMM050_RETURN_FUNCTION_TYPE bmm050_set_control_measurement_x \
	(unsigned char enable_disable)
{
	BMM050_RETURN_FUNCTION_TYPE comres = 0;
	unsigned char v_data1_u8r;
	if (p_bmm050 == BMM050_NULL) {
		return E_BMM050_NULL_PTR;
	}
	else {
		comres = p_bmm050->BMM050_BUS_READ_FUNC(
			p_bmm050->dev_addr,
			BMM050_SENS_CNTL_CHANNELX__REG,
			&v_data1_u8r, 1);
		v_data1_u8r = BMM050_SET_BITSLICE(v_data1_u8r,
										  BMM050_SENS_CNTL_CHANNELX,
										  enable_disable);
		comres |= p_bmm050->BMM050_BUS_WRITE_FUNC(
			p_bmm050->dev_addr,
			BMM050_SENS_CNTL_CHANNELX__REG,
			&v_data1_u8r, 1);
	}
	return comres;
}

BMM050_RETURN_FUNCTION_TYPE bmm050_set_control_measurement_y \
	(unsigned char enable_disable)
{
	BMM050_RETURN_FUNCTION_TYPE comres = 0;
	unsigned char v_data1_u8r;
	if (p_bmm050 == BMM050_NULL) {
		return E_BMM050_NULL_PTR;
	}
	else {
		comres = p_bmm050->BMM050_BUS_READ_FUNC(
			p_bmm050->dev_addr,
			BMM050_SENS_CNTL_CHANNELY__REG,
			&v_data1_u8r, 1);
		v_data1_u8r = BMM050_SET_BITSLICE(
			v_data1_u8r,
			BMM050_SENS_CNTL_CHANNELY,
			enable_disable);
		comres |= p_bmm050->BMM050_BUS_WRITE_FUNC(
			p_bmm050->dev_addr,
			BMM050_SENS_CNTL_CHANNELY__REG,
			&v_data1_u8r, 1);
	}
	return comres;
}

BMM050_RETURN_FUNCTION_TYPE bmm050_soft_reset(void)
{
	BMM050_RETURN_FUNCTION_TYPE comres = 0;
	unsigned char v_data_u8r;
	if (p_bmm050 == BMM050_NULL) {
		return E_BMM050_NULL_PTR;
	}
	else {
		v_data_u8r = BMM050_ON;

		comres = p_bmm050->BMM050_BUS_READ_FUNC(
			p_bmm050->dev_addr,
			BMM050_POWER_CNTL_SRST7__REG,
			&v_data_u8r, 1);
		v_data_u8r = BMM050_SET_BITSLICE(v_data_u8r,
										 BMM050_POWER_CNTL_SRST7,
										 BMM050_SOFT_RESET7_ON);
		comres |= p_bmm050->BMM050_BUS_WRITE_FUNC(
			p_bmm050->dev_addr,
			BMM050_POWER_CNTL_SRST7__REG, &v_data_u8r, 1);

		comres |= p_bmm050->BMM050_BUS_READ_FUNC(
			p_bmm050->dev_addr,
			BMM050_POWER_CNTL_SRST1__REG,
			&v_data_u8r, 1);
		v_data_u8r = BMM050_SET_BITSLICE(v_data_u8r,
										 BMM050_POWER_CNTL_SRST1,
										 BMM050_SOFT_RESET1_ON);
		comres |= p_bmm050->BMM050_BUS_WRITE_FUNC(
			p_bmm050->dev_addr,
			BMM050_POWER_CNTL_SRST1__REG,
			&v_data_u8r, 1);

		p_bmm050->delay_msec(BMM050_DELAY_SOFTRESET);
	}
	return comres;
}

BMM050_RETURN_FUNCTION_TYPE bmm050_get_raw_xyz(struct bmm050_mdata *mdata)
{
	BMM050_RETURN_FUNCTION_TYPE comres;
	unsigned char a_data_u8r[8];
	if (p_bmm050 == BMM050_NULL) {
		return E_BMM050_NULL_PTR;
	}
	else {
		comres = p_bmm050->BMM050_BUS_READ_FUNC(p_bmm050->dev_addr,
					BMM050_DATAX_LSB, a_data_u8r, 8);

		a_data_u8r[0] = BMM050_GET_BITSLICE(a_data_u8r[0],
					BMM050_DATAX_LSB_VALUEX);
		mdata->datax = (BMM050_S16) ((((BMM050_S16)
				   ((signed char) a_data_u8r[1]))
				  << SHIFT_LEFT_5_POSITION)
				 | a_data_u8r[0]);

		a_data_u8r[2] = BMM050_GET_BITSLICE(a_data_u8r[2],
					BMM050_DATAY_LSB_VALUEY);
		mdata->datay = (BMM050_S16) ((((BMM050_S16)
				   ((signed char) a_data_u8r[3]))
				  << SHIFT_LEFT_5_POSITION)
				 | a_data_u8r[2]);

		a_data_u8r[4] = BMM050_GET_BITSLICE(a_data_u8r[4],
					BMM050_DATAZ_LSB_VALUEZ);
		mdata->dataz = (BMM050_S16) ((((BMM050_S16)
				   ((signed char) a_data_u8r[5]))
				  << SHIFT_LEFT_7_POSITION)
				 | a_data_u8r[4]);

		a_data_u8r[6] = BMM050_GET_BITSLICE(a_data_u8r[6],
				BMM050_R_LSB_VALUE);
		mdata->resistance = (BMM050_U16) ((((BMM050_U16)
				a_data_u8r[7]) <<
				SHIFT_LEFT_6_POSITION) | a_data_u8r[6]);
	}
	return comres;
}

/* In this function X and Y axis is remapped,
 * this API is only applicable for BMX055*/
BMM050_RETURN_FUNCTION_TYPE bmm050_get_bmx055_remapped_raw_xyz \
	(struct bmm050_remapped_mdata *mdata)
{
	BMM050_RETURN_FUNCTION_TYPE comres;
	unsigned char a_data_u8r[8];
	if (p_bmm050 == BMM050_NULL) {
		return E_BMM050_NULL_PTR;
	}
	else {
		comres = p_bmm050->BMM050_BUS_READ_FUNC(p_bmm050->dev_addr,	\
												BMM050_BMX055_REMAPPED_DATAY_LSB, a_data_u8r, 8);

		a_data_u8r[0] = BMM050_GET_BITSLICE(a_data_u8r[0],
											BMM050_BMX055_REMAPPED_DATAY_LSB_VALUEY);
		mdata->datay = (BMM050_S16) ((((BMM050_S16)
									   ((signed char) a_data_u8r[1]))
									  << SHIFT_LEFT_5_POSITION)
									 | a_data_u8r[0]);

		a_data_u8r[2] = BMM050_GET_BITSLICE(a_data_u8r[2],
											BMM050_BMX055_REMAPPED_DATAX_LSB_VALUEX);
		mdata->datax = (BMM050_S16) ((((BMM050_S16)
									   ((signed char) a_data_u8r[3]))
									  << SHIFT_LEFT_5_POSITION)
									 | a_data_u8r[2]);
		mdata->datax = -mdata->datax;

		a_data_u8r[4] = BMM050_GET_BITSLICE(a_data_u8r[4],
											BMM050_DATAZ_LSB_VALUEZ);
		mdata->dataz = (BMM050_S16) ((((BMM050_S16)
									   ((signed char) a_data_u8r[5]))
									  << SHIFT_LEFT_7_POSITION)
									 | a_data_u8r[4]);

		a_data_u8r[6] = BMM050_GET_BITSLICE(a_data_u8r[6],
											BMM050_R_LSB_VALUE);
		mdata->resistance = (BMM050_U16) ((((BMM050_U16)
											a_data_u8r[7]) <<
										   SHIFT_LEFT_6_POSITION) | a_data_u8r[6]);
	}
	return comres;
}

BMM050_RETURN_FUNCTION_TYPE bmm050_set_mag_drdy_interrupt \
	(unsigned char enable_disable,
	unsigned char active_low0_high1)
{
	BMM050_RETURN_FUNCTION_TYPE comres = 0;
	unsigned char v_data1_u8r;
	if (p_bmm050 == BMM050_NULL) {
		return E_BMM050_NULL_PTR;
	}
	else {
		p_bmm050->delay_msec(50);

		comres = p_bmm050->BMM050_BUS_READ_FUNC(
			p_bmm050->dev_addr,
			BMM050_SENS_CNTL_DRDY_EN__REG,
			&v_data1_u8r, 1);

		v_data1_u8r = BMM050_SET_BITSLICE(v_data1_u8r,
										  BMM050_SENS_CNTL_CHANNELX,
										  (enable_disable ? 0 : 1));

		v_data1_u8r = BMM050_SET_BITSLICE(v_data1_u8r,
										  BMM050_SENS_CNTL_CHANNELY,
										  (enable_disable ? 0 : 1));

		v_data1_u8r = BMM050_SET_BITSLICE(v_data1_u8r,
										  BMM050_SENS_CNTL_CHANNELZ,
										  (enable_disable ? 0 : 1));

		v_data1_u8r = BMM050_SET_BITSLICE(v_data1_u8r,
										  BMM050_SENS_CNTL_DR_POLARITY,
										  (active_low0_high1 ? 1 : 0));
		v_data1_u8r = BMM050_SET_BITSLICE(v_data1_u8r,
										  BMM050_SENS_CNTL_DRDY_EN,
										  (enable_disable ? 1 : 0));

		comres |= p_bmm050->BMM050_BUS_WRITE_FUNC(
			p_bmm050->dev_addr,
			BMM050_SENS_CNTL_DRDY_EN__REG,
			&v_data1_u8r, 1);
	}
	return comres;
}
