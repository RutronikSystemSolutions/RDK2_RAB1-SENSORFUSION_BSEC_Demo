/******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for the RDK2_RAB1_BSEC_Demo
*              for ModusToolbox.
*
* Related Document: See README.md
*
*
*  Created on: 2022-06-29
*  Company: Rutronik Elektronische Bauelemente GmbH
*  Address: Jonavos g. 30, Kaunas 44262, Lithuania
*  Author: GDR
*
*******************************************************************************
* (c) 2019-2021, Cypress Semiconductor Corporation. All rights reserved.
*******************************************************************************
* This software, including source code, documentation and related materials
* ("Software"), is owned by Cypress Semiconductor Corporation or one of its
* subsidiaries ("Cypress") and is protected by and subject to worldwide patent
* protection (United States and foreign), United States copyright laws and
* international treaty provisions. Therefore, you may use this Software only
* as provided in the license agreement accompanying the software package from
* which you obtained this Software ("EULA").
*
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software source
* code solely for use in connection with Cypress's integrated circuit products.
* Any reproduction, modification, translation, compilation, or representation
* of this Software except as specified above is prohibited without the express
* written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer of such
* system or application assumes all risk of such use and in doing so agrees to
* indemnify Cypress against all liability.
*
* Rutronik Elektronische Bauelemente GmbH Disclaimer: The evaluation board
* including the software is for testing purposes only and,
* because it has limited functions and limited resilience, is not suitable
* for permanent use under real conditions. If the evaluation board is
* nevertheless used under real conditions, this is done at one’s responsibility;
* any liability of Rutronik is insofar excluded
*******************************************************************************/

#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"
#include "cy_em_eeprom.h"
#include "bsec.h"
#include "bsec_serialized_configurations_selectivity.h"

/* The size of data to store in EEPROM. Note flash size used will be
 * closest multiple of flash row size */
#define DATA_SIZE                       (BSEC_MAX_STATE_BLOB_SIZE)
/* The Simple Mode is turned off */
#define SIMPLE_MODE                     (0u)
/* Increases the flash endurance twice */
#define WEAR_LEVELING                   (8u)
/* The Redundant Copy is turned off */
#define REDUNDANT_COPY                  (0u)
/* The Blocking Write is turned on */
#define BLOCKING_WRITE                  (1u)

/*Em_EEPROM Location in the auxiliary flash*/
#define EM_EEPROM_ADDR					0x14000000
cy_stc_eeprom_context_t eepromContext;
cy_stc_eeprom_config_t eepromConfig =
{
    .eepromSize = DATA_SIZE,
    .simpleMode = SIMPLE_MODE,
    .wearLevelingFactor = WEAR_LEVELING,
    .redundantCopy = REDUNDANT_COPY,
    .blockingWrite = 1u,
    .userFlashStartAddr = (uint32_t)EM_EEPROM_ADDR,
};

#define STATE_SAVE_PERIOD	UINT32_C(360 * 60 * 1000) // 360 minutes - 4 times a day
// Helper functions declarations
void bsecCallback(const bme68x_data &input, const BsecOutput &outputs);
void checkBsecStatus(Bsec &bsec);
void updateBsecState(Bsec &bsec);
bool loadState(Bsec &bsec);
bool saveState(Bsec &bsec);
void errLeds(void);

// Create an object of the class Bsec
Bsec bsecInst(bsecCallback);

int main(void)
{
    cy_rslt_t result;

    /* Initialize the device and board peripherals */
    result = cybsp_init() ;
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    __enable_irq();

    /*Initialize Emulated EEPROM*/
    if (CY_EM_EEPROM_SUCCESS != Cy_Em_EEPROM_Init(&eepromConfig, &eepromContext))
    {
    	CY_ASSERT(0);
    }

    /*Initialize LEDs*/
    result = cyhal_gpio_init( LED1, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);
    if (result != CY_RSLT_SUCCESS)
    {CY_ASSERT(0);}
    result = cyhal_gpio_init( LED2, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);
    if (result != CY_RSLT_SUCCESS)
    {CY_ASSERT(0);}

    /*Enable debug output via KitProg UART*/
    result = cy_retarget_io_init( KITPROG_TX, KITPROG_RX, CY_RETARGET_IO_BAUDRATE);
    if (result != CY_RSLT_SUCCESS)
    {
    	CY_ASSERT(0);
    }
    printf("\x1b[2J\x1b[;H");

    /*BSEC Setup*/
    bsec_virtual_sensor_t sensorList[] =
    {
      BSEC_OUTPUT_RAW_TEMPERATURE,
      BSEC_OUTPUT_RAW_PRESSURE,
      BSEC_OUTPUT_RAW_HUMIDITY,
      BSEC_OUTPUT_RAW_GAS,
	  BSEC_OUTPUT_RAW_GAS_INDEX,
      BSEC_OUTPUT_IAQ,
	  BSEC_OUTPUT_STATIC_IAQ,
	  BSEC_OUTPUT_CO2_EQUIVALENT,
	  BSEC_OUTPUT_BREATH_VOC_EQUIVALENT,
      BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE,
      BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY,
	  BSEC_OUTPUT_COMPENSATED_GAS,
	  BSEC_OUTPUT_GAS_PERCENTAGE,
      BSEC_OUTPUT_GAS_ESTIMATE_1,
	  BSEC_OUTPUT_GAS_ESTIMATE_2,
	  BSEC_OUTPUT_GAS_ESTIMATE_3,
	  BSEC_OUTPUT_GAS_ESTIMATE_4
    };

    if(!bsecInst.begin(BME68X_I2C_ADDR_LOW, Wire)  		||
       !bsecInst.setConfig(bsec_config_selectivity)     ||
       !loadState(bsecInst)                             ||
       !bsecInst.updateSubscription(sensorList, ARRAY_LEN(sensorList), BSEC_SAMPLE_RATE_HIGH_PERFORMANCE))
    {
    	checkBsecStatus(bsecInst);
    }

    printf("\n\rBSEC library version %d.%d.%d.%d\n\r", bsecInst.getVersion().major, bsecInst.getVersion().minor, bsecInst.getVersion().major_bugfix, bsecInst.getVersion().minor_bugfix);
    CyDelay(10);

    for (;;)
    {
    	  if(!bsecInst.run())
    	  {
    		  checkBsecStatus(bsecInst);
    	  }

    	  cyhal_gpio_toggle(LED1);
    }
}

void errLeds(void)
{
	cyhal_gpio_write(LED1, true);
	cyhal_gpio_write(LED2, false);
	CyDelay(100);
	cyhal_gpio_write(LED2, true);
	CyDelay(100);
}

void checkBsecStatus(Bsec &bsec)
{
  int bme68x_status = (int)bsec.getBme68xStatus();
  int bsec_status = (int)bsec.getBsecStatus();

  if (bsec_status < BSEC_OK)
  {
	  printf("BSEC error code : %d\n\r", bsec_status);
      for (;;)
      {
    	  errLeds(); /* Halt in case of failure */
      }
  }
  else if (bsec_status > BSEC_OK)
  {
	  printf("BSEC warning code : %d\n\r", bsec_status);
  }

  if (bme68x_status < BME68X_OK)
  {
	  printf("BME68X error code : %d\n\r", bme68x_status);
      for (;;)
      {
    	  errLeds(); /* Halt in case of failure */
      }
  }
  else if (bme68x_status > BME68X_OK)
  {
	  printf("BME68X warning code : %d\n\r", bme68x_status);
  }
}

void bsecCallback(const bme68x_data &input, const BsecOutput &outputs)
{
  if (!outputs.len)
    return;

  printf("BSEC outputs:\n\r timestamp = %d\n\r", (int)(outputs.outputs[0].time_stamp / INT64_C(1000000)));
  for (uint8_t i = 0; i < outputs.len; i++)
  {
    const bsec_output_t &output = outputs.outputs[i];
    switch (output.sensor_id)
    {
      case BSEC_OUTPUT_IAQ:
    	  printf(" iaq =  %.2f acc %d\n\r", output.signal, output.accuracy);
        break;
      case BSEC_OUTPUT_STATIC_IAQ:
    	  printf(" static iaq =  %.2f\n\r", output.signal);
        break;
      case BSEC_OUTPUT_CO2_EQUIVALENT:
    	  printf(" co2 equivalent =  %.2f\n\r", output.signal);
        break;
      case BSEC_OUTPUT_BREATH_VOC_EQUIVALENT:
    	  printf(" breath voc equivalent =  %.2f\n\r", output.signal);
        break;
      case BSEC_OUTPUT_RAW_TEMPERATURE:
    	  printf(" temperature =  %.2f\n\r", output.signal);
        break;
      case BSEC_OUTPUT_RAW_PRESSURE:
    	  printf(" pressure =  %.2f\n\r", output.signal);
        break;
      case BSEC_OUTPUT_RAW_HUMIDITY:
    	  printf(" humidity =  %.2f\n\r", output.signal);
        break;
      case BSEC_OUTPUT_RAW_GAS:
    	  printf(" gas resistance =  %.2f\n\r", output.signal);
        break;
	  case BSEC_OUTPUT_RAW_GAS_INDEX:
		  printf(" gas index =  %.2f\n\r", output.signal);
        break;
      case BSEC_OUTPUT_STABILIZATION_STATUS:
    	  printf(" stabilization status =  %.2f\n\r", output.signal);
        break;
      case BSEC_OUTPUT_RUN_IN_STATUS:
    	  printf(" run in status =  %.2f\n\r", output.signal);
        break;
      case BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE:
    	  printf(" compensated temperature =  %.2f\n\r", output.signal);
        break;
      case BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY:
    	  printf(" compensated humidity =  %.2f\n\r", output.signal);
        break;
      case BSEC_OUTPUT_COMPENSATED_GAS:
    	  printf(" compensated gas =  %.2f\n\r", output.signal);
        break;
      case BSEC_OUTPUT_GAS_PERCENTAGE:
    	  printf(" compensated gas percentage =  %.2f\n\r", output.signal);
        break;
      case BSEC_OUTPUT_GAS_ESTIMATE_1:
      case BSEC_OUTPUT_GAS_ESTIMATE_2:
      case BSEC_OUTPUT_GAS_ESTIMATE_3:
      case BSEC_OUTPUT_GAS_ESTIMATE_4:
    	  printf(" gas estimate  %d = %.2f\n\r", ((int)(output.sensor_id + 1 - BSEC_OUTPUT_GAS_ESTIMATE_1)), output.signal);
        break;
      default:
        break;
    }
  }
  printf("------------------------------------------\n\r");

  updateBsecState(bsecInst);
}

void updateBsecState(Bsec &bsec)
{
  static uint16_t stateUpdateCounter = 0;
	bool update = false;

	if (stateUpdateCounter == 0)
	{
		const bsec_output_t* iaq = bsec.getOutput(BSEC_OUTPUT_IAQ);
		/* First state update when IAQ accuracy is >= 3 */
		if (iaq && iaq->accuracy >= 3)
		{
			update = true;
			stateUpdateCounter++;
		}
	}
	else if ((stateUpdateCounter * STATE_SAVE_PERIOD) < bsec.getTimeMs())
	{
		/* Update every STATE_SAVE_PERIOD minutes */
		update = true;
		stateUpdateCounter++;
	}

	if (update && !saveState(bsec))
	{
		checkBsecStatus(bsec);
	}
}

bool loadState(Bsec &bsec)
{
	  uint8_t bsecState[BSEC_MAX_STATE_BLOB_SIZE];
	  uint8_t data = 0;

	  Cy_Em_EEPROM_Read(0u, &data, 1u, &eepromContext);
	  if (data == BSEC_MAX_STATE_BLOB_SIZE)
	  {
	    // Existing state in EEPROM
		  printf("Reading state from EEPROM:\n\r");

	    for (uint8_t i = 0; i < BSEC_MAX_STATE_BLOB_SIZE; i++)
	    {
	    	Cy_Em_EEPROM_Read((i + 1), &data, 1u, &eepromContext);
	      bsecState[i] = data;
	      printf(" 0x%X", bsecState[i]);
	    }
	    printf("\n\r");

	    if(!bsec.setState(bsecState))
	      return false;
	  }
	  else
	  {
	    // Erase the EEPROM with zeroes
		  printf("Erasing EEPROM\n\r");

	    for (uint8_t i = 0; i <= BSEC_MAX_STATE_BLOB_SIZE; i++)
	    {
	    	data = 0;
	    	Cy_Em_EEPROM_Write(i, &data, 1u, &eepromContext);
	    }
	  }
	  return true;
}

bool saveState(Bsec &bsec)
{
	  uint8_t bsecState[BSEC_MAX_STATE_BLOB_SIZE];
	  uint8_t data = BSEC_MAX_STATE_BLOB_SIZE;

	    if (!bsec.getState(bsecState))
	    return false;

	    printf("Writing state to EEPROM:\n\r");

	  for (uint8_t i = 0; i < BSEC_MAX_STATE_BLOB_SIZE; i++)
	  {
		Cy_Em_EEPROM_Write((i + 1), &bsecState[i], 1u, &eepromContext);
		printf(" 0x%X", bsecState[i]);
	  }
	  printf("\n\r");

	  Cy_Em_EEPROM_Write(0, &data, 1u, &eepromContext);

	  return true;
}

/* [] END OF FILE */
