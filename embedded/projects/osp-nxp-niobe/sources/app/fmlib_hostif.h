/*
 * androidHostinterface.h
 *
 *  Created on: Mar 8, 2013
 *      Author: sungerfeld
 */

#ifndef ANDROIDHOSTINTERFACE_H_
#define ANDROIDHOSTINTERFACE_H_

#include <stdint.h>
//#include <timers.h>

#include "spi-sensor-hub-priv.h"

#ifndef PC_EMULATOR
#define SLAVE_NUM_TX_BUFFERS 25
#else
#define SLAVE_NUM_TX_BUFFERS 107  //for PC emulator
#endif

//extern uint64_t sensorEnable;      // bit map of (1 << sensorId) to show if sensor is enabled (1) or disabled (0)
uint8_t isSensorEnable(uint8_t sensorId);
//uint8_t areSensorEnable(uint64_t mask);


//extern uint16_t sensorDelay[SPI_SH_SENSOR_ID_COUNT];
uint16_t getSensorDelay(uint8_t sensorId);


extern uint16_t commited_length;

extern uint16_t broadcast_buf_wr;
extern uint16_t broadcast_buf_rd;

extern uint8_t broadcast_buf[SLAVE_NUM_TX_BUFFERS][SPI_SH_MAX_BROADCAST_BUFFER_SIZE];
extern uint8_t broadcast_buf_flags[SLAVE_NUM_TX_BUFFERS];
extern uint16_t broadcast_buf_used[SLAVE_NUM_TX_BUFFERS];
extern uint16_t last_transmitted_offset[SLAVE_NUM_TX_BUFFERS];




extern uint16_t timeStampExpansion;
extern uint8_t androidBroadcastTrigger;



uint8_t getLastCommand(void);


void init_android_broadcast_buffers(void);

typedef enum {
	COMMAND_PROCESS_INVALID = 0,
	COMMAND_PROCESS_GET,
	COMMAND_PROCESS_SET
} ProcessCommandType;	

uint8_t  process_command(uint8_t *rx_buf, uint16_t length);
ProcessCommandType post_on_broadcast_buffer(uint8_t *bigBuffer, uint16_t bigBufferLength);

void hostCommitDataTx(void);
void configureTimeCaptureTimers (void);
void configureSensorsTimeCapture (void);
void getTimeCapture(enum SPI_SH_SENSOR_ID sensorId, struct TimeStamp40 *result);
void calculate_commited_tx_buffer_size(void);

#endif /* ANDROIDHOSTINTERFACE_H_ */
