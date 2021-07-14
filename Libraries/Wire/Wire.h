/*
 * Wire.cpp
 *
 * Контроллер провода в головоломке
 *
 *      Author: Burnmind
 */


#ifndef WIRE_WIRE_H_
#define WIRE_WIRE_H_

#include "stm32f1xx_hal.h"

class Wire {
	public:
		Wire(uint8_t id);
		uint8_t isInited(void);
		uint8_t portIsChanged(void);
		uint8_t necessaryPortIsSet(void);
		uint8_t hasMistake(void);
		void setPort(uint8_t portId);
		uint8_t getCurrentPort(void);
		uint8_t isRightConnection(void);
		void setNecessaryPort(uint8_t portId);
		/*void rxHandler();
		void txHandler();*/

		UART_HandleTypeDef uart;
	private:
		uint8_t idUart;
		uint8_t dataReceived;
		uint8_t dataTransmitted;
		uint8_t currentPort;
		uint8_t necessaryPort;
		uint8_t changed;
		uint8_t inited;
		uint8_t necessaryPortSetted;
		uint8_t tempId;
		uint32_t idCounter;
};

#endif /* WIRE_WIRE_H_ */
