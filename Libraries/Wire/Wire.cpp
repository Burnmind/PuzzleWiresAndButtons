
#include "Wire.h"

uint8_t PORT_ID_1 = '7';
uint8_t PORT_ID_2 = '8';
uint8_t PORT_ID_3 = '9';

Wire::Wire(uint8_t id) {
	changed = 0;
	inited = 0;
	dataReceived = 0;
	necessaryPortSetted = 0;
	dataTransmitted = 1;
	idCounter = 0;
	tempId = 0;
}

uint8_t Wire::isInited(void) {
	if (inited != 0) {
		return(1);
	}

	return(0);
}

uint8_t Wire::portIsChanged(void) {
	if (changed != 0) {
		changed = 0;
		return(1);
	}

	return(0);
}

uint8_t Wire::hasMistake(void) {
	if (currentPort != necessaryPort) {
		return(1);
	}

	return(0);
}

uint8_t Wire::isRightConnection(void) {
	if (currentPort == necessaryPort) {
		return(1);
	}

	return(0);
}

uint8_t Wire::necessaryPortIsSet(void) {
	if (necessaryPortSetted != 0) {
		return(1);
	}

	return(0);
}

void Wire::setPort(uint8_t portId) {
	if (portId == PORT_ID_1 || portId == PORT_ID_2 || portId == PORT_ID_3) {
		if (idCounter > 10) {
			idCounter = 0;

			if (inited == 0) {
				inited = 1;
			} else {
				if (currentPort != portId) {
					changed = 1;
				}
			}

			currentPort = portId;
		} else {
			if (tempId == portId) {
				idCounter++;
			} else {
				tempId = portId;
				idCounter = 0;
			}
		}
	}
}

uint8_t Wire::getCurrentPort(void) {
	return currentPort;
}

void Wire::setNecessaryPort(uint8_t portId) {
	necessaryPortSetted = 1;
	necessaryPort = portId;
}
/*
void Wire::rxHandler(void) {
	dataReceived = 1;

	if (dataTransmitted != 0) {
		HAL_UART_Transmit_IT(&uart, &idUart, 1);
		dataReceived = 0;
		dataTransmitted = 0;
	}

	uint8_t receivedUartId;
	HAL_UART_Receive_IT(&uart, &receivedUartId, 1);

	Wire::setPort(receivedUartId);
}

void Wire::txHandler(void) {
	dataTransmitted = 1;

	if (dataReceived != 0) {
		HAL_UART_Transmit_IT(&uart, &idUart, 1);
		dataReceived = 0;
		dataTransmitted = 0;
	}
}*/
