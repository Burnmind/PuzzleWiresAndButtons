#include "Debounce.h"

Debounce::Debounce(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, uint32_t filterTime) {
	_GPIOx = GPIOx;
	_GPIO_Pin = GPIO_Pin;
	_filterTime = filterTime;
	_filterTimeCount = _filterTime /2;
	flagLow = 0;
	flagRising = 1;
	flagFalling = 0;

	// разрешение тактирования порта
	if( _GPIOx == GPIOA ) RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
	else if( _GPIOx == GPIOB ) RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
	else if( _GPIOx == GPIOC ) RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;
	else if( _GPIOx == GPIOD ) RCC->APB2ENR |= RCC_APB2ENR_IOPDEN;
	else if( _GPIOx == GPIOE ) RCC->APB2ENR |= RCC_APB2ENR_IOPEEN;

	// устанавливаем конфигурацию вывода на вход с подтягивающим резистором
	uint32_t i=0;
	uint16_t d = _GPIO_Pin;
	while(i < 16) {
		d = d >> 1;
		if(d == 0) break;
		i++;
	}

	if(i < 8) {
		_GPIOx->CRL |= 0b1000 << (i * 4);
		_GPIOx->CRL &= ~ (0b0111 << (i * 4));
	}
	else {
		_GPIOx->CRH |= 0b1000 << ((i - 8) * 4);
		_GPIOx->CRH &= ~ (0b0111 << ((i - 8) * 4));
	}

	_GPIOx->ODR |= 1 << i;
}

// метод ожидания стабильного состояния сигнала
// flagLow; // признак СИГНАЛ В НИЗКОМ УРОВНЕ
// flagRising; // признак БЫЛ ПОЛОЖИТЕЛЬНЫЙ ФРОНТ
// flagFalling; // признак БЫЛ ОТРИЦАТЕЛЬНЫЙ ФРОНТ
void Debounce::scanStability(void) {

	if( (flagLow == 0) == ((_GPIOx->IDR & _GPIO_Pin) != 0)) {
		// если состояние вывода не изменилось, то счетчик = 0
		_filterTimeCount = 0;
	} else {
	    // состояние вывода изменилось
		_filterTimeCount++;

		if( _filterTimeCount >= _filterTime ) {
			// состояние кнопки не мянялось в течение заданного времени
		    // состояние кнопки стало устойчивым
			flagLow = (~ flagLow) & 1; // инверсия признака состояния
			_filterTimeCount = 0;  // сброс счетчика времени

	        if ( flagLow != 0 ) {
	        	flagFalling = 1; // признак фронта кнопки на нажатие
	        } else {
	        	flagRising = 1; // признак фронта кнопки на отжатие
	        }
		}
	}
}

// метод фильтрации сигнала по среднему значению
// flagLow; // признак СИГНАЛ В НИЗКОМ УРОВНЕ
// flagRising; // признак БЫЛ ПОЛОЖИТЕЛЬНЫЙ ФРОНТ
// flagFalling; // признак БЫЛ ОТРИЦАТЕЛЬНЫЙ ФРОНТ
void Debounce::scanAverage(void) {

	if( (flagLow == 0) == ((_GPIOx->IDR & _GPIO_Pin) != 0)) {
		// состояние сигнала осталось прежним
		if ( _filterTimeCount != 0 ) _filterTimeCount--; // счетчик времени - 1 с ограничением на 0
	} else {
		// состояние сигнала изменилось
		_filterTimeCount++;   // +1 к счетчику времени

		if ( _filterTimeCount >= _filterTime ) {
			// порог
			flagLow = (~ flagLow) & 1; // инверсия признака состояния
			_filterTimeCount = 0;  // сброс счетчика времени

	        if ( flagLow != 0 ) {
	        	flagFalling = 1; // признак фронта кнопки на нажатие
	        } else {
	        	flagRising = 1; // признак фронта кнопки на отжатие
	        }
		}
	}
}

// метод установки времени фильтрации
void Debounce::setTime(uint32_t filterTime) {
	_filterTime = filterTime;
}

// чтение признака СИГНАЛ В НИЗКОМ УРОВНЕ
uint8_t Debounce::readFlagLow(void) {
	return(flagLow);
}

// чтение признака БЫЛ ПОЛОЖИТЕЛЬНЫЙ ФРОНТ
uint8_t Debounce::readFlagRising(void) {
	if(flagRising != 0) {
		flagRising = 0;
		return(1);
	}
	else return(0);
}

// чтение признака БЫЛ ОТРИЦАТЕЛЬНЫЙ ФРОНТ
uint8_t Debounce::readFlagFalling(void) {
	if(flagFalling != 0) {
		flagFalling = 0;
		return(1);
	}
	else return(0);
}
