/*
 * adc_scanner.h
 *
 *  Created on: 27 Oct 2022
 *      Author: Calvert
 */
#ifndef ADC_SCANNER_H_
#define ADC_SCANNER_H_
#include "database.h"

//struct data binaryBarCodeArr[44];

/* ADC scanner init*/
int adcScannerInit(void);
void TA3_0_IRQHandler(void);
/* Interrupt handler*/
void ADC14_IRQHandler(void);
void GetMyChar(void);
void TriggerIRTimerInput(void);
void TimeArrayClear();

#endif /* ADC_SCANNER_H_ */
