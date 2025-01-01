#define ADC_QUANTITY_MEASURE 64

#include <avr/io.h>
#include <avr/interrupt.h>

//For averaging measurements.
uint16_t adcMeasure = 0; //Sum adc for quantity measurement. 255 * 64 measure interval = 16 320 it less then max value for uint16_t.
uint8_t adcMeasureCounter = 0; //Count adc measure.
uint8_t adcValue = 0; //Current adc value;

uint8_t ledOn = 0; //Flag light LED.
uint32_t softTimer1Value = 0;

ISR(ADC_vect)
{
	if(adcMeasureCounter < ADC_QUANTITY_MEASURE)
	{
		adcMeasure+= ADCH;
		adcMeasureCounter++;
	}
	else
	{
		adcValue = adcMeasure / ADC_QUANTITY_MEASURE;
		adcMeasureCounter = 0;
		adcMeasure = 0;
		adcMeasure+= ADCH;
	}
}

uint8_t delay_cycles(uint32_t cycles){
   if(softTimer1Value < cycles)
   {
	 softTimer1Value++;
	 return 0;
   }
   else
   {
	 softTimer1Value = 0;
	 return 1;
   }
}

int main(void)
{
	DDRB = 0b00010000 ; //LED connect to PB4(PIN3).
	
	DIDR0 = 0b00000100; //Disabled digital input.
	ADMUX = 0b00100011;//External reference(VCC used as analog reference.), left alignment(AdLAR=1) ADC input ADC3 - PB3, Pin2
	ADCSRA = 0b11101010; //Enable adc, run measure,ADC Auto Trigger Enable,128êãö/4=clock
		
	sei();
	
	while (1)
	{
		if(delay_cycles(adcValue) == 0) continue;
		if(ledOn == 0)
		{
			PORTB|=  0b00010000;
			ledOn = 1;
		}
		else
		{
			PORTB&=  0b11101111;
			ledOn = 0;
		}
	}
}