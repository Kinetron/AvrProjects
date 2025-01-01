/*
 * PumpHysteresis.c
 *
 * Created: 24.12.2024 20:18:15
 * Author : Kinetron
 */ 

//Internal 128 kHz Oscillator /8
#define F_CPU 160000UL
#define HYSTERESIS_TIME 7 //7 seconds

#include <avr/io.h>
#include <util/delay.h>
#include <avr/wdt.h>

uint8_t ledOn = 0; //Flag light LED.
uint8_t pumpTimer = 0;

void ledBlink()
{
	 if(ledOn == 0)
	 {
		 PORTB |= 0b00001000;
		 ledOn = 1;
	 }
	 else
	 {
		 PORTB&= 0b11110111;
		 ledOn = 0;
	 }	
}

int main(void)
{
   //PB4(PIN3) - pressure sensor. 
   //PB2(PIN7) - emergency shutdown pump.
   DDRB = 0b00001010; //LED connect to PB3(PIN2). Pump on - PB1(PIN6) - inverted.
   PORTB|= 0b00001010; //Off PUMP
   
   wdt_enable(WDTO_4S); //Watchdog 4 second.
     
   while (1) 
   {
	 wdt_reset(); //We're alive and we're not hanging.
	 
	 //Just blink led and wait 1 second.
	 _delay_ms(1000);
	 ledBlink();
		 	 
	 //PB2(PIN7) - emergency shutdown pump. Water level is zero.
	 if( (PINB & 0b00000100) == 0b00000100)
	 {
		 PORTB|= 0b00000010; //Off PUMP
		 continue;
	 }
	 	 
	 //Check pressure sensor. PB4
	 if( (PINB & 0b00010000) == 0b00010000)
	 {		 
		PORTB&= 0b11111101; // On pump.
		pumpTimer = 0;
	 }
	 else
	 {	
		//Need off pump.
		if(pumpTimer < HYSTERESIS_TIME)
		{
			pumpTimer ++;
			continue;
		}
		
		PORTB|= 0b00000010; //Off pump.
	 }	 
   }
}

