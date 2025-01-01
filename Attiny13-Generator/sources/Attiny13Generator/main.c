//#define F_CPU 4800000UL

#define START_PULSE_LENGTH 200; //Default on mcu start.
#define MIN_PULSE_LENGTH 2
#define MAX_PULSE_LENGTH 400000 //2Hz
#define PULSE_LENGTH_STEP 20

#include <Util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>

uint8_t timeCounter1 = 0; //Software counter for button.
uint32_t pulseLength = 0; //Setup pulse length.
uint32_t pulseLengthCounter = 0; //Pulse length counter.
uint8_t pulseLevel = 0; //Level on output 1/0.

//Timer/Counter0 Overflow
ISR(TIM0_OVF_vect){

	//Software counter.
	timeCounter1 ++;
	if(timeCounter1 >= 5)
	{
		timeCounter1 = 0; //Clear software counter.
		
			uint8_t btnDown = PINB & 0b00010000;
			if(btnDown == 0 & pulseLength <= MAX_PULSE_LENGTH) //Push this button.
			{
				pulseLength+= PULSE_LENGTH_STEP;
			}

			uint8_t btnUp = PINB & 0b00001000;
			if(btnUp == 0 & pulseLength >= MIN_PULSE_LENGTH)
			{
				pulseLength-= PULSE_LENGTH_STEP;
			}	
	}	
}

//Change level on output.
void changeLevelOnOutput()
{
	if(pulseLevel == 1)
	{
		PORTB&= 0b11111110; //0 level.
		pulseLevel = 0;
	}
	else
	{
		pulseLevel = 1;
		PORTB|= 0x01; //1 level.
	}
}

int main(void)
{	 
	TCCR0B = 0b00000100; //Mcu clock/256
	TIMSK0 = 0b00000010; //Allow interrupt for tc0.	

	DDRB = 1; //Setup output on PB0.
	PORTB = 0x18; //Internal resistors for button to plus line.
	
	sei(); //Allow all interupts.
	
	changeLevelOnOutput(); //Setup logic 1.
	pulseLength = START_PULSE_LENGTH;
    while (1) 
    {
		pulseLengthCounter++; //Count pulse length.
		
		if(pulseLengthCounter > pulseLength)
		{
			pulseLengthCounter = 0;
			changeLevelOnOutput();
		}
    }
}

