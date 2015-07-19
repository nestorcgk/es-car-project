#include <car.h>

/********************************************************************************
 * Setup
 ********************************************************************************/

void setupIO(void)
{
	BUMPER_DDR = 0x00; 											//set bumper direction register as input 
	LEDS_DDR = (1 << PC0) | (1 << PC1);     //set PC0 and PC1 as output
	STEERING_DDR = (1 << PORTB5);    				//set steering pwm as output
  SWITCHES_DDR = 0x00;                		//set switch as input
	TACHOMETER_DDR = (0 << TACHOMETER_OUT); //set tachometer as input
}

void setupTimer(void)
{
	setupPIDTimer();
	setupHallTimer();
}

void setupPIDTimer(void)
{
	TCCR3B |= (1 << WGM32)|(1 << CS32)|( 1 << CS30);  //Enable CTC (Clear timer on compare)
	OCR3A = 3906;           //Set the timer ticks
	TIMSK3 |= (1 << OCIE3A);
}

void setupHallTimer(void){
  TCCR5B = (1 << CS52)| (1 << CS51)| (1 << CS50); // increase on T5 rising edge
	TCNT5 = 0x00;																		// set timer 5 counter to zero
}

void startPIDTimer(void)
{
	TCCR3B |= (1 << CS31) | (1 << CS30);   //starts timer 64 bit prescaler
	TIMSK3 |= (1 << OCIE3A);               //Enable interrupt
	TCNT3 = 0x00;                          //clear timer
}

void stopPIDTimer(void)
{
	TCCR3B &= ~((1 << CS32) | (1 << CS30));  //Stops the timer
	TIMSK3 = (0 << OCIE3A);                  //Disable interrupt
	TCNT3 = 0x00;                            //Clear timer
}

void setupMotorHbridge(void)
{
	MOTOR_DDR |= (1 << DDH3);                //enable output pin OC4A pin PH3	
	TCCR4A |= (1 << WGM41);                  //fast PWM setting WGM 14
	TCCR4B |= (1 << WGM42) | (1 << WGM43);
	TCCR4A |= (1 << COM4A1);                 //Select non inverting mode 
	ICR4 |= 799;                             //Genertate 20 kHz PWM considering 1 as prescaler
	//drive forward and enable H bridge
	HBRIDGE_DDR = (1 << DDK0) | (1 << DDK1) | (1 << DDK2) | (1 << DDK3); //set hbridge controller as output
	TCCR4B |= (1 << CS40); //set prescaler to 1 (NO prescaling) and start motor PWM
	HBRIDGE_PORT |= (1 << PK0) | (1 << PK1) | (1 << PK2) | (1 << PK3); //enable H bridge and break
	setMotorPower(0);
}

/********************************************************************************
 * Motor
 ********************************************************************************/

//speed value between 0 and 100, note: 100 is 33% of the dutycycle
void setMotorPower(uint8_t power) 
{	//SPEED LIMITER
	//After setting the power you must start the motor!
	if(power > 100)
	{
		power = 100;
	}else{
		if(power < 0)
		{
			power = 0;
		}
	}
  
	OCR4A = (power/100.0)*799*SPEED_LIMIT_DC; //maps speed variable into [0,264] with max dutycycle 33
}

void startMotor(void)
{
	HBRIDGE_PORT |= (1 << PORTK0); //enable CW rotation
	HBRIDGE_PORT &= ~(1 << PORTK1);
}

void motorBreak(void)
{
	HBRIDGE_PORT |= (1 << PK1);	//break to vcc
	HBRIDGE_PORT |= (1 << PK0);
}

/********************************************************************************
 * Steering
 ********************************************************************************/

void setSteering(void)
{
	DDRB |= (1 << DDB5); //enable output
	/* Configure timer1 */
	TCCR1A |= (1<<COM1A1)|(1<<COM1B1)|(1<<WGM11);	         // NON Inverted PWM
	TCCR1B |= (1<<WGM13)|(1<<WGM12)|(1<<CS11)|(1<<CS10);	 // prescaler N = 64 and mode 14 (fast PWM)
	// uint16_t f_pwm = 250; // f_PWM = 250Hz (Period = 4 ms)
	ICR1 = 999; //(F_CPU / (f_pwm * 1)) - 1;	// 999 for F_CPU = 16000000 and f_pwm = 250
}

void startSteering(int8_t angle)
{
	//setup position (for the angle from -45° to 45°) aka duty cycle for steering
	uint16_t lower_limit = 1150;	// 1150 mcs = 1,15 ms for -45° angle
	uint16_t upper_limit = 1850;	// 1850 mcs = 1,85 ms for 45° angle 
	OCR1A = (((angle + 45) * (upper_limit - lower_limit) / 90) + lower_limit) / 4; // 4 is from 1/(F_CPU/N) [mcs]
}

void stopSteering(void)
{
	TCCR1A &= ~((1<<COM1A1)|(1<<COM1B1)|(1<<WGM11)); // stop timer 1
	TCCR1B &= ~((1<<WGM13)|(1<<WGM12)|(1<<CS11)|(1<<CS10));
}

void addCornerDistance()
{
	if((cornerCounter < AI_ARRAY_LENGTH)&&(cornerFlag == 0))
	{
		cornerDistances[cornerCounter] = rotationsAmount;
		cornerFlag = 1;
		cornerCounter++;
	}	
}

double multiplyerCalculation(void)
{
	double multiplyer = 1;
	if(lapCounter > 1)
	{
		uint8_t pos = 0;
		while((cornerDistances[pos] < rotationsAmount) && (cornerDistances[pos] != 0xffff) && (pos < AI_ARRAY_LENGTH))
		{
			pos++;
		}
		
		if(cornerDistances[pos] - rotationsAmount > AI_DELTA)
		{
			multiplyer += (cornerDistances[pos] - rotationsAmount - AI_DELTA) * AI_COEFFICIENT;
			if(multiplyer > AI_MULTIPLYER_LIMIT) multiplyer = AI_MULTIPLYER_LIMIT;
		}
	}
	return multiplyer;
}

void steeringBySensors(uint8_t previous_sensors_values, uint8_t current_sensors_values)
{
	setSteering();
	current_sensors_values = getSensorsValues();

	if (current_sensors_values == 0x00)
	{
		current_sensors_values = previous_sensors_values;
	}

	uint8_t n = bitCount(current_sensors_values);

	countLaps(n);
	
	if (n > 1)
	{
		startSteering(0);
	}

	double speed_multiplyer = multiplyerCalculation();

	switch (current_sensors_values) 
	{
		case 0b00000001:
			startSteering(-45);
			changeSpeed((1-5*CORNER_SPEED_LIMIT)*car_speed);
			addCornerDistance();
			break;
		case 0b00000010:
			startSteering(-30);
			changeSpeed((1-4.5*CORNER_SPEED_LIMIT)*car_speed);
			addCornerDistance();
			break;
		case 0b00000100:
			startSteering(-15);
			changeSpeed((1-4*CORNER_SPEED_LIMIT)*car_speed);
			addCornerDistance();
			break;
		case 0b00001000:
			startSteering(-5);
			changeSpeed(car_speed*speed_multiplyer);
			break;
		case 0b00010000:
			startSteering(5);
			changeSpeed(car_speed*speed_multiplyer);
			break;
		case 0b00100000:
			startSteering(15);
			changeSpeed((1-4*CORNER_SPEED_LIMIT)*car_speed);
			addCornerDistance();
			break;
		case 0b01000000:
			startSteering(30);
			changeSpeed((1-4.5*CORNER_SPEED_LIMIT)*car_speed);
			addCornerDistance();
			break;
		case 0b10000000:
			startSteering(45);
			changeSpeed((1-5*CORNER_SPEED_LIMIT)*car_speed);
			addCornerDistance();
			break;	
	}
	
	if (current_sensors_values != 0x00)
	{
		previous_sensors_values = current_sensors_values;
	}
}

/********************************************************************************
 * Speed
 ********************************************************************************/

void changeSpeed(uint8_t speed) //sets motor power with PID
{
	if (PID_ENABLE == 0)
	{
		setMotorPower(speed);
	}
	else
	{
		current_speed = speed;		
	}
} 

uint8_t calculateSpeedPID(uint8_t new_speed)
{ //should be called every fixed period 
	double error = new_speed - measured_speed; //proportional Error
	
	//printInt((int) error, 0x03, 1, 10);

	double intErr = error*PID_PERIOD; //integral Error
	double dErr = (error-last_error)/PID_PERIOD; //Derivative Error
	double calculation = (KP*error + KI*intErr + KD*dErr); //total error
	last_error = error;
	int8_t result = calculation+current_speed;

	if (result <= 0) result = 0;
	return result; 
}

/********************************************************************************
 * Sensors
 ********************************************************************************/

void readSpeed(void)
{
	uint8_t tmp_tcnt5 = TCNT5;
	rotationsAmount += tmp_tcnt5;
	if(tmp_tcnt5 > maxSpeed){
		maxSpeed = tmp_tcnt5;
	}

	measured_speed = tmp_tcnt5 * 4;	//read current speed counter
	TCNT5 = 0x00;													//reset timer after reading
}

uint8_t getSensorsValues()
{
	if ((SAMPLE_SIZE > 1)&&(SAMPLE_SIZE < 32))
	{
		uint8_t sensors_status [SAMPLE_SIZE];	// sample array of sensors readings
		uint8_t result = 0;	                  // the resulting configuration of sensors readings

		/* Sample recording */
		for (uint8_t i = 0; i < SAMPLE_SIZE; i++)
		{
			sensors_status [i] = ~BUMPER_PIN;
		}
		
		/* Result calculation */
		for (uint8_t j = 0; j < 8; j++)	// from 0 to 7 sensor
		{
			uint32_t tmp = 0;	// the sample at the bit level

		  for (uint8_t i = 0; i < SAMPLE_SIZE; i++)	// tmp recording
			{
				if (sensors_status [i] & (1<<j))
				{
					  tmp |= 1 << i;
				}
			}

			uint8_t n = 0;	  // the number of 1 bits
			while (tmp != 0)	// counting 1 bits
			{
   				tmp = tmp & (tmp - 1);
   				n++;
			}
			if (n >= LOWER_LIMIT) result |= 1 << j; // comparison with the limit
		}
		return result;
	}
	uint8_t sensors_status = ~BUMPER_PIN;
	return sensors_status;
}

/********************************************************************************
 * Car
 ********************************************************************************/

void startCar()
{
	uint8_t previous_switches_values = 0x00;
	uint8_t switches_values = ~SWITCHES_PIN;

	for (uint8_t i = 0; i < (SAMPLE_SIZE-1); i++)	// SAMPLE_SIZE for function users starts from 1 
	{
		switches_values &= ~SWITCHES_PIN;  
	}

	if((switches_values & (1<<BLACK_SWITCH))&&(previous_switches_values!=switches_values)&&(buttonFlag == 0))
	{
		buttonFlag = 1;
		switch (carState) 
		{
			case 0:
				carState = 1;
				break;
			case 1:
				carState = 0;
				break;
		}
	}
	previous_switches_values = switches_values;
}

void startMovement()
{
	startMotor();
}

void stopMovement()
{
	motorBreak();
}

/********************************************************************************
 * Laps
 ********************************************************************************/

void countLaps(uint8_t n)
{
	if(n > 4 && lapFlag == 0)
	{
		if((cornerCounter + 1) < AI_ARRAY_LENGTH)
		{
			cornerDistances[cornerCounter + 1] = 0xffff;
		}
		lapFlag = 1;
		lapCounter++;
		rotationsAmount = 0;
		cornerCounter = 0;
	}
}

uint8_t bitCount (uint8_t value) {
  uint8_t count = 0;
  while (value > 0) 
  {           // until all bits are zero
    if ((value & 1) == 1)     // check lower bit
   	{
      count++;
    }
    value >>= 1;              // shift bits, removing lower bit
  }
  return count;
}

/********************************************************************************
 * LED
 ********************************************************************************/

// PC0 or PC1
void turnOnLED(uint8_t led){
  LEDS_PORT = (1 << led);
}

void turnOffLED(uint8_t led){
	LEDS_PORT &= ~(1 << led);
}

/********************************************************************************
 * Interrupts
 ********************************************************************************/

ISR(TIMER3_COMPA_vect)
{
	cornerFlag = 0;

	if(counter % 4 == 0)
	{
		buttonFlag = 0;
	}
	
	if(counter % 8 == 0)
	{
		lapFlag = 0;
	} 
	
	counter++;

	if (carState == 1) time++;

	readSpeed();

	if (PID_ENABLE == 1)
	{
		setMotorPower(calculateSpeedPID(current_speed));
	}
}


/********************************************************************************
 *  Display functions
 ********************************************************************************/

// USART initialization
void initUSART(void)
{
	UBRR1H = UBRRH_VALUE;	// defined in setbaud.h
	UBRR1L = UBRRL_VALUE;
	
	UCSR1A &= ~(1 << U2X1);
	
	UCSR1B |= (1 << RXEN1); // RXEN, Enable RX
	// UCSR1B |= (1 << RXCIE1); // RXCIE, Enable RX interrupt - test
	UCSR1B |= (1 << TXEN1); // TXEN, Enable TX
	
	UCSR1C = (1 << UCSZ11) | (1 << UCSZ10);   // 8 data bits, 1 stop bit

}

// sends the wanted byte to the TX line
void sendByteUSART(uint8_t data)
{
	// Wait for empty transmit buffer
	loop_until_bit_is_set(UCSR1A, UDRE1);
	
	// Send the given data
	UDR1 = data;
}

// reads a byte from RX line
uint8_t receiveByteUSART(void)
{
	// Wait until a byte has been received
	loop_until_bit_is_set(UCSR1A, RXC1);

	// Return received data
	return UDR1;
}

uint16_t makeColor(uint8_t red, uint8_t green, uint8_t blue)
{
	uint8_t msb, lsb;
	uint16_t color;
	
	if(0x1F < red) red = 0x1F;
	if(0x3F < green) green = 0x3F;
	if(0x1F < blue) blue = 0x1F;
	
	msb = ((red << 3) & ~0b00000111) | ((green >> 3) & ~0b11111000);
	lsb = ((green << 5) & ~0b00011111) | (blue & ~0b11100000);

	color = (((uint16_t)msb) << 8) | lsb;
	
	return color;
}

uint16_t makeColor2(uint8_t red, uint8_t green, uint8_t blue)
{
	uint16_t color = (((red & 0xf8) << 8) | ((green & 0xfc) << 3) | ((blue & 0xf8) >> 3));;
		
	return color;
}

void prepareDisplay(void)
{
	// Delay, wait for the display to be ready
	_delay_ms(500);
	_delay_ms(500);
}
 
void initLCD(void)
{
	uint8_t c = 0x00;
	
	while (c != 0x06)
	{
		sendByteUSART(0x55);
	
		c = receiveByteUSART();
	}
}

void setBackgroundColor(uint16_t color)
{
	uint8_t c = 0x00;
	
	while (c != 0x06)
	{
		sendByteUSART(0x42);
		sendByteUSART((color >> 8));
		sendByteUSART((uint8_t)color);
		
		c = receiveByteUSART();
	}
}

void eraseScreen()
{
	uint8_t c = 0x00;
	
	while (c != 0x06)
	{
		sendByteUSART(0x45);
		
		c = receiveByteUSART();
	}
}

void setOpaqueText()
{
	uint8_t c = 0x00;
	
	while (c != 0x06)
	{
		sendByteUSART(0x4F);
		sendByteUSART(0x01);
		
		c = receiveByteUSART();
	}
}

void setTransparentText()
{
	uint8_t c = 0x00;
	
	while (c != 0x06)
	{
		sendByteUSART(0x4F);
		sendByteUSART(0x00);
		
		c = receiveByteUSART();
	}
}

void putChar(char s, uint8_t col, uint8_t row, uint16_t color)
{
	uint8_t c = 0x00;
	
	while (c != 0x06)
	{
		sendByteUSART(0x54);
		sendByteUSART(s);
		sendByteUSART(col);
		sendByteUSART(row);
		sendByteUSART((color >> 8));
		sendByteUSART((uint8_t)color);
		
		c = receiveByteUSART();
	}
}

void putString(char *s, uint8_t col, uint8_t row, uint8_t font, uint16_t color)
{
	uint8_t c = 0x00;
	
	while (c != 0x06)
	{
		sendByteUSART(0x73);
		sendByteUSART(col);
		sendByteUSART(row);
		sendByteUSART(font); // 0 - 5x7, 1 - 8x8, 2 - 8x12, 3 - 12x16
		sendByteUSART((color >> 8));
		sendByteUSART((uint8_t)color);
	
		uint8_t i = 0;
		while (s[i]!='\0')
		{
			sendByteUSART(s[i]);
			i++;
		}
		sendByteUSART(s[i]);
	
		sendByteUSART(0x00);
		
		c = receiveByteUSART();
	}
}

void putPixel(uint16_t x, uint16_t y, uint16_t color)
{
	uint8_t c = 0x00;
	
	while (c != 0x06)
	{
		sendByteUSART(0x50);
		sendByteUSART((x >> 8));
		sendByteUSART((uint8_t)x);
		sendByteUSART((y >> 8));
		sendByteUSART((uint8_t)y);
		sendByteUSART((color >> 8));
		sendByteUSART((uint8_t)color);
		
		c = receiveByteUSART();
	}
}

void printInt(uint16_t val, uint8_t font,uint8_t col, uint8_t row)
{
			char c_buffer[12];
			snprintf(c_buffer, 12, "%10d", val);
			putString(c_buffer, col, row, font, makeColor(255, 0, 0));
}

/********************************************************************************
 *  MAIN LOOP
 ********************************************************************************/

int main(void)
{
	setupIO();
	setupTimer();
	setupMotorHbridge();

	prepareDisplay();
	initUSART();
	initLCD();

	// LCD tests
	eraseScreen();

	setBackgroundColor(makeColor(255, 255, 255));
	setOpaqueText();

	char s1[] = {"Best car ever!"};		
	putString(s1, 1, 0, 0x03, makeColor(255, 0, 0));

	sei();

 	for (;;)
	{
		startCar();
		if(carState != 0)
		{
			startMovement();
			steeringBySensors(previous_sensors_values, current_sensors_values);
		} 
		else
		{
			stopMovement();
			setupMotorHbridge();
		}
	}
	return 0;	
}
