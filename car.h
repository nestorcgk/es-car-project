#ifndef F_CPU
#define F_CPU 16000000UL	// 16 MHz
#endif

#define BAUD 9600

/********************************************************************************
 * includes
 ********************************************************************************/

#include <inttypes.h> 
#include <stdio.h>
#include <avr/io.h>   
#include <avr/interrupt.h>
#include <util/delay.h>
#include <util/setbaud.h>

/********************************************************************************
 * constants
 ********************************************************************************/
#define STEERING_PORT PORTB
#define STEERING_DDR DDRB
#define BUMPER_PIN PINA //bumper output
#define BUMPER_DDR DDRA 
#define TACHOMETER_PORT PORTL
#define TACHOMETER_PIN PINL
#define TACHOMETER_DDR DDRL
#define MOTOR_DDR DDRH
#define HBRIDGE_DDR DDRK
#define HBRIDGE_PORT PORTK
#define HALL_PORT PORTL
#define LEDS_PORT PORTC
#define LEDS_DDR  DDRC
#define SWITCHES_PIN PINE
#define SWITCHES_DDR DDRE
#define BLACK_SWITCH PE5
#define TACHOMETER_OUT PL2

#define LAP_LIMIT 6
#define AI_ARRAY_LENGTH 255
#define AI_DELTA 20
#define AI_COEFFICIENT 0.01
#define AI_MULTIPLYER_LIMIT 1.30
#define SPEED_LIMIT_DC 0.7
#define CORNER_SPEED_LIMIT 0.025
#define PID_PERIOD 0.25
#define KP 2.2
#define KI 0.25 
#define KD 0.05
#define SAMPLE_SIZE 4
#define LOWER_LIMIT 3
#define PID_ENABLE 1

/********************************************************************************
 *  variables
 ********************************************************************************/

// states
uint8_t carState = 0;
uint8_t buttonFlag = 0; 
uint8_t lapFlag = 0;
uint8_t flashFlag0 = 0;
uint8_t flashFlag1 = 0;
uint8_t cornerFlag = 0;
uint8_t time = 0;
// counter
volatile uint8_t speedCounter = 0;
volatile uint8_t lapCounter = 0;
volatile uint16_t counter = 0;
// speed
volatile uint8_t maxSpeed = 0;
volatile uint16_t rotationsAmount = 0;
uint8_t car_speed = 25;//the desired speed control 
uint8_t current_speed;//the last set value of the car speed
volatile uint8_t measured_speed = 0;
// position
uint16_t previous_position = 0;
// sensors
uint8_t previous_sensors_values = 0x00;
uint8_t current_sensors_values = 0x00;
// error
double last_error = 0.0;
// AI array
uint16_t cornerDistances [AI_ARRAY_LENGTH];
uint16_t cornerCounter = 0;

/********************************************************************************
 * functions
 ********************************************************************************/

void setupIO(void);
void setupMotorHbridge(void);
// steering
void setSteering(void);
void startSteering(int8_t);
void addCornerDistance(void);
double multiplyerCalculation(void);
void steeringBySensors(uint8_t, uint8_t);
void stopSteering(void);
// timer
void setupTimer(void);
void setupHallTimer(void);
void setupPIDTimer(void);
void startPIDTimer(void);
void stopPIDTimer(void);
// motor control
void setMotorPower(uint8_t);
void startMotor(void);
void changeSpeed(uint8_t);
void motorBreak(void);
uint8_t calculateSpeedPID(uint8_t);
// bumper
uint8_t getSensorsValues(void);
// button
void startMovement(void);
void stopMovement(void);
// led 
void turnOnLED(uint8_t);
void turnOffLED(uint8_t);
// hall sensor
void readSpeed(void);
// everything to start the controlling is done inside
void startCar(void);
uint8_t bitCount (uint8_t);

/********************************************************************************
 *  display functions
 ********************************************************************************/
void initUSART(void);
void sendByteUSART(uint8_t);
uint8_t receiveByteUSART(void);
uint16_t makeColor(uint8_t, uint8_t, uint8_t);
uint16_t makeColor2(uint8_t, uint8_t, uint8_t);
void prepareDisplay(void);
void initLCD(void);
void setBackgroundColor(uint16_t);
void eraseScreen(void);
void setOpaqueText(void);
void setTransparentText(void);
void putChar(char, uint8_t, uint8_t, uint16_t);
void putString(char*, uint8_t, uint8_t, uint8_t, uint16_t);
void putPixel(uint16_t, uint16_t, uint16_t);
void printInt(uint16_t, uint8_t, uint8_t, uint8_t);
