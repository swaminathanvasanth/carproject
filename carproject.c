/*	Aalto Univeristy 
*	Embedded Systems course
*/

#include "carproject.h"
#include <stdio.h>
#include <avr/io.h>
#include "util/delay.h"
#include "lcd.h"

#define F_CPU 16000000

#define PORTA_DDR 0x00  // initial value of DDRA (all the ports are input port)
#define PORTC_DDR 0xFF  // initial value of DDRC (all the ports are output port)

#define PORTA_PORT 0xFF // initial value of PORTA
#define PORTB_PORT 0xFF // initial value of PORTB
#define PORTC_PORT 0xFF // initial value of PORTC
#define PORTD_PORT 0xFF // initial value of PORTD

/* 
 * The servo configuration for different cars
 */
 
// configuration for blue car    
/*
volatile int zeroTurn = 88; 
volatile int rightMostTurn = 113; 
volatile int leftMostTurn = 65; 
volatile int expected_pulse = 850;

volatile int zeroTurn = 96; // 96
volatile int rightMostTurn = 118; 
volatile int leftMostTurn = 75; 
volatile int expected_pulse = 850;
*/
// configuration for red car    
/*
volatile int zeroTurn = 76; 
volatile int rightMostTurn = 106; 
volatile int leftMostTurn = 46; 
volatile int expected_pulse = 1400;
*/

// configuration for black car    
volatile int zeroTurn = 96; 
volatile int rightMostTurn = 76; 
volatile int leftMostTurn = 116; 
volatile int expected_pulse = 800;

// check whether the car is running
volatile int running = 0;

// the driving speed controller parameters (tachometer)
volatile int read_tacho = 0;
volatile int tach_counter = 0;
volatile int pulse_count = 0;

// lap count advanced feature
volatile int read_lap = 0;
volatile int lap_count = 0;
volatile int count_read_lap = 0;

// keypad reading buffer
volatile uint8_t read_keypad = 0;
volatile uint8_t keypad_value;
void keypad_handle(void);
void keypad_read(void);
void keypad_write(uint8_t cmd);

// initial values (1/3 duty cycle)
volatile int maxSpeed = 85; 
volatile int turnSpeed = 85;

// the steering control parameters
volatile int prevTotalWeight = 0;
volatile int prevTotalWeightSpeed = 0;
volatile int prevLightOn = 0;
volatile int averageWeightSpeed = 0;
volatile int maxTurn = 0;
volatile float Integral = 0.0;
int getCurrentPosition(void); // get the steering power at the current situation

/*
 *  Timer 0 overflow interrupt handling
 *  The timer 0 overflow interrupt is used for:
 *      - start/stop tachometer reading 
 *      - create some delays when reading the lap's starting point
 */
ISR(TIMER0_OVF_vect) {
    // reading tachometer in half second    
    if (read_tacho)
        if(tach_counter >= 0 && tach_counter <= 32)
            tach_counter++;
    
    // stop reading the lap's starting point in 1 second
    if (read_lap == 0)
        if (count_read_lap < 64)
            count_read_lap++;
        else
        {
            count_read_lap = 0;
            read_lap = 1; // enable lap counting again
        }
}

/*
 * External interrupt 1 handling
 * The external interrupt is used for:
 *      - count the pulse from the tachometer in the half second of the run
 *      - after the first half second, driving speed adjustment will be done
 */ 
ISR(INT1_vect) {

    // increase the pulse counter
    if (tach_counter >= 0 && tach_counter < 32)
        pulse_count++; 
        
    if (tach_counter == 32)
    {
        // stop the tachometer reading
        read_tacho = 0;
                
        // calculate the new maxSpeed and turnSpeed
        maxSpeed = maxSpeed * expected_pulse / pulse_count;
        turnSpeed = maxSpeed * 85 / 100; 
        
        // in case the new value is so big or small
        if (maxSpeed >= 85 || maxSpeed <= 45)
        {
            maxSpeed = 85;
            turnSpeed = 73;
        }
    
        GICR = (1 << INT2);
    }
}

/*
 * External interrupt 2 handling
 * The external interrupt is used for:
 *      - enable the keypad handling
 */ 
ISR(INT2_vect) {
    read_keypad = 1; // enable keypad handling
}

/* Main function */
int main(void) {

    // set the direction for PORTA, PORTB, PORTC, PORTD
    DDRA = PORTA_DDR;
    DDRC = PORTC_DDR;
    DDRB = _BV(PB1) | _BV(PB4) | _BV(PB7); // PB 1, 4, 7 are output
    DDRD = _BV(PD7) | _BV(PD4); // PD 4, 7 are output for motor, servo signaling

    // set the PORT value for PORTA, PORTB, PORTC, PORTD
    PORTA = PORTA_PORT;
    PORTB = PORTB_PORT;
    PORTC = PORTC_PORT;
    PORTD = PORTD_PORT;

    // Timer 0 settings, Timer 0 overflow interrupt enable
    TCCR0 = TCCR0 | ((1 << CS02) | (0 << CS01) | (1 << CS00)); // clk/1024
    TIMSK = TIMSK | (1 << TOIE0); 
    TCNT0 = 0;

    // External interrupt 1 enable
    GICR |= (1 << INT1);
    MCUCR |= ((1 << ISC11) | (1 << ISC10));
    
    // initializing the keypad with TWI Module
    GICR |= (1 << INT2);
    TWBR = 98; // TWBR = 98, TWPS = 1 -> SCL freq = 20kHz
    read_keypad = 0;
    keypad_write(0xF0);
    
    // Timer 2 is used to control the motor signaling
    TCCR2 = _BV(WGM20) | _BV(WGM21) | _BV(COM21) | _BV(CS21);     

    // set the PORTB value to drive the motor clockwise
    PORTB = PORTB | 0x08;
    PORTB = PORTB & 0x7F;
    
    // The timer 1 settings for Fast PWM mode (mode 14)    
    TCCR1B = TCCR1B | ((0 << ICNC1) | (0 << ICES1));    
    TCCR1B = TCCR1B | ((1 << WGM13) | (1 << WGM12));    
    TCCR1B = TCCR1B | ((1 << CS12) | (0 << CS11) | (0 << CS10));
    
    TCCR1A = TCCR1A | ((1 << COM1B1) | (0 << COM1B0)); 
    TCCR1A = TCCR1A | ((0 << FOC1B));   
    TCCR1A = TCCR1A | ((1 << WGM11) | (0 << WGM10));
    
    ICR1 = 1250; // 50 kHz
    TCNT1 = 0;  
    OCR1B = 85;
    
    int averageWeight = 0;  
    int prevAverageWeight = 0;  
    
    // The PID controller values for each of the car
    float Kp = 0.8; // blue = 0.9, red = 0.8
    float Ki = 0.0; // blue = 0.00015
    float Kd = 0.0; // blue = 1.9, red = 1.2
    float P = 0.0, I = 0.0, D = 0.0;
    float correctedWeight = 0.0;
    
    // initializing the LCD display
    lcd_init(LCD_DISP_ON);         
    lcd_clrscr();                  
    lcd_puts("Hello World");     
        
    // Global interrupt enable
    sei();       
    
    // the main loop
    while (1)
    {
        // after every 1ms, the required turning power will be read
        // the servo will turn to that angle
        averageWeight = getCurrentPosition();
        prevAverageWeight = averageWeight;
        OCR1B = zeroTurn + averageWeight;
        _delay_ms(1);
            
        // starting the car by pressing the START button in PB1
        if ((PINB | 0xF7) == 0xF7)
        {
            running = 1;
            read_lap = 1;
            read_tacho = 1;
        }
        
        // check if the keypad is required to read
        if (read_keypad == 1)
        {            
            keypad_handle();
            read_keypad = 0;
            _delay_ms(1);
        }
        
        if (running){
            // previous turning power will be saved
            averageWeight = getCurrentPosition();
            prevAverageWeight = averageWeight;
            
            while (1) 
            {
                // new turning power will be read
                averageWeight = getCurrentPosition();

                // if the car has to do the max turning, PID will not be applied
                if (maxTurn)
                {
                    OCR1B = zeroTurn + averageWeight;
                    Integral += (float) averageWeight;
                    maxTurn = 0;                    
                }
                else
                {
                    lcd_clrscr();
                    
                    // PID calculations
                    // if we assume that the target turning power is 0 (which means that no power is need and the car runs straighly stable), 
                    // averageWeight will be the error of PID controller
                    P = (float) averageWeight * Kp; // P = error * Kp
                    Integral += (float) averageWeight; 
                    I = Integral * Ki; // I = Integrate(error) * Ki
                    D = ((float) averageWeight - (float) prevAverageWeight) * Kd; // D = (new_error - old_error) * Kd
                    correctedWeight = P + I + D; // PID = P + I + D
        
                    // if the correctedWeight is bigger than big max turn or smaller than small turn,
                    // some adjustments has to be done to protect the servo
                    if (rightMostTurn > leftMostTurn)
                    {
                        if (zeroTurn + (int) correctedWeight < rightMostTurn && zeroTurn + (int) correctedWeight > leftMostTurn)
                            OCR1B = zeroTurn + (int) correctedWeight;
                        else
                            if (zeroTurn + (int) correctedWeight > rightMostTurn)
                                OCR1B = rightMostTurn;
                            else
                                OCR1B = leftMostTurn;
                    }
                    else
                    {
                        if (zeroTurn + (int) correctedWeight > rightMostTurn && zeroTurn + (int) correctedWeight < leftMostTurn)
                            OCR1B = zeroTurn + (int) correctedWeight;
                        else
                            if (zeroTurn + (int) correctedWeight > leftMostTurn)
                                OCR1B = leftMostTurn;
                            else
                                OCR1B = rightMostTurn;
                    }
                }

                // the car will run at full speed if no turning power is applied
                if (prevAverageWeight == 0)
                    OCR2 = maxSpeed;
                else
                {
                    // break the car in half ms when turning power is applied
                    PORTB = PORTB & 0xF7;
                    _delay_us(500);
                    PORTB = PORTB | 0x08;
                    OCR2 = turnSpeed - averageWeightSpeed; // slow speed
                }
                
                // previous turning power will be saved
                prevAverageWeight = averageWeight;
                
                // stop reading the sensor in 2 ms
                _delay_ms(2);
                
                // check if the keypad is required to read
                if (read_keypad == 1)
                {            
                    keypad_handle();
                    read_keypad = 0;
                    _delay_ms(1);
                }
                
            }
        }
    }

    return 0;
}

// get the current position (turning power) compared to the main line
// low pass filter will be used by averaging the previous and current sensor reading values
int getCurrentPosition(void)
{  
    int nLightOn = 0;
    int totalWeight = 0;
    int averageWeight = 0;
    int totalWeightSpeed = 0;
            
    // if there is no light on, the previous turning power will be applied
    // this will be helpful in case the sensor bumper is out of the track
    if (PINA == 0x00)
    {
        totalWeight = prevTotalWeight;
        nLightOn = prevLightOn;
        totalWeightSpeed = prevTotalWeightSpeed;
        maxTurn = 1; // this also means maxTurn
    }
    else if (PINA != 0x00)
    {            
        if ((PINA & 0x01) == 0x01) // the first sensor from right to left is reflected
        {
            nLightOn++;
            totalWeight += (rightMostTurn - zeroTurn); 
            totalWeightSpeed += 90;
            maxTurn = 1;
        }   
        
        if ((PINA & 0x02) == 0x02) // the second sensor from right to left is reflected
        {
        
            nLightOn++;
            totalWeight += ((rightMostTurn - zeroTurn) * 75) / 100;
            totalWeightSpeed += 90; 
        }    
            
        if ((PINA & 0x04) == 0x04) // the third sensor from right to left is reflected
        {        
            nLightOn++;
            totalWeight += ((rightMostTurn - zeroTurn) * 25) / 100;  
            totalWeightSpeed += 30;
        }    
            
        if ((PINA & 0x08) == 0x08) // the fourth sensor from right to left is reflected
        {

            nLightOn++;
            totalWeight += ((rightMostTurn - zeroTurn) * 10) / 100; 
            totalWeightSpeed += 5;
        }    
            
        if ((PINA & 0x10) == 0x10) // the fifth sensor from right to left is reflected
        {

            nLightOn++;
            totalWeight += ((leftMostTurn - zeroTurn) * 10) / 100;
            totalWeightSpeed += 5;
            
        }    
            
        if ((PINA & 0x20) == 0x20) // the sixth sensor from right to left is reflected
        {

            nLightOn++;
            totalWeight += ((leftMostTurn - zeroTurn) * 25) / 100;
            totalWeightSpeed += 30;
        }    
            
        if ((PINA & 0x40) == 0x40) // the seventh sensor from right to left is reflected
        {

            nLightOn++;
            totalWeight += ((leftMostTurn - zeroTurn) * 75) / 100;
            totalWeightSpeed += 90;
        }    
            
        if ((PINA & 0x80) == 0x80) // the eighth sensor from right to left is reflected
        {
            
            nLightOn++;
            totalWeight += (leftMostTurn - zeroTurn); 
            totalWeightSpeed += 90;
            maxTurn = 1;
        }
    }
    else
    {
        // protective mechanism for some unknown cases
        totalWeight = prevTotalWeight;
        nLightOn = prevLightOn;
        totalWeightSpeed = prevTotalWeightSpeed;
        maxTurn = 1;
    }
    
    // if the car is running, lap count will be assessed
    if (running)
    if (read_lap)
    {
        if (nLightOn > 6)  // probably the car just passed the starting line
        {
            Integral = 0.0; // reset the integral component of PID
            
            // increase the lap count and display to the LCD
            lap_count++;
            
            lcd_gotoxy(4, 0);
            lcd_puts("Lap "); 
            lcd_data(lap_count);
            
            // stop reading the lap count in 1 second
            read_lap = 0;
            count_read_lap = 0;
            
            // if lap count is over 3, stop the car
            if (lap_count > 3) 
            {
                running = 0;
                TCCR2 = 0;
                PORTB = 0x00;
            }
        }
    }  
    
    // sometimes, the sensor sees some particals not belongs to the track and has weird result
    // therefore, it is fine to make sure that the line position can be got by 1,2,3 lights,
    // moreover, if the previous and current reading has more than 1 light different, it can be ignored
    if (nLightOn > prevLightOn + 1 || nLightOn > 4)
    {
        return (prevTotalWeight / prevLightOn);
    }
    
    // low pass filter is applied by averaging the previous and current reading
    if (nLightOn + prevLightOn != 0)
    {
        averageWeight = (totalWeight + (int) prevTotalWeight) / (nLightOn + (int) prevLightOn);        
        averageWeightSpeed = ((int) averageWeightSpeed + (totalWeightSpeed / nLightOn / 5)) / 2;
    }
    else
    {   
        // preventing from zero division
        averageWeight = (totalWeight + (int) prevTotalWeight) / 2;
        averageWeightSpeed = ((int) averageWeightSpeed + (totalWeightSpeed / 10)) / 2;
    }        
    
    // the current state will be saved for the next reading
    prevTotalWeight = totalWeight;
    prevLightOn = nLightOn;
    prevTotalWeightSpeed = totalWeightSpeed;
    
    // maxTurn will happen only when the leftmost or rightmost LED light only, therefore, 
    // it is fine to reset maxTurn if more than 1 LED is on
    if (nLightOn > 1) maxTurn = 0;
    
    return averageWeight;
}

/*
 * Copyright Spark Fun Electronics 2009 Viliam Klein
 * 
 */
void i2cSendStart(void)
{
    // send start condition
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
}

void i2cSend(void)
{
    TWCR = (1 << TWINT) | (1 << TWEN);
}

void i2cSendStop(void)
{
    // transmit stop condition
    TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
}

void i2cWaitForComplete(void)
{
    // wait for i2c interface to complete operation
    while (!(TWCR & (1 << TWINT)));
}

void keypad_write(uint8_t cmd)
{
    i2cSendStart();
    i2cWaitForComplete();
    _delay_us(10);
    
    TWDR = 0x40; // slave device address 
    i2cSend();
    i2cWaitForComplete();
    _delay_us(10);
   
    // start sending
    TWDR = cmd;
    i2cSend();
    i2cWaitForComplete();
    _delay_us(10);
    
    i2cSendStop();
}

void keypad_read(void)
{
    i2cSendStart();
    i2cWaitForComplete();
    _delay_us(10);
  
    TWDR = 0x40; // slave device address
    i2cSend();
    i2cWaitForComplete();
    _delay_us(10);
    
    // start reading
    i2cSend();
    i2cWaitForComplete();
    _delay_us(10);
    keypad_value = TWDR;
    
    i2cSendStop();
}

void keypad_handle(void)
{
    int row = 5, column = 5;
    
    // configuration for blue car    
    int blue_1_zeroTurn = 88; 
    int blue_1_rightMostTurn = 113; 
    int blue_1_leftMostTurn = 65; 
    int blue_1_expected_pulse = 850;

    int blue_2_zeroTurn = 96; // 96
    int blue_2_rightMostTurn = 118; 
    int blue_2_leftMostTurn = 75; 
    int blue_2_expected_pulse = 850;

    // configuration for red car    
    int red_zeroTurn = 76; 
    int red_rightMostTurn = 106; 
    int red_leftMostTurn = 46; 
    int red_expected_pulse = 1400;

    // configuration for black car    
    int black_zeroTurn = 96; 
    int black_rightMostTurn = 76; 
    int black_leftMostTurn = 116; 
    int black_expected_pulse = 800;
    
    // if the car is running, stop the car when receiving keystroke
    if (running == 1)
    {
        running = 0;
        TCCR2 = 0;
        PORTB = 0x00;
        
        lcd_clrscr();
        lcd_puts("CAR STOP");
    }
    else
    {
        keypad_write(0xF0); // row reading
        _delay_ms(5);
        
        keypad_read();
        switch (keypad_value)
        {
            case 0x70: // 0111 0000 - first bit is low
                row = 1;
                break;
            case 0xB0: // 1011 0000
                row = 2;
                break;
            case 0xD0: // 1101 0000
                row = 3;
                break;
            case 0xE0: // 1110 0000
                row = 4;
                break;
            default: break;
        }
        
        keypad_write(0x0F); // column reading 
        _delay_ms(5);
                
        keypad_read();
        switch (keypad_value)
        {
            case 0x07: // 0000 0111 - first bit is low
                column = 1;
                break;
            case 0x0B: // 0000 1011
                column = 2;
                break;
            case 0x0D: // 0000 1101
                column = 3;
                break;
            case 0x0E: // 0000 1110
                column = 4;
                break;
            default: break;
        }     
    }
        
    // The corresponding car configuration will be load based on the pressed button
    if (row == 1 && column == 1)
    {
        zeroTurn = blue_1_zeroTurn; 
        rightMostTurn = blue_1_rightMostTurn; 
        leftMostTurn = blue_1_leftMostTurn; 
        expected_pulse = blue_1_expected_pulse;
        
        lcd_clrscr();
        lcd_puts("BLUE 1 CAR LOADED");
    }
    else if (row == 1 && column == 2)
    {
        zeroTurn = blue_2_zeroTurn; 
        rightMostTurn = blue_2_rightMostTurn; 
        leftMostTurn = blue_2_leftMostTurn; 
        expected_pulse = blue_2_expected_pulse;
        
        lcd_clrscr();
        lcd_puts("BLUE 2 CAR LOADED");
    }
    else if (row == 1 && column == 3)
    {
        zeroTurn = red_zeroTurn; 
        rightMostTurn = red_rightMostTurn; 
        leftMostTurn = red_leftMostTurn; 
        expected_pulse = red_expected_pulse;
        
        lcd_clrscr();
        lcd_puts("RED CAR LOADED");
    }
    else if (row == 1 && column == 4)
    {
        zeroTurn = black_zeroTurn; 
        rightMostTurn = black_rightMostTurn; 
        leftMostTurn = black_leftMostTurn; 
        expected_pulse = black_expected_pulse;
        
        lcd_clrscr();
        lcd_puts("BLACK CAR LOADED");
    }
}
