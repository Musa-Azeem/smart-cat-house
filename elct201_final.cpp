/**
 * ELCT201 Final Project
 * 
 * Driver Code for KL25Z microcontroller
 * Combination of motor, light, and temperature systems
 * 
 * Pins:
 *      PTB0:   Torque Sensor Input
 *      PTB1:   Temperature Sensor
 *      PTB2:   Light Sensor
 * 
 *      PTA1:   Pressure Pad Button 0 Interrupt
 *      
 *      PTC9:   Motor Up Output
 *      PTC8:   Motor Down Output
 *      PTA5:   Light Output
 *      PTA4:   Heater Output
 * 
 * Circuit Assumptions:
 *      Motor Current Measuring Resistor is 10 Ohm
 */

#include "mbed.h"
#include <iostream>

// ASSIGN PINS
// Sensor input
AnalogIn TorqueSensor(PTB0);
AnalogIn TemperatureSensor(PTB1);
AnalogIn LightSensor(PTB2);

// Pressure Pad Buttons
InterruptIn PressureButton0(PTA1);

// Output Signals
DigitalOut OutputMotorUp(PTC9);
DigitalOut OutputMotorDown(PTC8);
DigitalOut OutputLight(PTA5);
DigitalOut OutputHeater(PTA4);

// on-board LEDs
DigitalOut RED_LED(LED1);
DigitalOut GREEN_LED(LED2);
DigitalOut BLUE_LED(LED3);

// GLOBAL CONSTANTS
#define V_SUPPLY 3.3f                       // Microcontroller voltage supply 3.3 V

// Motor System
#define MOTOR_SERIES_RESISTANCE 10.0f       // Resistance of torque (current) sensing resistor in series with the Motor in Ohms
#define MOTOR_CURRENT_LIMIT 0.1f            // Threshold current in amps for motor to shut off
#define CYCLE_TIME 0.5f                     // Time in seconds for microcontroller to loop
#define DOOR_FALL_TIME 10.0f                // Time in seconds that it takes house door to close 
#define DOOR_RISE_TIME 10.0f                // Time in seconds that it takes house door to close 

// Light System
#define LDR_BIAS_RESISTOR 3300.0f           // Bias resistor (upper leg of voltage divider) for LDR
#define LIGHT_RESISTANCE_LIMIT 25000.0f     // Threshold resistance for Output Light activation

// Temperature System
#define THERMISTOR_BIAS_RESISTOR 10000.0f   // Bias resistor (lower leg of voltage divider) for thermistor
#define TEMPERATURE_LIMIT 10.0f             // Enter a temperature in Celsius here for temperature deactivation; NOTE: room temperature is 25C


// For LED function
#define RED 0
#define GREEN 1
#define BLUE 2

// GLOBAL VARIABLES
bool timer_up_en = false;                   // initialize with timer disabled
bool timer_down_en = false;                 // initialize with timer disabled
float timer_down_value = DOOR_FALL_TIME;    // initialize timer as time to close door
float timer_up_value = DOOR_RISE_TIME;      // initialize timer as time to open door
bool door_is_up = false;                    // True if door is up, false if it is down


void iterate_and_check_timer();
void light_LED(int LED);
void pressure_detected();
void pressure_relieved();
float get_motor_current();
void check_torque_sensor();
void attachInterrupts();
float get_photo_resistance();
void check_light_sensor();
float get_thermistor_temperature();
void check_temperature_sensor();


// Standard entry point in C++.
int main(void) {
    // Attach the functions to the hardware interrupt pins.
    attachInterrupts();
    
    // Initialize LED outputs to OFF (LED logic is inverted)
    RED_LED = 1;
    GREEN_LED = 1;
    BLUE_LED = 1;

    // Blink the blue LED once to indicate the code is running.
    BLUE_LED = !BLUE_LED;
    wait(1.0);
    BLUE_LED = !BLUE_LED;

    while(true) {

        // Check the analog inputs.
        check_torque_sensor();
        check_light_sensor();
        check_temperature_sensor();

        // Iterate and check timer
        iterate_and_check_timer();

        // Print Current state of ouputs
        cout << "\rOUTPUT MOTOR UP (PTC9): " << OutputMotorUp << endl;
        cout << "\rOUTPUT MOTOR DOWN (PTC8): " << OutputMotorDown << endl;
        cout << "\rOUTPUT LIGHT (PTA5): " << OutputLight << endl;
        cout << "\rOUTPUT HEATER (PTA4): " << OutputHeater << endl;
        

        wait(CYCLE_TIME); // Wait <cycle_time> seconds before repeating the loop.
    }
}

void attachInterrupts() {
    PressureButton0.fall(&pressure_detected);
    PressureButton0.rise(&pressure_relieved);
}

void iterate_and_check_timer() {
    /**
     * If timer is enabled, iterates timer by the time passed based on CYCLE_TIME
     * If the timer ends, stops motor
     * 
     * Does this for either the timer going up or the timer going down
     */

    // Control timer for motor going up
    if (timer_up_en) {
        // Advance by CYCLE_TIME each cycle if up timer is enabled
        timer_up_value -= CYCLE_TIME;

        if (timer_up_value <= 0) {
            // If timer reaches zero, stop motor and reset timer
            cout << "Stopping motor up" << endl;
            light_LED(RED);
            timer_up_en = false;                // disable timer
            timer_up_value = DOOR_RISE_TIME;        // reset timer
            OutputMotorUp = 0;
            door_is_up = true;                      // door has reached top
        }
    }

    // Control timer for motor going down
    else if (timer_down_en) {
        // Advance by CYCLE_TIME each cycle if timer is enabled
        timer_down_value -= CYCLE_TIME;

        if (timer_down_value <= 0) {
            // If timer reaches zero, stop motor and reset timer
            cout << "Stopping motor down" << endl;
            light_LED(RED);
            timer_down_en = false;                   // disable timer
            timer_down_value = DOOR_FALL_TIME;   // reset timer
            OutputMotorDown = 0;
            door_is_up = false;                      // door has reached bottom
        }
    }
}


void light_LED(int LED) {
    /**
     * This function turns only the given color LED on
     * Input:
     *   0 -> RED
     *   1 -> GREEN
     *   2 -> BLUE
     */

    RED_LED = 1;
    GREEN_LED = 1;
    BLUE_LED = 1;

    switch(LED) {
        case RED:   RED_LED = 0;    break;
        case GREEN: GREEN_LED = 0;  break;
        case BLUE:  BLUE_LED = 0;   break;
    }
}

// This function will be attached to the start button interrupt.
void pressure_detected() {
    /**
     * This function starts the motor going up if the door is not already up
     * It also startes the timer for going up
     * It should be called when pressure is detected on the pad
     */

    cout << "Pressure Detected" << endl;
    // Start motor up if the door is down and it is not already going up (timer is active)
    if (!door_is_up && !timer_up_en){
        cout << " - Start Motor Up" << endl;;
        OutputMotorUp = 1;
        timer_up_en = true;          // Start timer to stop motor
        light_LED(GREEN);
    }
}

void pressure_relieved() {
    /**
     * This function starts the motor going down if the door is up
     * It also starts the timer to count down until the motor
     *  should be stopped
     * It should be called when pressue on pad is relieved
     */

    cout << "Pressure Relieved" << endl;
    // Start motor down if the door is up and it is not already going down (timer is active)
    if (door_is_up && !timer_down_en){
        cout << "- Start Motor Down" << endl;
        OutputMotorDown = 1;
        timer_down_en = true;          // Start timer to stop motor
        light_LED(BLUE);
    }
}

float get_motor_current() {
    /**
     * This function will determine the motor current in amperes 
     *  1. Read the TorqueSensor value to get the A/D converter value
     *  2. Calculate voltage on the controller input pin (across the 10 Ohm 
     *     resistor) from the motor torque sensor by multiplying the digi value
     *     by the supply voltage
     *  3. Calculate motor current using Ohm's law from votlage and resistance 
     */  

    float motor_current_digi_value = TorqueSensor.read();
    float motor_current_volt_value = V_SUPPLY * motor_current_digi_value;
    float motor_current = motor_current_volt_value / MOTOR_SERIES_RESISTANCE;
    cout << "\rMotor Current: " << motor_current << endl;
    return motor_current;
}

void check_torque_sensor() {
    /**
     * This function will check if the current accross the motor is too high
     * If it is, it stops the motor whether it is going up or down
    */

    if (get_motor_current() >= MOTOR_CURRENT_LIMIT) {
        cout << "Torque Overload - stopping motor" << endl;
        OutputMotorUp = 0;        // Stop motor if going up   (usual case)
        OutputMotorDown = 0;      // Stop motor if going down (only emergency)
        light_LED(RED);
    }
}

float get_photo_resistance() {
    /**
     * This function will determine the LDR resistance in Ohms
     *  1. Read the LightSensor value to get the A/D converter value
     *  2. Calculate voltage on the controller input pin from the light sensor
     *     by multiplying the digi value by the supply voltage
     *  3. Calculate LDR resistance using the voltage divider equation
     */  
    float light_sensor_digi_value = LightSensor.read();
    float light_sensor_volt_value = V_SUPPLY*light_sensor_digi_value;
    float ldr_resistance = LDR_BIAS_RESISTOR*((V_SUPPLY - light_sensor_volt_value) / light_sensor_volt_value);
    cout << "\rLDR Resistance: " << ldr_resistance << endl;
    return ldr_resistance;
}
void check_light_sensor() {
    /**
     * This function will check if the light level has crossed the threshold by
     *  checking the LDR resistance
     * If it has, it activates the light
     */

    if (get_photo_resistance() >= LIGHT_RESISTANCE_LIMIT) {
        cout << "Output Light On" << endl;
        OutputLight = 1;
    }
    else {
        OutputLight = 0;
    }
}

float get_thermistor_temperature() {
    /**
     * This function will determine the temperature in degrees
     *  1. Read the TemperatureSensor value to get the A/D converter value
     *  2. Calculate voltage on the controller input pin from the temperature sensor
     *     by multiplying the digi value by the supply voltage
     *  3. Calculate thermistor resistance using the voltage divider equation
     *  4. Calculate temperature using the thermistor's linear relationship between
     *     resistance and temperature
     */  
    float temp_sensor_digi_value = TemperatureSensor.read();
    float temp_sensor_volt_value = V_SUPPLY*temp_sensor_digi_value;
    float thermistor_resistance = THERMISTOR_BIAS_RESISTOR*((V_SUPPLY - temp_sensor_volt_value) / temp_sensor_volt_value);
    float thermistor_temp = ((thermistor_resistance - 10000.0)/(-320.0)) + 25.0;
    cout << "Temperature: " << thermistor_temp << endl;
    return thermistor_temp;
}

void check_temperature_sensor() {
    /**
     * This function will check if the temperature has crossed the threshold by
     *  checking the thermistor measured temperature
     * If it has, it activates the heater
     */
    if (get_thermistor_temperature() <= TEMPERATURE_LIMIT) {
        cout << "Heater Activated" << endl;
        OutputHeater = 1; // 0.91; <- what is 0.91 for?
    }
    else {
        OutputHeater = 0;
    }
}