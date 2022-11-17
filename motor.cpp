#include "mbed.h"
#include <iostream>

AnalogIn TorqueSensor(PTB2);
InterruptIn START_BUTTON(PTD4);
InterruptIn STOP_BUTTON(PTA12);
DigitalOut RED_LED(LED1);
DigitalOut GREEN_LED(LED2);
DigitalOut BLUE_LED(LED3);
DigitalOut OUTPUT_MOTOR_UP(PTC3);
DigitalOut OUTPUT_MOTOR_DOWN(PTC9);

// GLOBAL CONSTANTS
#define V_SUPPLY 3.3f                   // Microcontroller voltage supply 3.3 V
#define MOTOR_SERIES_RESISTANCE 10.0f   // Resistance of torque (current) sensing resistor in series with the Motor
#define MOTOR_CURRENT_LIMIT = 0.1       // Threshold current in amps for motor to shut off
#define CYCLE_TIME = 0.5                // Time in seconds for microcontroller to loop
#define DOOR_FALL_TIME = 10             // Time in seconds that it takes house door to close 

// GLOBAL VARIABLES
bool timer_on = 0;

void start_timer() {
   timer_on = 1;
}

// This function will be attached to the start button interrupt.
void pressure_detected(void)
{
    cout << "Start Motor Up" << endl;
    RED_LED = 1;
    GREEN_LED = 0;
    OUTPUT_MOTOR_UP = 1;
}

void pressure_gone(void)
{
    cout << "Start Motor Down" << endl;
    BLUE_LED = 1;
    GREEN_LED = 0;
    RED_LED = 0;
    OUTPUT_MOTOR_DOWN = 1;
//    start_timer();
}

float get_motor_current(void)
{
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

void check_torque_sensor(void)
{
    /**
     * This function will check if the current accross the motor is too high
     * If it is, it stops the motor whether it is going up or down
    */

    if(get_motor_current() >= MOTOR_CURRENT_LIMIT) {
        cout << "Torque Overload - stopping motor" << endl;
        OUTPUT_MOTOR_UP = 0;        // Stop motor if going up   (usual case)
        OUTPUT_MOTOR_DOWN = 0;      // Stop motor if going down (only emergency)
        RED_LED = 0;
    }
}

// Standard entry point in C++.
int main(void)
{
    // Attach the functions to the hardware interrupt pins.
    START_BUTTON.fall(&pressure_detected);
    START_BUTTON.rise(&pressure_gone);
    
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

        // Print Current state of ouputs
        cout << "\rOUTPUT MOTOR UP PTC3: " << OUTPUT_MOTOR_UP << endl;
        cout << "\rOUTPUT MOTOR DOWN PTC9: " << OUTPUT_MOTOR_DOWN << endl;

        wait(CYCLE_TIME); // Wait <cycle_time> seconds before repeating the loop.
    }
}
// End of HardwareInterruptSeedCode
