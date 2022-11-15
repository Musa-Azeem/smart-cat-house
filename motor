// Last edit 10/19/2022
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

//int current_time = 0;
double cycle_time = 0.5;
//
//void startTimer(int time) {
//    while(current_time)
//}

#define Vsupply 3.3f //microcontroller voltage supply 3.3V

//variables for torque sensor
float MotorCurrentDigiValue; //the A/D converter value ready by the controller input pin
float MotorCurrentVoltValue; //the voltage on the controller input pin (across the 10 ohm resistor) from the motor torque sensor
float MotorCurrent; //computed from the voltage value
#define MotorSeriesResistance 10.0f //resistance of torque (current) sensing resistor in series with the Motor

float MotorCurrentLimit = 0.1; //enter a reference current in amperes for motor torque deactivatio

// This function will be attached to the start button interrupt.
void pressureDetected(void)
{
    cout << "Start Motor Up" << endl;
    RED_LED = 1;
    GREEN_LED = 0;
    OUTPUT_MOTOR_UP = 1;
}

void pressureGone(void)
{
    cout << "Start Motor Down" << endl;
    BLUE_LED = 1;
    GREEN_LED = 0;
    RED_LED = 0;
    OUTPUT_MOTOR_DOWN = 1;
//    startTimer(15);
}

//This function will determine the motor current in amperes
float getMotorCurrent(void)
{
    // 1. Read the TorqueSensor value and store it in MotorCurrentDigiValue
    // 2. Calculate MotorCurrentVoltValue from MotorCurrentDigiValue and Vsupply
    // 3. Calculate MotorCurrent using Ohm's law from MotorCurrentVoltValue and MotorSeriesResistance

    MotorCurrentDigiValue = TorqueSensor.read();
    MotorCurrentVoltValue = Vsupply * MotorCurrentDigiValue;
    MotorCurrent = MotorCurrentVoltValue / MotorSeriesResistance;
    return MotorCurrent;
}

// This function will check the Over Torque analog input.
void checkTorqueSensor(void)
{
    // Start Motor going up
    if(getMotorCurrent() >= MotorCurrentLimit) {
        cout << "Current Stop!" << endl;
        OUTPUT_MOTOR_UP = 0;
        OUTPUT_MOTOR_DOWN = 0;
        GREEN_LED = 1;
        RED_LED = 0;
    }
}

// Standard entry point in C++.
int main(void)
{
    // Attach the functions to the hardware interrupt pins.
    START_BUTTON.fall(&pressureDetected);
    START_BUTTON.rise(&pressureGone);
    
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
       //checkTorqueSensor();

        // Print Analog Values to screen
        cout << "\rMotor Current: " << getMotorCurrent() << endl;
        cout << "\rOUTPUT MOTOR UP PTC3: " << OUTPUT_MOTOR_UP << endl;
        cout << "\rOUTPUT MOTOR DOWN PTC9: " << OUTPUT_MOTOR_DOWN << endl;

        wait(cycle_time); // Wait 1 second before repeating the loop.
    }
}
// End of HardwareInterruptSeedCode
