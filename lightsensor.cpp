// Last edit 10/19/2022
#include "mbed.h"
#include <iostream>

AnalogIn LightSensor(PTB0);
AnalogIn TemperatureSensor(PTB1);
AnalogIn TorqueSensor(PTB2);
InterruptIn START_BUTTON(PTD4);
InterruptIn STOP_BUTTON(PTA12);
DigitalOut RED_LED(LED1);
DigitalOut GREEN_LED(LED2);
DigitalOut BLUE_LED(LED3);
DigitalOut OUTPUT_LIGHT(PTC1);


#define Vsupply 3.3f //microcontroller voltage supply 3.3V

//variables for light sensor
float LightSensorDigiValue; //the A/D converter value read by the controller input pin
float LightSensorVoltValue; //voltage on the controller input pin
float LdrResistance; //photoresistance value
#define LdrBiasResistor 3300.0f //Bias resistor (upper leg of voltage divider) for LDR


// Variables to hold control reference values.
// STUDENT: EDIT THESE VALUES
float LightResistanceLimit = 25000.0; //enter a resistance reference for LDR load activation

// This function will be attached to the start button interrupt.
void StartPressed(void)
{
    cout << "Start!" << endl;
    RED_LED = 1;
    GREEN_LED = 0;
    OUTPUT_LIGHT = 1;
}

// This function will be attached to the stop button interrupt.
void StopPressed(void)
{
    // STUDENT: EDIT HERE
    cout << "Stop!" << endl;
    GREEN_LED = 1;
    RED_LED = 0;
    OUTPUT_LIGHT = 0;
    
}

//convert the input voltage from the light sensor to an LDR resistance value
//Resistance is inversely proportional to the amount of light

float getPhotoResistance(void)
{
    LightSensorDigiValue = LightSensor.read(); //read the LightSensor A/D value
    LightSensorVoltValue = Vsupply*LightSensorDigiValue; //convert to voltage
    LdrResistance = LdrBiasResistor*((Vsupply - LightSensorVoltValue) / LightSensorVoltValue); //voltage divider equation to determine LDR resistance

    return LdrResistance;
}

// This function will check the LDR analog input.
// STUDENT: USE THIS AS AN EXAMPLE FOR THE TEMPERATURE AND TORQUE CHECK FUNCTIONS
void CheckLightSensor(void)
{
    if(getPhotoResistance() >= LightResistanceLimit) {
        cout << "LDR Start!" << endl;
        OUTPUT_LIGHT = 1;     // START
        RED_LED = 0;
        GREEN_LED = 1;

    }
    else {
        OUTPUT_LIGHT = 0; // STOP
        RED_LED = 1;
        GREEN_LED = 0;
    }
}
// Standard entry point in C++.
int main(void)
{
    // Attach the functions to the hardware interrupt pins.
    START_BUTTON.rise(&StartPressed);
    STOP_BUTTON.rise(&StopPressed);
    // Initialize LED OUTPUT_LIGHTs to OFF (LED logic is inverted)
    RED_LED = 1;
    GREEN_LED = 1;
    BLUE_LED = 1;
    
    // Blink the blue LED once to indicate the code is running.
    BLUE_LED = !BLUE_LED;
    wait(1.0);
    BLUE_LED = !BLUE_LED;
    
    while(true) {
        // Check the analog inputs.
        CheckLightSensor();
        
        // Print Analog Values to screen
        cout << "\n\rLDR Resistance: " << getPhotoResistance() << endl;
        cout << "\rOUTPUT_LIGHT PTC1: " << OUTPUT_LIGHT << endl;
        
        wait(1.0); // Wait 1 second before repeating the loop.
    }
}
// End of HardwareInterruptSeedCode
