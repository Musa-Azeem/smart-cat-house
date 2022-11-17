/**
 * VERSION FOR TESTING ON ARDUINO
 * 
 * ELCT201 Final Project
 * 
 * Driver Code for Arduino
 * 
 * Pins:
 *      PTA1:   Torque Sensor Input
 *      PTA4:   Pressure Pad Button 0 Interrupt
 *      PTA5:   Pressure Pad Button 1 Interrupt
 *      
 *      PTB0:   Motor Up Output
 *      PTB1:   Motor Down Output
 * 
 * Circuit Assumptions:
 *      Motor Current Measuring Resistor is 10 Ohms
 */

// C++ code
//


const int ledPin = 12;   //ledPin
const int buttonPin = 2; //Button to perform interrupt
int ledToggle = LOW;     //led state

// GLOBAL CONSTANTS
#define V_SUPPLY 5.0f                   // Microcontroller voltage supply 3.3 V
#define MOTOR_SERIES_RESISTANCE 1.0f    // Resistance of torque (current) sensing resistor in series with the Motor in Ohms
#define MOTOR_CURRENT_LIMIT 0.1         // Threshold current in amps for motor to shut off
#define CYCLE_TIME 0.5                  // Time in seconds for microcontroller to loop
#define DOOR_FALL_TIME 10.0              // Time in seconds that it takes house door to close 
#define DOOR_RISE_TIME 10.0              // Time in seconds that it takes house door to close 

// GLOBAL VARIABLES
bool timer_up_en = false;                   // initialize with timer disabled
bool timer_down_en = false;                 // initialize with timer disabled
float timer_down_value = DOOR_FALL_TIME;    // initialize timer as time to close door
float timer_up_value = DOOR_RISE_TIME;      // initialize timer as time to open door
bool door_is_up = false;                    // True if door is up, false if it is down

const int torquePin = 11;       // PWM analog
const int pressureButton0Pin = 2;
const int outputMotorUpPin = 6;
const int outputMotorDownPin = 5;

void setup() {
    
    pinMode(torquePin, INPUT);
    pinMode(pressureButton0Pin, INPUT_PULLUP);
    pinMode(outputMotorUpPin, OUTPUT);
    pinMode(outputMotorDownPin, OUTPUT);

    Serial.begin(9600);

    // Attach the functions to the hardware interrupt pins.
    attachInterrupt(digitalPinToInterrupt(pressureButton0Pin), on_pressure, CHANGE);
    // attachInterrupt(digitalPinToInterrupt(pressureButton0Pin), onPressure, FALLING);
    // attachInterrupt(digitalPinToInterrupt(pressureButton0Pin), onPressure, RISING);

}

void on_pressure() {
    
    if (digitalRead(pressureButton0Pin) == HIGH) {
        pressure_relieved();
    }
    else {
        Serial.println("Call pressure detected");
        pressure_detected();
    }
}

void iterate_and_check_timer() {
    /**
     * If timer is enabled, iterates timer by the time passed based on CYCLE_TIME
     * If the timer ends, stops motor
     */

    // Control timer for motor going up
    if (timer_up_en) {
        // Advance by CYCLE_TIME each cycle if up timer is enabled
        timer_up_value -= CYCLE_TIME;

        if (timer_up_value <= 0) {
            // If timer reaches zero, stop motor and reset timer
            Serial.println("Timer Complete - stop motor up");
            
            timer_up_en = false;                // disable timer
            timer_up_value = DOOR_RISE_TIME;        // reset timer
            digitalWrite(outputMotorUpPin, 0);
            door_is_up = true;                      // door has reached top
        }
    }

    // Control timer for motor going down
    else if (timer_down_en) {
        // Advance by CYCLE_TIME each cycle if timer is enabled
        timer_down_value -= CYCLE_TIME;

        if (timer_down_value <= 0) {
            // If timer reaches zero, stop motor and reset timer
            Serial.println("Timer Complete - stop motor down");

            timer_down_en = false;                   // disable timer
            timer_down_value = DOOR_FALL_TIME;   // reset timer
            digitalWrite(outputMotorDownPin, 0);
            door_is_up = false;                      // door has reached bottom
        }
    }
}

// This function will be attached to the start button interrupt.
void pressure_detected()
{
    /**
     * This function starts the motor going up if the door is not already up
     * It also startes the timer for going up
     * It should be called when pressure is detected on the pad
     */

    Serial.println("Pressure Detected");
    // Start motor up if the door is down and it is not already going up (timer is active)
    if (!door_is_up && !timer_up_en){
        Serial.println(" - Start Motor Up");
        digitalWrite(outputMotorUpPin, 1);
        timer_up_en = true;          // Start timer to stop motor
    }
}

void pressure_relieved()
{
    /**
     * This function starts the motor going down if the door is up
     * It also starts the timer to count down until the motor
     *  should be stopped
     * It should be called when pressue on pad is relieved
     */

    Serial.println("Pressure Gone");
    // Start motor down if the door is up and it is not already going down (timer is active)
    if (door_is_up && !timer_down_en){
        Serial.println("- Start Motor Down");
        digitalWrite(outputMotorDownPin, 1);
        timer_down_en = true;          // Start timer to stop motor
    }
}

float get_motor_current()
{
    /**
     * This function will determine the motor current in amperes 
     *  1. Read the TorqueSensor value to get the A/D converter value
     *  2. Calculate voltage on the controller input pin (across the 10 Ohm 
     *     resistor) from the motor torque sensor by multiplying the digi value
     *     by the supply voltage
     *  3. Calculate motor current using Ohm's law from votlage and resistance 
     */  

    float motor_current_digi_value = digitalRead(torquePin);
    float motor_current_volt_value = V_SUPPLY * motor_current_digi_value;
    float motor_current = motor_current_volt_value / MOTOR_SERIES_RESISTANCE;
    Serial.print("Motor Current: ");
    Serial.println(motor_current);

    return motor_current;
}

void check_torque_sensor()
{
    /**
     * This function will check if the current accross the motor is too high
     * If it is, it stops the motor whether it is going up or down
    */

    if(get_motor_current() >= MOTOR_CURRENT_LIMIT) {
        Serial.println("Torque Overload - stopping motor" );
        digitalWrite(outputMotorUpPin, 0);
        digitalWrite(outputMotorDownPin, 0);
    }
}

// Standard entry point in C++.
void loop() {
    // Check the analog inputs.
    check_torque_sensor();

    // Iterate and check timer
    iterate_and_check_timer();


    delay(CYCLE_TIME*1000); // Wait <cycle_time> seconds before repeating the loop.
}
// End of HardwareInterruptSeedCode
