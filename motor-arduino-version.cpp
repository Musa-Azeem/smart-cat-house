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
#define V_SUPPLY 3.3f                   // Microcontroller voltage supply 3.3 V
#define MOTOR_SERIES_RESISTANCE 10.0f   // Resistance of torque (current) sensing resistor in series with the Motor in Ohms
#define MOTOR_CURRENT_LIMIT 0.1         // Threshold current in amps for motor to shut off
#define CYCLE_TIME 500                  // Time in seconds for microcontroller to loop
#define DOOR_FALL_TIME 10               // Time in seconds that it takes house door to close 

// GLOBAL VARIABLES
bool timer_on = 0;                  // initialize with timer disabled
float timer_state = DOOR_FALL_TIME; // initialize timer as time to clode door

const int torquePin = 11;       // PWM analog
const int pressureButton0Pin = 2;
const int outputMotorUpPin = 6;
const int outputMotorDownPin = 5;

void setup() {
    
    pinMode(torquePin, INPUT);
    pinMode(pressureButton0Pin, INPUT_PULLUP);
    pinMode(outputMotorUpPin, OUTPUT)
    pinMode(outputMotorDownPin, OUTPUT)

    Serial.begin(9600);

    // Attach the functions to the hardware interrupt pins.
    attachInterrupt(digitalPinToInterrupt(PressurButton0Pin), pressure_detected, FALLING);
    attachInterrupt(digitalPinToInterrupt(PressurButton0Pin), pressure_relieved, RISING);

}

void start_timer() {
    /**
     * Starts timer to count down
     */

    Serial.print("Timer started");
    timer_on = 1;
}

void stop_timer() {
    /**
     * Stops count down timer
     */

    Serial.print("Timer Stopped" ;
    timer_on = 0;
}

void iterate_and_check_timer() {
    /**
     * If timer is enabled, iterates timer by the time passed based on CYCLE_TIME
     * If the timer ends, stops motor
     */

    if (timer_on) {
        // Advance by CYCLE_TIME each cycle if timer is enabled
        timer_state -= CYCLE_TIME;
    }

    if (timer_state <= 0) {
        // If timer reaches zero, stop motor and reset timer
        Serial.print("Timer Complete - stop motor down");

        stop_timer();                   // disable timer
        timer_state = DOOR_FALL_TIME;   // reset timer
        digitalWrite(outputMotorDownPin, 0)
    }
}

// This function will be attached to the start button interrupt.
void pressure_detected()
{
    /**
     * This function starts the motor going up
     * It should be called when pressure is detected on the pad
     */

    Serial.print("Pressure Detected - Start Motor Up" );
    digitalWrite(outputMotorUpPin, 1)
}

void pressure_relieved()
{
    /**
     * This function starts the motor going down
     * It also starts the timer to count down until the motor
     *  should be stopped
     * It should be called when pressue on pad is relieved
     */

    Serial.print("Pressure Gone - Start Motor Down" );
    digitalWrite(outputMotorDownPin, 1)
    start_timer();          // Start timer to stop motor
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
    Serial.print("\rMotor Current: " + motor_current );
    return motor_current;
}

void check_torque_sensor()
{
    /**
     * This function will check if the current accross the motor is too high
     * If it is, it stops the motor whether it is going up or down
    */

    if(get_motor_current() >= MOTOR_CURRENT_LIMIT) {
        Serial.print("Torque Overload - stopping motor" );
        digitalWrite(outputMotorUpPin, 0)
        digitalWrite(outputMotorDownPin, 0)
    }
}

// Standard entry point in C++.
void loop() {
    // Check the analog inputs.
    check_torque_sensor();

    // Iterate and check timer
    iterate_and_check_timer();

    // Print Current state of ouputs
    Serial.print("\rOUTPUT MOTOR UP PTC3: " + OutputMotorUp)
    Serial.print"\rOUTPUT MOTOR DOWN PTC9: " + OutputMotorDown)

    delay(CYCLE_TIME); // Wait <cycle_time> seconds before repeating the loop.
}
// End of HardwareInterruptSeedCode
