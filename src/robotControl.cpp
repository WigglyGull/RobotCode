/*
* Arduino Robot Control Software
*
* Serial control interface for an Arduino-based robot. 
*
* Please note the inclusion of DualMotorControl.h, which can be found in lib/DualMotorControl
*
* Copyright 2024 C. Varney, A. Walker, S. Baynes
* Free software under a MIT-0 license (see LICENSE.txt or https://github.com/aws/mit-0)
*/

#include <HardwareSerial.h>
#include <Arduino.h>
#include <UcTTDcMotor.h>
#include "DualMotorControl.h"

// Define pins
const PwmPin MOTOR_PIN_LF = D3;
const PwmPin MOTOR_PIN_LR = D9;
const PwmPin MOTOR_PIN_RF = D10;
const PwmPin MOTOR_PIN_RR = D11;

const int TRIGGER_PIN = 7;
const int ECHO_PIN = 6;
const int ENCODER_PIN_L = 2;
const int ENCODER_PIN_R = 4;

const int FORWARD_LED = 13;
const int TURN_LED = 12;
const int HAZARD_LED = 8;

// Define motor connections
UcTTDcMotor motorL(MOTOR_PIN_LF, MOTOR_PIN_LR); 
UcTTDcMotor motorR(MOTOR_PIN_RF, MOTOR_PIN_RR); 

// Configure the dualMotorControl library with pointers to our motor definitions, encoders and LEDs
DualMotorControl motorController(&motorL, &motorR, ENCODER_PIN_L, ENCODER_PIN_R); 


class Interactions {
    /*
    *   Handles interaction between the robot and the world (sensing and motion).
    */
    public:
        int pollDistance()
        {
            /*
            * Return the distance in centimetres to the closest obstacle
            * Modified from https://randomnerdtutorials.com/complete-guide-for-ultrasonic-sensor-hc-sr04
            */

            // The sensor is triggered by a HIGH pulse of 10 or more microseconds.
            // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
            digitalWrite(TRIGGER_PIN, LOW);
            delayMicroseconds(5);
            digitalWrite(TRIGGER_PIN, HIGH);
            delayMicroseconds(10);
            digitalWrite(TRIGGER_PIN, LOW);
            
            // Read the signal from the sensor: a HIGH pulse whose
            // duration is the time (in microseconds) from the sending
            // of the ping to the reception of its echo off of an object.
            pinMode(ECHO_PIN, INPUT);
            duration = pulseIn(ECHO_PIN, HIGH);
            
            // Convert the time into a distance
            return (duration/2) / DISTANCE_RATIO;
        }

        void printDistance()
        {
            /*
            * Prints the distance to the nearest obstacle to the serial console 
            */
            Serial.print("Nearest obstacle: ");
            Serial.print(pollDistance());
            Serial.print("cm\n\r");
        }

        void search() 
        {
            /*
            * Search mode - moves forward until it finds an obstacle
            */
            motorController.forward(60);
            for(size_t i = 0; i < 200; i++) {
                if (pollDistance() <= 10) {
                    Serial.print("Obstacle Found! Stopping.\n\r");
                    motorController.stop();
                    return;
                }
                delay(50);
            }
            Serial.print("Timed out finding obstacle. Stopping.\n\r");
            motorController.stop();
        }   
    private:
        // Ratio between pulse return time and distance in cm
        const float DISTANCE_RATIO = 29.1; 
        long duration;

};
    

class Control {
    /*
    * Handles the real-time driving mode of the robot
    */
   public:

        void forwardSafely() 
        {
            /*
            * Move the robot forward, constantly checking for obstacles
            */

            // If there's an obstacle, stop and prevent the robot from moving.
            if(interactions.pollDistance() <= 10){
                if (currentCommand == 'w') {
                    Serial.println("Hazard detected! Please reverse.\r\n");
                    currentCommand = 'h';
                    digitalWrite(HAZARD_LED, 1);
                }
                motorController.stop();
                return;
            }

            // If we're not currently stopped for an obstacle, can move forwards
           if (currentCommand != 'w' && currentCommand != 'h') {
                currentCommand = 'w';
                motorController.forward(speed);
            }

           
        }

        void handleKeypress() {
            /*
            * Switch the state of the robot when a new key is pressed.
            * Stores the last state and checks against current state to prevent repeated calls to motorController
            */
            switch (tolower(buffer)) {
                case 'w':
                    forwardSafely();
                    break;
                
                case 's':
                    if (currentCommand != 's') {
                        currentCommand = 's';
                        motorController.reverse(speed);
                    }
                    break;

                case 'a':
                    if (currentCommand != 'a') {
                        currentCommand = 'a';
                        motorController.turnLeft(speed);
                    }
                    break;

                case 'd':
                    if (currentCommand != 'd') {
                        currentCommand = 'd';
                        motorController.turnRight(speed);
                    }
                    break;

                case 'p':
                    interactions.printDistance();
                    break;
                    
            }
        }

        void start()
        {
            /*
            * Runs the robot in "interactive" mode - WASD controls, hold shift to move slower.
            */

            // Explain the interface to the user on startup
            Serial.println("Interactive mode: \n- Use WASD to move\n- Hold shift to move slower\n-Press 'x' to exit\n\n\r");
            
            // While the user hasn't requested to exit, read a new key if available
            while (buffer != 'x') {
                if ( Serial.available() ) {
                    noInputTime = 0;
                    // We have a command available. Read it to memory
                    buffer = Serial.read();

                    speed = isUpperCase(buffer) ? 40 : 100; // need to move these!!!
                    handleKeypress();
                } else {
                    if (noInputTime > 20) {
                            currentCommand = 'n';
                        } else {
                            noInputTime++;
                        }
                }

                delay(SERIAL_POLL_DELAY);
            }
        }

    private:
        // Serial poll delay should be as short as possible. Gaps between inputs are handled by noInputTime.
        uint8_t SERIAL_POLL_DELAY = 5;

        // Speeds are set to minimum and maximum that the motors support (without stall), should not need adjustment
        uint8_t FAST_SPEED = 100;
        uint8_t SLOW_SPEED = 40;

        Interactions interactions;
        size_t noInputTime = 0;
        char currentCommand = -1;
        int8_t speed = 0;
        char buffer = -1;
        
        
};


class CommandInterface {
    /*
    *   Handles interaction between the robot and the user over serial
    */
    public:
        void console()
        {
            /*
            * Start the command line interface. A state machine to move to a new control state depending on user input.
            */
            currentCommand[32] = {0};
            Serial.print(">");
            readToBuffer(currentCommand);


            // Compare the array of chars we read with the known commands we have stored, and perform the action.
            if ( memcmp(&currentCommand, &helpCommand, 4) == 0 ) {
                Serial.println("\nAvailable commands:");
                Serial.println("    control - Start interactive control of the robot\r");
                Serial.println("    move - Move the robot a specified distance\r");
                Serial.println("    turn - Turn the robot a specified number of degrees\r");
                Serial.println("    distance - Display the distance to the nearest obstacle in front of the robot\n\r");

            } else if ( memcmp(&currentCommand, &controlCommand, 7) == 0 ) {
                control.start();

            } else if ( memcmp(&currentCommand, &moveCommand, 4) == 0 ) {
                motorController.moveRobot(atoi(currentCommand + 5));

            } else if ( memcmp(&currentCommand, &turnCommand, 4) == 0 ) {
                motorController.turnRobotByDegrees(atoi(currentCommand + 5));

            } else if ( memcmp(&currentCommand, &distanceCommand, 8) == 0 ) {
                interactions.printDistance();

            } else if ( memcmp(&currentCommand, &stopCommand, 4) == 0 ) {
                motorController.stop();

            } else if ( memcmp(&currentCommand, &searchCommand, 6) == 0 ) {
                interactions.search();
            }
        }

    private:
        // Store the known commands in memory to allow comparisons later
        const char helpCommand[5] = "help";
        const char controlCommand[8] = "control";
        const char moveCommand[5] = "move";
        const char turnCommand[5] = "turn";
        const char distanceCommand[9] = "distance";
        const char stopCommand[5] = "stop";
        const char searchCommand[7] = "search";

        Interactions interactions;
        Control control;

        void readToBuffer(char* currentCommand) 
        /*
        * Read an array of chars from the serial interface into a buffer, ending when \n or \r is sent
        */
        {
            do {
                while ( !Serial.available() ) {} // Wait until a command is available
                buffer = Serial.read(); // Read the character
                Serial.print(buffer); // Echo the character back to the serial user

                // For each character we read, store it in the currentCommand buffer
                if ( commandLength < 32 ) {
                    if ( buffer == 8 && commandLength > 0) {
                        commandLength--;
                    } else {
                        currentCommand[commandLength] = buffer;
                        commandLength++;
                    }
                }

            } 
            // Break on newline or carriage return
            while (buffer != 10 && buffer != 13);
            currentCommand[commandLength] = '\0';
        }

    private:
        // Define the buffers to store the current char, current command and it's length
        size_t commandLength = 0;
        char buffer = -1;
        char currentCommand[32] = {0};


};


void setup() 
{
    /*
    * Run the required setup code to begin the robot
    */
    //Serial Port begin
    Serial.begin (9600);

    // Define inputs and outputs
    pinMode(TRIGGER_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    pinMode(FORWARD_LED, OUTPUT);
    pinMode(TURN_LED, OUTPUT);
    pinMode(HAZARD_LED, OUTPUT);

    // Set up motors
    motorL.init();  // Starts PWM @ ~31kHz
    motorR.init();  // Starts PWM @ ~31kHz

    CommandInterface commandInterface;
    
    Serial.print("Robot Connected! Type 'help' for a list of commands\n\n\r");
    commandInterface.console();
  
}

// Unsure if this needs to remain defined, to be tested later today
void loop() 
{

}