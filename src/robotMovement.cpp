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
const int LEDS[3] = {FORWARD_LED, TURN_LED, HAZARD_LED};


// Console Commands - should maybe use a seperate file for this.
const char helpCommand[5] = "help";
const char controlCommand[8] = "control";
const char moveCommand[5] = "move";
const char turnCommand[5] = "turn";
const char distanceCommand[9] = "distance";

size_t commandLength = 0;
char buffer;


// Ratio between pulse return time and distance in cm
const float DISTANCE_RATIO = 29.1;   

// Define motor connections
UcTTDcMotor motorL(MOTOR_PIN_LF, MOTOR_PIN_LR); 
UcTTDcMotor motorR(MOTOR_PIN_RF, MOTOR_PIN_RR); 
DualMotorControl motorController(&motorL, &motorR, ENCODER_PIN_L, ENCODER_PIN_R); 
long duration;

//Code to get distance without a libary code based on  forum post https://forum.arduino.cc/t/code-to-use-ultrasonic-sensors-without-pulsein-function-or-newping-library/354239
//returns distance from object in front in centermeters
int pollDistance()
{
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
bool hasStopped = false;
char currentCommand;
int currentLed;

void printDistance()
{
    Serial.print("Nearest obstacle: ");
    Serial.print(pollDistance());
    Serial.print("cm\n");
}

void startInteractiveMode()
{
    /*
    * Runs the robot in "interactive" mode - WASD controls, hold shift to move slower.
    */

    Serial.println("Interactive mode: \n- Use WASD to move\n- Hold shift to move slower\n-Press Ctrl+D to exit\n\n");

    

    //Turn leds off
    // This needs to not happen, need to "latch" on or off
    for (int i = 0; i < 2; i++){
        digitalWrite(LEDS[i], LOW);  
    }


    //Stops and cancles all movement if sensors pick up an object within 5cm
    // Should only check this if W pressed
    int distance = pollDistance();
    if(distance <= 10){
        if(!hasStopped){
            hasStopped = true;
            Serial.print("Hazard detected please move");
        }
        motorController.stop();
        Serial.read(); //stops it reading input and excuting it after the hazard is removed
        digitalWrite(HAZARD_LED, HIGH); 
        return;
    }
    
    //Moves Arduino based on WASD
    while (buffer != -1) {
        

        if ( Serial.available() ) {
            // We have a command available. Read it to memory
            buffer = Serial.read();

            int speed = isUpperCase(buffer) ? 60 : 100;
            hasStopped = false;

            if (buffer == 'W' || buffer == 'w') {
                if (currentCommand != 'w') {
                    currentCommand = 'w';
                    motorController.forward(speed);
                    digitalWrite(FORWARD_LED, HIGH); 
                }
            } else if (buffer == 'S' || buffer == 's') {
                if (currentCommand != 's') {
                    currentCommand = 's';
                    motorController.reverse(speed);
                }
            } else if (buffer == 'A' || buffer == 'a') {
                if (currentCommand != 'a') {
                    currentCommand = 'a';
                    motorController.turnLeft(speed);
                    digitalWrite(TURN_LED, HIGH); 
                }
            } else if (buffer == 'D' || buffer == 'd') {
                if (currentCommand != 'd') {
                    currentCommand = 'd';
                    motorController.turnRight(speed);
                    digitalWrite(TURN_LED, HIGH);
                }
            }  else if (buffer == 'p') {
                printDistance();
            }
            
        } else {
            currentCommand = 'n';
            motorController.stop();
        }
        delay(50);
    }
}

void readToBuffer(char* currentCommand) {
    buffer = -1;
    commandLength = 0;

    do {
        while ( !Serial.available() ) {} 
        buffer = Serial.read();
        Serial.print(buffer);
        if ( commandLength < 32 ) {
            if ( buffer == 8 && commandLength > 0) {
                commandLength--;
            } else {
                currentCommand[commandLength] = buffer;
                commandLength++;
            }
        }

    } while (buffer != 10 && buffer != 13);    
}


void console()
{
    char currentCommand[32] = {0};

    Serial.print("> ");
    readToBuffer(currentCommand);


    if ( memcmp(&currentCommand, &helpCommand, 4) == 0 ) {
        Serial.println("Available commands:");
        Serial.println("    control - Start interactive control of the robot");
        Serial.println("    move - Move the robot a specified distance");
        Serial.println("    turn - Turn the robot a specified number of degrees");
        Serial.println("    distance - Display the distance to the nearest obstacle in front of the robot");

    } else if ( memcmp(&currentCommand, &distanceCommand, 8) == 0 ) {
        Serial.print("Closest obstacle: ");
        Serial.print(pollDistance());
        Serial.println("cm\n");

    } else if ( memcmp(&currentCommand, &controlCommand, 7) == 0 ) {
        startInteractiveMode();

    } else if ( memcmp(&currentCommand, &moveCommand, 4) == 0 ) {
        motorController.moveRobot(atoi(currentCommand + 5));

    } else if ( memcmp(&currentCommand, &turnCommand, 4) == 0 ) {
        motorController.turnRobotByDegrees(atoi(currentCommand + 5));

    } else if ( memcmp(&currentCommand, &distanceCommand, 8) == 0 ) {
        printDistance();

    } else {
        Serial.println("Unrecognised command. Type 'help' for a list of commands");
    }
}



void setup() 
{
  //Serial Port begin
  Serial.begin (9600);

  //Define inputs and outputs
  pinMode(TRIGGER_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(FORWARD_LED, OUTPUT);
  pinMode(TURN_LED, OUTPUT);
  pinMode(HAZARD_LED, OUTPUT);

  //Sets up motor
  motorL.init();  // Starts PWM @ ~31kHz
  motorR.init();  // Starts PWM @ ~31kHz

  Serial.print("Robot Connected! Type 'help' for a list of commands\n\n");
}




void loop() 
{
    console();
    delay(50);
}

// Command interface
// Start at prompt robot >
// command interactive:
//      wasd control, shift to sneak, dont let hit obstacles, distance feedback etc
// command move x:
//      move forward x cm
// command turn x:
//      turn x degrees
// command offest [left, right] x:
//      offset pwm of left or right motor by some value between 0 and 10
// command distance:
//      print distance to closest object (or none if above thresh)