#include <Arduino.h>
#include "TeensyTimerTool.h"
#include "QuadEncoder.h"
#include "LiquidCrystal.h"

#define outputA 3
#define outputB 2
// RotaryEncoder
int lastStateA;
int currentStateA;
int counter = 0;
String currentDirectory= "";

const int LED = 13;

uint32_t _mCurrentPosition; // 0 to 4294967295
uint32_t oldPosition = 0;
// uint32_t _mCurrentPosition1;
// uint32_t oldPosition1= 0;

using namespace TeensyTimerTool;
OneShotTimer t4(TMR1);
PeriodicTimer t3(GPT1);

const int ENCODER_A_PIN = 30; // on T41. left the 3rd from SC-Card side
const int ENCODER_B_PIN = 31; // on T41. left the 2rd from SC-Card side

QuadEncoder positionEncoder(1, ENCODER_A_PIN, ENCODER_B_PIN, 1);

// void LED_ON()
// {
//     digitalWriteFast(LED, HIGH); 
//     t4.trigger(10'000);                  // trigger t4 to switch of after 10ms
// }

// void LED_OFF()
// {
//     digitalWriteFast(LED, LOW);

// }
int ledState = LOW;
long blinkCount = 0;
IntervalTimer timer;
// _positionCompareEnable One Complete Cycle of Encoder

int getAngle(){
    return _mCurrentPosition % (2400);  // To get the Angle of Encoder Rotation
}

int getRotation(){
    return _mCurrentPosition / (2400);  // get One Complete Rotation of Encoder
}

void Blinked(){
    if(ledState == LOW && getRotation() == currentStateA){
        ledState = HIGH;
        blinkCount = blinkCount +1;
    }else{
        ledState = LOW;
    }
    digitalWrite(LED, ledState);

}

void setup(){

    Serial.begin(9600); // initalising baud rate
    while (!Serial && millis() < 5000); // USB connection valid for 5sec after disconnection
   

    lastStateA = digitalRead(outputA);
    //t3.begin(LED_ON, 1'000'000); //10ms


    // Encoder init
    positionEncoder.setConfigInitialPosition();
    positionEncoder.EncConfig.positionInitialValue = 0;
    positionEncoder.init();
    _mCurrentPosition = positionEncoder.read();

    delay(5000);

    pinMode(outputA, INPUT);
    pinMode(outputB, INPUT);
    pinMode(LED, OUTPUT);

    timer.begin(Blinked, 100);
    // timer.begin(getAngle(), 10);
    // timer.begin(getRotation(), 10);

}

void loop(){
    // For Encoder Position
    long blinkCopy; //hold blinkCount copy

    _mCurrentPosition = positionEncoder.read();

    if(oldPosition != _mCurrentPosition){

        Serial.printf("Current Position value: %ld\n", _mCurrentPosition);
        uint16_t Tmp = positionEncoder.getHoldDifference();
        short PosDiff = *((short*)&Tmp);
        Serial.printf("Position difference: %d\n", PosDiff); // Encoder Stop(hold) Position
        // Serial.printf("Position Hold Revolution: %d\n", positionEncoder.getHoldRevolution());
        // Serial.printf("Index: %d\n", positionEncoder.indexCounter);
        Serial.println();
    }

    oldPosition = _mCurrentPosition;

    // For Rotary Encoder

    currentStateA = digitalRead(outputA);

    if(currentStateA != lastStateA && currentStateA == 1) // count the rotation of encoder
    {
        // detect Direction 
        if(digitalRead(outputB) != currentStateA)   
        {
            counter --;
            currentDirectory = "Anticlockwise";
            // t4.begin(LED_ON); 
            // t4.begin(LED_OFF);
            // digitalWriteFast(LED, HIGH);
            // delay(100);
            // digitalWriteFast(LED, LOW);
        }else{
            counter ++;
            currentDirectory = "Clockwise";
            // t4.begin(LED_ON);
            // t4.begin(LED_OFF);
            // digitalWriteFast(LED, HIGH);
            // delay(100);
            // digitalWriteFast(LED, LOW);
        }

    Serial.println();
    Serial.print("Direction: ");
    Serial.print(currentDirectory);
    Serial.print(" | Counter: ");
    Serial.println(counter);
    //Serial.print("Starts Blinking");
    Serial.println();
   
    }
    noInterrupts();
    blinkCopy = blinkCount;
    interrupts();

   // Serial.print("blinkCount: ");
    Serial.println(blinkCopy);
    Serial.print("Position Counter Angle: ");
    Serial.println(getAngle());
    Serial.println();
    Serial.print("Encoder Rotation: ");
    Serial.println(getRotation());
    Serial.println();

    lastStateA = currentStateA;

    delay(100);

}


