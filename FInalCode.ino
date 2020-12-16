//this is a test program that should contrain the robot to follow a line.

//THE LOOP AT THE BOTTOM RUNS THE WHOLE PROGRAM
//SET THE CONTROLLER GAINS AT THE START on the line Robot Bot(2,0.5,0.5)
//these can be changed, eg for a pure proportional, choose (2,0,0)
//(proportional, integral, derivative)
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <Servo.h> 

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor * myMotorLeft = AFMS.getMotor(1);
Adafruit_DCMotor * myMotorRight = AFMS.getMotor(2);
// Create a servo object 
Servo Servo1;

bool run = false;
int servoStart = 0;
int servoClose = 60;

class Robot {
    public:
        /*ALL PINS */
        int frontLeft = A0; // input pin for FRONT LEFT light sensor
        int frontRight = A1;
        int colourPin = 0;
        int offAxisRight = 4;
        int offAxisLeft = 3;
        int backMiddle = 5;
        int startButtonPin = 2;
        int distanceSensor = A2;
        

        int ledPinFirst = 13;
        int ledPinSecond = 12;
        int ledPinThird = 11;

        static const int NUMBER_OF_SENSOR_POSITIVES = 10;

        /* SENSOR VALUES */
        int frontLeftVal = 0;
        int frontRightVal = 0;
        int farRightVal = 0;
        int rightVals[NUMBER_OF_SENSOR_POSITIVES] = {0};
        int farLeftVal = 0;
        int leftVals[NUMBER_OF_SENSOR_POSITIVES] = {0};
        int backMiddleVal = 0;
        int middleVals[NUMBER_OF_SENSOR_POSITIVES] = {0};
        float distanceFrontVal = 0;
        float distanceVals[NUMBER_OF_SENSOR_POSITIVES] = {0};
        float distanceAvr;
        int colourPinVal = 0;

        /* MOTORS */
        float motorSpeed = 100; //EDIT THIS 
        int motorDiff = 100;
        float lineFollowDampingFactor = 0.9;
        float motorSpeedLeft;
        float motorSpeedRight;
        float speedDifference = 0;
        int pos = 85;  // variable to store the servo position

        /* THRESHOLDS */
        int lineSensorThreshold = 300;
        float distanceSensorThreshold = 460;

        /*DEALING WITH BOXES*/
        int pillPosition = 0;
        bool onTargetBox = false;
        bool clockwise = true;
        bool hasBoxAtm = false;

        enum class PositionList { START_BOX,START,FIRST_JUNCTION,TUNNEL,MAIN_T_JUNCTION,PILL,BLUE_TRACK,BLUE_T,BLUE_CORNER, BLUE_SIDE };
        enum class Directions {TOWARDS_PILL,AWAY_FROM_PILL};
        enum class BoxCol {RED,BLUE,NO_BOX};

        PositionList position = PositionList::START_BOX;//START_BOX;
        Directions direction = Directions::TOWARDS_PILL;//TOWARDS_PILL;
        BoxCol currentBoxCol = BoxCol::NO_BOX;

        //the whole contorl structre
            /*line follow to t junction, the turn left
            if red 
                check other side ()  -> go back round until you hit the t-junction, then carry on until you hit something on the other side
                if red 
                    put it on the left hand side ()  -> pickup a red, 180, cross t- junction, carry on for a short while then place on the left hand side 
                    then go back right, pickup and place blue,
                    come back, head right and pickup and place blue

                    come back and place the left red in the right hand spot, 
                    then go back and plave the last left red in hte clockwise spot
                    then go home
                else if blue 
                    place the blue and check the right hand side again
                    if red:
                        move to left hand side 
                        then come back, go right pick up and place the last blue
                        and then place the left red on the right
                        then place the last red clockwise
                    else 
                        place the blue,
                        come back and place the left red on the right
                        go across and place the left red clockwise on the pill
            else pickup and place blue
                come back and check the left hand side again 
                if red:
                    check other side
                    if red: 
                        place this red on left
                        go back and deal with final blue on right
                        go back and place the first lefthandside red on the right hand side
                        then deal with second red on the left hand side
                else blue:
                    place blue and 
                    place the right red on the right
                    place the clockwise red on the back of the pill.
            */

        float checkAllSensorValues(bool listVals) {
            //Check ALL the sensor values

            frontLeftVal = analogRead(frontLeft);
            frontRightVal = analogRead(frontRight);
            
            //NEW METHOD OF ENSURING RELIABILITY (TLDR:TAKES AT LEAST 5 OF LAST 10 READINGS TO CHANGE)

            int leftSum = 0 ;
            int rightSum = 0;
            int middleSum = 0;
            float distanceSum = 0;
            
            for(int i = 1; i<NUMBER_OF_SENSOR_POSITIVES -1 ; i++ ){
                rightVals[i] = rightVals[i+1];
                leftVals[i] = leftVals[i+1];
                middleVals[i] = middleVals[i+1];
                distanceVals[i] = distanceVals[i+1];

                rightSum += rightVals[i];
                leftSum +=leftVals[i];
                middleSum +=middleVals[i];
                distanceSum += distanceVals[i];
            }
            leftVals[NUMBER_OF_SENSOR_POSITIVES -1] = digitalRead(offAxisLeft) ;
            rightVals[NUMBER_OF_SENSOR_POSITIVES -1] = digitalRead(offAxisRight) ;
            middleVals[NUMBER_OF_SENSOR_POSITIVES -1] = digitalRead(backMiddle) ;
            distanceVals[NUMBER_OF_SENSOR_POSITIVES -1] = analogRead(distanceSensor); 

            leftSum += leftVals[NUMBER_OF_SENSOR_POSITIVES -1];
            rightSum +=rightVals[NUMBER_OF_SENSOR_POSITIVES -1];
            middleSum +=middleVals[NUMBER_OF_SENSOR_POSITIVES -1];
            distanceSum += distanceVals[NUMBER_OF_SENSOR_POSITIVES -1];

            farLeftVal = leftSum > 8 ? 1: 0; 
            farRightVal = rightSum > 8 ? 1: 0; 
            backMiddleVal = middleSum > 8 ? 1: 0; 
            distanceFrontVal = distanceSum / NUMBER_OF_SENSOR_POSITIVES;

            if(listVals){
                // Serial.print("front left val:  ");
                // Serial.println(frontLeftVal);
                // Serial.print("front right val: ");
                // Serial.println(frontRightVal);

                // Serial.print("far right val:");
                // Serial.println(farRightVal);
                // Serial.print("far left val:");
                // Serial.println(farLeftVal);
                // Serial.print("back middle Val: ");
                // Serial.println(backMiddleVal);
                // Serial.print("distanceSensor: ");
                // Serial.println(distanceFrontVal);
                // Serial.println("                                     ");
                // Serial.println("                                     ");
            }
            colourPinVal = digitalRead(colourPin);
        }

        void flashLEDS(){
            static unsigned long timeStart = millis();
            static int state = LOW;
            unsigned long timeNow = millis();
            unsigned long time = timeNow - timeStart;
            if(time > 500) {
                time = 0;
                timeStart = timeNow;
                state = (state == HIGH) ? LOW : HIGH;
            }

            digitalWrite(ledPinSecond,state);

            switch (currentBoxCol){
                case BoxCol::BLUE:
                    digitalWrite(ledPinThird,HIGH);
                    digitalWrite(ledPinFirst,LOW);

                    break;
                case BoxCol::RED:
                    digitalWrite(ledPinThird,LOW);
                    digitalWrite(ledPinFirst,HIGH);
                    break;
                case BoxCol::NO_BOX:
                    digitalWrite(ledPinThird,LOW);
                    digitalWrite(ledPinFirst,LOW);
                    break;
            }
        }

        void onOffSwitch() {
            //sets start program to true at the push of the button
            int buttonState = digitalRead(startButtonPin);
            static bool lockSwitch = false;

            if(run == false){
                while(buttonState == 1 && run == false){
                    int buttonState = digitalRead(startButtonPin);
                    if(buttonState == 0){
                        Serial.println("Begin Program");
                        run = true;
                        lockSwitch = true;
                        return;
                    }
                }
            }
            
            if(buttonState == 0 && lockSwitch == false && run == true){
                Serial.println("Pause Program");
                run = false;
                runMotors(0,0);
                digitalWrite(ledPinFirst, LOW);
                digitalWrite(ledPinSecond, LOW);
                digitalWrite(ledPinThird,LOW);
                lockSwitch = true;
                while(run == false){
                    int buttonState = digitalRead(startButtonPin);
                    if (buttonState == 1 && lockSwitch ==true) {
                        lockSwitch = false;
                    }
                    if(buttonState == 0 && lockSwitch == false){
                        Serial.println("Continue Program");
                        run = true;
                        lockSwitch = true;
                        return;
                    }
                }
            }
            if(buttonState == 1 && lockSwitch == true){
                lockSwitch = false;
            }
        }
        
        void utilityFunction(){
            checkAllSensorValues(false);
            flashLEDS();
            onOffSwitch();
            return;
        }

        void follow(int loops){
            int timer = 0;
            while (timer < loops){
                timer +=1;
                binaryFollowLine();
            }
            return;
        }

        void runMotors(int motorLeftVal,int motorRightVal){
            int motorDirectionLeft = motorLeftVal > 0 ? BACKWARD : FORWARD;
            int motorDirectionRight = motorRightVal > 0 ? BACKWARD : FORWARD;
            if(motorLeftVal == 0 && motorRightVal == 0){
                motorDirectionLeft = 0;
                motorDirectionRight = 0;
            }
            //Serial.println("running");
            myMotorLeft->setSpeed(abs(motorLeftVal));
            myMotorLeft->run(motorDirectionLeft);
            myMotorRight->setSpeed(abs(motorRightVal));
            myMotorRight->run(motorDirectionRight);

            //Serial.println("motors running");
        }

        void binaryFollowLine(bool flashLED = true, int increaseRate = 100) {
            checkAllSensorValues(false);
            onOffSwitch();
            if (flashLED) flashLEDS(); // actually the utility function is contained within binary follow line
            
            //NOTE THIS CODE
            if(frontLeftVal > lineSensorThreshold && frontRightVal > lineSensorThreshold){
                
                speedDifference = 0;
            }  //THIS CODE SHOULD ALWAYS FIX TARGET BOX PROBLEMS. IF IT DOES NOT WORK THEN REMOVE IT
            else if (frontLeftVal > lineSensorThreshold) {
                speedDifference = increaseRate;
            }
            else if (frontRightVal > lineSensorThreshold) {
                speedDifference = -1 * increaseRate;
            }
            
            if(position == PositionList::PILL){
                if(farLeftVal == 1 && farRightVal == 1 && onTargetBox == false){
                    if(clockwise==true){
                        pillPosition += 1;
                        Serial.println("hit front of clockwise target box");
                    } 
                    else{
                        pillPosition -= 1;
                        Serial.println("hit front of anticlockwise target box");
                    }
                    onTargetBox = true;
                }
                if(farLeftVal == 0 && farRightVal == 0 && onTargetBox == true){
                    onTargetBox = false;
                    Serial.println("hit the end of the clockwsie target box");
                }
                if(onTargetBox == true){
                    speedDifference = 0;
                } 
            }

            if(position == PositionList::BLUE_TRACK){
                if(frontLeftVal == 1 && frontRightVal == 1){
                    speedDifference=0;
                }
                if(frontRightVal == 0 && frontLeftVal == 0){
                    speedDifference =0;
                }
            }

            speedDifference *= lineFollowDampingFactor;
            motorSpeedLeft = motorSpeed - speedDifference;
            motorSpeedRight = motorSpeed + speedDifference;
            runMotors(motorSpeedLeft,motorSpeedRight);
            return;
        }

        void turnLeft() {
            Serial.println("Turning Left");
            //WAIT FOR FAR LEFT TO TRIGGER
            runMotors(-1*motorSpeed,1*motorSpeed);
            if(frontLeftVal > lineSensorThreshold ){
                while (frontLeftVal > lineSensorThreshold){
                    utilityFunction();
                }
            }
            while(frontLeftVal < lineSensorThreshold ) utilityFunction();
            
            while(frontRightVal < lineSensorThreshold ) utilityFunction();
            
            follow(100);
            return;
        }

        void turnRight() {
            Serial.println("Turning Right");
            //WAIT FOR FAR LEFT TO TRIGGER
            runMotors(1*motorSpeed,-1*motorSpeed);
            if(frontRightVal > lineSensorThreshold ){
                while (frontRightVal > lineSensorThreshold){
                    utilityFunction();
                }
            }
            while(frontRightVal < lineSensorThreshold )  utilityFunction();
            
            while(frontLeftVal < lineSensorThreshold ) utilityFunction();
            
            follow(100);
            return;
        }

        void turn180(bool anticlockwise = false) {
            Serial.println("Turning 180");
            
            // while(backMiddle == 1 ){
            //     checkAllSensorValues(false);
            //     runMotors(1*motorSpeed,-1*motorSpeed);
            // }
            if(anticlockwise){
                runMotors(-1*motorSpeed,1*motorSpeed);
                while( farLeftVal == 0 ) utilityFunction();
                
                while(frontLeftVal < lineSensorThreshold ) utilityFunction();
                
                while(frontRightVal < lineSensorThreshold ) utilityFunction();
                

            }
            else {
                runMotors(1*motorSpeed,-1*motorSpeed);
                while( farRightVal == 0 ) utilityFunction();
                
                while(frontRightVal < lineSensorThreshold ) utilityFunction();
                
                while(frontLeftVal < lineSensorThreshold ) utilityFunction();
                
            }

            
            return;

        }

        void servosOpen(bool open = true){
            runMotors(0,0);
            if (open){
                for (pos = servoClose; pos >= servoStart; pos -= 1) { // goes from 0 degrees to 180 degrees
                    // in steps of 1 degree
                    Servo1.write(pos);              // tell servo to go to position in variable 'pos'
                    delay(50); 
                    utilityFunction();
                }                      
                Serial.println("opened Claws");
                hasBoxAtm = false;
                currentBoxCol = BoxCol::NO_BOX;
            }   
            else{
                for (pos = servoStart; pos <= servoClose; pos += 1) { // goes from 0 degrees to 180 degrees
                    // in steps of 1 degree
                    Servo1.write(pos);              // tell servo to go to position in variable 'pos'
                    delay(50);   
                    utilityFunction();                    // waits 15ms for the servo to reach the position
                }   
                Serial.println("closed Claws");
                hasBoxAtm = true;
            }
        }

        //used either having just deposited or decided against picking up, tune so that it just misses block on 180
        void reverseAndTwist(){
            int timer = 0;
            Serial.println("reversing");
            while (timer < 600){
                timer +=1;
                utilityFunction();
                runMotors(-1*motorSpeed,-1*motorSpeed);
            }
            Serial.println("finished reversing");
            turn180();
        }

        void placeBox(){
            /*
                reverse a bit
                diposit box
                reverse some more
                set onTargetlocation to zero 
                set currentBox col to no Box
                SET HASBOXATM TO FALSE
                do a 180
            */
            Serial.println("placing box");
            int timer = 0;
            while (timer < 400){
                timer +=1;
                runMotors(-1*motorSpeed,-1*motorSpeed);
                utilityFunction();
            }
            runMotors(0,0);
            servosOpen(true); 
            timer = 0;
            runMotors(-1*motorSpeed,-1*motorSpeed);
            while (timer < 500){
                timer +=1;
                checkAllSensorValues(false);
                flashLEDS();
            }     
            turn180();
            onTargetBox = false;
        }

        void approachAndCheckBoxColour(){
            while(!(distanceFrontVal > 500)) binaryFollowLine();
            
            Serial.println("found box");
            while(currentBoxCol == BoxCol::NO_BOX){
                utilityFunction();
                checkBoxColour();
            }
        }

        
        void headHomeFromTunnel(){
            while(!(position == PositionList::FIRST_JUNCTION)){
                binaryFollowLine();
                if(farRightVal ==1 ){
                    position = PositionList::FIRST_JUNCTION;
                    Serial.println("reached the first junction for the last time, heading back towards the start");
                    int timer = 0;
                    while (timer < 100){
                        runMotors(motorSpeed,motorSpeed);
                        flashLEDS();
                        timer +=1;
                    }
                }
            }
            while(!(position == PositionList::START)){
                binaryFollowLine();
                if(farRightVal ==1 && farLeftVal ==1){
                    position = PositionList::START;
                    Serial.println("reached start, stopping now");
                    follow(1000);
                    runMotors(0,0);
                    Serial.println("stopped, program complete!");
                }
            }
            while(true){
                continue;
            }
        }

        void checkBoxColour(){
            Serial.println("Checking Box Colour");
            static char colour[20];
            strcpy(colour,"blue");
            currentBoxCol = BoxCol::BLUE;
            while(distanceFrontVal > 350 ){
                checkAllSensorValues(false);
                binaryFollowLine(false);
                if(colourPinVal == 1){
                    strcpy(colour,"red");
                    currentBoxCol = BoxCol::RED;
                }
            }
            runMotors(0,0);
            utilityFunction();
            runMotors(0,0);
            //Serial.println(colour);
        }

        void travelFromTunnelToBlueT(){
            // // goes from tunel back to main junction
            while(!(position == PositionList::FIRST_JUNCTION)){
                binaryFollowLine();
                if(farRightVal == 1 ){
                    position = PositionList::FIRST_JUNCTION;
                }
            }
            while(!(position == PositionList::BLUE_TRACK)){
                utilityFunction();
                turnRight();
                position = PositionList::BLUE_TRACK;
                follow(200);  // NEEDED LATENCY !!!!!!!!!!!!!!!!!!!!!!!!!
            }
            while(!(position == PositionList::BLUE_T)){
                binaryFollowLine();
                if(farRightVal == 1 && farLeftVal == 1){
                    position = PositionList::BLUE_T;
                }
            }
            Serial.println("reached blue T");
        }

        void travelFromBlueTracktoTJUNC(){
            while(!(position == PositionList::FIRST_JUNCTION)){
                binaryFollowLine();
                if(farRightVal == 1){
                    position = PositionList::FIRST_JUNCTION;
                    Serial.println("reached first t junction from blue track");
                    follow(300);
                }
            }
            while(!(position == PositionList::MAIN_T_JUNCTION)){
                binaryFollowLine();
                if(farRightVal ==1 && farLeftVal ==1){
                    position = PositionList::MAIN_T_JUNCTION;
                }
            }
        }
        //places first blue box, starts from tunnel and ends on T-junction
        void placeSecondBlueBox(){
            Serial.println("Placing Second Blue Block");
            // // goes from tunel back to main junction
            travelFromTunnelToBlueT();
            Serial.println("reached blue T");
            while(!(position == PositionList::BLUE_SIDE)){
                utilityFunction();
                lineSensorThreshold = 200;
                turnLeft();
                position = PositionList::BLUE_SIDE;
            }
            direction = Directions::AWAY_FROM_PILL;
            while(!(direction == Directions::TOWARDS_PILL)){
                utilityFunction();
                int timer = 0;
                while (timer < 20){
                    timer +=1;
                    runMotors(motorSpeed,motorSpeed);
                    utilityFunction();
                }
                servosOpen(true);
                timer = 0;
                while (timer < 1000){
                    timer +=1;
                    runMotors(-motorSpeed,-motorSpeed);
                    utilityFunction();
                }
                direction = Directions::TOWARDS_PILL;
            }
            Serial.println("finished manual seqence");

            while(!(position == PositionList::BLUE_T)){
                binaryFollowLine();
                if(farLeftVal==1){
                    position = PositionList::BLUE_T;
                }
            }
            Serial.println("Reached Blue T just placed");
            while(!(position == PositionList::BLUE_TRACK)){
                utilityFunction();
                turnLeft();
                lineSensorThreshold = 300;
                position =  PositionList::BLUE_TRACK;
            }
            Serial.println("Reached Blue Track");
            travelFromBlueTracktoTJUNC();
        }

        void placeFirstBlueBox(){
            Serial.println("Placing First Blue Block");
            travelFromTunnelToBlueT();
            //jumps across to other side of the square
            int timer = 0;
            while (timer < 1300){
                timer +=1;
                runMotors(0.65*motorSpeed,1.1*motorSpeed);
                utilityFunction();
            }
            runMotors(0,0);
            servosOpen(true);
            timer = 0;
            while (timer < 1300){
                timer +=1;
                runMotors(-0.65*motorSpeed,-1.1*motorSpeed);
                utilityFunction();
            }
            timer = 0;
            while (timer < 500){
                timer +=1;
                runMotors(-1*motorSpeed,-1*motorSpeed);
                utilityFunction();
            }
            direction = Directions::TOWARDS_PILL;
            
            while(!(position == PositionList::BLUE_TRACK)){
                utilityFunction();
                turn180();
                position = PositionList::BLUE_TRACK;
            }
            travelFromBlueTracktoTJUNC();
        }
        
        void checkOtherSideFromClockwise() {
            Serial.println("Checking other Side From Clockwise");
            while(!(pillPosition == 0)){
                binaryFollowLine();
            }
            crossTInAntiClockDirection();
            approachAndCheckBoxColour();
        }

        void dealWithTwoClockwiseReds(){
            Serial.println("Dealing with Two Reds");
            while(!(position == PositionList::PILL)){
                utilityFunction();
                turnLeft();
                position = PositionList::PILL;
            }
            approachAndCheckBoxColour();

            servosOpen(false); //pick up box
            
            while(!(clockwise==false)){
                utilityFunction();
                turn180();
                clockwise = false;
            }
            while(!(pillPosition == 0)){
                binaryFollowLine();
            }
            crossTInAntiClockDirection();
            while(!(pillPosition == 0)) binaryFollowLine();
            
            while(!(onTargetBox==true)) binaryFollowLine();
            
            while(!(clockwise==true)){
                utilityFunction();
                placeBox();
                clockwise = true;
            }
            while(!(pillPosition == 0)) binaryFollowLine();
            
            crossTInClockwiseDirection();
            approachAndCheckBoxColour();

            servosOpen(false);  //pick up box
            
            while(!(pillPosition == 0)) binaryFollowLine();
            while(!(onTargetBox==true)) binaryFollowLine();
            
            utilityFunction();
            placeBox();
            clockwise = false;
            //NO NEED TO CHECK PILL NOW COS DONE PRETTY MUCH
            while(!(farRightVal == true)) binaryFollowLine();
            
            while(!(position== PositionList::TUNNEL)){
                utilityFunction();
                turnRight();
                position = PositionList::TUNNEL;
            }
            headHomeFromTunnel();
        }
        
        //picks up blue block turns and turns right at T junction (used for placeblue)
        void ClockwisepickUpAndReturnT(){
            Serial.println("Returning to T");
            servosOpen(false); //pick up box
            while(!(clockwise==false)){
                utilityFunction();
                turn180();
                clockwise = false;
            }
            while(!(pillPosition== 0)) binaryFollowLine();
            
            while(!(position == PositionList::MAIN_T_JUNCTION)){
                binaryFollowLine();
                if (farRightVal==1){
                    position = PositionList::MAIN_T_JUNCTION;
                }
            }
            Serial.println("Main T");
            while(!(position == PositionList::TUNNEL)){
                utilityFunction();
                turnRight();
                direction = Directions::AWAY_FROM_PILL;
                position = PositionList::TUNNEL;
            }
            Serial.println("Tunnel");
            follow(2000);
        }

        //picks up blue block turns and turns left at T junction (used for placeblue)
        void AntiClockpickUpAndReturnT(){
            Serial.println("Picking up box then Returning to T from the anticlockwise side");
            servosOpen(false); //pick up box
            clockwise = false;
            turn180(true);
            clockwise = true;

            while(!(pillPosition== 0)) binaryFollowLine();
            
            while(!(position == PositionList::MAIN_T_JUNCTION)){
                binaryFollowLine();
                if (farLeftVal==1){
                    position = PositionList::MAIN_T_JUNCTION;
                }
            }
            while(!(position == PositionList::TUNNEL)){
                utilityFunction();
                turnLeft();
                direction = Directions::AWAY_FROM_PILL;
                position = PositionList::TUNNEL;
            }
            follow(2000);
        }
        
        //picks up red and places it on clockwise side of T junction before turning round and passing T junction
        void placeRedTemporary(){
            Serial.println("Temporarily placing Red Block");
            servosOpen(false); //pick up box
            while(!(clockwise==true)){
                utilityFunction();
                turn180(true);
                clockwise = true;
            }
            while(!(pillPosition== 0)) binaryFollowLine();
            
            crossTInClockwiseDirection();
            follow(400);
            runMotors(0,0);
            servosOpen(true); 
            reverseAndTwist();
            clockwise = false;
            
            crossTInAntiClockDirection();
        }

        //crosses t junction from clockwise dealing with pill position reset
        void crossTInAntiClockDirection(){
            Serial.println("Crossing T Junction in the anticlockwise direction");
            while(!(position == PositionList::MAIN_T_JUNCTION )){
                binaryFollowLine();
                if(farRightVal == 1){
                    position = PositionList::MAIN_T_JUNCTION;
                }
            }
            while(!(position == PositionList::PILL)){
                binaryFollowLine();
                if( farRightVal == 0){
                    position = PositionList::PILL;
                    pillPosition = 0;
                    
                }
            }
            clockwise = false;
        }

        //crosses t junction from anticlockwise dealing with pill position reset
        void crossTInClockwiseDirection(){
            Serial.println("Crossing T Junction in the clockwise direction");
            while(!(position == PositionList::MAIN_T_JUNCTION )){
                binaryFollowLine();
                if(farLeftVal == 1){
                    position = PositionList::MAIN_T_JUNCTION;
                }
            }
            while(!(position == PositionList::PILL)){
                binaryFollowLine();
                if( farLeftVal == 0){
                    position = PositionList::PILL;
                    pillPosition = 0;
                    
                }
            }
            clockwise = true;
        }

        //CLOCK 1 IS RED
        //ANTICLOCK 1 IS RED 
        //ANTICLOKC 2 IS BLUE
        //STARTS FROM TUNNEL
        void testProgram(){
            Serial.println("Running Test Program");
            while(!(position== PositionList::MAIN_T_JUNCTION)){
                binaryFollowLine();
                if(farRightVal ==1 && farLeftVal ==1){
                    position = PositionList::MAIN_T_JUNCTION;
                }
            }
            while(!(position == PositionList::PILL)){
                utilityFunction();
                turnRight();
                position = PositionList::PILL;
                clockwise = false;
            }
            approachAndCheckBoxColour();
            currentBoxCol = BoxCol::RED;
            //JUST CHECKING NOT BLUE BOX
            while(currentBoxCol == BoxCol::BLUE){
                Serial.println("seeing blue in Anti1, must be errror");
            }
            if(currentBoxCol == BoxCol::RED){
                //ANTICLOCK 1 RED
                //TEMPORARY PLACE THEN CHECK ANTI AGAIN
                placeRedTemporary();
                approachAndCheckBoxColour();
                currentBoxCol = BoxCol::BLUE;
                //JUST CHECKING NOT red BOX
                while(currentBoxCol == BoxCol::RED) Serial.println("seeing blue in Anti1, must be errror");
                
                AntiClockpickUpAndReturnT();
                placeFirstBlueBox();
                dealWithTwoClockwiseReds();
            }
        }

        void runProgram(){

            if(position == PositionList::TUNNEL && direction == Directions::TOWARDS_PILL) goto FIRST_TUNNEL;  //THESE GOTO CLAUSES PROVIDE EASY TESTING HOPEFULLY OF SOME BITS
            if(position == PositionList::PILL && clockwise == true) goto CHECK_FIRST_PILL_CLOCKWISE;
            bool comingHomeTest = false;
            if(position == PositionList::TUNNEL && comingHomeTest == true) goto COMING_HOME_FROM_TUNNEL;
            bool placeRedTest = false;
            if(placeRedTest == true){
                position = PositionList::PILL;
                goto PLACE_RED_CLOCKWISE_TEST;
            }
            // testProgram();
            //FIRST CHECKS FIRST ANTICLOCK BLOCK
            // servosOpen(false);
            // placeFirstBlueBox();
            //placeSecondBlueBox();  
            while(!(position == PositionList::START)){
                binaryFollowLine();
                if(farLeftVal == 1 && farRightVal == 1){
                    position = PositionList::START;
                    Serial.println("left the start box");
                    follow(200);  // NEEDED LATENCY !!!!!!!!!!!!!!!!!!!!!!!!!
                }
            }
            while(!(position == PositionList::FIRST_JUNCTION)){
                binaryFollowLine();
                if( farLeftVal == 1){
                    position = PositionList::FIRST_JUNCTION;
                    Serial.println("first junction towards pill no boxes yet");
                    follow(200);  // NEEDED LATENCY !!!!!!!!!!!!!!!!!!!!!!!!!
                }
            }
            while(!(position == PositionList::TUNNEL)){
                binaryFollowLine();
                if( farLeftVal == 0){
                    position = PositionList::TUNNEL;
                    Serial.println("at tunnel on way to pill first time");
                }
            }
            FIRST_TUNNEL: while(!(position == PositionList::MAIN_T_JUNCTION)){
                binaryFollowLine();
                if(farLeftVal == 1 && farRightVal == 1){
                    position = PositionList::MAIN_T_JUNCTION;
                    Serial.println("MTJ first time no box");
                }
            }
            while(!(position == PositionList::PILL)){
                utilityFunction();
                turnLeft();
                position = PositionList::PILL;
                clockwise = true;
                Serial.println("On Pill first time going clockwise");
            }
            CHECK_FIRST_PILL_CLOCKWISE: approachAndCheckBoxColour();
            if(currentBoxCol == BoxCol::BLUE){
                Serial.println("on way to place first clockwise blue");
                //CLOCKWISE 1 IS BLUE
                //PLACES BLUE AND LOOKS CLOCKWISE AGAIN
                ClockwisepickUpAndReturnT();
                placeFirstBlueBox();
                
                while(!(position == PositionList::PILL)){
                    utilityFunction();
                    turnLeft();
                    position = PositionList::PILL;
                    clockwise = true;
                    Serial.println("back on pill, looking for second clockwise");
                }
                approachAndCheckBoxColour();
                if(currentBoxCol == BoxCol::BLUE){
                    Serial.println("heading to place the second clockwise blue");
                    //CLOCKWISE 2 IS BLUE
                    //CLOCKWISE 2 IS BLUE
                    //ANTICLOCK 1 & 2 RED
                    //PLACES SECOND BLUE THEN DEALS WITH REMAINING REDS
                    ClockwisepickUpAndReturnT();
                    placeSecondBlueBox();
                    while(!(position == PositionList::PILL)){
                        binaryFollowLine();
                        turnLeft();
                        position = PositionList::PILL;
                        Serial.println("on pill clockwise looking for first of remaining reds");
                    }
                    approachAndCheckBoxColour();
                    servosOpen(false); //pick up the box
                    Serial.println("picked up first of remaining reds");
                    PLACE_RED_CLOCKWISE_TEST: while(!(pillPosition==1)) binaryFollowLine();     //this line contains a link to the place red test which can run, avoiding one box and then depositing the thingy at the next
                    
                    Serial.println("avoided first pill");
                    while(!(onTargetBox==true)) binaryFollowLine();  // this should be checking for a target box as well
                    
                    Serial.println("hit red target box");
                    while(!(clockwise==false)){
                        utilityFunction();
                        placeBox();
                        Serial.println("placed first of reds, and heading anticlockwise");
                        clockwise = false;
                    }
                    checkOtherSideFromClockwise();
                    servosOpen(false);  //pick up the box
                    turn180();
                    clockwise = true;
                    while(!(pillPosition == 0)){
                        binaryFollowLine();
                    }
                    crossTInClockwiseDirection();
                    Serial.println("crossing pill for the last time to place the last red");
                    while(!(onTargetBox==true)) binaryFollowLine();
                    
                    placeBox();
                    Serial.println("on way home now");
                    clockwise = false;
                    //NO NEED TO CHECK PILL POSITION HERE AS ALREADY COMING HOME
                    while(!(position == PositionList::MAIN_T_JUNCTION)){
                        binaryFollowLine();
                        if (farRightVal==1){
                            position = PositionList::MAIN_T_JUNCTION;
                        }
                    }
                    while(!(position == PositionList::TUNNEL)){
                        utilityFunction();
                        turnRight();
                        direction = Directions::AWAY_FROM_PILL;
                        position = PositionList::TUNNEL;
                    }
                    follow(2000);
                    COMING_HOME_FROM_TUNNEL: headHomeFromTunnel();
                } 
                else{
                    Serial.println("clockwise 2 was red");
                    //CLOCKWISE 2 IS RED
                    //NOW CHECKING OTHER SIDE
                    
                    utilityFunction();
                    reverseAndTwist();
                    clockwise = false;
                    currentBoxCol = BoxCol::NO_BOX;
                    
                    checkOtherSideFromClockwise();
                    if(currentBoxCol == BoxCol::BLUE){
                        //ANTICLOCK 1 IS BLUE
                        //ANTICLOCK 2 IS RED
                        AntiClockpickUpAndReturnT();
                        placeSecondBlueBox();
                        while(!(position == PositionList::PILL)){
                            utilityFunction();
                            turnRight();
                            position = PositionList::PILL;
                            clockwise = false;
                        }
                        approachAndCheckBoxColour();
                        servosOpen(false);  //pick up the box
                        while(!(pillPosition==-1)) binaryFollowLine();
    
                        while(!(onTargetBox==true)) binaryFollowLine();
                        
                        utilityFunction();
                        placeBox();
                        clockwise = true;
                        
                        while(!(pillPosition==0)) binaryFollowLine();
                        
                        crossTInClockwiseDirection();
                        approachAndCheckBoxColour();
                        servosOpen(false);   //pick up the box
                        
                        utilityFunction();
                        turn180();
                        clockwise = false;
                        
                        while(!(pillPosition==0)) binaryFollowLine();
                        
                        crossTInAntiClockDirection();
                        while(!(onTargetBox==true)) binaryFollowLine();
                        
                        utilityFunction();
                        placeBox();
                        clockwise = true;
                        //NO NEED TO CHECK PILL POSITION HERE AS ON WAY HOME
                        while(!(position == PositionList::MAIN_T_JUNCTION)){
                            binaryFollowLine();
                            if (farLeftVal==1){
                                position = PositionList::MAIN_T_JUNCTION;
                            }
                        }
                        while(!(position == PositionList::TUNNEL)){
                            utilityFunction();
                            turnLeft();
                            direction = Directions::AWAY_FROM_PILL;
                            position = PositionList::TUNNEL;
                        }
                        follow(2000);
                        headHomeFromTunnel();
                    } 
                    else{
                        //CLOCKWISE 1 WAS BLUE
                        //CLOCKWISE 2 IS RED
                        //ANTICLOCK 1 IS RED
                        //ANTICLOCK 2 IS BLUE
                        placeRedTemporary();
                        approachAndCheckBoxColour();
                        AntiClockpickUpAndReturnT();
                        placeSecondBlueBox();
                        dealWithTwoClockwiseReds();
                    }
                }
            } 
            else {
                Serial.println("turning round and checking the other side");
                //CLOCKWISE 1 IS RED
                //CHECKING OTHER SIDE
                
                utilityFunction();
                reverseAndTwist();
                clockwise = false;
                currentBoxCol = BoxCol::NO_BOX;
                
                checkOtherSideFromClockwise();
                if(currentBoxCol == BoxCol::BLUE){
                    //CLOCKWISE 1 IS RED
                    //ANTICLOCK 1 IS BLUE
                    AntiClockpickUpAndReturnT();
                    placeFirstBlueBox();
                    while(!(position == PositionList::PILL)){
                        utilityFunction();
                        turnRight();
                        position = PositionList::PILL;
                    }
                    approachAndCheckBoxColour();
                    if(currentBoxCol == BoxCol::BLUE){
                        //CLOCKWISE 1 IS RED
                        //ANTICLOCK 1 WAS BLUE
                        //ANTICLOCK 2 IS BLUE
                        //CLOCKWISE 2 IS RED
                        ClockwisepickUpAndReturnT();
                        placeSecondBlueBox();
                        dealWithTwoClockwiseReds();
                    } 
                    else{
                        //CLOCKWISE 1 IS RED
                        //ANTICLOCK 1 WAS BLUE
                        //ANTICLOCK 2 IS RED
                        //SO CLOCKWISE 2 MUST BE BLUE
                        //TAKING RED ACROSS TO T
                        placeRedTemporary();
                        approachAndCheckBoxColour();
                        AntiClockpickUpAndReturnT();
                        placeSecondBlueBox();
                        dealWithTwoClockwiseReds();
                    }
                }
                else{
                    //CLOCKWISE 1 IS RED
                    //ANTICLOCK 1 IS RED
                    //CLOCKWISE 2 IS BLUE
                    //ANTICLOCK 2 IS BLUE
                    placeRedTemporary();
                    approachAndCheckBoxColour();
                    AntiClockpickUpAndReturnT();
                    placeFirstBlueBox();
                    while(!(position == PositionList::PILL)){
                        utilityFunction();
                        turnRight();
                        position = PositionList::PILL;
                    }
                    approachAndCheckBoxColour();
                    AntiClockpickUpAndReturnT();
                    placeSecondBlueBox();
                    dealWithTwoClockwiseReds();
                }
            }
        }
};

Robot Bot;

void setup() {

    AFMS.begin();
    Serial.begin(9600);
    Servo1.attach(10);

    pinMode(Bot.startButtonPin, INPUT);
    pinMode(Bot.offAxisLeft,INPUT);
    pinMode(Bot.offAxisRight,INPUT);

    pinMode(Bot.ledPinFirst,OUTPUT);
    pinMode(Bot.ledPinSecond,OUTPUT);
    pinMode(Bot.ledPinThird,OUTPUT);

    Servo1.write(servoStart);
    delay(10);
}

void loop() {
    Bot.runProgram();
}
