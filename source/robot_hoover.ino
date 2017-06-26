//------------------------------------------------------------------------------
//
// Robot Hoover, Version 2.0
// By Andrew Brown. 
// GNU Open Licence 2017
// Feel free to use, adapt and share code freely without restriction
//
//------------------------------------------------------------------------------

/* TODO LIST

1. stuck detection, after 4 attempts to turn and still below minimum distance. or just 10 loops without a distance.
2. bumper detection
3. buzzer for alarms?


*/



  #include <NewPing.h>
  
  // GLOBAL DEFINITIONS
  
  #define motorsEnablePin 6
  #define motor1DirPin 2
  #define motor1PulsePin 3
  #define motor2DirPin 4
  #define motor2PulsePin 5
  #define fanEnablePin 7
  #define batteryPin A0
  #define ledPin 13
  #define ledBattPin 1
  
  #define leftSonar 9
  #define rightSonar 10

  NewPing sonarLeft(leftSonar, leftSonar, 100);
  NewPing sonarRight(rightSonar, rightSonar, 100);
  
  #define bumper1 11
  #define bumper2 12
  


  //-------------------------- USER PARAMETERS --------------------------

  const float voltageBatCharged = 12.88;      // Voltage measured when battery fully charged 

  const int minDistance = 10;                    //Mínimun distance of the sensor

  const float battCutOffVolts = 9.8;          // Voltage at which to switch off the fan and motors to protect battery

  const int motorSpeed = 400;                 // Controls Hoover speed, Lower value = faster, between 200-1000 depending on your set up should work
  
  
  //-------------------------- END OF USER PARAMETERS --------------------------

  #define DEBUG 0
  
  unsigned int runMode = 0;                     //run Mode 0=standby, 1=running, 2=stuck, 3=low batt, 4=end program

  float distanceLeft, distanceRight;

  // Variables will change:
  int bumperLeft = 0;                         // variable for reading the pushbutton status
  int bumperRight = 0;                        // variable for reading the pushbutton status

  
  

void setup() {
  
  // Fan Motor
  pinMode(fanEnablePin, OUTPUT);


  //Enable Motors pin
  pinMode(motorsEnablePin, OUTPUT);
  digitalWrite(motorsEnablePin, HIGH);      //Disable Motors at start up
  
  //Motor1
  pinMode(motor1DirPin, OUTPUT);
  pinMode(motor1PulsePin, OUTPUT);
  //Motor2
  pinMode(motor2DirPin, OUTPUT);
  pinMode(motor2PulsePin, OUTPUT);

  //LED
  pinMode(ledPin, OUTPUT);
  pinMode(ledBattPin, OUTPUT);
  digitalWrite(ledBattPin, HIGH); 

  
  //INPUTS
  
  
  //Bumper
  pinMode(bumper1, INPUT_PULLUP); 
  pinMode(bumper2, INPUT_PULLUP); 
  
 
  //Batt
  pinMode(batteryPin, INPUT);


  // START UP Process
  
  // Initialize serial
  Serial.begin(115200); 

  //Wait about 5 s and initialize fan if voltage ok
  waitBlinking(5,1); //5 seconds at 1 Hz

  batteryCheck();

    if (runMode == 1) {digitalWrite(fanEnablePin, HIGH);}     //Switch on Fan


  unsigned long loop_timer = micros() + 4000;                                             //Set the loop_timer variable at the next end loop time
}


// -------- END SET UP -------

void loop() {
    
  if (runMode == 2) {     // Stuck Mode

    //Switch off Fan, Flash Both LED's. Disable Motors
    digitalWrite(fanEnablePin, LOW);
    digitalWrite(motorsEnablePin, HIGH);
    digitalWrite(ledPin, HIGH);
    digitalWrite(ledBattPin, LOW);
    delay(300);
    digitalWrite(ledPin, LOW);
    digitalWrite(ledBattPin, HIGH);
    delay(300);
  }
  

 
  if (runMode == 4) {     // End of Time Limit, slow flash LED for 1 minute

    digitalWrite(fanEnablePin, LOW);
    digitalWrite(motorsEnablePin, HIGH);
    waitBlinking(60,0.5); //5 seconds at 1 Hz
     
  }
  
  if (runMode == 1) {         //MAIN running LOOP
    
    //  Do Distance Measure
    //  Check Bumpers
    //  Do Forwards or Turn
    //  If stuck turn around
    //  If still stuck, go stuck mode.


    //Distance Check
    distanceLeft = sonarLeft.ping_cm();
    distanceRight = sonarRight.ping_cm();    

   
    if (distanceLeft < minDistance && distanceRight < minDistance){     //Approaching head on.

      if (distanceLeft < distanceRight){        //Turn away from cloest side
        moveMotors(1,-1);
      } else{
        moveMotors(-1,1);
      }
      
    }

    else if (distanceLeft < minDistance) {       //Too close to something on left, move right a bit)

      while (distanceLeft < minDistance){
        moveMotors(0,-0.5);
        distanceLeft = sonarLeft.ping_cm();
      }
      
    }

    else if (distanceRight < minDistance) {       //Too close to something on right, move left a bit)

      while (distanceRight < minDistance){
        moveMotors(-0.5,0);
        distanceRight = sonarRight.ping_cm();
      }
      
    } else {

      // Move forwards 2 rotations
      moveMotors(2,2);
    }
    
    
  }
  
  
 if (slow_loop_counter >= 20){

    batteryCheck();

 }

// -------- END LOOP -------


void waitBlinking(int n, int frequency){
  //blink for n seconds at frequency hz
  for (int i=1; i <= n; i++){
    for(int j=1; j<=frequency; j++){
      digitalWrite(ledPin, HIGH);   
      delay((1000/frequency)/2);   //Half time on            
      digitalWrite(ledPin, LOW);   
      delay((1000/frequency)/2);   //Half time off
    }
   } 
}



void  batteryCheck(){
  int readInput, oneRead;
  float voltage;
  
  // Take average of 5 readings 
  for (int i=0; i<5; i++){
    oneRead = analogRead(batteryPin);
    readInput = readInput + oneRead;
  }
  readInput = readInput / 5;  //Take the Average of the 5 readinds
  
  voltage = (((readInput*4.9)/1000)*voltageBatCharged ) / 5; // resolution of analog input = 4.9mV per Voltage 
  
  if (DEBUG==1) {Serial.print(" Battery= ");  Serial.print(voltage);}
  
  if(voltage<=battCutOffVolts){ //battery limit of discharge, 
    runMode = 3;
    digitalWrite(ledBattPin, HIGH);       // Turn On Low Batt LED
    digitalWrite(ledPin, LOW);            // Turn off Run LED
    digitalWrite(motorsEnablePin, HIGH);       // Disable Motors
    
    }
  else {
    if(runMode==0){         //If Battery Ok and in standby mode, switich to ready mode
        runMode=1;
        digitalWrite(ledBattPin, LOW);       // Turn Off Low Batt LED
        } 
    }

  
  } 


  void moveMotors(float left,  float right){

  // Calling this void with 2 variables, allows you to request 1 full rotation of the left or right or both wheels.
  // The left, right variables equal the number of full rotations required .
  // Negative numbers are reverse, positive are forwards direction.

    // For Full Step 1.8deg steppers, 200 steps equals 1 full rotation
    // Change the 200 value if using different stepper motors or steps
    
    unsigned int rotationLeft = abs(left) * 200;
    unsigned int rotationRight = abs(right) * 200;

    // Set Direction

    if (left<0) {digitalWrite(motor1DirPin, LOW);} else {digitalWrite(motor1DirPin, HIGH);}
    if (right<0) {digitalWrite(motor2DirPin, LOW);} else {digitalWrite(motor2DirPin, HIGH);}
    

    //Run motors, number of rotations based on passed variables.
    //This runs if either motor requires a pulse
    // In order to move both motors at same(almost) time, check rotation values are bigger than zero.
    // Then delay low pulse by speed and decrease remaining pulses by 1.
    // Once both rotation values are zero, movement is complete.
    

    if (rotationLeft>0 || rotationRight>0 ){
      
        if (rotationLeft > 0){
          digitalWrite(motor1PulsePin,HIGH); 
          rotationLeft --;
        }
    
        if (rotationRight > 0){
          digitalWrite(motor2PulsePin,HIGH); 
          rotationRight --;
        }
    
          delayMicroseconds(motorSpeed); 
          digitalWrite(motor1PulsePin,LOW); 
          digitalWrite(motor2PulsePin,LOW); 
          delayMicroseconds(motorSpeed); 
    }
  
    }

  
