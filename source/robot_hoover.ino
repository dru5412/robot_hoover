
  // Vacuum Robot Ver2.0
  // Andrew Brown
  // Open Source, feel free to use, modify, share as you see fit.
  // Last updated 28th Jun 2017
  
  
  #include <NewPing.h>



 //-------------------------- USER PARAMETERS --------------------------

  const float voltageBatCharged = 12.88;            // Voltage measured when battery fully charged 

  const float battCutOffVolts = 10.5;               // Voltage at which to switch off the fan and motors to protect battery
   
  const int minDistance = 6;                        //Mínimun distance of the sensor before turning Default: 6

  int pwmSpeed = 170;                               //PWM output for motor speed, betwen 0(stop) and 255(full speed). Default : 200
   
  int motorDriftOffSet = 5;                         // Helps if constant turning while moving forwards. Due to motor differences. Start about 5% of PWM value
  //-------------------------- END OF USER PARAMETERS --------------------------

  
  boolean control = true;
  boolean stuckMode = false;
  int counter, counter2, bumperState, encoderCheck, switchTurn, stuckTurn, lastTurn;
 
  #define encoderPin 2
  #define motor1Pin1  3
  #define motor1Pin2  5
  #define motor2Pin1  6
  #define motor2Pin2  9
  
  #define leftSonar 7
  #define rightSonar 8
  
  #define fanEnablePin 10
  #define bumperPin 11
  #define ledBattPin 12
  #define ledPin 13
 
  #define batteryPin A1
  
  NewPing sonarLeft(leftSonar, leftSonar, 200);
  
  NewPing sonarRight(rightSonar, rightSonar, 200);

//////////////CODE/////////////
void setup() {
  //Initialize outputs and inputs

  //Fan motor as output
  pinMode(fanEnablePin, OUTPUT);
  //Motor1
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  //Motor2
  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);
  //LED
  pinMode(ledPin, OUTPUT);  
  digitalWrite(ledPin, HIGH);
  pinMode(ledBattPin, OUTPUT);
  digitalWrite(ledBattPin, HIGH);
  
  //INPUTS
  // initialize the pushbutton inputs 
  //Bumper
  pinMode(bumperPin, INPUT_PULLUP); 
  pinMode(encoderPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderPin), encoderReset, CHANGE);

  //Batt
  pinMode(batteryPin, INPUT);
  // Initialize serial
  Serial.begin(115200);    
  
  //Set PWM with offset 
  pwmSpeed = pwmSpeed - abs(motorDriftOffSet);

  
  
  ///////////////////////////////Wait////////////////////////////////////////
  //Wait about 5 s and initialize fan if voltage ok
  waitBlinking(5,1); //5 seconds at 1 Hz
  //Crank (initialize the fan because the voltage drops when cranking)
  if(readBattery(batteryPin)>11.0){
    digitalWrite(fanEnablePin, HIGH); //Turn the Fan ON
    delay(1000); //For 1000ms
    digitalWrite(ledBattPin, LOW);
  }
  else {
     control = false;
     digitalWrite(ledBattPin, HIGH);
    }
} 
//////////Functions To Use //////////


// Called via interupt on the encoderPin. each time the wheel moves.
void encoderReset(){
  encoderCheck =0; 
}


void waitBlinking(int n, int frequency){ // Used to flash the main led
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

//Manipulate direction according the desired movement of the motors

void forwardMotors(int moveTime){  
   analogWrite(motor1Pin1, pwmSpeed - motorDriftOffSet); 
   analogWrite(motor1Pin2, 0); 
   analogWrite(motor2Pin1, pwmSpeed + motorDriftOffSet); 
   analogWrite(motor2Pin2, 0); 
   delay(moveTime);
}

void backRightMotors(int moveTime){ 
   analogWrite(motor1Pin1, 0); 
   analogWrite(motor1Pin2, pwmSpeed-50); 
   analogWrite(motor2Pin1, 0);
   analogWrite(motor2Pin2, 0); 
   delay(moveTime);
}
void backLeftMotors(int moveTime){ 
   analogWrite(motor1Pin1, 0); 
   analogWrite(motor1Pin2, 0); 
   analogWrite(motor2Pin1, 0);
   analogWrite(motor2Pin2, pwmSpeed-50); 
   delay(moveTime);
}
void rightMotors(int moveTime){ 
   analogWrite(motor1Pin1, 0); 
   analogWrite(motor1Pin2, pwmSpeed-50); 
   analogWrite(motor2Pin1, pwmSpeed-50);
   analogWrite(motor2Pin2, 0); 
   delay(moveTime);
}
void leftMotors(int moveTime){ 
   analogWrite(motor1Pin1, pwmSpeed-50); 
   analogWrite(motor1Pin2, 0); 
   analogWrite(motor2Pin1, 0);
   analogWrite(motor2Pin2, pwmSpeed-50); 
   delay(moveTime);
}
void backwardMotors(int moveTime){
   analogWrite(motor1Pin1, 0); 
   analogWrite(motor1Pin2, pwmSpeed+ motorDriftOffSet);
   analogWrite(motor2Pin1, 0); 
   analogWrite(motor2Pin2, pwmSpeed+ motorDriftOffSet); 
   delay(moveTime);
}
void stopMotors(){ 
   analogWrite(motor1Pin1, 0);
   analogWrite(motor1Pin2, 0); 
   analogWrite(motor2Pin1, 0); 
   analogWrite(motor2Pin2, 0); 
}

float  readBattery(int input){
  int readInput;
  float voltage;
  readInput = analogRead(input);
  voltage = (((readInput*4.9)/1000)*voltageBatCharged ) / 5; // resolution of analog input = 4.9mV per Voltage 
  return voltage;
  } 

void batteryControl(int input){
  //Turn everything off in case the battery is low
  float v_battery;
  v_battery = readBattery(input);
  if(v_battery<=battCutOffVolts){ 
    control = false;
    digitalWrite(ledBattPin, HIGH);
    }
  else {
    //Do nothing Convention
    digitalWrite(ledBattPin, LOW);
    }
}
/////////////////////////////////////////////////MAIN CODE//////////////////////////////
void loop(){


    //First read bumper state and distance measurements 
    bumperState = digitalRead(bumperPin);
    float distanceLeft = sonarLeft.ping_cm();
    delay(1);
    float distanceRight = sonarRight.ping_cm(); 

#if DEBUG>0
    Serial.print("left:");
    Serial.print(distanceLeft);
    Serial.print(" right:");
    Serial.println(distanceRight);
    Serial.println(lastTurn);
#endif
  
  // Main loop if control = TRUE.
  if (control){
    
     digitalWrite(ledPin, HIGH); // Enable the main LED to show we are working.


    if (encoderCheck >100) {

      backwardMotors(800);
      if (distanceLeft < distanceRight){        //Turn away from closest side
       rightMotors(1000);
      } else{
       leftMotors(1000);
      }

      if (encoderCheck > 100 ){ //now its moved a bit check the encoder is now moving, if not then stuck!

        control=false;
        stuckMode = true;
         Serial.println("!!!STUCK!!!");
         exit;
      }
      
    }


  if (counter>2){
     backwardMotors(800);
     if(lastTurn==2){ rightMotors(1100);Serial.println("back right");}
     else {leftMotors(1100);Serial.println("back left");}
     counter=0;
     Serial.println("Counter 3");
  }

 if (distanceLeft < minDistance && distanceRight < minDistance){     //Approaching head on.

      backwardMotors(800); //backward delay of 500ms
      Serial.println("wall");
      if (distanceLeft < distanceRight){        //Turn away from closest side
       rightMotors(1000);
      } else{
       leftMotors(1000);
      }
      
    }

    else if (distanceLeft < minDistance) {       //Too close to something on left, move right a bit)

        if (counter ==0){
        backwardMotors(20);
        backLeftMotors(400);
        }
        else if (counter==1){    
          backwardMotors(20);
          backLeftMotors(600);
        }
         counter++;
         counter2=0;
         Serial.println("sense left");
         lastTurn=1;
      
      
    }

    else if (distanceRight < minDistance) {       //Too close to something on right, move left a bit)

     
        if (counter ==0){
        backwardMotors(20);
        backRightMotors(400);
        }
        else if (counter==1){    
          backwardMotors(20);
          backRightMotors(600);
        }
         counter++;
         counter2=0;
         Serial.println("sense right");
         lastTurn=2;
      
      
    }


     if (bumperState==0){
     
      backwardMotors(500); //backward delay of 500ms
      rightMotors(400);
      Serial.print("  bump ");
         
    }
  
      else {
        forwardMotors(2);
        counter2 ++;
        encoderCheck ++;
        if (counter2 >200){counter2=0; counter=0; Serial.println("counter reset");}
           }
  
  
      }
  
  else if (!control){
    //If the battery is low, turn everything off
    digitalWrite(fanEnablePin, LOW); //Turn the Fan OFF
    stopMotors();

    if (stuckMode == true){

    digitalWrite(ledPin, HIGH);
    digitalWrite(ledBattPin, LOW);
    delay(200);
    digitalWrite(ledPin, LOW);
    digitalWrite(ledBattPin, HIGH);
    delay(200);
    
    
    } else {
    waitBlinking(1,3);  //blink as warning 3hz in a loop
    Serial.print(" Low Battery! ");
    Serial.println();
    }
  }
}
