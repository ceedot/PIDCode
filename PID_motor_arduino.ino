//Libraries:
#include <math.h> 

//============================================
//    Functions
//============================================

void PID_Motors();
void Motors();
void ReadMotorSpeed_1();
void ReadMotorSpeed_2();

//============================================
//    Variables / Constants
//============================================

//---- Define Motor Signals ------------------

// Motor Encoders
uint8_t Motor_Encoder1 = 2;
uint8_t Motor_Encoder2 = 3;
// Motor Direction of Rotation:
uint8_t Motor_Dir = 5;
// PWM Signals to Motors:
uint8_t Motor_PWM = 9;

//--- PID Control Variables ------------------

// Time 
volatile unsigned long Time = micros();
volatile unsigned long Prev_Time = micros();
volatile float T_Diff=0;
// Errors from Current and Previous Velocity Measurements
volatile double Stored_Err[3]={0};
// PWM Values
volatile double PWM=0;
// Desired Velocity
volatile float Desired_Vel=1; 
// Measured Velocity
volatile double Motor_Vel=0;
// Accumulated Encoder Reads Over Each Sampling Period
volatile float Reads=0;
// Direction of Wheel
volatile bool Dir=0;

// PID Constants
volatile double Kp = 10;
volatile double Ki = 200;
volatile double Kd = 0;

//============================================
//    Control Libraries
//============================================

void ReadMotorSpeed_1(){
  //This function is called when the ISR triggers and is used to count encoder signals
  if(digitalRead(Motor_Encoder2)==0){
    Reads++; //Increments the Reads from the Left Wheel's Encoder
  }
  else{
    Reads--; //Decrements the Reads from the Left Wheel's Encoder
  }
}

void ReadMotorSpeed_2(){
  if(digitalRead(Motor_Encoder1)==0){
    Reads--; //Decrements the Reads from the Left Wheel's Encoder
  }
  else{
    Reads++; //Increments the Reads from the Left Wheel's Encoder
  }
}

void PID_Motors(){
  Time=micros();
  //if 100us has passed then sample each motor's velocity
  if(Time>=Prev_Time+10000){
    T_Diff=(Time-Prev_Time)/1000000;
    Prev_Time = Time;
    
    //Measure angular velocity of motor
    Motor_Vel = (Reads*(2.0*3.14/1500)*T_Diff);

    //Prints Angular Velocity of Motor
    Serial.print(Motor_Vel);

    //Set the Reads of the Encoders to zero to start the next sample
    Reads=0;

    //Calculate the Errors for the last 3 samples
    Stored_Err[2]=Stored_Err[1];
    Stored_Err[1]=Stored_Err[0];
    Stored_Err[0]=Desired_Vel-Motor_Vel;

    //Use PID control to determine the new PWM values that are applied to the motor
    PWM += (Kp*(Stored_Err[0]-Stored_Err[1]))+(Ki*Stored_Err[0]*T_Diff)+((Kd*(Stored_Err[0]-2*Stored_Err[1]+Stored_Err[2]))/T_Diff);
    
    //Check for Overflow/Underflow
    if(PWM<=-255){
      PWM=-255;
    }
    else if(PWM>=255){
      PWM=255;
    }
  }
}

void Motors(){
  if(PWM>=0){
    //Motor -> Forward Motion
    digitalWrite(Motor_Dir,0);
    analogWrite(Motor_PWM,(int)abs(PWM));
  }
  else{
    //Motor -> Backwards Motion
    digitalWrite(Motor_Dir,1);
    analogWrite(Motor_PWM,(int)abs(255+PWM));
  }
}

//============================================
//    Initialisation
//============================================

void setup(){
  //Fast PWM Enable
  TCCR2A = _BV(COM2A1) | _BV(COM2B1) | _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS22);

  //Set Up Pins
  pinMode(Motor_Encoder1,INPUT);
  pinMode(Motor_Encoder2,INPUT);
  pinMode(Motor_Dir,OUTPUT);
  pinMode(Motor_PWM,OUTPUT);

  //Set up Interrupts 
  attachInterrupt(INT0, ReadMotorSpeed_1, RISING);
  attachInterrupt(INT1, ReadMotorSpeed_2, RISING);
  
  //Starts Serial Communications
  Serial.begin(9600);
}

//============================================
//    Main Loop for code
//============================================

void loop(){
  PID_Motors();
  Motors();
}
