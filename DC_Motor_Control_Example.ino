/**
   \file DC_Motor_Control_Example.ino
   \author Eduard Codres, Mario Martinez
   \copyright Manchester Robotics Ltd.
   \date October, 2020
   \brief Example program controlling the speed of a motor.
*/

/**
   \brief Include the libraries to be used
*/
#include "MCR2_MotorDriver.h"
#include "MCR2_Encoder.h"
#include "MCR2_PIDController.h"
#include <HardwareSerial.h>

using namespace std;
HardwareSerial SerialPC(2);  // Assuming UART2, adjust according to your board documentation
#define RXD 16
#define TXD 17
/**
   \brief Define the MotorDriver, Encoder Pins and rotation sign 
    For ESP32 Motor Driver: PWMpin_L=4, Pin A=15, Pin B=18, Sign=[-1,1] // Encoder: PinA=34, PinB=36, Sign=[-1,1]
    For Arduino Motor Driver: PWMpin_L=2, Pin A=3, Pin B=4, Sign=[-1,1] // Encoder: PinA=18, PinB=19, Sign=[-1,1]
*/
/*motor 1 - right*/
  #define PWMpin_R 4
  #define pinA_R   15
  #define pinB_R   18

  #define encoderPinA_R 34
  #define encoderPinB_R 36
  
/*motor 2 - left*/
  #define PWMpin_L 2
  #define pinA_L   13
  #define pinB_L   14

  #define encoderPinA_L 39
  #define encoderPinB_L 35

  #define motorSign -1
  #define encoderSign_R -1
  #define encoderSign_L -1
/**
   @defgroup motor_control Motor Control
   This group defines the global variables to configure the motors, encoders and controller
   @{
*/

/**
   \brief Motor Configuration Variables
*/
int motR_pins[3] = {PWMpin_R, pinA_R, pinB_R};     //Define the Motor Pins
int motR_sign = motorSign;                 //Define the motor rotation sign
int motL_pins[3] = {PWMpin_L, pinA_L, pinB_L};     //Define the Motor Pins
int motL_sign = motorSign;                 //Define the motor rotation sign

/**
   \brief Encoder Configuration Variables
*/
int encoderR_pins[2] = {encoderPinA_R, encoderPinB_R};
float gear_ratio_r = 34;            //Define gear ratio
float encoderR_ticks = 48;          //Number of encoder ticks
int encoderR_sign = -1;             //Sign of the encoder velocity
int encoderL_pins[2] = {encoderPinA_L, encoderPinB_L};
float gear_ratio_l = 34;            //Define gear ratio
float encoderL_ticks = 48;          //Number of encoder ticks
int encoderL_sign = -1;             //Sign of the encoder velocity

/**
   \brief PID Controller Configuration Variables
*/
double Kp_R=0.050;                     //The Kp parameter of the pid controller
double Ti_R=0.08;                    //The Ti parameter of the pid controller
double Td_R=0;                       //The Td parameter of the pid controller
double uR_min=-1;                    //The lower limit of the controller output
double uR_max=1;                     //The upper limit of the controller output
float desired_R=0;

double Kp_L=0.050;                     //The Kp parameter of the pid controller
double Ti_L=0.08;                    //The Ti parameter of the pid controller
double Td_L=0;                       //The Td parameter of the pid controller
double uL_min=-1;                    //The lower limit of the controller output
double uL_max=1;                     //The upper limit of the controller output
float desired_L=0;

int flag=1;
double u=0;

/** @} end of motor_control*/

/**
   \brief Define set up functions and initialise Objects
*/
MotorDriver motor_R, motor_L;
Encoder encoder_R, encoder_L;
PIDController PID_R, PID_L;
void setupMotEnc();
void setupControl();

void setup() {
  setupMotEnc();
  setupControl();
  Serial.begin(9600);
  SerialPC.begin(9600, SERIAL_8N1, RXD, TXD);  // Ensure these pins are valid for UART2 on your board
}

void loop() {
  if (SerialPC.available() > 0) {  // Check if data is available to read from the serial port
    int receivedChar = SerialPC.read();  // Read a character from the serial port

    // Check if the character is a control character (newline or carriage return)
    if (receivedChar != '\n' && receivedChar != '\r') {  
      int receivedInteger = receivedChar;  // Directly convert the received char to integer
      Serial.print("Received Integer: ");
      Serial.println(receivedInteger);  // Print the received integer
      if (receivedInteger == 1){
        Serial.println("Go");
        motor_R.MotorWrite(-0.31); 
        motor_L.MotorWrite(-0.3);   
        }
       else if (receivedInteger == 2){
        Serial.println("Turn right");
        motor_R.MotorWrite(-0.3); 
        motor_L.MotorWrite(0.3);   
        }
       else if (receivedInteger == 3){
        Serial.println("Turn left");
        motor_R.MotorWrite(0.3); 
        motor_L.MotorWrite(-0.3);  
        }
       else if (receivedInteger == 4){
        Serial.println("Make a U-turn");
        }
      else {
        Serial.println("Stop");
        motor_R.MotorWrite(0); 
        motor_L.MotorWrite(0);      
        }
    }
  }
//float omega=350.0;
//desired_R=10*sin(omega*millis()/1e6);                 //Set Point Definition
//encoder_R.ReadSensors();                              // Read Encoders
//u=PID_R.GetControl(desired_R,encoder_R.Get_Speed());  //Get Control
//motor_R.MotorWrite(-10);                                //Motor Write

//desired_L=10*sin(omega*millis()/1e6);                 //Set Point Definition
//encoder_L.ReadSensors();                              // Read Encoders
//u=PID_L.GetControl(desired_L,encoder_L.Get_Speed());  //Get Control
//motor_L.MotorWrite(-10);                                //Motor Write


//Serial.print(desired_R);                              //Print the desired, open "Arduino Serial Plot" to see the results.
//Serial.print("\t");
//Serial.print(encoder_R.Get_Speed());
//Serial.print("\n");
//
//Serial.print(desired_L);                              //Print the desired, open "Arduino Serial Plot" to see the results.
//Serial.print("\t");
//Serial.print(encoder_L.Get_Speed());
//Serial.print("\n");

delay(100);

}


void setupMotEnc()
{
  //Setup the Motor Object
  motor_R.SetSign(motR_sign);                                            //Set up motor sign
  #ifdef ESP32
  motor_R.DriverSetup(motR_pins[0], 0, motR_pins[1], motR_pins[2]);      //Setup the motor pins (PWMpin, Pin A, Pin B)
  #else
  motor_R.DriverSetup(motR_pins[0], motR_pins[1], motR_pins[2]);         //Setup the motor pins (PWMpin, Pin A, Pin B)
  #endif
  motor_R.MotorWrite(0);                                                 //Write 0 velocity to the motor when initialising

  motor_L.SetSign(motL_sign);                                            //Set up motor sign
  #ifdef ESP32
  motor_L.DriverSetup(motL_pins[0], 0, motL_pins[1], motL_pins[2]);      //Setup the motor pins (PWMpin, Pin A, Pin B)
  #else
  motor_L.DriverSetup(motL_pins[0], motL_pins[1], motL_pins[2]);         //Setup the motor pins (PWMpin, Pin A, Pin B)
  #endif
  motor_L.MotorWrite(0);                                                 //Write 0 velocity to the motor when initialising

  //Setup the Encoder Object
  encoder_R.SetTicksPerRev(encoderR_ticks);                              //Set the number of enconder pulses per revolution
  encoder_R.SetGearRatio(gear_ratio_r);                                  //Set the gear ratio
  encoder_R.SetSign(encoderR_sign);                                      //Set the encoder velocity sign
  encoder_R.Encoder_setup(encoderR_pins[0], encoderR_pins[1], 'R');      //Set encoder pins

  encoder_L.SetTicksPerRev(encoderL_ticks);                              //Set the number of enconder pulses per revolution
  encoder_L.SetGearRatio(gear_ratio_l);                                  //Set the gear ratio
  encoder_L.SetSign(encoderL_sign);                                      //Set the encoder velocity sign
  encoder_L.Encoder_setup(encoderL_pins[0], encoderL_pins[1], 'L');      //Set encoder pins
}

void setupControl()
{
  //Setup the PID Control Object
  PID_R.SetParameters(Kp_R,Ti_R,Td_R);                                    //Set up the PID Parameters
  PID_R.SetControlLimits(uR_min,uR_max);                                  //Set control output Limits
  
  PID_L.SetParameters(Kp_L,Ti_L,Td_L);                                    //Set up the PID Parameters
  PID_L.SetControlLimits(uL_min,uL_max);                                  //Set control output Limits
}
