/*  ------------------------------------------------------------------
 *  IEEE Quadcopter Drone Arduino Code
 *  ------------------------------------------------------------------
 *  Version: 0.001
 *  Last edited: 2018-03-23
 *  
 *  ------------------------------------------------------------------
 *  VERSION NOMENCLATURE
 *  ------------------------------------------------------------------
 *  MAJOR REVISIONS:
 *  0.XXX - Code changes before a functional code
 *  1.XXX - First functional code that allows the drone to fly
 *  2.XXX, 3.XXX, and so on - Major revisions to the code
 *  
 *  MINOR REVISIONS:
 *  Any XXX denotes minor changes that make a substantial impact to the
 *  overall function of the code itself. Small tweaks which do not
 *  actually change the function of the code should not be upgraded to
 *  a new minor revision.
 *  
 *  ------------------------------------------------------------------
 *  CHANGELOG
 *  ------------------------------------------------------------------
 *  VER     DATE          AUTHOR          DESCRIPTION BELOW DATE
 *  0.001   2018-03-23    Roger Kassouf
 *          Start of the new code. Added all baseline functionalities
 *          from the previous code versions. Includes the following
 *          IDE tabs:
 *          PinDefinitions.h
 *          a_DecodeTransmitterInput
 *          b_MotorControl
 *          c_ControlAlgorithm
 *          Note that due to the circumstances of not having the
 *          transmitter handy, I was unable to test the core function
 *          of this segment of code. However, I see no reason why it
 *          would not work, as it was copied directly from the
 *          old code.
 *  0.002   2018-03-23    Roger Kassouf
 *          Added functions for the IMU which enable sensing of the 
 *          absolute angles for roll and pitch. For yaw, it is not
 *          really dependable, so that can possibly be ignored. The
 *          base functions are presented in another tab, which is
 *          d_IMU. I commented out the TX signal error since in any
 *          other case, the Serial Monitor would be flodded with
 *          outputs. Future versions should commment out old code
 *          serial debugging to see the pitch/roll/yaw IMU outputs.
 *  0.003   2018-07-14    Chen Liang
 *          Fixed yaw reading error. Updated related files. See VSTS
 *          for changes and excluded factors.
 *  0.004   2018-07-21    Chen Liang        
 *          Added PID control algorithm (NOTE: new lib added. See VSTS)
 *  ------------------------------------------------------------------
 *  DESCRIPTION AND IMPORTANT SAFETY INFORMATION
 *  ------------------------------------------------------------------
 *  This code will be used to control the operation of the quadcopter.
 *  The code will do the following:
 *  - The code will accept inputs from the following devices:
 *    - Mode 2 joystick controller inputs via the receiver
 *      - Throttle Command
 *      - Pitch Command
 *      - Roll Command
 *      - Yaw Command
 *    - Sensor data from the MPU-9250 Internal Measurement Unit
 *      - Pitch Angle
 *      - Roll Angle
 *      - Yaw Angle
 *  - The code will control the motion of:
 *    - Each of the four motors on the quadcopter
 *    - The quadcopter as a whole
 *  The code will not:
 *  - Be able to prevent any injury to person or property which is
 *    associated with the operation, proper or improper, of the 
 *    quadcopter or its associated equipment, or prevent any
 *    unexpected interference or disturbances to its operation
 *  - Assure that the operation of the quadcopter does not violate
 *    any laws or statues
 *  Please be careful to ensure that all proper safety and practical
 *  considerations are taken into account before operation.
 *  ------------------------------------------------------------------
 */

#include <Servo.h>
#include <Wire.h>
#include <TimerOne.h>
#include "PinDefinitions.h"
#include "quaternionFilters.h"
#include "MPU9250.h"
#include "constants.h"
#include <PID_v1.h>
MPU9250 myIMU;

//--Global variables
  //--Used as main control signal from tx input to motor speed settings. 
  unsigned int *quadSignal; //although this is global scope; we'll pass as argument
  unsigned int *motorSpeeds; // used as input to motor control
  const bool ENABLE_MOTORS = true;
  bool TxSignalError = false; //true if error exists in Tx
  bool skipControlTransfer = false;
  bool waitForCalibration = false; // will have a waiting procedure at the start if true
  // Definitions of indexes in txSignal. ki for "const index"
  const int THROTTLE = 2; 
  const int PITCH    = 1;
  const int ROLL     = 0;
  const int YAW      = 3;

  
  //initialize PID: NOTE: all parameters must be double(*)
  //PID myPID(quadSignal,motorSpeeds,setpoint,consKp,consKi,consKd,DIRECT);
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  
  IMU_Setup();

  //--Setup transmitter decoding
  clearTxCycleFlag(); // true at the end of each tx signal cycle
  initTransmitterDecoding();
  
  quadSignal = new unsigned int[4];
  motorSpeeds = new unsigned int[4];
  
  if(ENABLE_MOTORS) {
    initMotorControl();
    setMotorsToMin();
  }
  //myPID.SetMode(AUTOMATIC);
  Serial.println("Setup complete");
}

void loop() {
  //1. Get IMU readings and calculate yaw pitch roll.
  IMU_Loop();
  
  //2. Get input from transmitter
  getTxInput(quadSignal);
  if(TxSignalError) {
    // Serial.print("Tx signal lost!..."); // Serial.println(micros());
    setMotorsToMin();
    skipControlTransfer = true;
  } else  {
    //printTxSignals(quadSignal); // uncomment to see tx signals
  }
   
  //3. Tranform Tx signal to motor speed settings.
  if(skipControlTransfer) {
    skipControlTransfer = false;
  } else {
    //controlTransfer(quadSignal, motorSpeeds); 
  }
  //printMotorValues(motorSpeeds);
  
  //--Send signal to motors, if no errors exist && they're enabled
  //--TODO; check that this logical check is working correctly
  if(!TxSignalError && ENABLE_MOTORS) {
    powerMotors(motorSpeeds);
  }
}

//--Probably unnecessary b/c quad will rarely be shutdown in a civil manner.
//  included for best practice.
void destroy() {
  delete quadSignal;
}

//--Prints processed TX input values.
void printTxSignals(unsigned int *txSignal) {
  //--Note: something messes up character encoding when trying to print 
  //  different data types at once. To print, follow following format:
  Serial.print("Throttle: "); Serial.print(txSignal[THROTTLE]); Serial.print("\t");
  Serial.print("Roll: \t");   Serial.print(txSignal[ROLL]);     Serial.print("\t");
  Serial.print("Pitch: ");    Serial.print(txSignal[PITCH]);    Serial.print("\t");
  Serial.print("Yaw: \t");    Serial.print(txSignal[YAW]);      Serial.print("\t");
  Serial.println();
}

//--Prints motor output values.
void printMotorValues(unsigned int *motorsOut) { 
  Serial.print("Motor 1: "); Serial.print(motorsOut[0]); Serial.print('\t');
  Serial.print("Motor 2: "); Serial.print(motorsOut[1]); Serial.print('\t');
  Serial.print("Motor 3: "); Serial.print(motorsOut[2]); Serial.print('\t');
  Serial.print("Motor 4: "); Serial.print(motorsOut[3]); Serial.print('\t');
  Serial.println();
}

//--Utility to quickly set values to zero
void clearQuadSignal() {
  quadSignal[0] = 0;
  quadSignal[1] = 0;
  quadSignal[2] = 0;
  quadSignal[3] = 0;
}
