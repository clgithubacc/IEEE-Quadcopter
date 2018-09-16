/* c_ControlAlgorithm.ino
    Author: Aaron Pycraft, Roger Kassouf
    Contributors: Roger Kassouf, Iwan Martin, Chen Liang, Aaron Pycraft
    Date last modified: 2017-03-04
    Objective: sketch contains functions that convert tx signals into motor
               control signals. 
    INPUTS: tx signals: [Roll, pitch, throttle, yaw]
    OUTPUTS: motor output values [motor1, motor2, motor3, motor4]
    Notes: controlTransfer function below follows closely from the doc
          "Design of a discrete open loop control algorithm.docx"
          
          The control algorithm is written as a series of operations on 
          variables as they transistion from state 0 (raw tx time signals)
          to the final state
*/

//--Constants used throughout algorithm
// kState1Min, kState1Max; need not be declared b/c of forced compilation order
const int kState3Midpoint = (kState1Max-kState1Min)/2; // 100

//--Motor transformation constants
const int kThrottleBound = (kState0Max - kState0Min)/4; // 200
const int kRotationsBound = (kThrottleBound/2)-1; // 100

//--Variables used in controlTransfer
  int T, R, P, Y;   // local copies of the 4 tx signals
  int M1, M2, M3, M4; // temp variables for motor speeds
  int NT, NP, NR, NY; // transformation factors
  double MP, MR;  //Input of pitch and row in degrees
  double setpointp,setpointr,setpointy;
  double output;
  //aggressive and conservative pid for pitch, roll, and yaw
//  double aggKpP=4, aggKiP=0.2, aggKdP=1;
//  double consKpP=1, consKiP=0.05, consKdP=0.25;
//  double aggKpR=4, aggKiR=0.2, aggKdR=1;
//  double consKpR=1, consKiR=0.05, consKdR=0.25;
//  double aggKpY=4, aggKiY=0.2, aggKdY=1;
//  double consKpY=1, consKiY=0.05, consKdY=0.25;

  double consKpP=1, consKiP=0.05, consKdP=0.25;
  double consKpR=1, consKiR=0.05, consKdR=0.25;
  double consKpY=1, consKiY=0.05, consKdY=0.25;

  void PIDCompute();
  void txSignalToAngle();
  //pid
  PID myPIDP(&pitchd, output, &setpointp, consKpP, consKiP, consKdP, DIRECT);
  PID myPIDR(&rolld, output, &setpointr, consKpR, consKiR, consKdR, DIRECT);
  PID myPIDY(&yawd, output, &setpointy, consKpY, consKiY, consKdY, DIRECT);
/* Inputs are txSignal, which enter this function in state 2 
 *  INPUT:  txSignal  = [R2 P2 Y2 T2], 2 denotes state 2 of values
 *  OUTPUT: motorsOut = [M1 M2 M3 M4]
 */
void controlTransfer(const unsigned int *txSignal, unsigned int *motorsOut) {
  // abandon array in favor of more readable operations on the signals
  // the output will be contained in motorsOut anyways
  // probably a bad idea to read/write the same place VERY FAST

  T = txSignal[THROTTLE];
  R = txSignal[ROLL];
  P = txSignal[PITCH];
  Y = txSignal[YAW];

  txSignalToAngle();
//  M1 = kServoMin;
//  M2 = kServoMin;
//  M3 = kServoMin;
//  M4 = kServoMin;
  
  // Transform to state 4 (skip 3); map values from 0 to state 3 resolution
  // to get lower bound of txSignal to be 0 to $STATE3_MAX_VALUE
  if (T>40 && T<=200){
    NT = map(T, 40, 200, 250, 500);
  }else{
    NT=0;
  }

//Sep 16: Add PID
  PIDCompute(); // Compute NP NR NY using PID
// PID output [-100, 100]

  
  //Serial.print("Pitch in degrees is : "); Serial.print(MP); 
  //Serial.print("   Raw in degrees is   :"); Serial.print(MR); Serial.println();
    //DEBUG print N[T-P]
/*      Serial.print("Throttle: "); Serial.print(NT); Serial.print("\t");
  Serial.print("Roll: \t");   Serial.print(NR);     Serial.print("\t");
  Serial.print("Pitch: ");    Serial.print(NP);    Serial.print("\t");
  Serial.print("Yaw: \t");    Serial.print(NY);      Serial.print("\t");
  Serial.println();*/
    
    
    
    // Ensures that no commands are executed which would give invalid Servo numbers.
  // state 4 achieved


  //DEBUG set motors to throttle

//  M1 = NT*4 + kServoMin;
//  M2 = kServoMin;
//  M3 = kServoMin;
//  M4 = kServoMin;
////  M2 = NT*4 + kServoMin;
////  M3 = NT*4 + kServoMin;
////  M4 = NT*4 + kServoMin;
  

//  M1 = NT - NP + NY + kServoMin;
//  M2 = NT - NR - NY + kServoMin;
//  M3 = NT + NP + NY + kServoMin;
//  M4 = NT + NR - NY + kServoMin;

  M1 = NT + NP + NR + NY + kServoMin;
  M2 = NT - NP + NR - NY + kServoMin;
  M3 = NT - NP - NR + NY + kServoMin;
  M4 = NT + NP - NR - NY + kServoMin;
  /*
  M1 = NT + kServoMin;
  M2 = NT + kServoMin;
  M3 = NT + kServoMin;
  M4 = NT + kServoMin;
  */
  
  /* NOTE: This gives a potential value of any motor speed between 1200 and 1600.
  in Servo microseconds. We have calibrated the ESCs for 2000, which indicates a
  difference between 0% and 50% power at any given time. If one desired to use the
  full power range, they would simply multiply {M1, M2, M3, M4} by 2. */
    
  // Step 5: Re-map the speed values to each motor

  //DEBUG Print current motor output
//  Serial.print("Motor Raw: ");
//  Serial.println(NT*4);
  
  motorsOut[0] = M1;
  motorsOut[1] = M2;
  motorsOut[2] = M3;
  motorsOut[3] = M4; 
  
}
void InitializePID(){
  myPIDP.SetMode(AUTOMATIC);
  myPIDR.SetMode(AUTOMATIC);
  myPIDY.SetMode(AUTOMATIC);
  
  myPIDP.SetOutputLimits(-100.0, 100.0);
  myPIDR.SetOutputLimits(-100.0, 100.0);
  myPIDY.SetOutputLimits(-100.0, 100.0);
}
//void PIDControl (double* input, const unsigned int* motorSpeeds, const unsigned int* txSignal,
//        double Kp, double Ki, double Kd, int ControllerDirection){
/**
 * Convert transmitter input to motor speed with PID using PID lib
 * @param output              motor speed as a const unsigned int*
 * @param txSignal            transmitter input as const unsigned int *
 *                            (will be mapped to angle based on 
 *                            predefined range)
 * @param kp,ki,kd            P,I,D values
 * @param ControllerDirection controller direction (DIRECT or REVERSE). 
 *                            See lib for details.
 */
void PIDCompute (){
  //1. update setpoint

  // P R Y ---- PID Function --- > NP NR NY
  myPIDP.Compute();
  myPIDR.Compute();
  myPIDY.Compute();

  NP = -constrain(NP, -NT/2,NT/2)/2;
  NR = constrain(NR , -NT/2,NT/2)/2;
  NY = constrain(NY, -NT/2,NT/2)/2;

//2. choose aggressive/conservative pid
//  double gap = abs(Setpoint-Input); //distance away from setpoint
//  if (gap < 10)
//  {  //we're close to setpoint, use conservative tuning parameters
//    myPID.SetTunings(consKp, consKi, consKd);
//  }
//  else
//  {
//     //we're far from setpoint, use aggressive tuning parameters
//     myPID.SetTunings(aggKp, aggKi, aggKd);
//  }
}

/**
 * Map txSignal to angle based on pre defined pitch row yaw range
 * Result will be stored in setpoint
 * @param txSignal  transmitter input
 * @param setpoint  mapped angle value
 */
void txSignalToAngle(){
  //TODO: update txSignal range in following map functions.
  R = txSignal[ROLL];
  P = txSignal[PITCH];
  Y = txSignal[YAW];
  //Pitch
  setpointp=map(P, 40, 200, pitchMin, pitchMax);
  //Roll
  setpointr=map(R, 0, 200, rollMin, rollMax);
  //Yaw
  setpointy=map(Y, 0, 200, yawMin, yawMax);
}

