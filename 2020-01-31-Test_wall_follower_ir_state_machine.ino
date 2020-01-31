// Test de développement robot autonome

#include <Servo.h>
//#include <PID_v1.h>
#include <Wire.h>

const int MPU = 0x68; // MPU6050 I2C address
float AccX, AccY, AccZ;
float GyroX, GyroY, GyroZ;
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
float roll, pitch, yaw;
float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
float elapsedTime, currentTime, previousTime;
int c = 0;
float y = 0;

//variable du pid
float pi = 3.1416;
float command_yaw_rate = 0; //commande de 0 rad/s
float error_yaw_rate = 0; 
float previous_error_yaw_rate = 0;
float pid = 0;
float pid_p = 0;
float pid_i = 0;
float pid_d = 0;
float kp = 2;
float ki = 0.5;
float kd = 0.5;

//Motor Connections
//Change this if you wish to use another diagram
//Moteur A a gauche 
//Moteur B a droite 
const int EnA = 5;
const int EnB = 6;
const int In1 = 8;
const int In2 = 9;
const int In3 = 10;
const int In4 = 11;

//definition des pins et variables pour sensor de distance 
// defines pins numbers
const int trigPin = 12;
const int echoPin = 13;
int previous_right_distance = 0;
const int trigPin2 = 2;
const int echoPin2 = 3;
float distance = 0;
float previous_distance = 0;
int distance_lue2 = 0;


////variables et objets PID
////initial tuning parameters
//double Setpoint, Input, Output;
//double Kp=2, Ki=1, Kd=1;
//PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

//variables wall follower
float regions[5] = {0,0,0,0,0}; //right, fright,front,fleft,left   
int state_active = 0; //variable globale
//state
//    0: 'find the wall',
//    1: 'turn right',
//    2: 'follow the wall',
//    3: 'stop',

// servo object
Servo myservo;
int pos = 0;

//variables loop
int T = 80; //periode de rafraichissement du servo ms
int t_actuel = 0;
int t1 = 0;
bool v_pos = 1; //direction de la rotation 1:0>180, 0:180>0
int defined_pos[5] = {0,54,90,126,180};
int i = 0;
int correction = 0;

void setup()
{
  // All motor control pins are outputs
  pinMode(EnA, OUTPUT);
  pinMode(EnB, OUTPUT);
  pinMode(In1, OUTPUT);
  pinMode(In2, OUTPUT);
  pinMode(In3, OUTPUT);
  pinMode(In4, OUTPUT);

  //setup du sensor de distance 
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input
  pinMode(trigPin2, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin2, INPUT); // Sets the echoPin as an Input

//  //setup de la pin pour le servo
//  myservo.attach(3);

  Serial.begin(9600);
  Wire.begin();                      // Initialize comunication
  Wire.beginTransmission(MPU);       // Start communication with MPU6050 // MPU=0x68
  Wire.write(0x6B);                  // Talk to the register 6B
  Wire.write(0x00);                  // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true);        //end the transmission
  
//  //setup PID
//  myPID.SetOutputLimits(-30, 30);
//  //valeur cible de la distance robot-mur
//  Setpoint = 20;  
//  //turn the PID on
//  myPID.SetMode(AUTOMATIC);
}

//lecture detecteur
void lecture_ultrason()
{ 
  float distance = 0;
  float distance_right = 0;
  regions[1] = detection_avant();
  regions[0] = detection_arriere();
//  if(pos >= 0 && pos <= 36){
//    regions[0] = distance;
//  }
//  if(pos > 36 && pos <= 72){
//    regions[1] = distance;
//  }
//  if(pos > 72 && pos <= 108){
//    regions[2] = distance;
//  }
//  if(pos > 108 && pos <= 144){
//    regions[3] = distance;
//  }
//  if(pos > 144 && pos <= 180){
//    regions[4] = distance;
//  }
}

void change_state(int state)
{
  if(state != state_active){
    //reset pid
    error_yaw_rate = 0;
    previous_error_yaw_rate = 0;
    pid_p = 0;
    pid_i = 0;
    pid_d = 0;
    state_active = state;
  }
}

//recois la lecture du capteur et prend la décision d'état à adopter.
void take_action()
{
  char state_description[] = "";
  float d = 13;  //distance_min_detection en cm
  //note: dans les conditions tous les fright(1) sont remplacés par right (0) 
  if(distance >= 10 && distance <= 23){
     change_state(2);
     v_robot(200,0);
     Serial.println("12-25");
  }
  else if(distance > 23 && distance < 35){
    change_state(1);
    v_robot(0,-pi);
    Serial.println("Turn right");
  }
//  else if(distance < 10 && regions[1] <= 12){
//    change_state(4);
//    v_robot(0,-pi);
//    Serial.println("Turn right");
//  }
  else{
    change_state(3);
    v_robot(0,0);
    Serial.println("Stop");
  }
}

//fonction du detecteur qui renvoie la distance en cm
int detection_avant()
{
  // defines variables
  long duration;
  float distance;
  
  // Clears the trigPin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  
  // Calculating the distance
  distance= duration*0.034/2;
  return distance;
}

//fonction du detecteur qui renvoie la distance en cm
int detection_arriere()
{
  // defines variables
  long duration;
  float distance;
  
  // Clears the trigPin
  digitalWrite(trigPin2, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin2, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin2, LOW);
  
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin2, HIGH);
  
  // Calculating the distance
  distance= duration*0.034/2;
  return distance;
}

void goStraight()   //run both motors in the same direction
{ 
  // turn on motor A
  digitalWrite(In1, HIGH);
  digitalWrite(In2, LOW);
  // set speed to 150 out 255
  analogWrite(EnA, 128);
  // turn on motor B
  digitalWrite(In3, HIGH);
  digitalWrite(In4, LOW);
  // set speed to 150 out 255
  analogWrite(EnB, 128);
}

void stopMoving()
{
  // now turn off motors
  digitalWrite(In1, LOW);
  digitalWrite(In2, LOW);  
  digitalWrite(In3, LOW);
  digitalWrite(In4, LOW);
}

//controle la vitesse linéaire et angulaire du robot (mm/s) et (rad/s)
void v_robot(float linear_x, float angular_z)
{
  //variables du robot
  float d = 132; //distance entre les roue en mm
  float VtoV = 0.64; //facteur de conversion entre la vitesse en mm/s et la vitesse en valeur [0,255] 128/200
  int corrected_command_Vleft = 0;
  int corrected_command_Vright = 0;
  
  //calcul de la commande d'asservissement
  if(!(linear_x == 0 && angular_z == 0)){
    
//    if(elapsedTime > 0.010){
//      error_yaw_rate = angular_z - (distance - previous_distance)*10/(elapsedTime*115);
//    }
//    else{
//      error_yaw_rate = 0;
//    }
    error_yaw_rate = angular_z - GyroZ*pi/180;
    pid_p = kp*error_yaw_rate;
    pid_i = pid_i + ki*error_yaw_rate;
    pid_d = kd*((error_yaw_rate-previous_error_yaw_rate)/elapsedTime);
    pid = pid_p + pid_d + pid_i;
    previous_error_yaw_rate = error_yaw_rate;
    corrected_command_Vleft = (int) linear_x*VtoV - (pid*d*VtoV)/2;
    corrected_command_Vright = (int) linear_x*VtoV + (pid*d*VtoV)/2;
    
    if(angular_z > 0 && linear_x == 0){
      corrected_command_Vleft = 0;
      corrected_command_Vright = (int) (pid*d*VtoV);
    }
    if(angular_z < 0 && linear_x == 0){
      corrected_command_Vleft = (int) (pid*d*VtoV);
      corrected_command_Vright = 0;
    }
  }
  else{
    corrected_command_Vleft = 0;
    corrected_command_Vright = 0;
  }
  
//  Serial.print("Vleft");
//  Serial.print(corrected_command_Vleft);
//  Serial.print(" / ");
//  Serial.print("corrected_command_Vright");
//  Serial.println(corrected_command_Vright);
//  Serial.print("pid");
//  Serial.println(pid);
  
//  int corrected_command_Vleft = (int) linear_x*VtoV - (angular_z*d*VtoV)/2;
//  int corrected_command_Vright = (int) linear_x*VtoV + (angular_z*d*VtoV)/2;
  
  //Controle de la saturation
  if(corrected_command_Vleft > 255){
    corrected_command_Vleft = 255;
  }
  if(corrected_command_Vleft <= 0){
    corrected_command_Vleft = 0;
  }
  if(corrected_command_Vright > 255){
    corrected_command_Vright = 255;
  }
  if(corrected_command_Vright <= 0){
    corrected_command_Vright = 0;
  }

  //controle des moteurs
  // turn on motor A
  digitalWrite(In1, HIGH);
  digitalWrite(In2, LOW);
  // set speed to 150 out 255
  analogWrite(EnA, corrected_command_Vleft);
  // turn on motor B
  digitalWrite(In3, HIGH);
  digitalWrite(In4, LOW);
  // set speed to 150 out 255
  analogWrite(EnB, corrected_command_Vright);
}

//basically...state machine 
void loop()
{ 
  // === Read acceleromter data === //
  Wire.beginTransmission(MPU);
  Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
  //For a range of +-2g, we need to divide the raw values by 16384, according to the datasheet
  AccX = (Wire.read() << 8 | Wire.read()) / 16384.0; // X-axis value
  AccY = (Wire.read() << 8 | Wire.read()) / 16384.0; // Y-axis value
  AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0; // Z-axis value
  // Calculating Roll and Pitch from the accelerometer data
  accAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI) + 2.37; // AccErrorX ~(0.58) See the calculate_IMU_error()custom function for more details
  accAngleY = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI) + 3.04; // AccErrorY ~(-1.58)

  // === Read gyroscope data === //
  previousTime = currentTime;        // Previous time is stored before the actual time read
  currentTime = millis();            // Current time actual time read
  elapsedTime = (currentTime - previousTime) / 1000; // Divide by 1000 to get seconds
  Wire.beginTransmission(MPU);
  Wire.write(0x43); // Gyro data first register address 0x43
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 4 registers total, each axis value is stored in 2 registers
  GyroX = (Wire.read() << 8 | Wire.read()) / 131.0; // For a 250deg/s range we have to divide first the raw value by 131.0, according to the datasheet
  GyroY = (Wire.read() << 8 | Wire.read()) / 131.0;
  GyroZ = (Wire.read() << 8 | Wire.read()) / 131.0;
  // Correct the outputs with the calculated error values
  GyroX = GyroX + 2.10; // GyroErrorX ~(-0.56)
  GyroY = GyroY - 0.59; // GyroErrorY ~(2)
  GyroZ = GyroZ + 0.12; // GyroErrorZ ~ (-0.8)

  // Currently the raw values are in degrees per seconds, deg/s, so we need to multiply by sendonds (s) to get the angle in degrees
  gyroAngleX = gyroAngleX + GyroX * elapsedTime; // deg/s * s = deg
  gyroAngleY = gyroAngleY + GyroY * elapsedTime;
  yaw =  yaw + GyroZ * elapsedTime;

  // Complementary filter - combine acceleromter and gyro angle values
  roll = 0.96 * gyroAngleX + 0.04 * accAngleX;
  pitch = 0.96 * gyroAngleY + 0.04 * accAngleY;
  y = y + (AccY + 0.04) * 9.81 * 100 * pow(elapsedTime, 2); 

  //lecture US
//  lecture_ultrason();
  //lecture et conversion en cm
  distance = analogRead(A1); 
  distance = 0.0001*pow(distance,2) - 0.146*distance + 52.451;
  Serial.println(distance);
  take_action();
  
  previous_distance = distance;
  delay(50);
}
