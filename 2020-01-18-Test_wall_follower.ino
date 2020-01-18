// Test de développement robot autonome

#include <Servo.h>
//#include <PID_v1.h>

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
int distance_lue = 0;

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
//    1: 'turn left',
//    2: 'follow the wall',

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

  //setup de la pin pour le servo
  myservo.attach(3);

  Serial.begin(9600);
  
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
  distance = detection();
  if(pos >= 0 && pos <= 36){
    regions[0] = distance;
  }
  if(pos > 36 && pos <= 72){
    regions[1] = distance;
  }
  if(pos > 72 && pos <= 108){
    regions[2] = distance;
  }
  if(pos > 108 && pos <= 144){
    regions[3] = distance;
  }
  if(pos > 144 && pos <= 180){
    regions[4] = distance;
  }
}

void change_state(int state)
{
  if(state != state_active){
    state_active = state;
  }
}

//recois la lecture du capteur et prend la décision d'état à adopter.
void take_action()
{
  char state_description[] = "";
  float d = 13;  //distance_min_detection en cm
  //note: dans les conditions tous les fright(1) sont remplacés par right (0)
  if(regions[2] > d && regions[3] > d && regions[0] > d){
        //state_description = 'case 1 - nothing';
        change_state(0);
        Serial.println("case1");
  }
  else if(regions[2] < d && regions[3] > d && regions[0] > d){
      //state_description = 'case 2 - front';
      change_state(1);
      Serial.println("case2");
  }
  else if(regions[2] > d && regions[3] > d && regions[0] < d){
      //state_description = 'case 3 - fright';
      change_state(2);
      Serial.println("case3");
  }
  else if(regions[2] > d && regions[3] < d && regions[0] > d){
      //state_description = 'case 4 - fleft';
      change_state(0);
      Serial.println("case4");
  }
  else if(regions[2] < d && regions[3] > d && regions[0] < d){
      //state_description = 'case 5 - front && fright';
      change_state(1);
      Serial.println("case5");
  }
  else if(regions[2] < d && regions[3] < d && regions[0] > d){
      //state_description = 'case 6 - front && fleft';
      change_state(1);
      Serial.println("case6");
  }
  else if(regions[2] < d && regions[3] < d && regions[0] < d){
      //state_description = 'case 7 - front && fleft && fright';
      change_state(1);
      Serial.println("case7");
  }
  else if(regions[2] > d && regions[3] < d && regions[0] < d){
      //state_description = 'case 8 - fleft && fright';
      change_state(0);
      Serial.println("case8");
  }
  else{
      //state_description[] = 'unknown case';
  }  
}

void find_wall()
{
  //Serial.println("find wall");
  v_robot(150,-0.5);
}

void turn_left()
{
  //Serial.println("turn left");
  v_robot(0,1);
}

void follow_the_wall()
{
  //Serial.println("follow the wall");
  v_robot(150,0);
}


//fonction du detecteur qui renvoie la distance en cm
int detection()
{
  // defines variables
  long duration;
  int distance;
  
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
void v_robot(int linear_x, int angular_z)
{
  //variables du robot
  float d = 132; //distance entre les roue en mm
  float VtoV = 0.64; //facteur de conversion entre la vitesse en mm/s et la vitesse en valeur [0,255] 128/200
  int corrected_command_Vleft = (int) linear_x - (angular_z*d*VtoV)/2;
  int corrected_command_Vright = (int) linear_x + (angular_z*d*VtoV)/2;

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

//void keepTheLine()
//{
//  int vAbase = 100;
//  int vBbase = 80;
//  int vAcorrection = 0;
//  int vA;
//
//  vAcorrection = -(Setpoint - Output) / 10 * 30;
//  vA = vAbase + vAcorrection;
//  // turn on motor A
//  digitalWrite(In1, HIGH);
//  digitalWrite(In2, LOW);
//  // set speed to 150 out 255
//  analogWrite(EnA, vA);
//  // turn on motor B
//  digitalWrite(In3, HIGH);
//  digitalWrite(In4, LOW);
//  // set speed to 150 out 255
//  analogWrite(EnB, 80);
//}

//basically...state machine 
void loop()
{ 
  t_actuel = millis();
  if(t_actuel - t1 >= T){
    t1 = millis();
    if(v_pos == 1){
      i++;
      myservo.write(defined_pos[i]);
      pos = defined_pos[i]; 
    }
    else if(v_pos == 0){
      i--;
      myservo.write(defined_pos[i]);
      pos = defined_pos[i]; 
    }
    if(i <= 0){
      v_pos = 1;
    }
    if(i >= 4){
      v_pos = 0;
    } 
//    Serial.print("R :");
//    Serial.print(regions[0]);
//    Serial.print(" FR:");
//    Serial.print(regions[1]);
//    Serial.print(" F: ");
//    Serial.print(regions[2]);
//    Serial.print(" FL: ");
//    Serial.print(regions[3]);
//    Serial.print(" L: ");
//    Serial.println(regions[4]);
  }
  
  lecture_ultrason();
  take_action();
  if(state_active == 0){
    find_wall();
  }
  else if(state_active == 1){
    turn_left();
  }
  else if(state_active == 2){
    follow_the_wall();
  }
}
