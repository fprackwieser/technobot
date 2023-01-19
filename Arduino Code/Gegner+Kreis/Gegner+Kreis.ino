#include <Servo.h>
#define SERVO_PIN 11
#define LPT 2  // scan loop coumter
#define BUZZ_PIN 13

#define IN1 7   //Right motor(K1/K2) direction Pin 7
#define IN2 8   //Right motor(K1/K2) direction Pin 8
#define IN3 9   //Left motor(K3/K4) direction Pin 9
#define IN4 10  //Left motor(K3/K4) direction Pin 10
#define ENA 5   //D5 to ENA PWM speed pin for Right motor(K1/K2)
#define ENB 6   //D6 to ENB PWM speed pin for Left motor(K3/K4)

#define Echo_PIN 2  // Ultrasonic Echo pin connect to D2
#define Trig_PIN 3  // Ultrasonic Trig pin connect to D3

//Define L298N Dual H-Bridge Motor Controller Pins
#define speedPinR 5          //  RIGHT PWM pin connect MODEL-X ENA
#define RightMotorDirPin1 7  //Right Motor direction pin 1 to MODEL-X IN1
#define RightMotorDirPin2 8  //Right Motor direction pin 2 to MODEL-X IN2
#define speedPinL 6          // Left PWM pin connect MODEL-X ENB
#define LeftMotorDirPin1 9   //Left Motor direction pin 1 to MODEL-X IN3
#define LeftMotorDirPin2 10  //Left Motor direction pin 1 to MODEL-X IN4


/*From left to right, connect to D3,A1-A3 ,D10*/
#define LFSensor_0 A0  //OLD D3
#define LFSensor_1 A1
#define LFSensor_2 A2
#define LFSensor_3 A3
#define LFSensor_4 A4  //OLD D10

#define FAST_SPEED 100
#define SPEED 70
#define TURN_SPEED 150
#define BACK_SPEED1 150
#define BACK_SPEED2 120
#define MID_SPEED 50
#define SLOW_SPEED 80

int leftscanval, centerscanval, rightscanval, ldiagonalscanval, rdiagonalscanval;
const int distancelimit = 30;      //distance limit for obstacles in front
const int sidedistancelimit = 30;  //minimum distance in cm to obstacles at both sides (the car will allow a shorter distance sideways)
int distance;
int numcycles = 0;

const int turntime = 50;  //Time the robot spends turning (miliseconds)
const int backtime = 50;  //Time the robot spends turning (miliseconds)

int thereis;
int n=0;
Servo head;

void go_Advance()  //motor rotate clockwise -->robot go ahead
{
  digitalWrite(IN4, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN1, LOW);
  analogWrite(speedPinL, 1);
  analogWrite(speedPinR, 1);
}
void go_Back()  //motor rotate counterclockwise -->robot go back
{
  digitalWrite(IN4, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN1, HIGH);
  analogWrite(speedPinL, 1);
  analogWrite(speedPinR, 1);
}
void stop_Stop()  //motor brake -->robot stop
{
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(speedPinL, 1);
  analogWrite(speedPinR, 1);
  set_Motorspeed(0, 0);
}
void go_Right()  //left motor rotate clockwise and right motor rotate counterclockwise -->robot turn right
{
  digitalWrite(IN4, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN1, HIGH);
  analogWrite(speedPinL, 1);
  analogWrite(speedPinR, 1);
  set_Motorspeed(SPEED, 0);
}
void go_Left()  //left motor rotate counterclockwise and right motor rotate clockwise -->robot turn left
{
  digitalWrite(IN4, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN1, LOW);
  analogWrite(speedPinL, 1);
  analogWrite(speedPinR, 1);
  set_Motorspeed(0, SPEED);
}
/*set motor speed */
void set_Motorspeed(int lspeed, int rspeed)  //change motor speed
{
  analogWrite(ENB, lspeed);  //lspeed:0-255
  analogWrite(ENA, rspeed);  //rspeed:0-255
}

/*motor control*/
void go_Advance1(void)  //Forward
{
  digitalWrite(RightMotorDirPin1, LOW);
  digitalWrite(RightMotorDirPin2, HIGH);
  digitalWrite(LeftMotorDirPin1, LOW);
  digitalWrite(LeftMotorDirPin2, HIGH);
  analogWrite(speedPinL, 1);
  analogWrite(speedPinR, 1);
}
void go_Left1(int t = 0)  //Turn left
{
  digitalWrite(RightMotorDirPin1, LOW);
  digitalWrite(RightMotorDirPin2, HIGH);
  digitalWrite(LeftMotorDirPin1, HIGH);
  digitalWrite(LeftMotorDirPin2, LOW);
  analogWrite(speedPinL, 0);
  analogWrite(speedPinR, 200);
  delay(t);
}
void go_Right1(int t = 0)  //Turn right
{
  digitalWrite(RightMotorDirPin1, HIGH);
  digitalWrite(RightMotorDirPin2, LOW);
  digitalWrite(LeftMotorDirPin1, LOW);
  digitalWrite(LeftMotorDirPin2, HIGH);
  analogWrite(speedPinL, 200);
  analogWrite(speedPinR, 0);
  delay(t);
}
void go_Back1(int t = 0)  //Reverse
{
  digitalWrite(RightMotorDirPin1, HIGH);
  digitalWrite(RightMotorDirPin2, LOW);
  digitalWrite(LeftMotorDirPin1, HIGH);
  digitalWrite(LeftMotorDirPin2, LOW);
  analogWrite(speedPinL, 200);
  analogWrite(speedPinR, 200);
  delay(t);
}
void stop_Stop1()  //Stop
{
  digitalWrite(RightMotorDirPin1, LOW);
  digitalWrite(RightMotorDirPin2, LOW);
  digitalWrite(LeftMotorDirPin1, LOW);
  digitalWrite(LeftMotorDirPin2, LOW);
}
/*set motor speed */
void set_Motorspeed1(int speed_L, int speed_R) {
  analogWrite(speedPinL, speed_L);
  analogWrite(speedPinR, speed_R);
}


int watch() {
  long echo_distance;
  digitalWrite(Trig_PIN, LOW);
  delayMicroseconds(5);
  digitalWrite(Trig_PIN, HIGH);
  delayMicroseconds(15);
  digitalWrite(Trig_PIN, LOW);
  echo_distance = pulseIn(Echo_PIN, HIGH);
  echo_distance = echo_distance * 0.01657;  //how far away is the object in cm
  Serial.println((int)echo_distance);
  return round(echo_distance);
}

String watchsurrounding() {
  /*  obstacle_status is a binary integer, its last 5 digits stands for if there is any obstacles in 5 directions,
 *   for example B101000 last 5 digits is 01000, which stands for Left front has obstacle, B100111 means front, right front and right ha
 */

  int obstacle_status = B100000;
  centerscanval = watch();
  if (centerscanval < distancelimit) {

    obstacle_status = obstacle_status | B100;
  }
  head.write(120);
  delay(10);
  ldiagonalscanval = watch();
  if (ldiagonalscanval < distancelimit) {

    obstacle_status = obstacle_status | B1000;
  }
  head.write(170);  //Didn't use 180 degrees because my servo is not able to take this angle
  delay(10);
  leftscanval = watch();
  if (leftscanval < sidedistancelimit) {

    obstacle_status = obstacle_status | B10000;
  }

  head.write(90);  //use 90 degrees if you are moving your servo through the whole 180 degrees
  delay(10);
  centerscanval = watch();
  if (centerscanval < distancelimit) {

    obstacle_status = obstacle_status | B100;
  }
  head.write(40);
  delay(10);
  rdiagonalscanval = watch();
  if (rdiagonalscanval < distancelimit) {

    obstacle_status = obstacle_status | B10;
  }
  head.write(0);
  delay(10);
  rightscanval = watch();
  if (rightscanval < sidedistancelimit) {

    obstacle_status = obstacle_status | 1;
  }
  head.write(90);  //Finish looking around (look forward again)
  delay(10);
  String obstacle_str = String(obstacle_status, BIN);
  obstacle_str = obstacle_str.substring(1, 6);

  return obstacle_str;  //return 5-character string standing for 5 direction obstacle status
}

void auto_avoidance() {

  ++numcycles;
  if (numcycles >= LPT) {  //Watch if something is around every LPT loops while moving forward

    String obstacle_sign = watchsurrounding();  // 5 digits of obstacle_sign binary value means the 5 direction obstacle status
    Serial.print("begin str=");
    Serial.println(obstacle_sign);
    if (obstacle_sign == "10000") {
      Serial.println("SLIT left");
      set_Motorspeed(SPEED, FAST_SPEED);
      go_Advance();
      auto_tracking();

      delay(turntime);

    } else if (obstacle_sign == "00001") {
      Serial.println("SLIT right");
      set_Motorspeed(FAST_SPEED, SPEED);
      go_Advance();
      auto_tracking();

      delay(turntime);

    } else if (obstacle_sign == "11100" || obstacle_sign == "01000" || obstacle_sign == "11000" || obstacle_sign == "10100" || obstacle_sign == "01100" || obstacle_sign == "00100" || obstacle_sign == "01000") {
      Serial.println("hand right");
      go_Left();
      set_Motorspeed(TURN_SPEED, TURN_SPEED);
      auto_tracking();
      delay(turntime);

    } else if (obstacle_sign == "00010" || obstacle_sign == "00111" || obstacle_sign == "00011" || obstacle_sign == "00101" || obstacle_sign == "00110" || obstacle_sign == "01010") {
      Serial.println("hand left");
      go_Right();
      set_Motorspeed(TURN_SPEED, TURN_SPEED);
      auto_tracking();
      delay(turntime);

    }

    else if (obstacle_sign == "01111" || obstacle_sign == "10111" || obstacle_sign == "11111") {
      Serial.println("hand back left");
      go_Advance();
      set_Motorspeed(BACK_SPEED1, BACK_SPEED2);
      auto_tracking();
      delay(backtime);

    } else if (obstacle_sign == "11011" || obstacle_sign == "11101" || obstacle_sign == "11110" || obstacle_sign == "01110") {
      Serial.println("hand back right");
      go_Advance();
      set_Motorspeed(BACK_SPEED2, BACK_SPEED1);
      auto_tracking();
      delay(backtime);

    }
  

    else Serial.println("no handle");
    numcycles = 0;  //Restart count of cycles
  }/* else {
    set_Motorspeed(TURN_SPEED, TURN_SPEED);
    go_Right();  // if nothing is wrong go forward using go() function above.
    delay(backtime);
  }*/

  //else  Serial.println(numcycles);

  distance = watch();              // use the watch() function to see if anything is ahead (when the robot is just moving forward and not looking around it will test the distance in front)
  /*if (distance < distancelimit) {  // The robot will just stop if it is completely sure there's an obstacle ahead (must test 25 times) (needed to ignore ultrasonic sensor's false signals)
    Serial.println("final go back");
    go_Advance();
    set_Motorspeed(BACK_SPEED1, BACK_SPEED2);
    delay(backtime);
    ++thereis;
  }*/
  if (distance > distancelimit) {
    thereis = 0;
  }  //Count is restarted
  if (thereis > 25) {
    Serial.println("final stop");
    thereis = 0;
  }
}

char sensor[5];
/*read sensor value string, 1 stands for black, 0 starnds for white, i.e 10000 means the first sensor(from left) detect black line, other 4 sensors detected white ground */
String read_sensor_values() {
  int sensorvalue = 32;
  sensor[0] = !digitalRead(LFSensor_0);
  sensor[1] = !digitalRead(LFSensor_1);
  sensor[2] = !digitalRead(LFSensor_2);
  sensor[3] = !digitalRead(LFSensor_3);
  sensor[4] = !digitalRead(LFSensor_4);
  sensorvalue += sensor[0] * 16 + sensor[1] * 8 + sensor[2] * 4 + sensor[3] * 2 + sensor[4];

  String senstr = String(sensorvalue, BIN);
  senstr = senstr.substring(1, 6);

  return senstr;
}

boolean flag = false;

void auto_tracking() {
  String sensorval = read_sensor_values();
  Serial.println(sensorval);
  if (sensorval == "11110" || sensorval == "11100" || sensorval == "11101") {
    //The black line is in the left of the car, need  left turn
    go_Left1();  //Turn left
    set_Motorspeed1(MID_SPEED, MID_SPEED);
    n=0;
  }

  if (sensorval == "01111" || sensorval == "10111" || sensorval == "00111") {  //The black line is  on the right of the car, need  right turn
    go_Right1();                                                               //Turn right
    set_Motorspeed1(MID_SPEED, MID_SPEED);
    n=0;
  }

  if (sensorval == "00000" || sensorval == "01011" || sensorval == "10011" || sensorval == "00011" || sensorval == "01101" || sensorval == "00101" || sensorval == "11010" || sensorval == "11001" || sensorval == "11000" || sensorval == "10010" || sensorval == "10000" || sensorval == "10100" || sensorval == "10110") {
    go_Back1();  //The car front touch white line, need to reverse
    set_Motorspeed1(FAST_SPEED, FAST_SPEED);
    n=0;
  }
  if(sensorval == "11111"){
    n=1;
  }
  else n=0;
}

void setup() {
  /******L298N******/
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  pinMode(RightMotorDirPin1, OUTPUT);
  pinMode(RightMotorDirPin2, OUTPUT);
  pinMode(speedPinL, OUTPUT);

  pinMode(LeftMotorDirPin1, OUTPUT);
  pinMode(LeftMotorDirPin2, OUTPUT);
  pinMode(speedPinR, OUTPUT);
  stop_Stop1();  //stop move

  stop_Stop();  //stop move
  /*init HC-SR04*/
  pinMode(Trig_PIN, OUTPUT);
  pinMode(Echo_PIN, INPUT);
  /*init buzzer*/
  pinMode(BUZZ_PIN, OUTPUT);
  digitalWrite(BUZZ_PIN, HIGH);
  digitalWrite(Trig_PIN, LOW);
  /*init servo*/
  head.attach(SERVO_PIN);
  head.write(90);
  delay(1000);
  Serial.begin(9600);
}

void loop() {
  if(n==1) auto_avoidance();
  else auto_tracking();
  Serial.print("n");
  Serial.println(n);
  

}
