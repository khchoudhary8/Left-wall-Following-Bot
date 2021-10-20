//HI THERE THIS IS KUMAR HARSH
//BTECH10457.20
//It is a left wall following bot using two hcsr04 and two bo motors.


#define TRIGGER_PINL  3
#define ECHO_PINL     2
#define vcc   6

#define TRIGGER_PINF  5
#define ECHO_PINF     4

int en1 = 7;
int en2 = 8;
int en3 = 12;
int en4 = 11;
int enA = 9;
int enB = 10;

//condition return values
int NO_OBSTACLE = 0;
int OBSTACLE = 1;

//left distance to follow is 10cm nd front obstacle distance is 15cm
int DISTANCE= 10;
int FRONT_DISTANCE = 15;

int Left_Status = 0;
int Front_Status = 0;

//Obstacle in Left PID values
float Kp_l = 30;
float Ki_l = 0.003;
float Kd_l = 12;

//Obstacle in Front Sharp right turn PID values
float Kp_F = 29;
float Ki_F = 0; 
float Kd_F = 20; 

//variables for left pid
float Error = 0;
int  Correction = 0;
float Integral = 0;
float Derivative = 0;
float LastError = 0;

//variables for front pid
float Error_F = 0;
float Correction_F = 0;
float Integral_F = 0;
float Derivative_F = 0;
float LastError_F = 0;

int LeftTurnSpeed = 0;
int RightTurnSpeed = 0;

//here 0 stands for true and 1 stands for false
int rightTurnBegin = 0;
int leftTurnBegin = 0;
int straightLineBegin = 0;
long Left,Front;
unsigned long minutes = 1000;

void setup() {
//initialize  
  delay(2000);
  Serial.begin(9600);//serial monitor output
  for (int i= 6; i<= 12; i++)
  pinMode(i, OUTPUT);
  digitalWrite(vcc , HIGH);//vcc 5v for front sensor
  pinMode(TRIGGER_PINF, OUTPUT);
  pinMode(TRIGGER_PINL, OUTPUT);
  pinMode(ECHO_PINF ,INPUT);
  pinMode(ECHO_PINL , INPUT);


}

void loop() {
  //code here is running repetatively
 if (millis() < minutes * 25) {
beginning:

    Front_Status = GET_FRONT_STATUS();//what is the status in front

    if(Front_Status == OBSTACLE) {

        //I Initialize WALL only once just in the beginning of right turn
        if (rightTurnBegin == 0)
          INITIALIZE_WALL();
          CONTINUE_WALL_FRONT();

        //right turn flag false, until the next time there arises a right turn
        rightTurnBegin = 1;
        straightLineBegin = 0;

        goto beginning;
    }

    //when a the bot starts following the straight line(until there is a obstacle,initialize the flag to true
    rightTurnBegin = 0;

    Left = readRangeWall();//checking left status
    Error = Left - DISTANCE;//calculate error proportional
    Integral = (Error + Integral);//integral error
    Derivative = (Error - LastError);// derivative error

    Correction = Kp_l * Error + Kd_l * Derivative + Ki_l * Integral;//for left wall follow, pid correction
    Serial.print("Left correction:");
    Serial.print(Correction);
    Serial.print("\n");

    //if error is less than 10, means the robot is following the wall correctly, keep continuing
    if (Error < 10 && Front_Status ==NO_OBSTACLE) {

      //Initialize the wall, just in the beginning of straight line
      if (straightLineBegin == 0)
        INITIALIZE_WALL();

      if(Correction > 127 && Correction > 0)//since the base speed is 128 we want the motor to have values only between
        Correction = 127;// 0-255 so we map 128+- correction so that it always falls in our desired range of pwm 0-255

      if(Correction < -127 && Correction < 0)
        Correction = -127;//l298 handles negative speeds like 255 in +ve  gives max speed but -255 is zero speed and -1 is full speed whith no direction changes whatsoever 

      LeftTurnSpeed = 128 -Correction;
      RightTurnSpeed = 128 +Correction;

      leftTurnBegin = 0;
      straightLineBegin = 1;

    } else if( Front_Status ==NO_OBSTACLE) {
      
      //if error is greater than 10, means the robot has to take left turn and there should not be any obstacle in front

      //Initialize the wall and wall front, just in the beginning of left turn
      if (leftTurnBegin == 0) {
        INITIALIZE_WALL();
        INITIALIZE_WALL_FRONT();
      }

      //PID for left turn
      int speed = 2.8 * Error + 9 * Derivative;//same thing as above just that condition has reversed , the bot is making left turn to get nearer to the wall and eliminate errors

      if (speed > 127 && speed > 0)
        speed = 127;

      if (speed < -127 && speed < 0)
        speed = -127;

      LeftTurnSpeed = 128 - (speed);
      RightTurnSpeed = 128 + (speed);

      leftTurnBegin = 1;
      straightLineBegin = 0;

    }
    analogWrite(enA ,RightTurnSpeed);//setting pwm speed
    analogWrite(enB,LeftTurnSpeed);
    Serial.print("UNDER LEFT CORRECTION");//some debug help using serial montiro
    Serial.print("RMS:");
    Serial.print(RightTurnSpeed);
    Serial.print("\n");
    Serial.print("LMS:");
    Serial.print(LeftTurnSpeed);
     Serial.print("\n");
  
    FORWARD();//sets the l298n to make the wheels right and left both to go forward.
   
   
    LastError = Error;
    INITIALIZE_WALL_FRONT();

}
else{
   digitalWrite(en1,LOW);   
    digitalWrite(en2,LOW);
    digitalWrite(en3, LOW);  
    digitalWrite(en4, LOW);
  }
}



void INITIALIZE_WALL(void) {//just initialization for smooth operation
  Integral = 0;
  Derivative = 0;
  LastError = 0;
}

void INITIALIZE_WALL_FRONT(void) {
  Integral_F = 0;
  Derivative_F = 0;
  LastError_F = 0;
}


//When there is an obstacle in front, take a right turn 

void CONTINUE_WALL_FRONT(void) {//pid for right turn when wall detected at front

        Front = readRangeWall();// function declared below returns the distance in cms from the from sensor
        Error_F = (Front - FRONT_DISTANCE);
        Integral_F = (Error_F + Integral_F);
        Derivative_F = (Error_F -LastError_F);

        Correction_F = Kp_F * Error_F + Kd_F * Derivative_F +Ki_F * Integral_F;
        Serial.print("Front Correction:");//debug help
        Serial.print(Correction_F);
        Serial.print("\n");

        if(Correction_F > 127 && Correction_F > 0)//same as above
          Correction_F = 127;

        if(Correction_F < -127 && Correction_F < 0)
          Correction_F = -127;

        LeftTurnSpeed = 128 - Correction_F;
        RightTurnSpeed = 128 +Correction_F;
        analogWrite(enA ,RightTurnSpeed);
        analogWrite(enB,LeftTurnSpeed);
        Serial.print("UNDER FRONT CORRECTION");
        Serial.print("RMS:");
        Serial.print(RightTurnSpeed);
        Serial.print("\n");
        Serial.print("LMS:");
        Serial.print(LeftTurnSpeed);
        Serial.print("\n");
        FORWARD();
        delay(800);//makes sure of a complete turn 

        LastError_F = Error_F;//for next pid correction caluclation

}


int GET_LEFT_STATUS(void) {// returnss whether left wall is present or not
  Serial.print("Left Distance=");
  Serial.print(readRangeWall());
     if(readRangeWall() < DISTANCE)
       return OBSTACLE;
     else
      return NO_OBSTACLE;
}

int GET_FRONT_STATUS(void) {//whether front obstacle is present or not
  Serial.print("Front Distance=");
  Serial.print(readRangeFront());
     if(readRangeFront() < FRONT_DISTANCE)
       return OBSTACLE;
     else
       return NO_OBSTACLE;
}

int readRangeFront(){// initializes and completes the hcsr04 commands and finally obtains and returns the front sensor value
 long dur;
 digitalWrite(TRIGGER_PINF, LOW); 
 delayMicroseconds(2);
 digitalWrite(TRIGGER_PINF, HIGH);
 delayMicroseconds(10);
 digitalWrite(TRIGGER_PINF, LOW);
 dur = pulseIn(ECHO_PINF, HIGH);
 return (dur/50);
}

int readRangeWall(){// initializes and completes the hcsr04 commands and finally obtains and returns the left sensor value
  long dur;
 digitalWrite(TRIGGER_PINL, LOW); 
 delayMicroseconds(2);
 digitalWrite(TRIGGER_PINL, HIGH);
 delayMicroseconds(10);
 digitalWrite(TRIGGER_PINL, LOW);
 dur = pulseIn(ECHO_PINL, HIGH);
 return (dur/50);
}
  void FORWARD() {//sets the motor driver pins for a forward motion 
    digitalWrite(en1,HIGH);   
    digitalWrite(en2,LOW);
    digitalWrite(en3, HIGH);  
    digitalWrite(en4, LOW);
    }
