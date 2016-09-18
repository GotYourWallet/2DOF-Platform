//2 DOF Platform actuated using standard MTG 995 Servos @4.8V
//by Philipp Schilcher

//references: Sweep by BARRAGAN & Scott Fitzgerald

#include <Servo.h>

//time for the program to wait before in/decrementing the angle [ms]
#define wait 30
//time a drawing stroke takes to finish the whole 180 degress
#define drawDelay 3000
//maximum actuation angle of a servo
#define servoRange 180

//2 Servo objects for controlling 2 actual servos in the yaw (lower) and pitch (upper) direction
Servo servoYaw;
Servo servoPitch;

//the current position (angle) of the servo
int posYaw;
int posPitch;

//Draw a stroke in x direction (+-) (bounds left 0 to right 180)
int drawX(int distance) {
  int angle = 0;
  //add the requested offset to the current position
  angle = posYaw+distance;
  
  //check left and right bound
  if (angle <=0) {
    angle = 0;
  }
  else if (angle >= 180) {
    angle =180;
  }

  //turn the servo by the requested distance up to the set bounds.
  servoYaw.write(angle);
  posYaw=angle;
  // It takes "delay" milliseconds for the drawing stroke to finish. Divide the maximum time it could take by the maximum angle the servo could reach. Multiply by the actual angle you want to move. Use abs() to make the delay always positive.
  delay(abs(drawDelay/servoRange*distance));

  return 0;
}

//Draw a stroke in +-y direction (bounds top 0 to bottom 90)

int drawY(int distance) {
  int angle = 0;
  //add the requested offset to the current position
  angle = posPitch+distance;
  
  //check upper and lower bound
  if (angle <=0) {
    angle = 0;
  }
  else if (angle >= 90) {
    angle =90;
  }

  //turn the servo by the requested distance up to the set bounds.
  servoPitch.write(angle);
  posPitch=angle;
  // It takes "delay" milliseconds for the drawing stroke to finish. Divide the maximum time it could take by the maximum angle the servo could reach. Multiply by the actual angle you want to move. Use abs() to make the delay always positive.
  delay(abs(drawDelay/servoRange*distance));

  return 0;
}

void setup() {
  //connect servos to Arduino using pwm pin 5 and 6
  servoYaw.attach(5);
  servoPitch.attach(6);
  //setup LED pin
  pinMode(7,OUTPUT);
}

void loop() {

  
  //Initialize Positon
  servoPitch.write(0);
  posPitch = 0;
  servoYaw.write(0);
  posYaw = 0;

  delay(2000);

  drawX(30);
  delay(1000);
  
  //T
  digitalWrite(7,HIGH);
  drawY(30);
  drawY(-30);
  drawX(-20);
  drawX(40);
  digitalWrite(7,LOW);

  drawX(15);
  delay(1000);
  
  //E
  digitalWrite(7,HIGH);
  drawX(15);
  drawX(-15);
  drawY(15);
  drawX(15);
  drawX(-15);
  drawY(15);
  drawX(15);
  drawX(-15);
  digitalWrite(7,LOW);

  drawY(-30);
  drawX(30);
  delay(1000);
 
  //O (38 by 50)
  digitalWrite(7,HIGH);
  drawX(2);
  drawY(2);
  drawX(4);
  drawY(2);
  drawX(3);
  drawY(3);
  drawX(1);
  drawY(3);
  drawX(1);
  drawY(2);
  
  drawY(5);
  
  drawX(-1);
  drawY(2);
  drawX(-1);
  drawY(3);
  drawX(-3);
  drawY(3);
  drawX(-4);
  drawY(2);
  drawX(-2);
  drawY(2);

  drawX(-2);
  drawY(-2);
  drawX(-4);
  drawY(-2);
  drawX(-3);
  drawY(-3);
  drawX(-1);
  drawY(-3);
  drawX(-1);
  drawY(-2);

  drawY(-5);

  drawX(1);
  drawY(-2);
  drawX(1);
  drawY(-3);
  drawX(3);
  drawY(-3);
  drawX(4);
  drawY(-2);
  drawX(2);
  drawY(-2);
  digitalWrite(7,LOW);

}
