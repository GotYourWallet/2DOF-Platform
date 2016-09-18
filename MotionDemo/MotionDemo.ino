//2 DOF Platform actuated using standard MTG 995 Servos @4.8V
//by Philipp Schilcher

//references: Sweep by BARRAGAN & Scott Fitzgerald

#include <Servo.h>

//time for the program to wait before in/decrementing the angle [ms]
#define wait 30
//time r a drawing stroke takes to finish the whole 180 degress
#define drawDelay 1000
//maximum actuation angle of a servo
#define servoRange 180

//2 Servo objects for controlling 2 actual servos in the yaw (lower) and pitch (upper) direction
Servo servoYaw;
Servo servoPitch;

//the current position (angle) of the servo
int posYaw;
int posPitch;


void setup() {
  //connect servos to Arduino using pwm pin 5 and 6
  servoYaw.attach(5);
  servoPitch.attach(6);
}

void loop() {

  //Initialize Positon
  servoPitch.write(0);
  servoYaw.write(0);

  delay(2000);
  
//turn the lower servo in the yaw direction from 0 to 180 degrees
  for (posYaw = 0; posYaw <=180; posYaw += 2) {
    servoYaw.write(posYaw);
    delay(wait); //wait between increments for the servo to reach the requested position
  }

  //turn it back
  for (posYaw = 180; posYaw >= 0; posYaw -= 10) {
    servoYaw.write(posYaw);
    delay(wait);
  }

  delay(1000);
  
  //turn the upper servo in the pitch direction from 0 to 180 degrees
  for (posPitch = 0; posPitch <=80; posPitch += 2) {
    servoPitch.write(posPitch);
    delay(wait); //wait between increments for the servo to reach the requested position
  }

  //turn it back
  for (posPitch = 80; posPitch >= 0; posPitch -= 2) {
    servoPitch.write(posPitch);
    delay(wait);
  }

   servoYaw.write(45);
   delay(500);

     //turn the upper servo in the pitch direction from 0 to 180 degrees
  for (posPitch = 0; posPitch <=80; posPitch += 5) {
    servoPitch.write(posPitch);
    delay(wait); //wait between increments for the servo to reach the requested position
  }

  //turn it back
  for (posPitch = 80; posPitch >= 0; posPitch -= 5) {
    servoPitch.write(posPitch);
    delay(wait);
  }
  
  delay(500);

     servoYaw.write(100);
   delay(500);

     //turn the upper servo in the pitch direction from 0 to 180 degrees
  for (posPitch = 0; posPitch <=80; posPitch += 5) {
    servoPitch.write(posPitch);
    delay(wait); //wait between increments for the servo to reach the requested position
  }

  //turn it back
  for (posPitch = 80; posPitch >= 0; posPitch -= 5) {
    servoPitch.write(posPitch);
    delay(wait);
  }
  
  delay(500);

  //draw a square

  servoYaw.write(160);
  delay(drawDelay);
  servoPitch.write(70);
  delay(drawDelay);
  servoYaw.write(30);
  delay(drawDelay);
  delay(200);
  servoPitch.write(0);
  delay(drawDelay);
  servoYaw.write(160);
  delay(2000);

}
