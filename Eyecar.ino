// C++ code
//
#include<Servo.h>
#include<SoftwareSerial.h>
#include "NewPing.h"
//Pines del sensor
#define onOffMotor 12
#define TRIGGER_PIN 7
#define ECHO_PIN 8
#define A1 6
#define A2 5
#define B1 3
#define B2 2
#define speedPin 4
#define speed 170
//Centimetros
#define MAX_DISTANCE 500
SoftwareSerial BtSerial(10,11);
Servo motor;
int direcction = 0;
bool stop=false;
int actualPos=45;
NewPing sonar(TRIGGER_PIN,ECHO_PIN,MAX_DISTANCE);
void setup()
{
  motor.attach(9);
  Serial.begin(9600);
  BtSerial.begin(9600);
  motor.write(45);
  pinMode(A1,OUTPUT);
  pinMode(A2,OUTPUT);
  pinMode(B1,OUTPUT);
  pinMode(B2,OUTPUT);
  pinMode(speedPin,OUTPUT);
  pinMode(onOffMotor,OUTPUT);
  digitalWrite(A1, LOW);
  digitalWrite(A2, LOW);
  digitalWrite(B1, LOW);
  digitalWrite(B2, LOW);
  analogWrite(speedPin,speed);
  digitalWrite(onOffMotor,HIGH);
}

int girarDerecha(){
  if(actualPos < 60){
    actualPos += 5;
    motor.write(actualPos);
    delay(50);
  }
  return actualPos;
}
int girarIzquierda(){
  if(actualPos > 30){
    actualPos -= 5;
    motor.write(actualPos);
    delay(50);
  }
  return actualPos;
}
int centrar(){
  while(actualPos!=45){
    if(actualPos>45){
      actualPos-= 5;
      motor.write(actualPos);
      delay(150);
    }
    else{
      actualPos+= 5;
      motor.write(actualPos);
      delay(150);
    }
  }
  return actualPos;
}

void forward() {          //function of forward 
  analogWrite(A1, 255);
  analogWrite(A2, 0);
  analogWrite(B1, 255);
  analogWrite(B2, 0);
}

void backward() {         //function of backward
  analogWrite(A1, 0);
  analogWrite(A2, 210);
  analogWrite(B1, 0);
  analogWrite(B2, 210);
}

void Stop() {              //function of stop
  digitalWrite(A1, LOW);
  digitalWrite(A2, LOW);
  digitalWrite(B1, LOW);
  digitalWrite(B2, LOW);
}

void test(){
  motor.write(45);
  actualPos=45;
  delay(3000);
  motor.write(0);
  delay(3000);
  motor.write(90);
}


void testMotor(){
  forward();
  Serial.println("forward");
  delay(3000);
  //backward();
  //Serial.println("backward");
  //delay(3000);
  Stop();
  Serial.println("Stop");
  delay(3000);
}

void loop()
{
  //motor.write(45);
  if(BtSerial.available()){
    char value = BtSerial.read();
    Serial.write(value);
    Serial.write((int)value-48);
    switch((int)value-48){
      case 0:actualPos= centrar();
        break;
      case 1: actualPos=girarIzquierda();
        break;
      case 2: actualPos=girarDerecha();
        break;
      case 3: forward();
        break;
      case 4 : Stop();
        break;
      case 5: stop=true;
        Serial.write("Stoping");
        break;
    }
  }
  //test();
  //testMotor();
  if(sonar.ping_cm() < 20 && sonar.ping_cm() != 0 ){
    digitalWrite(onOffMotor,LOW);
    Serial.print("Distance = ");
    Serial.print(sonar.ping_cm());
    Serial.println(" cm");
  }
  else{
    digitalWrite(onOffMotor,HIGH);
  }
  while(stop){}

}