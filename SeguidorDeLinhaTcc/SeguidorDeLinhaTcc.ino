//including the libraries
#include <AFMotor.h>
#include <NewPing.h>
#include <NewTone.h>   
#include <SoftwareSerial.h>
#include <TinyGPS.h>

//defining pins and variables
//ir sensor
#define right A0
#define left A1

//led
#define ledPin A4

//buzzer
#define buzzerPin A5

//defining motors
//Front Right
AF_DCMotor motor1(1, MOTOR12_1KHZ);
//Front Left
AF_DCMotor motor2(2, MOTOR12_1KHZ);
//Back Right
AF_DCMotor motor3(4, MOTOR34_1KHZ);
//Back Left
AF_DCMotor motor4(3, MOTOR34_1KHZ);
//motor speed
#define MOTOR_SPEED 90

//ultrasonic sensor
#define TRIGGER_PIN A2
#define ECHO_PIN A3
#define max_distance 50
NewPing sonar(TRIGGER_PIN, ECHO_PIN, max_distance);

//Porta Serial Bluetooth
SoftwareSerial serialBluetooth(8, 9); //RX,TX

//Porta Serial Gps
//SoftwareSerial serialGps(11, 10); //RX,TX

TinyGPS gps;

int state = 0;
bool isAlertObstacleSended = false;
bool isAgvTurnedOn = false;

void setup() {
  //declaring pin types
  pinMode(left,INPUT);
  pinMode(right,INPUT);
  pinMode(ledPin,OUTPUT);
  pinMode(buzzerPin,OUTPUT);
  serialBluetooth.begin(9600);
  //serialGps.begin(9600);
  Serial.begin(9600);
}

void loop(){
  bool recebido = false;
  while (serialGps.available()){
    char cIn = serialGps.read();
    recebido = gps.encode(cIn);
    Serial.print(recebido);
  }

  if(recebido){
    long latitude, longitude;
    unsigned long idadeInfo;
    gps.get_position(&latitude, &longitude, &idadeInfo);

    if (latitude != TinyGPS::GPS_INVALID_F_ANGLE) {
        Serial.print("Latitude: ");
        Serial.println(float(latitude) / 100000, 6);
    }

     if (longitude != TinyGPS::GPS_INVALID_F_ANGLE) {
        Serial.print("Longitude: ");
        Serial.println(float(longitude) / 100000, 6);
    }

    int ano; 
    byte mes, dia, hora, minuto, segundo, centesimo;
    gps.crack_datetime(&ano, &mes, &dia, &hora, &minuto, &segundo, &centesimo, &idadeInfo);

    Serial.print("Data (GMT): ");
    Serial.print(dia);
    Serial.print("/");
    Serial.print(mes);
    Serial.print("/");
    Serial.println(ano);
    Serial.print("Horario (GMT): ");
    Serial.print(hora - 3);
    Serial.print(":");
    Serial.print(minuto);
    Serial.print(":");
    Serial.print(segundo);
    Serial.print(":");
    Serial.println(centesimo);
  }

  if(serialBluetooth.available()){
    state = serialBluetooth.read();
    Serial.print(state); 
    if(state == '1'){
      digitalWrite(ledPin, HIGH);
    } else if (state == '2'){
      digitalWrite(ledPin, LOW);
    } else if (state == '3'){
      isAgvTurnedOn = true;
    } else if (state == '4'){
      isAgvTurnedOn = false;
    } else if (state == '5'){
      isAlertObstacleSended = false;
    }
  }
  if(hasObjectInPath() && isAgvTurnedOn){
    Stop();
    NewTone(buzzerPin, 900);
    if(!isAlertObstacleSended){
      serialBluetooth.write("Obstaculo Enontrado");
      isAlertObstacleSended = true;
    }
    
    delay(100);
  } else if(digitalRead(left)==0 && digitalRead(right)==0 && isAgvTurnedOn){
    //Move Forward no line detected
    motor1.run(FORWARD);
    motor2.run(FORWARD);
    motor3.run(FORWARD);
    motor4.run(FORWARD);
    motor1.setSpeed(MOTOR_SPEED);
    motor2.setSpeed(MOTOR_SPEED);
    motor3.setSpeed(MOTOR_SPEED);
    motor4.setSpeed(MOTOR_SPEED);
    noNewTone(buzzerPin);
  } else if(digitalRead(left)==1 && digitalRead(right)==0 && isAgvTurnedOn){
    //Line in Left
    motor1.run(BACKWARD);
    motor2.run(FORWARD);
    motor3.run(BACKWARD);
    motor4.run(FORWARD);
    motor1.setSpeed(MOTOR_SPEED);
    motor2.setSpeed(MOTOR_SPEED);
    motor3.setSpeed(MOTOR_SPEED);
    motor4.setSpeed(MOTOR_SPEED);
    noNewTone(buzzerPin);
  } else if(digitalRead(left)==0 && digitalRead(right)==1 && isAgvTurnedOn){
    //Line in Right
    motor1.run(FORWARD);
    motor2.run(BACKWARD);
    motor3.run(FORWARD);
    motor4.run(BACKWARD);
    motor1.setSpeed(MOTOR_SPEED);
    motor2.setSpeed(MOTOR_SPEED);
    motor3.setSpeed(MOTOR_SPEED);
    motor4.setSpeed(MOTOR_SPEED);
    noNewTone(buzzerPin);
  } else if(digitalRead(left)==1 && digitalRead(right)==1 && isAgvTurnedOn){
    //Line in Front
    Stop();
    noNewTone(buzzerPin);
  }
}

bool hasObjectInPath() {
  int distance = getDistance();
  if (distance <= 15) {
    return true;
  } else {
    return false;
  }
}

int getDistance() {
  delay(50);
  int cm = sonar.ping_cm();
  //if ultrasonic sensor fails and return 0
  if (cm == 0) {
    cm = 100;
  }
  return cm;
}

void Stop() {
  motor1.setSpeed(0);
  motor2.setSpeed(0);
  motor3.setSpeed(0);
  motor4.setSpeed(0);
}
