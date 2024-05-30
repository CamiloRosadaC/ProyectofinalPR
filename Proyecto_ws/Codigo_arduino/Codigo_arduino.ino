#include <Servo.h>

Servo myservo;  // crea un objeto servo
int pos = 0;    // variable para almacenar la posición del servo

void setup() {
  Serial.begin(9600);  // inicializa la comunicación serie
  myservo.attach(9);   // adjunta el servo en el pin 9
  myservo.write(0);    // inicia el servo en la posición 0 grados
}

void loop() {
  if (Serial.available() > 0) {
    char received = Serial.read();  // lee el byte entrante

    if (received == '0') {
      myservo.write(0);  // mueve el servo a 0 grados
    } else if (received == '1') {
      myservo.write(90);  // mueve el servo a 90 grados
    }
  }
}
