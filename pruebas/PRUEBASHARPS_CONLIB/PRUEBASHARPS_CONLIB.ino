//import the library in the sketch
#include <SharpIR.h>

//Create a new instance of the library
//Call the sensor "sensor"
//The model of the sensor is "GP2YA41SK0F"
//The sensor output pin is attached to the pin A0
SharpIR sensor1(SharpIR::GP2Y0A41SK0F, A5);
SharpIR sensor2(SharpIR::GP2Y0A41SK0F, A7);
SharpIR sensor3(SharpIR::GP2Y0A41SK0F, A3);

void setup() {
  Serial.begin(9600);  //Enable the serial comunication
}

void loop() {
  int distance1 = sensor1.getDistance();  //Calculate the distance in centimeters and store the value in a variable
  Serial.print("Sensor izq: ");  //Print the value to the serial monitor
  Serial.println(distance1);  //Print the value to the serial monitor

  int distance2 = sensor2.getDistance();  //Calculate the distance in centimeters and store the value in a variable
  Serial.print("Sensor der: ");  //Print the value to the serial monitor
  Serial.println(distance2);  //Print the value to the serial monitor

  int distance3 = sensor3.getDistance();  //Calculate the distance in centimeters and store the value in a variable
  Serial.print("Sensor centro: ");  //Print the value to the serial monitor
  Serial.println(distance3);  //Print the value to the serial monitor
}