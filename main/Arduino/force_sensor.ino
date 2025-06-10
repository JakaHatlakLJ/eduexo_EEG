#include <movingAvg.h>

int forceAnalogInPin = A3;
int measure;
const float ForceOffset = -377;
const float coefficient = 0.199327;
const float offset = -0.0633;
float force;
int sensorMovingAvg;
movingAvg mySensor(100);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  delay(1000);
  mySensor.begin();
}

void loop() {
  // put your main code here, to run repeatedly:
  measure = analogRead(forceAnalogInPin);
  measure += ForceOffset;
  sensorMovingAvg = mySensor.reading(measure);
  force = coefficient * sensorMovingAvg + offset; // [N]
  
  byte* b = (byte*) &force;         // Pointer to float bytes
  Serial.write(b, sizeof(force));   // Send 4 bytes representing the float

}
