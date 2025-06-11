#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#define ACEL_THRSHLD 6.0

Adafruit_MPU6050 mpu;

const int interruptPin = 33;

bool motionLast20seg = false;

unsigned long instanteAnterior = 0;

float x_offset = 0;
float y_offset = 0;
float z_offset = 0;

void setup(void) {
  Serial.begin(115200);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  //setupt motion detection
  mpu.setHighPassFilter(MPU6050_HIGHPASS_0_63_HZ);
  mpu.setMotionDetectionThreshold(15);
  mpu.setMotionDetectionDuration(50);
  mpu.setInterruptPinLatch(true);	// Keep it latched.  Will turn off when reinitialized.
  mpu.setInterruptPinPolarity(true);
  mpu.setMotionInterrupt(true);

  Serial.println("");
  delay(100);
}

void loop() {
  // sensors_event_t a, g, temp;
  // mpu.getEvent(&a, &g, &temp);
  // a.acceleration.x -= x_offset;
  // a.acceleration.y -= y_offset;
  // a.acceleration.z -= z_offset;

  // Serial.print(a.acceleration.x);
  // Serial.print(", Y: ");
  // Serial.print(a.acceleration.y);
  // Serial.print(", Z: ");
  // Serial.print(a.acceleration.z);
  // Serial.println(" m/s^2");

  if(mpu.getMotionInterruptStatus()) {
    motionLast20seg = true;
    instanteAnterior = millis();

    /* Get new sensor events with the readings */
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    a.acceleration.x -= x_offset;
    a.acceleration.y -= y_offset;
    a.acceleration.z -= z_offset;

    float x = a.acceleration.x;
    float y = a.acceleration.y;
    float z = a.acceleration.z;

    char maior = 'w';

    if(abs(x) >= abs(z) && abs(x) >= abs(y)){
      maior = 'x';
    } if(abs(z) >= abs(x) && abs(z) >= abs(y)){
      maior = 'z';
    } if(abs(y) >= abs(z) && abs(y) >= abs(x)){
      maior = 'y';
    }

    if(x >= ACEL_THRSHLD && maior == 'x'){
      Serial.println("----------------------------");
      Serial.print(x); Serial.print("\t");
      Serial.print(y); Serial.print("\t");
      Serial.print(z); Serial.print("\t");
      Serial.println("frente");
      Serial.println("----------------------------");
    } else if (x <= -ACEL_THRSHLD && maior == 'x'){
      Serial.println("----------------------------");
      Serial.print(x); Serial.print("\t");
      Serial.print(y); Serial.print("\t");
      Serial.print(z); Serial.print("\t");
      Serial.println("tras");
      Serial.println("----------------------------");
    } else if (z <= -ACEL_THRSHLD && maior == 'z'){
      Serial.println("----------------------------");
      Serial.print(x); Serial.print("\t");
      Serial.print(y); Serial.print("\t");
      Serial.print(z); Serial.print("\t");
      Serial.println("cima");
      Serial.println("----------------------------");
    } else if (z >= ACEL_THRSHLD && maior == 'z'){
      Serial.println("----------------------------");
      Serial.print(x); Serial.print("\t");
      Serial.print(y); Serial.print("\t");
      Serial.print(z); Serial.print("\t");
      Serial.println("baixo");
      Serial.println("----------------------------");
    } else if (y <= -ACEL_THRSHLD && maior == 'y'){
      Serial.println("----------------------------");
      Serial.print(x); Serial.print("\t");
      Serial.print(y); Serial.print("\t");
      Serial.print(z); Serial.print("\t");
      Serial.println("direita");
      Serial.println("----------------------------");
    } else if (y >= ACEL_THRSHLD && maior == 'y'){
      Serial.println("----------------------------");
      Serial.print(x); Serial.print("\t");
      Serial.print(y); Serial.print("\t");
      Serial.print(z); Serial.print("\t");
      Serial.println("esquerda");
      Serial.println("----------------------------");
    } 
  }

  else if (millis() - instanteAnterior > 2000) {
    motionLast20seg = false;
    instanteAnterior = millis();
 }

 if(!motionLast20seg){
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  x_offset = a.acceleration.x;
  y_offset = a.acceleration.y;
  z_offset = a.acceleration.z;
 }

  delay(10);
}