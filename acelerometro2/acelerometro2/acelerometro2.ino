#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <GFButton.h>

#define ACEL_THRSHLD 5.0
#define TAM 20

Adafruit_MPU6050 mpu;

GFButton botao1(4);
GFButton botao2(15); 

int i = 0;
char gesto[TAM];

bool gravando = true;

const int interruptPin = 33;

bool motionLast20seg = false;

unsigned long instanteAnterior = 0;

float x_offset = 0;
float y_offset = 0;
float z_offset = 0;

void btn1(GFButton &botao1){
  Serial.println(">> BOTAO 1 PRESSIONADO <<");
  if(gravando){
    gesto[i] = '\0';
    gravando = false;
    i = 0;
    for(int j = 0; gesto[j] != '\0'; j++){
      Serial.print(gesto[j]); Serial.print(" ");
    }
    Serial.println("");
  } 
  else { gravando = true; }
}

void btn2(GFButton &botao2){
  Serial.println(">> BOTAO 2 PRESSIONADO <<");
}

void gravacao(char mov){
  if(gravando){
    gesto[i] = mov;
    i = (++i)%TAM;
  }
}

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

  botao1.setPressHandler(btn1);
  botao2.setPressHandler(btn2);

  Serial.println("");
  delay(100);
}

void loop() {
  botao1.process();
  botao2.process();
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
      gravacao('f');
    } else if (x <= -ACEL_THRSHLD && maior == 'x'){
      Serial.println("----------------------------");
      Serial.print(x); Serial.print("\t");
      Serial.print(y); Serial.print("\t");
      Serial.print(z); Serial.print("\t");
      Serial.println("tras");
      gravacao('t');
      Serial.println("----------------------------");
    } else if (z <= -ACEL_THRSHLD && maior == 'z'){
      Serial.println("----------------------------");
      Serial.print(x); Serial.print("\t");
      Serial.print(y); Serial.print("\t");
      Serial.print(z); Serial.print("\t");
      Serial.println("cima");
      gravacao('c');
      Serial.println("----------------------------");
    } else if (z >= ACEL_THRSHLD && maior == 'z'){
      Serial.println("----------------------------");
      Serial.print(x); Serial.print("\t");
      Serial.print(y); Serial.print("\t");
      Serial.print(z); Serial.print("\t");
      Serial.println("baixo");
      gravacao('b');
      Serial.println("----------------------------");
    } else if (y <= -ACEL_THRSHLD && maior == 'y'){
      Serial.println("----------------------------");
      Serial.print(x); Serial.print("\t");
      Serial.print(y); Serial.print("\t");
      Serial.print(z); Serial.print("\t");
      Serial.println("direita");
      gravacao('d');
      Serial.println("----------------------------");
    } else if (y >= ACEL_THRSHLD && maior == 'y'){
      Serial.println("----------------------------");
      Serial.print(x); Serial.print("\t");
      Serial.print(y); Serial.print("\t");
      Serial.print(z); Serial.print("\t");
      Serial.println("esquerda");
      gravacao('e');
      Serial.println("----------------------------");
    } 
  // reseta os valores do acelerometro
  x_offset = a.acceleration.x;
  y_offset = a.acceleration.y;
  z_offset = a.acceleration.z;
  }

  else if (millis() - instanteAnterior > 500) {
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