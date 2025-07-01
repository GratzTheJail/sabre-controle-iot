#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <string.h>
#include <Wire.h>
#include <GFButton.h>
#include <ArduinoJson.h>
 
#define GYRO_THRSHLD 0.5
#define TAM_GRAVACAO 20
#define TAM_GERAL (15 * TAM_GRAVACAO)
#define PAUSA_MOV 300
#define E 0.1 // = sqrt(0.1*0.1*3)


Adafruit_MPU6050 mpu;
const int MPU_INT_PIN = 4;

GFButton botao1(4);
GFButton botao2(15);

int i = 0; // iterador gravação do gesto
char gesto[TAM_GRAVACAO] = "yyyyyyy";
// TESTANDO
int j = 0; // iterador gesto atual
char gestoEmMovimento[TAM_GERAL] = "xxxxxxxx";

bool gravando = true;
bool movimentando = true;

const int interruptPin = 33;


bool motionLast20seg = false;

unsigned long instanteAnterior = 0;
unsigned long instAntGesto = 0;

bool movimentoAtivo = false;
unsigned long tempoUltimoMovimento = 0;
unsigned long tempoDeMovimento = 0;
unsigned long inicioMovimento = 0;
float x_media = 0, y_media = 0, z_media = 0;
int soma = 0;

float gx_calib;
float gy_calib;
float gz_calib;

float x_offset = 0;
float y_offset = 0;
float z_offset = 0;

void btn1(GFButton &botao1){
  Serial.println(">> BOTAO 1 PRESSIONADO <<");
  if(gravando){ // para de gravar
    gravando = false;
    for(int K = 0; gesto[K] != '\0'; K++){
      Serial.print(gesto[K]); Serial.print(" ");
    }
    Serial.println("");
  } 
  else { // começa a gravar
    gravando = true; 
    i = 0;
    strcpy(gesto, "yyyyy");

  }
}

void btn2(GFButton &botao2){
  Serial.println(">> BOTAO 2 PRESSIONADO <<");
  if(movimentando){
    movimentando = false;
    Serial.println(gesto);
    Serial.println(gestoEmMovimento);
  } else{ 
    movimentando = true; 
    j = 0;
    strcpy(gestoEmMovimento, "xxxxx");
  }
}

void movimento(char mov){
  char movimento[2] = {mov, '\0'};
  if(gravando){
    // gesto[i] = mov;
    strcpy(&gesto[i], movimento);
    i = (++i)%(TAM_GRAVACAO - 1);
    strcpy(gestoEmMovimento, "xxxxx");
  }
  if(!gravando && movimentando){
    strcpy(&gestoEmMovimento[j], movimento);
    j = (++j)%(TAM_GERAL - 1);
  }
}

void setup(void) {
  Serial.begin(115200);
  Wire.begin(21, 22);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  // if (!mpu.begin()) {
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  //setupt motion detection
  mpu.setHighPassFilter(MPU6050_HIGHPASS_0_63_HZ);
  mpu.setMotionDetectionThreshold(3.5);
  mpu.setMotionDetectionDuration(400);
  mpu.setInterruptPinLatch(true);	// Keep it latched.  Will turn off when reinitialized.
  mpu.setInterruptPinPolarity(true);
  mpu.setMotionInterrupt(true);

  botao1.setPressHandler(btn1);
  // botao1.setReleaseHandler(btn1);
  botao2.setPressHandler(btn2);
  // botao2.setReleaseHandler(btn2);

  Serial.println("");
  const int CALIB_SAMPLES = 500;
  float gx_calib = 0, gy_calib = 0, gz_calib = 0;

  for(int i=0; i<CALIB_SAMPLES; i++) {
      sensors_event_t a, g, temp;
      mpu.getEvent(&a, &g, &temp);
      gx_calib += g.gyro.x;
      gy_calib += g.gyro.y;
      gz_calib += g.gyro.z;
      delay(5);
  }

  gx_calib /= CALIB_SAMPLES;
  gy_calib /= CALIB_SAMPLES;
  gz_calib /= CALIB_SAMPLES;

  // leitura do arquivo JSON
  if (!SD.begin()) {
    Serial.println("SD init failed!");
    return;
  }

  File file = SD.open("/gestos.json");
  if (!file) {
    Serial.println("Failed to open json file");
    return;
  }

  DynamicJsonDocument doc(2048);
  deserializeJson(doc, file);
  file.close();
  // LIDO E SALVO EM DOC
}

void loop() {
  botao1.process();
  botao2.process();
  sensors_event_t a, g, temp;
  int x,y,z;
  mpu.getEvent(&a, &g, &temp);
  // a.acceleration.x -= x_offset;
  // a.acceleration.y -= y_offset;
  // a.acceleration.z -= z_offset;
  // x = a.acceleration.x;
  // y = a.acceleration.y; 
  // z = a.acceleration.z; 
  x = g.gyro.x - gx_calib;
  y = g.gyro.y - gy_calib;
  z = g.gyro.z - gz_calib;

  // Serial.println("--------------------");
  // Serial.print(x);
  // Serial.print(",\tY: ");
  // Serial.print(y);
  // Serial.print(",\tZ: ");
  // Serial.print(z);
  // Serial.println("\to/s");
  // delay(30);

// TODO : TESTAR COM ACELEROMETRO + GIROSCOPIO

  float magnitude = sqrt( x*x + y*y + z*z );

  if(!movimentoAtivo && magnitude > GYRO_THRSHLD){
        movimentoAtivo = true;
        inicioMovimento = millis();
        x_media = 0;
        y_media = 0;
        z_media = 0;
        soma = 0;
        // Serial.println("Comecou o movimento");
  }
  if(movimentoAtivo && magnitude > E) {
      // Serial.println("Esta rolando o movimento");
    x_media += x;
    y_media += y;
    z_media += z;
    soma++;
  }

// TODO : RETESTAR COM ACELEROMETRO COM SOMA++ Q TAVA FALTANDO

  // termino de movimento (acelerometro prox de zero)
  if(magnitude < E && movimentoAtivo && (millis() - inicioMovimento > PAUSA_MOV)){
    // Serial.println("TERMINOU O MOVIMENTO");
    movimentoAtivo = false;

    x_media = x_media / (float)soma;
    y_media = y_media / (float)soma;
    z_media = z_media / (float)soma;

    // Determina direção predominante
    char maior = 'w';
    if (abs(x_media) >= abs(z_media) && abs(x_media) >= abs(y_media)) {
        maior = 'x';
    } else if (abs(z_media) >= abs(x_media) && abs(z_media) >= abs(y_media)) {
        maior = 'z';
    } else if (abs(y_media) >= abs(z_media) && abs(y_media) >= abs(x_media)) {
        maior = 'y';
    }

    // Processa movimento baseado na média
    if (x_media > 0 && maior == 'x') { // x_media >= GYRO_THRSHLD &&
        Serial.println("frente");
        movimento('f');
    } else if (x_media < 0 && maior == 'x') { // x_media <= -GYRO_THRSHLD &&
        Serial.println("tras");
        movimento('t');
    } else if (z_media < 0 && maior == 'z') { // z_media <= -GYRO_THRSHLD &&
        Serial.println("cima");
        movimento('c');
    } else if (z_media > 0 && maior == 'z') { // z_media >= GYRO_THRSHLD &&
        Serial.println("baixo");
        movimento('b');
    } else if (y_media < 0 && maior == 'y') { // y_media <= -GYRO_THRSHLD &&
        Serial.println("direita");
        movimento('d');
    } else if (y_media > 0 && maior == 'y') { // y_media >= GYRO_THRSHLD &&
        Serial.println("esquerda");
        movimento('e');
    }

    // Atualiza offsets
    // x_offset = a.acceleration.x;
    // y_offset = a.acceleration.y;
    // z_offset = a.acceleration.z;
  }

  // caso movimento muito curto
  else if(magnitude < E && movimentoAtivo && (millis() - inicioMovimento <= PAUSA_MOV)){
    // Serial.println("Terminou o movimento rapido demais");
    movimentoAtivo = false;
  }

  // if(mpu.getMotionInterruptStatus() /*&& millis() - instAntGesto > PAUSA_MOV*/) {
  //   motionLast20seg = true;
  //   instanteAnterior = millis();
  //   instAntGesto = millis();

  //   /* Get new sensor events with the readings */
  //   sensors_event_t a, g, temp;
  //   mpu.getEvent(&a, &g, &temp);
  //   a.acceleration.x -= x_offset;
  //   a.acceleration.y -= y_offset;
  //   a.acceleration.z -= z_offset;

  //   float x = a.acceleration.x;
  //   float y = a.acceleration.y;
  //   float z = a.acceleration.z;

  //   char maior = 'w';

  //   // movimento em so uma direção
  //   if(abs(x) >= abs(z) && abs(x) >= abs(y)){
  //     maior = 'x';
  //   } else if(abs(z) >= abs(x) && abs(z) >= abs(y)){
  //     maior = 'z';
  //   } else if(abs(y) >= abs(z) && abs(y) >= abs(x)){
  //     maior = 'y';
  //   }

  //   if(x >= GYRO_THRSHLD && maior == 'x'){
  //     Serial.println("frente");
  //     movimento('f');
  //   } else if (x <= -GYRO_THRSHLD && maior == 'x'){
  //     Serial.println("tras");
  //     movimento('t');
  //   } else if (z <= -GYRO_THRSHLD && maior == 'z'){
  //     Serial.println("cima");
  //     movimento('c');
  //   } else if (z >= GYRO_THRSHLD && maior == 'z'){
  //     Serial.println("baixo");
  //     movimento('b');
  //   } else if (y <= -GYRO_THRSHLD && maior == 'y'){
  //     Serial.println("direita");
  //     movimento('d');
  //   } else if (y >= GYRO_THRSHLD && maior == 'y'){
  //     Serial.println("esquerda");
  //     movimento('e');
  //   } 
  // // reseta os valores do acelerometro
  // x_offset = a.acceleration.x;
  // y_offset = a.acceleration.y;
  // z_offset = a.acceleration.z;
  // }

  else if (millis() - instanteAnterior > 700) {
    motionLast20seg = false;
    instanteAnterior = millis();
 }

//  if(!motionLast20seg){
//   sensors_event_t a, g, temp;
//   mpu.getEvent(&a, &g, &temp);
//   x_offset = a.acceleration.x;
//   y_offset = a.acceleration.y;
//   z_offset = a.acceleration.z;
//  }

 if(!gravando && movimentando){
  if(strstr(gestoEmMovimento, gesto) != NULL){
    Serial.println("=================");
    Serial.println("GESTO 1");
    Serial.print(gesto);
    Serial.println(gestoEmMovimento);
    Serial.println("=================");
    strcpy(gestoEmMovimento, "xxxxx");
  }
 }

}