#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <string.h>
#include <Wire.h>
#include <GFButton.h>
#include <WiFi.h>
#include <AsyncMqttClient.h>
#include <ArduinoJson.h>
#include <EEPROM.h> 

// definições para movimento
#define GYRO_THRSHLD 0.5
#define TAM_GRAVACAO 20
#define TAM_GERAL (15 * TAM_GRAVACAO)
#define PAUSA_MOV 300
#define E 0.1 // = sqrt(0.1*0.1*3)

#define EEPROM_SIZE 1028

// definições hardware
Adafruit_MPU6050 mpu;
const int MPU_INT_PIN = 4;
const int interruptPin = 33;
GFButton botao1(4);
GFButton botao2(15);

// controle de gesto
int i = 0; // iterador gravação do gesto
char gesto[TAM_GRAVACAO] = "yyyyyyy";
// gesto sendo feito
int j = 0; // iterador gesto atual
char gestoEmMovimento[TAM_GERAL] = "xxxxxxxx";
bool gravando = true;
bool movimentando = true;


// leituras MPU
float x_media = 0, y_media = 0, z_media = 0;
int soma = 0;
float gx_calib, gy_calib, gz_calib;
float x_offset = 0;
float y_offset = 0;
float z_offset = 0;


// controle de tempo
bool motionLast20seg = false;
unsigned long instanteAnterior = 0;
unsigned long instAntGesto = 0;
bool movimentoAtivo = false;
unsigned long tempoUltimoMovimento = 0;
unsigned long tempoDeMovimento = 0;
unsigned long inicioMovimento = 0;


// Wi-Fi credentials
const char* WIFI_SSID     = "Projeto";
const char* WIFI_PASSWORD = "2022-11-07";

// MQTT
AsyncMqttClient mqttClient;
const char* MQTT_HOST          = "192.168.0.111";
const uint16_t MQTT_PORT       = 1883;
const char* MQTT_CLIENT_ID     = "ESP32_Lightsaber";
const char* MQTT_LOGIN         = "mqttuser";
const char* MQTT_PASSWORD      = "1234";
const char* MQTT_TOPIC_GESTURE = "sabre/comando";
const char* MQTT_TOPIC_RECEIVED= "sabre/comando/gesto";
StaticJsonDocument<1024> doc;

void gravacao_de_gesto(){
  if(gravando){ // para de gravar
    gravando = false;
    for(int K = 0; gesto[K] != '\0'; K++){
      Serial.print(gesto[K]); Serial.print(" ");
    }
    Serial.println("");
    // TODO: dar um publish numa mensagem no MQTT com o novo gesto! :)
    // formato de envio (json): ["D","F",...]
    if(strlen(gesto) > 0 && gesto[0] != 'y'){
      char gestoEnvio[TAM_GERAL];
      gestoEnvio[0] = '[';
      for(int i = 0; gesto[i] != '\0'; i++){
        strncat(gestoEnvio, "\"", 2);
        char temp[2];
        temp[0] = gesto[i] - ('a' - 'A');
        temp[1] = '\0';
        strncat(gestoEnvio, temp, 3);
        strncat(gestoEnvio, "\",", 3);
      }
      for(int i = 0; gestoEnvio[i] != '\0'; i++)
        if(gestoEnvio[i+1] == '\0')
          gestoEnvio[i] = '\0';
      strncat(gestoEnvio, "]", 2);
      // test example
      publish_event(MQTT_TOPIC_GESTURE, gestoEnvio);
      Serial.println("Mensagem Enviada:");
      Serial.println(gestoEnvio);
    }
  } 
  else { // começa a gravar
    gravando = true; 
    i = 0;
    strcpy(gesto, "yyyyy");
  }
}
// inicio/fim de gravação de gesto
void btn1(GFButton &botao1){
  Serial.println(">> BOTAO 1 PRESSIONADO <<");
  gravacao_de_gesto();
}


// inicio/fim de leitura do movimento atual para comparação com gesto
void leitura_mov(){
  if(movimentando){ // fim do movimento
    movimentando = false;
    Serial.println(gesto);
    Serial.println(gestoEmMovimento);
  } else{ // inicio do movimento
    movimentando = true; 
    j = 0;
    strcpy(gestoEmMovimento, "xxxxx");
  }
}
void btn2(GFButton &botao2){
  Serial.println(">> BOTAO 2 PRESSIONADO <<");
  leitura_mov();
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


// AUXILIARES EEPROM
void saveDocToEEPROM(){
  char json[1024];
  uint32_t len = serializeJson(doc, json);
  
  // escreve tamanho antes da propria string
  EEPROM.put(0, len);

  // escreve json transformado em string
  for(int i = 0; i < len; i++){
    EEPROM.put(sizeof(len) + i, json[i]);
  }
  
  EEPROM.commit();
}

void loadDocFromEEPROM(){
  uint32_t len;
  
  EEPROM.get(0, len);

  if(len >= 1024 || len <= 0){
    Serial.println("Tamanho lido da EEPROM incorreto");
    return;
  }

  char json[1024];
  for(int i = 0; i < len; i++){
    EEPROM.get(sizeof(len) + i, json[i]);
  }
  json[len] = '\0';

  // transforma string em jsonDoc
  DeserializationError error = deserializeJson(doc, json);
  if (error) {
    Serial.println("Falha ao ler da EEPROM");
    return;
  }
}



// WIFI EVENTS
void on_wifi_connected(WiFiEvent_t, WiFiEventInfo_t) {
  Serial.println("[DEBUG] WiFi connected");

  mqttClient.connect();
}
void on_wifi_disconnected(WiFiEvent_t, WiFiEventInfo_t) {
  Serial.println("[DEBUG] WiFi disconnected, retrying..");

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

// MQTT EVENTS
void on_mqtt_connected(bool sessionPresent) {
  Serial.println("[DEBUG] MQTT connected");
  uint16_t packetIdSub = mqttClient.subscribe(MQTT_TOPIC_RECEIVED, 2);
}
void on_mqtt_disconnected(AsyncMqttClientDisconnectReason reason) {
  Serial.println("[DEBUG] MQTT disconnected, retrying..");
  mqttClient.connect();
}
void publish_event(const char* topic, const char* evt) {
  // char buf[64];
  // snprintf(buf, sizeof(buf), "{\"event\":\"%s\",\"ts\":%lu}", evt, millis());
  mqttClient.publish(topic, 2, false, evt);
}
void on_mqtt_message(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total) {
  payload[len] = '\0';
  Serial.println("[DEBUG] Message recieved!");
  // Desserializar a string do MQTT em JSON na variavel global doc para fazer o test dos gestos
  StaticJsonDocument<1024> docTemp;
  docTemp = doc;
  DeserializationError error = deserializeJson(doc, payload);

// checa erro
  if (error) {
    Serial.print("Erro ao desserializar JSON: ");
    Serial.println(error.c_str());
    doc = docTemp;
    return;
  }
  if (!doc.is<JsonArray>()) {
    Serial.println("O JSON não é um array!");
  }

// escreve na EEPROM
  saveDocToEEPROM();
}



void setup(void) {
  // inicia Serial e MPU
  Serial.begin(115200);
  Wire.begin(21, 22);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens
  Serial.println("Adafruit MPU6050 test!");
  if (!mpu.begin()) { // Try to initialize!
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  EEPROM.begin(EEPROM_SIZE);

  // configuração do MPU
  //setupt motion detection
  mpu.setHighPassFilter(MPU6050_HIGHPASS_0_63_HZ);
  mpu.setMotionDetectionThreshold(3.5);
  mpu.setMotionDetectionDuration(400);
  mpu.setInterruptPinLatch(true);	// Keep it latched.  Will turn off when reinitialized.
  mpu.setInterruptPinPolarity(true);
  mpu.setMotionInterrupt(true);



// calibração do MPU
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
  Serial.println("[DEBUG] MPU inicializado e calibrado");


  // carrega gestos da EEPROM
  loadDocFromEEPROM();
  Serial.println("[DEBUG] Gestos da sessao anterior carregados");

  // configuração botoes
  botao1.setPressHandler(btn1);
  // botao1.setReleaseHandler(btn1);
  botao2.setPressHandler(btn2);
  // botao2.setReleaseHandler(btn2);


  // mqtt
  mqttClient.onConnect(on_mqtt_connected);
  mqttClient.onDisconnect(on_mqtt_disconnected);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  mqttClient.setCredentials(MQTT_LOGIN, MQTT_PASSWORD);
  mqttClient.setClientId(MQTT_CLIENT_ID);
  mqttClient.onMessage(on_mqtt_message);
  

  // wifi
  WiFi.onEvent(on_wifi_connected, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_GOT_IP);
  WiFi.onEvent(on_wifi_disconnected, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_DISCONNECTED);
  // connect to wifi, which will then connect to mqtt
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
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

// TODO : TESTAR COM ACELEROMETRO + GIROSCOPIO

  float magnitude = sqrt( x*x + y*y + z*z );

// INICIO DO MOVIMENTO
  if(!movimentoAtivo && magnitude > GYRO_THRSHLD){
        movimentoAtivo = true;
        inicioMovimento = millis();
        x_media = 0;
        y_media = 0;
        z_media = 0;
        soma = 0;
  } // DURANTE O MOVIMENTO
  if(movimentoAtivo && magnitude > E) {
    x_media += x;
    y_media += y;
    z_media += z;
    soma++;
  }

// TODO : RETESTAR COM ACELEROMETRO COM SOMA++ Q TAVA FALTANDO

  // TERMINOU O MOVIMENTO (giroscopio prox de zero)
  if(magnitude < E && movimentoAtivo && (millis() - inicioMovimento > PAUSA_MOV)){
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

// VERIFICA SE MOVIMENTO ATUAL É ALGUM GESTO DO JSON RECEBIDO
 if(!gravando && movimentando && doc.is<JsonArray>()){
  JsonArray array = doc.as<JsonArray>();
  for (int i = 0; i < array.size(); i++) {
    if (array[i].is<const char*>()){
      const char* item = array[i].as<const char*>();
      strcpy(gesto, item);
      if(strstr(gestoEmMovimento, gesto) != NULL){
        Serial.println("=================");
        Serial.print("GESTO "); Serial.println(i);
        Serial.print(gesto);
        Serial.println(gestoEmMovimento);
        Serial.println("=================");
        strcpy(gestoEmMovimento, "xxxxx");
        leitura_mov();
      }
    }
  }
 }

// PARTE DEIXADA DE LADO -- ACELEROMETRO
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

//   else if (millis() - instanteAnterior > 700) {
//     motionLast20seg = false;
//     instanteAnterior = millis();
//  }

//  if(!motionLast20seg){
//   sensors_event_t a, g, temp;
//   mpu.getEvent(&a, &g, &temp);
//   x_offset = a.acceleration.x;
//   y_offset = a.acceleration.y;
//   z_offset = a.acceleration.z;
//  }

}