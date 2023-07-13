/*
* Função criada para a fotocélula com sensores de temperatura, umidade, acelerômetro (para identificar se a fotocelula caiu, a assim um eventual
* poste), LDR (identificação se o ambiente está escuro para acionamento da luz automaticamente).
* Todo o código foi pensado para comunicação LoRa e envio de pacote no modelo do Cayenne (já está estruturado e documentado na internet).
* São utilizadas Tasks diferente para envio do pacote LoRa e acionamento das luzes, isto porquê o WHILE não permite delay.
* No incio da TaskZero, que realiza envio de pacote LoRa, é desativdo o watchdog da task para que não necessite de delay.
* Todos os tipos escolhidos pros sensores foram adaptados dos tipos de sensores já existentes no cayenne, de forma a não 
* diminuir a precisão do dado e nem perder valores.
*/

#include "LoRaWan_APP.h"
#include "DHT.h"
#include "Adafruit_TCS34725.h"
#include<Wire.h>//Biblioteca para comunicação I2C
#include <esp_task_wdt.h>

const int MPU_addr=0x68; //Endereço do sensor Accel
int16_t AcX,AcY,AcZ; //Variaveis para pegar os valores medidos
uint16_t r, g, b, c, lux;
const uint8_t pinoLDR = 12;
const uint8_t pinoDHT = 23;
float temperatura = 0;
uint8_t umidade = 0;

//Variáveis de recebimento temporário dos dados dos sensores
uint8_t temp_x100_msb, temp_x100_lsb, umid_x100_msb, umid_x100_lsb,  lux_msb, lux_lsb, AcX_msb, AcX_lsb, AcY_msb, AcY_lsb, AcZ_msb, AcZ_lsb;

//Variável com valores dos canais aos quais os sensores serão enviados
int ChannelLux = 0, ChannelUmid = 1, ChannelTemp = 2, ChannelAc = 3, ChannelflagLuz = 4, ChannelflagAce = 5;

//variável com o tipo dos sensores (segundo a tabela do cayenne)
int TypeLux = 101, TypeUmid = 104, TypeTemp = 103, TypeAc = 113, TypeFlag = 102;

//Falg de identificção se a luz está acesa ou não
uint8_t flagLuz = 0, flagAce=0;
const int pinoLuz = 25; //Pino de saída da luz

DHT dht(pinoDHT, DHT11); //pinagem do DHT

/* Initialise with specific int time and gain values */
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_1X);


/* OTAA para*/
//Chaves para OTAA
uint8_t devEui[] = { 0x9f, 0xfc, 0x77, 0x6f, 0x2d, 0x54, 0xe3, 0x61 };
uint8_t appEui[] = { 0x26, 0x78, 0xf2, 0x1f, 0x93, 0x92, 0x0c, 0x59, 0x53, 0x89, 0x97, 0xd3, 0x7c, 0x0b, 0xfd, 0x4f };
uint8_t appKey[] = { 0xec, 0x81, 0xf4, 0x32, 0x0f, 0xb3, 0xdd, 0x35, 0xa1, 0x32, 0x25, 0x0e, 0x10, 0x53, 0x3f, 0xc9 };

/* ABP para*/
//Chaves para ABP
uint8_t nwkSKey[] = { 0xdb, 0x9b, 0x69, 0xd1, 0x9a, 0xa4, 0xc6, 0x6c, 0x9f, 0x37, 0xbf, 0xec, 0xc8, 0x6d, 0x75, 0x70 };
uint8_t appSKey[] = { 0x83, 0xde, 0xf7, 0x79, 0xdc, 0x00, 0x4f, 0x46, 0x94, 0xf2, 0x2e, 0x5d, 0x90, 0x28, 0xf8, 0x2a };
uint32_t devAddr = (uint32_t) 0x008d2f9a;


/*LoraWan channelsmask, default channels 0-7*/
//Definição da subbanda utilizada
uint16_t userChannelsMask[6] = { 0x00FF, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 };  // Sub banda 1

/*LoraWan region, select in arduino IDE tools*/
//Região do LoRa escolhida via IDE
LoRaMacRegion_t loraWanRegion = ACTIVE_REGION;

/*LoraWan Class, Class A and Class C are supported*/
//Classe de trabalho do device LoRa
DeviceClass_t loraWanClass = CLASS_A;

/*the application data transmission duty cycle.  value in [ms].*/
//Tempo de envio de mensagens via LoRa existe limite mínimo
uint32_t appTxDutyCycle = 1500;

/*OTAA or ABP*/
//Define se OTAA (true) ou ABP (false)
bool overTheAirActivation = true;

/*ADR enable*/
//Habilita o DataRate adaptativo
bool loraWanAdr = true;

/* Indicates if the node is sending confirmed or unconfirmed messages */
//Habilida o pacote com confirmação de recebimento (ACK)
bool isTxConfirmed = true;

/* Application port */
//Define porta que irá trabalhar (1 a 255)
uint8_t appPort = 2;

uint8_t confirmedNbTrials = 4;

//Variáveis que indicam o núcleo de trabalho
static uint8_t taskCoreZero = 0;
static uint8_t taskCoreOne = 1;

//Prepara o payload para o modelo cayenne
static void prepareTxFrame(uint8_t port) {
    /*Cada sensor que será enviado no modelo cayenne deve seguir o seguinte padrão de montagem
     * [CANAL DO SENSOR] [TIPO DO SENSOR] [VALOR A SER ENVIADO]
     * Tudo deve ser em uint8_t, por isso para valores muito grandes deve-se dividi-los
    */
    // Tamanho do pacote (modificar quando aumentar ou diminuir a quantidade de sensores enviados)
    appDataSize = 25;

    //Sensor de temperatura (ºC)
    appData[0] = (uint8_t) ChannelTemp;
    appData[1] = (uint8_t) TypeTemp;
    appData[2] = temp_x100_msb;
    appData[3] = temp_x100_lsb;

    //Sensor de umidade (%)
    appData[4] = (uint8_t) ChannelUmid;
    appData[5] = (uint8_t) TypeUmid;
    appData[6] = umidade * 2;

    //Sensor de luz (luz)
    appData[7] = (uint8_t) ChannelLux;
    appData[8] = (uint8_t) TypeLux;
    appData[9] = lux_msb;
    appData[10] = lux_lsb;

    //Sensor do acelerometro
    appData[11] = (uint8_t) ChannelAc;
    appData[12] = (uint8_t) TypeAc;
    //Eixo X
    appData[13] = AcX_msb;
    appData[14] = AcX_lsb;
    //Eixo Y
    appData[15] = AcY_msb;
    appData[16] = AcY_lsb;
    //Eixo Z
    appData[17] = AcZ_msb;
    appData[18] = AcZ_lsb;

    //Flag de identificação do estado da luz
    appData[19] = (uint8_t) ChannelflagLuz;
    appData[20] = (uint8_t) TypeFlag;
    appData[21] = flagLuz;

    //Flag de identificação do alerta do acelerometro (caso de queda)
    appData[22] = (uint8_t) ChannelflagAce;
    appData[23] = (uint8_t) TypeFlag;
    appData[24] = flagAce;
}

//if true, next uplink will add MOTE_MAC_DEVICE_TIME_REQ

void setup() {
    Serial.begin(115200);

    Wire.begin(); //Inicia a comunicação I2C
    Wire.beginTransmission(MPU_addr); //Começa a transmissao de dados para o sensor
    Wire.write(0x6B); // registrador PWR_MGMT_1
    Wire.write(0); // Manda 0 e "acorda" o MPU 6050
    Wire.endTransmission(true);

    Mcu.begin();
    dht.begin();
    //Inicializamos nosso sensor DHT11
    tcs.begin();
    //Definimos o modo de saída
    pinMode(pinoLuz, OUTPUT);
    deviceState = DEVICE_STATE_INIT;

    
    //Declaração da primeira Task
     xTaskCreatePinnedToCore(
                coreTaskZero, /* Função que implementa a tarefa do bluetooth */
                "TaskBle0", /* Nome da tarefa */
                10000, /* Numero de palavras a serem alocados para uso com a pilha da tarefa */
                NULL, /* parâmetro de entrada para a tarefa (pode ser NULL) */
                1, /* prioridade da tarefa (0 a N) */
                NULL, /* task handle referencia para a tareffa (pode ser NULL) */
                taskCoreZero); /* Núcleo que executará a tarefa */
   delay(500);

  //Declaração da segunda Task
   xTaskCreatePinnedToCore(
                coreTaskFour, /* Função que implementa a tarefa do bluetooth */
                "TaskGPS", /* Nome da tarefa */
                10000, /* Numero de palavras a serem alocados para uso com a pilha da tarefa */
                NULL, /* parâmetro de entrada para a tarefa (pode ser NULL) */
                1, /* prioridade da tarefa (0 a N) */
                NULL, /* task handle referencia para a tareffa (pode ser NULL) */
                taskCoreOne); /* Núcleo que executará a tarefa */
   delay(500);
}

//Chamada da primeira Task
void coreTaskZero( void* pvParameters){

  String taskMessage = "Tarefa 0 executando no nucleo: ";
  taskMessage = taskMessage + xPortGetCoreID();
  Serial.println(taskMessage);

  Serial.begin(115200);

    Wire.begin(); //Inicia a comunicação I2C
    Wire.beginTransmission(MPU_addr); //Começa a transmissao de dados para o sensor
    Wire.write(0x6B); // registrador PWR_MGMT_1
    Wire.write(0); // Manda 0 e "acorda" o MPU 6050
    Wire.endTransmission(true);

    Mcu.begin();
    dht.begin();
    //Inicializamos nosso sensor DHT11
    tcs.begin();
    //Definimos o modo de saída
    deviceState = DEVICE_STATE_INIT;
    esp_task_wdt_init(30, false); //desabilita o watchdog e assim não há necessidade de delay na Task

//Loop da primeira Task
while(1){
    switch (deviceState) {
        case DEVICE_STATE_INIT:
        {
            #if (LORAWAN_DEVEUI_AUTO)
                LoRaWAN.generateDeveuiByChipID();
            #endif
            LoRaWAN.init(loraWanClass, loraWanRegion);
            break;
        }
        case DEVICE_STATE_JOIN: //Envia o request pro servidor LoRa
        {
            LoRaWAN.join();
            break;
        }
        case DEVICE_STATE_SEND: //Faz a leitura dos sensores e chama função de montagem do pacote para envio
        {

            //Sensor de temperatura
            temperatura = dht.readTemperature();
            umidade = dht.readHumidity();
            uint16_t temp_x100 = temperatura * 10;
            temp_x100_msb = (uint8_t) ((temp_x100 & 0xFF00) >> 8);
            temp_x100_lsb = (uint8_t) ((temp_x100 & 0x00FF));
            uint8_t umid_x100 = umidade;

            //Sensor de luz
            tcs.getRawData(&r, &g, &b, &c);//Pega os valores "crus" do sensor referentes ao Vermelho(r), Verde(g), Azul(b) e da Claridade(c).
            lux = tcs.calculateLux(r, g, b);
            lux_msb = (uint8_t) ((lux & 0xFF00) >> 8);
            lux_lsb = (uint8_t) ((lux & 0x00FF));

            //Sensor do acelerometro
            Wire.beginTransmission(MPU_addr); //Começa a transmissao de dados para o sensor
            Wire.write(0x3B); // registrador dos dados medidos (ACCEL_XOUT_H)
            Wire.endTransmission(false);
            Wire.requestFrom(MPU_addr,14,true); // faz um "pedido" para ler 14 registradores, que serão os registrados com os dados medidos
            AcX=Wire.read()<<8|Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
            AcY=Wire.read()<<8|Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
            AcZ=Wire.read()<<8|Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)

            //Multiplicação necessária pra conversão do cayenne
            AcX = AcX*1000;
            AcY = AcY*1000;
            AcZ = AcZ*1000;          
            
            //Ajuste de variáveis de Acx para envio LoRa (uint8_t)
            AcX_msb = (uint8_t) ((AcX & 0xFF00) >> 8);
            AcX_lsb = (uint8_t) ((AcX & 0x00FF));

            //Ajuste de variáveis de Acy para envio LoRa (uint8_t)
            AcY_msb = (uint8_t) ((AcY & 0xFF00) >> 8);
            AcY_lsb = (uint8_t) ((AcY & 0x00FF));

            //Ajuste de variáveis de Acz para envio LoRa (uint8_t)
            AcZ_msb = (uint8_t) ((AcZ & 0xFF00) >> 8);
            AcZ_lsb = (uint8_t) ((AcZ & 0x00FF));

            prepareTxFrame(appPort); //Função pra montagemdo pacote em formato cayenne
            LoRaWAN.send(); //Envio do pacote
            deviceState = DEVICE_STATE_CYCLE;
            break;
        }
        case DEVICE_STATE_CYCLE: //Espera o envio do pacote
        {
            // Schedule next packet transmission
            txDutyCycleTime = appTxDutyCycle; /* + randr(-APP_TX_DUTYCYCLE_RND, APP_TX_DUTYCYCLE_RND); */
            LoRaWAN.cycle(txDutyCycleTime);
            deviceState = DEVICE_STATE_SLEEP;
            break;
        }
        case DEVICE_STATE_SLEEP:
        {
            LoRaWAN.sleep(loraWanClass);
            break;
        }
        default:
        {
            deviceState = DEVICE_STATE_INIT;
            break;
        }
      }
} 
      
}

//Declaração da segunda Task
void coreTaskFour( void* pvParameters){

  String taskMessage = "Tarefa 0 executando no nucleo: ";
  taskMessage = taskMessage + xPortGetCoreID();
  Serial.println(taskMessage);

  Serial.begin(115200);

    Wire.begin(); //Inicia a comunicação I2C
    Wire.beginTransmission(MPU_addr); //Começa a transmissao de dados para o sensor
    Wire.write(0x6B); // registrador PWR_MGMT_1
    Wire.write(0); // Manda 0 e "acorda" o MPU 6050
    Wire.endTransmission(true);

    Mcu.begin();
    dht.begin();
    //Inicializamos nosso sensor DHT11
    tcs.begin();
    //Definimos o modo de saída
//Loop da segunda Task (sempre necessário)
while(1){

  //Lógica do Acelerometro
  Wire.beginTransmission(MPU_addr); //Começa a transmissao de dados para o sensor
  Wire.write(0x3B); // registrador dos dados medidos (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true); // faz um "pedido" para ler 14 registradores, que serão os registrados com os dados medidos
  AcX=Wire.read()<<8|Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  AcY=Wire.read()<<8|Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8|Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)   

  if(abs(AcY)>5000 || abs(AcX)>5000){ 
    flagAce = 1;
  }
  else flagAce = 0;

  //Lógica da Iluminação
  tcs.getRawData(&r, &g, &b, &c);//Pega os valores "crus" do sensor referentes ao Vermelho(r), Verde(g), Azul(b) e da Claridade(c).
  lux = tcs.calculateLux(r, g, b);
  if(lux < 20 && flagAce == 0){
    digitalWrite(pinoLuz, HIGH);
    flagLuz = 1;
  }
  else{
    digitalWrite(pinoLuz, LOW);
    flagLuz = 0;
  }

  vTaskDelay(500); //delay das task para não ativar o watchdog
}
}

void loop() {
}
