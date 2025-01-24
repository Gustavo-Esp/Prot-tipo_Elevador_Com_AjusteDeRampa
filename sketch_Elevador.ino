#include <ESP32Servo.h>
#include <LiquidCrystal_I2C.h>
#include <math.h>
#include <AiEsp32RotaryEncoder.h>
#include <TM1637Display.h>
#include <WiFi.h>
#include <PubSubClient.h>

#define BAJ 35 // Input - Botão de ajuste dos parâmetros.
#define BAUM 27 // Input - Botão de aumento - parâmetros.
#define BDIM 14 // Input - Botão de diminuição - parâmetros.
#define BSA1 25 // Input - Botão seletor de andar 1.
#define BSA2 33 // Input - Botão seletor de andar 2.
#define BSA3 32 // Input - Botão seletor de andar 3.
#define BSAT 26 // Input - Botão seletor de andar térreo.
#define BC1 18 // Input - Botão chamada de andar 1.
#define BC2 19 // Input - Botão chamada de andar 2.
#define BC3 23 // Input - Botão chamada de andar 3.
#define BCT 17 // Input - Botão chamada de térreo.
#define BE 4 // Input - Botão de emergência.
#define SI 16 // Input - Sinalização de portas fechadas.
#define MOTOR 0 // Output - Motor BC para movimentar o elevador.
#define DisI 6 // Output - Display 7 segmentos interno.
#define DisE1 7 // Output - Display 7 segmentos externo 1 Andar.
#define DisE2 8 // Output - Display 7 segmentos externo 2 Andar.
#define DisE3 9 // Output - Display 7 segmentos externo 3 Andar.
#define DisE4 10 // Output - Display 7 segmentos externo Térreo.
#define DisC 11 // Output - Display LCD Interno para configuração de parâmetros.
//C[2]
#define C1 15
#define C2 13
//Encoder
#define ENCODER_CLK 5
#define ENCODER_DT 12
//Display
#define CLK_TM1637 2
#define DIO_TM1637 4

AiEsp32RotaryEncoder encoder = AiEsp32RotaryEncoder(
    ENCODER_DT, ENCODER_CLK, -1, -1);

LiquidCrystal_I2C lcd(0x27, 16, 2);
TM1637Display display(CLK_TM1637, DIO_TM1637);

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

enum Estados_aciElevador {
  Parado = 0,
  IniSub = 1,
  Subindo = 2,
  FimSub = 3,
  IniDesc = 4,
  Descendo = 5,
  FimDesc = 6
}acionamentoElev;

enum Estados_movElevador {
  Ta = 0,
  Tb = 1,
  SubA1 = 2,
  SubA2 = 3,
  SubA3 = 4,
  A3a = 5,
  A3b = 6,
  DescA2 = 7,
  DescA1 = 8,
  DescT = 9,
  A1a = 10,
  A1b = 11,
  A1c = 12,
  A1d = 13,
  A2a = 14,
  A2b = 15,
  A2c = 16,
  A2d = 17
}movimentoElev;

enum Estados_parametros {
  Operando = 0,
  Configurar_T = 1,
  INC_T = 2,
  DEC_T = 3,
  Configurar_A1 = 4,
  INC_A1 = 5,
  DEC_A1 = 6,
  Configurar_A2 = 7,
  INC_A2 = 8,
  DEC_A2 = 9,
  Configurar_A3 = 10,
  INC_A3 = 11,
  DEC_A3 = 12,
  Configurar_VM = 13,
  INC_VM = 14,
  DEC_VM = 15
}parametros;

enum Estados_chamadaT {
  DesatT = 0,
  AtivT = 1
}chamadoT;

enum Estados_chamadaA1 {
  DesatA1 = 0,
  AtivA1 = 1
}chamadoA1;

enum Estados_chamadaA2 {
  DesatA2 = 0,
  AtivA2 = 1
}chamadoA2;

enum Estados_chamadaA3 {
  DesatA3 = 0,
  AtivA3 = 1
}chamadoA3;

// Valores em cm
int T = 130;   
int a1 = 330;  
int a2 = 330;  
int a3 = 330;
//Valor em %
int VM = 80; 
double porcentagem;

int SP;
int PI_;
int distancia1, distancia2;

Estados_movElevador estadoAnterior;

const char* ssid = "Wokwi-GUEST";
const char* key = "";
const char* broker = "test.mosquitto.org";
int port = 1883;

const char* topico_SP = "topicoSP";
const char* topicoMovimentoElev = "topicoMovimentoElev";
const char* topicoAcionamentoElev = "topicoAcionamentoElev";
const char* topicoChamadoTElev = "topicoChamadoTElev";
const char* topicoChamadoA1Elev = "topicoChamadoA1Elev";
const char* topicoChamadoA2Elev = "topicoChamadoA2Elev";
const char* topicoChamadoA3Elev = "topicoChamadoA3Elev";
const char* topicoParametroElev = "topicoParametroElev";
const char* topicoMotor = "topicoMotor";
const char* topicoC2LED= "topicoC2LED";
const char* topicoC1LED = "topicoC1LED";

byte estadoParametroFlag;
byte estadoChamadoTFlag;
byte estadoChamadoA1Flag;
byte estadoChamadoA2Flag;
byte estadoChamadoA3Flag;
byte estadoMovimentoFlag;
byte estadoAcionamentoFlag;

int SPaux = -1;
double Motoraux = -1;
int distancia;

void setup() {
  Serial.begin(115200);

  pinMode(MOTOR, OUTPUT);
  pinMode(DisI, OUTPUT);
  pinMode(DisE1, OUTPUT);
  pinMode(DisE2, OUTPUT);
  pinMode(DisE3, OUTPUT);
  pinMode(DisE4, OUTPUT);
  pinMode(DisC, OUTPUT);
  pinMode(C1, OUTPUT);
  pinMode(C2, OUTPUT);

  pinMode(SI, INPUT_PULLDOWN);
  pinMode(BE, INPUT_PULLDOWN);
  pinMode(BCT, INPUT_PULLDOWN);
  pinMode(BC3, INPUT_PULLDOWN);
  pinMode(BC2, INPUT_PULLDOWN);
  pinMode(BC1, INPUT_PULLDOWN);
  pinMode(BSAT, INPUT_PULLDOWN);
  pinMode(BSA3, INPUT_PULLDOWN);
  pinMode(BSA2, INPUT_PULLDOWN);
  pinMode(BSA1, INPUT_PULLDOWN);
  pinMode(BDIM, INPUT_PULLDOWN);
  pinMode(BAUM, INPUT_PULLDOWN);
  pinMode(BAJ, INPUT_PULLDOWN);

  encoder.begin();
  encoder.setup([] { encoder.readEncoder_ISR(); }); 
  encoder.setBoundaries(0, 1220, false); 
  encoder.setAcceleration(VM); 
  encoder.reset(0); 

  chamadoT = DesatT;
  chamadoA1 = DesatA1;
  chamadoA2 = DesatA2;
  chamadoA3 = DesatA3;
  parametros = Operando;
  movimentoElev = Ta;
  acionamentoElev = Parado;
  
  lcd.init();
  lcd.backlight();

  display.setBrightness(7); 
  display.clear();    

  conexaoWiFi();
  conexaoBroker();

  estadoParametroFlag = 1;
  estadoChamadoTFlag = 1;
  estadoChamadoA1Flag = 1;
  estadoChamadoA2Flag = 1;
  estadoChamadoA3Flag = 1;
  estadoMovimentoFlag = 1;
  estadoAcionamentoFlag = 1;
}

void Operando_(){
  if (digitalRead(BAJ)){
    estadoParametroFlag = 1;
    lcd.clear();
    parametros = Configurar_T;
  }
}

void Configurar_T_(){
  lcd.setCursor(0, 0);
  lcd.print(T);
  if (digitalRead(BAJ)){
    estadoParametroFlag = 1;
    lcd.clear();
    parametros = Configurar_A1;
  }
  else{
    if (digitalRead(BAUM)){
      estadoParametroFlag = 1;
      parametros = INC_T;
    }
    else{
      if (digitalRead(BDIM)){
        estadoParametroFlag = 1;
        parametros = DEC_T;
      }
    }
  }
}

void Configurar_A1_(){
  lcd.setCursor(0, 0);
  lcd.print(a1);
  if (digitalRead(BAJ)){
    estadoParametroFlag = 1;
    lcd.clear();
    parametros = Configurar_A2;
  }
  else{
    if (digitalRead(BAUM)){
      estadoParametroFlag = 1;
      parametros = INC_A1;
    }
    else{
      if (digitalRead(BDIM)){
        estadoParametroFlag = 1;
        parametros = DEC_A1;
      }
    }
  }
}

void Configurar_A2_(){
  lcd.setCursor(0, 0);
  lcd.print(a2);
  if (digitalRead(BAJ)){
    estadoParametroFlag = 1;
    lcd.clear();
    parametros = Configurar_A3;
  }
  else{
    if (digitalRead(BAUM)){
      estadoParametroFlag = 1;
      parametros = INC_A2;
    }
    else{
      if (digitalRead(BDIM)){
        estadoParametroFlag = 1;
        parametros = DEC_A2;
      }
    }
  }
}

void Configurar_A3_(){
  lcd.setCursor(0, 0);
  lcd.print(a3);
  if (digitalRead(BAJ)){
    estadoParametroFlag = 1;
    lcd.clear();
    parametros = Configurar_VM;
  }
  else{
    if (digitalRead(BAUM)){
      estadoParametroFlag = 1;
      parametros = INC_A3;
    }
    else{
      if (digitalRead(BDIM)){
        estadoParametroFlag = 1;
        parametros = DEC_A3;
      }
    }
  }
}

void Configurar_VM_(){
  lcd.setCursor(0, 0);
  lcd.print(VM);
  lcd.setCursor(2, 0);
  lcd.print("%");
  if (digitalRead(BAJ)){
    estadoParametroFlag = 1;
    lcd.clear();
    parametros = Operando;
  }
  else{
    if (digitalRead(BAUM)){
      estadoParametroFlag = 1;
      parametros = INC_VM;
    }
    else{
      if (digitalRead(BDIM)){
        estadoParametroFlag = 1;
        parametros = DEC_VM;
      }
    }
  }
}

void INC_T_(){
  estadoParametroFlag = 1;
  if (T < 140){
    T = T + 1;
  }
  parametros = Configurar_T;
}

void INC_A1_(){
  estadoParametroFlag = 1;
  if (a1 < 340){
    a1 = a1 + 1;
  }
  parametros = Configurar_A1;
}

void INC_A2_(){
  estadoParametroFlag = 1;
  if (a2 < 340){
    a2 = a2 + 1;  
  }
  parametros = Configurar_A2;
}

void INC_A3_(){
  estadoParametroFlag = 1;
  if (a3 < 340){
    a3 = a3 + 1;
  }
  parametros = Configurar_A3;
}

void INC_VM_(){
  estadoParametroFlag = 1;
  if (VM < 85){
    VM = VM + 1;
  }
  parametros = Configurar_VM;
}

void DEC_T_(){
  estadoParametroFlag = 1;
  if (T > 120){
    T = T - 1;
  }
  parametros = Configurar_T;
}

void DEC_A1_(){
  estadoParametroFlag = 1;
  if (a1 > 320){
    a1 = a1 - 1;
  }
  parametros = Configurar_A1;
}

void DEC_A2_(){
  estadoParametroFlag = 1;
  if (a2 > 320){
    a2 = a2 - 1;
  }
  parametros = Configurar_A2;
}

void DEC_A3_(){
  estadoParametroFlag = 1;
  if (a3 > 320){
    a3 = a3 - 1;
  }
  parametros = Configurar_A3;
}

void DEC_VM_(){
  estadoParametroFlag = 1;
  if (VM > 75){
    VM = VM - 1;
  }
  parametros = Configurar_VM;
}

void ProcessoParametros(){
  switch (parametros){
    case Operando:
      if (estadoParametroFlag){
        estadoParametroFlag = 0;
        mqttClient.publish("topicoParametroElev", "Operando");
      }
      Operando_();
      break;
    case Configurar_T:
      if (estadoParametroFlag){
        estadoParametroFlag = 0;
        mqttClient.publish("topicoParametroElev", "Configurar_T");
      }
      Configurar_T_();
      break;
    case INC_T:
      if (estadoParametroFlag){
        estadoParametroFlag = 0;
        mqttClient.publish("topicoParametroElev", "INC_T");
      }
      INC_T_();
      break;
    case DEC_T:
      if (estadoParametroFlag){
        estadoParametroFlag = 0;
        mqttClient.publish("topicoParametroElev", "DEC_T");
      }
      DEC_T_();
      break;
    case Configurar_A1:
      if (estadoParametroFlag){
        estadoParametroFlag = 0;
        mqttClient.publish("topicoParametroElev", "Configurar_A1");
      }
      Configurar_A1_();
      break;
    case INC_A1:
      if (estadoParametroFlag){
        estadoParametroFlag = 0;
        mqttClient.publish("topicoParametroElev", "INC_A1");
      }
      INC_A1_();
      break;
    case DEC_A1:
      if (estadoParametroFlag){
        estadoParametroFlag = 0;
        mqttClient.publish("topicoParametroElev", "DEC_A1");
      }
      DEC_A1_();
      break;
    case Configurar_A2:
      if (estadoParametroFlag){
        estadoParametroFlag = 0;
        mqttClient.publish("topicoParametroElev", "Configurar_A2");
      }
      Configurar_A2_();
      break;
    case INC_A2:
      if (estadoParametroFlag){
        estadoParametroFlag = 0;
        mqttClient.publish("topicoParametroElev", "INC_A2");
      }
      INC_A2_();
      break;
    case DEC_A2:
      if (estadoParametroFlag){
        estadoParametroFlag = 0;
        mqttClient.publish("topicoParametroElev", "DEC_A2");
      }
      DEC_A2_();
      break;
    case Configurar_A3:
      if (estadoParametroFlag){
        estadoParametroFlag = 0;
        mqttClient.publish("topicoParametroElev", "Configurar_A3");
      }
      Configurar_A3_();
      break;
    case INC_A3:
      if (estadoParametroFlag){
        estadoParametroFlag = 0;
        mqttClient.publish("topicoParametroElev", "INC_A3");
      }
      INC_A3_();
      break;
    case DEC_A3:
      if (estadoParametroFlag){
        estadoParametroFlag = 0;
        mqttClient.publish("topicoParametroElev", "DEC_A3");
      }
      DEC_A3_();
      break;
    case Configurar_VM:
      if (estadoParametroFlag){
        estadoParametroFlag = 0;
        mqttClient.publish("topicoParametroElev", "Configurar_VM");
      }
      Configurar_VM_();
      break;
    case INC_VM:
      if (estadoParametroFlag){
        estadoParametroFlag = 0;
        mqttClient.publish("topicoParametroElev", "INC_VM");
      }
      INC_VM_();
      break;
    case DEC_VM:
      if (estadoParametroFlag){
        estadoParametroFlag = 0;
        mqttClient.publish("topicoParametroElev", "DEC_VM");
      }
      DEC_VM_();
      break;
  }
}

void DesatT_(){
  if (digitalRead(BCT) || digitalRead(BSAT)) {
    estadoChamadoTFlag = 1;
    chamadoT = AtivT;
  }
}

void DesatA1_(){
  if (digitalRead(BC1) || digitalRead(BSA1)) {
    estadoChamadoA1Flag = 1;
    chamadoA1 = AtivA1;
  }
}

void DesatA2_(){
  if (digitalRead(BC2) || digitalRead(BSA2)) {
    estadoChamadoA2Flag = 1;
    chamadoA2 = AtivA2;
  }
}

void DesatA3_(){
  if (digitalRead(BC3) || digitalRead(BSA3)) {
    estadoChamadoA3Flag = 1;
    chamadoA3 = AtivA3;
  }
}

void AtivT_(){
  if (SP >= (T - 5) && SP <= (T + 5) && acionamentoElev == Parado) { 
    estadoChamadoTFlag = 1;
    chamadoT = DesatT;
  }
}

void AtivA1_(){
  if (SP >= (T + a1 - 5) && SP <= (T + a1 + 5) && acionamentoElev == Parado) {
    estadoChamadoA1Flag = 1; 
    chamadoA1 = DesatA1;
  }
}

void AtivA2_(){
  if (SP >= (T + a1 + a2 - 5) && SP <= (T + a1 + a2 + 5) && acionamentoElev == Parado) {
    estadoChamadoA2Flag = 1; 
    chamadoA2 = DesatA2;
  }
}

void AtivA3_(){
  if (SP >= (T + a1 + a2 + a3 - 5) && SP <= (T + a1 + a2 + a3 + 5) && acionamentoElev == Parado) {
    estadoChamadoA3Flag = 1; 
    chamadoA3 = DesatA3;
  }
}

void ProcessoChamados(){
  switch (chamadoT){
    case DesatT:
      if (estadoChamadoTFlag){
        estadoChamadoTFlag = 0;
        mqttClient.publish("topicoChamadoTElev", "DesatT");
      }
      DesatT_();
      break;
    case AtivT:
      if (estadoChamadoTFlag){
        estadoChamadoTFlag = 0;
        mqttClient.publish("topicoChamadoTElev", "AtivT");
      }
      AtivT_();
      break;
  }
  switch (chamadoA1){
    case DesatA1:
      if (estadoChamadoA1Flag){
        estadoChamadoA1Flag = 0;
        mqttClient.publish("topicoChamadoA1Elev", "DesatA1");
      }
      DesatA1_();
      break;
    case AtivA1:
      if (estadoChamadoA1Flag){
        estadoChamadoA1Flag = 0;
        mqttClient.publish("topicoChamadoA1Elev", "AtivA1");
      }
      AtivA1_();
      break;
  }
  switch (chamadoA2){
    case DesatA2:
      if (estadoChamadoA2Flag){
        estadoChamadoA2Flag = 0;
        mqttClient.publish("topicoChamadoA2Elev", "DesatA2");
      }
      DesatA2_();
      break;
    case AtivA2:
      if (estadoChamadoA2Flag){
        estadoChamadoA2Flag = 0;
        mqttClient.publish("topicoChamadoA2Elev", "AtivA2");
      }
      AtivA2_();
      break;
  }
  switch (chamadoA3){
    case DesatA3:
      if (estadoChamadoA3Flag){
        estadoChamadoA3Flag = 0;
        mqttClient.publish("topicoChamadoA3Elev", "DesatA3");
      }
      DesatA3_();
      break;
    case AtivA3:
      if (estadoChamadoA3Flag){
        estadoChamadoA3Flag = 0;
        mqttClient.publish("topicoChamadoA3Elev", "AtivA3");
      }
      AtivA3_();
      break;
  }
} 

void Ta_(){
  mostrarNumeroCentralizado(0);
  if (digitalRead(SI)){
    estadoMovimentoFlag = 1;
    estadoAnterior = Ta;
    movimentoElev = Tb;
  }
}

void Tb_(){
  if (digitalRead(BCT)){
    estadoMovimentoFlag = 1;
    display.clear();
    movimentoElev = Ta;
  }
  else{
    if ((chamadoA1 == AtivA1 || chamadoA2 == AtivA2 || chamadoA3 == AtivA3) && 
        !digitalRead(SI)){
      estadoMovimentoFlag = 1;
      movimentoElev = SubA1;
    }
  }
}

void SubA1_(){
  if (chamadoA1 == DesatA1){
    estadoMovimentoFlag = 1;
    movimentoElev = SubA2;
  }
  else{
    if (SP >= (a1 + T)){
      estadoMovimentoFlag = 1;
      display.clear();
      movimentoElev = A1a;
    }
  }
}

void A1a_(){
  mostrarNumeroCentralizado(1);
  if (digitalRead(SI)){
    estadoMovimentoFlag = 1;
    estadoAnterior = A1a;
    movimentoElev = A1b;
  }
}

void A1b_(){
  if (digitalRead(BC1)){
    estadoMovimentoFlag = 1;
    movimentoElev = A1a;
  }
  else{
    if ((chamadoA2 == AtivA2 || chamadoA3 == AtivA3) && !digitalRead(SI)){
      estadoMovimentoFlag = 1;
      movimentoElev = SubA2;
    }
    else{
      if (chamadoT == AtivT && chamadoA2 == DesatA2 && chamadoA3 == DesatA3 && 
          !digitalRead(SI)){
        estadoMovimentoFlag = 1;
        movimentoElev = DescT;
      }
    }
  }
}

void SubA2_(){
  if (chamadoA2 == DesatA2){
    estadoMovimentoFlag = 1;
    movimentoElev = SubA3;
  }
  else{
    if (SP >= (a2 + a1 + T)){
      estadoMovimentoFlag = 1;
      display.clear();
      movimentoElev = A2a;
    }
  }
}

void A2a_(){
  mostrarNumeroCentralizado(2);
  if (digitalRead(SI)){
    estadoMovimentoFlag = 1;
    estadoAnterior = A2a;
    movimentoElev = A2b;
  }
}

void A2b_(){
  if (digitalRead(BC2)){
    estadoMovimentoFlag = 1;
    movimentoElev = A2a;
  }
  else{
    if (chamadoA3 == AtivA3 && !digitalRead(SI)){
      estadoMovimentoFlag = 1;
      movimentoElev = SubA3;
    }
    else{
      if ((chamadoT == AtivT || chamadoA1 == AtivA1) && chamadoA3 == DesatA3 && 
          !digitalRead(SI)){
        estadoMovimentoFlag = 1;
        movimentoElev = DescA1;
      }
    }
  }
}

void SubA3_(){
  if (SP >= (a3 + a2 + a1 + T)){
    estadoMovimentoFlag = 1;
    display.clear();
    movimentoElev = A3a;
  }
}

void A3a_(){
  mostrarNumeroCentralizado(3);
  if (digitalRead(SI)){
    estadoMovimentoFlag = 1;
    estadoAnterior = A3a;
    movimentoElev = A3b;
  }
}

void A3b_(){
  if (digitalRead(BC3)){
    estadoMovimentoFlag = 1;
    movimentoElev = A3a;
  }
  else{
    if ((chamadoT == AtivT || chamadoA1 == AtivA1 || chamadoA2 == AtivA2) && 
        !digitalRead(SI)){
      estadoMovimentoFlag = 1;
      movimentoElev = DescA2;
    }
  }
}

void DescA2_(){
  if (SP <= (a2 + a1 + T)){
    estadoMovimentoFlag = 1;
    display.clear();
    movimentoElev = A2c;
  }
  else{
    if (chamadoA2 == DesatA2){
      estadoMovimentoFlag = 1;
      movimentoElev = DescA1;
    }
  }
}

void A2c_(){
  mostrarNumeroCentralizado(2);
  if (digitalRead(SI)){
    estadoMovimentoFlag = 1;
    estadoAnterior = A2c;
    movimentoElev = A2d;
  }
}

void A2d_(){
  if (digitalRead(BC2)){
    estadoMovimentoFlag = 1;
    movimentoElev = A2c;
  }
  else{
    if (chamadoA3 == AtivA3 && chamadoA1 == DesatA1 && chamadoT == DesatT && 
        !digitalRead(SI)){
      estadoMovimentoFlag = 1;
      movimentoElev = SubA3;
    }
    else{
      if ((chamadoT == AtivT || chamadoA1 == AtivA1) && !digitalRead(SI)){
        estadoMovimentoFlag = 1;
        movimentoElev = DescA1;
      }
    }
  }
}

void DescA1_(){
  if (SP <= (a1 + T)){
    estadoMovimentoFlag = 1;
    display.clear();
    movimentoElev = A1c;
  }
  else{
    if (chamadoA1 == DesatA1){
      estadoMovimentoFlag = 1;
      movimentoElev = DescT;
    }
  }
}

void A1c_(){
  mostrarNumeroCentralizado(1);
  if (digitalRead(SI)){
    estadoMovimentoFlag = 1;
    estadoAnterior = A1c;
    movimentoElev = A1d;
  }
}

void A1d_(){
  if (digitalRead(BC1)){
    estadoMovimentoFlag = 1;
    movimentoElev = A1c;
  }
  else{
    if (chamadoT == AtivT && !digitalRead(SI)){
      estadoMovimentoFlag = 1;
      movimentoElev = DescT;
    }
    else{
      if ((chamadoA2 == AtivA2 || chamadoA3 == AtivA3) && chamadoT == DesatT && 
          !digitalRead(SI)){
        estadoMovimentoFlag = 1;
        movimentoElev = SubA2;
      }
    }
  }
}

void DescT_(){
  if (SP <= T){
    estadoMovimentoFlag = 1;
    display.clear();
    movimentoElev = Ta;
  }
}

void ProcessoMovimentacao(){
  switch (movimentoElev){
    case Ta:
      if (estadoMovimentoFlag){
        estadoMovimentoFlag = 0;
        mqttClient.publish("topicoMovimentoElev", "Ta");
      }
      Ta_();
      break;
    case Tb:
      if (estadoMovimentoFlag){
        estadoMovimentoFlag = 0;
        mqttClient.publish("topicoMovimentoElev", "Tb");
      }
      Tb_();
      break;
    case SubA1:
      if (estadoMovimentoFlag){
        estadoMovimentoFlag = 0;
        mqttClient.publish("topicoMovimentoElev", "SubA1");
      }
      SubA1_();
      break;
    case SubA2:
      if (estadoMovimentoFlag){
        estadoMovimentoFlag = 0;
        mqttClient.publish("topicoMovimentoElev", "SubA2");
      }
      SubA2_();
      break;
    case SubA3:
      if (estadoMovimentoFlag){
        estadoMovimentoFlag = 0;
        mqttClient.publish("topicoMovimentoElev", "SubA3");
      }
      SubA3_();
      break;
    case A3a:
      if (estadoMovimentoFlag){
        estadoMovimentoFlag = 0;
        mqttClient.publish("topicoMovimentoElev", "A3a");
      }
      A3a_();
      break;
    case A3b:
      if (estadoMovimentoFlag){
        estadoMovimentoFlag = 0;
        mqttClient.publish("topicoMovimentoElev", "A3b");
      }
      A3b_();
      break;
    case DescA2:
      if (estadoMovimentoFlag){
        estadoMovimentoFlag = 0;
        mqttClient.publish("topicoMovimentoElev", "DescA2");
      }
      DescA2_();
      break;
    case DescA1:
      if (estadoMovimentoFlag){
        estadoMovimentoFlag = 0;
        mqttClient.publish("topicoMovimentoElev", "DescA1");
      }
      DescA1_();
      break;
    case DescT:
      if (estadoMovimentoFlag){
        estadoMovimentoFlag = 0;
        mqttClient.publish("topicoMovimentoElev", "DescT");
      }
      DescT_();
      break;
    case A1a:
      if (estadoMovimentoFlag){
        estadoMovimentoFlag = 0;
        mqttClient.publish("topicoMovimentoElev", "A1a");
      }
      A1a_();
      break;
    case A1b:
      if (estadoMovimentoFlag){
        estadoMovimentoFlag = 0;
        mqttClient.publish("topicoMovimentoElev", "A1b");
      }
      A1b_();
      break;
    case A1c:
      if (estadoMovimentoFlag){
        estadoMovimentoFlag = 0;
        mqttClient.publish("topicoMovimentoElev", "A1c");
      }
      A1c_();
      break;
    case A1d:
      if (estadoMovimentoFlag){
        estadoMovimentoFlag = 0;
        mqttClient.publish("topicoMovimentoElev", "A1d");
      }
      A1d_();
      break;
    case A2a:
      if (estadoMovimentoFlag){
        estadoMovimentoFlag = 0;
        mqttClient.publish("topicoMovimentoElev", "A2a");
      }
      A2a_();
      break;
    case A2b:
      if (estadoMovimentoFlag){
        estadoMovimentoFlag = 0;
        mqttClient.publish("topicoMovimentoElev", "A2b");
      }
      A2b_();
      break;
    case A2c:
      if (estadoMovimentoFlag){
        estadoMovimentoFlag = 0;
        mqttClient.publish("topicoMovimentoElev", "A2c");
      }
      A2c_();
      break;
    case A2d:
      if (estadoMovimentoFlag){
        estadoMovimentoFlag = 0;
        mqttClient.publish("topicoMovimentoElev", "A2d");
      }
      A2d_();
      break;
  }
}

void Parado_(){
  PI_ = SP;
  digitalWrite(C2, LOW);
  digitalWrite(C1, LOW);
  mqttClient.publish("topicoC2LED", "LOW");
  mqttClient.publish("topicoC1LED", "LOW");
  analogWrite(MOTOR, 0);
  if (movimentoElev == SubA1 || movimentoElev == SubA2 || movimentoElev == SubA3){
    estadoAcionamentoFlag = 1;
    acionamentoElev = IniSub;
  }
  else{
    if (movimentoElev == DescT || movimentoElev == DescA1 || movimentoElev == DescA2){
      estadoAcionamentoFlag = 1;
      acionamentoElev = IniDesc;
    }
  }
}

void IniSub_(){
  digitalWrite(C2, HIGH);
  digitalWrite(C1, LOW);
  mqttClient.publish("topicoC2LED", "HIGH");
  mqttClient.publish("topicoC1LED", "LOW");
  if (/*SP > distancia1*/SP - PI_ > 50){
    estadoAcionamentoFlag = 1;
    acionamentoElev = Subindo;
  }
}

void Subindo_(){
  digitalWrite(C2, HIGH);
  digitalWrite(C1, LOW);
  mqttClient.publish("topicoC2LED", "HIGH");
  mqttClient.publish("topicoC1LED", "LOW");
  if (/*SP > distancia2*/SP - PI_ > 270){
    estadoAcionamentoFlag = 1;
    acionamentoElev = FimSub;
  }
}

void FimSub_(){
  digitalWrite(C2, HIGH);
  digitalWrite(C1, LOW);
  mqttClient.publish("topicoC2LED", "HIGH");
  mqttClient.publish("topicoC1LED", "LOW");
  if (movimentoElev == A1a || movimentoElev == A2a || movimentoElev == A3a){
    estadoAcionamentoFlag = 1;
    acionamentoElev = Parado;
  }
}

void IniDesc_(){
  digitalWrite(C2, HIGH);
  digitalWrite(C1, HIGH);
  mqttClient.publish("topicoC2LED", "HIGH");
  mqttClient.publish("topicoC1LED", "HIGH");
  if (/*SP < distancia1*/ PI_ - SP > 50){
    estadoAcionamentoFlag = 1;
    acionamentoElev = Descendo;
  }
}

void Descendo_(){
  digitalWrite(C2, HIGH);
  digitalWrite(C1, HIGH);
  mqttClient.publish("topicoC2LED", "HIGH");
  mqttClient.publish("topicoC1LED", "HIGH");
  if (/*SP < distancia2*/PI_ - SP > 270){
    estadoAcionamentoFlag = 1;
    acionamentoElev = FimDesc;
  }
}

void FimDesc_(){
  digitalWrite(C2, HIGH);
  digitalWrite(C1, HIGH);
  mqttClient.publish("topicoC2LED", "HIGH");
  mqttClient.publish("topicoC1LED", "HIGH");
  if (movimentoElev == A2c || movimentoElev == A1c || movimentoElev == Ta){
    estadoAcionamentoFlag = 1;
    acionamentoElev = Parado;
  }
}

void ProcessoAcionamento(){
  switch (acionamentoElev){
    case Parado:
      if (estadoAcionamentoFlag){
        estadoAcionamentoFlag = 0;
        mqttClient.publish("topicoAcionamentoElev", "Parado");
      }
      Parado_();
      break;
    case IniSub:
      if (estadoAcionamentoFlag){
        estadoAcionamentoFlag = 0;
        mqttClient.publish("topicoAcionamentoElev", "IniSub");
      }
      IniSub_();
      break;
    case Subindo:
      if (estadoAcionamentoFlag){
        estadoAcionamentoFlag = 0;
        mqttClient.publish("topicoAcionamentoElev", "Subindo");
      }
      Subindo_();
      break;
    case FimSub:
      if (estadoAcionamentoFlag){
        estadoAcionamentoFlag = 0;
        mqttClient.publish("topicoAcionamentoElev", "FimSub");
      }
      FimSub_();
      break;
    case IniDesc:
      if (estadoAcionamentoFlag){
        estadoAcionamentoFlag = 0;
        mqttClient.publish("topicoAcionamentoElev", "IniDesc");
      }
      IniDesc_();
      break;
    case Descendo:
      if (estadoAcionamentoFlag){
        estadoAcionamentoFlag = 0;
        mqttClient.publish("topicoAcionamentoElev", "Descendo");
      }
      Descendo_();
      break;
    case FimDesc:
      if (estadoAcionamentoFlag){
        estadoAcionamentoFlag = 0;
        mqttClient.publish("topicoAcionamentoElev", "FimDesc");
      }
      FimDesc_();
      break;
  }
}

double AcelRampaT_A1(){
  char auxChar[5];
  distancia1 = (a1*0.3) + T;
  distancia2 = (a1*0.7) + T;
  double m1 = VM/(a1*0.3);
  double m2 = -VM/abs((a1-(a1*0.7)));
  if (SP <= ((a1*0.3) + T)){
    porcentagem = m1*(SP-T);
  }
  else{
    if (SP >= ((a1*0.7) + T)){
      porcentagem = VM + m2*((SP-T) - a1*0.7);
    }
    else{
      porcentagem = VM;
    }
  }  
  if (porcentagem <= VM && porcentagem >= 0){
    if (porcentagem != Motoraux){
      Motoraux = porcentagem;
      sprintf(auxChar, "%.1f%%", Motoraux);
      mqttClient.publish("topicoMotor", auxChar);
    }
  }
  analogWrite(MOTOR, porcentagem);
  return porcentagem;
}

double AcelRampaT_A2(){
  char auxChar[5];
  distancia1 = ((a2+a1)*0.3) + T;
  distancia2 = ((a2+a1)*0.7) + T;
  double m1 = VM/((a2+a1)*0.3);
  double m2 = -VM/abs(((a2+a1)-((a2+a1)*0.7)));
  if (SP <= (((a2+a1)*0.3) + T)){
    porcentagem = m1*(SP-T);
  }
  else{
    if (SP >= (((a2+a1)*0.7) + T)){
      porcentagem = VM + m2*((SP-T) - (a2+a1)*0.7);
    }
    else{
      porcentagem = VM;
    }
  }
  if (porcentagem <= VM && porcentagem >= 0){
    if (porcentagem != Motoraux){
      Motoraux = porcentagem;
      sprintf(auxChar, "%.1f%%", Motoraux);
      mqttClient.publish("topicoMotor", auxChar);
    }
  }
  analogWrite(MOTOR, porcentagem);
  return porcentagem;
}

double AcelRampaT_A3(){
  char auxChar[5];
  distancia1 = ((a3+a2+a1)*0.3) + T;
  distancia2 = ((a3+a2+a1)*0.7) + T;
  double m1 = VM/((a3+a2+a1)*0.3);
  double m2 = -VM/abs(((a3+a2+a1)-((a3+a2+a1)*0.7)));
  if (SP <= (((a3+a2+a1)*0.3) + T)){
    porcentagem = m1*(SP-T);
  }
  else{
    if (SP >= (((a3+a2+a1)*0.7) + T)){
      porcentagem = VM + m2*((SP-T) - (a3+a2+a1)*0.7);
    }
    else{
      porcentagem = VM;
    }
  }
  if (porcentagem <= VM && porcentagem >= 0){
    if (porcentagem != Motoraux){
      Motoraux = porcentagem;
      sprintf(auxChar, "%.1f%%", Motoraux);
      mqttClient.publish("topicoMotor", auxChar);
    }
  }
  analogWrite(MOTOR, porcentagem);
  return porcentagem;
}

double AcelRampaA1_A2(){
  char auxChar[5];
  distancia1 = (a2*0.3) + a1 + T;
  distancia2 = (a2*0.7) + a1 + T;
  double m1 = VM/(a2*0.3);
  double m2 = -VM/abs((a2-(a2*0.7)));
  if (SP <= ((a2*0.3) + a1 + T)){
    porcentagem = m1*(SP-a1-T);
  }
  else{
    if (SP >= ((a2*0.7) + a1 + T)){
      porcentagem = VM + m2*((SP-a1-T) - a2*0.7);
      if (porcentagem < 0){
        porcentagem = 0;
      }
    }
    else{
      porcentagem = VM;
    }
  }
  if (porcentagem <= VM && porcentagem >= 0){
    if (porcentagem != Motoraux){
      Motoraux = porcentagem;
      sprintf(auxChar, "%.1f%%", Motoraux);
      mqttClient.publish("topicoMotor", auxChar);
    }
  }
  analogWrite(MOTOR, porcentagem);
  return porcentagem;
}

double AcelRampaA1_A3(){
  char auxChar[5];
  distancia1 = ((a3+a2)*0.3) + a1 + T;
  distancia2 = ((a3+a2)*0.7) + a1 + T;
  double m1 = VM/((a3+a2)*0.3);
  double m2 = -VM/abs(((a3+a2)-((a3+a2)*0.7)));
  if (SP <= (((a3+a2)*0.3) + a1 + T)){
    porcentagem = m1*(SP - a1 - T);
  }
  else{
    if (SP >= (((a3+a2)*0.7) + a1 + T)){
      porcentagem = VM + m2*((SP - a1 - T) - (a3+a2)*0.7);
    }
    else{
      porcentagem = VM;
    }
  }
  if (porcentagem <= VM && porcentagem >= 0){
    if (porcentagem != Motoraux){
      Motoraux = porcentagem;
      sprintf(auxChar, "%.1f%%", Motoraux);
      mqttClient.publish("topicoMotor", auxChar);
    }
  }
  analogWrite(MOTOR, porcentagem);
  return porcentagem;
}

double AcelRampaA2_A3(){
  char auxChar[5];
  distancia1 = (a3*0.3) + a1 + a2 + T;
  distancia2 = (a3*0.7) + a1 + a2 + T;
  double m1 = VM/(a3*0.3);
  double m2 = -VM/abs((a3-(a3*0.7)));
  if (SP <= ((a3*0.3) + a1 + a2 + T)){
    porcentagem = m1*(SP - a1 - a2 - T);
  }
  else{
    if (SP >= ((a3*0.7) + a1 + a2 + T)){
      porcentagem = VM + m2*((SP - a1 - a2 - T) - a3*0.7);
    }
    else{
      porcentagem = VM;
    }
  }
  if (porcentagem <= VM && porcentagem >= 0){
    if (porcentagem != Motoraux){
      Motoraux = porcentagem;
      sprintf(auxChar, "%.1f%%", Motoraux);
      mqttClient.publish("topicoMotor", auxChar);
    }
  }
  analogWrite(MOTOR, porcentagem);
  return porcentagem;
}

double AcelRampaA1_T(){
  char auxChar[5];
  distancia1 = (a1*0.7) + T;
  distancia2 = (a1*0.3) + T;
  double m1 = VM/(a1*0.3);
  double m2 = -VM/abs((a1-(a1*0.7)));
  if (SP >= ((a1*0.7) + T)){
     porcentagem = VM + m2*((SP - T) - a1*0.7);
  }
  else{
    if (SP >= ((a1*0.3) + T)){
      porcentagem = VM;
    }
    else{
      porcentagem = m1*(SP - T);
    }
  }
  if (porcentagem <= VM && porcentagem >= 0){
    if (porcentagem != Motoraux){
      Motoraux = porcentagem;
      sprintf(auxChar, "%.1f%%", Motoraux);
      mqttClient.publish("topicoMotor", auxChar);
    }
  }
  analogWrite(MOTOR, porcentagem);
  return porcentagem;
}

double AcelRampaA2_T(){
  char auxChar[5];
  distancia1 = ((a1+a2)*0.7) + T;
  distancia2 = ((a1+a2)*0.3) + T;
  double m1 = VM/((a1+a2)*0.3);
  double m2 = -VM/abs(((a1+a2)-((a1+a2)*0.7)));
  if (SP >= (((a1+a2)*0.7) + T)){
     porcentagem = VM + m2*((SP - T) - (a1+a2)*0.7);
  }
  else{
    if (SP >= (((a1+a2)*0.3) + T)){
      porcentagem = VM;
    }
    else{
      porcentagem = m1*(SP - T);
    }
  }
  if (porcentagem <= VM && porcentagem >= 0){
    if (porcentagem != Motoraux){
      Motoraux = porcentagem;
      sprintf(auxChar, "%.1f%%", Motoraux);
      mqttClient.publish("topicoMotor", auxChar);
    }
  }
  analogWrite(MOTOR, porcentagem);
  return porcentagem;
}

double AcelRampaA3_T(){
  char auxChar[5];
  distancia1 = ((a1+a2+a3)*0.7) + T;
  distancia2 = ((a1+a2+a3)*0.3) + T;
  double m1 = VM/((a1+a2+a3)*0.3);
  double m2 = -VM/abs(((a1+a2+a3)-((a1+a2+a3)*0.7)));
  if (SP >= (((a1+a2+a3)*0.7) + T)){
     porcentagem = VM + m2*((SP - T) - (a1+a2+a3)*0.7);
  }
  else{
    if (SP >= (((a1+a2+a3)*0.3) + T)){
      porcentagem = VM;
    }
    else{
      porcentagem = m1*(SP - T);
    }
  }
  if (porcentagem <= VM && porcentagem >= 0){
    if (porcentagem != Motoraux){
      Motoraux = porcentagem;
      sprintf(auxChar, "%.1f%%", Motoraux);
      mqttClient.publish("topicoMotor", auxChar);
    }
  }
  analogWrite(MOTOR, porcentagem);
  return porcentagem;
}

double AcelRampaA3_A2(){
  char auxChar[5];
  distancia1 = (a3*0.7) + a1 + a2 + T;
  distancia2 = (a3*0.3) + a1 + a2 + T;
  double m1 = VM/(a3*0.3);
  double m2 = -VM/abs(a3-(a3*0.7));
  if (SP >= ((a3*0.7) + a1 + a2 + T)){
     porcentagem = VM + m2*((SP - a1 - a2 - T) - a3*0.7);
  }
  else{
    if (SP >= ((a3*0.3) + a1 + a2 + T)){
      porcentagem = VM;
    }
    else{
      porcentagem = m1*(SP - a1 - a2 - T);
    }
  }
  if (porcentagem <= VM && porcentagem >= 0){
    if (porcentagem != Motoraux){
      Motoraux = porcentagem;
      sprintf(auxChar, "%.1f%%", Motoraux);
      mqttClient.publish("topicoMotor", auxChar);
    }
  }
  analogWrite(MOTOR, porcentagem);
  return porcentagem;
}

double AcelRampaA3_A1(){
  char auxChar[5];
  distancia1 = ((a3+a2)*0.7) + a1 + T;
  distancia2 = ((a3+a2)*0.3) + a1 + T;
  double m1 = VM/((a3+a2)*0.3);
  double m2 = -VM/abs((a3+a2)-((a3+a2)*0.7));
  if (SP >= (((a3+a2)*0.7) + a1 + T)){
     porcentagem = VM + m2*((SP - a1 - T) - (a3+a2)*0.7);
  }
  else{
    if (SP >= (((a3+a2)*0.3) + a1 + T)){
      porcentagem = VM;
    }
    else{
      porcentagem = m1*(SP - a1 - T);
    }
  }
  if (porcentagem <= VM && porcentagem >= 0){
    if (porcentagem != Motoraux){
      Motoraux = porcentagem;
      sprintf(auxChar, "%.1f%%", Motoraux);
      mqttClient.publish("topicoMotor", auxChar);
    }
  }
  analogWrite(MOTOR, porcentagem);
  return porcentagem;
}

double AcelRampaA2_A1(){
  char auxChar[5];
  distancia1 = (a2*0.7) + a1 + T;
  distancia2 = (a2*0.3) + a1 + T;
  double m1 = VM/(a2*0.3);
  double m2 = -VM/abs(a2-(a2*0.7));
  if (SP >= (((a2*0.7) + a1 + T))){
     porcentagem = VM + m2*((SP - a1 - T) - a2*0.7);
  }
  else{
    if (SP >= ((a2*0.3) + a1 + T)){
      porcentagem = VM;
    }
    else{
      porcentagem = m1*(SP - a1 - T);
    }
  }
  if (porcentagem <= VM && porcentagem >= 0){
    if (porcentagem != Motoraux){
      Motoraux = porcentagem;
      sprintf(auxChar, "%.1f%%", Motoraux);
      mqttClient.publish("topicoMotor", auxChar);
    }
  }
  analogWrite(MOTOR, porcentagem);
  return porcentagem;
}

void AjusteDeRampa(){
  switch (movimentoElev){
    case SubA1:
      if (chamadoA1 == AtivA1 && !digitalRead(SI) && estadoAnterior == Ta){
        AcelRampaT_A1();
      }
      else{
        if (chamadoA2 == AtivA2 && !digitalRead(SI) && estadoAnterior == Ta){
          AcelRampaT_A2();
        }
        else{
          if (chamadoA3 == AtivA3 && !digitalRead(SI) && estadoAnterior == Ta){
            AcelRampaT_A3();
          }
        }
      }
      break;
    case SubA2:
      if (chamadoA2 == AtivA2 && !digitalRead(SI) && estadoAnterior == Ta){
        AcelRampaT_A2();
      }
      else{
        if (chamadoA3 == AtivA3 && !digitalRead(SI) && estadoAnterior == Ta){
          AcelRampaT_A3();
        }
        else{
          if (chamadoA2 == AtivA2 && !digitalRead(SI) && estadoAnterior == A1a){
            AcelRampaA1_A2();
          }
          else{
            if (chamadoA3 == AtivA3 && !digitalRead(SI) && estadoAnterior == A1a){
              AcelRampaA1_A3();
            }
            else{
              if (chamadoA2 == AtivA2 && !digitalRead(SI) && estadoAnterior == A1c){
                AcelRampaA1_A2();
              }
              else{
                if (chamadoA3 == AtivA3 && !digitalRead(SI) && estadoAnterior == A1c){
                  AcelRampaA1_A3();
                }
              }
            }
          }
        }
      }
      break;
    case SubA3:
      if (chamadoA3 == AtivA3 && !digitalRead(SI) && estadoAnterior == Ta){
        AcelRampaT_A3();
      }
      else{
        if (chamadoA3 == AtivA3 && !digitalRead(SI) && estadoAnterior == A1a){
          AcelRampaA1_A3();
        }
        else{
          if (chamadoA3 == AtivA3 && !digitalRead(SI) && estadoAnterior == A2a){
            AcelRampaA2_A3();
          }
          else{
            if (chamadoA3 == AtivA3 && !digitalRead(SI) && estadoAnterior == A1c){
              AcelRampaA1_A3();
            }
            else{
              if (chamadoA3 == AtivA3 && !digitalRead(SI) && estadoAnterior == A1c){
                AcelRampaA2_A3();
              }
            }
          }
        }
      }
      break;
    case DescA2:
      if (chamadoT == AtivT && !digitalRead(SI) && estadoAnterior == A3a){
          AcelRampaA3_T();
      }
      else{
        if (chamadoA1 == AtivA1 && !digitalRead(SI) && estadoAnterior == A3a){
          AcelRampaA3_A1();
        }
        else{
          if (chamadoA2 == AtivA2 && !digitalRead(SI) && estadoAnterior == A3a){
            AcelRampaA3_A2();
          }
        }
      }
      break;
    case DescA1:
      if (chamadoT == AtivT && !digitalRead(SI) && estadoAnterior == A3a){
        AcelRampaA3_T();
      }
      else{
        if (chamadoA1 == AtivA1 && !digitalRead(SI) && estadoAnterior == A3a){
          AcelRampaA3_A1();
        }
        else{
          if (chamadoT == AtivT && !digitalRead(SI) && estadoAnterior == A2c){
            AcelRampaA2_T();
          }
          else{
            if (chamadoA1 == AtivA1 && !digitalRead(SI) && estadoAnterior == A2c){
              AcelRampaA2_A1();
            }
            else{
              if (chamadoT == AtivT && !digitalRead(SI) && estadoAnterior == A2a){
                AcelRampaA2_T();
              }
              else{
                if (chamadoA1 == AtivA1 && !digitalRead(SI) && estadoAnterior == A2a){
                  AcelRampaA2_A1();
                }
              }
            }
          }
        }
      }
    break;
    case DescT:
      if (chamadoT == AtivT && !digitalRead(SI) && estadoAnterior == A3a){
        AcelRampaA3_T();
      }
      else{
        if (chamadoT == AtivT && !digitalRead(SI) && estadoAnterior == A2c){
          AcelRampaA2_T();
        }
        else{
          if (chamadoT == AtivT && !digitalRead(SI) && estadoAnterior == A2a){
            AcelRampaA2_T();
          }
          else{
            if (chamadoT == AtivT && !digitalRead(SI) && estadoAnterior == A1c){
              AcelRampaA1_T();
            }
            else{
              if (chamadoT == AtivT && !digitalRead(SI) && estadoAnterior == A1a){
                AcelRampaA1_T();
              }
            }
          }
        }
      }
      break;
  }
}

void DistEncoder(){
  char auxChar[8];
  if (SP != SPaux){
    SPaux = SP;
    sprintf(auxChar, "%i cm", SPaux);
    mqttClient.publish("topico_SP", auxChar);
  }
}

void mostrarNumeroCentralizado(int numero) {
  int numeroDeDigitos = 1;
  int posicaoInicial = (4 - numeroDeDigitos) / 2; 
  
  display.showNumberDec(numero, false, numeroDeDigitos, posicaoInicial);
}

void conexaoWiFi() {
  //Conexão ao Wi-Fi
  Serial.print("Conectando-se ao Wi-Fi ");  
  Serial.print(ssid);
  Serial.print(" ");
  WiFi.begin(ssid, key, 6);
  while (WiFi.status() != WL_CONNECTED) {
    delay(100);
    Serial.print(".");
  }
  Serial.println(" Conectado!");
}

void conexaoBroker() {
  //Conexão ao Broker
  mqttClient.setServer(broker, port);
  while (!mqttClient.connected()) {
    Serial.print("Conectando-se ao broker ");    
    //if (mqttClient.connect(WiFi.macAddress().c_str())) {
    if (mqttClient.connect("UMNOMEMUITODIFICILDEADVINHAR")) {
      Serial.println(" Conectado!");            
    } else {
      Serial.print(".");      
      delay(100);
    }
  }
}

void loop() {
  SP = encoder.readEncoder();
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Reconexão ao Wi-Fi...");
    conexaoWiFi();
  }
  if (!mqttClient.connected()) {
    Serial.println("Reconexão ao broker...");
    conexaoBroker();
  }
  ProcessoParametros();
  ProcessoChamados();
  ProcessoMovimentacao();
  ProcessoAcionamento();
  AjusteDeRampa();
  DistEncoder();
  
  delay(500);
}
