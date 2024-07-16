
#include <SparkFun_TB6612.h>
#include <QTRSensors.h>
#include <SoftwareSerial.h>

//Declarando variveis dos motores
#define motorLeftSpeed    11
#define motorLeftA        10
#define motorLeftB        9
#define stby              8
#define motorRightA       6
#define motorRightB       7
#define motorRightSpeed   5

const int offsetA =  1;
const int offsetB = -1;

//Estanciando biblioteca dos motores
Motor motorLeft  = Motor(motorLeftA,  motorLeftB,  motorLeftSpeed,  offsetA, stby);
Motor motorRight = Motor(motorRightA, motorRightB, motorRightSpeed, offsetB, stby);

//Variaveis Auxiliares Velocidade
#define speedMin   0
#define speedMax   100
int speedBase = 40;
int speedBoost = 40;
int speedFinal = 40;
int tim = 50;
int TempoDaSalvacao = 10000;
int controleLimite = 200;

//Declarando variaveis do Array
QTRSensors arraySensors;
const uint8_t sensorCount = 6;
uint16_t valuesSensors[sensorCount];

#define emitterPin 12

//VARIAVEIS PID
float   Kp = 1.45,
        Ki = 0.0,
        Kd = 15.0;
        
uint16_t position = 0;

int error         = 0;
int setPoint      = 50;

int P = 0;
int I = 0;
int D = 0;

int PIDValue  = 0;

int lastError = 0;
int countI    = 0;

/*SENSORES LATERAIS*/
#define sensorSideLeft  A7
#define sensorSideRight A6

uint16_t vSSLeft  = 0,
         vSSRight = 0;

int media = 600;
int final = 0;
int contMilli[] = {0,0}; 

int geo  = 0;
int geo1 = 0;
int geo2 = 0;
int geo3 = 0;
int geo4 = 0;
int geo5 = 0;

/*BLUETOOTH*/
const int pinoRX = 0;
const int pinoTX = 1;
int dadoBluetooth = 0;

SoftwareSerial bluetooth(pinoRX, pinoTX);

/*BUZZER*/
#define buzzer 13

/*LEDs*/
#define led 4

/*BOTÔES*/
const int btnLeft  = 2;
const int btnRight = 3;

/*CONFIG INICIAL DOS SENSORES*/
void initRobot(){
  //Inicializando Array
  arraySensors.setTypeAnalog();
  arraySensors.setSensorPins((const uint8_t[]){A5, A4, A3, A2, A1, A0}, sensorCount);
  arraySensors.setEmitterPin(emitterPin);

  //Led e Buzzer
  onLed(500);
  onBuzzer(100);
  onLed(500);
}

/*FUNÇÃO PARA PISCAR LED E APITAR BUZZER NO COMEÇO DA CORRIDA*/
void startRun(){
  onBuzzer(1000);
  delay(10);
  onLed();
  onBuzzer(100);
  delay(10);
  offLed();
  onBuzzer(100);
  delay(10);
  onLed();
  onBuzzer(100);
  delay(10);
  offLed();
  onBuzzer(1000);
}

/*FUNÇÃO PARA PISCAR LED E APITAR BUZZER NO FINAL DA CORRIDA*/
void finishRun(){
  motorLeft.brake();
  motorRight.brake();
  onBuzzer(100);
  delay(10);
  onLed();
  onBuzzer(100);
  delay(10);
  offLed();
  onBuzzer(100);
  delay(10);
  onLed();
  onBuzzer(100);
  delay(10);
  offLed();
  onBuzzer(100);
}

/*FUNÇÕES DO LED*/
void onLed(int time){ digitalWrite(led, HIGH); delay(time); digitalWrite(led, LOW);}
void onLed(){digitalWrite(led, HIGH);}
void offLed(){digitalWrite(led, LOW);}

/*FUNÇÕES DO BUZZER*/
void onBuzzer(int time){digitalWrite(buzzer, HIGH);delay(time);digitalWrite(buzzer, LOW);}
void onBuzzer(){digitalWrite(buzzer, HIGH);}
void offBuzzer(){digitalWrite(buzzer, LOW);}

/*FUNÇÕES PARA CHECAR SE O BOTÃO FOI PRECIONADO*/
bool btnPressed(int btn){
  if(digitalRead(btn) == LOW)
    return true;
  else
    return false;
}

void calibrateArray() {
  Serial.println("Calibrando Sensores Array...");

  delay(1000);

  onLed();
  onBuzzer(10);

  for(uint16_t i = 0; i < 200; i++){ 
    arraySensors.calibrate(); 
    
    }

  onBuzzer(10);
  offLed();

  Serial.println("Sensores Calibrados!");

  delay(2000);
}


void followLine(){
  //Parada pelo Sensor Lateral
  detectMarker();

  // Cálculo do PID.
  position = arraySensors.readLineWhite(valuesSensors);

  position = map(position, 0, 5000, 0, 100);
  
  error = position - setPoint;

  P = error;
  D = (error - lastError);

  if (D >=  50) D =  50;
  if (D <= -50) D = -50;

  PIDValue = (P*Kp) + (D*Kd);

  if (PIDValue >=  50) PIDValue =  50;
  if (PIDValue <= -50) PIDValue = -50;

  // Atualiza os valores do erro Anterior.
  lastError = error;

  // Incremento da velocidade
  if(position>40 && position<60){
    if(speedFinal<speedBoost){
      speedFinal = speedFinal+0.5;      
    }

    if(contMilli[0]==0){
      contMilli[0] = millis();
    }else{
      contMilli[1] = millis();
    }
    
    if(contMilli[1]-contMilli[0]>=TempoDaSalvacao && speedFinal<70 && TempoDaSalvacao!=10000){
      speedFinal = speedFinal + 5;
    }
    
  }else{
    speedFinal = speedBase;
    contMilli[0]=0;
    contMilli[1]=0;
  }
  
  // Atribuição da velocidade dos motores.
  int speedRight = speedFinal + PIDValue;
  int speedLeft  = speedFinal - PIDValue;

  // Checa se as velocidades ultrapassam 255 ou são mensores que 0, corrige caso necessario
  if(speedRight > speedMax) speedRight = speedMax;
  if(speedLeft  > speedMax) speedLeft = speedMax;
  
  if(speedRight < speedMin) speedRight = speedMin;
  if(speedLeft  < speedMin) speedLeft  = speedMin;

  //Redução geral da velocidade
  speedRight = map(speedRight, 0, 100, 0, controleLimite);
  speedLeft  = map(speedLeft,  0, 100, 0, controleLimite);

  //Atribui as velocidades aos motores
  motorLeft.drive(speedLeft);
  motorRight.drive(speedRight);  
}

void readSideSensors() {
  vSSLeft  = analogRead(sensorSideLeft);
  vSSRight = analogRead(sensorSideRight);
    
  if(vSSLeft < media) vSSLeft = 1; 
  else vSSLeft = 0;
  
  if(vSSRight < media) vSSRight = 1; 
  else vSSRight = 0;
  
}

void detectMarker() {
  readSideSensors();

  if(vSSLeft == 0 && vSSRight == 0) { geo = 0; }
  if(vSSLeft == 1 && vSSRight == 0) { geo = 1; }
  if(vSSLeft == 0 && vSSRight == 1) { geo = 2; }
  if(vSSLeft == 1 && vSSRight == 1) { geo = 3; }

  if(geo1 == geo){
    offLed();
    offBuzzer();
  }
  else if(geo1 != geo) {
    if(geo == 0 && geo1 == 1 && geo2 == 0) {
      markerLeft();
    }
    else if(geo == 0 && geo1 == 2 && geo2 == 0) {
      markerRight();
    }
    else if(geo == 0 && ((geo1 == 3) || (geo2 == 3) || (geo3 == 3))) {
      intersection();
    }
    geo5 = geo4;
    geo4 = geo3;
    geo3 = geo2;
    geo2 = geo1;
    geo1 = geo;
  }
}

void intersection(){ onLed(); onBuzzer();}

void markerLeft(){ onLed(); onBuzzer();}

void markerRight() {
  onLed();
  onBuzzer();

  final++;

  if(final >= 2) {
    offBuzzer();
    offLed();
    delay(tim);
    motorLeft.brake();
    motorRight.brake();
    delay(5000);
    contMilli[0] = 0;
    final = 0;
  } 
}

void set_PD_40(){
  speedBase = 30;
  speedBoost = 30;
  controleLimite = 105;
  Kp = 1.45;
  Kd = 15.00;
  
  delay(20);
  onBuzzer(50);
  delay(20);
}
void set_PD_30(){
  speedBase = 30;
  speedBoost = 30;
  Kp = 1.45;
  Kd = 15.00;

  delay(20);
  onBuzzer(50);
  delay(20);
}
void set_PD_20(){
  speedBase = 20;
  speedBoost = 25;
  Kp = 0.75;
  Kd = 3.00;  
  
  delay(20);
  onBuzzer(50);
  delay(20);
}
void set_P_Mais(){
  Kp = Kp + 0.25;
}
void set_P_Menos(){
  Kp = Kp - 0.25;
}
void set_D_Mais(){
  Kd = Kd + 0.25;
}
void set_D_Menos(){
  Kd = Kd - 0.25;
}
void set_Speed_Mais(){
  speedBase = speedBase + 1;  
}
void set_Speed_Menos(){
  speedBase = speedBase - 1;
}
void set_SpeedBoost_Mais(){
  speedBoost = speedBoost + 1;
}
void set_SpeedBoost_Menos(){
  speedBoost = speedBoost - 1;
}
void teste_Sensores(){
  for(int i=0; i<8; i++){
    position = arraySensors.readLineWhite(valuesSensors);
    position = map(position, 0, 5000, 0, 100);
    Serial.println(position);
    delay(1000);
  }
}

void setup() {
  Serial.begin(9600);

  //Setas os pinos como entrada ou saida
  pinMode(motorLeftSpeed,   OUTPUT);
  pinMode(motorLeftA,       OUTPUT);
  pinMode(motorLeftB,       OUTPUT);
  pinMode(stby,             OUTPUT);
  pinMode(motorLeftB,       OUTPUT);
  pinMode(motorLeftA,       OUTPUT);
  pinMode(motorLeftSpeed,   OUTPUT);
  
  pinMode(buzzer,           OUTPUT);
  pinMode(led,              OUTPUT);
  pinMode(btnLeft,          INPUT_PULLUP);
  pinMode(btnRight,         INPUT_PULLUP);   

  initRobot();  

  bluetooth.begin(9600);
  bluetooth.print("$");
  bluetooth.print("$");
  bluetooth.print("$");
}

void loop() {
  if(btnPressed(btnLeft)){
    startRun();
    while(!btnPressed(btnLeft) && bluetooth.available() == 0){
      followLine();
    }
    finishRun();
  }
  if(btnPressed(btnRight)){
    calibrateArray();
  }
  if(bluetooth.available() > 0){
      dadoBluetooth = bluetooth.read();
 
    if(dadoBluetooth == '5'){
      set_PD_40();
    }else if(dadoBluetooth == '3'){
      set_PD_30();
    }else if(dadoBluetooth  == '2'){
      set_PD_20();
    }else if(dadoBluetooth == 'P'){
      set_P_Mais();
    }else if(dadoBluetooth == 'p'){
      set_P_Menos();
    }else if(dadoBluetooth == 'D'){
      set_D_Mais();
    }else if(dadoBluetooth == 'd'){
      set_D_Menos();
    }else if(dadoBluetooth == 'S'){
      set_Speed_Mais();
    }else if(dadoBluetooth == 's'){
      set_Speed_Menos();
    }else if(dadoBluetooth == 'I'){
      set_SpeedBoost_Mais();
    }else if(dadoBluetooth == 'i'){
      set_SpeedBoost_Menos();
    }else if(dadoBluetooth == 'T'){
      teste_Sensores();
    }else if(dadoBluetooth == 'B'){
      TempoDaSalvacao = TempoDaSalvacao + 1000;
    }else if(dadoBluetooth == 'b'){
      TempoDaSalvacao = TempoDaSalvacao - 1000;
    }else if(dadoBluetooth == 'C'){
      controleLimite = controleLimite + 5;
    }else if(dadoBluetooth == 'c'){
      controleLimite = controleLimite - 5;
    }else if(dadoBluetooth == 'Y'){
      controleLimite = controleLimite + 1;
    }else if(dadoBluetooth == 'y'){
      controleLimite = controleLimite - 1;
    }else if(dadoBluetooth == 'K'){
      tim = tim + 10;
    }else if(dadoBluetooth == 'k'){
      tim = tim - 10;
    }

    

    if(dadoBluetooth != 'T' && dadoBluetooth != 'w'){
      Serial.println(speedBase);
      Serial.println(speedBoost);
      Serial.println(Kp);
      Serial.println(Kd);
      Serial.println(TempoDaSalvacao);
      Serial.println(controleLimite);
      Serial.println(tim);
    }
  }
}
