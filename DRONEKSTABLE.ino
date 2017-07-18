/////////////////////////////////////INCLUSION DE LIBRERIAS WIFI///////////////////////////////
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
////////////////////////////////////INCLUSION DE LIBRERIAS PID/////////////////////////////////
#include "I2Cdev.h"
#include <PID_v1.h>
#include "Wire.h"
#include <Servo.h>
#include "MPU6050_6Axis_MotionApps20.h"
////////////////////////////////////DEFINICIONES DEL WIFI/////////////////////////////////
const char* ssid = "TazAtherton";
const char* password = "29605091";
const char* mqtt_server = "192.168.0.20";
//CONF PUBSUBLIB
WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
char msg[50];
int value = 0;
////////////////////////////////////DEFINICIONES DEL PID////////////////////////////////
/*************************
*          PID           *
*         LIMITES        *
*************************/
static uint16_t unThrottleIn;

#define OUTPUT_LIMITS 50
///////////////////////CARIABLES DE MOTORES//////////////////
int outputTL, outputTR, outputBR, outputBL, auxTL, auxTR, auxBR, auxBL;
int mpuYaw, mpuPitch, mpuRoll;
int motorGain = 35;
///////////////////////VARIABLES DEL PID/////////////////////
double pitchSetpoint, pitchInput, pitchOutput;
double rollSetpoint, rollInput, rollOutput;
//////////////////////////TUNING DEL PID/////////////////////
double PitchaggKp=.80, PitchaggKi=0, PitchaggKd=0;
double PitchconsKp=.53, PitchconsKi=0.02, PitchconsKd=0.12;

double RollaggKp=.80, RollaggKi=0.00, RollaggKd=0;
double RollconsKp=.53, RollconsKi=0.02, RollconsKd=0.12;
//Specify the links and initial tuning parameters
PID pitchPID(&pitchInput, &pitchOutput, &pitchSetpoint, PitchconsKp, PitchconsKi, PitchconsKd, DIRECT);
PID rollPID(&rollInput, &rollOutput, &rollSetpoint, RollconsKp, RollconsKi, RollconsKd, DIRECT);

long previousMillis = 0;
int derecha = 0; int izquierda = 0; int arriba = 0; int abajo = 0; 
//////////////////////definiciones y servos MPU////////////////////////
      MPU6050 mpu;////////definicion del MPU
      /////VARIABLES DEL MPU/////////////////////////////////////
      bool dmpReady = false;
      uint8_t mpuIntStatus;
      uint8_t devStatus;
      uint16_t packetSize;
      uint16_t fifoCount;
      uint8_t fifoBuffer[64];
      
      Quaternion q;
      VectorInt16 aa;
      VectorInt16 aaReal;
      VectorInt16 aaWorld;
      VectorFloat gravity;
      float euler[3];
      float ypr[3];
///////////////////////////////VARIBLE DE INTERRUPCIONES///////////////////////////////////////
      volatile bool mpuInterrupt = false;
      void dmpDataReady() {
          mpuInterrupt = true;
      }
      
#define MOTORTL_OUT_PIN 5  //D1   5
#define MOTORTR_OUT_PIN 0  //D3    0
#define MOTORBR_OUT_PIN 4 //D2     4
#define MOTORBL_OUT_PIN 16  //D0    16

Servo servoMotorTL;
Servo servoMotorTR;
Servo servoMotorBR;
Servo servoMotorBL;
//////////////////////////////


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////****SETUP****/////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////





void setup() {
derecha = 0; izquierda = 0; arriba = 0; abajo = 0; 
pitchInput = 0;
rollInput = 0;
///////////////////////CONFIGURAICON BASICA DEL PID//////////////////////////////
pitchPID.SetMode(AUTOMATIC);
rollPID.SetMode(AUTOMATIC);
pitchPID.SetOutputLimits(0, 30);///30 en ambos noc porque
rollPID.SetOutputLimits(0, 30);///30 en ambos noc porque
//////////////////////////////////SETUP MPU////////////////////////////////////////////
    Wire.begin(14, 12);
//////////////////////////////////SETUP WIFI///////////////////////////////////////////
    Serial.begin(115200);
    setup_wifi();
    client.setServer(mqtt_server, 1883);
    client.setCallback(callback);
//////////////////////////////////ATTACH MOTORES///////////////////////////////////////////
servoMotorTL.attach(MOTORTL_OUT_PIN);
servoMotorTR.attach(MOTORTR_OUT_PIN);
servoMotorBL.attach(MOTORBL_OUT_PIN);
servoMotorBR.attach(MOTORBR_OUT_PIN);
    arm();///ARMADO DE MOTORES
//////////////////////////////////INICIALIZACION DEL MPU///////////////////////////////////
        mpu.initialize();
        Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
        devStatus = mpu.dmpInitialize();
        //OFFSETS supongo que es la descalibracion minima por la posicion del modulo MPU
        mpu.setXGyroOffset(220);
        mpu.setYGyroOffset(76);
        mpu.setZGyroOffset(-85);
        mpu.setZAccelOffset(1788);
        // interrupcion o algo INICIO DEL MPU
        if (devStatus == 0) {
              mpu.setDMPEnabled(true);
              attachInterrupt(0, dmpDataReady, RISING);
              mpuIntStatus = mpu.getIntStatus();
              dmpReady = true;
              packetSize = mpu.dmpGetFIFOPacketSize();
          } else {
              Serial.print(F("DMP Initialization failed (code "));
              Serial.print(devStatus);
              Serial.println(F(")"));
          }
}



//SETTIMING PARAMS
// CALLBACK///////
                            int cortarPayload(String mensaje){
                              String mensajerecortado=mensaje.substring(0,4);
                              int velocidad = mensajerecortado.toInt();
                              if ((velocidad>=1000) && (velocidad<=1550)){
                                return velocidad;
                              }
                              else return 1000;
                              return mensajerecortado.toInt();
                            }
  void callback(char* topic, byte* payload, unsigned int length) 
  {
      Serial.print("Command from MQTT broker is : [");
      Serial.print(topic);
      int p =(char)payload[0]-'0';
      if(p==0){unThrottleIn=1000;}
      if(p==1){unThrottleIn=1280;}
      if(p==2){unThrottleIn=1350;}
      if(p==3){unThrottleIn=1400;}
      if(p==4){unThrottleIn=1450;}
      if(p==7){izquierda=90;}
      if(p==6){derecha=90;}
      if(p==9){abajo=90;}
      if(p==8){arriba=90;}
  } 
// ///CALLBACK




//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////****LOOP****/////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


double max_pitch=0;
double max_roll=0;



void loop() {
  if (!dmpReady) return;
  while (!mpuInterrupt && fifoCount < packetSize) {}
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();
  fifoCount = mpu.getFIFOCount();
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        mpu.resetFIFO();
        Serial.println(F("FIFO OVERFLOW!!!!!!!"));
    } else if (mpuIntStatus & 0x02) {   
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        fifoCount -= packetSize;
    }
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  mpuYaw = ypr[0] * 180/M_PI;
  mpuRoll = ypr[1] * 180/M_PI;
  mpuPitch = ypr[2] * 180/M_PI;
  
  pitchInput = mpuPitch;
  rollInput = mpuRoll;
/////////////////////////////////////////PID PITCH//////////////////////////////////////////////////////////////////
            pitchSetpoint = 0; // Valor Deseado (en grados)
            //pitchInput = map(unPitchIn, 900, 2000, -30, 30); // Valor de entrada (necesita convertirse a grados)
            //double Pitchgap = abs(pitchSetpoint-pitchInput); // Distancia hasta setpoint (Error)
            //if(Pitchgap<5) {  // Estamos lejos del setpoint, usar parametros conservativos
                //pitchPID.SetTunings(PitchconsKp, PitchconsKi, PitchconsKd);
                if(pitchOutput>=max_pitch||max_pitch==30)max_pitch=pitchOutput;
            //} else {
              // Estamos muy cerca del setpoint, usa parametros agresivos
              pitchPID.SetTunings(PitchaggKp, PitchaggKi, PitchaggKd);
            //}
            pitchPID.Compute();
/////////////////////////////////////////PID ROLL//////////////////////////////////////////////////////////////////          
            rollSetpoint = 0; // Valor deseado (necesita convertirse a grados)
            //rollInput = map(unRollIn, 900, 2000, -30, 30); // Valor de entrada (necesita convertirse a grados)
            //double Rollgap = abs(rollSetpoint-rollInput); // Distancia hasta setpoint (Error)
            //if(Rollgap<5) {  // Estamos lejos del setpoint, usar parametros conservativos
                //rollPID.SetTunings(RollconsKp, RollconsKi, RollconsKd);
                if(rollOutput>=max_roll||max_roll==30)max_roll=rollOutput;
            //} else {
              // Estamos muy cerca del setpoint, usa parametros agresivos
              rollPID.SetTunings(RollaggKp, RollaggKi, RollaggKd);
            //}
            rollPID.Compute();

          Serial.println("///////////////////////////////////////////////////////////////////////////");
          Serial.print("PITCH INPUT: ");Serial.print(pitchInput);
            //Serial.print("->MAXPITCH: ");Serial.println(max_pitch);
            //Serial.print("->PITCH OUTPUT: ");Serial.println(pitchOutput);
          Serial.print("ROLL INPUT: ");Serial.print(rollInput);
            //Serial.print("->MAXPITCH: ");Serial.println(max_roll);
            //Serial.print("->ROLL OUTPUT: ");Serial.println(rollOutput);
    
    if(true) {
          //Serial.println("///////////////////////////////////////////////////////////////////////////");
          /*outputTR = unThrottleIn;
          outputTL = unThrottleIn;
          outputBL = unThrottleIn;
          outputBR = unThrottleIn;*/
        /////////////////////////////////ESTABILIZADOR/////////////////////////////////////
          if(mpuPitch > 0) {//enciendeizquierda          
            outputBR = unThrottleIn + rollOutput; outputTR = unThrottleIn - rollOutput;
            outputBL = unThrottleIn + rollOutput; outputTL = unThrottleIn - rollOutput;
          }
          else if (mpuPitch < 0){//enciendederecha
            outputBR = unThrottleIn - rollOutput ; outputTR = unThrottleIn + rollOutput ;
            outputBL = unThrottleIn - rollOutput ; outputTL = unThrottleIn + rollOutput ;
          }
          else {
            outputBR = unThrottleIn ; outputTR = unThrottleIn ;
            outputBL = unThrottleIn ; outputTL = unThrottleIn ;
          }
          // Inclinar Adelante (Grados negativos)
          if(mpuRoll < 0) {//enciendearriba
            outputBR = outputBR + pitchOutput ; outputTR = outputTR + pitchOutput ;
            outputBL = outputBL - pitchOutput; outputTL = outputTL - pitchOutput ;
          }
          else if(mpuRoll > 0) {//enciendeabajo
            outputBR = outputBR - pitchOutput; outputTR = outputTR - pitchOutput;
            outputBL = outputBL + pitchOutput ; outputTL = outputTL + pitchOutput ;
          }
          else{//enciendetodo
            outputBR = outputBR ; outputTR = outputTR ;
            outputBL = outputBL ; outputTL = outputTL ;
          }
          ////////////////////////VIRAR//////////////////////////
                unsigned long currentMillis = millis();      
                if(currentMillis - previousMillis > 3000) {
                        previousMillis = currentMillis;
                        if(derecha==90){
                          derecha=0; 
                        }
                        if(izquierda==90){
                          izquierda=0;  
                        }
                        if(arriba==90){
                          arriba=0;  
                        }
                        if(abajo==90){
                          abajo=0;  
                        }
                  }
 
    /////////////////////////////////////////////////////////
            /*outputBR = unThrottleIn ; outputTR = unThrottleIn ;
            outputBL = unThrottleIn ; outputTL = unThrottleIn ;*/
          if(derecha==90){
            outputTR+=derecha;
            outputTL+=derecha; 
          }
          if(izquierda==90){
            outputBR+=izquierda;
            outputBL+=izquierda; 
          }
          if(arriba==90){
            outputTR+=arriba;
            outputBR+=arriba;
          }
          if(abajo==90){
            outputTL+=abajo;
            outputBL+=abajo; 
            
          }
          /*if(mpuRoll > 0) {          
            outputBR = unThrottleIn + pitchOutput; outputTR = unThrottleIn - pitchOutput;
            outputBL = unThrottleIn + pitchOutput; outputTL = unThrottleIn - pitchOutput;
          }
          else if (mpuRoll < 0){
            outputBR = unThrottleIn - pitchOutput; outputTR = unThrottleIn + pitchOutput;
            outputBL = unThrottleIn - pitchOutput; outputTL = unThrottleIn + pitchOutput;
          }
          else{
             outputBR = unThrottleIn; outputTR = unThrottleIn;
             outputBL = unThrottleIn; outputTL = unThrottleIn;
          }
          // Inclinar Adelante (Grados negativos)
          if(mpuPitch < 0) {
            outputTR = unThrottleIn  + rollOutput; outputTL = unThrottleIn  - rollOutput;
            outputBR = unThrottleIn  + rollOutput; outputBL = unThrottleIn  - rollOutput;
          }
          else if(mpuPitch > 0) {
            outputTR = unThrottleIn  - rollOutput; outputTL = unThrottleIn  + rollOutput;
            outputBR = unThrottleIn  - rollOutput; outputBL = unThrottleIn  + rollOutput;
          }
          else{
            outputTR = unThrottleIn; outputTL = unThrottleIn;
            outputBR = unThrottleIn; outputBL = unThrottleIn;
          }/*

          /*outputTR = unThrottleIn - pitchOutput + rollOutput;
          outputBR = unThrottleIn + pitchOutput + rollOutput;
          outputBL = unThrottleIn + pitchOutput - rollOutput;
          outputTL = unThrottleIn - pitchOutput - rollOutput;
          //////////////////ESTABILIZADOR CON PID///////////////////////////////////
          outputTR = unThrottleIn - pitchOutput + rollOutput;
          outputBR = unThrottleIn + pitchOutput + rollOutput;
          outputBL = unThrottleIn + pitchOutput - rollOutput;
          outputTL = unThrottleIn - pitchOutput - rollOutput;
          /////////////////////////////////ESTABILIZADOR/////////////////////////////////////
          /*outputTR = unThrottleIn - pitchOutput + rollOutput;
          outputBR = unThrottleIn + pitchOutput + rollOutput;
          outputBL = unThrottleIn + pitchOutput - rollOutput;
          outputTL = unThrottleIn - pitchOutput - rollOutput;*/
    }
  
  initMotors(outputTL,outputTR,outputBR,outputBL);
  //LOOP CLIENT
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  //LOOP CLIENT
}









////////////////////////////////////FUNCIONES DE LOS MOTORES////////////////////////////////////
void virar(){
  
}
void arm() {
  servoMotorTL.writeMicroseconds(1000);
  servoMotorTR.writeMicroseconds(1000);
  servoMotorBL.writeMicroseconds(1000);
  servoMotorBR.writeMicroseconds(1000); 
}
void initMotors(int tl, int tr, int br, int bl) {

  /*Serial.print("TL: ");Serial.println(tl);
  Serial.print("TR: ");Serial.println(tr);
  Serial.print("BR: ");Serial.println(br);
  Serial.print("BL: ");Serial.println(bl);*/
 
  servoMotorTL.writeMicroseconds(tl);
  servoMotorTR.writeMicroseconds(tr);
  servoMotorBR.writeMicroseconds(br);
  servoMotorBL.writeMicroseconds(bl);
  
}
/////////////////////////////////////FUNCIONES DEL WIFI///////////////////////////////////////////
// /////SETUP WIFI//////////////
void setup_wifi() {
   delay(100);
  // We start by connecting to a WiFi network
    Serial.print("Connecting to ");
    Serial.println(ssid);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) 
    {
      delay(500);
      Serial.print(".");
    }
  randomSeed(micros());
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}
void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) 
  {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP8266Client-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    //if you MQTT broker has clientID,username and password
    //please change following line to    if (client.connect(clientId,userName,passWord))
    if (client.connect(clientId.c_str()))
    {
      Serial.println("connected");
     //once connected to MQTT broker, subscribe command if any
      client.subscribe("motor");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 6 seconds before retrying
      delay(6000);
    }
  }
} //end reconnect()

