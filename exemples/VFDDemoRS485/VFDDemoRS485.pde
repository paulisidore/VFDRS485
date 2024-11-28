/*
    RS485 Demo on RS485 serial communication.
    Created by Paul Isidore A. NIAMIE, Nov 24, 2024.
    Released into the public domain.
    Ce module fonctionne avec le port Serial2 du PLCA6 ou compatible
*/
#include <PLCA6TRANS.h>
#include <VFDRS485.h>

#define RXPIN 15
const int Ctr485=A1;
PLCA6TRANS MyPLC;
VFDRS485 vfd("Moteur_A", 1, Ctr485, RXPIN, 1) ;

VFDRS485::msg_t MessageRecus ;

void setup(){
  Serial.begin(9600);
  Serial.println("VFD on RS485 Test") ;

  vfd.setEventOnReceive(onReceive);

  MyPLC.ErrorOFF();
  MyPLC.RunON();
  delay(1000);
  MyPLC.RunOFF();
  Serial.println("PLCA6TRANS is Ready.") ;

}

void loop(){
    delay(500);
    vfd.CheckEvent();
    if (MyPLC.InputX(MyPLC.X1)){
        MyPLC.RunON();
        MyPLC.RelayON(MyPLC.Y1);
        if (vfd.IsRunning == 0){
            vfd.StartVFD();
        }
    }else{
        if (vfd.IsRunning == 1){
            vfd.StopVFD();
        }
        MyPLC.RunOFF();
        MyPLC.RelayOFF(MyPLC.Y1);
    }
    if(vfd.IsRunning){
        Serial.print("Motor Freq: ");
        Serial.print(vfd.OutputFreq );
        Serial.println("Hz");

        Serial.print("Motor Current: ");
        Serial.print(vfd.OutputCurrent );
        Serial.println("A");
    }else{
        if(vfd.LAST_ERROR_CODE){
            Serial.print("ERROR: ");
            Serial.println(vfd.LAST_ERROR_CODE );
            vfd.LAST_ERROR_CODE=0;
        }
    }
}

void onReceive(VFDRS485::msg_t msg){
    Serial.println("Message recus dans Console");
}