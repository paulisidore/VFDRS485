/*
    VFDRS485.cpp - Library for VFD control over RS485 serial communication.
    Created by Paul Isidore A. NIAMIE, Nov 24, 2024.
    Released into the public domain.
    Ce module fonctionne avec le port Serial2 du PLCA6 ou compatible
*/
#include "Arduino.h"
#include "VFDRS485.h"

VFDRS485::VFDRS485(){

}
VFDRS485::VFDRS485(const char* vfd_name, int vfd_adr, int ctr485=A1, int rx_pin=15, int debug = 0){

    /**
     * @brief AFFECTATION INITIALE VARIABLE PUBLIQUE
     * 
     */
         //Allocate enough memory for copying the input string.
        Nom = (char*) malloc( strlen(vfd_name) * sizeof(char) + 1 );
        //Copy the input string to the class's field.
        strcpy( Nom, vfd_name );
        RS485_ID = vfd_adr ; //ID Adresse du VFD à controler
        Ctr485 = ctr485 ; //A1 ;
        // assign the Arduino pin that must be connected to RE-DE RS485 transceiver
        TXEN = Ctr485 ; 
        RXPIN = rx_pin ;
        VFD_TIMEOUT=3000 ; // 3sec de timeout si le Vfd ne reponds pas
        TEMP_MAX_REV = 20000 ; //20 secondes avant Revers
        ACTIVE_CONSOLE  = debug;

        //console = _console ; //Console de déboguage
        IsRunning = 0 ;
        LAST_ERROR_CODE = 0 ;

        pinMode(RXPIN, INPUT);
        pinMode(TXEN, OUTPUT);

        CMD_OUTPUT_FREQ_H = 0x70 ;
        CMD_OUTPUT_FREQ_L = 0x00 ;

        CMD_OUTPUT_CURRENT_H = 0x70 ;
        CMD_OUTPUT_CURRENT_L = 0x04 ;

        CMD_OUTPUT_POWER_H = 0x70 ;
        CMD_OUTPUT_POWER_L = 0x05 ;

        CMD_OUTPUT_MOTOR_TEMP_H = 0x70 ;
        CMD_OUTPUT_MOTOR_TEMP_L = 0x22 ;

        CMD_ERROR_H = 0x70 ;
        CMD_ERROR_L = 0x2D ;


    // -------------------------------------------------------------------------------------------------------------

    /**
     * @brief VARIABLE PRIVEE ET CONSTANTE
     * 
     */
        RS485Transmit  =  HIGH ;
        RS485Receive  =   LOW ;

        MOTEUR_ON =1;
        MOTEUR_OFF =0;

        ANALOG_ASC = 1;
        ANALOG_DESC = 0;

        p=1;

        freqDem=10000 ;
        tmp_freq=0;

        ReponseEnCour=0;

        analog_encour = 0;
        analog_ledmin = 10 ;
        analog_step = 20 ;
        analog_max = 255;
        analog_sens = ANALOG_ASC;

        ETAT_MOTEUR = MOTEUR_OFF ;

        temp_func =0 ;
        SENS_ROTATION_AV =1 ;
        SENS_ROTATION_AR =0 ;
        SENS_ACT = SENS_ROTATION_AV ;

        msg_t message ;
        
        //byteReceive[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
        for (int p=0;p<7;p++){
            byteReceive[p] = 0xFF;
        }
        
    // --------------------------------------------------------------------------------------------------------

    vfd.ID=RS485_ID;
    vfd.PRESENT = 0;
    vfd.LAST_FUNC_H =0xFF;
    vfd.LAST_FUNC_L=0xFF;
    vfd.LAST_REP_H=0xFF;
    vfd.LAST_REP_L=0xFF;
    vfd.TEMP_START= 0;
    vfd.EN_QUESTION=0 ;

    runDejaSend=0;
    WriteToSerial("VFD ",0);
    WriteToSerial(Nom, 0);
    WriteToSerial( ": OK",1);
}

VFDRS485::~VFDRS485(void){
    free(Nom);
}

int VFDRS485::SendMsgInRS485(uint8_t cmd_rw, msg_t &msg){
    byte byteCheck[] = {msg.DEST_ID, cmd_rw, msg.FUNC_ACT_H, msg.FUNC_ACT_L, msg.REG_ADR_H, msg.REG_ADR_L};
    unsigned mycrc = crc_chk_value(byteCheck,sizeof(byteCheck)) ; 
    uint8_t MSB=highByte(mycrc);
    uint8_t LSB=lowByte(mycrc);
     
    byte byteToSend[] = {msg.DEST_ID, cmd_rw, msg.FUNC_ACT_H, msg.FUNC_ACT_L, msg.REG_ADR_H, msg.REG_ADR_L, LSB, MSB}; // Correcte
      
    digitalWrite(Ctr485, RS485Transmit);  //On met l'etat en Haut pour transmettre
    byteSend=Serial2.write(byteToSend, sizeof(byteToSend));
    Serial2.flush();
    delay(4); //On attends pour3.5 caractère avant le prochain envoie
    digitalWrite(Ctr485, RS485Receive);  
    return 1;
}

int VFDRS485::WriteToVFD(msg_t &msg, uint8_t CmdPH, uint8_t CmdPL, uint8_t ValPH, uint8_t ValPL){
    byte byteCheck[] = {vfd.ID, 0x06, CmdPH, CmdPL, ValPH, ValPL};
    unsigned mycrc = crc_chk_value(byteCheck,sizeof(byteCheck)) ; 
    uint8_t MSB=highByte(mycrc);
    uint8_t LSB=lowByte(mycrc);
    msg.DEST_ID=vfd.ID ;
    msg.OP_CODE=0x06 ;
    msg.FUNC_ACT_H = CmdPH;
    msg.FUNC_ACT_L = CmdPL;
    msg.REG_ADR_H = ValPH ;
    msg.REG_ADR_L = ValPL ;
    msg.CRC_ENVOIE = mycrc ;
    msg.CRC_RECUS = 0x0000 ;
    msg.REPONSE = 0xFFFF ;  
    byte byteToSend[] = {vfd.ID, 0x06, CmdPH, CmdPL, ValPH, ValPL, LSB, MSB}; // Correcte
    vfd.LAST_FUNC_H =msg.FUNC_ACT_H;
    vfd.LAST_FUNC_L=msg.FUNC_ACT_L;
    vfd.LAST_REP_H=0xFF;
    vfd.LAST_REP_L=0xFF;
    vfd.TEMP_START= millis();
    vfd.EN_QUESTION = 1;
  
    digitalWrite(Ctr485, RS485Transmit);  //On met l'etat en Haut pour transmettre
    byteSend=Serial2.write(byteToSend, sizeof(byteToSend));
    Serial2.flush();
    delay(4); //On attends pour3.5 caractère avant le prochain envoie
    digitalWrite(Ctr485, RS485Receive);  
    
    return 1;
}

int VFDRS485::ReadFromVFD(msg_t &msg, uint8_t CmdPH, uint8_t CmdPL, uint8_t ValPH, uint8_t ValPL){
    byte byteCheck[] = {vfd.ID, 0x03, CmdPH, CmdPL, ValPH, ValPL};
    unsigned mycrc = crc_chk_value(byteCheck,sizeof(byteCheck)) ; 
    uint8_t MSB=highByte(mycrc);
    uint8_t LSB=lowByte(mycrc);
    msg.DEST_ID=vfd.ID ;
    msg.OP_CODE=0x06 ;
    msg.FUNC_ACT_H = CmdPH;
    msg.FUNC_ACT_L = CmdPL;
    msg.REG_ADR_H = ValPH ;
    msg.REG_ADR_L = ValPL ;
    msg.CRC_ENVOIE = mycrc ;
    msg.CRC_RECUS = 0x0000 ;
    msg.REPONSE = 0xFFFF ;  
    byte byteToSend[] = {vfd.ID, 0x03, CmdPH, CmdPL, ValPH, ValPL, LSB, MSB}; // Correcte
    vfd.LAST_FUNC_H =msg.FUNC_ACT_H;
    vfd.LAST_FUNC_L=msg.FUNC_ACT_L;
    vfd.LAST_REP_H=0xFF;
    vfd.LAST_REP_L=0xFF;
    vfd.TEMP_START= millis();
    vfd.EN_QUESTION = 1;
  
    digitalWrite(Ctr485, RS485Transmit);  //On met l'etat en Haut pour transmettre
    byteSend=Serial2.write(byteToSend, sizeof(byteToSend));
    Serial2.flush();
    delay(4); //On attends pour3.5 caractère avant le prochain envoie
    digitalWrite(Ctr485, RS485Receive);  
    
    return 1;
}

void VFDRS485::CheckReponseVFD(){
  if (Serial2.available() > 0) {
      ReponseEnCour=1;
      int nbByte=8;
      int i=0;
      
      for (int p=0;p<nbByte;p++){
        if (byteReceive[p] == 0xFF){
          if (Serial2.available()){
            uint16_t v = Serial2.read();
            byteReceive[p]= v ;
            //break;
          }else{
            break;
          }
        }
      }
      int bRec=0;
      for (int p=0;p<nbByte;p++){
        if (byteReceive[p] == 0xFF){
          break;
        }
        bRec++ ;
      }
      msg_t last_reponse ;
      
      if (bRec == nbByte){
        //Une reponse entièere recus
        last_reponse.DEST_ID = byteReceive[0] ;
        last_reponse.OP_CODE = byteReceive[1] ;
        last_reponse.FUNC_ACT_H = byteReceive[2] ;
        last_reponse.FUNC_ACT_L = byteReceive[3] ;
        last_reponse.REG_ADR_H = byteReceive[4] ;
        last_reponse.REG_ADR_L = byteReceive[5] ;
        last_reponse.REPONSE = byteReceive[6] ;
        last_reponse.CRC_ENVOIE = byteReceive[7] ;
        last_reponse.CRC_RECUS = byteReceive[8] ;

        WriteToSerial("<=VFD_REPONSE ");
        WriteToSerial(last_reponse.DEST_ID);
        WriteToSerial(": ");
        WriteToSerial(last_reponse.REPONSE);

        if (last_reponse.DEST_ID == vfd.ID){
            //Le VFD à repondu a une demande d'ecriture
            if (vfd.PRESENT == 0){
                vfd.PRESENT = 1;
                vfd.LAST_FUNC_H =last_reponse.FUNC_ACT_H;
                vfd.LAST_FUNC_L=last_reponse.FUNC_ACT_L;
                vfd.LAST_REP_H=last_reponse.REG_ADR_H;
                vfd.LAST_REP_L=last_reponse.REG_ADR_L;
                vfd.TEMP_START= 0;
                vfd.EN_QUESTION = 0;
                WriteToSerial("VFD ",0);
                WriteToSerial(Nom,0);
                WriteToSerial(" PRESENT",1);
            }
            if(last_reponse.FUNC_ACT_H == CMD_OUTPUT_FREQ_H && last_reponse.FUNC_ACT_L == CMD_OUTPUT_FREQ_L){
                //Reponse de la demande de fréquence
                OutputFreq = strtol(last_reponse.REPONSE, NULL, 16);
                if(last_reponse.REPONSE > 0x01){
                    IsRunning=1;
                }else{
                    IsRunning=0;
                }
            }
            if(last_reponse.FUNC_ACT_H == CMD_OUTPUT_CURRENT_H && last_reponse.FUNC_ACT_L == CMD_OUTPUT_CURRENT_L){
                OutputCurrent = strtol(last_reponse.REPONSE, NULL, 16);
            }
            if(last_reponse.FUNC_ACT_H == CMD_OUTPUT_POWER_H && last_reponse.FUNC_ACT_L == CMD_OUTPUT_POWER_L){
                OuputPower = strtol(last_reponse.REPONSE, NULL, 16);
            }
            if(last_reponse.FUNC_ACT_H == CMD_OUTPUT_MOTOR_TEMP_H && last_reponse.FUNC_ACT_L == CMD_OUTPUT_MOTOR_TEMP_L){
                OuputTemperature = strtol(last_reponse.REPONSE, NULL, 16);
            }
            if(last_reponse.FUNC_ACT_H == CMD_ERROR_H && last_reponse.FUNC_ACT_L == CMD_ERROR_L){
                LAST_ERROR_CODE = strtol(last_reponse.REPONSE, NULL, 16);
                if(LAST_ERROR_CODE>0){
                    WriteToSerial("VFD ",0);
                    WriteToSerial(Nom,0);
                    WriteToSerial(" ERROR: ",0);
                    WriteToSerial(LAST_ERROR_CODE,1);
                }
            }

            if(onReceive != NULL){
                onReceive(last_reponse);
            }                   
        }
        for (int p=0;p<nbByte;p++){
            byteReceive[p] = 0xFF;
        }
        
      }
  }else{
      CheckNoReponseVFD();
  }
}

bool VFDRS485::IsOnLine(){
     if (vfd.PRESENT == 1){
        return true;
     }
     return false;
}

void VFDRS485::CheckNoReponseVFD(){
    if (vfd.TEMP_START > 0 &&  vfd.EN_QUESTION == 1){
        int duree = millis() - vfd.TEMP_START;
        if (duree > VFD_TIMEOUT){
            //Aucune réponse du VFD
            vfd.TEMP_START =0;
            if(vfd.PRESENT > 0){
                //Déconnecté
                if(onDisconnect != NULL){
                    onDisconnect(Nom, vfd.ID);
                }
            }
            vfd.PRESENT = 0;
            vfd.EN_QUESTION = 0;
            if(onTimeOut != NULL){
                onTimeOut(Nom,vfd.ID);
            }
        }
    }
}

void VFDRS485::WriteToSerial(const char* msg, int IsLast = 1){
    if(ACTIVE_CONSOLE ){
        if(IsLast){
            Serial.println(msg) ;
        }else{
            Serial.print(msg) ;
        }
        
    }    
}

void VFDRS485::StopVFD(){
    uint8_t CmdH = 0x20 ;
    uint8_t CmdL = 0x00 ;
    uint8_t ValH = 0x00 ;
    uint8_t ValL = 0x05 ;
    VFDRS485::WriteToVFD(message, CmdH, CmdL, ValH, ValL);
    runDejaSend=0;
    ETAT_MOTEUR = MOTEUR_OFF ;
    delay(500);
    CheckReponseVFD();
}

void VFDRS485::StartVFD(){
    WriteToVFD(message, 0x20, 0x00, 0x00, 0x01);
    runDejaSend=1;    
    ETAT_MOTEUR = MOTEUR_ON ;
    temp_func = millis();
    SENS_ACT = SENS_ROTATION_AV ;
    CheckReponseVFD();
}

void VFDRS485::ReverseVFD(){
    WriteToVFD(message, 0x20, 0x00, 0x00, 0x02);
    runDejaSend=1;    
    ETAT_MOTEUR = MOTEUR_ON ;
    temp_func = millis();
    SENS_ACT = SENS_ROTATION_AR;
    CheckReponseVFD();
}

void VFDRS485::GetFreqVFD(){
    ReadFromVFD(message, CMD_OUTPUT_FREQ_H, CMD_OUTPUT_FREQ_L, 0x00, 0x02);
    CheckReponseVFD();
}

void VFDRS485::GetCurrentVFD(){
    ReadFromVFD(message, CMD_OUTPUT_CURRENT_H, CMD_OUTPUT_CURRENT_L, 0x00, 0x02);
    CheckReponseVFD();
}

void VFDRS485::GetMotorPowerVFD(){
    ReadFromVFD(message, CMD_OUTPUT_POWER_H, CMD_OUTPUT_POWER_L, 0x00, 0x02);
    CheckReponseVFD();
}

void VFDRS485::GetMotorTempVFD(){
    ReadFromVFD(message, CMD_OUTPUT_MOTOR_TEMP_H, CMD_OUTPUT_MOTOR_TEMP_L, 0x00, 0x02);
    CheckReponseVFD();
}

void VFDRS485::GetErrorVFD(){
    ReadFromVFD(message, CMD_ERROR_H, CMD_ERROR_L, 0x00, 0x02);
    CheckReponseVFD();
}

void VFDRS485::setEventOnReceive(onReceiveEvent onDataReceiveFuncPtr){
    onReceive = onDataReceiveFuncPtr ;
}

void VFDRS485::setEventOnDisConnect(onDisConnectEvent onDisConnectFuncPtr){
    onDisconnect = onDisConnectFuncPtr ;
}

void VFDRS485::CheckEvent(){
    GetFreqVFD();
    GetCurrentVFD();
    GetMotorPowerVFD();
    GetMotorTempVFD();
    GetErrorVFD();
}

unsigned int VFDRS485::crc_chk_value(unsigned char *data_value,unsigned char length){
  unsigned int crc_value=0xFFFF; 
  int i;
  while(length--){
    crc_value^=*data_value++; 
    for(i=0;i<8;i++){
      if(crc_value&0x0001){  
        crc_value=(crc_value>>1)^0xa001;
      }else{  
        crc_value=crc_value>>1;
      }
    }
  }
  return(crc_value);
}