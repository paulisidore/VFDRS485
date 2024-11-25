/*
    VFDRS485.cpp - Library for VFD control over RS485 serial communication.
    Created by Paul Isidore A. NIAMIE, Nov 24, 2024.
    Released into the public domain.
    Ce module fonctionne avec le port Serial2 du PLCA6 ou compatible
*/
#ifndef VFDRS485_h
#define VFDRS485_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#endif

class VFDRS485 {
    
    public:
        VFDRS485(char[15] *vfd_name, int vfd_adr, int ctr485=A1, int rx_pin=15, HardwareSerial *_console = NULL);

        char Nom[15] ;
        int RS485_ID ;
        int Ctr485 ; //A1;
        // assign the Arduino pin that must be connected to RE-DE RS485 transceiver
        int TXEN ; 
        int RXPIN ;
        byte byteStop[];
        byte byteFreqGet[];
        int TEMP_MAX_REV ;
        int VFD_TIMEOUT ; //Send command timeout
        HardwareSerial console ; //Console de déboguage

        /**
         * @brief VFD ModBus Command H for receive Ouput Frequency in Hz (Hertz)
         */
        uint8_t CMD_OUTPUT_FREQ_H ;
        /**
         * @brief VFD ModBus Command L for receive Ouput Frequency in Hz (Hertz)
         */
        uint8_t CMD_OUTPUT_FREQ_L ;

        /**
         * @brief VFD ModBus Command H for receive Ouput current in Ampere
         */
        uint8_t CMD_OUTPUT_CURRENT_H ;
        /**
         * @brief VFD ModBus Command L for receive Ouput current in Ampere
         */
        uint8_t CMD_OUTPUT_CURRENT_L ;

        /**
         * @brief VFD ModBus Command H for receive Ouput power in kW
         */
        uint8_t CMD_OUTPUT_POWER_H ;
        /**
         * @brief VFD ModBus Command L for receive Ouput power in kW
         */
        uint8_t CMD_OUTPUT_POWER_L ;

        /**
         * @brief VFD ModBus Command H for receive Motor Temperature in C (Celcus)
         */
        uint8_t CMD_OUTPUT_MOTOR_TEMP_H ;
        /**
         * @brief VFD ModBus Command L for receive Motor Temperature in C (Celcus)
         */
        uint8_t CMD_OUTPUT_MOTOR_TEMP_L ;

        /**
         * @brief VFD ModBus Command H for receive Error Code
         */
        uint8_t CMD_ERROR_H ;
        /**
         * @brief VFD ModBus Command L for receive Error Code
         */
        uint8_t CMD_ERROR_L ;

        /**
         * @brief return VFD status
         * 
         */
        int IsRunning ; // 1 => Motor is Running ; 0 => Motor is Off

        /**
         * @brief Actual running Frequency (Hz) in Herzt
         * 
         */
        int OutputFreq ;

        /**
         * @brief Actual running output Current (A) in Ampere
         */
        int OutputCurrent ;

        /**
         * @brief Actual running output Power (kW) in Kilo Watt
         * 
         */
        int OuputPower ;

        /**
         * @brief Actual running motor Temperature (°C) in celcus
         * 
         */
        int OuputTemperature ;

        /**
         * @brief Return last VFD Error Code
         */
        int LAST_ERROR_CODE ;

        /**
         * @brief Send Message to VFD
         * 
         * @param msg 
         * @param CmdPH 
         * @param CmdPL 
         * @param ValPH 
         * @param ValPL 
         * @return int 
         */
        int VFDRS485::WriteToVFD(msg_t &msg, uint8_t CmdPH, uint8_t CmdPL, uint8_t ValPH, uint8_t ValPL);

        /**
         * @brief Read Message from VFD
         * 
         * @param msg 
         * @param CmdPH 
         * @param CmdPL 
         * @param ValPH 
         * @param ValPL 
         * @return int 
         */
        int VFDRS485::ReadFromVFD(msg_t &msg, uint8_t CmdPH, uint8_t CmdPL, uint8_t ValPH, uint8_t ValPL);

        /**
         * @brief Check Received Message from VFD
         * 
         */
        void VFDRS485::CheckReponseVFD();

        /**
         * @brief Check if the VFD is connected or not
         * 
         */
        bool VFDRS485::IsOnLine() ;

        /**
         * @brief Stop running VFD. Motor will stop
         */
        void VFDRS485::StopVFD();

        /**
         * @brief Start running VFD. Motor will run in clockwise mode
         * 
         */
        void VFDRS485::StartVFD() ;

        /**
         * @brief Start running VFD in reverse mode. Motor will run in anti-clockwise mode
         */
        void VFDRS485::ReverseVFD();

        /**
         * @brief Get actual running frequency on VFD
         * 
         */
        void VFDRS485::GetFreqVFD() ;

        /**
         * @brief Get actual output current
         */
        void VFDRS485::GetCurrentVFD() ;

        /**
         * @brief Get actual Motor Power in kW
         */
        void VFDRS485::GetMotorPowerVFD();

        /**
         * @brief Get actual running Motor Temperature in °C (Celcus)
         * 
         */
        void VFDRS485::GetMotorTempVFD() ;

        /**
         * @brief Get last error code from VFD 
         */
        void VFDRS485::GetErrorVFD();

        /**
         * @brief This sub must be run in Arduino main loop for check and update VFD state
         */
        void VFDRS485::CheckEvent() ;

        //Events CallBack
        using onReceiveEvent = void (*)(msg_t msg); //type aliasing
        using onTimeOutEvent = void (*)(char[] vfd_nom, int vfd_adr);
        using onDisConnectEvent = void (*)(char[] vfd_nom, int vfd_adr);
        
        /**
         * @brief set on receive data callback function
         */
        void VFDRS485::setEventOnReceive(onReceiveEvent onDataReceiveFuncPtr);

        /**
         * @brief set on time out callback function 
         * @param onTimeOutFuncPtr 
         */
        void VFDRS485::setEventOnTimeOut(onTimeOutEvent onTimeOutFuncPtr);

        /**
         * @brief set on VFD Disconnect callback function
         * @param onDisConnectFuncPtr 
         */
        void VFDRS485::setEventOnDisConnect(onDisConnectEvent onDisConnectFuncPtr);

    private:
        /**
     * @brief VARIABLE PRIVEE ET CONSTANTE
     * 
     */
        int RS485Transmit ;
        int RS485Receive;

        const int MOTEUR_ON;
        const int MOTEUR_OFF;

        const int ANALOG_ASC;
        const int ANALOG_DESC;

        byte byteSend;
        int runDejaSend;
        int monitorFreqSend;
        int waitReponse;
        int used;
        int p;

        int freqDem ;
        int tmp_freq ;

        const byte numChars ;
        char Reponse[] ;
        int ReponseEnCour;

        int analog_encour;
        int analog_ledmin ;
        int analog_step  ;
        int analog_max ;
        int analog_sens ;

        int ETAT_MOTEUR ;

        int temp_func ;
        int SENS_ROTATION_AV ;
        int SENS_ROTATION_AR ;
        int SENS_ACT ;

        typedef struct {
            uint8_t DEST_ID;          /*!< Slave address between 1 and 247. 0 means broadcast */
            uint8_t OP_CODE;          /*!< Function code: 1, 2, 3, 4, 5, 6, 15 or 16 */
            uint8_t FUNC_ACT_H;         /*!< Commde Poid Fort */
            uint8_t FUNC_ACT_L;         /*!< Commde Poid Faible */
            uint16_t REG_ADR_H;    /*!< Address of the first register to access at slave/s */
            uint16_t REG_ADR_L;    /*!< Address of the first register to access at slave/s */
            uint16_t REPONSE;
            uint16_t CRC_ENVOIE;
            uint16_t CRC_RECUS;  
        }
        msg_t;

        typedef struct {
        uint8_t ID;          /*!< Slave address between 1 and 247. 0 means broadcast */
        uint8_t PRESENT ;   //SI le vfd est prsent il a deja repondu a au moin un appel
        uint8_t LAST_FUNC_H;         /*!< Commde Poid Fort */
        uint8_t LAST_FUNC_L;         /*!< Commde Poid Faible */
        uint16_t LAST_REP_H;    /*!< Address of the first register to access at slave/s */
        uint16_t LAST_REP_L;    /*!< Address of the first register to access at slave/s */ 
        int TEMP_START ;  //Stock le temps de depart de la commande
        int EN_QUESTION ;
        }
        vfd_t;

        msg_t message ;
        
        byte byteReceive[] ;
        const char hexCharArray[];

        vfd_t vfd ;

        /**
         * @brief This event is raised on received data.
         * Param: msg_t 
         */
        onReceiveEvent onReceive ;

        /**
         * @brief This event is raised on communication timeout
         */
        onTimeOutEvent onTimeOut ;

        /**
         * @brief This event is raised on VFD Disconnection
         * 
         */
        onDisConnectEvent onDisconnect ;

        unsigned int VFDRS485::crc_chk_value(unsigned char *data_value,unsigned char length) ;

        /**
         * @brief Check if VFD is not anser in time
         * 
         */
        void VFDRS485::CheckNoReponseVFD() ;

    // --------------------------------------------------------------------------------------------------------

}
#endif