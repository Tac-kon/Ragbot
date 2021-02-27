#include "main.h"
#include "mbed.h"

Serial device(D1,D0);

void Servo::torque (uint8_t ID, int data){
    unsigned char TxData[9];    // TransmitByteData [9byte]
    unsigned char CheckSum = 0; // CheckSum calculation
    
    TxData[0] = 0xFA;           // Header
    TxData[1] = 0xAF;           // Header
    TxData[2] = ID;             // ID
    TxData[3] = 0x00;           // Flags
    TxData[4] = 0x24;           // Address
    TxData[5] = 0x01;           // Length
    TxData[6] = 0x01;           // Count
    TxData[7] = data;           // Data 
    
    // CheckSum calculation
    for(int i=2; i<=7; i++){
        CheckSum = CheckSum ^ TxData[i];                // XOR from ID to Data
    }
    TxData[8] = CheckSum;       // Sum
    
    // Send Packet 
    for(int i=0; i<=8; i++){
        device.putc(TxData[i]);
    }
    //wait_us(0.1);               // Wait for transmission
}

void Servo::rotate (uint8_t ID, int data){

    unsigned char TxData[10];   // TransmitByteData [10byte]
    unsigned char CheckSum = 0; // CheckSum calculation
    data=10*data;
    
    TxData[0] = 0xFA;           // Header
    TxData[1] = 0xAF;           // Header
    TxData[2] = ID;             // ID
    TxData[3] = 0x00;           // Flags
    TxData[4] = 0x1E;           // Address
    TxData[5] = 0x02;           // Length
    TxData[6] = 0x01;           // Count
                                // Data
    TxData[7] = (unsigned char)0x00FF & data;           // Low byte
    TxData[8] = (unsigned char)0x00FF & (data >> 8);    // Hi  byte
    
    // CheckSum calculation
    for(int i=2; i<=8; i++){
        CheckSum = CheckSum ^ TxData[i];                // XOR from ID to Data
    }
    TxData[9] = CheckSum;       // Sum
    
    // Send Packet
    for(int i=0; i<=9; i++){
        device.putc(TxData[i]);
    }
    //wait_us(250);               // Wait for transmission
}

void Servo::rt_pkt(uint8_t ID, uint8_t Adr)
{
    uint8_t TxData[8];
    //uint8_t RxData[8];
    unsigned char TxCheckSum=0;
    //unsigned char RxCheckSum=0;
    //int Data=0;
    
    TxData[0] = 0xFA;//Hdr
    TxData[1] = 0xAF;//Hdr_
    TxData[2] = ID;//ID_
    TxData[3] = 0x0F;//Flg_
    TxData[4] = Adr;//Adr_
    TxData[5] = 0x02;//Len_
    TxData[6] = 0x00;
    
    for(int i = 2; i < 7; i++) {
        TxCheckSum = TxCheckSum ^ TxData[i]; // ID～DATAまでのXOR
    }
    TxData[7] = TxCheckSum;  
    for(int i = 0; i <= 7; i++) {
        device.putc(TxData[i]);
    }
}

double Servo::readangle(uint8_t id){
    uint8_t get_angle[10];//

    rt_pkt(id,0x2A);

    for(int i = 0; i <= 9; i++) {
        for(int j=0; !device.readable(); j++) { //
            if(j>1000) return -1;
            wait_us(0.1);
        }
        get_angle[i] = device.getc();
    }

    short angle = (short) get_angle[8] << 8 | get_angle[7];
    double anglef = (double) (-1)*angle/10.0;

    return anglef;
}


double Servo::readomega(uint8_t id){ 
    uint8_t get_angle[10];//

    rt_pkt(id,0x2E);

    for(int i = 0; i <= 9; i++) {
        for(int j=0; !device.readable(); j++) { //
            if(j>1000) return -1;
            wait_us(0.1);
        }
        get_angle[i] = device.getc();
    }

    short angle = (short) get_angle[8] << 8 | get_angle[7];
    double anglef = (double) (-1)*angle/10.0;

    return anglef;
}
