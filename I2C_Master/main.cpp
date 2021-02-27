#include "mbed.h"
#include "main.h"

//パラメータ設定
const int N_trans=8;
const int N_msg=5;
const int N_char=4;
const double dt=0.02;

Ticker timer1;
Ticker timer2;


//I2C設定
I2C master(D4,D5); //SDA,SCL
const uint8_t addr[N_trans]={0xA0,0xA2,0xA4,0xA6,0xA8,0xB0,0xB2,0xB4};
const double E[N_msg]={1000.0,1000.0,100.0,100.0,100.0};
char msg_unit[N_char];
char msg_rcv[N_char*N_msg];      // msg to transmit
char msg_snd[N_char*N_msg];   
int data_rcv[N_msg*N_trans];
int data_snd[N_msg*N_trans];

Serial pc(USBTX,USBRX); 

void encode_I2C(double code, double Magni)
{
    for(int i=0;i<N_char;i++) msg_unit[i]=0;
    int tmp_code = (int) (code * Magni);
    snprintf(msg_unit,sizeof(msg_unit),"%d",tmp_code);
}

// decode
//引数: message unit(デコードされる), 等分する値  戻り値: デコードされた値
double decode_I2C(char *decode, double deMagni)
{
    int tmp = 0.0;
    double tmp1 = 0.0;
    tmp=atoi(decode);
    tmp1 = (double) (tmp/deMagni);
    return tmp1;
}

void init(void){
    
    pc.baud(115200);
    //master.frequency(400000);
    //master.write(addr[j],msg_trans,4);
    pc.printf("Project_Gokai_start.\n");
    wait(5.5);
}

void i2c_rcv()
{
    int i,j,k;
    for(i=0;i<N_char*N_msg;i++)
    {
        msg_snd[i]=0;
        msg_rcv[i]=0;
    }
    
    for(j=0;j<N_trans;j++)
    {
        master.read(addr[j],msg_rcv, N_char*N_msg);
        for(i=0;i<N_msg;i++)
        {
            for(k=0;k<N_char;k++){
                msg_unit[k] = msg_rcv[N_char*i+k];
            //msg_unit[1] = msg_rcv[4*i+1];
            //msg_unit[2] = msg_rcv[4*i+2];
            //msg_unit[3] = msg_rcv[4*i+3];
            }
            data_rcv[N_msg*j+i] = decode_I2C(msg_unit,E[i]);
            pc.printf("%d:%lf ",N_msg*j+i,data_rcv[N_msg*j+i]);
        }
    }
    pc.printf("\n"); 
}
    //wait(dt);
   
void i2c_snd(){ 
    int i,j,k;
    for(i=0;i<N_msg-1;i++) data_snd[i]=0;
    data_snd[N_msg*N_trans-1]=0;
    
    for(i=1;i<N_trans;i++){
        for(j=0;j<N_msg-1;j++){
            data_snd[N_msg*i+j]=data_rcv[N_msg*(i-1)+j];
        }
        data_snd[N_msg*(i+1)-1]=data_rcv[N_msg*(i+2)-1];
    }
    
    for(j=0;j<N_trans;j++)
    {
        for(i=0;i<N_msg;i++)
        {
            encode_I2C(data_snd[N_msg*j+i], E[i]);
            //master.write(addr[j],msg_trans,4*N_msg);
            for(k=0;k<N_char;k++){
                msg_snd[N_char*i+k]=msg_unit[k];
            }
            //msg_snd[4*i+1]=msg_unit[1];
            //msg_snd[4*i+2]=msg_unit[2];
            //msg_snd[4*i+3]=msg_unit[3];
        }
        master.write(addr[j],msg_snd,N_char*N_msg);
    }
    //wait(dt);
}

int main() {
    init();
    timer1.attach(&rcv_,dt);
    timer2.attach(&i2c_snd,dt);
    while(1){}
}
