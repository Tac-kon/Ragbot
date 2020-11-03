#include "mbed.h"
#include "main.h"

const double PI=3.14159;

//パラメータ設定
const int seg=2;
const double dt=0.02;
const double omega=1.0;
const double sigma_a = 1.0;
const double sigma_b = 0.5;
const double sigma_f = 1.0;
const double sigma_h = 1.0;
const double sigma_r = 10.0;
//const double psi = (-1)*PI/3;
const double theta_max=20.0;
const double theta_min=-20.0;
const int N_msg=5;
const int N_char=4;
const double k=0.001;
const double c=0.0;

const uint8_t addr=0xA0;

//fsrピンの設定
AnalogIn FsrRF(A3);
AnalogIn FsrLF(A0);
AnalogIn FsrRH(A2);
AnalogIn FsrLH(A1);
//LEDピンの設定
DigitalOut LedRF(D5);
DigitalOut LedLF(D6);
DigitalOut LedRH(D7);
DigitalOut LedLH(D8);
//I2C通信の設定
I2CSlave slave(D4,D5); // SDA,SCL
//const uint8_t addr=0xA0;
const double E[N_msg]={1000.0,1000.0,100.0,100.0,100.0};
char msg_unit[N_char];
char msg_snd[N_char*N_msg];      // msg to transmit
char msg_rcv[N_char*N_msg]; 
double data_snd[N_msg];
double data_rcv[N_msg];
//PC
Serial pc(USBTX,USBRX); 
//サーボモータの設定
Serial motor(D1,D0); // tx=D1, rx=D2
Servo servo;
Joint r[seg];
Joint l[seg];
Joint body[seg];
Joint rf;
Joint lf;
Joint h_body;

void encode_I2C(double code, double Magni)
{
    for(int i=0;i<N_char;i++) msg_unit[i]=0;
    int tmp_code = (int) (code * Magni);
    snprintf(msg_unit,sizeof(msg_unit),"%d",tmp_code);
}

double decode_I2C(char *decode, double deMagni)
{
    int tmp = 0.0;
    double tmp1 = 0.0;
    tmp=atoi(decode);
    tmp1 = (double) tmp/deMagni;
    return tmp1;
}

int fa(double phi0, double phi1)
{
    int flag;
    if(cos(phi0)>0&&phi1>=PI/2&&phi1<=PI) flag=1;
    else flag=0;
    return flag;
}

void init(void){
    int i;
    
    motor.baud(115200);        // baud Rate = 115.2kbps [Futaba default]
    pc.baud(115200);
    
    //slave.frequency(400000);
    slave.address(addr);
    
    //サーボidの宣言
    body[0].id = 0x01; r[0].id=0x02; l[0].id=0x03; 
    body[1].id = 0x04; r[1].id=0x05; l[1].id=0x06; 
    
    //サーボの初期化
    for(i=0;i<seg;i++){
        r[i].phi=(rand()%10+1)/100; l[i].phi=(rand()%10+1)/100; 
        body[i].tau=0.0; //body[i].omg=0.0;
        r[i].gp=10.0; l[i].gp=-10.0; body[i].gp=0.0;

        servo.torque(r[i].id,0x01); 
        servo.torque(l[i].id,0x01);
        servo.torque(body[i].id,0x01);
        servo.rotate(r[i].id,(int)(r[i].gp));
        servo.rotate(l[i].id,(int)(l[i].gp));
        servo.rotate(body[i].id,(int)(body[i].gp));
    }
    wait(2.0);
    
    //fsrのオフセットを設定(20回の平均をとる)
    for(i=0;i<20;i++){
      r[0].N_off+=FsrRF.read();
      l[0].N_off+=FsrLF.read();
      r[1].N_off+=FsrRH.read();
      l[1].N_off+=FsrLH.read();
      wait(dt);
    }
    for(i=0;i<seg;i++){
    r[i].N_off =r[i].N_off/20;
    l[i].N_off =l[i].N_off/20;
    }
    wait(3.0);
}

void measure_data()
{   
    int i;
    for(i=0;i<seg;i++)
    {
        r[i].N = fabs(FsrRF.read()-r[i].N_off);
        l[i].N = fabs(FsrLF.read()-l[i].N_off);
        if(r[i].N < 0.01) r[i].N=0.0;
        if(l[i].N < 0.01) l[i].N=0.0;
        body[i].pp=servo.readangle(body[i].id);
        body[i].v_rot=servo.readomega(body[i].id);
    }
}

void calc(void)
{   
    body[0].tau = sigma_h*(l[0].N-r[0].N)-k*body[0].pp-c*body[0].v_rot;
    body[1].tau = sigma_f*(r[0].N-l[0].N)+sigma_h*(l[1].N-r[1].N)-k*body[1].pp-c*body[1].v_rot;
    r[0].phi += dt*(omega+sigma_b*(body[0].tau-body[1].tau)*cos(r[0].phi));
    r[1].phi += dt*(omega+sigma_a*(fa(r[0].phi,r[1].phi)*cos(r[0].phi)*cos(r[1].phi))+sigma_b*(body[1].tau-h_body.tau)*cos(r[1].phi));
    l[0].phi += dt*(omega-sigma_b*(body[0].tau-body[1].tau)*cos(l[0].phi));;
    l[1].phi += dt*(omega+sigma_a*(fa(l[0].phi,l[1].phi)*cos(l[0].phi)*cos(l[1].phi))-sigma_b*(body[1].tau-h_body.tau)*cos(l[1].phi));
   
    for(int i=0;i<seg;i++)
    {
        if(r[i].phi>2*PI) r[i].phi=r[i].phi-2*PI;
        if(l[i].phi>2*PI) l[i].phi=l[i].phi-2*PI;
    }
    pc.printf("\n");
    for(int i=0;i<seg;i++){
        r[i].gp = 10*sin(r[i].phi);
        l[i].gp = -10*sin(l[i].phi);
        
        body[i].gp = (-1)*(body[i].pp + sigma_r*body[i].tau);
        if(body[i].gp > theta_max){
             body[i].gp = (-1)*theta_max; 
             pc.printf("theta%d_max\n",i);
            }
        else if(body[i].gp < theta_min) {
            body[i].gp = (-1)*theta_min;
            pc.printf("theta%d_min\n",i);
            }
        
        servo.rotate(r[i].id, r[i].gp);
        servo.rotate(l[i].id, l[i].gp);
        servo.rotate(body[i].id, body[i].gp);
    }
}

void i2c_comm()
{
    int i,j;
    int receive;
    
    receive=slave.receive();
    switch(receive){
        case I2CSlave::ReadAddressed:
            measure_data();
            for(i=0;i<N_char*N_msg;i++)
            {
                msg_snd[i]=0;
                msg_rcv[i]=0;
            }
            data_snd[0]=body[0].tau;
            data_snd[1]=r[1].N;
            data_snd[2]=l[1].N;
            data_snd[3]=r[1].phi;
            data_snd[4]=l[1].phi;
            
            for(i=0;i<N_msg;i++)
            {
                encode_I2C(data_snd[i], E[i]);
                for(j=0;j<N_char;j++)
                {
                    msg_snd[N_char*i+j]=msg_unit[j];
                }
            }
            slave.write(msg_snd,N_char*N_msg);
            break;
         case I2CSlave::WriteAddressed: 
            measure_data();
            slave.read(msg_rcv, N_char*N_msg); 
            for(i=0;i<N_msg;i++)
            {
                for(j=0;j<N_char;j++)
                {
                msg_unit[j] = msg_rcv[N_char*i+j];
                }
                data_rcv[i] = decode_I2C(msg_unit,E[i]);
            }
            
            rf.N=msg_rcv[0];
            lf.N=msg_rcv[1];
            rf.phi=msg_rcv[2];
            lf.phi=msg_rcv[3];
            h_body.tau=msg_rcv[4];
            calc();
            break;
        default:
            break;
        }
}

int main() {
    init();
    while(1){
        i2c_comm();
    }
}
