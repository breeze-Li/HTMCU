//2021��11��29��:�����������

#include "BA45F5250.h"

#define  EN1 _pa7
#define  EN2 _pa3

#define  OpenLED_R    _isgenc |= 0b10000010;		///ISINK1 ���Ź����ʹ��,50mA
#define  OpenLED_G    _isgenc |= 0b10000001;		///ISINK0 ���Ź����ʹ��,50mA
#define  CloseLED_R   _isgenc &= 0b10000001;		///ISINK1 ���Ź��������
#define  CloseLED_G   _isgenc &= 0b10000010;		///ISINK0 ���Ź��������

volatile bit  Tb0_flg;
volatile bit  Tb1_flg;
volatile bit  transfer_complete_flag;		//���մ�����ɱ�־
volatile bit  RxFlag;       				//�����������ݱ�־

volatile unsigned char ComPC;		        //��������ָ��
volatile unsigned char ComCah[24];          //���ͻ�����
volatile unsigned char ComRPC;		        //��������ָ��
volatile unsigned char ComRCah[24];         //���ջ���
volatile unsigned char pointer_length;		//ָ�볤��(��¼���ܵ��ֽڳ���)

void Stm0AInit(void);
void UARTInit(void);
void SendByte(unsigned char l);
void SendData(unsigned char L);
void SetPack(void);
void DelaymS(unsigned int t);
void Beep(unsigned char t);
void Twinkle_R(void);
void Twinkle_G(void);
//void __attribute((interrupt(0x30))) STB0(void);
//void __attribute((interrupt(0x34))) STB1(void);
//void __attribute((interrupt(0x3c))) UART(void);
//void __attribute((interrupt(0x2c))) STM0AIQR(void);

void main() {
    unsigned char i, j, k;
    unsigned int m;

    _scc = 0x01;
    _hircc = 0x05;
    while(_hircf == 0);


	ComRPC=0;
	ComPC=0;
	pointer_length=0;
	transfer_complete_flag=0;
	Tb1_flg=0;
	Tb0_flg=0;
	RxFlag=0;
//    _mp0 = 0x80;
//    _acc = 128;
//
//    while(_acc != 0) {
//        _iar0 = 0;
//        ++_mp0;
//        _acc = _acc - 1;
//    }
//
//    for(i = 1; i < 8; i++) {
//        _mp1l = 0x80;
//        _mp1h = i;
//        _acc = 128;
//
//        while(_acc != 0) {
//            _iar1 = 0;
//            ++_mp1l;
//            _acc = _acc - 1;
//        }
//    }
//
//    _mp1h = 0;
//    _iar1 = 0;
//    _mp1l = 128;

   //��ʼ������IO
   
//   _pac =0b00000111;
//   _papu=0b00000101;
//   
//   _pbc =0b11101001; 
//   _pbpu=0b11001010;
//   
//   _pcc =0b11111111;
//   _pcpu=0b11111111;
///*   _pawu=0x02; */   
//   
//   _sledc0=0xFF;
//   _sledc1=0;
//   
//   _pa=0b00000000;
//   _pb=0b00000100;
//   _pc=0b00000000;
//   
//   //��ʼ��IO���ù���
//   _pas0=0b00001100;
//   _pas1=0b00011011;
//   _pbs0=0b11001110;   //����
//   _pbs1=0b00001100;
//   _pcs0=0b00000000;
//   _pcs1=0;
//   _ifs1=0b00000000;
   
   _wdtc=0x57;

    //��ʼ��ʱ��
    _pscr = 2;		///32k
    _tb0c = 0x07;
    _tb0e = 1;   //ʱ��0����1�붨ʱ
    _tb1c = 0x06; //ʱ��1����Ϊ0.5��
    _tb1e = 1;
//
//    //��ʼ��PWM
//    _ptmc0 =	0b00000000;	//���÷�Ƶ�� FSYS/4   4Mϵͳʱ��
//    _ptmc1 =	0b10101000;
//    _ptmc2 =	0b00000000;
//    _ptmal =	0x9c;
//    _ptmah =	0;	   //DUTY=50%
//    _ptmrpl =	0x39;
//    _ptmrph =	0x01;	//PERIOD=3.2KHz
    

    //��ʱ��0
    _tb0on = 1;
    _tb1on = 1;
    _emi = 1;
    
    Twinkle_G();
    DelaymS(200);
    Twinkle_G();
	
	UARTInit();
	Stm0AInit();
    while(1) {
//        if(Tb1_flg) {
//            Tb1_flg = 0;
////          Beep(80);
//            Twinkle_R();
//            SendByte(0x10);
//            //Twinkle_G();
//        }
//			Twinkle_R();
//			DelaymS(1000);
//			Twinkle_G();
//			if(RxFlag){
//				RxFlag=0;
//				
//				Twinkle_R();
//			}
			if(transfer_complete_flag){
				transfer_complete_flag=0;
				Twinkle_R();
				SendData(20);
				SendByte(pointer_length);
			}
//			DelaymS(1000);	
//			SendByte(10);
        	GCC_CLRWDT();
    }
}


void DelaymS(unsigned int t) {
    unsigned int i;

    for(i = 0; i < t; i++)
        GCC_DELAY(1000);
}
//�������� p Ƶ�ʣ�v ����,t ����ʱ��mS
void Beep(unsigned char t) {
    //   _ptmrpl=	(unsigned char)(Hz[BeepHz] & 0xff);
    //   _ptmrph=	(unsigned char)(Hz[BeepHz] >>8);
    //   _ptmal=(unsigned char)((Hz[BeepHz]>>1) & 0xff);
    //   _ptmah=(unsigned char)(Hz[BeepHz] >>9);
    _pton = 1;
    EN1 = 1;
    EN2 = 0;
    DelaymS(t);
    _pton = 0;
    EN1 = 0;
    EN2 = 0;
}
/**
 * [__attribute description] 1S��ʱ��
 * @param void [description]
 */
void __attribute((interrupt(0x30))) STB0(void) {
    _tb0f = 0;
    Tb0_flg = 1;
}
/**
 * [__attribute description]0.5s��ʱ��
 * @param void [description]
 */
void __attribute((interrupt(0x34))) STB1(void) {
    _tb1f = 0;
    Tb1_flg = 1;
}

void Twinkle_R(void)
{
   OpenLED_R;
   DelaymS(5);  //5mS
   CloseLED_R;
}

void Twinkle_G(void)
{
   OpenLED_G;
   DelaymS(5);  //5mS
   CloseLED_G;
}
void SetPack(void)
{
	//��������ͷ
	ComCah[0]=0xcc;
	ComCah[1]=0x99;
	ComCah[2]=0x99;
	ComCah[3]=0xcc;
}

void SendData(unsigned char L)
{
	//����L���ֽ�
	unsigned char i;
	for(i=0; i<L; i++) {
		_txr_rxr=ComRCah[i];
		while(_txif==0);
		while(_tidle==0);
	}
}

void SendByte(unsigned char l)
{
	//����1���ֽ�
	_txr_rxr=l;
	while(_txif==0);//�ȴ����ݴӻ��������ص���λ�Ĵ�����
	while(_tidle==0);//�ȴ����ݷ������
}

void __attribute((interrupt(0x3c))) UART(void)
{
	//�����жϺ���
	RxFlag=1;
	_mff=0;
	_urf=0;
/*	_st0on=1;				//�򿪶�ʱ��*/
	while(_rxif>0) {
		ComRCah[ComRPC]=_txr_rxr;
		if(ComRPC<24)
			ComRPC++;
		else
			ComRPC=0;
	}

	//----------------���㶨ʱ��------------
	_st0on=0;
	//GCC_DELAY(50);//�����ʱ�ܹؼ�,�޴˲������������
	_st0on=1;
	//--------------------------------------
}

void UARTInit(void)
{
	//��ʼ��UART			
   _ucr1=0b10000000;
   _ucr2=0b11101100;
   _brg=25;				//4Mʱ��ʱ������9600
   _ure=1;
   _mfe=1;
	// --------------GD700-----------------
   _pbs03=1;			
   _pbs02=1;
   _pbc1=0;				//PB1��TX,���
   _pbs06=1;
   _pbs07=1;			//PB3��RX
   _pbpu3=1;			//pb3����
   _ifs00=0;			//RX ����Դ����ΪPb3
	//-------------------------------------
	_sime=1;			//USIM�ж�ʹ��
	_emi=1;				//���ж�
}

void Stm0AInit(void)
{
	//��ʼ��STM0A,���ڴ��ڶ�ʱ
	_stm0c0 = 0b01000000;//TIM0ʱ�� = fusb ; TIM0ʱ�� = 32kHz ;
	_stm0c1 = 0b11000001;		//��ʱ��ģʽ���Ƚϼ�����Aƥ����0������(STMA�жϺ�Ͳ��õ������������) ����������һ��Ϊ(1/32k)S
	_stm0al = 64;		// 2ms��ʱ
	_stm0ah = 0;			//

	_stm0ae = 1;		//�Ƚϼ�����Aƥ���ж�ʹ��
	_emi = 1;			//ʹ�����ж�

	_st0on = 0;			//�رն�ʱ��
}

void __attribute((interrupt(0x2c))) STM0AIQR(void)
{	//2ms��ʱ����
//	static unsigned int sec = 0;
	_stm0af=0;						//����жϱ�־
	//if(++sec>300) {					//3���룬������ɣ��رն�ʱ��
	//	sec=0;						//����������
	transfer_complete_flag=1;	//������ɱ�־��1
	
	pointer_length=ComRPC;		//��¼ָ�볤��
	ComRPC=0;					//����ָ������
	_st0on=0;					//�ر�STM0A������
	//	SendByte(10);

	//}
}
