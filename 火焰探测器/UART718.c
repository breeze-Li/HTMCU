
//FD7800��ͨѶЭ�飬����ɡ�����һ��������ָ���У׼��ִ�У���ӦЭ��ΪFD7800����Э��

#include "BA45F5542.h"

/*#define FD718*/
#define SD628

#define  OpenLED    {_isgenc |= 0b10000010;_isgdata1=0b00000000;}	///ISINK1 ���Ź����ʹ��,50mA
#define  CloseLED   _isgenc &= 0b10000001;		///ISINK1 ���Ź��������
#define  reverseLED   _isgenc ^= 0b10000010;	///ISINK1 ��ת

typedef unsigned char uc;
typedef unsigned int uint;



//�����ж�����
DEFINE_ISR(UART, 0x10);
DEFINE_ISR(STMAIQR, 0x2C);


volatile unsigned char ComPC;		        //��������ָ��
volatile unsigned char ComCah[24];          //���ͻ�����
volatile unsigned char ComRPC;		        //��������ָ��
volatile unsigned char ComRCah[24];         //���ջ���
volatile unsigned char pointer_length;		//ָ�볤��(��¼���ܵ��ֽڳ���)
volatile unsigned int LifeCount,LifeTimer;  //������ʱ����λ��12Сʱ
volatile unsigned char MyID[3];        		//��ƷID

volatile uc num_of_pulses_per_sec[15] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15};		//15����ÿ����������
volatile uc sampling_pointer;				//��ѯ����ָ��
//volatile uc num_of_pulses_per_sec_bak[15];	//15����ÿ��������������
//volatile uc sampling_pointerc_bak;			//��ѯ���ݲ���ָ��

volatile bit  RxFlag;       				//�����������ݱ�־
volatile bit  XORFlag;       				//���������Ƿ�Ϊָ���־
volatile bit  transfer_complete_flag;		//���մ�����ɱ�־
volatile bit  read_sampling_flag;			//��������־
volatile unsigned long temp;		        //��ʱ����
//volatile unsigned int sec;					//���ں����ʱ


//-----------�洢��ַ����---------------------------//
/*
0  1  2  3
4  5  6
7  8  9  A  B  C  D  E  F  10
11 12 13 14 15 16 17 18 19 1A
1B 1C 1D 1E 1F 20 01 22 23 24
25
*/
#define IDAdd 0x00          //�豸ID  H+M+L+Xor
#define LifeHAdd 0x04	    //����ʱ�䵥λ12Сʱ H+L+Xor
#define CALIBRATION_12M 0x07	    //12M�궨����
#define CALIBRATION_17M 0x11	    //17M�궨����
#define CALIBRATION_24M 0x1B	    //24M�궨����
#define next 0x25	    //��һ��



void DelaymS(unsigned int t)
{
	//��ʱT����
	unsigned int i;
	for(i=0; i<t; i++)
		GCC_DELAY(1000);
}

void Twinkle(void)
{
	//����
	OpenLED;
	DelaymS(5);  //5mS
	CloseLED;
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
		_utxr_rxr=ComCah[i];
		while(_utxif==0);
		while(_utidle==0);
	}
}

void SendByte(unsigned char l)
{
	//����1���ֽ�
	_utxr_rxr=l;
	while(_utxif==0);
	while(_utidle==0);
}

void UART(void)
{
	//�����жϺ���
	//RxFlag=1;
	_usimf=0;
	_ston=1;				//�򿪶�ʱ��
	while(_urxif>0) {
		ComRCah[ComRPC]=_utxr_rxr;
		if(ComRPC<24)
			ComRPC++;
		else
			ComRPC=0;
	}

	//----------------���㶨ʱ��------------
	_ston=0;
	GCC_DELAY(200);//�����ʱ�ܹؼ�,�޴˲������������
	_ston=1;
	//--------------------------------------
}

void STMAIQR(void)
{	/*
	  �ʼ����ǣ�����һ�ֽ�ǰ����ʱ����һ�ֽڽ�����ɺ���������ٿ���ʱ����CCRA��һ��
	  ���ڽ���һ�ֽڵ�ʱ��,������һ�ֽ�Ҳ�����������������������㣬�ͻ����CCRA�жϡ�
	  ��һ��д�����,������9600ʱ,����һ�ֽ�ֻ��Ҫ1MS,��������ǵø�10MS������������
	  (��ʱ��7�ֽ�),8-24��д�����ݽ���ʱ��������,ֻ������12������,��,ԭ����û����������
	  ������,����10MSֻ�ܽ���12��[0.01/(8/9600)=12],���滹�����ݵĻ�ָ����0��ʼ���ܡ�
	  ���������������sec��ֵ��������,����1MS���ɣ�������ʼ��ʵ�ֵĹ��ܡ�
	  8-26:��CCRA��ֵ���64����ʵ��2MS��ʱ������Ҫsec�����ˣ�Ҳ�������жϴ������жϺ����Ĵ�����
	*/
//	static unsigned int sec = 0;
	_stmaf=0;						//����жϱ�־
	//if(++sec>2) {					//3���룬������ɣ��رն�ʱ��
		//sec=0;						//����������
	transfer_complete_flag=1;	//������ɱ�־��1
	
	pointer_length=ComRPC;		//��¼ָ�볤��
	ComRPC=0;					//����ָ������
	_ston=0;					//�ر�STMA������
	Twinkle();
	//}
}

void shift_data(unsigned char n)
{
	//���ܻ�������ת�Ƶ����ͻ���
	unsigned char i;
	for(i=0; i<n; i++) {
		ComCah[i]=ComRCah[i];
	}
}

void StmAInit(void)
{
	//��ʼ��STMA,���ڴ��ڶ�ʱ
	_stmc0 = 0b01000000;//TIM0ʱ�� = fusb ; TIM0ʱ�� = 32kHz ;
	_stmc1 = 0xc1;		//��ʱ��ģʽ���Ƚϼ�����Aƥ����0������(STMA�жϺ�Ͳ��õ������������) ����������һ��Ϊ(1/32k)S
	_stmal = 64;		// 2ms��ʱ
	_stmah = 0;			//

	_stmae = 1;			//�Ƚϼ�����Aƥ���ж�ʹ��
	_emi = 1;			//ʹ�����ж�

	_ston = 0;			//�رն�ʱ��
}

void UARTInit(void)
{
	//��ʼ��UART
	_umd=1;				//������
	_uucr1=0x80;
	_uucr2=0b11101100;
	_ubrg=25;			//4Mʱ��ʱ������9600
	#ifdef SD628
	// --------------SD628-----------------
	_pbs03=1;			//PB1��TX
	_pbs07=1;			//PB3��RX
	_pbpu3=1;			//pb3����
	_ifs01=1;			//RX ����Դ����ΪPB3
	#endif
	#ifdef FD718
	// --------------FD718-----------------
   _pas06=1;			//PA3��TX
   _pas15=1;			//PA6��RX
   _papu6=1;			//pa6����
   _ifs00=1;			//RX ����Դ����ΪPa6
	//-------------------------------------
	#endif
	
	_usime=1;			//USIM�ж�ʹ��
	_emi=1;				//���ж�
}

unsigned char RD_EE(unsigned char RD_EE_addr)
{
	//��EE
	_emi=0;
	_eea=RD_EE_addr;
	_mp1l=0x40;
	_mp1h=0x01;
	_iar1=_iar1|0x03;
//	_iar1.1=1;
//	_iar1.0=1;
	while(_iar1&0x01);
	_iar1=0x00;
	_mp1h=0x00;
	RD_EE_addr=_eed;
	_emi=1;
	return RD_EE_addr;
}

void WR_EE(unsigned char WR_EE_addr,unsigned char WR_EE_data)
{
	//дEE
	_emi=0;
	_eea=WR_EE_addr;
	_eed=WR_EE_data;
	_mp1l=0x40;
	_mp1h=0x01;
	_iar1=_iar1|0b00001000;
	_iar1=_iar1|0b00000100;
	while(_iar1&0b00000100) ;
	_iar1=0x00;
	_mp1h=0x00;
	_emi=1;
}

void WR_EE_LAST(uc WR_EE_addr,uc DATA_addr,uc L)
{
	//����д��EE��ַ��ComRCah�д�д�����ݵĵ�ַ��д�볤�ȡ�
	unsigned char i;
	for(i=0; i<L; i++) {
		WR_EE(WR_EE_addr+i,ComRCah[DATA_addr+i]);
	}
}

void RE_SET(volatile uc CALIBRATION_ADD,uc n)
{
	//�������á��������õ��׵�ַ��ָ���š�
	uc i;						//ָ����
	SetPack();					//����ͷ
	ComCah[4] = 0X0b;			//
	ComCah[5] = n;
	ComCah[16]=0x0B ^ n;		//У��4 5��λ
	for(i=0; i<10; i++) {
		ComCah[6+i] = RD_EE(CALIBRATION_ADD+i);		//����λ��ʼ���ö�ȡ������
		ComCah[16] ^= ComCah[6+i];					//����У��6-10λ
	}
}

void TestFlash(void)
{
	//ȡEE����
	unsigned char TempCah[10];
	//ȡID
	MyID[0]=RD_EE(IDAdd);
	MyID[1]=RD_EE(IDAdd+1);
	MyID[2]=RD_EE(IDAdd+2);

	TempCah[0]=RD_EE(IDAdd+3);
	TempCah[1]=MyID[0] + MyID[1] + MyID[2];
	if(TempCah[0]!=TempCah[1]) {			//���û��д��ID�������0 0 0
		MyID[0]=0;
		MyID[1]=0;
		MyID[2]=0;
	}
	//ȡ����ʱ��
	TempCah[0]=RD_EE(LifeHAdd);
	TempCah[1]=RD_EE(LifeHAdd+1);
	TempCah[2]=RD_EE(LifeHAdd+2);
	TempCah[3]=TempCah[0] + TempCah[1];
	if(TempCah[2]==TempCah[3]) {
		LifeCount=TempCah[0];
		LifeCount=(LifeCount<<8)+TempCah[1];
	} else {
		TempCah[0]=RD_EE(LifeHAdd+0x20);
		TempCah[1]=RD_EE(LifeHAdd+0x21);
		TempCah[2]=RD_EE(LifeHAdd+0x22);
		TempCah[3]=TempCah[0] + TempCah[1];
		if(TempCah[2]==TempCah[3]) {
			LifeCount=TempCah[0];
			LifeCount=(LifeCount<<8)+TempCah[1];
		} else
			LifeCount=0;
	}
}

void clear_sampling_buff(void){
	//����������ݻ�����
	uc i=15;
	while(i--) num_of_pulses_per_sec[i]=0xff;
	sampling_pointer=0;			//ָ������
	}
	


void main()
{
	unsigned char i,j,k;


	_wdtc=0x57;					//8�빷
	_scc=0x01;
	_hircc=0b00000101;
	while(_hircf==0);	//4Mʱ��

	StmAInit();				//��ʼ��STMA,���ڴ��ڶ�ʱ
	UARTInit();				//��ʼ��UART


	//-----------------������ʼ��---------------------//
	RxFlag=0;
	ComPC=0;
	ComRPC=0;
	XORFlag=0;
	transfer_complete_flag=0;
	LifeTimer=0;

	//SetPack();

//	_pas0=0b00000000;
//   _pas1=0b10010010;	///PA7��AN1 //_pas1=0b00010010;
//
//   _pbs0=0b10001000;
//   _pbs1=0b00000000;
//
//   _ifs0=0b00000010;





	while(1) {



#if 1
		if(transfer_complete_flag) {
			SendByte(23);
			transfer_complete_flag=0;
			switch(1) {
				//-------�������ͷ,������˳�---------//
			case 1:
				if(ComRCah[0]!=0xcc)
					break;
			case 2:
				if(ComRCah[1]!=0x99)
					break;
			case 3:
				if(ComRCah[2]!=0x99)
					break;
			case 4:
				if(ComRCah[3]!=0xcc)
					break;
				//--------------------------------------//
			default:
				if(pointer_length>=7 && pointer_length==ComRCah[4]+6) { //ָ�����ȷ
					//----------ָ�����У��----------------------------//
					j=ComRCah[4];
					for(k=0; k<ComRCah[4]; k++)
						j=j ^ ComRCah[5+k];
					//----------------------------------------------//
					if(j==ComRCah[ComRCah[4]+5]) {	//ָ��У����ȷ
						switch(ComRCah[5]) {		//ָ������
						case 0x80:					//������
							if(ComRCah[4]==1) {
								SetPack();
								ComCah[4]=1;
								ComCah[5]=0x80;
								ComCah[6]=0x81;
								SendData(7);
							}
							break;
						case 0x81:					//дID
							if(ComRCah[4]==4) {
								ComRCah[9]=ComRCah[6]+ComRCah[7]+ComRCah[8];
								WR_EE(IDAdd,ComRCah[6]);
								WR_EE(IDAdd+1,ComRCah[7]);
								WR_EE(IDAdd+2,ComRCah[8]);
								WR_EE(IDAdd+3,ComRCah[9]);
								TestFlash();
								SetPack();
								ComCah[4]=1;
								ComCah[5]=0x81;
								ComCah[6]=0x80;
								SendData(7);
							}
							break;
						case 0x84:					//��ID
							if(ComRCah[4]==1) {
								SetPack();
								TestFlash();		
								ComCah[4]=4;
								ComCah[5]=0x84;
								ComCah[6]=MyID[0];
								ComCah[7]=MyID[1];
								ComCah[8]=MyID[2];
								ComCah[9]=ComCah[8] ^ ComCah[7] ^ ComCah[6] ^ 0x80;	//0x80=0x84^0x04
								SendData(10);
							}
							break;
						case 0x88:					//дʹ��ʱ��
							if(ComRCah[4]==3) {
								WR_EE(LifeHAdd,ComRCah[6]);
								WR_EE(LifeHAdd+1,ComRCah[7]);
								WR_EE(LifeHAdd+2,ComRCah[6]+ComRCah[7]);
								WR_EE(LifeHAdd+0x20,ComRCah[6]);
								WR_EE(LifeHAdd+0x21,ComRCah[7]);
								WR_EE(LifeHAdd+0x22,ComRCah[6]+ComRCah[7]);
								TestFlash();
								LifeTimer=0;		//1S��ʱ����
								SetPack();
								ComCah[4]=1;
								ComCah[5]=0x88;
								ComCah[6]=0x89;
								SendData(7);
							}break;
						case 0x87:					//��ʹ��ʱ��
							if(ComRCah[4]==1) {
								SetPack();
								ComCah[4]=3;
								ComCah[5]=0x87;
								ComCah[6]=LifeCount	>> 8;
								ComCah[7]=(unsigned char)(LifeCount & 0xff);
								ComCah[8]=ComCah[7] ^ ComCah[6] ^ ComCah[5] ^ 3;
								SendData(9);
							}
							break;
						case 0x90:					//����12M������
							if(ComRCah[4]==0x0B) {
								WR_EE_LAST(CALIBRATION_12M,6,10);//д������
								SetPack();
								ComCah[4]=1;
								ComCah[5]=0x90;
								ComCah[6]=0x91;
								SendData(7);
							}
							break;
						case 0x91:					//����17M������
							if(ComRCah[4]==0x0B) {
								WR_EE_LAST(CALIBRATION_17M,6,10);//д������
								SetPack();
								ComCah[4]=1;
								ComCah[5]=0x91;
								ComCah[6]=0x90;
								SendData(7);
							}
							break;
						case 0x92:					//����24M������
							if(ComRCah[4]==0x0B) {
								WR_EE_LAST(CALIBRATION_24M,6,10);//д������
								SetPack();
								ComCah[4]=1;
								ComCah[5]=0x92;
								ComCah[6]=0x93;
								SendData(7);
							}
							break;
							//----------���������Ϊ������----------------//
						case 0x93:
						case 0x94:
						case 0x95:
							if(ComRCah[4]==0x01) {
								switch(ComRCah[5]) {
								//----------ѡ����ĸ������ȵ�����-----------------//
								case 0x93: temp = CALIBRATION_12M;break;
								case 0x94: temp = CALIBRATION_17M;break;
								case 0x95: temp = CALIBRATION_24M;break;
								}
								RE_SET(temp, ComRCah[5]);
								SendData(17);
							}
							break;
						case 0x97:				//���
							clear_sampling_buff();
							
						case 0x96:				//������
							if(ComRCah[4]==0x01) {
								//���ܵ���������־������������������ȴ�15����������֮����
								//���͸���λ��
								read_sampling_flag=1;			//��������־
//								SetPack();
//								ComCah[4]=16;
//								ComCah[5]=0x96;
//								ComCah[21]=0x96 ^ 16;
//								//-----------�����������뷢�ͻ�����---------------//
//								for(i=0; i<15; i++) {
//									ComCah[6+i] = num_of_pulses_per_sec[i];
//									ComCah[21] ^= ComCah[6+i];
//									//����������ݻ�����
//									num_of_pulses_per_sec[i] = 0xff;
//									};
//								SendData(22);			//��������	
//								//--------------------------------------//
							}
							break;

							
						}
					}
				}
			}
		}

#endif
		GCC_CLRWDT();
	}
}