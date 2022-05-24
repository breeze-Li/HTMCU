//2021-9-8:���벶�����,��Ƭ�����Բ�������ܵ�����������.
//2021-9-9:������յ���ѭ�����⣬��ͨ�����ڶ�ȡÿ������������
//2021-9-11:����ͨѶЭ������벶��,��һ��ͨ��Э��ʵ�ֹ���.
//2021��9��16��:����д���úͶ�����,����ʩ�ӵ�������.
//2021��9��28��13:51:58:��ÿ(��/��)��������ܺ���Ϊһ�����ݷ���.
//2021��10��9��14:34:53:������ȡEE����������.(bit�������Ͳ��ɶ���ʱ��ʼ��.)

#include "BA45F5542-2.h"

typedef unsigned char uc;
typedef unsigned int uint;

#define  alarm_output  _pa1         /*�������  io*/
#define  pulse_input  _pa4          /*�����������*/
/*ѭ�����岻�ɺ궨��
#define  current_cycle	2500		//���������(΢��)
#define  current_draw_time 5		//�����ʱ��(΢��)
#define  current_wait_time (current_cycle-current_draw_time)	//������ȴ�ʱ��(΢��)
#define  number_of_cycles  (1000000/current_cycle) 				//һ��ѭ������
*/							

#define  OpenLED	_isgenc |= 0b10000010; 		///ISINK1 ���Ź����ʹ��,50mA
#define  CloseLED	 _isgenc &= 0b10000001;		///ISINK1 ���Ź��������
#define  ToggleLED	  _isgenc ^= 0b00000010;	///ISINK1 ��ת
#define  OpenISINK0	   _isgenc |= 0b10000001;	///ISINK0 ���Ź����ʹ��
#define  CloseISINK0	_isgenc &= 0b10000010;	///ISINK0 ���Ź��������

#define 	MIN_CURRENT	0X08					//������Сֵ
#define		MAX_CURRENT 0X15					//�������ֵ

//�ж�����
DEFINE_ISR(UART, 0x10);
DEFINE_ISR(STMAIQR, 0x2C);
DEFINE_ISR(PTMAIQR, 0x24);


//----------�������Сѡ��-----------------//
//nst unsigned int Hz[13]= {0x0e,0x10,0x11,0x12,0x13,0x14,0x16,0x18,0x19,0x1a,0x1c,0x1e,0x1f};
							//300,328, 336, 340, 344,  352, 356,360, 368, 376, 380, 388, 394  V
//const unsigned int sink_current_level[6]= {0x10,0x11,0x12,0x13,0x14,0x15};		//ISINK0 ���Ź������С


volatile unsigned char ComPC;		        //��������ָ��
volatile unsigned char ComCah[24];          //���ͻ�����
volatile unsigned char ComRPC;		        //��������ָ��
volatile unsigned char ComRCah[24];         //���ջ���
volatile unsigned char pointer_length;		//ָ�볤��(��¼���ܵ��ֽڳ���)
volatile unsigned int LifeCount,LifeTimer;  //������ʱ����λ��12Сʱ
volatile unsigned char MyID[3];        		//��ƷID

//----------ͳ��Ѳ�쳤�������ܺ�,ÿ��ˢ��-------------//
#define inspection_length 3					//Ѳ�쳤��
volatile uc inspection_Array[inspection_length]={0,0,0};	//Ѳ������

volatile uc sampling_pointer;						//��ѯ����ָ��
volatile uc num_of_pulses_per_sec[15] = {0};//15����ÿ����������
//0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
//											0xff,0xff,0xff,0xff,0xff,0xff,0xff
										
//volatile uc num_of_pulses_per_sec_bak[15];	//15����ÿ��������������
//volatile uc sampling_pointerc_bak;			//��ѯ���ݲ���ָ��

volatile bit  receive_complete_flag;		//������ɱ�־
volatile bit  read_sampling_flag;			//��������־
volatile bit  set_complete_flag;			//д��������ɱ�־
volatile uc CALIBRATION_ADD;				//�����õ�ַ
volatile unsigned long temp;		        //��ʱ����
volatile uc temporary_count=0;				//��ʱ����
volatile uc light_count=0;					//���Ƽ���
//----------�������------------------------//
volatile uc Alarm_count=3;					//������������
volatile uc ALARM_CNT=0;					//�ж�����1����
volatile uc ALARM_CNT2=0;					//�ж�����2����
volatile uc ALARM_NUM=1;						//�ж�����1
volatile uc ALARM_NUM2=2;						//�ж�����2
//-----------�洢��ַ����---------------------------//
/*
0  1  2  3
4  5  6
7  8  9  A  B  C  D  E  F  10 11 
12 13 14 15 16 17 18 19 1A 1B 1C 
1D 1E 1F 20 01 22 23 24 25 26 27
*/
#define IDAdd 0x00          //�豸ID  H+M+L+Xor
#define LifeHAdd 0x04	    //����ʱ�䵥λ12Сʱ H+L+Xor
#define CALIBRATION_12M 0x07	    //12M�궨����(9���о�����,1����������,1����־λ)
#define CALIBRATION_17M 0x12	    //17M�궨����
#define CALIBRATION_24M 0x1D	    //24M�궨����

#define next 0x28	    //��һ��

//----------��������------------------------//
void serial_communication(void);
void DelaymS(unsigned int t);
void PTMInit(void);
void UARTInit(void);
void provide_current_2500uS(void);
void Twinkle(void);
void SendByte(unsigned char l);
void SetPack(void);
void SendData(unsigned char L);
void UART(void);
void StmAInit(void);
void TestFlash(void);
void clear_sampling_buff(void);
void WR_SET(volatile uc CALIBRATION_ADD);
void RE_SET(volatile uc CALIBRATION_ADD,uc n);
unsigned char RD_EE(unsigned char RD_EE_addr);
void WR_EE_LAST(uc WR_EE_addr,uc DATA_addr,uc L);
void WR_EE(unsigned char WR_EE_addr,unsigned char WR_EE_data);
uc AddList(volatile uc *p,uc data);


void main()
{

	_wdtc=0b01010110;			///4��
	_scc=0x01;			///����Ƶ(ϵͳʱ��ΪfH)
	_hircc=0x05;		///4M
	while(_hircf==0);   //4Mʱ�ӣ�����Ƶ
	
//----------��ʼ��-------------------------//
	PTMInit();			//���벶׽��ʼ��
	StmAInit();			//��ʼ��STMA,���ڴ��ڶ�ʱ
	UARTInit();			//���ڳ�ʼ��	
	
//----------�����������----------------------//
	_isgdata1=0x00;
	_isgdata0=0X14;
	_isgen=1;
//-----------------������ʼ��---------------------//
	temp = 0;
	ComPC = 0;
	ComRPC = 0;
	sampling_pointer = 0;
	receive_complete_flag = 0;
	set_complete_flag = 0;
	LifeTimer = 0;
	temporary_count = 0;
	
	volatile uint z;					//1Sѭ������
	
	Twinkle();
	DelaymS(300);
	Twinkle();
	
	#define P30S 0
	
	//----------��չ------��ȡEE��������----------------------//
	SendByte(1);
	temp=RD_EE(0x11);
	SendByte(temp);
	if(1==temp){
			SendByte(RD_EE(CALIBRATION_12M+1));
			SendByte(RD_EE(CALIBRATION_12M+2));
			SendByte(RD_EE(CALIBRATION_12M+9));
		_isgdata0=RD_EE(CALIBRATION_12M+9);
		Alarm_count=RD_EE(CALIBRATION_12M);
		ALARM_NUM=RD_EE(CALIBRATION_12M+1);
		ALARM_NUM2=RD_EE(CALIBRATION_12M+2);
	}
	//--------------------------------------//
	
	while(1) {
	
//		for(z=0; z < 15; z++){
//			temp = AddList(inspection_Array,z);
//			DelaymS(400);
//			SendByte(inspection_Array[0]);
//			SendByte(inspection_Array[1]);
//			SendByte(inspection_Array[2]);
//			SendByte(temp);
//			
//			GCC_CLRWDT();
//		}
//		DelaymS(1000);
//		GCC_CLRWDT();

#if 1
		
		if(set_complete_flag){//��ȡEE����������
			set_complete_flag=0;
			_isgdata0=RD_EE(CALIBRATION_ADD+9);		//�������С
			Alarm_count=RD_EE(CALIBRATION_12M);
			SendByte(_isgdata0);
			}

		for(z=0; z < 400; z++){
			//����1�������
			OpenISINK0;//��ISINK0
			GCC_DELAY(5);	//��ʱ5΢��
			CloseISINK0;//�ر�ISINK0
			GCC_DELAY(2495);	//��ʱ2495΢��
		}
		
		if(++light_count>5){
			//6������һ��
			Twinkle();
			light_count=0;
		}
		
		//SendByte(sampling_pointer);
		//DelaymS(100);
//		num_of_pulses_per_sec[sampling_pointer]=temporary_count;//��������
//		SendByte(num_of_pulses_per_sec[sampling_pointer]);		//���͵�ǰ��������

		temp = AddList(inspection_Array,temporary_count);
		//DelaymS(400);
		SendByte(inspection_Array[0]);
		SendByte(inspection_Array[1]);
		SendByte(inspection_Array[2]);
		SendByte(temp);
		
		if(temporary_count>=2) {	//��������������2
			ALARM_CNT=ALARM_CNT+1;
			ALARM_CNT2++;
  		  }      
		else {
		  ALARM_CNT=0;
		  if(temporary_count>=1) ALARM_CNT2++;
		  else ALARM_CNT2=0;
		}
		if(ALARM_CNT>=ALARM_NUM || ALARM_CNT2>=ALARM_NUM2 || temp>=Alarm_count){
			OpenLED
			while(1){
				serial_communication();
				GCC_CLRWDT();
			}
		}
		
//----------�������-------------------//
//		SetPack();
//		ComCah[4]=1;
//		ComCah[5]=temp;
//		ComCah[6]=1^temp;
//		SendData(7);
//--------------------------------------//
		
//		if(sampling_pointer==14 /*&& read_sampling_flag*/){
//			//������15�Ҷ�������־
//			//DelaymS(100);
//			for(i=0; i<15; i++) {
//				SendByte(num_of_pulses_per_sec[i]);
//				//����������ݻ�����
//				num_of_pulses_per_sec[i] = 0xff;
//				};
//			}
		//DelaymS(100);
		sampling_pointer++;
		sampling_pointer = sampling_pointer%15;
		temporary_count=0;
		
	#if P30S
		temp++;
		if(temp==30) SendByte(0xff);
		while(temp==30){
			serial_communication(); //����ͨѶ����
			GCC_CLRWDT();
		}
	#endif
	
#endif
		serial_communication();//����ͨѶ����
		GCC_CLRWDT();
	}	
}
void serial_communication(void){
	//����ͨѶ����
	volatile uc i,j,k;
	#if 1
		if(receive_complete_flag) {
			receive_complete_flag=0;
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
								CALIBRATION_ADD=CALIBRATION_12M;
								WR_SET(CALIBRATION_ADD);
								WR_EE(CALIBRATION_ADD+10, 1);	///��չ
								set_complete_flag=1;
								SetPack();
								ComCah[4]=1;
								ComCah[5]=0x90;
								ComCah[6]=0x91;
								SendData(7);
							}
							break;
						case 0x91:					//����17M������
							if(ComRCah[4]==0x0B) {
								CALIBRATION_ADD=CALIBRATION_17M;
								WR_SET(CALIBRATION_ADD);
								set_complete_flag=1;
								SetPack();
								ComCah[4]=1;
								ComCah[5]=0x91;
								ComCah[6]=0x90;
								SendData(7);
							}
							break;
						case 0x92:					//����24M������
							if(ComRCah[4]==0x0B) {
								CALIBRATION_ADD=CALIBRATION_24M;
								WR_SET(CALIBRATION_ADD);
								set_complete_flag=1;
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
								case 0x93: CALIBRATION_ADD = CALIBRATION_12M;break;
								case 0x94: CALIBRATION_ADD = CALIBRATION_17M;break;
								case 0x95: CALIBRATION_ADD = CALIBRATION_24M;break;
								}
								RE_SET(CALIBRATION_ADD, ComRCah[5]);
								SendData(17);
							}
							break;
						case 0x97:				//���
							clear_sampling_buff();
							SendByte(0xFF);
							SendByte(_isgdata0);
							SendByte(Alarm_count);
							temp=0;
//							SetPack();
//							ComCah[4]=1;
//							ComCah[5]=0x97;
//							ComCah[6]=0x96;
//							SendData(7);
							break;
						case 0x96:				//������
							if(ComRCah[4]==0x01) {
								//���ܵ���������־������������������ȴ�15����������֮����
								//���͸���λ��
								read_sampling_flag=1;			//��������־
								SetPack();
								ComCah[4]=1;
								ComCah[5]=0x96;
								ComCah[6]=0x97;
								SendData(7);
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
						case 0x98:				//��λ
							SendByte(0xFF);
							DelaymS(8000);
							break;
						}
					}
				}
			}
		}

#endif
	}
	
//----------��ʼ���ຯ��----------------------//
void UARTInit(void)
{
	//��ʼ��UART
	_umd=1;				//������
	_uucr1=0x80;
	_uucr2=0b11101100;
	_ubrg=25;			//4Mʱ��ʱ������9600

	// --------------FD718-----------------
   _pas06=1;			//PA3��TX
   _pas15=1;			//PA6��RX
   _papu6=0;			//pa6����
   _ifs00=1;			//RX ����Դ����ΪPa6
	//-------------------------------------
	
	_usime=1;			//USIM�ж�ʹ��
	_emi=1;				//���ж�
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

	//_ston = 0;			//�رն�ʱ��
}

void PTMInit(void) //���벶���ʼ��
{


	_pas10 = 0;
	_pas11 = 0;		//PA4����ΪPA4/PTCK
	_pac4 = 1;		//����
	_pa4 = 0;		//Ĭ�ϵ͵�ƽ
	_papu4=0;		//������������

	_ptmc0 = 0b00100000;//PIMʱ�� = FH/16 = 4M/16;
	_ptmc1 = 0b01000010;//��׽����ģʽ,PTCK ���������벶׽��������ֵ�������� CCRA,
	_ptmc2 = 0x00;//��׽����ģʽʱ��������������ѡ��λ

	_ptmae = 1;//�Ƚϼ�����Aƥ���ж�ʹ��
	_emi = 1;//ʹ�����ж�

	_pton = 1;//��PTM��ʱ��
}

//----------�ж��ຯ��-----------------------//
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
{	
	_stmaf=0;					//����жϱ�־
	receive_complete_flag=1;	//������ɱ�־��1
	pointer_length=ComRPC;		//��¼ָ�볤��
	ComRPC=0;					//����ָ������
	_ston=0;					//�ر�STMA������
	//Twinkle();
	//}
}

void PTMAIQR(void)
{
	//��׽������
	_ptmaf=0;
	temporary_count++; //������һ
	//Twinkle();
}


//----------һ���Զ��庯��---------------------//
void provide_current_2500uS(void) 	//2500΢��(2.5MS)��һ�ι����
{

	OpenISINK0;//��ISINK0
	GCC_DELAY(5);	//��ʱ5΢��
	CloseISINK0;//�ر�ISINK0
	GCC_DELAY(2495);	//��ʱ2495΢��
}

void DelaymS(unsigned int t)
{
	//��ʱһ����
	volatile unsigned int c;
	for(c=0; c<t; c++) GCC_DELAY(1000);///4Mʱ��
}

void Twinkle(void)
{
	//����
	OpenLED;
	GCC_DELAY(5000);  //5mS
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

void SendByte(unsigned char l)
{
	//_emi=0;
	//����1���ֽ�
	_utxr_rxr=l;
	while(_utxif==0);
	while(_utidle==0);
	//_emi=1;
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

void RE_SET(volatile uc CALIBRATION_ADD,uc n)
{
	//�������á�����:�������õ��׵�ַ��ָ���š�
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

void WR_SET(volatile uc CALIBRATION_ADD){
	//д�������á��������õ��׵�ַ
	//����д��9������,��10��������С��Ҫ�ж�
	WR_EE_LAST(CALIBRATION_ADD,6,9);
	//��������
	//--�������С����Сֵ,��д����Сֵ,�������ֵ,д�����ֵ,����д��궨ֵ--//
	if(ComRCah[15]<MIN_CURRENT) 
		WR_EE(CALIBRATION_ADD+9,MIN_CURRENT);
	else if(ComRCah[15]>MAX_CURRENT)
		WR_EE(CALIBRATION_ADD+9,MAX_CURRENT);
	else WR_EE(CALIBRATION_ADD+9,ComRCah[15]);
	//-----------------------------------------------------------------------//	
	}
	
void WR_EE_LAST(uc WR_EE_addr,uc DATA_addr,uc L)
{
	//����д��EE��ַ��ComRCah�д�д�����ݵĵ�ַ��д�볤�ȡ�
	unsigned char i;
	for(i=0; i<L; i++) {
		WR_EE(WR_EE_addr+i,ComRCah[DATA_addr+i]);
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
	uc i=14;
	while(i--) num_of_pulses_per_sec[i]=0xff;
	sampling_pointer=0;			//ָ������
	}
	
uc AddList(volatile uc *p,uc data){
	//��������ǰ��,ĩβ���data.���Ѳ������
	p[0]=p[1];
	p[1]=p[2];
	p[2]=data;
	return p[0]+p[1]+p[2];
	}
	