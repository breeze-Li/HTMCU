//2021-9-8:���벶�����,��Ƭ�����Բ�������ܵ�����������
//2021-9-9:������յ���ѭ�����⣬��ͨ�����ڶ�ȡÿ������������


#include "BA45F5542-2.h"

typedef unsigned char uc;
typedef unsigned int uint;

#define  alarm_output  _pa1         /*�������  io*/
#define  pulse_input  _pa4          /*�����������*/

#define  current_cycle	2500		//���������(΢��)
#define  current_draw_time 5		//�����ʱ��(΢��)
#define  current_wait_time (current_cycle-current_draw_time)	//������ȴ�ʱ��(΢��)
#define  number_of_cycles  (1000000/current_cycle) 				//һ��ѭ������

#define  OpenLED	_isgenc |= 0b10000010; 		///ISINK1 ���Ź����ʹ��,50mA
#define  CloseLED	 _isgenc &= 0b10000001;		///ISINK1 ���Ź��������
#define  ToggleLED	  _isgenc ^= 0b00000010;	///ISINK1 ��ת
#define  OpenISINK0	   _isgenc |= 0b10000001;	///ISINK0 ���Ź����ʹ��
#define  CloseISINK0	_isgenc &= 0b10000010;	///ISINK0 ���Ź��������

uc temporary_count=0;			//��ʱ����

//�ж�����
DEFINE_ISR(PTMAIQR, 0x24);

//----------�������Сѡ��-----------------//
const unsigned int Hz[13]= {0x0e,0x10,0x11,0x12,0x13,0x14,0x16,0x18,0x19,0x1a,0x1c,0x1e,0x1f};
							//300,328, 336, 340, 344,  352, 356,360, 368, 376, 380, 388, 394  V
unsigned int count[6]={0,0,0,0,0,0};


void DelaymS(unsigned int t);
void PTMInit(void);
void UARTInit(void);
void provide_current_2500uS(void);
void Twinkle(void);
void SendByte(unsigned char l);


void main()
{

	_wdtc=0b01010110;			///4��
	_scc=0x01;			///����Ƶ(ϵͳʱ��ΪfH)
	_hircc=0x05;		///4M
	while(_hircf==0);   //4Mʱ�ӣ�����Ƶ
	
	//----------��ʼ��-------------------------//
	PTMInit();			//���벶׽��ʼ��
	UARTInit();				//���ڳ�ʼ��
	
	
	//----------�����������----------------------//
	_isgdata1=0x00;
	_isgdata0=0X10;//0X1CHz[k]
	_isgen=1;
	
	//temporary_count=1;


	Twinkle();
	DelaymS(2000);
	Twinkle();

	
	while(1) {

		
		uint z;
		uc j;
//		for (j=0;j<6;j++){
//			for(z=0; z < 400; z++) {
//				provide_current_2500uS();
//				GCC_CLRWDT();
//				}
//				count[j] = temporary_count;
//				temporary_count=0;
//			}

		for(z=0; z < number_of_cycles; z++){
			//����1�������
			OpenISINK0;//��ISINK0
			GCC_DELAY(current_cycle);	//��ʱ5΢��

			CloseISINK0;//�ر�ISINK0
			GCC_DELAY(current_wait_time);	//��ʱ2495΢��
			}
		
		
		//DelaymS(1000);
		Twinkle();
		SendByte(temporary_count);
		temporary_count=0;
//		SendByte(count[0]);
//		SendByte(count[1]);
//		SendByte(count[2]);
//		SendByte(count[3]);
//		SendByte(count[4]);
//		SendByte(count[5]);

		GCC_CLRWDT();
	}
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
	unsigned int c;
	for(c=0; c<t; c++) GCC_DELAY(1000);///4Mʱ��
}


void Twinkle(void)
{
	//����
	OpenLED;
	GCC_DELAY(5000);  //5mS
	CloseLED;
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