//2021-9-8:���벶�����,��Ƭ�����Բ�������ܵ����������壬���ļ����ϣ�����FD7800-1.C�ļ�



#include "BA45F5542-2.h"
#define  s0 _pa6         			/*����0*/
#define  s1 _pa3         			/*����1*/
#define  alarm_output  _pa4         /*�������  io*/
#define  pulse_input  _pa1          /*�����������*/

#define  current_cycle	2500		//���������(΢��)
#define  current_draw_time 5		//�����ʱ��(΢��)
#define  current_wait_time (current_cycle-current_draw_time)	//������ȴ�ʱ��(΢��)
#define  number_of_cycles  (1000000/current_cycle) 				//һ��ѭ������

#define  OpenLED	_isgenc |= 0b10000010; 		///ISINK1 ���Ź����ʹ��,50mA
#define  CloseLED	 _isgenc &= 0b10000001;		///ISINK1 ���Ź��������
#define  ToggleLED	  _isgenc ^= 0b00000010;	///ISINK1 ��ת
#define  OpenISINK0	   _isgenc |= 0b10000001;	///ISINK0 ���Ź����ʹ��
#define  CloseISINK0	_isgenc &= 0b10000010;	///ISINK0 ���Ź��������


//�ж�����
DEFINE_ISR(PTMAIQR, 0x24);
//DEFINE_ISR(UART0, 0x30);
//DEFINE_ISR(UART1, 0x34);

unsigned char anjian1=0;
unsigned char anjian2=0;
unsigned char count=0;
unsigned int Temp=12;
unsigned int z=0;
unsigned int i=0;
unsigned int j=0;
unsigned int k=10;
unsigned int TimerFlag=1;
unsigned int Flag=0;
const unsigned int Hz[13]= {0x0e,0x10,0x11,0x12,0x13,0x14,0x16,0x18,0x19,0x1a,0x1c,0x1e,0x1f};
//300,328, 336, 340, 344,  352, 356,360, 368, 376, 380, 388, 394  V
void anjian(void);
void INT1(void);
void DelaymS(unsigned int t);
void time_init(void);
void Select_sensitivity(void);
void PTMInit(void);
void provide_current_2500uS(void);
void Twinkle(void);

void main()
{

	_wdtc=0x57;			///8��
	_scc=0x01;			///����Ƶ(ϵͳʱ��ΪfH)
	_hircc=0x05;		///4M
	while(_hircf==0);   //4Mʱ�ӣ�����Ƶ
	
	PTMInit();			//���벶׽��ʼ��


	_papu3=0;
	_papu6=0;         //ȡ��ʹ������
	// _papu4=1;

	_pac1=0;
	_pa1=0;
	//_pac4=1;         //0�����1����
	_pac3=1;         //in
	_pac6=1;         //out ���

	_emi=1;

	_pas0=0x11; //pa3
	_pas1=0x00; //pa6//pa4

	_isgdata1=0x00;
	_isgdata0=0X15;//0X1CHz[k]
	_isgen=1;



	while(1) {
		//	 _papu4=1;


//		_isgs1=1;
//		DelaymS(20);
//		for(z=0; z<1000; z++) { //1000
//			_isgs1=0;
//			_isgs0=0;
//			DelaymS(20);
//			_isgs0=1;
//			GCC_DELAY(1);
//			GCC_CLRWDT();
//		}


		//OpenLED;
		for(z=0; z < number_of_cycles; z++) {

			OpenISINK0;//��ISINK0
			//DelaymS(20);	//��ʱ20����
			//_pa1=1;
			GCC_DELAY(current_draw_time);	//��ʱ10΢��

			CloseISINK0;//�ر�ISINK0
			//_pa1=0;
			GCC_DELAY(current_wait_time);	//��ʱ2490΢��
			//DelaymS(20);	//��ʱ20����
		}
		//CloseLED;
		//DelaymS(200);
		//Twinkle();



//		if(pulse_input==1) { //����
//			_isgs1=1;
//			_papu4=1;
//		}
		GCC_CLRWDT();
	}
}

void provide_current_2500uS(void) 	//2500΢��(2.5MS)��һ�ι����
{

	OpenISINK0;//��ISINK0
	//DelaymS(20);	//��ʱ20����
	GCC_DELAY(10);	//��ʱ10΢��

	CloseISINK0;//�ر�ISINK0
	GCC_DELAY(2480);	//��ʱ2490΢��
	//DelaymS(20);	//��ʱ20����
}

void Select_sensitivity(void)
{
	//���ݿ���ѡ��������
	anjian();
	count=anjian1+anjian2;
	switch(count) {
	case 0: { //�궨��
		_pas0=0x51;   //PA3ΪTX
		_pas1=0x21;  //PA6ΪRX
		_isgs1=0;    //isink0��
		break;
	}
	case 1: {
		_pas0=0b00010001; //pa3//A0��A2��ͨѶ
		_pas1=0b01000001; //pa6//PA4��STPB
		Temp=3;        //����������
		break;
	}
	case 2: {
		_pas0=0b00010001; //pa3
		_pas1=0b01000001; //pa6
		Temp=6;        //����������
		break;
	}
	case 3: {
		_pas0=0b00010001; //pa3
		_pas1=0b01000001; //pa6
		Temp=12;        //һ�������ȣ���ͣ�
		break;
	}
	default:
		break;
	}
}



void PTMInit(void) //���벶���ʼ��
{


	_pas10 = 0;
	_pas11 = 0;//PA4����ΪPTCK
	_pac4 = 1;//����
	_pa4 = 0;//Ĭ�ϵ�ƽ
	_papu4=0;//��������

	_ptmc0 = 0b00100000;//PIMʱ�� = sys / 16; PIMʱ�� = 8MHz / 16 = 500kHz;
	_ptmc1 = 0b01000010;//��׽����ģʽ,PTCK ���������벶׽��������ֵ�������� CCRA,
	_ptmc2 = 0x00;//��׽����ģʽʱ��������������ѡ��λ


	//_ptmrpl = 200;//2us * 100 = 200us
	//_ptmrph = 0;//ptmrph�Ĵ���Ϊ8λ��625 = 0000 0010 0111 0001; _ptmrph����ֵ������ȡ�߰�λ���� 0000 0010
	//	_ptmal = 25;//2us * 50 = 100us
	//	_ptmah = 0;// ռ�ձ�Ϊ1/4

	_ptmae = 1;//�Ƚϼ�����Aƥ���ж�ʹ��
	_emi = 1;//ʹ�����ж�

	_pton = 1;//��PTM��ʱ��
}

void PTMAIQR(void)
{
	_ptmaf=0;
	Twinkle();

}

void anjian(void)
{
	if(s0==1) {
		GCC_DELAY(10);
		if(s0==1) anjian1=1;
	}
	if(s1==1) {
		GCC_DELAY(10);
		if(s1==1) anjian2=2;
	}
}

void DelaymS(unsigned int t)
{
	//��ʱһ����
	unsigned int c;
	for(c=0; c<t; c++)
		GCC_DELAY(1000);///4Mʱ��
}


void Twinkle(void)
{
	//����
	OpenLED;
	DelaymS(5);  //5mS
	CloseLED;
}

/*
void time_init(void)
{
   _stmc0=0b00110000;      //fsub
   _stmc1=0b00000000;
   _stmdl=769%256;
   _stmdh=769/256;

   //_int0f=1;


}*/
/*
void INT0(void)
{
   _int0f=0;
   LoseFlag=1;
}
*/