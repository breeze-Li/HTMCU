///C:\Program Files\Holtek MCU Development Tools\HT-IDE3000V8.x\BIN
//2021-9-8:���벶�����,��Ƭ�����Բ�������ܵ�����������.
//2021-9-9:������յ���ѭ�����⣬��ͨ�����ڶ�ȡÿ������������
//2021-9-11:����ͨѶЭ������벶��,��һ��ͨ��Э��ʵ�ֹ���.
//2021��9��16��:����д���úͶ�����,����ʩ�ӵ�������.
//2021��9��28��13:51:58:��ÿ(��/��)��������ܺ���Ϊһ�����ݷ���.
//2021��10��9��14:34:53:������ȡEE����������.(bit�������Ͳ��ɶ���ʱ��ʼ��.)
//2021��10��14��:�����оݡ�
//2021��11��15��13:04:35:�Ż��о�,������ſ�ʼ�ж�,�����ݴ��ʡ�
//2021��11��18�գ���A0��A2����ԭ���Ĵ��ڽ�������ѡ�񿪹ء�(��Ҫֱ�Ӷ�_pa,
//				  ��Ϊ����Ҳ��Pa��,TxRx���Ϻ��仯,ֱ�Ӷ�paҪ����0,2��λ��
//				  �ɴ���¼ʱ���뿪�ر���ȫ������)
//2021��11��19��:������ѡ�����,����,���������ɡ�
//2022��2��25�գ������������
//2022��3��31�գ��ж���ʽ�оݣ�ǰ�涼����......��
//2022��6��21�գ����ó�ʼ�оݣ�ʡ�Ե�һ�α궨
//2022��9��30�գ�����������gengral_array��¼���20�����ݣ������Ա����ݣ�����д��EE����¼ÿ�������ȵı��������о�

#include "BA45F5542-2.h"

typedef unsigned char uc;
typedef unsigned int uint;

#define  alarm_output  _pa1         /*�������  io*/
#define  pulse_input  _pa4          /*�����������*/

#define  OpenLED	_isgenc |= 0b10000010; 		///ISINK1 ���Ź����ʹ��,50mA
#define  CloseLED	 _isgenc &= 0b10000001;		///ISINK1 ���Ź��������
#define  ToggleLED	  _isgenc ^= 0b00000010;	///ISINK1 ��ת
#define  OpenISINK0	   _isgenc |= 0b10000001;	///ISINK0 ���Ź����ʹ��
#define  CloseISINK0	_isgenc &= 0b10000010;	///ISINK0 ���Ź��������

#define 	MIN_CURRENT	0X08					//������Сֵ
#define		MAX_CURRENT 0X1F					//�������ֵ

#define     TYPE 0x12
#define     VERSION 0x10

//�ж�����
DEFINE_ISR(UARTIQR, 0x10);
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
//----------ͳ��Ѳ�쳤�������ܺ�,ÿ��ˢ��-------------//
#define inspection 5
#define statistics 30
volatile uc inspection_Array[inspection] = {0};	//Ѳ�����飨ʱ�䴰����������
volatile uc statistics_Array[statistics] = {0};	//ͳ������(�����ж�)
volatile uc gengral_Array[20] = {0};	//��ͨ����(��¼���30�����ݣ�д��EE)
//volatile uc sampling_pointer;						//��ѯ����ָ��
//volatile uc num_of_pulses_per_sec[15] = {0};//15����ÿ����������

volatile bit  receive_complete_flag;		//������ɱ�־
volatile bit  read_sampling_flag;			//��������־
volatile bit  set_complete_flag;			//д��������ɱ�־
volatile bit  trigger_flag;					//�����оݱ�־
volatile bit  fire_flag;					//�𾯱�־
volatile bit  fire_flag1;					//�𾯱�־
volatile bit  calibration_flag;				//�궨��־
volatile bit  gameover;						//��������
volatile unsigned int LifeCount,LifeTimer;  //������ʱ����λ��12Сʱ
volatile unsigned char MyID[3];        		//��ƷID
volatile uc pulses_count=0;					//�������
volatile uc light_count=0;					//���Ƽ���
volatile uc zero_pulses_count=0;			//�������������
volatile uc sensitivity_value=4;			//ѡ��������ֵ����ʼ��Ϊ0123֮���ֵ��
volatile uc sensitivity_value_old=4;		//ѡ�������Ⱦ�ֵ���Ա��ϵ�ѡ�������ȣ�
volatile unsigned long temp;		        //��ʱ����
//----------�о�1  B��m��ʱ�䴰����A��m��ʱ�䴰��������������n---//
volatile uc period=1;						//ʱ�䴰����(m)
volatile uc pulses_per_period=2;			//ÿʱ�䴰�ڵ��������(n)
volatile uc alarm_period=2;					//������ʱ�䴰����(A)
volatile uc continuous_period=10;			//������ʱ�䴰����(B)
volatile uc period_count=0;					//ʱ�䴰���ȼ���
volatile uc alarm_period_count=0;			//������ʱ�䴰����
volatile uc continuous_period_count=0;		//������ʱ�䴰����
volatile uc continuous_period_count1=0;		//������ʱ�䴰����1
//----------�о�2  ----------------------------//
volatile uc count=0;						//�о�2ʱ�����
volatile uc total_pause =10;				//������
volatile uc total_pause_time =15;				//������
volatile uc total_pause_count =0;			//���������
//volatile uc total_time=0;

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
volatile uc CALIBRATION_ADD=CALIBRATION_12M;	//Ĭ�϶����õ�ַ

#define ERROR 0x28	    //��־
#define DATA 0x2B		//30miao����

//----------��������------------------------//

void write_criteria(void);
void select_sensitivity(void);
void serial_communication(void);
void DelaymS(unsigned int t);
void PTMInit(void);
void UARTInit(void);
void MCUinit(void);
void Init_Criterion(void);
void Twinkle(void);
void Twinkles(uc i);
void Fire_Alarm(void);
void SendByte(uc);
void SetPack(void);
void SendData(uc);
void UARTIQR(void);
void StmAInit(void);
void TestFlash(void);
void clear_sampling_buff(void);
void clear_inspection_Array(void);
void WR_SET(volatile uc );
void RE_SET(volatile uc ,uc n);
uc RD_EE(unsigned char );
void WR_EE_LAST(uc ,uc ,uc L);
void WR_EE(unsigned char ,unsigned char );
uc Add_Array(volatile uc *p, uc data, uc len);
void Add_gengral(uc data);

void main()
{
//-----------------������ʼ��---------------------//
	volatile uint z=0;					//1Sѭ������
	volatile uc life[3]={0};
	volatile uc i=0,j=0;					//�жϱ�������
//----------��ʼ��-------------------------//
	PTMInit();			//���벶׽��ʼ��
	StmAInit();			//��ʼ��STMA,���ڴ��ڶ�ʱ
	UARTInit();			//���ڳ�ʼ��
	MCUinit();
	Twinkle();
	DelaymS(200);		//��ʱд��������
	Init_Criterion();
	TestFlash();
	while(1) {
		if(set_complete_flag){//��ȡEE����������
			set_complete_flag=0;
			SendByte(period);
			SendByte(pulses_per_period);
			SendByte(alarm_period);
			SendByte(continuous_period);
			SendByte(total_pause);
			SendByte(total_pause_time);
			SendByte(_isgdata0);
			clear_inspection_Array();
		}
		
//----------�Ǳ궨״̬�¿��ر仯ʱ��д������ʱѡ��������------------------//
		if(!calibration_flag)
			select_sensitivity();
//------------------------------------------------------//

		for(z=0; z < 50; z++){
			//����1�������
			OpenISINK0;			//��ISINK0
			GCC_DELAY(5);		//��ʱ5΢��
			CloseISINK0;		//�ر�ISINK0
			GCC_DELAY(19995);	//��ʱ2495΢��
		}
		
		if(++LifeTimer > 43200) { //43200 12hours
            LifeTimer = 0;

            if(LifeCount <= 2160) { //2160 3years
                LifeCount++;
                //�洢ʱ��
                life[0] = LifeCount >> 8;
                WR_EE(LifeHAdd, life[0]);
                life[1] = LifeCount & 0xff;
                WR_EE(LifeHAdd + 1, life[1]);
                life[2] = life[0] + life[1];
                WR_EE(LifeHAdd + 2, life[2]);
            } else
                gameover = 1;
        }
		
		if(++light_count>4){
			//5�����ƣ���������������
			if(gameover){
				Twinkles(2);
			}else {
				Twinkle();
			}
			light_count=0;
//			fire_flag=1;
//			trigger_flag=1;
		}
		
//----------�궨״̬�´������-------------------//
/*		SendByte(pulses_count);*/
		if(calibration_flag){
			SetPack();
			ComCah[4]=1;
			ComCah[5]=pulses_count;
			ComCah[6]=pulses_count ^ 1;
			SendData(7);
		}
//--------------------------------------//
	
	
	Add_gengral(pulses_count);//��¼
	/*----------�������忪ʼ�����о�-------------*/	
	if (pulses_count){						//���������
		//SendByte(0x89);
		zero_pulses_count=0;				//��������
		trigger_flag=1;						//��ʼ�����о�
	}									
	else if(trigger_flag){	 				//���û���������Ѿ������о�
		//SendByte(0x99);
		//zero_pulses_count++;				//������һ
		if (++zero_pulses_count>30){
			//SendByte(0x90);
			zero_pulses_count=0;			//����15��û������
			trigger_flag=0;					//ȡ�������о�
			//----------�оݼ�������-----------------//
			period_count=0;
			alarm_period_count=0;
			continuous_period_count=0;
			//continuous_period_count1=0;
		}									
	}	
	if(trigger_flag && (!calibration_flag)){	//�Ǳ궨״̬��ʼ�����о�
		//�о�2 nS����������
		
		//Add_gengral(pulses_count);
		if(++count < total_pause_time){
			total_pause_count += pulses_count;
			if(total_pause_count>=total_pause) fire_flag1=1;
			}else{
				count=0;
				total_pause_count=0;
		}
		
		//--------period_countѭ��-----------------//
		period_count++;
		period_count=period_count % period;
		//-----------------------------------------//
		temp = Add_Array(inspection_Array,pulses_count,period);//�������
		
		//����ʱ�䴰��
		if(period_count==0){
			//---------continuous_period_countѭ��--------------//
			//continuous_period_count++;
			//continuous_period_count=continuous_period_count%continuous_period;
			
			//ʱ�䴰��������ͳ������
			Add_Array(statistics_Array, temp, continuous_period);
			for(i=0; i<continuous_period; i++){
				//SendByte(statistics_Array[i]);
				//�ж�ʱ�䴰�������Ƿ�ﵽ����ֵ,A+1
				if(statistics_Array[i] >= pulses_per_period){
					alarm_period_count++;
				};	
			}	
		};
//		SendByte(alarm_period_count);
//		SendByte(alarm_period);
		if(alarm_period_count >= alarm_period) {
			fire_flag=1;
		}else{
			alarm_period_count=0;//���������������Ȼ���ۼ�
		}
		if(fire_flag || fire_flag1) {
			//��¼�������
			j = (fire_flag<<4) + fire_flag1;
			switch(sensitivity_value){
			
			case 3://S1=S2=0��12��������
					WR_EE(ERROR, j);break;
			case 2://S1=1��S2=0��17��������
			case 1: 
					WR_EE(ERROR+1, j);break;		
			
			case 0://S1=S2=1��24��������
					WR_EE(ERROR+2, j);break;
			}
			//��¼20S����
			for(i=0;i<20;i++){
				WR_EE(DATA+i,gengral_Array[i]);
				SendByte(gengral_Array[i]);
			}
			Fire_Alarm();
		}

//		//����ʱ�䴰��(B),A��������
//		if(continuous_period_count == 0 && period_count== 0 ){
//			//��B��,��m������һ��ĩ���㣬��Ϊm��1��ʼ������
//			//�����������ݶ�����ƶ���һλ��
//			//�����B������һ�����㡣����B��M��Ϊ�㡣
//			//++continuous_period_count1;
//			//if(continuous_period_count1 == 4-period){ 
//				//continuous_period_count1=0;
//				//clear_inspection_Array();
//				alarm_period_count=0;
//				//SendByte(0xff);
//			//	}
//		}
	}
		pulses_count=0;		   //����������
		//serial_communication();//����ͨѶ����
		GCC_CLRWDT();
	}	
}

void serial_communication(void){
	//����ͨѶ����
	volatile uc i,j,k;
	if(receive_complete_flag) {
		receive_complete_flag=0;
		switch(1) {
			//-------�������ͷ,������˳�---------//
		case 1:
			if(ComRCah[0]!=0xcc) break;
		case 2:
			if(ComRCah[1]!=0x99) break;
		case 3:
			if(ComRCah[2]!=0x99) break;
		case 4:
			if(ComRCah[3]!=0xcc) break;
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
					case 0x87:					//��ʹ��ʱ��
						if(ComRCah[4]==1) {
							SetPack();
							TestFlash();
							ComCah[4]=3;
							ComCah[5]=0x87;
							ComCah[6]=LifeCount	>> 8;
							ComCah[7]=(unsigned char)(LifeCount & 0xff);
							ComCah[8]=ComCah[7] ^ ComCah[6] ^ ComCah[5] ^ 3;
							SendData(9);
						}
						break;
					case 0x88:					//дʹ��ʱ��
						if(ComRCah[4]==3) {
							WR_EE(LifeHAdd,ComRCah[6]);
							WR_EE(LifeHAdd+1,ComRCah[7]);
							WR_EE(LifeHAdd+2,ComRCah[6]+ComRCah[7]);
							TestFlash();
							LifeTimer=0;		//ÿ1S��ʱ����
							SetPack();
							ComCah[4]=1;
							ComCah[5]=0x88;
							ComCah[6]=0x89;
							SendData(7);
						}break;
					case 0x89:
						if(ComRCah[4]==1) {
							SetPack();
							ComCah[4]=2;
							ComCah[5]=0x89;
							ComCah[6]=TYPE;
							ComCah[7]=VERSION;
							ComCah[8]=ComCah[7] ^ ComCah[6] ^ 0x8B;	//0x80=0x89^0x02
							SendData(9);
						}
					case 0x90:					//����12M������
						if(ComRCah[4]==0x0B) {
							CALIBRATION_ADD=CALIBRATION_12M;
							WR_SET(CALIBRATION_ADD);
							WR_EE(CALIBRATION_ADD+10, 1);	///д��EE��־
							set_complete_flag=1;
							TestFlash();
							SetPack();
							ComCah[4]=1;
							ComCah[5]=0x90;
							ComCah[6]=0x91;
							SendData(7);
							clear_inspection_Array();
						}
						break;
					case 0x91:					//����17M������
						if(ComRCah[4]==0x0B) {
							CALIBRATION_ADD=CALIBRATION_17M;
							WR_SET(CALIBRATION_ADD);
							WR_EE(CALIBRATION_ADD+10, 1);	///д��EE��־
							set_complete_flag=1;
							TestFlash();
							SetPack();
							ComCah[4]=1;
							ComCah[5]=0x91;
							ComCah[6]=0x90;
							SendData(7);
							clear_inspection_Array();
						}
						break;
					case 0x92:					//����24M������
						if(ComRCah[4]==0x0B) {
							CALIBRATION_ADD=CALIBRATION_24M;
							WR_SET(CALIBRATION_ADD);
							WR_EE(CALIBRATION_ADD+10, 1);	///д��EE��־
							set_complete_flag=1;
							TestFlash();
							SetPack();
							ComCah[4]=1;
							ComCah[5]=0x92;
							ComCah[6]=0x93;
							SendData(7);
							clear_inspection_Array();
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
						//clear_sampling_buff();
						SendByte(0xFF);
						SendByte(period);
						SendByte(pulses_per_period);
						SendByte(alarm_period);
						SendByte(continuous_period);
						SendByte(total_pause);
						SendByte(total_pause_time);
						SendByte(_isgdata0);
						SendByte(RD_EE(ERROR));
						SendByte(RD_EE(ERROR + 1));
						SendByte(RD_EE(ERROR + 2));
						SendByte(fire_flag);
						SendByte(fire_flag1);
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
					case 0x98 :				//��λ
						SetPack();
						ComCah[4]=1;
						ComCah[5]=0x98;
						ComCah[6]=0xFB;
						SendData(7);
						WR_EE(ERROR, 0);
						WR_EE(ERROR + 1, 0);
						WR_EE(ERROR + 2, 0);
						DelaymS(8000);
						break;
					case 0x99 :
						calibration_flag=1;  //�궨��־��һ
						SetPack();
						ComCah[4]=1;
						ComCah[5]=0x99;
						ComCah[6]=0x98;
						SendData(7);
						break;
					case 0x9a :
						calibration_flag=0;
						SetPack();
						ComCah[4]=1;
						ComCah[5]=0x9A;
						ComCah[6]=0x9B;
						SendData(7);
						break;
					case 0x9b :
						//����
						for(i=0;i<20;i++){
							SendByte(RD_EE(DATA+i));
//							WR_EE(DATA+i,gengral_Array[i]);
						}
//						fire_flag=1;
//						trigger_flag=1;
					}
				}
			}
		}
	}
}

//----------��ʼ���ຯ��----------------------//
void MCUinit(void){
	_wdtc=0b01010101;	///2��
	_scc=0x01;			///����Ƶ(ϵͳʱ��ΪfH)
	_hircc=0x05;		///4M
	while(_hircf==0);   //4Mʱ�ӣ�����Ƶ
	//----------��������-----------------//
	_pac1=0;			//���
	_pa1 = 0;
	_pac2=1;			//����
	_pac0=1;
	_papu0=1;			//����
	_papu2=1;
	//----------�����������----------------------//
	_isgdata1=0x00;
	_isgdata0=0X0a;
	_isgen=1;
	
	temp = 0;
	ComPC = 0;
	ComRPC = 0;
//	sampling_pointer = 0;
	receive_complete_flag = 0;
	set_complete_flag = 0;
	calibration_flag=0;
	pulses_count = 0;
	trigger_flag=0;
	fire_flag=0;
	fire_flag1=0;
	gameover=0;
	LifeTimer = 0;
	
}
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
}

void PTMInit(void) //���벶���ʼ��
{
	_pas10 = 0;
	_pas11 = 0;		//PA4����ΪPA4/PTCK
	_pac4 = 1;		//����
	_pa4 = 0;		//Ĭ�ϵ͵�ƽ
	_papu4=0;		//������������

	_ptmc0 = 0b00100000;//PIMʱ�� = FH/16 = 4M/16;
	_ptmc1 = 0b01000010;//��׽����ģʽ,PTCK ���������벶׽��������ֵ�������� CCRA
	_ptmc2 = 0x00;//��׽����ģʽʱ��������������ѡ��λ

	_ptmae = 1;//�Ƚϼ�����Aƥ���ж�ʹ��
	_emi = 1;//ʹ�����ж�

	_pton = 1;//��PTM��ʱ��
}

//----------�ж��ຯ��-----------------------//
void UARTIQR(void)
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
	GCC_DELAY(50);//�����ʱ�ܹؼ�,�޴˲������������
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
	serial_communication();
}

void PTMAIQR(void)
{	//��׽������
	_ptmaf=0;
	pulses_count++; //������1
}

//----------һ���Զ��庯��---------------------//
void Fire_Alarm(void){
	//����
	_isgs0 = 0;
	OpenLED				//����
	_papu1=1;			//����PA1
	alarm_output=1;		//PA1���1
	while(1){
		serial_communication();
		GCC_CLRWDT();	
	}
}
	
void SetPack(void)
{	//��������ͷ
	ComCah[0]=0xcc;
	ComCah[1]=0x99;
	ComCah[2]=0x99;
	ComCah[3]=0xcc;
}

void SendByte(unsigned char l)
{	//����1���ֽ�
	_utxr_rxr=l;
	while(_utxif==0);
	while(_utidle==0);
}

void SendData(unsigned char L)
{	//����L���ֽ�
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
{	//�������á�����:�������õ��׵�ַ��ָ���š�
	uc i;						//
	SetPack();					//����ͷ
	ComCah[4] = 0X0b;			//ָ����
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
{	//����д��EE��ַ��ComRCah�д�д�����ݵĵ�ַ��д�볤�ȡ�
	unsigned char i;
	for(i=0; i<L; i++) {
		WR_EE(WR_EE_addr+i,ComRCah[DATA_addr+i]);
	}
}

void select_sensitivity(void){
	//ѡ�������ȿ���ֵ
	sensitivity_value=_pa0+(_pa2<<1);

	if(sensitivity_value != sensitivity_value_old){
		switch(sensitivity_value){
		/*
		 A0		A2		S1		S2		sensitivity_value
		 1		1		0		0		3 12M
		 0		1		1		0		2 17M
		 1		0		0		1		1 17M
		 0		0		1		1		0 24M		*/

		//S1=S2=0��12��������
		case 3:
				CALIBRATION_ADD=CALIBRATION_12M;
				break;
		//S1=1��S2=0��17��������
		case 2:
				CALIBRATION_ADD=CALIBRATION_17M;
				break;
		//S1=0��S2=1��17��������
		case 1: 
				CALIBRATION_ADD=CALIBRATION_17M;
				break;		
		//S1=S2=1��24��������
		case 0:
				CALIBRATION_ADD=CALIBRATION_24M;
				break;
		}
		TestFlash();
		sensitivity_value_old=sensitivity_value;	
	}	
}	

void DelaymS(unsigned int t)
{	//��ʱһ����
	volatile unsigned int c;
	for(c=0; c<t; c++) GCC_DELAY(1000);///4Mʱ��
}

void Twinkle(void)
{	//����
	OpenLED;
	GCC_DELAY(5000);  //5mS
	CloseLED;
}

void Twinkles(uc i){
	while(i--){
		Twinkle();
		DelaymS(200);	
	}
}

void TestFlash(void)
{	uc i=2;
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
	} 
	else LifeCount=0;
	//����������
	//�Ѿ��궨��д��,��������1����ʾ��������δ�궨
	switch(CALIBRATION_ADD){
		case 0x07://12M
			if(1==RD_EE(0x11)){
				write_criteria();
			}else{
				OpenLED
				while(i--){
					DelaymS(500);
					GCC_CLRWDT();	
				}CloseLED;}
			break;
		case 0x12://17M
			if(1==RD_EE(0x1C)){
				write_criteria();
			}else{
				OpenLED
				while(i--){
					DelaymS(500);
					GCC_CLRWDT();	
				}CloseLED;
			}break;
		case 0x1D://24M
			if(1==RD_EE(0x27)){
				write_criteria();
			}else{
				OpenLED
				while(i--){
					DelaymS(500);
					GCC_CLRWDT();
				}CloseLED;
			}break;
	}
}

void write_criteria(void){//д���о�
		period=RD_EE(CALIBRATION_ADD);
		pulses_per_period=RD_EE(CALIBRATION_ADD+1);
		alarm_period=RD_EE(CALIBRATION_ADD+2);
		continuous_period=RD_EE(CALIBRATION_ADD+3);
		_isgdata0=RD_EE(CALIBRATION_ADD+9);
		total_pause=RD_EE(CALIBRATION_ADD+4);
		total_pause_time = RD_EE(CALIBRATION_ADD+5);
	
		//�ı�ɼ�ֵ�������¸�ѭ���ָ����뿪�ص�������
		sensitivity_value_old=4;		
}
		
void clear_inspection_Array(void){
	//���Ѳ������
	uc i;
	for(i=0; i< inspection; i++){
		inspection_Array[i]=0x00;
	}
	//continuous_period_count=0;			//��������
	period_count=0;
	for(i=0; i< statistics; i++){
		statistics_Array[i]=0x00;
	}	
}

uc Add_Array(volatile uc *p, uc data, uc len){
	//��������ǰ��,ĩβ���data.����len�����ܺ�
	volatile uc i,num;
	for(i=0,num=0; i<len-1; i++){
		p[i]=p[i+1];
		num = num+p[i];
	}
	//ѭ��֮��i=len-1;�����һλ
	p[i] = data;
	num = num + data;
	return num;
}
void Add_gengral(uc data){
	volatile uc i;
	for(i=0; i<19; i++){
		gengral_Array[i]=gengral_Array[i+1];
	}
	gengral_Array[i] = data;
}



void Init_Criterion(void){
	if(1 != RD_EE(0x11)){
		
		WR_EE(CALIBRATION_12M,1);
		WR_EE(CALIBRATION_12M+1,3);
		WR_EE(CALIBRATION_12M+2,3);
		WR_EE(CALIBRATION_12M+3,10);
		WR_EE(CALIBRATION_12M+4,22);
		WR_EE(CALIBRATION_12M+5,20);
		WR_EE(CALIBRATION_12M+9,0x16);
		WR_EE(CALIBRATION_12M+10,1);
	}
	if(1 != RD_EE(0x1c)){
		
		WR_EE(CALIBRATION_17M,1);
		WR_EE(CALIBRATION_17M+1,3);
		WR_EE(CALIBRATION_17M+2,3);
		WR_EE(CALIBRATION_17M+3,10);
		WR_EE(CALIBRATION_17M+4,20);
		WR_EE(CALIBRATION_17M+5,20);
		WR_EE(CALIBRATION_17M+9,0x16);
		WR_EE(CALIBRATION_17M+10,1);
	}
	if(1 != RD_EE(0x27)){
		
		WR_EE(CALIBRATION_24M,1);
		WR_EE(CALIBRATION_24M+1,3);
		WR_EE(CALIBRATION_24M+2,3);
		WR_EE(CALIBRATION_24M+3,10);
		WR_EE(CALIBRATION_24M+4,18);
		WR_EE(CALIBRATION_24M+5,20);
		WR_EE(CALIBRATION_24M+9,0x16);
		WR_EE(CALIBRATION_24M+10,1);
	}	
}
