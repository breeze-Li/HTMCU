//���
//2022��8��26�գ����ڣ�int1�ж�
//2022��8��28�գ�ADC
//2022��8��29�գ��¶ȱ���ѹ���
//2022��8��30�գ�������
//2022��8��31�գ��͹��ģ�9-10UA��,�����������(����ÿ����Ʒ��ͬ������˲����˳�����[�ѽ��])
//2022��9��1�գ�����Ҫ�����Ƶƣ�֮ǰ���Լ�ʱ�����ɿ����ߴӻ��߽���������
//				�����Ǽ�����������ͬһ���߲���ʵ�֣�TB0��������������1UA
//2022��9��29�գ���������ģʽ(5-7UA������ģʽ(1.5)+ldo(3)+��ѹ����(0.5)=5UA��)
//				 [��Ҫ������˯�߼��˾�ʱ������Ȼ�����쳣 -> �Լ�ģʽ�����ſ��Ʒ�����������������ʱ���Ա��֣�����ʱҲ����������������ģʽ��PWM�źſ��ƣ�����ʱ��PWM]
//2022��11��18�գ�����ͨѶЭ�飬�ɱ궨���¶ȼ���ѹ��ֵ
#include "BA45F5440.h"
typedef unsigned char uc;
typedef unsigned int ut;
typedef unsigned long int ul;
volatile uc temp;

//#define  BUZZER
#define  U	'U'
#define  P	'P'
#define  BUZZRE_ON	_pton = 1;
#define  BUZZRE_OFF	{_pton = 0;_pa6 = 1;}
#define  CONNECT_IN  _pa0
#define  CONNECT_OUT _pa5
#define  OpenLED_R    _isgenc |= 0b10000010;	///ISINK1 ���Ź����ʹ��,50mA
#define  CloseLED_R   _isgenc = 0b00000000;	///ISINK1 ���Ź��������
//#define  BT_LOW		0xc2 //7.96
//#define  BT_HIGH 	0xc7//8.24V
//2022��10��27��		
//0xb0-->6.7V(δ�ӵ���)
#define  Def_BT_LOW		0xA7 //����7.2/3V��ʵ��7.1V
#define  Def_BT_HIGH 	0xAA//7.4V
#define  Def_AlarmTmp   59 //Ĭ�ϱ����¶�
#define  TempAdd	1	//
#define  Temp_XOR	2
#define  BT_LOWAdd  3
#define  BT_HIGHAdd 4
#define  BT_XOR 	5

uc BT_LOW = 0;
uc BT_HIGH = 0;
uc AlarmTmp = 0;
uc Battery_AD = 0;
//volatile bit Tb1_flag,Tb0_flag;
struct  {
    uc Tb1_flag :1;		
    uc Tb0_flag :1;	
    uc Interconnect :1;		
    uc Int1_dwflag :1;
    uc Int1_upflag :1;		
    uc Low_battery:1;		
    uc SenErr:1;	//����������	
    uc IsFireFlag:1;//����
    uc ExitFireFlag:1;//Ԥ�˾� 9.29 ����Э�������ߵ��˾�����
    uc Buzzer:1;
    uc Buzzer1:1;	//�������	
    uc Communicating:1;	//����ͨѶ	
}Flags;	//key_status
volatile uc Master;//�������M
volatile uc SenErrCount;//�������
volatile uc TestCnt;	//��������
volatile uc BZ_Count;	//��������
volatile uc BZ_Count1;	//��������
volatile uc Test_Count;	//���Լ���
//ͨѶ���
volatile uc pointer_length;		//ָ�볤��(��¼���ܵ��ֽڳ���)
volatile uc ComRPC;		       	 //��������ָ��
volatile uc ComRCah[15];         //���ջ���
volatile uc transfer_complete_flag;

//0-83��
const uc Chack_AD[84] ={222,221,219,218,216,215,213,211,209,207,206,\
						204,202,200,198,195,193,191,189,187,184,\
						182,180,177,175,172,170,167,165,162,160,\
						157,155,152,149,147,144,142,139,137,134,\
						131,129,126,124,121,119,116,114,111,109,\
						107,104,102,100,97,95,93,91,89,87,\
						85,83,81,79,77,75,73,71,69,68,\
						66,64,63,61,60,58,57,55,54,52,51,50,49};


void DelaymS(ut);
void UART_PTPInit(uc);
void SendByte(uc);
uc RD_EE(uc);
void WR_EE(uc,uc);
void SendWord(unsigned long Word);
void Int1init(void);
void MCUINIT(void);
ut GetADC(uc);
void GET_battery(void);
uc GET_temperature(void);
void Twinkle(void);
void LB_Alarm(void);
void Stm0AInit(void);
void Test_EE(void);
void Com_Management(void);

void main(){
	MCUINIT();
	UART_PTPInit(U);
	Int1init();
	LB_Alarm();
	Stm0AInit();
	Test_EE();
	while(1) {
		/*�����Լ�ͻ�*/
		/*�����ڷ���*/
		/*�ӻ���������*/
		/*ͨѶ*/
		if( Flags.Int1_dwflag==0 &&  Flags.IsFireFlag ==0 && \
			Flags.Buzzer == 0 && \
			Flags.Interconnect ==0 &&\
			Flags.Communicating == 0
			){
			if(Flags.Tb1_flag == 0 || Flags.Tb0_flag ==0)
        		_halt();
        }
 		
        if(Flags.Tb1_flag ) {
            Flags.Tb1_flag = 0;	
			temp = GET_temperature();
			//����ʱ��NTC��С����Ӱ���ѹ��⣬ʹ�¶�>50�㣬
 			//�����е�ѹ��⣬��39S���һ�Ρ�
 			//�¶����ߣ���ѹ�����½����ڵ�ѹ�����һ���ݿɽ����
 			Test_Count++;
			if(!Flags.IsFireFlag && Test_Count > 38 ){
				
				Test_Count =0;
				GET_battery();
//				if(temp < 45){
//					if(Flags.Low_battery) LB_Alarm();
//				}
				if(Flags.Low_battery){
					LB_Alarm();
				}
				else if(Flags.SenErr){
					LB_Alarm();
					DelaymS(100);
					LB_Alarm();
				}else{
					Twinkle();
				}
			}
        }
        //�𾯺��Լ쿪���������ӱ���ֿ���
        //������ã��޷��˳�������(���ӽ�����������)
        //����������жϻ���״̬��ֻ�ܻ���һ�Σ������ڶ��μ������Ϊ�ߺ�͹ص����ߣ�

//    		if( Flags.IsFireFlag || Flags.Int1_dwflag || Flags.Interconnect){
//				_tb0e = 1;	
//				_tb0on = 1;
//				Flags.Buzzer = 1;
//    		}
//				//����������
//				CONNECT_OUT =1;
//				Master = 1;
//			}else if( ){
//				_tb0e = 1;	
//				_tb0on = 1;
//				Flags.Buzzer = 1;
//				//�ӻ���������
//				CONNECT_OUT = 0;
//				Master = 2;
//			}
        if(Flags.Tb0_flag){
        	Flags.Tb0_flag = 0;
        	//��������
        	if(!Master){
        		Flags.Interconnect = CONNECT_IN  ;
	        	if( Flags.IsFireFlag || Flags.Int1_dwflag){
	        		//����������
					CONNECT_OUT =1;
					Master = 1;
	        	}else if(Flags.Interconnect){
	        		//�ӻ���������
					CONNECT_OUT = 0;
					Master = 2;
	        	}
        	}
        	if(Master == 1){
        		//����
        		switch(++BZ_Count){
	        		case 1:BUZZRE_ON; Twinkle(); break ;
	        		case 2:BUZZRE_OFF; break ;
	        		case 3:BUZZRE_ON; Twinkle(); break ;
	        		case 4:BUZZRE_OFF; break ;
	        		case 5:BUZZRE_ON; Twinkle(); break ;
	        		case 6:BUZZRE_OFF; break ;
	        		case 7: break ;
					case 8: BZ_Count=0;
		    				if(Flags.ExitFireFlag && Flags.Int1_upflag){
//		    					_tb0e = 0;	
//								_tb0on = 0;
								Flags.Buzzer = 0;
								CONNECT_OUT = 0;
								Master = 0;
								Flags.Int1_dwflag =0;
								Flags.ExitFireFlag = 0;
								Flags.IsFireFlag = 0;
								} 
							break;
        		}
        	}
        	//�ӻ�
        	if(Master == 2){
        		switch(++BZ_Count){
	        		case 1:BUZZRE_ON; break ;
	        		case 2:BUZZRE_OFF;break ;
	        		case 3:BUZZRE_ON break ;
	        		case 4:BUZZRE_OFF break ;
	        		case 5:BUZZRE_ON break ;
	        		case 6:BUZZRE_OFF break ;
	        		case 7: break ;
					case 8: BZ_Count=0;
		    				if(!Flags.IsFireFlag){
//		    					_tb0e = 0;	
//								_tb0on = 0;
								Flags.Buzzer = 0;
								Master = 0;} 
							break;
        		}
        	}
        }
        		
    GCC_CLRWDT();
	} 
}

void Com_Management(void){
	uc j=0, k=0;						
	transfer_complete_flag=0;
	if(ComRCah[0]==0xcc){	//�궨ͨѶ
		if(pointer_length>=7){
			j=ComRCah[4];
		    for(k=0;k<ComRCah[4];k++)   
            	j=j ^ ComRCah[5+k];
            if(j==ComRCah[ComRCah[4]+5]){
            	switch(ComRCah[5]){
            		case 0x80:
            		   if(ComRCah[4]==1){
               	           SendByte(0xcc);
               	           SendByte(0x99);
               	           SendByte(0x99);
               	           SendByte(0xcc);
               	           SendByte(0x01);
               	           SendByte(0x80);
               	           SendByte(0x81);
                        }
               	        break;
                   	 case 0x89:		//������
                   	 	if(ComRCah[4]==1){
                   	 	   SendByte(0xcc);
               	           SendByte(0x99);
               	           SendByte(0x99);
               	           SendByte(0xcc);
               	           SendByte(0x03);
               	           SendByte(0x86);
               	           SendByte(temp);
               	           SendByte(AlarmTmp);
               	           SendByte(Battery_AD);
               	           SendByte(BT_LOW);
               	           SendByte(BT_HIGH);
               	           j = 0;
               	           if(Flags.IsFireFlag) j |= (1<<7);
               	           if(Flags.Low_battery) j |= (1<<6);
               	           if(Flags.SenErr) j |= (1<<5);
               	           SendByte(j);
                   	 	}
                   	 	break;
                   	 case 0x86:		//д�¶�
                   	 	if(ComRCah[4]==4){
                   	 		WR_EE(TempAdd, ComRCah[6]);
                   	 		WR_EE(Temp_XOR, ComRCah[6] ^ 0xff);
                   	 		WR_EE(BT_LOWAdd, ComRCah[7]);
                   	 		WR_EE(BT_HIGHAdd, ComRCah[8]);
                   	 		WR_EE(BT_XOR, ComRCah[7] ^ ComRCah[8]);
                   	 	   SendByte(0xcc);
               	           SendByte(0x99);
               	           SendByte(0x99);
               	           SendByte(0xcc);
               	           SendByte(0x01);
               	           SendByte(0x86);
               	           SendByte(0x85);
                   	 	}break;
                   	 case 0x87:		//�ָ�Ĭ��
                   	 	if(ComRCah[4]==1){
/*                   	 		WR_EE(TempAdd, ComRCah[6]);*/
                   	 		WR_EE(Temp_XOR,0xff);
//                   	 		WR_EE(BT_LOWAdd, ComRCah[7]);
//                   	 		WR_EE(BT_HIGHAdd, ComRCah[8]);
                   	 		WR_EE(BT_XOR, 255);
                   	 	   SendByte(0xcc);
               	           SendByte(0x99);
               	           SendByte(0x99);
               	           SendByte(0xcc);
               	           SendByte(0x01);
               	           SendByte(0x87);
               	           SendByte(0x86);
                   	 	}break;
            	}
            }
		}
	}
	Test_EE();
	Flags.Communicating = 0;
}


void Test_EE(void){
	if((RD_EE(TempAdd) ^ RD_EE(TempAdd+1)) == 0xff) AlarmTmp = RD_EE(TempAdd);
	else AlarmTmp = Def_AlarmTmp;
	if((RD_EE(BT_LOWAdd) ^ RD_EE(BT_HIGHAdd)) == RD_EE(BT_XOR)){
		BT_LOW  = RD_EE(BT_LOWAdd);
		BT_HIGH = RD_EE(BT_HIGHAdd);
	}
	else{
		BT_LOW  = Def_BT_LOW;
		BT_HIGH = Def_BT_HIGH;
	}
}

void MCUINIT(void){
    _scc = 0x01;
    _hircc = 0x05;
    while(_hircf == 0);
	//----------����--------------------------//
	_mp0 = 0x80;
	_acc = 128;
	while(_acc != 0) {
		_iar0 = 0;
		++_mp0;
		_acc = _acc - 1;
	}
	_mp1l = 0x80;
	_mp1h = 1;
	_acc = 128;
	
	while(_acc != 0) {
		_iar1 = 0;
		++_mp1l;
		_acc = _acc - 1;
	}
	_mp1h = 0;
	_iar1 = 0;
	_mp1l = 128;
	//--------------------------------------//
	/*******��ë��ʼ��********/    
	_wdtc=0x57;
   
    //ʱ����ʼ��
    _pscr = 2;		///32k
    _tb0c = 0x06;
    _tb0e = 0;   //ʱ��0����0.5�붨ʱ
    _tb0e = 1;	
	_tb0on = 1;
    _tb1c = 0x07; //ʱ��1����Ϊ1��
    _tb1e = 1;
	_tb1on=1;
	_emi=1;				//���ж�
	
	//----------ADC����(���)---------------------//
	_pas17 = 1;//pa7->AN1
	_pac7 = 1;//����
	_pas11 = 1;//pa4->AN0
	_pac4 = 1;//����
	//----------ADC����(�¶�)---------------------//
	_pbc4 = 0;
	_pb4 = 1;
	//----------��������--------------------//
	_pac2 = 1;
	_papu2 = 1;//PA2
	_pa2 = 0;
	//��������
	_pac0 = 1;
	_pa0 = 0;
	//�������
	_pac5 = 0;
	_pa5 = 0;
}

void Int1init(void){
	_pac1=1;	//����
	_pa1 = 1;
	_papu1=1;	//����
	_pawu1 = 1;
	/*INT0S1~INT0S0
	00������
	01��������
	10���½���
	11��˫��*/
	_int1s1=1;
	_int1s0=1;	//˫��
	_int1e=1;	//ʹ��int1
}

void __attribute((interrupt(0x0c))) INT1(void){
	//int0�ж�
	_int1f=0;
	if(_pa1==1){
		Flags.Int1_upflag = 1;
	}else{
		Flags.Int1_dwflag = 1;
		Flags.Int1_upflag = 0;
	}
}

void UART_PTPInit(uc i)
{	//���ںͷ��������ų�ʼ��

	//#ifdef BUZZER
		//----------������---------------//
		//�Լ�ģʽ
	//	_pac3 = 1;_pac6 = 0;
	//	_pa3 = 0; _pa6 = 1;
		//����ģʽ
		_pac3 = 0;
		_papu3 = 1;
		_pa3 = 1;
		_pas14 = 1;		//pa6 -> PTP
		_pac6 = 0;
		_pa6 = 1;
		//����Ƭ����
	    _ptmc0 =	0b00000000;	//���÷�Ƶ�� FSYS/4   4Mϵͳʱ��
	    _ptmc1 =	0b10101000;	//PWMͬ�����Ч���
	    _ptmc2 =	0b00000000;
	    _ptmal =	0x9C;
	    _ptmah =	0;	   //DUTY=50%
	    _ptmrpl =	0x39;
	    _ptmrph =	0x01;	//PERIOD=3.2KHz
		
	//	//��ʼ��UART,���������Ļ���ֻ����PA2��rx,��tx	
	//	_umd=1;				//������		
	//	_uucr1=0b10000000;
	//	_uucr2=0b11101100;
	//	_ubrg=25;				//4Mʱ��ʱ������9600
//		_usime=1;				//USIM�ж�ʹ��
//		_pas04 = 1;
//		_papu2=1;			//pa2����
//		_ifs00=0;			//RX ����Դ����ΪPa2
//		//-------------------------------------		
//		_emi=1;				//���ж�
	
		//��ʼ��UART	
		_umd=1;				//������		
		_uucr1=0b10000000;
		_uucr2=0b11101100;
		_ubrg=25;				//4Mʱ��ʱ������9600
		_usime=1;				//USIM�ж�ʹ��
		// --------------HD232-----------------
		_pas07=0;			
		_pas06=1;
		_papu3=1;
		_pac3=0;				//Pa3��TX,���
	//	_pas15=1;			
	//	_pas14=0;			//Pa6��RX
	//	_papu6=1;			//pa6����
	//	_ifs00=1;			//RX ����Դ����ΪPa6
		_pas04 = 1;
		_papu2=1;			//pa2����
		_ifs00=0;			//RX ����Դ����ΪPa2
		//-------------------------------------		
		_emi=1;				//���ж�

}

uc RD_EE(unsigned char RD_EE_addr)
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

void Stm0AInit(void)
{
	//��ʼ��STM0A,���ڴ��ڶ�ʱ
	_stmc0 = 0b01000000;		//TIM0ʱ�� = fusb ; TIM0ʱ�� = 32kHz ;
	_stmc1 = 0b11000001;		//��ʱ��ģʽ���Ƚϼ�����Aƥ����0������(STMA�жϺ�Ͳ��õ������������) ����������һ��Ϊ(1/32k)S
	_stmal = 96;				// 96 2ms��ʱ
	_stmah = 0;				//

	_stmae = 1;		//�Ƚϼ�����Aƥ���ж�ʹ��
	_emi = 1;			//ʹ�����ж�
	//_ston = 0;			//�رն�ʱ��
}
void __attribute((interrupt(0x2c))) STMAIQR(void)
//DEFINE_ISR(STMAIQR, 0x2c)
{	//2ms��ʱ����
	_stmaf=0;						//����жϱ�־
	transfer_complete_flag=1;	//������ɱ�־��1
	pointer_length=ComRPC;		//��¼ָ�볤��
	ComRPC=0;					//����ָ������
	_ston=0;					//�ر�STM0A������
	
	Com_Management();
	
}

void __attribute((interrupt(0x10))) UART(void)
{
	_usimf=0;
	Flags.Communicating = 1;
	//WR_EE(1, temp);
	
	//�����жϺ���
	/*	RxFlag=1;*/
	while(_urxif>0) {
		ComRCah[ComRPC]=_utxr_rxr;
		if(ComRPC<15)
			ComRPC++;
		else
			ComRPC=0;
	}
	//----------------���㶨ʱ��------------
	_ston=0;
	//GCC_DELAY(100);
	_ston=1;
	//--------------------------------------
}

void SendByte(unsigned char l)
{
	//����1���ֽ�
	_utxr_rxr=l;
	while(_utxif==0);//�ȴ����ݴӻ��������ص���λ�Ĵ�����
	while(_utidle==0);//�ȴ����ݷ������
}
void SendWord(unsigned long Word){
	SendByte(Word >> 8);
    SendByte(Word & 0xFF);
}
void Twinkle(void){
	OpenLED_R
	DelaymS(50);
	CloseLED_R
}


void DelaymS(unsigned int t) {
    unsigned int i;
    for(i = 0; i < t; i++)
        GCC_DELAY(1000);
}
/**
 * [__attribute description] 1S��ʱ��
 * @param void [description]
 */
void __attribute((interrupt(0x30))) STB0(void) {
    _tb0f = 0;
    Flags.Tb0_flag = 1;
}
/**
 * [__attribute description]0.5s��ʱ��
 * @param void [description]
 */
void __attribute((interrupt(0x34))) STB1(void) {
    _tb1f = 0;
    Flags.Tb1_flag = 1;
}

unsigned int GetADC(unsigned char c)
{
   ut r=0;
   unsigned long sum=0;		//4ƽ���ã������Դ�
   uc i;
   //uc t1,t2;
   switch(c)
   {
      case 1:
         _sadc0=0b00000001;
         _sadc1=0b00001010;       //AD��λ��AN1���¶�
         break;
      case 2:	
         _sadc0=0b00000000;
         _sadc1=0b00001010;       //AD��λ��AN0.���
   	     break;
   	  default :
//   	   _sadc0=0b00000100;
//         _sadc1=0b00001010;       //AD��λ��AN4���¶�
   	  	 break;
   }
   
   //E3 DE DB D9 D8 D7 D6 D6 ���
   _adcen=1;
   for(i=0;i<4;i++){//4��ƽ��
	   
	   _start=1;
	   _start=0;
	   while(_adbz==1);
//	   SendByte(_sadoh);
//	   SendWord(_sadol);
//	   SendWord(_sadoh<<4);
//	   SendWord(_sadol>>4);
/*����ADRFSΪ��ֵ��_sadohʼ��Ϊ8λ��_sadolʼ��Ϊ��4λ*/
	   //r = (_sadoh<<4) + (_sadol>>4); //12λȫ��ȡ��
	   r = _sadoh;
	   sum += r;
   }
     
   if(c==3){
   		r=_sadoh;
   		_adcen=0;
   		return r;
   }
	
    _adcen=0;
/*    SendWord(sum);*/
    r = sum>>2;
//	SendByte(1);
/*    SendWord(r);*/
    return r;
}

void GET_battery(void){
	//��أ�ʵ��Ϊ3.53��7.79V->C5;9.65V->D7
	//ul tempAD;
	Battery_AD = GetADC(2);
//	tempAD = tempAD * 330;
//	tempAD = tempAD / 255;
//	tempAD = tempAD * 35;//��ѹ
//	SendByte(tempAD >> 8);
//	SendByte(tempAD & 0XFF );
//	SendByte(((tempAD/1000)<<4) + ((tempAD%1000)/100));
//	SendByte((((tempAD%100)/10)<<4) + (tempAD%10));
	
	if(Flags.Low_battery){
		if(Battery_AD > BT_HIGH) Flags.Low_battery=0;
	} else{
		if(Battery_AD < BT_LOW) Flags.Low_battery=1;
	}
	
}
uc GET_temperature(void){
	//�¶�
	ut TempAD;
	uc i,r;
	_pb4 = 0;
	GCC_DELAY(100);
	TempAD = GetADC(1);
	_pb4 = 1;
//	SendByte(TempAD);
//	SendByte(((TempAD/1000)<<4) + ((TempAD%1000)/100));
//	SendByte((((TempAD%100)/10)<<4) + (TempAD%10));
	
	SenErrCount=SenErrCount<<1;
	if(TempAD >= 250 || TempAD <= 5){
		SenErrCount=SenErrCount | 1;	
		if(SenErrCount==255)
			Flags.SenErr=1;               //����������
	}
	//ȡ�¶�
   if(TempAD>Chack_AD[0])   
		r=0;
   else
   {
      if(TempAD<Chack_AD[83])   
         r=83;
      else
      {
         for(i=0;i<84;i++)
         {
            if(TempAD >= Chack_AD[i])
            {
               break;
            }
         }
         //r=((i/10)<<4) + (i%10);
         r=i;
      }
   }

	TestCnt=TestCnt<<1;
	if(r > AlarmTmp)
		TestCnt=TestCnt | 1;
	if((TestCnt & 0x7)==7)
		Flags.IsFireFlag=1;
	if((TestCnt & 0x7)==0)
		Flags.ExitFireFlag=1;
		
	return r;
}
void LB_Alarm(void){//low_battery
	OpenLED_R
	BUZZRE_ON
	DelaymS(50);
	CloseLED_R
	BUZZRE_OFF	
}
