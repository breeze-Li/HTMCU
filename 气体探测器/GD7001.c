//C:\Program Files\Holtek MCU Development Tools\HT-IDE3000V8.x\BIN\HT-IDE3000.EXE
//���
//2021��11��29��:�����������
//2021��12��7��:�¶Ȳ�����ʮλ�˷Ų���
//2022��1��6�գ�����ͨѶ���
//2022��2��11�գ���ȡADֵ��ʹ��12λ�˷ţ�
//2022��3��29�գ�����ͨѶЭ�飨����д��У�飩
//2022��4��7�գ��𾯼��˳�(����)
//2022��4��22�գ�����
//2022��4��29�գ�����(�˽�WiFi�����ϱ��·�ʹ��)
//2022��5��7�գ�WiFi���������ϴ�
//2022��5��13�գ��������
//2022��5��23�գ����ư���(Ӳ������EN1��EN2)
//2022��6��30�գ�ȼ���ٷֱ�ֵ(����ʱʵʱ�ϴ�[�������͵ĿӰ�]���˾���ָ�״̬)
//2022��7��1�գ��Ż��ϴ��·�����
//2022��7��5�գ��Լ칦����ɣ���Ӱ����߼�->�ж��Ƿ񳤰���������Լ�󴥷��̰������⡿

/*[�����]:
������ź�ǿ�ȡ�[���]
���Լ�ʱ�Ĺ�����ʾ��[������ֹͣ��ʱ��̧��ֹͣ���жϹ��ϼ��Լ��Ƿ�ɹ���������ֵ��Ϊ4��]
���͹���ģʽ���ϵ�WiFi�����߼���
��������ʱ��������е�ָʾ��Ϣ��
*/

#include "BA45F5250.h"
/*#include <stdilb.h>*/
typedef unsigned char uc;
typedef unsigned int ut;
typedef unsigned long int ul;

#define  EN1 _pb4
#define  EN2 _pa7
#define  TEMPIO _pb2		//�¶Ȳ�����������
#define  R 'R'
#define  G 'G'
#define  Y 'Y'
#define  set_warm_up_time 1	//Ԥ��ʱ��
#define  Hz_L 0X158			//2.9K
#define  Hz_M 0X14D			//3.0K
#define  Hz_H 0X142			//3.1K
//345, 341, 338, 334, 330, 327, 323, 320, 316, 313

#define  WIFI_COM			//����WiFiͨѶ


//----------WiFiָ��----------------------//
#define  test      1
#define  ch4_state 2
#define  ch4_value 5
#define  life      11
#define  fault     45
#define  mute      14
#define  signal    101

#define  MyType 9
#define  MyVer  0x10
//#define  ABS(X) ((X)>=0)?(X):(-(X))

#define  OpenLED_R    _isgenc |= 0b10000010;	///ISINK1 ���Ź����ʹ��,50mA
#define  OpenLED_G    _isgenc |= 0b10000001;	///ISINK0 ���Ź����ʹ��,50mA
#define  CloseLED_R   _isgenc &= 0b10000001;	///ISINK1 ���Ź��������
#define  CloseLED_G   _isgenc &= 0b10000010;	///ISINK0 ���Ź��������
#define  ALARM_OUT 	  {_papu0 =1;_pa0 =1;}

volatile bit  Tb0_flg;
volatile bit  Tb1_flg;
//volatile bit  RxFlag;       				//�����������ݱ�־
volatile bit  transfer_complete_flag;		//���մ�����ɱ�־
volatile bit  MatchErr;			//У׼����
volatile bit  BaseErr;			//��ֵ����
volatile bit  FireErr;
volatile bit  FullErr;
volatile bit  SenShort;
volatile bit  SenOpen;
volatile bit  IsFire;
volatile bit  Buzzerflag;
//volatile bit  tamperflag;
volatile bit  selftestflag;
volatile bit  Int0flag; 
volatile bit  Timer1Flag;
volatile uc ComPC;		   	     //��������ָ��
volatile uc ComCah[54];          //���ͻ�����
volatile uc ComRPC;		       	 //��������ָ��
volatile uc ComRCah[24];         //���ջ���
volatile uc MyID[3];
volatile uc pointer_length;		//ָ�볤��(��¼���ܵ��ֽڳ���)

struct  {
    ut current_OPAvalue;	//��ǰ�˷�ֵ
    ut current_refvalue;	//��ǰ��λ��ֵ
    ut Base_value;     		// ��ʼֵ
    ut Fire_value;     		// �𾯱궨ֵ
    ut Full_value;     		// 100%LEL
}Valves={0,0,0,0,0};

struct  {
    uc First_blood :1;		// ��1���¼�
    uc Double_kill :1;		// 2���¼�
    uc Triple_kill :1;		// 3���¼�
    uc Unstoppable :1;		// �����¼�
    uc double_flag :1;		// ˫����־
    uc long_flag:1;			// ������־
    uc long_flaged :1;		// ����������־
    uc long_to_up :1;		// ������̧���־
}Key_S={0,0,0,0,0,0,0,0};	//key_status

struct {
    uc check_result :2;	//�Լ�״̬ 	0�Լ���1�ɹ�2ʧ��3�����Լ�
    uc gas_state :1;	//ȼ��״̬	0δ����1����
    uc lifeOver :1;  	//���� 		0����  1����ʹ��
    uc muteflag :1;  	//����		0δ����1����
    uc faultflag :3;  	//����ֵ 	0�޹���  1��·2��·4�궨����
	uc fult_muteflag :1;  	//���Ͼ���		0δ����1����
	uc :7;
}ps={0,0,0,0,0,0,};//product_status
					
volatile ut OpAveVal[2] = {0};//�˷�ƽ��ֵ
volatile ut RefAveVal[2]= {0};//��׼ƽ��ֵ
volatile uc BaseCount =0;
volatile uc ShortCount =0;
volatile uc OpenCount =0;
volatile uc FireOutTimer=0;			//�𾯱��ּ���
volatile ut LifeCount=0;
volatile uc FireCount = 0;
volatile uc Fire_OutFireCount=0;	//�˳��𾯼���
volatile uc alarmcount=0;			//������������
volatile uc TempAD,CurTemp;         //�¶Ȳ���
//volatile uc OP1,OP0;
//volatile uc sadoh,sadol;
volatile uc warm_up_time=0;			//Ԥ��ʱ��
volatile uc lightcount=0;
volatile uc time_3S = 0;			//3S��ʱ
volatile uc time_30S = 0;

//----------wifi���-------------------//
volatile bit  SetupNetState;  	 //���������ɹ�
volatile uc ZigBeeNetState;  	  //����״̬
volatile uc  ZigBeeVer;       	 //Э��汾�ţ�
volatile uc signal_strength;		//�ź�ǿ��
//PID
const uc WifiPID[37]={"{\"p\":\"bmsojo0y3pmh7qwj\",\"v\":\"1.0.0\"}"};
//-15�ȵ�55��
const uc TMP[71]={30,32,34,35,37,39,
                 41,43,45,47,49,51,53,55,58,60,
                 62,65,67,70,72,75,77,80,83,86,
                 88,91,94,97,100,102,105,108,111,114,
                 117,120,122,125,128,131,133,136,139,141,
                 145,147,150,152,155,157,160,162,164,167,
                 170,172,174,176,178,180,182,184,186,188,
                 190,192,194,196,198};

//-----------�洢��ַ����---------------------------//
#define Base_valueAdd 0x00		//��ֵ
#define Fire_valueAdd 0x03		//������
#define Full_valueAdd 0x06		//����
#define OPA_multipleAdd 0x09
#define IDAdd 0x10          	//�豸ID  H+M+L+Xor
#define LifeHAdd 0x14	    	//����ʱ�䵥λ12Сʱ H+L+Xor
#define SetupNetStateAdd 0x15	//��������

void Stm0AInit(void);
void Stm1Init(void);
void UARTInit(void);
void OPA_int(void);
void INT0Init(void);
void S_ADJ_OPA(void );
void S_ADJ_OPA1(void );
void SendByte(uc l);
void sendstring(char *p);
void SendWord(unsigned long Word);
void SendData(uc L);
void NBSendData(unsigned char l);
uc RD_EE(uc RD_EE_addr);
void Read_EE(uc , uc *buff, uc L);
void WR_EE(uc ,uc);
void Write_EE(uc ,uc ,uc );
void DelaymS(ut t);
void Beep(uc t);
void Beeps(uc t);
void BuzzerOn(uc);
void Twinkle(uc);
uc GetFault(void);
int Getch4_value(void);
ut GetADC(uc c);
void get_temperature(void);
uc GetTemp(void);
void Com_Management(void);
void wifi_upload(uc i);
void SetPack(void);
void TestFlash(void);
void function(void);
void keyDetect(void);




void main() {
    unsigned char i;

    _scc = 0x01;
    _hircc = 0x05;
    while(_hircf == 0);

	ComRPC=0;
	ComPC=0;
	pointer_length=0;
	transfer_complete_flag=0;
	Tb1_flg=0;
	Tb0_flg=0;
	//RxFlag=0;
	SenShort =0;
	SenOpen =0;
	MatchErr=0;
	IsFire =0;
	Buzzerflag = 0;
	selftestflag=0;
	Int0flag = 0;
	Timer1Flag = 0;

    _mp0 = 0x80;
    _acc = 128;

    while(_acc != 0) {
        _iar0 = 0;
        ++_mp0;
        _acc = _acc - 1;
    }

    for(i = 1; i < 8; i++) {
        _mp1l = 0x80;
        _mp1h = i;
        _acc = 128;

        while(_acc != 0) {
            _iar1 = 0;
            ++_mp1l;
            _acc = _acc - 1;
        }
    }

    _mp1h = 0;
    _iar1 = 0;
    _mp1l = 128;
    
   /*******������ʼ��********/
		
   
    _wdtc=0x57;
    
    _pbc2=0;
    TEMPIO=1;			//�¶Ȳ�����������
    ps.check_result = 3;
	ps.lifeOver     = 1;
    
    //----------ADC����(�¶�)---------------------//
    _pbc5=1;
	_pbs13=1;
	_pbs12=1;
	//----------�����������---------------------//
	_pac0=0;
	_pa0=0;
	
	_pa7=0;
	_pb4=0;
	_pa2=0;
		
    //��ʼ��ʱ��
    _pscr = 2;		///32k
    _tb0c = 0x07;
    _tb0e = 1;   //ʱ��0����1�붨ʱ
    _tb1c = 0x06; //ʱ��1����Ϊ0.5��
    _tb1e = 1;

    //��ʼ��PWM
    _pas15=0;
    _pas14=1;	//PA6ΪPTP
    _pac6 = 0;	//pa6���
    _pac7 = 0;
    _pbc4 = 0;
    _ptmc0 =	0b00000000;	//���÷�Ƶ�� FSYS/4   4Mϵͳʱ��
    _ptmc1 =	0b10101000;
    _ptmc2 =	0b00000000;
    _ptmal =	0x9C;
    _ptmah =	0;	   //DUTY=50%
    _ptmrpl =	0x39;
    _ptmrph =	0x01;	//PERIOD=3.2KHz
    
   //��ʼ����ʱ��1 ���� 960/32000=30mS
   _stm1c0=0b01000000;      //fsys=32K��
   _stm1c1=0b11000001;
   _stm1al=0xC0;
   _stm1ah=3;
   _stm1ae=1;
   _st1on=0;
    

    //��ʱ��0
    _tb0on = 1;
    _tb1on = 1;
    _emi = 1;
	
	UARTInit();
	Stm0AInit();
	INT0Init();
	TestFlash();
	
	OPA_int();
	DelaymS(20);
	S_ADJ_OPA();
	S_ADJ_OPA1();
//	SendByte(sizeof(char));				//1
//	SendByte(sizeof(short int));		//2
//	SendByte(sizeof(int));				//2
//	SendByte(sizeof(long));				//4

//Twinkle(G);
//DelaymS(100);
//Twinkle(G);

//volatile ut count=0; 
//volatile ut count1=0;

//_isgdata1=0;
//OpenLED_G
//OpenLED_R
//while(1)
//{
//	ut m1,m2,m3;
//	m1=-1;m2=-2;m3=-100;
//	SendByte(m1);
//	SendByte(m2);
//	SendByte(m3);
//	SendByte(m3 / m2);
//	SendByte(m3 / 2);
//	DelaymS(1000);
//    GCC_CLRWDT();
//};
    while(1) {
//        if(Tb1_flg) {
//            Tb1_flg = 0;
//        }
			
            if(Timer1Flag){
            	//��������
            	Timer1Flag=0;
            	keyDetect();	
            }
			if(Key_S.First_blood){
	            	Key_S.First_blood=0;
#ifdef VoiceTest
	            	//----------��������------------------------//
	            	if(!IsFire){
		            	BaseErr=1;
				        IsFire=1;
				        Buzzerflag = 1;
			        }
			        else{
			        	BaseErr=0;
			            IsFire=0;
			        }
			        //******************************************/
#endif
	            	ps.muteflag = !ps.muteflag;
	            	if(ps.muteflag){
	            		Beeps(1);
	            	}
	            	else {
	            		Beeps(2);
	            	}
	            	wifi_upload(mute);
	            	DelaymS(50);
	            	wifi_upload(ch4_value);
	            	DelaymS(50);
	            	//��ȡ�ź�ǿ��
	            	ComCah[0]=0x55;
					ComCah[1]=0xaa;
					ComCah[2]=0;
					ComCah[3]=11;
					ComCah[4]=0;
					ComCah[5]=0;
                	NBSendData(6);
                	
	        }
            if(Key_S.Double_kill){
            	Key_S.Double_kill=0; 
            	ps.fult_muteflag = !ps.fult_muteflag;
            	SendByte(ps.fult_muteflag);
            	if(ps.fult_muteflag){
            		Beeps(1);
            	}
            	else {
            		Beeps(2);
            	}	
            }
			
			if(Key_S.Triple_kill){
				Key_S.Triple_kill=0;
				ComCah[0]=0x55;
				ComCah[1]=0xaa;
				ComCah[2]=0;
				ComCah[3]=4;
				ComCah[4]=0;
				ComCah[5]=1;
				ComCah[6]=0;
            	NBSendData(7);
            	Beep(80);
            }
            
            if(Key_S.Unstoppable){
				
				//Key_S.long_flag����ֻ��һ���ж�
				Key_S.long_flaged = 1;	//��������
				if(Key_S.long_flag){
					//Key_S.long_flag = 0;	
					ps.check_result = 0;	//�Լ���
					wifi_upload(0);
				}
				
				ps.faultflag = GetFault();
				if(ps.faultflag){	//����
					ps.check_result = 2;	//ʧ��
					wifi_upload(0);
					Twinkle(Y);
    				Beep(80);
    				DelaymS(100);
    				Twinkle(Y);
    				Beep(80);
				}else if(IsFire && (ps.check_result ==0)){
					ps.check_result = 1;	//�ɹ�
					Beep(250);
					Twinkle(G);
					wifi_upload(0);
				}
				Key_S.Unstoppable=0;
            }
            
			if(Key_S.long_to_up){
				Key_S.long_to_up=0;
				Key_S.Unstoppable=0;
				Key_S.long_flag = 0;
				SendByte(0xfa);
            }
			if(Tb0_flg){
				Tb0_flg=0;
			    GCC_CLRWDT();
//				get_temperature();
//				CurTemp=GetTemp();
//				SendByte(TempAD);
//				SendByte(((CurTemp/10)<<4)+(CurTemp%10));
/*				DelaymS(10);*/
//				SendByte(Key_S.First_blood);
//				SendByte(Key_S.Double_kill);
//				SendByte(Key_S.Triple_kill);
//				SendByte(Key_S.Unstoppable);
//				SendByte(Key_S.double_flag);
//				SendByte(Key_S.long_flag);
//				SendByte(Key_S.long_flaged);
//				SendByte(Key_S.long_to_up);
//				SendByte(srand(1)%100);
//				SendByte(srand(2)%100);
//				sendstring("hello world");
				function();
				

				
//				if((++lightcount > 4) && (IsFire==0)) {
//					lightcount=0;Twinkle(G);
				}

			if(Int0flag){
//				Int0flag=0;
//				_isgenc ^= 0b00000010;
//				_pb2 ^= 1;
//				SendByte(9);
				
			}
			if(transfer_complete_flag) Com_Management();
        	GCC_CLRWDT();
    }
}

void function(void){
	unsigned int sum, a;
	
	a = GetADC(2);
	Valves.current_OPAvalue = a;
	//----------ȡƽ��ֵ------------------------//
	sum = (OpAveVal[0] << 2) + OpAveVal[1];  ///������һ�ε������λ��
    sum = sum - OpAveVal[0];
    sum = sum + a;
    OpAveVal[0] = (sum >> 2); 	
    OpAveVal[1] = (sum & 3);  	///����ȡ�����λ(��Ϣ�ʹ����ֵƽ���仯)
//	SendWord(a);
//	SendWord(OpAveVal[0]);

    a = GetADC(1); 				//AN3 ��׼(����������)
    Valves.current_refvalue = a;
    sum = (RefAveVal[0] << 2) + RefAveVal[1];
    sum = sum - RefAveVal[0];
    sum = sum + a;
    RefAveVal[0] = (sum >> 2); //4ƽ��
    RefAveVal[1] = (sum & 3);
//    SendWord(a);
//    SendWord(RefAveVal[0]);
	if(warm_up_time > set_warm_up_time) {        ///Ԥ�����
		if(RefAveVal[0] > 1901 && RefAveVal[0] < 2194){		//1.3V<AN3<1.5V   ��Դ��ѹ2.8V
			
			//�жϿ�·����·���ж�ǰ�߼����ƣ����ڲ���������ʱ�Զ�����
            a = 0;
            BaseCount <<= 1;
            ShortCount <<= 1;
            OpenCount <<= 1;
            FireCount <<= 1;
            Fire_OutFireCount <<= 1;
			
			//----------���е��迪·����·�ж�--------------//
			
			if(OpAveVal[0] > (RefAveVal[0] + 1635)){ //AN0>AN3+1.11V{
                a = 1;
                OpenCount = OpenCount | 1;
            }
            if(OpAveVal[0] < (RefAveVal[0] - 731)){ //AN0<AN3-0.5V{
                a = 1;
                ShortCount = ShortCount | 1;
            }  
        }
        else{
        	a = 1;
            BaseCount = BaseCount << 1;
            BaseCount = BaseCount | 1; // У׼����
        }
        if(ShortCount == 0xff) SenShort = 1;

        if(ShortCount == 0	 ) SenShort = 0;

        if(OpenCount == 0xff ) SenOpen = 1;

        if(OpenCount == 0	 ) SenOpen = 0;

        if(BaseCount == 0xff ) MatchErr = 1;

        if(BaseCount == 0	 ) MatchErr = 0;
        
        if(!a){
			if( !FireErr && !FullErr && !BaseErr){
				///����ֵ���ڵ��� ��׼ֵ+������ ->10%�ģ���ը���ޣ�
				if(OpAveVal[0] > RefAveVal[0] + Valves.Fire_value){
					FireCount = FireCount | 1;	
				}
				///����ֵС�ڵ��� ��׼ֵ+������/2->10%�ı궨����2��5%LEL����ը���ޣ�
				else if(OpAveVal[0] < RefAveVal[0] + (Valves.Fire_value >> 1)){
					Fire_OutFireCount |= 1;
				}
				
				if(IsFire==0){
					if(FireCount == 0xFF) {
						IsFire=1; 
						//�Լ�ʱ���죬Ҫ��ʾ����
						if(!Key_S.long_flag) Buzzerflag = 1;
						wifi_upload(ch4_state);
					}
				}
				else if(Fire_OutFireCount == 0xFF){
					IsFire=0;
					wifi_upload(ch4_state); //������֮�����״̬
				}
			}
        }
        
        if((++time_3S > 2)&& !IsFire && !Buzzerflag){
        	time_3S=0;
        	Twinkle(G);
        	}
        	
        if(++time_30S >= 30){
        	time_30S=0;
        	if((!IsFire) && (!ps.fult_muteflag))
        	{
        		if(SenOpen || SenShort || FireErr ||\
        		   FullErr || BaseErr  || MatchErr )
        		{
        			Twinkle(Y);
        			Beep(100);
        		}
    			else if (!ps.lifeOver){
    				Twinkle(Y);
    				Beep(100);
    				DelaymS(100);
    				Twinkle(Y);
    				Beep(100);
    			}
        	}
        }
	}
	else
	{
		warm_up_time++;
        Twinkle(G);
	}
//	SendByte(FireCount);
//	SendByte(Fire_OutFireCount);
//	SendByte(IsFire);
//	SendByte(alarmcount);
	
	//---------�����������ƣ�buzzerflag������������------//
	if(Buzzerflag ){
		//SendByte(alarmcount);
    	switch(++alarmcount) {
	        case 1:
	        	if(!ps.muteflag)
	            	BuzzerOn(1);
	            OpenLED_R
	            break;
	        case 2:
		        BuzzerOn(0);
	            CloseLED_R
	            break;
	        case 3:
		        if(!ps.muteflag)
		            BuzzerOn(2);
	            OpenLED_R
	            break;
	        case 4:
				BuzzerOn(0);
	            CloseLED_R
	            break;
	        case 5:
		        if(!ps.muteflag)
		            BuzzerOn(3);
	            OpenLED_R
	            break;
	        case 6:
	            BuzzerOn(0);
	            CloseLED_R
	            break;
	        case 7:
	            alarmcount = 0;
	            if(!IsFire) {//�˳��𾯣�����״̬
	            	Buzzerflag=0;
	            	//wifi_upload(ch4_state);
	            }
	            break;
	    }
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
            			if(ComRCah[4]==1)
                            {
                   	           SetPack();
			                   ComCah[4]=1;
			                   ComCah[5]=0x80;
			                   ComCah[6]=0x81;
			                   SendData(7);
                            }
                   	        break;
            		case 0x81:		//дID
            			if(ComRCah[4]==4)
                            {
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
                   	case 0x82:   //�����Ա�����
                            if(ComRCah[4]==1)
                            {                            
                   	           SetPack();
			                   ComCah[4]=1;
			                   ComCah[5]=0x82;
			                   ComCah[6]=0x83;
			                   SendData(7);
			                   BaseErr=1;
			                   IsFire=1;
			                   Buzzerflag = 1;
                            }
                   	        break;
                   	case 0x83:   //�رղ��Ա�����
                            if(ComRCah[4]==1){
                   	           SetPack();
			                   ComCah[4]=1;
			                   ComCah[5]=0x83;
			                   ComCah[6]=0x82;
			                   SendData(7);
			                   BaseErr=0;
			                   IsFire=0;
                            }
                   	        break;
            		case 0x84:		// ID
            			if(ComRCah[4]==1)
                            {
                   	           SetPack();
			                   ComCah[4]=4;
		                       ComCah[5]=0x84;
			                   ComCah[6]=MyID[0];
			                   ComCah[7]=MyID[1];
			                   ComCah[8]=MyID[2];
			                   ComCah[9]=ComCah[8] ^ ComCah[7] ^ ComCah[6] ^ 0x80;
			                   SendData(10);
                            }
                   	        break;
					case 0x85:		//��̽��������
						if(ComRCah[4]==1){
               	           SetPack();
		                   ComCah[4] = 8;
	                       ComCah[5] = 0x85;
		                   ComCah[6] = Valves.Base_value >> 8;
		                   ComCah[7] = Valves.Base_value & 0xFF;
		                   ComCah[8] = Valves.Fire_value >> 8;
		                   ComCah[9] = Valves.Fire_value & 0xFF;
		                   ComCah[10] = Valves.Full_value >> 8;
		                   ComCah[11] = Valves.Full_value & 0xFF;
		                   ComCah[12] = RD_EE(OPA_multipleAdd);
		                   ComCah[13] = 0x82^ComCah[6]^ComCah[7]^ComCah[8]^\
		                   				ComCah[9]^ComCah[10]^ComCah[11]^ComCah[12];//82=85^7
		                   SendData(14);
                        }
                   	    break;
					case 0x86:		//д����
						if(ComRCah[4]==8){
							ComCah[1] = ComRCah[6];
			                ComCah[2] = ComRCah[7];
			                ComCah[3] = ComRCah[6] ^ ComRCah[7];
			                ComCah[4] = ComRCah[8];
			                ComCah[5] = ComRCah[9];
			                ComCah[6] = ComRCah[8] ^ ComRCah[9];
			                ComCah[7] = ComRCah[10];
			                ComCah[8] = ComRCah[11];
			                ComCah[9] = ComRCah[10] ^ ComRCah[11];
			                ComCah[10] = ComRCah[12];
							ComCah[11] = ComRCah[12] ^ 0xFF;
							Write_EE(Base_valueAdd, 1, 11);
							TestFlash();
							SetPack();
							ComCah[4] = 1;
							ComCah[5] = 0x86;
							ComCah[6] = 0x87;
							SendData(7);
							}
						break;
					case 0x87:		//������
						if(ComRCah[4]==1){
							SetPack();
							ComCah[4] = 3;
							ComCah[5] = 0x87;
							ComCah[6] = LifeCount >> 8;
			                ComCah[7] = LifeCount & 0xFF;
			                ComCah[8] = 3 ^ 0x87 ^ ComCah[6] ^ ComCah[7];
							SendData(9);
						}
						break;
					case 0x88:		//д����
						if(ComRCah[4]==3){
							LifeCount = (ComRCah[6]<<8) + ComRCah[7];
							ComCah[4] = ComRCah[6];
			                ComCah[5] = ComRCah[7];
			                ComCah[6] = ComRCah[6] ^ ComRCah[7];
			                Write_EE(LifeHAdd, 4, 3);
			                TestFlash();
			                SetPack();
			                ComCah[4] = 1;
			                ComCah[5] = 0x88;
			                ComCah[6] = 0x89;
			                SendData(7);	
						}
						break;
					case 0x89:		//������
						if(ComRCah[4]==1){
							SetPack();
							ComCah[4] = 12;
			                ComCah[5] = 0x89;
			                ComCah[6] = MyType;
							ComCah[7] = MyVer;
							ComCah[8] = OpAveVal[0] >> 8;
							ComCah[9] = OpAveVal[0] & 0xFF;
							ComCah[10] = RefAveVal[0] >> 8;
							ComCah[11] = RefAveVal[0] & 0xFF;
							ComCah[12] = Valves.current_OPAvalue >> 8;
							ComCah[13] = Valves.current_OPAvalue & 0xFF;
							ComCah[14] = Valves.current_refvalue >> 8;
							ComCah[15] = Valves.current_refvalue & 0xFF;
							ComCah[16] = 0;
							if(SenShort) ComCah[ 16] = ComCah[ 16] | 1;
			                if(SenOpen)  ComCah[ 16] = ComCah[ 16] | 2;
			                if(!ps.lifeOver) ComCah[ 16] = ComCah[ 16] | 4;
			                if(MatchErr) ComCah[ 16] = ComCah[ 16] | 8;
			                if(BaseErr) ComCah[ 16] = ComCah[ 16] | 16;
			                if(FireErr) ComCah[ 16] = ComCah[ 16] | 32;
			                if(FullErr) ComCah[ 16] = ComCah[ 16] | 64;
			                if(IsFire)  ComCah[ 16] = ComCah[ 16] | 128;
							ComCah[17] =  ComCah[4] ^ ComCah[5] ^ ComCah[6] \
										^ ComCah[7] ^ ComCah[8] ^ ComCah[9] \
										^ ComCah[10] ^ ComCah[11] ^	ComCah[12] \
										^ ComCah[13] ^ ComCah[14] ^	ComCah[15] \
										^ ComCah[16];	
							SendData(18);
						}
						break;
					case 0x97:
						SendWord(Valves.Base_value);
						SendWord(Valves.Fire_value);
						SendWord(Valves.Full_value);
						break;
					case 0x98:
						//wifi_upload(0);
						if(ComRCah[6] == 0){
							if(ComRCah[7] == 0) CloseLED_G
							else OpenLED_G;
						}
						if(ComRCah[6] == 1){
							if(ComRCah[7] == 0) CloseLED_R
							else OpenLED_R;
						}
						SendByte(_isgdata0);
						SendByte(_isgdata1);
						break;
#if 0	
					case 0x99:

					SendByte(ComRCah[6]);
					SendByte(ComRCah[7]);
					Percentage1 = ComRCah[6];
					Percentage1 <<= 8;
					Percentage1 += ComRCah[7];
					SendWord(Percentage1);
					Percentage1 *= 100;
					SendWord(Percentage1);
					Percentage = Percentage1 / 1122;
					SendWord(Percentage);
					if(Percentage1 < 0) {	//Percentage1<0,Percentage<0
						Percentage1 = 0;
						Percentage = 0;
					};
					if((Percentage > 0) && (Percentage < 1000)){
						//return Percentage;
					}else{
						Percentage = 1000;
					}
					DelaymS(100);
						wifi_upload(ch4_value);
#endif
//					case 0x88:
//					case 0x89:
//					case 0x90:
//					case 0x91:

            	}
            }
		}
	}
#ifdef WIFI_COM
	else{		//WiFiͨѶ
		//ͷУ��
		if(ComRCah[0] == 0x55 && ComRCah[1] == 0xAA){
			//SendByte(1);SendByte(pointer_length);SendByte(ComRCah[5]);
			//����У��
			if(pointer_length == (ComRCah[5]+7)){//SendByte(2);
				//��У��
				for(k=0; k < (pointer_length-1); k++)   
            		j=j + ComRCah[k];
            	if(j == ComRCah[pointer_length-1]){//SendByte(3);
            		ZigBeeVer=ComRCah[2];
            		//ѡ��������
            		switch(ComRCah[3]){
	        			case 1:		//��ѯ��Ʒ��Ϣ
	        				ComCah[0]=0x55;
	       	  	             ComCah[1]=0xaa;
	       	  	             ComCah[2]=ZigBeeVer;
	       	  	       	     ComCah[3]=1;
	       	  	       	     ComCah[4]=0;
	       	  	       	     ComCah[5]=36;
	       	  	       	     for(j=0;j<36;j++)
	       	  	       	        ComCah[6+j]=WifiPID[j];
	       	  	       	     NBSendData(42);
	       	  	       	     break;
	       	  	       	case 2:   //֪ͨ����״̬
	       	  	       	     ZigBeeNetState=ComRCah[6];
	       	  	       	     ComCah[0]=0x55;
	       	  	             ComCah[1]=0xaa;
	       	  	             ComCah[2]=ZigBeeVer;
	       	  	       	     ComCah[3]=2;
	       	  	       	     ComCah[4]=0;
	       	  	       	     ComCah[5]=0;
	       	  	       	     NBSendData(6);
	       	  	       	     if(ZigBeeNetState==4)   //�Ѿ����ӵ��ƶ�
	       	  	       	     {   //�����ɹ���ʾ
	       	  	       	        if(SetupNetState==0)
	       	  	       	        {
	       	  	       	           SetupNetState=1;
	       	  	       	           //�洢״̬
	       	  	       	           WR_EE(SetupNetStateAdd,1);
	                   	           WR_EE(SetupNetStateAdd+1,0xfe);
	                   	           Twinkle(R);
	       	     	               Beep(150);
	       	     	               DelaymS(100);
	       	                       Twinkle(R);
	       	     	               Beep(150);
	       	  	       	        }
	       	  	       	        DelaymS(3000);
	       	  	       	        wifi_upload(0);
	       	  	       	     }
       	  	       	     	break;
       	  	       	     case 3:    //��������
	       	  	       	 case 4:	//AP����
	       	  	       	     //ɾ��һ��ָ��
	//       	  	       	     ZB_SubFun=3;  //�ȴ������ɹ�
	//       	  	       	     ZB_TestTimer=0;     //���Լ�ʱ��
	//                         ZB_ComCount=0;
	       	  	       	     //ɾ��������־
	       	  	       	     WR_EE(SetupNetStateAdd,0);
				             WR_EE(SetupNetStateAdd+1,0);
				             SetupNetState=0;
	       	  	       	     break;
	       	  	       	 case 9:	//�·�
	       	  	       	 	 if(ComRCah[6] == 14){
	       	  	       	 	 	ps.muteflag = ComRCah[10];
	       	  	       	 	 	ComCah[0]=0x55;
								ComCah[1]=0xaa;
								ComCah[2]=14;
								ComCah[3]=9;
								ComCah[4]=0;
								ComCah[5]=0;
								NBSendData(6);
								DelaymS(50);//����ָ��
								wifi_upload(mute);//����APP״̬
	       	  	       	 	 }break;
	       	  	       	 case 11:	//�ź�ǿ��
	       	  	       	 	 if(ComRCah[6] == 1){
	       	  	       	 	 	signal_strength = ComRCah[7];
	       	  	       	 	 }else{
	       	  	       	 	 	signal_strength = -1;
	       	  	       	 	 }
	       	  	       	 	 DelaymS(50);
							 wifi_upload(signal);//����APP״̬
							 break;
            		}
            	}
			}
		}
	}
#endif
}

void wifi_upload(uc i){
	int Percentage;
	ComCah[0]=0x55;
	ComCah[1]=0xaa;
	ComCah[2]=ZigBeeVer;
	ComCah[3]=5;
	ComCah[4]=0;
	switch (i){
		case test://�Լ� test
			ComCah[5]=5;
			ComCah[6]=1;
			ComCah[7]=4;
			ComCah[8]=0;
			ComCah[9]=1;
			ComCah[10] = ps.check_result;
			NBSendData(11);
			break;
		case ch4_state:
			ComCah[5]=5;
			ComCah[6]=2;
			ComCah[7]=4;
			ComCah[8]=0;
			ComCah[9]=1;
			ComCah[10] = IsFire ? 1:0;
			NBSendData(11);break;
		case ch4_value://ȼ��ֵch4_value���ٷֱȣ�
			ComCah[5]=8;
			ComCah[6]=ch4_value;
			ComCah[7]=2;
			ComCah[8]=0;
			ComCah[9]=4;
			//ֻ�õ�λ��1000����
			ComCah[10]=0;
			ComCah[11]=0;
			Percentage = Getch4_value();
			ComCah[12]=Percentage >> 8;
			ComCah[13]=Percentage & 0xff;
			NBSendData(14);
			break;
		case signal://�ź�ǿ��
			ComCah[5]=8;
			ComCah[6]=signal;
			ComCah[7]=2;
			ComCah[8]=0;
			ComCah[9]=4;
			//ֻ�õ�λ��100����
			ComCah[10]=0;
			ComCah[11]=0;
			ComCah[12]=0;
			ComCah[13]=signal_strength;
			NBSendData(14);
			break;
		case life://����������Ϊ1������Ϊ0��life
			ComCah[5]=5;
			ComCah[6]=11;
			ComCah[7]=1;
			ComCah[8]=0;
			ComCah[9]=1;
			ComCah[10]=ps.lifeOver;
			NBSendData(11);break;
		case mute://���� mute
			ComCah[5]=5;
			ComCah[6]=14;
			ComCah[7]=1;
			ComCah[8]=0;
			ComCah[9]=1;
			ComCah[10]=ps.muteflag;
			NBSendData(11);break;
		case fault:
			//���ϱ�־��λ����->0b00000 �궨 У׼ ��������fault
			ComCah[5]=5;
			ComCah[6]=45;
			ComCah[7]=5;
			ComCah[8]=0;
			ComCah[9]=1;
			ComCah[10] = GetFault();
			NBSendData(11);
			break;
		default:			
			//�Լ�
			ComCah[5] = 33;	//������ ���ָ���ӳ���
			//ComCah[6] = 5;
			ComCah[6]  = 1;
			ComCah[7]  = 4;
			ComCah[8]  = 0;
			ComCah[9] = 1;
			ComCah[10] = ps.check_result;
			//ȼ��ֵ
			//ComCah[11] = 8;
			ComCah[11] = 5;
			ComCah[12] = 2;
			ComCah[13] = 0;
			ComCah[14] = 4;
			ComCah[15] = 0;
			ComCah[16] = 0;
			Percentage = Getch4_value();
			ComCah[17] = Percentage >> 8;
			ComCah[18] = Percentage & 0xff;
			//����
			//ComCah[20] = 5;
			ComCah[19] = 11;
			ComCah[20] = 1;
			ComCah[21] = 0;
			ComCah[22] = 1;
			ComCah[23] = ps.lifeOver;;
			//����
			//ComCah[26] = 5;
			ComCah[24] = 14;
			ComCah[25] = 1;
			ComCah[26] = 0;
			ComCah[27] = 1;
			ComCah[28] = ps.muteflag;
			//����
			//ComCah[32] = 5;
			ComCah[29] = 45;
			ComCah[30] = 5;
			ComCah[31] = 0;
			ComCah[32] = 1;
			ComCah[33] =  GetFault();
			//��״̬
			ComCah[34] = 2;
			ComCah[35] = 4;
			ComCah[36] = 0;
			ComCah[37] = 1;
			ComCah[38] = IsFire? 1:0;
			NBSendData(39);
			break;
	}
	
}

void TestFlash(void)
{
	BaseErr = 0;
	FireErr = 0;
	FullErr = 0;
	unsigned char c, b[4];
	//ȡID
	Read_EE(IDAdd, b, 4);
	c = b[0] + b[1] + b[2];
	if(c != b[3]) {			//���û��д��ID�������FF FF FF
		MyID[0]=255;
		MyID[1]=255;
		MyID[2]=255; 
	}else{
		MyID[0]=b[0];
		MyID[1]=b[1];
		MyID[2]=b[2]; 
	}
	//ȡ����ֵ
	Read_EE(Base_valueAdd, b, 3);
    if((b[0] ^ b[1]) == b[2]) {
	    if(b[0] == 0 && b[1] == 0) {
	        b[0] = 0x08;
	        b[1] = 0x00;
	    }
		Valves.Base_value = (b[0] << 8) + b[1];
    } else {
        Valves.Base_value = 0;
        BaseErr= 1;	//δд����ֵ
    }
    //ȡ������
    Read_EE(Fire_valueAdd, b, 3);
    if((b[0] ^ b[1]) == b[2]) {
        if(b[0] == 0 && b[1] == 0) {
            b[0] = 0x00;            ///0xb8=184
            b[1] = 0xB8;
        }
        Valves.Fire_value = (b[0] << 8) + b[1];
    } else {
        Valves.Fire_value = 0;
        FireErr = 1; //û�궨��
    }
    //ȡ����
    Read_EE(Full_valueAdd, b, 3);
    if((b[0] ^ b[1]) == b[2]) {
        if(b[0] == 0 && b[1] == 0) {    ///0x600=1536
            b[0] = 0x06;
            b[1] = 0x00;
        }
        Valves.Full_value = (b[0] << 8) + b[1];
    } else {
        Valves.Full_value = 0;
        FullErr = 1;//δ�궨����
    }
    //ȡ�Ŵ���
    Read_EE(OPA_multipleAdd, b, 2);           //�Ŵ���
    if((b[0] ^ b[1]) == 0xff) {
        b[0] = b[0] & 15;

        if(b[0] > 2){
            b[0] = 1;
            WR_EE(OPA_multipleAdd, 1);
        }
    } 
    else {
        b[0] = 1;
    }
    //�˴���ȡ���ķŴ���д��Ĵ���
    //--------------------------------------//
    switch(b[0]){
    	case 0:_sdpgac1 = 0b11000000 + 2;break;//5
    	case 1:_sdpgac1 = 0b11000000 + 4;break;//9
    	case 2:_sdpgac1 = 0b11000000 + 8;break;//17
    }
    //--------------------------------------//
    
    //̽�����ۼ�����ʱ��   12Сʱ�ı���
    Read_EE(LifeHAdd, b, 3);		 
    if((b[0] ^ b[1]) == b[2])
        LifeCount = (b[0] << 8) + b[1];
    else
        LifeCount = 0; //�������ڼ���
    
}

uc GetFault(void){
	uc i=0;
	if(SenOpen || SenShort)				i |= 1;
	if(MatchErr)						i |= 2;
	if(BaseErr || FireErr || FullErr)	i |= 4;
	if(!ps.lifeOver)					i |= 8;
	return i;
}

int Getch4_value(void){
	//��ȡ�ϴ��ٷֱ�ֵ(һ��Ҫ�궨����)
	long int Percentage1, Percentage;
//	SendWord(OpAveVal[0]);
//	SendWord(RefAveVal[0]);
	Percentage1 = OpAveVal[0]-RefAveVal[0];
//	SendWord(Percentage1);
	
	if(Percentage1 & 0x8000) {	//Percentage1<0,Percentage<0
		return 0;
	};
	
	Percentage1 *= 1000;
//	SendWord(Percentage1);
	Percentage = Percentage1 / Valves.Full_value;
//	SendWord(Percentage);
	if((Percentage >= 0) && (Percentage <= 1000)){
		return Percentage;
	}else{
		return 1000;
	}
}

//��ⰴ��������˫�������������� 
void keyDetect(void){
	static uc  down_counter = 0; //�����������¼�����	
	static uc  up_counter = 0;   //��Ч������̧�𰴼���ļ�����
	static uc  up_flag =0;      //��Ч����̧�𰴼������ɰ���̧���־
	//static uc  Key_S.double_flag =0; //���´���
	if(_pa3 == 1){

		if(down_counter <= 134)          //������ֵ 4S������ʵ��������ģ���
		{
			down_counter++; 
		}
		else                             //�������µ�4S�����жϳ���ʱ����������ɳ�����־ 
		{ 	
			Key_S.Unstoppable = 1;        //��������־��λ
			Key_S.long_flag = 1;
			//----------��ⳤ��-----------------//
			//_st1on = 0;
			down_counter = 0;             //������¼���
			//Key_S.double_flag = 0;
			//----��ⳤ��̧�����ж�-----//
			return ;                       //��������
		}
	}
	else
	{	
		if(down_counter > 2)        //������ֵ������ʱ�����60ms����Ϊ��Ч���ɸ��ģ���
		{
			up_flag = 1;            //��Ч����̧�𰴼������ɰ���̧���־ 
			//�����ϴε���ʱ����1S֮�䣬�ٴε���������Ϊ�ٴη�������¼�
			if( up_counter > 1 && up_counter < 35)   //˫�������ֵ���ɸ��ģ���
			{ 
				//���˫������־��λ
				if(Key_S.double_flag == 1){
					Key_S.double_flag = 0;
					Key_S.Triple_kill = 1;     //��������־��λ
					_st1on = 0;
					up_flag = 0;               //�������̧���־
					up_counter = 0;            //�������̧�����
					
				}
				else
				{
					Key_S.double_flag=1;	//��ʾ��������
					//up_flag = 0; �������̧���־(Ҫ���������ȷ�������λ�������)
					up_counter = 0;      	//�������̧�����(�´ε����������)
				}
				//����̧��
//				if(Key_S.Unstoppable){
//					Key_S.Unstoppable=0;
//					Key_S.long_flaged =0;
//					ps.check_result = 0;	
//					_st1on = 0;
//					down_counter = 0;            //������¼���
//				}				
			} 
		} 
        down_counter = 0;
	};
	if(up_flag) //��Ч����̧�����������
       up_counter++;
	//1S�� ,û������������Ϊ�������;��������������ֵ��һ����ģ���
	if(up_counter > 35)      
	{ 
		if(Key_S.double_flag == 1){
			Key_S.Double_kill = 1;     //˫������־��λ
		}else if(!Key_S.long_flaged){	//֮ǰû�г���
			Key_S.First_blood = 1;    //��������־��λ	
	  	}else{
	  		Key_S.long_flaged = 0;		//��������������������0
	  	}
	  	_st1on = 0;
		up_counter = 0;
		up_flag = 0;
		Key_S.double_flag=0;		//ͬup_flag��Ҫ���������
	}
//	SendByte(down_counter);
//	SendByte(up_counter);
//	SendByte(click_counter);
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
void Beeps(uc t){
	while(t--){
		Beep(80);
		DelaymS(100);
	}
}
//v ����
void BuzzerOn(uc v){
	switch(v)
   {
      case 1:		//3K	Ƶ�ʽ�������Ƚ���
      	_ptmrpl=(uc)(Hz_L & 0xff);
    	_ptmrph=(uc)(Hz_L >>8);
         _ptmal=(uc)((Hz_L >>3) & 0xff);
         _ptmah=(uc)(Hz_L >>11);
         _pton=1;
         EN1=0;
         EN2=1;
         break;
      case 2:		//3.1k
       	_ptmrpl=(uc)(Hz_M & 0xff);
    	_ptmrph=(uc)(Hz_M >>8);
         _ptmal=(uc)((Hz_M>>2) & 0xff);
         _ptmah=(uc)(Hz_M >>10);
         _pton=1;
         EN1=1;
         EN2=0;
         break;
      case 3:		//3.2k
        _ptmrpl=(uc)(Hz_H & 0xff);
    	_ptmrph=(uc)(Hz_H >>8);
         _ptmal=(uc)((Hz_H>>1) & 0xff);
         _ptmah=(uc)(Hz_H>>9);	  
         _pton=1;
         EN1=1;
         EN2=1;
         break;
      default:
         _pton=0;
         EN1=0;
         EN2=0;
   }  
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

void Twinkle(uc color)
{
	if(color=='R')
	{	OpenLED_R;
   		DelaymS(15);  //5mS
   		CloseLED_R;
   	}else if(color=='G'){
   		OpenLED_G;
		DelaymS(15);  //5mS
		CloseLED_G;
	}else{
		OpenLED_R;
		OpenLED_G;
   		DelaymS(15);  //5mS
   		CloseLED_R;
   		CloseLED_G;
	}
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
		_txr_rxr=ComCah[i];
		while(_txif==0);
		while(_tidle==0);
	}
}

void sendstring(char *p){
	//����ASCII
	while(*p != '\0'){
		SendByte(*p);
		p++;
	}	
}

//����  l
void NBSendData(unsigned char l)
{
   unsigned char i,j;
   j=0;
   for(i=0;i<l;i++)
   {
      j=j+ComCah[i];
      _txr_rxr=ComCah[i];
      while(_tidle==0);
      while(_txif==0);
	  //while((_usr & 2)==0);
   }
   _txr_rxr=j;
   while(_tidle==0);
   while(_txif==0);
   //while((_usr & 2)==0);
}

void SendByte(unsigned char l)
{
	//����1���ֽ�
	_txr_rxr=l;
	while(_txif==0);//�ȴ����ݴӻ��������ص���λ�Ĵ�����
	while(_tidle==0);//�ȴ����ݷ������
}
void SendWord(unsigned long Word){
	SendByte(Word >> 8);
    SendByte(Word & 0xFF);
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

void Read_EE(uc Address, uc *buff, uc L){
	//��������EE��ַ��������飬��ȡ����
	unsigned char i;
	for(i=0; i<L; i++) {
		buff[i] = RD_EE(Address+i);
	}
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

void Write_EE(uc WR_EE_addr,uc DATA_addr,uc L)
{	//����д��EE��ַ��ComCah�д�д�����ݵĵ�ַ��д�볤�ȡ�
	unsigned char i;
	for(i=0; i<L; i++) {
		WR_EE(WR_EE_addr+i,ComCah[DATA_addr+i]);
	}
}

void __attribute((interrupt(0x08))) INT0(void){
	//int0�ж�
	if(_pa3==1){
		_int0f=0;
		Int0flag=1;
		_st1on = 1;	//��stm1
	}else{
		if(Key_S.Unstoppable){
			Key_S.long_to_up=1;
			_st1on = 0;	
			FireCount = 0;
			//IsFire=0;			//�Լ�ɹ����˳���
		}
	}
	
}

void INT0Init(void){
	_pas07=0;
	_pas06=0;	//PA3/INT0/STP1I
	_ifs13 = 0;_ifs12=1;//PA3��int0
	_pac3=1;	//����
	_pa3 = 0;
/*	_papu3=1;	//����*/
	/*INT0S1~INT0S0
	00������
	01��������
	10���½���
	11��˫��*/
	_int0s1=1;
	_int0s0=1;	//˫��
	_emi=1;
	_int0e=1;	//ʹ��int0
}

void __attribute((interrupt(0x3c))) UART(void)
{
	_mff=0;
	_urf=0;
	//30MS��ʱ
	if(_stm1af){
		_stm1af=0;	
		Timer1Flag=1;
	}
	else{
		//�����жϺ���
		/*	RxFlag=1;*/
		while(_rxif>0) {
			ComRCah[ComRPC]=_txr_rxr;
			if(ComRPC<24)
				ComRPC++;
			else
				ComRPC=0;
		}
		//----------------���㶨ʱ��------------
		_st0on=0;
		_st0on=1;
		//--------------------------------------
	}
}
void Stm1Init(void){
	 //��ʼ����ʱ��1 ���� 30mS
   _stm1c0=0b01000000;      //fsys=32K��
   _stm1c1=0b11000001;
   _stm1al=0xC0;
   _stm1ah=3;
   _stm1ae=1;
   _st1on=0; 
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
	//_sime=1;			//USIM�ж�ʹ��
	_emi=1;				//���ж�
}

void Stm0AInit(void)
{
	//��ʼ��STM0A,���ڴ��ڶ�ʱ
	_stm0c0 = 0b01000000;		//TIM0ʱ�� = fusb ; TIM0ʱ�� = 32kHz ;
	_stm0c1 = 0b11000001;		//��ʱ��ģʽ���Ƚϼ�����Aƥ����0������(STMA�жϺ�Ͳ��õ������������) ����������һ��Ϊ(1/32k)S
	_stm0al = 64;				// 2ms��ʱ
	_stm0ah = 0;				//

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

//����AD��ͨ����c=1 OP0��c=2 OP1��c=3 �¶�
unsigned int GetADC(unsigned char c)
{
   ut r=0;
   unsigned long sum=0;		//4ƽ���ã������Դ�
   uc i;
   //uc t1,t2;
   switch(c)
   {
      case 1:
         _sadc0=0b00001111;
         _sadc1=0b01001010;       //AD��λ��OP0
         break;
      case 2:	
         _sadc0=0b00001111;
         _sadc1=0b01101010;       //AD��λ��OP1
   	     break;
   	  default :
   	  	 _sadc0=0b00000100;
         _sadc1=0b00001010;       //AD��λ��AN4���¶�
   	  	 break;
   }
   
   for(i=0;i<4;i++){//4��ƽ��
	   _adcen=1;
	   _start=1;
	   _start=0;
	   while(_adbz==1);
//	   SendWord(_sadoh);
//	   SendWord(_sadol);
//	   SendWord(_sadoh<<4);
//	   SendWord(_sadol>>4);
/*����ADRFSΪ��ֵ��_sadohʼ��Ϊ8λ��_sadolʼ��Ϊ��4λ*/
	   r = (_sadoh<<4) + (_sadol>>4); //12λȫ��ȡ��
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

void get_temperature(void){
	//ȡ���¶�AD
   TEMPIO=0;
   GCC_DELAY(500);       //50uS��ʱ
   TempAD=GetADC(3);     //�¶�AD
   GCC_DELAY(500);       //50uS��ʱ
   TEMPIO=1;
}

//-15��---55��,���¶ȵ����λΪ1����-1��=0x81;-15��=0x8f;
unsigned char GetTemp(void)            //ȡ���¶�
{
   unsigned char i,r;
   if(TempAD<TMP[0])   
       r=0;
   else
   {
      if(TempAD>TMP[70])   
         r=70;
      else
      {
         for(i=0;i<71;i++)
         {
            if(TempAD<=TMP[i])
            {
               break;
            }
         }
         r=i;
      }
   }
   if(r<15)
   {
      r=15-r;
      r=r | 0x80;	///����λ��λ������
   }
   else
      r=r-15;
   return r; 	
}
	
void OPA_int(void)
{	
	//----------OPA����----------------------//
	_pas10=1;
	_pas11=1;			//A0O-PA4
	_pas12=0;
	_pas13=1;			//A1O-PA5
	_pbs01=1;			//A0PB-PB0
	_pas02=1;
	_pas03=1;			//A1PI-PA1
	_pac1= 1;
	_pac4= 0;
	_pac5= 0;
	_pbc0= 1;
	
	//��ʼ��OPA
	_sda0c=	0b01000001;		//enable OPAMP0,bandwidth=600KHz
	_sda1c=	0b01000010;		//enable OPAMP1,bandwidth=2MHz
	_sdsw=	0b01101000;		//ѡ�񿪹�	;---ON:SDS3		
	_sdpgac0=20;				//���� R1 ;N*100K
	//�����=(1+R2/(R3+10K))*������
	//R3=40,R2=200(5��)��400��9������800��17����
	//bit7~bit6�O��R3(00=10K,01=20K,10=30K,11=40k),bit5~bit0���� R2 ;N*100K
	_sdpgac1= 0b11000000+4;	//R3=40K,R2=400K,�Ŵ�9��	
}
//;===========================================================
//;��������:S_ADJ_OPA
//;��������: У�� OPA
//;����: 
//;���: 
//;��ע: 
//;=========================================================
void S_ADJ_OPA(void )
{
	unsigned char R_TMP0,R_TMP1,R_TMP2;
S_ADJ_OPA_LOOP_ADD_S:
	R_TMP0=0x00;
	_sda0vos=0b11000000;
//	OPA_ADJ_DELAY();
    GCC_DELAY(5000);
	R_TMP2=0x00;
	if(_sda0o) 	R_TMP2=R_TMP2|0x01;	//R_TMP2.0=1;	
S_ADJ_OPA_LOOP_ADD:
	++R_TMP0;
	++_sda0vos;
//	OPA_ADJ_DELAY();
    GCC_DELAY(5000);
	if(_sda0o!=(R_TMP2&0x01)) goto S_ADJ_OPA_LOOP_ADD_OK;	//R_TMP2.0
	_acc=0b00111111&_sda0vos;
	if(_acc==0b00111111) goto S_ADJ_OPA_LOOP_ADD_S;
	goto S_ADJ_OPA_LOOP_ADD;	
S_ADJ_OPA_LOOP_ADD_OK:

S_ADJ_OPA_LOOP_SUB_S:
	R_TMP1=		0b00111111;
	_sda0vos=	0b11111111;
//	OPA_ADJ_DELAY();
    GCC_DELAY(5000);
	R_TMP2=0x00;
	if(_sda0o) 	R_TMP2=R_TMP2|0x01;	//R_TMP2.0=1;
S_ADJ_OPA_LOOP_SUB:
	--R_TMP1;
	--_sda0vos;
//	OPA_ADJ_DELAY();
    GCC_DELAY(5000);
	if(_sda0o!=(R_TMP2&0x01)) goto S_ADJ_OPA_LOOP_SUB_OK;	//R_TMP2.0
	_acc=0b00111111&_sda0vos;
	if(_acc==0b00000000) goto S_ADJ_OPA_LOOP_SUB_S;
	goto S_ADJ_OPA_LOOP_SUB;
S_ADJ_OPA_LOOP_SUB_OK:
	_sda0vos=(R_TMP1+R_TMP0)/2;
}

//;===========================================================
//;��������:S_ADJ_OPA
//;��������: У�� OPA
//;����: 
//;���: 
//;��ע: 
//;=======================================================
void S_ADJ_OPA1(void )
{
	unsigned char R_TMP0,R_TMP1,R_TMP2;
S_ADJ_OPA1_LOOP_ADD_S:
	R_TMP0=0x00;
	_sda1vos=0b11000000;
//	OPA_ADJ_DELAY();
	GCC_DELAY(5000);
	R_TMP2=0x00;
	if(_sda1o) 	R_TMP2=R_TMP2|0x01;	//R_TMP2.0=1;	
S_ADJ_OPA1_LOOP_ADD:
	++R_TMP0;
	++_sda1vos;
//	OPA_ADJ_DELAY();
	GCC_DELAY(5000);
	if(_sda1o!=(R_TMP2&0x01)) goto S_ADJ_OPA1_LOOP_ADD_OK;	//R_TMP2.0
	_acc=0b00111111&_sda1vos;
	if(_acc==0b00111111) goto S_ADJ_OPA1_LOOP_ADD_S;
	goto S_ADJ_OPA1_LOOP_ADD;	
S_ADJ_OPA1_LOOP_ADD_OK:

S_ADJ_OPA1_LOOP_SUB_S:
	R_TMP1=		0b00111111;
	_sda1vos=	0b11111111;
//	OPA_ADJ_DELAY();
	GCC_DELAY(5000);
	R_TMP2=0x00;
	if(_sda1o) 	R_TMP2=R_TMP2|0x01;	//R_TMP2.0=1;
S_ADJ_OPA1_LOOP_SUB:
	--R_TMP1;
	--_sda1vos;
//	OPA_ADJ_DELAY();
	GCC_DELAY(5000);
	if(_sda1o!=(R_TMP2&0x01)) goto S_ADJ_OPA1_LOOP_SUB_OK;
	_acc=0b00111111&_sda1vos;
	if(_acc==0b00000000) goto S_ADJ_OPA1_LOOP_SUB_S;
	goto S_ADJ_OPA1_LOOP_SUB;
S_ADJ_OPA1_LOOP_SUB_OK:

	_sda1vos=(R_TMP1+R_TMP0)/2;		
}
