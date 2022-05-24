//2021��11��29��:�����������
///2021��12��7��:�¶Ȳ�����ʮλ�˷Ų���
//2022��1��6�գ�����ͨѶ���
//2022��2��11�գ�����ԭ518����ȡADֵ��ʹ��12λ�˷ţ�


#include "BA45F5250.h"
typedef unsigned char uc;
typedef unsigned int ut;
typedef unsigned long int ul;

#define  EN1 _pa7
#define  EN2 _pa3
#define  TEMPIO _pb2		//�¶Ȳ�����������
#define  ABS(X) ((X)>=0)?(X):(-(X))

#define  OpenLED_R    _isgenc |= 0b10000010;		///ISINK1 ���Ź����ʹ��,50mA
#define  OpenLED_G    _isgenc |= 0b10000001;		///ISINK0 ���Ź����ʹ��,50mA
#define  CloseLED_R   _isgenc &= 0b10000001;		///ISINK1 ���Ź��������
#define  CloseLED_G   _isgenc &= 0b10000010;		///ISINK0 ���Ź��������

#define IDAdd 0x10          //�豸ID  H+M+L+Xor
#define LifeHAdd 0x14	    //����ʱ�䵥λ12Сʱ H+L+Xor

volatile bit  Tb0_flg;
volatile bit  Tb1_flg;
volatile bit  RxFlag;       				//�����������ݱ�־
volatile bit  transfer_complete_flag;		//���մ�����ɱ�־
volatile bit  BaseErr;
volatile bit  FireErr;
volatile bit  FullErr;

volatile unsigned char ComPC;		        //��������ָ��
volatile unsigned char ComCah[24];          //���ͻ�����
volatile unsigned char ComRPC;		        //��������ָ��
volatile unsigned char ComRCah[24];         //���ջ���
volatile unsigned char MyID[3];
volatile unsigned char pointer_length;		//ָ�볤��(��¼���ܵ��ֽڳ���)

volatile unsigned int op_value;				//��ǰ����ֵ
volatile unsigned int reference_value;		//��λ��adֵ
volatile unsigned int initial_value;			//��ʼֵ
volatile unsigned int fire_increment;		//������
volatile unsigned int fire_range;			//������
volatile unsigned int OpAveVal[2] = {0};
volatile unsigned int RefAveVal[2]= {0};

volatile unsigned char TempAD,CurTemp;         //�¶Ȳ���
volatile unsigned char  OP1,OP0;
volatile unsigned char sadoh,sadol;
volatile unsigned char lightcount=0;
//-15�ȵ�55��
const unsigned char TMP[71]={30,32,34,35,37,39,
                             41,43,45,47,49,51,53,55,58,60,
                             62,65,67,70,72,75,77,80,83,86,
                             88,91,94,97,100,102,105,108,111,114,
                             117,120,122,125,128,131,133,136,139,141,
                             145,147,150,152,155,157,160,162,164,167,
                             170,172,174,176,178,180,182,184,186,188,
                             190,192,194,196,198};


void Stm0AInit(void);
void UARTInit(void);
void OPA_int(void);
void S_ADJ_OPA(void );
void S_ADJ_OPA1(void );
void SendByte(uc l);
void SendWord(unsigned long Word);
void SendData(uc L);
uc RD_EE(uc RD_EE_addr);
void Read_EE(uc , uc *buff, uc L);
void WR_EE(uc ,uc);
void Write_EE(uc ,uc ,uc );
void SetPack(void);
void DelaymS(ut t);
void Beep(uc t);
void Twinkle_R(void);
void Twinkle_G(void);
ut GetADC(uc c);
int get_difference(void);
void get_temperature(void);
uc GetTemp(void);
void Com_Management(void);
void SetPack(void);
void TestFlash(void);
void function(void);

void main() {
    unsigned char i, j, k;
    unsigned int m;

    _scc = 0x01;
    _hircc = 0x05;
    while(_hircf == 0);


	ComRPC = 0;
	ComPC = 0;
	pointer_length = 0;
	transfer_complete_flag = 0;
	Tb1_flg = 0;
	Tb0_flg = 0;
	RxFlag = 0;
	FullErr = 0;
	BaseErr = 0;
	FireErr = 0;

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
    
if(0){
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
	}
   /*******������ʼ��********/
	  op_value = 0;				
	  reference_value = 0;	
	  initial_value = 0;			
	  fire_increment = 0;	
	  fire_range = 0;			
   
    _wdtc=0x57;
    
    _pbc2=0;
    TEMPIO=1;			//�¶Ȳ�����������
    
    //----------ADC����(�¶�)---------------------//
	_pbs13=1;
	_pbs12=1;
	_pbc5=1;
	
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
	SendByte(15);

	OPA_int();
	DelaymS(20);
	S_ADJ_OPA1();
	S_ADJ_OPA1();
//	SendByte(sizeof(char));				//1
//	SendByte(sizeof(short int));		//2
//	SendByte(sizeof(int));				//2
//	SendByte(sizeof(long));				//4
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
//			if(transfer_complete_flag){
//				transfer_complete_flag=0;
//				Twinkle_R();
//				//SendData(20);
//				m=pointer_length-1;
//				WR_EE(0,ComRCah[ComRPC]);
//				WR_EE(2,ComRCah[m]);
//				SendByte(pointer_length);
//				SendByte(RD_EE(0));
//				SendByte(RD_EE(2));
//			}
			if(Tb0_flg){
				Tb0_flg=0;
				
				SendByte(15);
				
				get_temperature();
				CurTemp=GetTemp();
				SendByte(TempAD);
				SendByte(((CurTemp/10)<<4)+(CurTemp%10));
			
				DelaymS(10);
				function();
				//Twinkle_R();
				if(lightcount++  > 4) {lightcount=0;Twinkle_R();}
				}
			if(transfer_complete_flag){Com_Management();
				Twinkle_G();
			
			}
        	GCC_CLRWDT();
    }
}

void function(void){
	unsigned int sum, a;
	
	a = GetADC(2);
	op_value = a;
	sum = (OpAveVal[0] << 2) + OpAveVal[1];  ///������һ�ε������λ
    sum = sum - OpAveVal[0];
    sum = sum + a;
    OpAveVal[0] = (sum >> 2); 	//4ƽ��
    OpAveVal[1] = (sum & 3);  	///����ȡ�����λ
    
	SendWord(OpAveVal[0]);

    a = GetADC(1); 				//AN3 ��׼(����������)
    reference_value = a;
    sum = (RefAveVal[0] << 2) + RefAveVal[1];
    sum = sum - RefAveVal[0];
    sum = sum + a;
    RefAveVal[0] = (sum >> 2); //4ƽ��
    RefAveVal[1] = (sum & 3);
    
    SendWord(RefAveVal[0]);
}

void Com_Management(void){
	uc j,k;						
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
            		case 0x84:		// ID
            			if(ComRCah[4]==1)
                            {
                               TestFlash();
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
//					case 0x80:
//					case 0x81:
//					case 0x82:
//					case 0x83:
//					case 0x84:
//					case 0x85:
//					case 0x86:
//					case 0x87:
//					case 0x88:
//					case 0x89:
//					case 0x90:
//					case 0x91:

            	}
            }
		}
	}
	else{	//WiFiͨѶ
		
	}
}


void TestFlash(void)
{
	BaseErr = 0;
	FireErr = 0;
	FullErr = 0;
	unsigned char c, b[4];
	//ȡEE����
	unsigned char TempCah[10];
	//ȡID
	Read_EE(IDAdd, b, 4);
	c = b[0] + b[1] + b[2];
	if(c != b[3]) {			//���û��д��ID�������FF FF FF
		MyID[0]=255;
		MyID[1]=255;
		MyID[2]=255; 
	}
	Read_EE();
    if((b[0] ^ b[1]) == b[2]) {
	    if(b[0] == 0 && b[1] == 0) {
	        b[0] = 0x08;
	        b[1] = 0x00;
	    }
		MyThread.c4 = (b[0] << 8) + b[1];
    } else {
        MyThread.c4 = 0;
        BaseErr = 1;
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
		_txr_rxr=ComCah[i];
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
{	//����д��EE��ַ��ComRCah�д�д�����ݵĵ�ַ��д�볤�ȡ�
	unsigned char i;
	for(i=0; i<L; i++) {
		WR_EE(WR_EE_addr+i,ComRCah[DATA_addr+i]);
	}
}


void __attribute((interrupt(0x3c))) UART(void)
{
	//�����жϺ���
	RxFlag=1;
	_mff=0;
	_urf=0;
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
         _sadc1=0b00001100;       //AD��λ��AN4���¶�
   	  	 break;
   }
   
   for(i=0;i<4;i++){//4��ƽ��
	   _adcen=1;
	   _start=1;
	   _start=0;
	   while(_adbz==1);
	   r = (_sadoh<<4) + (_sadol>>4);
	   sum += r;
   }
     
   if(c==3){
   		r=_sadoh;
   		_adcen=0;
   		return r;
   }
//    sadoh=_sadoh;
//    sadol=_sadol;
//    SendByte(0xFE);
//    SendByte(sadoh);
//	SendByte(sadol);
	/*ʮ��λ��ADֵȡǰʮλ********
    t1=sadoh<<2;
	t2=sadol>>6;
    sadol=t1+t2;
    ///sadol=sadoh<<2+sadol>>6;��Ч
    
    sadoh=sadoh>>6;
	//	r = sadoh<<8;
	//	r += sadol;
	///r = sadoh<<8+sadol;��Ч*/
	
    _adcen=0;
    r = sum>>2;
//    SendByte(r >> 8);
//    SendByte(r & 0xFF);
    SendWord(r);
    return r;
}

int get_difference(void){  
	//��ȡ��ֵ
	volatile ut op0,op1;
	op0=GetADC(1);
	//SendByte(op0 >> 4);
	//SendByte(op0 & 0xFF);
	DelaymS(10);
	op1=GetADC(2);
	//SendByte(op1 >> 4);
	//SendByte(op1 & 0xFF);
	return (op1-op0);
}

void get_temperature(void){
	//ȡ���¶�AD
   TEMPIO=0;
   GCC_DELAY(500);       //50uS��ʱ
   TempAD=GetADC(3);     //�¶�AD
   GCC_DELAY(500);       //50uS��ʱ
   //TEMPIO=1;
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
	_pac1=1;
	_pac4=0;
	_pac5=0;
	_pbc0=1;
	
	//��ʼ��OPA
	_sda0c=	0b01000001;		//enable OPAMP0,bandwidth=40KHz
	_sda1c=	0b01000010;		//enable OPAMP1,bandwidth=600KHz
	_sdsw=	0b01101000;		//ѡ�񿪹�	;---ON:SDS3		
	_sdpgac0=20;				//���� R1 ;N*100K
	//bit7~bit6�O��R3(00=10K,01=20K,10=30K,11=40k),bit5~bit0���� R2 ;N*100K
	_sdpgac1= 0b01000000+1;	//R3=20K,R2=100K,�Ŵ�6��	
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