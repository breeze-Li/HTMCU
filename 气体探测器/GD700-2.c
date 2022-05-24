//2021年11月29日:串口配置完成
///2021年12月7日:温度采样及十位运放采样
//2022年1月6日：串口通讯框架

#include "BA45F5250.h"
typedef unsigned char uc;
typedef unsigned int ut;
typedef unsigned long int ul;

#define  EN1 _pa7
#define  EN2 _pa3
#define  TEMPIO _pb2		//温度采样控制引脚
#define  ABS(X) ((X)>=0)?(X):(-(X))

#define  OpenLED_R    _isgenc |= 0b10000010;		///ISINK1 引脚灌电流使能,50mA
#define  OpenLED_G    _isgenc |= 0b10000001;		///ISINK0 引脚灌电流使能,50mA
#define  CloseLED_R   _isgenc &= 0b10000001;		///ISINK1 引脚灌电流除能
#define  CloseLED_G   _isgenc &= 0b10000010;		///ISINK0 引脚灌电流除能

#define IDAdd 0x10          //设备ID  H+M+L+Xor
#define LifeHAdd 0x14	    //运行时间单位12小时 H+L+Xor

volatile bit  Tb0_flg;
volatile bit  Tb1_flg;
volatile bit  RxFlag;       				//串口有新数据标志
volatile bit  transfer_complete_flag;		//接收传输完成标志

volatile unsigned char ComPC;		        //发送数据指针
volatile unsigned char ComCah[24];          //发送缓冲区
volatile unsigned char ComRPC;		        //接收数据指针
volatile unsigned char ComRCah[24];         //接收缓存
volatile unsigned char MyID[3];
volatile unsigned char pointer_length;		//指针长度(记录接受的字节长度)

volatile unsigned char TempAD,CurTemp;         //温度采样
volatile unsigned char  OP1,OP0;
volatile unsigned char sadoh,sadol;
//-15度到55度
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
void SendData(uc L);
uc RD_EE(uc RD_EE_addr);
void WR_EE(uc ,uc);
void SetPack(void);
void DelaymS(ut t);
void Beep(uc t);
void Twinkle_R(void);
void Twinkle_G(void);
uc GetADC(uc c);
int get_difference(void);
void get_temperature(void);
uc GetTemp(void);
void Com_Management(void);
void SetPack(void);
void TestFlash(void);
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
	if(0){
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

   //初始化所有IO
   
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
//   //初始化IO复用功能
//   _pas0=0b00001100;
//   _pas1=0b00011011;
//   _pbs0=0b11001110;   //串口
//   _pbs1=0b00001100;
//   _pcs0=0b00000000;
//   _pcs1=0;
//   _ifs1=0b00000000;
	}
   
    _wdtc=0x57;
    
    _pbc2=0;
    TEMPIO=1;			//温度采样控制引脚
    
    //----------ADC引脚(温度)---------------------//
	_pbs13=1;
	_pbs12=1;
	_pbc5=1;
	
    //初始化时基
    _pscr = 2;		///32k
    _tb0c = 0x07;
    _tb0e = 1;   //时基0设置1秒定时
    _tb1c = 0x06; //时基1设置为0.5秒
    _tb1e = 1;
//
//    //初始化PWM
//    _ptmc0 =	0b00000000;	//设置分频比 FSYS/4   4M系统时钟
//    _ptmc1 =	0b10101000;
//    _ptmc2 =	0b00000000;
//    _ptmal =	0x9c;
//    _ptmah =	0;	   //DUTY=50%
//    _ptmrpl =	0x39;
//    _ptmrph =	0x01;	//PERIOD=3.2KHz
    

    //开时基0
    _tb0on = 1;
    _tb1on = 1;
    _emi = 1;
    
    Twinkle_G();
    DelaymS(200);
    Twinkle_G();
	
	UARTInit();
	Stm0AInit();

	OPA_int();
	DelaymS(20);
	S_ADJ_OPA1();
	S_ADJ_OPA1();
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
//			DelaymS(1000);
//			get_temperature();
//			CurTemp=GetTemp();
//			SendByte(TempAD);
//			SendByte(((CurTemp/10)<<4)+(CurTemp%10));
			DelaymS(1000);
			m=get_difference();
			SendByte(m);
			Twinkle_R();
			
			if(transfer_complete_flag)
				Com_Management();
			
        	GCC_CLRWDT();
    }
}

void Com_Management(void){
	uc j,k;
	transfer_complete_flag=0;
	if(ComRCah[0]==0xcc){	
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
            		case 0x81:
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
            		case 0x84:
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
//            		case 0x80:
//            		case 0x80:
//            		case 0x80:
//            		case 0x80:
//            		case 0x80:
//            		case 0x80:
//            		case 0x80:
//            		case 0x80:
            	}
            }
		}
	}
}
void TestFlash(void)
{
	//取EE数据
	unsigned char TempCah[10];
	//取ID
	MyID[0]=RD_EE(IDAdd);
	MyID[1]=RD_EE(IDAdd+1);
	MyID[2]=RD_EE(IDAdd+2);

	TempCah[0]=RD_EE(IDAdd+3);
	TempCah[1]=MyID[0] + MyID[1] + MyID[2];
	if(TempCah[0]!=TempCah[1]) {			//如果没有写入ID，则读出0 0 0
		MyID[0]=0;
		MyID[1]=0;
		MyID[2]=0;
	}
}
void DelaymS(unsigned int t) {
    unsigned int i;
    for(i = 0; i < t; i++)
        GCC_DELAY(1000);
}
//报警声音 p 频率，v 音量,t 持续时间mS
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
 * [__attribute description] 1S定时到
 * @param void [description]
 */
void __attribute((interrupt(0x30))) STB0(void) {
    _tb0f = 0;
    Tb0_flg = 1;
}
/**
 * [__attribute description]0.5s定时到
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
	//发送数据头
	ComCah[0]=0xcc;
	ComCah[1]=0x99;
	ComCah[2]=0x99;
	ComCah[3]=0xcc;
}

void SendData(unsigned char L)
{
	//发送L个字节
	unsigned char i;
	for(i=0; i<L; i++) {
		_txr_rxr=ComCah[i];
		while(_txif==0);
		while(_tidle==0);
	}
}

void SendByte(unsigned char l)
{
	//发送1个字节
	_txr_rxr=l;
	while(_txif==0);//等待数据从缓冲器加载到移位寄存器中
	while(_tidle==0);//等待数据发送完成
}

unsigned char RD_EE(unsigned char RD_EE_addr)
{
	//读EE
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
	//写EE
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

void __attribute((interrupt(0x3c))) UART(void)
{
	//接受中断函数
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

	//----------------清零定时器------------
	_st0on=0;
	//GCC_DELAY(50);//这个延时很关键,无此不可清除计数器
	_st0on=1;
	//--------------------------------------
}

void UARTInit(void)
{
	//初始化UART			
   _ucr1=0b10000000;
   _ucr2=0b11101100;
   _brg=25;				//4M时钟时波特率9600
   _ure=1;
   _mfe=1;
	// --------------GD700-----------------
   _pbs03=1;			
   _pbs02=1;
   _pbc1=0;				//PB1做TX,输出
   _pbs06=1;
   _pbs07=1;			//PB3做RX
   _pbpu3=1;			//pb3上拉
   _ifs00=0;			//RX 输入源引脚为Pb3
	//-------------------------------------
	_sime=1;			//USIM中断使能
	_emi=1;				//总中断
}

void Stm0AInit(void)
{
	//初始化STM0A,用于串口定时
	_stm0c0 = 0b01000000;		//TIM0时钟 = fusb ; TIM0时钟 = 32kHz ;
	_stm0c1 = 0b11000001;		//定时器模式、比较计数器A匹配清0计数器(STMA中断后就不用单独清计数器了) 计数器计数一次为(1/32k)S
	_stm0al = 64;				// 2ms定时
	_stm0ah = 0;				//

	_stm0ae = 1;		//比较计数器A匹配中断使能
	_emi = 1;			//使能总中断

	_st0on = 0;			//关闭定时器
}

void __attribute((interrupt(0x2c))) STM0AIQR(void)
{	//2ms定时跑完
//	static unsigned int sec = 0;
	_stm0af=0;						//清除中断标志
	//if(++sec>300) {					//3毫秒，传输完成，关闭定时器
	//	sec=0;						//清除毫秒计数
	transfer_complete_flag=1;	//传输完成标志置1
	
	pointer_length=ComRPC;		//记录指针长度
	ComRPC=0;					//接受指针置零
	_st0on=0;					//关闭STM0A计数器
	//	SendByte(10);

	//}
}

//测试AD的通道，c=1 OP0，c=2 OP1，c=3 温度
unsigned char GetADC(unsigned char c)
{
   ut r=0;
   uc t1,t2;
   switch(c)
   {
      case 1:
         _sadc0=0b00001111;
         _sadc1=0b01001010;       //AD定位到OP0
         break;
      case 2:	
         _sadc0=0b00001111;
         _sadc1=0b01101010;       //AD定位到OP1
   	     break;
   	  default :
   	  	 _sadc0=0b00000100;
         _sadc1=0b00001100;       //AD定位到AN4、温度
   	  	 break;
   }	
   _adcen=1;
   _start=1;
   _start=0;
   while(_adbz==1);  
   if(c==3){
   		r=_sadoh;
   		_adcen=0;
   		return r;
   }
    sadoh=_sadoh;
    sadol=_sadol;
    SendByte(0xFE);
    SendByte(sadoh);
	SendByte(sadol);
	//*十二位的AD值取前十位********/
    t1=sadoh<<2;
	t2=sadol>>6;
    sadol=t1+t2;
    ///sadol=sadoh<<2+sadol>>6;无效
    
    sadoh=sadoh>>6;
	//	r = sadoh<<8;
	//	r += sadol;
	///r = sadoh<<8+sadol;无效
	/*********************************/
    _adcen=0;
    return 0;
}

int get_difference(void){
	//获取差值
	volatile ut op0,op1;
	op0=GetADC(1);
	SendByte(sadoh);
	SendByte(sadol);
	GCC_DELAY(50);
	op1=GetADC(2);
	SendByte(sadoh);
	SendByte(sadol);
	return (op1-op0);
}

void get_temperature(void){
	//取得温度AD
   TEMPIO=0;
   GCC_DELAY(500);       //50uS延时
   TempAD=GetADC(3);     //温度AD
   GCC_DELAY(500);       //50uS延时
   TEMPIO=1;
}

//-15度---55度,负温度的最高位为1，如-1度=0x81;-15度=0x8f;
unsigned char GetTemp(void)            //取得温度
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
      r=r | 0x80;	///符号位置位，表负数
   }
   else
      r=r-15;
   return r; 	
}
	
void OPA_int(void)
{	
	//----------OPA引脚----------------------//
	_pas10=1;
	_pas11=1;			//A0O
	_pas12=0;
	_pas13=1;			//A1O
	_pbs01=1;			//A0PB
	_pas02=1;
	_pas03=1;			//A1PI
	_pac1=1;
	_pac4=0;
	_pac5=0;
	_pbc0=1;
	
	//初始化OPA
	_sda0c=	0b01000001;		//enable OPAMP0,bandwidth=40KHz
	_sda1c=	0b01000010;		//enable OPAMP1,bandwidth=600KHz
	_sdsw=		0b01101000;		//选择开关	;---ON:SDS3		
	_sdpgac0=	20;				//设置 R1 ;N*100K
	//bit7~bit6設置R3(00=10K,01=20K,10=30K,11=40k),bit5~bit0设置 R2 ;N*100K
	_sdpgac1=	0b01000000+1;	//R3=20K,R2=100K,放大6倍	
}

//;===========================================================
//;函数名称:S_ADJ_OPA
//;函数功能: 校正 OPA
//;输入: 
//;输出: 
//;备注: 
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
//;函数名称:S_ADJ_OPA
//;函数功能: 校正 OPA
//;输入: 
//;输出: 
//;备注: 
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