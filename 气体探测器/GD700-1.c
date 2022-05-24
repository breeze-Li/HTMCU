//2021年11月29日:串口配置完成

#include "BA45F5250.h"

#define  EN1 _pa7
#define  EN2 _pa3

#define  OpenLED_R    _isgenc |= 0b10000010;		///ISINK1 引脚灌电流使能,50mA
#define  OpenLED_G    _isgenc |= 0b10000001;		///ISINK0 引脚灌电流使能,50mA
#define  CloseLED_R   _isgenc &= 0b10000001;		///ISINK1 引脚灌电流除能
#define  CloseLED_G   _isgenc &= 0b10000010;		///ISINK0 引脚灌电流除能

volatile bit  Tb0_flg;
volatile bit  Tb1_flg;
volatile bit  transfer_complete_flag;		//接收传输完成标志
volatile bit  RxFlag;       				//串口有新数据标志

volatile unsigned char ComPC;		        //发送数据指针
volatile unsigned char ComCah[24];          //发送缓冲区
volatile unsigned char ComRPC;		        //接收数据指针
volatile unsigned char ComRCah[24];         //接收缓存
volatile unsigned char pointer_length;		//指针长度(记录接受的字节长度)

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
   
   _wdtc=0x57;

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
		_txr_rxr=ComRCah[i];
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

void __attribute((interrupt(0x3c))) UART(void)
{
	//接受中断函数
	RxFlag=1;
	_mff=0;
	_urf=0;
/*	_st0on=1;				//打开定时器*/
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
	_stm0c0 = 0b01000000;//TIM0时钟 = fusb ; TIM0时钟 = 32kHz ;
	_stm0c1 = 0b11000001;		//定时器模式、比较计数器A匹配清0计数器(STMA中断后就不用单独清计数器了) 计数器计数一次为(1/32k)S
	_stm0al = 64;		// 2ms定时
	_stm0ah = 0;			//

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
