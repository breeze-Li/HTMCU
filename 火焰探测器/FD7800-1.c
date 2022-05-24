//2021-9-8:输入捕获完成,单片机可以捕获到紫外管的上升沿脉冲
//2021-9-9:解决昨日的死循环问题，可通过串口读取每秒脉冲数量。


#include "BA45F5542-2.h"

typedef unsigned char uc;
typedef unsigned int uint;

#define  alarm_output  _pa1         /*报警输出  io*/
#define  pulse_input  _pa4          /*检测脉冲输入*/

#define  current_cycle	2500		//灌电流周期(微秒)
#define  current_draw_time 5		//灌电流时间(微秒)
#define  current_wait_time (current_cycle-current_draw_time)	//灌电流等待时间(微秒)
#define  number_of_cycles  (1000000/current_cycle) 				//一秒循环次数

#define  OpenLED	_isgenc |= 0b10000010; 		///ISINK1 引脚灌电流使能,50mA
#define  CloseLED	 _isgenc &= 0b10000001;		///ISINK1 引脚灌电流除能
#define  ToggleLED	  _isgenc ^= 0b00000010;	///ISINK1 反转
#define  OpenISINK0	   _isgenc |= 0b10000001;	///ISINK0 引脚灌电流使能
#define  CloseISINK0	_isgenc &= 0b10000010;	///ISINK0 引脚灌电流除能

uc temporary_count=0;			//临时计数

//中断向量
DEFINE_ISR(PTMAIQR, 0x24);

//----------灌电流大小选择-----------------//
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

	_wdtc=0b01010110;			///4秒
	_scc=0x01;			///不分频(系统时钟为fH)
	_hircc=0x05;		///4M
	while(_hircf==0);   //4M时钟，不分频
	
	//----------初始化-------------------------//
	PTMInit();			//输入捕捉初始化
	UARTInit();				//串口初始化
	
	
	//----------灌电流发生器----------------------//
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
			//持续1秒的脉冲
			OpenISINK0;//打开ISINK0
			GCC_DELAY(current_cycle);	//延时5微妙

			CloseISINK0;//关闭ISINK0
			GCC_DELAY(current_wait_time);	//延时2495微妙
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


//----------初始化类函数----------------------//
void UARTInit(void)
{
	//初始化UART
	_umd=1;				//做串口
	_uucr1=0x80;
	_uucr2=0b11101100;
	_ubrg=25;			//4M时钟时波特率9600

	// --------------FD718-----------------
   _pas06=1;			//PA3做TX
   _pas15=1;			//PA6做RX
   _papu6=0;			//pa6上拉
   _ifs00=1;			//RX 输入源引脚为Pa6
	//-------------------------------------
	
	_usime=1;			//USIM中断使能
	_emi=1;				//总中断
}


void PTMInit(void) //输入捕获初始化
{


	_pas10 = 0;
	_pas11 = 0;		//PA4复用为PA4/PTCK
	_pac4 = 1;		//输入
	_pa4 = 0;		//默认低电平
	_papu4=0;		//除能上拉电阻

	_ptmc0 = 0b00100000;//PIM时钟 = FH/16 = 4M/16;
	_ptmc1 = 0b01000010;//捕捉输入模式,PTCK 上升沿输入捕捉，计数器值将锁存至 CCRA,
	_ptmc2 = 0x00;//捕捉输入模式时计数器清零条件选择位

	_ptmae = 1;//比较计数器A匹配中断使能
	_emi = 1;//使能总中断

	_pton = 1;//打开PTM定时器
}

//----------中断类函数-----------------------//
void PTMAIQR(void)
{
	//捕捉到脉冲
	_ptmaf=0;
	temporary_count++; //计数加一
	//Twinkle();
}


//----------一般自定义函数---------------------//
void provide_current_2500uS(void) 	//2500微秒(2.5MS)给一次灌电流
{

	OpenISINK0;//打开ISINK0
	GCC_DELAY(5);	//延时5微妙

	CloseISINK0;//关闭ISINK0
	GCC_DELAY(2495);	//延时2495微妙
}

void DelaymS(unsigned int t)
{
	//延时一毫秒
	unsigned int c;
	for(c=0; c<t; c++) GCC_DELAY(1000);///4M时钟
}


void Twinkle(void)
{
	//闪灯
	OpenLED;
	GCC_DELAY(5000);  //5mS
	CloseLED;
}

void SendByte(unsigned char l)
{
	//_emi=0;
	//发送1个字节
	_utxr_rxr=l;
	while(_utxif==0);
	while(_utidle==0);
	//_emi=1;
}