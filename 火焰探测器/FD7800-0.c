//2021-9-8:输入捕获完成,单片机可以捕获到紫外管的上升沿脉冲，该文件作废，另起FD7800-1.C文件



#include "BA45F5542-2.h"
#define  s0 _pa6         			/*按键0*/
#define  s1 _pa3         			/*按键1*/
#define  alarm_output  _pa4         /*报警输出  io*/
#define  pulse_input  _pa1          /*检测脉冲输入*/

#define  current_cycle	2500		//灌电流周期(微秒)
#define  current_draw_time 5		//灌电流时间(微秒)
#define  current_wait_time (current_cycle-current_draw_time)	//灌电流等待时间(微秒)
#define  number_of_cycles  (1000000/current_cycle) 				//一秒循环次数

#define  OpenLED	_isgenc |= 0b10000010; 		///ISINK1 引脚灌电流使能,50mA
#define  CloseLED	 _isgenc &= 0b10000001;		///ISINK1 引脚灌电流除能
#define  ToggleLED	  _isgenc ^= 0b00000010;	///ISINK1 反转
#define  OpenISINK0	   _isgenc |= 0b10000001;	///ISINK0 引脚灌电流使能
#define  CloseISINK0	_isgenc &= 0b10000010;	///ISINK0 引脚灌电流除能


//中断向量
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

	_wdtc=0x57;			///8秒
	_scc=0x01;			///不分频(系统时钟为fH)
	_hircc=0x05;		///4M
	while(_hircf==0);   //4M时钟，不分频
	
	PTMInit();			//输入捕捉初始化


	_papu3=0;
	_papu6=0;         //取消使能上拉
	// _papu4=1;

	_pac1=0;
	_pa1=0;
	//_pac4=1;         //0输出；1输入
	_pac3=1;         //in
	_pac6=1;         //out 输出

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

			OpenISINK0;//打开ISINK0
			//DelaymS(20);	//延时20毫妙
			//_pa1=1;
			GCC_DELAY(current_draw_time);	//延时10微妙

			CloseISINK0;//关闭ISINK0
			//_pa1=0;
			GCC_DELAY(current_wait_time);	//延时2490微妙
			//DelaymS(20);	//延时20毫妙
		}
		//CloseLED;
		//DelaymS(200);
		//Twinkle();



//		if(pulse_input==1) { //报警
//			_isgs1=1;
//			_papu4=1;
//		}
		GCC_CLRWDT();
	}
}

void provide_current_2500uS(void) 	//2500微秒(2.5MS)给一次灌电流
{

	OpenISINK0;//打开ISINK0
	//DelaymS(20);	//延时20毫妙
	GCC_DELAY(10);	//延时10微妙

	CloseISINK0;//关闭ISINK0
	GCC_DELAY(2480);	//延时2490微妙
	//DelaymS(20);	//延时20毫妙
}

void Select_sensitivity(void)
{
	//根据开关选择灵敏度
	anjian();
	count=anjian1+anjian2;
	switch(count) {
	case 0: { //标定用
		_pas0=0x51;   //PA3为TX
		_pas1=0x21;  //PA6为RX
		_isgs1=0;    //isink0关
		break;
	}
	case 1: {
		_pas0=0b00010001; //pa3//A0和A2做通讯
		_pas1=0b01000001; //pa6//PA4做STPB
		Temp=3;        //三级灵敏度
		break;
	}
	case 2: {
		_pas0=0b00010001; //pa3
		_pas1=0b01000001; //pa6
		Temp=6;        //二级灵敏度
		break;
	}
	case 3: {
		_pas0=0b00010001; //pa3
		_pas1=0b01000001; //pa6
		Temp=12;        //一级灵敏度（最低）
		break;
	}
	default:
		break;
	}
}



void PTMInit(void) //输入捕获初始化
{


	_pas10 = 0;
	_pas11 = 0;//PA4复用为PTCK
	_pac4 = 1;//输入
	_pa4 = 0;//默认电平
	_papu4=0;//上拉电阻

	_ptmc0 = 0b00100000;//PIM时钟 = sys / 16; PIM时钟 = 8MHz / 16 = 500kHz;
	_ptmc1 = 0b01000010;//捕捉输入模式,PTCK 上升沿输入捕捉，计数器值将锁存至 CCRA,
	_ptmc2 = 0x00;//捕捉输入模式时计数器清零条件选择位


	//_ptmrpl = 200;//2us * 100 = 200us
	//_ptmrph = 0;//ptmrph寄存器为8位，625 = 0000 0010 0111 0001; _ptmrph填入值，右移取高八位，即 0000 0010
	//	_ptmal = 25;//2us * 50 = 100us
	//	_ptmah = 0;// 占空比为1/4

	_ptmae = 1;//比较计数器A匹配中断使能
	_emi = 1;//使能总中断

	_pton = 1;//打开PTM定时器
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
	//延时一毫秒
	unsigned int c;
	for(c=0; c<t; c++)
		GCC_DELAY(1000);///4M时钟
}


void Twinkle(void)
{
	//闪灯
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