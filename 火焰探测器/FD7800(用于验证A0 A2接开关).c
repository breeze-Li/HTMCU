///C:\Program Files\Holtek MCU Development Tools\HT-IDE3000V8.x\BIN
//2021-9-8:输入捕获完成,单片机可以捕获到紫外管的上升沿脉冲.
//2021-9-9:解决昨日的死循环问题，可通过串口读取每秒脉冲数量。
//2021-9-11:整合通讯协议和输入捕获,下一步通过协议实现功能.
//2021年9月16日:完善写设置和读设置,设置施加电流保护.
//2021年9月28日13:51:58:将每(二/三)秒的脉冲总和作为一次数据发送.
//2021年10月9日14:34:53:开机读取EE的设置数据.(bit数据类型不可定义时初始化.)
//2021年10月14日:完善判据。
//2021年11月15日13:04:35:优化判据,有脉冲才开始判断,增加容错率。
//2021年11月18日：用A0和A2代替原来的串口接灵敏度选择开关。

#include "BA45F5542-2.h"

typedef unsigned char uc;
typedef unsigned int uint;

#define  alarm_output  _pa1         /*报警输出  io*/
#define  pulse_input  _pa4          /*检测脉冲输入*/

#define  OpenLED	_isgenc |= 0b10000010; 		///ISINK1 引脚灌电流使能,50mA
#define  CloseLED	 _isgenc &= 0b10000001;		///ISINK1 引脚灌电流除能
#define  ToggleLED	  _isgenc ^= 0b00000010;	///ISINK1 反转
#define  OpenISINK0	   _isgenc |= 0b10000001;	///ISINK0 引脚灌电流使能
#define  CloseISINK0	_isgenc &= 0b10000010;	///ISINK0 引脚灌电流除能

#define 	MIN_CURRENT	0X08					//电流最小值
#define		MAX_CURRENT 0X1F					//电流最大值

//中断向量
DEFINE_ISR(UARTIQR, 0x10);
DEFINE_ISR(STMAIQR, 0x2C);
DEFINE_ISR(PTMAIQR, 0x24);

//----------灌电流大小选择-----------------//
//nst unsigned int Hz[13]= {0x0e,0x10,0x11,0x12,0x13,0x14,0x16,0x18,0x19,0x1a,0x1c,0x1e,0x1f};
							//300,328, 336, 340, 344,  352, 356,360, 368, 376, 380, 388, 394  V
//const unsigned int sink_current_level[6]= {0x10,0x11,0x12,0x13,0x14,0x15};		//ISINK0 引脚灌电流大小


volatile unsigned char ComPC;		        //发送数据指针
volatile unsigned char ComCah[24];          //发送缓冲区
volatile unsigned char ComRPC;		        //接收数据指针
volatile unsigned char ComRCah[24];         //接收缓存
volatile unsigned char pointer_length;		//指针长度(记录接受的字节长度)
volatile unsigned int LifeCount,LifeTimer;  //寿命计时，单位：12小时
volatile unsigned char MyID[3];        		//产品ID

//----------统计巡检长度脉冲总和,每秒刷新-------------//
#define inspection_length 15							//巡检长度
volatile uc inspection_Array[inspection_length]={0};	//巡检数组

//volatile uc sampling_pointer;						//查询采样指针
//volatile uc num_of_pulses_per_sec[15] = {0};//15秒内每秒的脉冲个数

volatile bit  calibration_complete_flag;	//标定完成标志(标定完成后即可选择灵敏度)
volatile bit  receive_complete_flag;		//接收完成标志
volatile bit  read_sampling_flag;			//读采样标志
volatile bit  set_complete_flag;			//写入设置完成标志
volatile bit  trigger_flag;					//触发判据标志
volatile uc pulses_count=0;					//脉冲计数
volatile uc light_count=0;					//闪灯计数
volatile uc zero_pulses_count=0;			//连续零脉冲计数
volatile uc sensitivity_value=1;			//选择灵敏度值
volatile unsigned long temp;		        //临时数据
volatile uc count=0;
//----------判据  B个m秒时间窗内有A个m秒时间窗的脉冲总数大于n---//
volatile uc period=3;						//时间窗长度(m)
volatile uc pulses_per_period=3;			//每时间窗内的脉冲个数(n)
volatile uc alarm_period=3;					//报警的时间窗个数(A)
volatile uc continuous_period=5;			//连续的时间窗个数(B)
volatile uc period_count=0;					//时间长度计数
volatile uc alarm_period_count=0;			//报警的时间窗计数
volatile uc continuous_period_count=0;		//连续的时间窗计数
volatile uc continuous_period_count1=0;		//连续的时间窗计数1
//-----------存储地址定义---------------------------//
/*
0  1  2  3
4  5  6
7  8  9  A  B  C  D  E  F  10 11 
12 13 14 15 16 17 18 19 1A 1B 1C 
1D 1E 1F 20 01 22 23 24 25 26 27
*/
#define IDAdd 0x00          //设备ID  H+M+L+Xor
#define LifeHAdd 0x04	    //运行时间单位12小时 H+L+Xor
#define CALIBRATION_12M 0x07	    //12M标定数据(9个判据数据,1个电流数据,1个标志位)
#define CALIBRATION_17M 0x12	    //17M标定数据
#define CALIBRATION_24M 0x1D	    //24M标定数据
volatile uc CALIBRATION_ADD=CALIBRATION_12M;	//默认读设置地址

#define next 0x28	    //下一个

//----------函数声明------------------------//
void select_sensitivity(void);
void serial_communication(void);
void DelaymS(unsigned int t);
void PTMInit(void);
void UARTInit(void);
void provide_current_2500uS(void);
void Twinkle(void);
void Twinkles(uc i);
void SendByte(unsigned char);
void SetPack(void);
void SendData(unsigned char);
void UARTIQR(void);
void StmAInit(void);
void TestFlash(void);
void clear_sampling_buff(void);
void clear_inspection_Array(void);
void WR_SET(volatile uc CALIBRATION_ADD);
void RE_SET(volatile uc CALIBRATION_ADD,uc n);
unsigned char RD_EE(unsigned char RD_EE_addr);
void WR_EE_LAST(uc WR_EE_addr,uc DATA_addr,uc L);
void WR_EE(unsigned char WR_EE_addr,unsigned char WR_EE_data);
unsigned char Add_Array(volatile uc *p, uc data, uc len);


void main()
{
	_wdtc=0b01010101;	///2秒
	_scc=0x01;			///不分频(系统时钟为fH)
	_hircc=0x05;		///4M
	while(_hircf==0);   //4M时钟，不分频
	
//----------初始化-------------------------//
	PTMInit();			//输入捕捉初始化
	StmAInit();			//初始化STMA,用于串口定时
	UARTInit();			//串口初始化	
	
//----------灌电流发生器----------------------//
	_isgdata1=0x00;
	_isgdata0=0X0A;
	_isgen=1;
//-----------------变量初始化---------------------//
	volatile uint z=0;					//1S循环计用
	temp = 0;
	ComPC = 0;
	ComRPC = 0;
//	sampling_pointer = 0;
	calibration_complete_flag=0;
	receive_complete_flag = 0;
	set_complete_flag = 0;
	pulses_count = 0;
	trigger_flag=0;
	LifeTimer = 0;
	
	Twinkle();
	DelaymS(300);
	Twinkle();
	
	//----------------读取EE设置数据----------------------//
	SendByte(0x0A);
	temp=RD_EE(0x11);
	SendByte(temp);
	if(1==temp){
			SendByte(RD_EE(CALIBRATION_12M));
			SendByte(RD_EE(CALIBRATION_12M+1));
			SendByte(RD_EE(CALIBRATION_12M+2));
			SendByte(RD_EE(CALIBRATION_12M+3));
			SendByte(RD_EE(CALIBRATION_12M+9));
		//----------写入设置------------------//
		period=RD_EE(CALIBRATION_12M);
		pulses_per_period=RD_EE(CALIBRATION_12M+1);
		alarm_period=RD_EE(CALIBRATION_12M+2);
		continuous_period=RD_EE(CALIBRATION_12M+3);
		_isgdata0=RD_EE(CALIBRATION_12M+9);
	}
	//--------------------------------------//
	

	while(1) {
//		for(z=0; z < 15; z++){
//			temp = AddList(inspection_Array,z,period_count);
//			DelaymS(400);
//			SendByte(inspection_Array[0]);
//			SendByte(inspection_Array[1]);
//			SendByte(inspection_Array[2]);
//			SendByte(inspection_Array[3]);
//			SendByte(inspection_Array[4]);
//			SendByte(inspection_Array[5]);
//			SendByte(temp);
//			
//			GCC_CLRWDT();
//		}

//----------标定完成则用开关选择灵敏度------------------//
		_pac2=1;			//输入
		_pac0=1;
		_papu0=1;			//上拉
		_papu2=1;
		sensitivity_value=_pa0+(_pa2<<1);
		switch(sensitivity_value){
			//S1=S2=0，做串口
			case 0:count=1;//SendByte(0x11);
					//UARTInit();			//串口初始化
					break;
			
			//S1=1，S2=0，12米灵敏度
			case 1:count=2;
							//SendByte(0x12);
//								CALIBRATION_ADD=CALIBRATION_12M;
//								TestFlash();
							break;
			
			//S1=0，S2=1，17米灵敏度
			case 2:count=3;//SendByte(0x13);
//								CALIBRATION_ADD=CALIBRATION_17M;
//								TestFlash();
							break;
			
			//S1+S2=1，24米灵敏度
			case 3:count=4;//SendByte(0x14);
							/*CALIBRATION_ADD=CALIBRATION_24M;
							TestFlash();*/
							break;
		}
		Twinkles(count);
		SendByte(count);
		//Twinkle();
		DelaymS(1000);
//	if(sensitivity_value != _pa){
//		do{
//			//只执行一次
//			select_sensitivity();
//			}while(sensitivity_value != _pa);
//	}
	
//------------------------------------------------------//

#if 0
		if(set_complete_flag){//读取EE的设置数据
			set_complete_flag=0;
//			period=RD_EE(CALIBRATION_12M);
//			pulses_per_period=RD_EE(CALIBRATION_12M+1);
//			alarm_period=RD_EE(CALIBRATION_12M+2);
//			continuous_period=RD_EE(CALIBRATION_12M+3);
//			_isgdata0=RD_EE(CALIBRATION_ADD+9);		//灌电流大小
			SendByte(period);
			SendByte(pulses_per_period);
			SendByte(alarm_period);
			SendByte(continuous_period);
			SendByte(_isgdata0);
			clear_inspection_Array();
		}
		
		DelaymS(1000);
//		for(z=0; z < 400; z++){
//			//持续1秒的脉冲
//			OpenISINK0;			//打开ISINK0
//			GCC_DELAY(5);		//延时5微妙
//			CloseISINK0;		//关闭ISINK0
//			GCC_DELAY(2495);	//延时2495微妙
//		}
		
//		if(++light_count>5){
//			//6秒闪灯一次
//			Twinkle();
//			light_count=0;
//		}
		
//----------打包发送-------------------//
//		SetPack();
//		ComCah[4]=1;
//		ComCah[5]=pulses_count;
//		ComCah[6]=pulses_count ^ 1;
//		SendData(7);
//--------------------------------------//
/*		
	/*----------从有脉冲开始触发判据-------------/	
	if (pulses_count){						//如果有脉冲
		SendByte(0x89);
		zero_pulses_count=0;				//计数清零
		trigger_flag=1;						//开始触发判据
	}									
	else if(trigger_flag){	 				//如果没有脉冲且已经触发判据
		SendByte(0x99);
		//zero_pulses_count++;				//计数加一
		if (++zero_pulses_count>29){
			SendByte(0x90);
			zero_pulses_count=0;			//连续30秒没有脉冲
			trigger_flag=0;					//取消触发判据
			//----------判据计数清零-----------------//
			period_count=0;
			alarm_period_count=0;
			continuous_period_count=0;
			continuous_period_count1=0;
		}									
	}
*/		
		
		
	if(trigger_flag){				//开始触发判据
		
		//--------period_count循环-----------------//
		period_count++;
		period_count=period_count % period;
		//-----------------------------------------//
		temp = Add_Array(inspection_Array,pulses_count,period);//添加数据
		
		//单个时间窗满
		if(period_count==0){
			//---------continuous_period_count循环--------------//
			continuous_period_count++;
			continuous_period_count=continuous_period_count%continuous_period;
			
			//时间窗脉冲数达到报警值,A+1
			if(temp >= pulses_per_period){
				alarm_period_count++;
			};
		};
		
//			SendByte(inspection_Array[0]);
//			SendByte(inspection_Array[1]);
//			SendByte(inspection_Array[2]);
//			SendByte(inspection_Array[3]);
//			SendByte(inspection_Array[4]);
//			SendByte(inspection_Array[5]);
//			SendByte(temp);
//			SendByte(alarm_period_count);
//			SendByte(period_count);
//			SendByte(continuous_period_count);
		
		if(alarm_period_count>=alarm_period){
			//报警
			OpenLED
			while(1){
				serial_communication();
				GCC_CLRWDT();
			}
		}
		//连续时间窗满(B),A计数清零
		if(continuous_period_count == 0 && period_count== 0 ){
			//在B满,且m满的那一秒末清零，因为m从1开始计数，
			//所以所有数据都向后移动了一位。
			//因此在B满的下一秒清零。所以B和M都为零。
			//++continuous_period_count1;
			//if(continuous_period_count1 == 4-period){ 
				//continuous_period_count1=0;
				clear_inspection_Array();
				alarm_period_count=0;
				SendByte(0xff);
			//	}
		}
	}	
		
	pulses_count=0;					//脉冲数清零

#endif
		serial_communication();//串口通讯函数
		GCC_CLRWDT();
	}	
}

void serial_communication(void){
	//串口通讯函数
	volatile uc j,k;
	if(receive_complete_flag) {
		receive_complete_flag=0;
		switch(1) {
			//-------检查数据头,错误就退出---------//
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
			if(pointer_length>=7 && pointer_length==ComRCah[4]+6) { //指令长度正确
				//----------指令异或校验----------------------------//
				j=ComRCah[4];
				for(k=0; k<ComRCah[4]; k++)
					j=j ^ ComRCah[5+k];
				//----------------------------------------------//
				if(j==ComRCah[ComRCah[4]+5]) {	//指令校验正确
					switch(ComRCah[5]) {		//指令类型
					case 0x80:					//读在线
						if(ComRCah[4]==1) {
							SetPack();
							ComCah[4]=1;
							ComCah[5]=0x80;
							ComCah[6]=0x81;
							SendData(7);
						}
						break;
					case 0x81:					//写ID
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
					case 0x84:					//读ID
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
					case 0x88:					//写使用时间
						if(ComRCah[4]==3) {
							WR_EE(LifeHAdd,ComRCah[6]);
							WR_EE(LifeHAdd+1,ComRCah[7]);
							WR_EE(LifeHAdd+2,ComRCah[6]+ComRCah[7]);
							WR_EE(LifeHAdd+0x20,ComRCah[6]);
							WR_EE(LifeHAdd+0x21,ComRCah[7]);
							WR_EE(LifeHAdd+0x22,ComRCah[6]+ComRCah[7]);
							TestFlash();
							LifeTimer=0;		//1S计时清零
							SetPack();
							ComCah[4]=1;
							ComCah[5]=0x88;
							ComCah[6]=0x89;
							SendData(7);
						}break;
					case 0x87:					//读使用时间
						if(ComRCah[4]==1) {
							SetPack();
							ComCah[4]=3;
							ComCah[5]=0x87;
							ComCah[6]=LifeCount	>> 8;
							ComCah[7]=(unsigned char)(LifeCount & 0xff);
							ComCah[8]=ComCah[7] ^ ComCah[6] ^ ComCah[5] ^ 3;
							SendData(9);
						}
						break;
					case 0x90:					//设置12M灵敏度
						if(ComRCah[4]==0x0B) {
							CALIBRATION_ADD=CALIBRATION_12M;
							WR_SET(CALIBRATION_ADD);
							WR_EE(CALIBRATION_ADD+10, 1);	///写入EE标志
							set_complete_flag=1;
							TestFlash();
							SetPack();
							ComCah[4]=1;
							ComCah[5]=0x90;
							ComCah[6]=0x91;
							SendData(7);
						}
						break;
					case 0x91:					//设置17M灵敏度
						if(ComRCah[4]==0x0B) {
							CALIBRATION_ADD=CALIBRATION_17M;
							WR_SET(CALIBRATION_ADD);
							set_complete_flag=1;
							TestFlash();
							SetPack();
							ComCah[4]=1;
							ComCah[5]=0x91;
							ComCah[6]=0x90;
							SendData(7);
						}
						break;
					case 0x92:					//设置24M灵敏度
						if(ComRCah[4]==0x0B) {
							CALIBRATION_ADD=CALIBRATION_24M;
							WR_SET(CALIBRATION_ADD);
							set_complete_flag=1;
							TestFlash();
							SetPack();
							ComCah[4]=1;
							ComCah[5]=0x92;
							ComCah[6]=0x93;
							SendData(7);
						}
						break;
						//----------三种情况均为读设置----------------//
					case 0x93:
					case 0x94:
					case 0x95:
						if(ComRCah[4]==0x01) {
							switch(ComRCah[5]) {
							//----------选择读哪个灵敏度的设置-----------------//
							case 0x93: CALIBRATION_ADD = CALIBRATION_12M;break;
							case 0x94: CALIBRATION_ADD = CALIBRATION_17M;break;
							case 0x95: CALIBRATION_ADD = CALIBRATION_24M;break;
							}
							RE_SET(CALIBRATION_ADD, ComRCah[5]);
							SendData(17);
						}
						break;
					case 0x97:				//点火
						//clear_sampling_buff();
						calibration_complete_flag=1;
						SendByte(0xFF);
						SendByte(_isgdata0);
						break;
					case 0x96:				//读采样
						if(ComRCah[4]==0x01) {
							//接受到读采样标志，清零采样缓冲区。等待15个数据满了之后再
							//发送给上位机
							read_sampling_flag=1;			//读采样标志
							SetPack();
							ComCah[4]=1;
							ComCah[5]=0x96;
							ComCah[6]=0x97;
							SendData(7);
//								SetPack();
//								ComCah[4]=16;
//								ComCah[5]=0x96;
//								ComCah[21]=0x96 ^ 16;
//								//-----------采样数据送入发送缓存区---------------//
//								for(i=0; i<15; i++) {
//									ComCah[6+i] = num_of_pulses_per_sec[i];
//									ComCah[21] ^= ComCah[6+i];
//									//清除采样数据缓冲区
//									num_of_pulses_per_sec[i] = 0xff;
//									};
//								SendData(22);			//发送数据	
//								//--------------------------------------//
						}
						break;
					case 0x98 :				//复位
						SendByte(0xFB);
						DelaymS(8000);
						break;
					}
				}
			}
		}
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

void StmAInit(void)
{
	//初始化STMA,用于串口定时
	_stmc0 = 0b01000000;//TIM0时钟 = fusb ; TIM0时钟 = 32kHz ;
	_stmc1 = 0xc1;		//定时器模式、比较计数器A匹配清0计数器(STMA中断后就不用单独清计数器了) 计数器计数一次为(1/32k)S
	_stmal = 64;		// 2ms定时
	_stmah = 0;			//
	
	_stmae = 1;			//比较计数器A匹配中断使能
	_emi = 1;			//使能总中断
}

void PTMInit(void) //输入捕获初始化
{
	_pas10 = 0;
	_pas11 = 0;		//PA4复用为PA4/PTCK
	_pac4 = 1;		//输入
	_pa4 = 0;		//默认低电平
	_papu4=0;		//除能上拉电阻

	_ptmc0 = 0b00100000;//PIM时钟 = FH/16 = 4M/16;
	_ptmc1 = 0b01000010;//捕捉输入模式,PTCK 上升沿输入捕捉，计数器值将锁存至 CCRA
	_ptmc2 = 0x00;//捕捉输入模式时计数器清零条件选择位

	_ptmae = 1;//比较计数器A匹配中断使能
	_emi = 1;//使能总中断

	_pton = 1;//打开PTM定时器
}

//----------中断类函数-----------------------//
void UARTIQR(void)
{
	//接受中断函数
	//RxFlag=1;
	_usimf=0;
	_ston=1;				//打开定时器
	while(_urxif>0) {
		ComRCah[ComRPC]=_utxr_rxr;
		if(ComRPC<24)
			ComRPC++;
		else
			ComRPC=0;
	}

	//----------------清零定时器------------
	_ston=0;
	GCC_DELAY(50);//这个延时很关键,无此不可清除计数器
	_ston=1;
	//--------------------------------------
}

void STMAIQR(void)
{	
	_stmaf=0;					//清除中断标志
	receive_complete_flag=1;	//传输完成标志置1
	pointer_length=ComRPC;		//记录指针长度
	ComRPC=0;					//接受指针置零
	_ston=0;					//关闭STMA计数器
}

void PTMAIQR(void)
{	//捕捉到脉冲
	_ptmaf=0;
	pulses_count++; //计数加1
}

//----------一般自定义函数---------------------//
void select_sensitivity(void){
	//选择灵敏度
	_uucr1=0x00;
	_uucr2=0x00;
//   	_pas06=0;			//PA3做PA3
//  	_pas15=0;			//PA6做PA6
	_pac2=1;			//输入
	_pac0=1;
	_papu0=1;			//上拉
	_papu2=1;			
	
	if(sensitivity_value != _pa){
		switch(_pa){
			//S1=S2=0，做串口
			case 0:count=1;
					UARTInit();			//串口初始化
					break;
			
			//S1=1，S2=0，12米灵敏度
			case 0b00000001:count=2;
//								CALIBRATION_ADD=CALIBRATION_12M;
//								TestFlash();
							break;
			
			//S1=0，S2=1，17米灵敏度
			case 0b00000100:count=3;
//								CALIBRATION_ADD=CALIBRATION_17M;
//								TestFlash();
							break;
			
			//S1+S2=1，24米灵敏度
			case 0b00000101:count=4;
							/*CALIBRATION_ADD=CALIBRATION_24M;
							TestFlash();*/
							break;
		}
		sensitivity_value=_pa;
	}
	Twinkles(count);
}
	

void DelaymS(unsigned int t)
{	//延时一毫秒
	volatile unsigned int c;
	for(c=0; c<t; c++) GCC_DELAY(1000);///4M时钟
}

void Twinkle(void)
{	//闪灯
	OpenLED;
	GCC_DELAY(5000);  //5mS
	CloseLED;
}

void Twinkles(uc i){
	while(i--){
		Twinkle();
		DelaymS(200);	
	}
	
//	switch(i){
//		case 1:	OpenLED;
//				GCC_DELAY(50000);  //5mS
//				CloseLED;
//		case 2:	OpenLED;
//				GCC_DELAY(100000);  //5mS
//				CloseLED;
//		case 3:	OpenLED;
//				GCC_DELAY(150000);  //5mS
//				CloseLED;
//		case 4:	OpenLED;
//				GCC_DELAY(200000);  //5mS
//				CloseLED;
//		}
	}
	
void SetPack(void)
{	//发送数据头
	ComCah[0]=0xcc;
	ComCah[1]=0x99;
	ComCah[2]=0x99;
	ComCah[3]=0xcc;
}

void SendByte(unsigned char l)
{	//发送1个字节
	_utxr_rxr=l;
	while(_utxif==0);
	while(_utidle==0);
}

void SendData(unsigned char L)
{	//发送L个字节
	unsigned char i;
	for(i=0; i<L; i++) {
		_utxr_rxr=ComCah[i];
		while(_utxif==0);
		while(_utidle==0);
	}
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

void RE_SET(volatile uc CALIBRATION_ADD,uc n)
{	//读设置用。参数:储存设置的首地址，指令编号。
	uc i;						//
	SetPack();					//设置头
	ComCah[4] = 0X0b;			//指令编号
	ComCah[5] = n;
	ComCah[16]=0x0B ^ n;		//校验4 5两位
	for(i=0; i<10; i++) {
		ComCah[6+i] = RD_EE(CALIBRATION_ADD+i);		//第六位开始放置读取的数据
		ComCah[16] ^= ComCah[6+i];					//继续校验6-10位
	}
}

void WR_SET(volatile uc CALIBRATION_ADD){
	//写入设置用。储存设置的首地址
	//连续写入9个设置,第10个电流大小需要判断
	WR_EE_LAST(CALIBRATION_ADD,6,9);
	//电流保护
	//--如果电流小于最小值,则写入最小值,大于最大值,写入最大值,否则写入标定值--//
	if(ComRCah[15]<MIN_CURRENT) 
		WR_EE(CALIBRATION_ADD+9,MIN_CURRENT);
	else if(ComRCah[15]>MAX_CURRENT)
		WR_EE(CALIBRATION_ADD+9,MAX_CURRENT);
	else WR_EE(CALIBRATION_ADD+9,ComRCah[15]);
	//-----------------------------------------------------------------------//	
	}
	
void WR_EE_LAST(uc WR_EE_addr,uc DATA_addr,uc L)
{	//连续写，EE地址，ComRCah中待写入数据的地址，写入长度。
	unsigned char i;
	for(i=0; i<L; i++) {
		WR_EE(WR_EE_addr+i,ComRCah[DATA_addr+i]);
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
	//取运行时间
	TempCah[0]=RD_EE(LifeHAdd);
	TempCah[1]=RD_EE(LifeHAdd+1);
	TempCah[2]=RD_EE(LifeHAdd+2);
	TempCah[3]=TempCah[0] + TempCah[1];
	if(TempCah[2]==TempCah[3]) {
		LifeCount=TempCah[0];
		LifeCount=(LifeCount<<8)+TempCah[1];
	} else {
		TempCah[0]=RD_EE(LifeHAdd+0x20);
		TempCah[1]=RD_EE(LifeHAdd+0x21);
		TempCah[2]=RD_EE(LifeHAdd+0x22);
		TempCah[3]=TempCah[0] + TempCah[1];
		if(TempCah[2]==TempCah[3]) {
			LifeCount=TempCah[0];
			LifeCount=(LifeCount<<8)+TempCah[1];
		} else
			LifeCount=0;
	}
	//设置灵敏度
	if(1==RD_EE(0x11)){
		//----------写入设置------------------//
		period=RD_EE(CALIBRATION_ADD);
		pulses_per_period=RD_EE(CALIBRATION_ADD+1);
		alarm_period=RD_EE(CALIBRATION_ADD+2);
		continuous_period=RD_EE(CALIBRATION_ADD+3);
		_isgdata0=RD_EE(CALIBRATION_ADD+9);
	}
}

//void clear_sampling_buff(void){
//	//清除采样数据缓冲区
//	uc i=14;
//	while(i--) num_of_pulses_per_sec[i]=0xff;
//	sampling_pointer=0;			//指针清零
//	}

void clear_inspection_Array(void){
	//清除巡检数组
	uc i=14;
	while(i--) inspection_Array[i]=0x00;
	continuous_period_count=0;			//计数清零
	period_count=0;
	}

uc Add_Array(volatile uc *p, uc data, uc len){
	//数组数据前移,末尾添加data.返回len长度总数
	volatile uc i,num;
	for(i=0,num=0; i<len-1; i++){
		p[i]=p[i+1];
		num = num+p[i];
		}
	//循环之后i=len-1;即最后一位
	p[i] = data;
	num = num + data;
	return num;
	}
