///C:\Program Files\Holtek MCU Development Tools\HT-IDE3000V8.x\BIN
//2021-9-8:输入捕获完成,单片机可以捕获到紫外管的上升沿脉冲.
//2021-9-9:解决昨日的死循环问题，可通过串口读取每秒脉冲数量。
//2021-9-11:整合通讯协议和输入捕获,下一步通过协议实现功能.
//2021年9月16日:完善写设置和读设置,设置施加电流保护.
//2021年9月28日13:51:58:将每(二/三)秒的脉冲总和作为一次数据发送.
//2021年10月9日14:34:53:开机读取EE的设置数据.(bit数据类型不可定义时初始化.)
//2021年10月14日:完善判据。
//2021年11月15日13:04:35:优化判据,有脉冲才开始判断,增加容错率。
//2021年11月18日：用A0和A2代替原来的串口接灵敏度选择开关。(不要直接读_pa,
//				  因为串口也是Pa口,TxRx接上后会变化,直接读pa要或上0,2两位。
//				  由此烧录时拨码开关必须全部拨下)
//2021年11月19日:灵敏度选择完成,至此,程序基本完成。
//2022年2月25日：添加寿命到期
//2022年3月31日：列队形式判据（前面都错了......）
//2022年6月21日：内置初始判据，省略第一次标定
//2022年9月30日：报警后利用gengral_array记录最后20秒数据（复盘自报数据），并写入EE，记录每种灵敏度的报警触发判据

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

#define     TYPE 0x12
#define     VERSION 0x10

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
//----------统计巡检长度脉冲总和,每秒刷新-------------//
#define inspection 5
#define statistics 30
volatile uc inspection_Array[inspection] = {0};	//巡检数组（时间窗脉冲总数）
volatile uc statistics_Array[statistics] = {0};	//统计数组(报警判断)
volatile uc gengral_Array[20] = {0};	//普通数组(记录最后30秒数据，写入EE)
//volatile uc sampling_pointer;						//查询采样指针
//volatile uc num_of_pulses_per_sec[15] = {0};//15秒内每秒的脉冲个数

volatile bit  receive_complete_flag;		//接收完成标志
volatile bit  read_sampling_flag;			//读采样标志
volatile bit  set_complete_flag;			//写入设置完成标志
volatile bit  trigger_flag;					//触发判据标志
volatile bit  fire_flag;					//火警标志
volatile bit  fire_flag1;					//火警标志
volatile bit  calibration_flag;				//标定标志
volatile bit  gameover;						//生命到期
volatile unsigned int LifeCount,LifeTimer;  //寿命计时，单位：12小时
volatile unsigned char MyID[3];        		//产品ID
volatile uc pulses_count=0;					//脉冲计数
volatile uc light_count=0;					//闪灯计数
volatile uc zero_pulses_count=0;			//连续零脉冲计数
volatile uc sensitivity_value=4;			//选择灵敏度值（初始化为0123之外的值）
volatile uc sensitivity_value_old=4;		//选择灵敏度旧值（以便上电选择灵敏度）
volatile unsigned long temp;		        //临时数据
//----------判据1  B个m秒时间窗内有A个m秒时间窗的脉冲总数大于n---//
volatile uc period=1;						//时间窗长度(m)
volatile uc pulses_per_period=2;			//每时间窗内的脉冲个数(n)
volatile uc alarm_period=2;					//报警的时间窗个数(A)
volatile uc continuous_period=10;			//连续的时间窗个数(B)
volatile uc period_count=0;					//时间窗长度计数
volatile uc alarm_period_count=0;			//报警的时间窗计数
volatile uc continuous_period_count=0;		//连续的时间窗计数
volatile uc continuous_period_count1=0;		//连续的时间窗计数1
//----------判据2  ----------------------------//
volatile uc count=0;						//判据2时间计数
volatile uc total_pause =10;				//总脉冲
volatile uc total_pause_time =15;				//总脉冲
volatile uc total_pause_count =0;			//总脉冲计数
//volatile uc total_time=0;

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

#define ERROR 0x28	    //标志
#define DATA 0x2B		//30miao数据

//----------函数声明------------------------//

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
//-----------------变量初始化---------------------//
	volatile uint z=0;					//1S循环计用
	volatile uc life[3]={0};
	volatile uc i=0,j=0;					//判断报警数用
//----------初始化-------------------------//
	PTMInit();			//输入捕捉初始化
	StmAInit();			//初始化STMA,用于串口定时
	UARTInit();			//串口初始化
	MCUinit();
	Twinkle();
	DelaymS(200);		//延时写入灵敏度
	Init_Criterion();
	TestFlash();
	while(1) {
		if(set_complete_flag){//读取EE的设置数据
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
		
//----------非标定状态下开关变化时或写入设置时选择灵敏度------------------//
		if(!calibration_flag)
			select_sensitivity();
//------------------------------------------------------//

		for(z=0; z < 50; z++){
			//持续1秒的脉冲
			OpenISINK0;			//打开ISINK0
			GCC_DELAY(5);		//延时5微妙
			CloseISINK0;		//关闭ISINK0
			GCC_DELAY(19995);	//延时2495微妙
		}
		
		if(++LifeTimer > 43200) { //43200 12hours
            LifeTimer = 0;

            if(LifeCount <= 2160) { //2160 3years
                LifeCount++;
                //存储时间
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
			//5秒闪灯，寿命到期闪两次
			if(gameover){
				Twinkles(2);
			}else {
				Twinkle();
			}
			light_count=0;
//			fire_flag=1;
//			trigger_flag=1;
		}
		
//----------标定状态下打包发送-------------------//
/*		SendByte(pulses_count);*/
		if(calibration_flag){
			SetPack();
			ComCah[4]=1;
			ComCah[5]=pulses_count;
			ComCah[6]=pulses_count ^ 1;
			SendData(7);
		}
//--------------------------------------//
	
	
	Add_gengral(pulses_count);//记录
	/*----------从有脉冲开始触发判据-------------*/	
	if (pulses_count){						//如果有脉冲
		//SendByte(0x89);
		zero_pulses_count=0;				//计数清零
		trigger_flag=1;						//开始触发判据
	}									
	else if(trigger_flag){	 				//如果没有脉冲且已经触发判据
		//SendByte(0x99);
		//zero_pulses_count++;				//计数加一
		if (++zero_pulses_count>30){
			//SendByte(0x90);
			zero_pulses_count=0;			//连续15秒没有脉冲
			trigger_flag=0;					//取消触发判据
			//----------判据计数清零-----------------//
			period_count=0;
			alarm_period_count=0;
			continuous_period_count=0;
			//continuous_period_count1=0;
		}									
	}	
	if(trigger_flag && (!calibration_flag)){	//非标定状态开始触发判据
		//判据2 nS内脉冲总数
		
		//Add_gengral(pulses_count);
		if(++count < total_pause_time){
			total_pause_count += pulses_count;
			if(total_pause_count>=total_pause) fire_flag1=1;
			}else{
				count=0;
				total_pause_count=0;
		}
		
		//--------period_count循环-----------------//
		period_count++;
		period_count=period_count % period;
		//-----------------------------------------//
		temp = Add_Array(inspection_Array,pulses_count,period);//添加数据
		
		//单个时间窗满
		if(period_count==0){
			//---------continuous_period_count循环--------------//
			//continuous_period_count++;
			//continuous_period_count=continuous_period_count%continuous_period;
			
			//时间窗总数加入统计数组
			Add_Array(statistics_Array, temp, continuous_period);
			for(i=0; i<continuous_period; i++){
				//SendByte(statistics_Array[i]);
				//判断时间窗脉冲数是否达到报警值,A+1
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
			alarm_period_count=0;//清除报警计数，不然会累加
		}
		if(fire_flag || fire_flag1) {
			//记录报警情况
			j = (fire_flag<<4) + fire_flag1;
			switch(sensitivity_value){
			
			case 3://S1=S2=0，12米灵敏度
					WR_EE(ERROR, j);break;
			case 2://S1=1，S2=0，17米灵敏度
			case 1: 
					WR_EE(ERROR+1, j);break;		
			
			case 0://S1=S2=1，24米灵敏度
					WR_EE(ERROR+2, j);break;
			}
			//记录20S数据
			for(i=0;i<20;i++){
				WR_EE(DATA+i,gengral_Array[i]);
				SendByte(gengral_Array[i]);
			}
			Fire_Alarm();
		}

//		//连续时间窗满(B),A计数清零
//		if(continuous_period_count == 0 && period_count== 0 ){
//			//在B满,且m满的那一秒末清零，因为m从1开始计数，
//			//所以所有数据都向后移动了一位。
//			//因此在B满的下一秒清零。所以B和M都为零。
//			//++continuous_period_count1;
//			//if(continuous_period_count1 == 4-period){ 
//				//continuous_period_count1=0;
//				//clear_inspection_Array();
//				alarm_period_count=0;
//				//SendByte(0xff);
//			//	}
//		}
	}
		pulses_count=0;		   //脉冲数清零
		//serial_communication();//串口通讯函数
		GCC_CLRWDT();
	}	
}

void serial_communication(void){
	//串口通讯函数
	volatile uc i,j,k;
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
					case 0x87:					//读使用时间
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
					case 0x88:					//写使用时间
						if(ComRCah[4]==3) {
							WR_EE(LifeHAdd,ComRCah[6]);
							WR_EE(LifeHAdd+1,ComRCah[7]);
							WR_EE(LifeHAdd+2,ComRCah[6]+ComRCah[7]);
							TestFlash();
							LifeTimer=0;		//每1S计时清零
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
							clear_inspection_Array();
						}
						break;
					case 0x91:					//设置17M灵敏度
						if(ComRCah[4]==0x0B) {
							CALIBRATION_ADD=CALIBRATION_17M;
							WR_SET(CALIBRATION_ADD);
							WR_EE(CALIBRATION_ADD+10, 1);	///写入EE标志
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
					case 0x92:					//设置24M灵敏度
						if(ComRCah[4]==0x0B) {
							CALIBRATION_ADD=CALIBRATION_24M;
							WR_SET(CALIBRATION_ADD);
							WR_EE(CALIBRATION_ADD+10, 1);	///写入EE标志
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
						calibration_flag=1;  //标定标志置一
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
						//报警
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

//----------初始化类函数----------------------//
void MCUinit(void){
	_wdtc=0b01010101;	///2秒
	_scc=0x01;			///不分频(系统时钟为fH)
	_hircc=0x05;		///4M
	while(_hircf==0);   //4M时钟，不分频
	//----------开关引脚-----------------//
	_pac1=0;			//输出
	_pa1 = 0;
	_pac2=1;			//输入
	_pac0=1;
	_papu0=1;			//上拉
	_papu2=1;
	//----------灌电流发生器----------------------//
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
	serial_communication();
}

void PTMAIQR(void)
{	//捕捉到脉冲
	_ptmaf=0;
	pulses_count++; //计数加1
}

//----------一般自定义函数---------------------//
void Fire_Alarm(void){
	//报警
	_isgs0 = 0;
	OpenLED				//亮灯
	_papu1=1;			//上拉PA1
	alarm_output=1;		//PA1输出1
	while(1){
		serial_communication();
		GCC_CLRWDT();	
	}
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

void select_sensitivity(void){
	//选择灵敏度开关值
	sensitivity_value=_pa0+(_pa2<<1);

	if(sensitivity_value != sensitivity_value_old){
		switch(sensitivity_value){
		/*
		 A0		A2		S1		S2		sensitivity_value
		 1		1		0		0		3 12M
		 0		1		1		0		2 17M
		 1		0		0		1		1 17M
		 0		0		1		1		0 24M		*/

		//S1=S2=0，12米灵敏度
		case 3:
				CALIBRATION_ADD=CALIBRATION_12M;
				break;
		//S1=1，S2=0，17米灵敏度
		case 2:
				CALIBRATION_ADD=CALIBRATION_17M;
				break;
		//S1=0，S2=1，17米灵敏度
		case 1: 
				CALIBRATION_ADD=CALIBRATION_17M;
				break;		
		//S1=S2=1，24米灵敏度
		case 0:
				CALIBRATION_ADD=CALIBRATION_24M;
				break;
		}
		TestFlash();
		sensitivity_value_old=sensitivity_value;	
	}	
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
}

void TestFlash(void)
{	uc i=2;
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
	} 
	else LifeCount=0;
	//设置灵敏度
	//已经标定则写入,否则亮灯1秒提示该灵敏度未标定
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

void write_criteria(void){//写入判据
		period=RD_EE(CALIBRATION_ADD);
		pulses_per_period=RD_EE(CALIBRATION_ADD+1);
		alarm_period=RD_EE(CALIBRATION_ADD+2);
		continuous_period=RD_EE(CALIBRATION_ADD+3);
		_isgdata0=RD_EE(CALIBRATION_ADD+9);
		total_pause=RD_EE(CALIBRATION_ADD+4);
		total_pause_time = RD_EE(CALIBRATION_ADD+5);
	
		//改变旧键值，用于下个循环恢复拨码开关的灵敏度
		sensitivity_value_old=4;		
}
		
void clear_inspection_Array(void){
	//清除巡检数组
	uc i;
	for(i=0; i< inspection; i++){
		inspection_Array[i]=0x00;
	}
	//continuous_period_count=0;			//计数清零
	period_count=0;
	for(i=0; i< statistics; i++){
		statistics_Array[i]=0x00;
	}	
}

uc Add_Array(volatile uc *p, uc data, uc len){
	//数组数据前移,末尾添加data.返回len长度总和
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
