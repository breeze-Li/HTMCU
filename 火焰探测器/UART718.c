
//FD7800的通讯协议，已完成。接受一条完整的指令后校准并执行，对应协议为FD7800调试协议

#include "BA45F5542.h"

/*#define FD718*/
#define SD628

#define  OpenLED    {_isgenc |= 0b10000010;_isgdata1=0b00000000;}	///ISINK1 引脚灌电流使能,50mA
#define  CloseLED   _isgenc &= 0b10000001;		///ISINK1 引脚灌电流除能
#define  reverseLED   _isgenc ^= 0b10000010;	///ISINK1 反转

typedef unsigned char uc;
typedef unsigned int uint;



//定义中断向量
DEFINE_ISR(UART, 0x10);
DEFINE_ISR(STMAIQR, 0x2C);


volatile unsigned char ComPC;		        //发送数据指针
volatile unsigned char ComCah[24];          //发送缓冲区
volatile unsigned char ComRPC;		        //接收数据指针
volatile unsigned char ComRCah[24];         //接收缓存
volatile unsigned char pointer_length;		//指针长度(记录接受的字节长度)
volatile unsigned int LifeCount,LifeTimer;  //寿命计时，单位：12小时
volatile unsigned char MyID[3];        		//产品ID

volatile uc num_of_pulses_per_sec[15] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15};		//15秒内每秒的脉冲个数
volatile uc sampling_pointer;				//查询采样指针
//volatile uc num_of_pulses_per_sec_bak[15];	//15秒内每秒的脉冲个数备份
//volatile uc sampling_pointerc_bak;			//查询备份采样指针

volatile bit  RxFlag;       				//串口有新数据标志
volatile bit  XORFlag;       				//接受数据是否为指令标志
volatile bit  transfer_complete_flag;		//接收传输完成标志
volatile bit  read_sampling_flag;			//读采样标志
volatile unsigned long temp;		        //临时数据
//volatile unsigned int sec;					//串口毫秒计时


//-----------存储地址定义---------------------------//
/*
0  1  2  3
4  5  6
7  8  9  A  B  C  D  E  F  10
11 12 13 14 15 16 17 18 19 1A
1B 1C 1D 1E 1F 20 01 22 23 24
25
*/
#define IDAdd 0x00          //设备ID  H+M+L+Xor
#define LifeHAdd 0x04	    //运行时间单位12小时 H+L+Xor
#define CALIBRATION_12M 0x07	    //12M标定数据
#define CALIBRATION_17M 0x11	    //17M标定数据
#define CALIBRATION_24M 0x1B	    //24M标定数据
#define next 0x25	    //下一个



void DelaymS(unsigned int t)
{
	//延时T毫秒
	unsigned int i;
	for(i=0; i<t; i++)
		GCC_DELAY(1000);
}

void Twinkle(void)
{
	//闪灯
	OpenLED;
	DelaymS(5);  //5mS
	CloseLED;
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
		_utxr_rxr=ComCah[i];
		while(_utxif==0);
		while(_utidle==0);
	}
}

void SendByte(unsigned char l)
{
	//发送1个字节
	_utxr_rxr=l;
	while(_utxif==0);
	while(_utidle==0);
}

void UART(void)
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
	GCC_DELAY(200);//这个延时很关键,无此不可清除计数器
	_ston=1;
	//--------------------------------------
}

void STMAIQR(void)
{	/*
	  最开始想的是，接受一字节前开定时器，一字节接受完成后清计数器再开定时器。CCRA定一个
	  大于接受一字节的时间,如果最后一字节也接受完成了则计数器不在清零，就会产生CCRA中断。
	  第一次写就奇怪,波特率9600时,传输一字节只需要1MS,但是这里非得给10MS才能正常接受
	  (当时是7字节),8-24日写长数据接受时又有问题,只接受了12个数据,操,原来是没有正常清零
	  计数器,所以10MS只能接受12个[0.01/(8/9600)=12],后面还有数据的话指针会从0开始接受。
	  正常清零计数器后sec的值就正常了,大于1MS即可，这才是最开始想实现的功能。
	  8-26:把CCRA的值变成64即可实现2MS定时，不需要sec变量了，也减少了中断次数和中断函数的处理量
	*/
//	static unsigned int sec = 0;
	_stmaf=0;						//清除中断标志
	//if(++sec>2) {					//3毫秒，传输完成，关闭定时器
		//sec=0;						//清除毫秒计数
	transfer_complete_flag=1;	//传输完成标志置1
	
	pointer_length=ComRPC;		//记录指针长度
	ComRPC=0;					//接受指针置零
	_ston=0;					//关闭STMA计数器
	Twinkle();
	//}
}

void shift_data(unsigned char n)
{
	//接受缓存数据转移到发送缓存
	unsigned char i;
	for(i=0; i<n; i++) {
		ComCah[i]=ComRCah[i];
	}
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

	_ston = 0;			//关闭定时器
}

void UARTInit(void)
{
	//初始化UART
	_umd=1;				//做串口
	_uucr1=0x80;
	_uucr2=0b11101100;
	_ubrg=25;			//4M时钟时波特率9600
	#ifdef SD628
	// --------------SD628-----------------
	_pbs03=1;			//PB1做TX
	_pbs07=1;			//PB3做RX
	_pbpu3=1;			//pb3上拉
	_ifs01=1;			//RX 输入源引脚为PB3
	#endif
	#ifdef FD718
	// --------------FD718-----------------
   _pas06=1;			//PA3做TX
   _pas15=1;			//PA6做RX
   _papu6=1;			//pa6上拉
   _ifs00=1;			//RX 输入源引脚为Pa6
	//-------------------------------------
	#endif
	
	_usime=1;			//USIM中断使能
	_emi=1;				//总中断
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

void WR_EE_LAST(uc WR_EE_addr,uc DATA_addr,uc L)
{
	//连续写，EE地址，ComRCah中待写入数据的地址，写入长度。
	unsigned char i;
	for(i=0; i<L; i++) {
		WR_EE(WR_EE_addr+i,ComRCah[DATA_addr+i]);
	}
}

void RE_SET(volatile uc CALIBRATION_ADD,uc n)
{
	//读设置用。储存设置的首地址，指令编号。
	uc i;						//指令编号
	SetPack();					//设置头
	ComCah[4] = 0X0b;			//
	ComCah[5] = n;
	ComCah[16]=0x0B ^ n;		//校验4 5两位
	for(i=0; i<10; i++) {
		ComCah[6+i] = RD_EE(CALIBRATION_ADD+i);		//第六位开始放置读取的数据
		ComCah[16] ^= ComCah[6+i];					//继续校验6-10位
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
}

void clear_sampling_buff(void){
	//清除采样数据缓冲区
	uc i=15;
	while(i--) num_of_pulses_per_sec[i]=0xff;
	sampling_pointer=0;			//指针清零
	}
	


void main()
{
	unsigned char i,j,k;


	_wdtc=0x57;					//8秒狗
	_scc=0x01;
	_hircc=0b00000101;
	while(_hircf==0);	//4M时钟

	StmAInit();				//初始化STMA,用于串口定时
	UARTInit();				//初始化UART


	//-----------------变量初始化---------------------//
	RxFlag=0;
	ComPC=0;
	ComRPC=0;
	XORFlag=0;
	transfer_complete_flag=0;
	LifeTimer=0;

	//SetPack();

//	_pas0=0b00000000;
//   _pas1=0b10010010;	///PA7用AN1 //_pas1=0b00010010;
//
//   _pbs0=0b10001000;
//   _pbs1=0b00000000;
//
//   _ifs0=0b00000010;





	while(1) {



#if 1
		if(transfer_complete_flag) {
			SendByte(23);
			transfer_complete_flag=0;
			switch(1) {
				//-------检查数据头,错误就退出---------//
			case 1:
				if(ComRCah[0]!=0xcc)
					break;
			case 2:
				if(ComRCah[1]!=0x99)
					break;
			case 3:
				if(ComRCah[2]!=0x99)
					break;
			case 4:
				if(ComRCah[3]!=0xcc)
					break;
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
								WR_EE_LAST(CALIBRATION_12M,6,10);//写入设置
								SetPack();
								ComCah[4]=1;
								ComCah[5]=0x90;
								ComCah[6]=0x91;
								SendData(7);
							}
							break;
						case 0x91:					//设置17M灵敏度
							if(ComRCah[4]==0x0B) {
								WR_EE_LAST(CALIBRATION_17M,6,10);//写入设置
								SetPack();
								ComCah[4]=1;
								ComCah[5]=0x91;
								ComCah[6]=0x90;
								SendData(7);
							}
							break;
						case 0x92:					//设置24M灵敏度
							if(ComRCah[4]==0x0B) {
								WR_EE_LAST(CALIBRATION_24M,6,10);//写入设置
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
								case 0x93: temp = CALIBRATION_12M;break;
								case 0x94: temp = CALIBRATION_17M;break;
								case 0x95: temp = CALIBRATION_24M;break;
								}
								RE_SET(temp, ComRCah[5]);
								SendData(17);
							}
							break;
						case 0x97:				//点火
							clear_sampling_buff();
							
						case 0x96:				//读采样
							if(ComRCah[4]==0x01) {
								//接受到读采样标志，清零采样缓冲区。等待15个数据满了之后再
								//发送给上位机
								read_sampling_flag=1;			//读采样标志
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

							
						}
					}
				}
			}
		}

#endif
		GCC_CLRWDT();
	}
}