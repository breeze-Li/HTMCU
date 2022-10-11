//2022年8月26日：串口，int1中断
//2022年8月28日：ADC
//2022年8月29日：温度表，电压检测
//2022年8月30日：蜂鸣器
//2022年8月31日：低功耗（9-10UA）,互联输入输出(由于每个产品不同步，因此不会退出报警[已解决])
//2022年9月1日：据张要求，完善灯，之前是自检时按键松开后走从机线结束声音，
//				现在是检测和声音放在同一条线才能实现，TB0常开，功耗增加1UA
//2022年9月29日：改用它激模式(5-7UA【休眠模式(1.5)+ldo(3)+电压采样(0.5)=5UA】)
//				 [需要合理处理睡眠及退警时机，不然声音异常 -> 自激模式用引脚控制蜂鸣器，引脚在休眠时会自保持，休眠时也可正常发声，它激模式用PWM信号控制，休眠时无PWM]
#include "BA45F5440.h"
typedef unsigned char uc;
typedef unsigned int ut;
typedef unsigned long int ul;
volatile uc temp;

#define  BUZZER
#define  BUZZRE_ON	_pton = 1;
#define  BUZZRE_OFF	{_pton = 0;_pa6 = 1;}
#define  CONNECT_IN _pa0
#define  CONNECT_OUT _pa5
#define  OpenLED_R    _isgenc |= 0b10000010;	///ISINK1 引脚灌电流使能,50mA
#define  CloseLED_R   _isgenc = 0b00000000;	///ISINK1 引脚灌电流除能
//#define  BT_LOW		0xc2 //7.96
//#define  BT_HIGH 	0xc7//8.24V
#define  BT_LOW		0xB0 //7.4V
#define  BT_HIGH 	0xB2//
#define  AlarmTmp   60 //报警温度
//volatile bit Tb1_flag,Tb0_flag;
struct  {
    uc Tb1_flag :1;		
    uc Tb0_flag :1;	
    uc Interconnect :1;		
    uc Int1_dwflag :1;
    uc Int1_upflag :1;		
    uc Low_battery:1;		
    uc SenErr:1;	//传感器故障	
    uc IsFireFlag:1;//报警
    uc ExitFireFlag:1;//预退警 9.29 用于协调两条线的退警过程
    uc Buzzer:1;
    uc Buzzer1:1;	//三声完毕	
}Flags;	//key_status
volatile uc Master;//错误计数M
volatile uc SenErrCount;//错误计数
volatile uc TestCnt;	//报警计数
volatile uc BZ_Count;	//声音计数
volatile uc BZ_Count1;	//声音计数
volatile uc Test_Count;	//测试计数

//0-83℃
const uc Chack_AD[84] ={222,221,219,218,216,215,213,211,209,207,206,\
				204,202,200,198,195,193,191,189,187,184,182,180,\
				177,175,172,170,167,165,162,160,157,155,152,149,\
				147,144,142,139,137,134,131,129,126,124,121,119,\
				116,114,111,109,107,104,102,100,97,95,93,91,89,87,\
				85,83,81,79,77,75,73,71,69,68,66,64,63,61,60,58,57,\
				55,54,52,51,50,49};


void DelaymS(ut);
void UARTInit(void);
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


void main(){
	MCUINIT();
	UARTInit();
	Int1init();
	LB_Alarm();

	while(1) {
		/*主机自检和火警*/
		/*主机在发声*/
		/*从机接收总线*/
		if( Flags.Int1_dwflag==0 &&  Flags.IsFireFlag ==0 && \
			Flags.Buzzer == 0 && \
			Flags.Interconnect ==0 \
			){
			if(Flags.Tb1_flag == 0 || Flags.Tb0_flag ==0)
        		_halt();
        }
 		
        if(Flags.Tb1_flag ) {
            Flags.Tb1_flag = 0;	
			temp = GET_temperature();
			//检测火警时，NTC减小，会影响电压检测，使温度>50°，
 			//不进行电压检测，且39S检测一次。
 			Test_Count++;
			if(!Flags.IsFireFlag && Test_Count > 38 ){
				Test_Count =0;
				if(temp < 50){
					GET_battery();
					if(Flags.Low_battery) LB_Alarm();
				}
				if(Flags.SenErr){
					LB_Alarm();
					DelaymS(100);
					LB_Alarm();
				}
			}
        }
        //火警和自检开声音（主从必须分开）
        //如果公用，无法退出报警，(主从交替拉高总线)
        //如果公用且判断互联状态，只能互联一次（主机第二次检测总线为高后就关掉总线）

//    		if( Flags.IsFireFlag || Flags.Int1_dwflag || Flags.Interconnect){
//				_tb0e = 1;	
//				_tb0on = 1;
//				Flags.Buzzer = 1;
//    		}
//				//主机开总线
//				CONNECT_OUT =1;
//				Master = 1;
//			}else if( ){
//				_tb0e = 1;	
//				_tb0on = 1;
//				Flags.Buzzer = 1;
//				//从机不开总线
//				CONNECT_OUT = 0;
//				Master = 2;
//			}
        if(Flags.Tb0_flag){
        	Flags.Tb0_flag = 0;
        	//互联输入
        	if(!Master){
        		Flags.Interconnect = CONNECT_IN  ;
	        	if( Flags.IsFireFlag || Flags.Int1_dwflag){
	        		//主机开总线
					CONNECT_OUT =1;
					Master = 1;
	        	}else if(Flags.Interconnect){
	        		//从机不开总线
					CONNECT_OUT = 0;
					Master = 2;
	        	}
        	}
        	if(Master == 1){
        		//主机
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
        	//从机
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

void MCUINIT(void){
    _scc = 0x01;
    _hircc = 0x05;
    while(_hircf == 0);
	//----------清零--------------------------//
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
	/*******金毛初始化********/    
	_wdtc=0x57;
   
    //时基初始化
    _pscr = 2;		///32k
    _tb0c = 0x06;
    _tb0e = 0;   //时基0设置0.5秒定时
    _tb0e = 1;	
	_tb0on = 1;
    _tb1c = 0x07; //时基1设置为1秒
    _tb1e = 1;
	_tb1on=1;
	_emi=1;				//总中断
	
	//----------ADC引脚(电池)---------------------//
	_pas17 = 1;//pa7->AN1
	_pac7 = 1;//输入
	_pas11 = 1;//pa4->AN0
	_pac4 = 1;//输入
	//----------ADC引脚(温度)---------------------//
	_pbc4 = 0;
	_pb4 = 1;
	//----------其他引脚--------------------//
	_pac2 = 1;
	_papu2 = 1;//PA2
	_pa2 = 0;
	//互联输入
	_pac0 = 1;
	_pa0 = 0;
	//互联输出
	_pac5 = 0;
	_pa5 = 0;
}

void Int1init(void){
	_pac1=1;	//输入
	_pa1 = 1;
	_papu1=1;	//上拉
	_pawu1 = 1;
	/*INT0S1~INT0S0
	00：除能
	01：上升沿
	10：下降沿
	11：双沿*/
	_int1s1=1;
	_int1s0=1;	//双沿
	_int1e=1;	//使能int1
}

void __attribute((interrupt(0x0c))) INT1(void){
	//int0中断
	_int1f=0;
	if(_pa1==1){
		Flags.Int1_upflag = 1;
	}else{
		Flags.Int1_dwflag = 1;
		Flags.Int1_upflag = 0;
	}
}

void UARTInit(void)
{	//串口和蜂鸣器引脚初始化
#ifdef BUZZER
	//----------蜂鸣器---------------//
	//自激模式
//	_pac3 = 1;_pac6 = 0;
//	_pa3 = 0; _pa6 = 1;
	//它激模式
	_pac3 = 0;
	_papu3 = 1;
	_pa3 = 1;
	_pas14 = 1;		//pa6 -> PTP
	_pac6 = 0;
	_pa6 = 1;
	//蜂鸣片驱动
    _ptmc0 =	0b00000000;	//设置分频比 FSYS/4   4M系统时钟
    _ptmc1 =	0b10101000;	//PWM同相高有效输出
    _ptmc2 =	0b00000000;
    _ptmal =	0x9C;
    _ptmah =	0;	   //DUTY=50%
    _ptmrpl =	0x39;
    _ptmrph =	0x01;	//PERIOD=3.2KHz
	
//	//初始化UART,开蜂鸣器的话，只能用PA2做rx,无tx	
//	_umd=1;				//做串口		
//	_uucr1=0b10000000;
//	_uucr2=0b11101100;
//	_ubrg=25;				//4M时钟时波特率9600
//	_usime=1;				//USIM中断使能
//	_pas04 = 1;
//	_papu2=1;			//pa2上拉
//	_ifs00=0;			//RX 输入源引脚为Pa2
//	//-------------------------------------		
//	_emi=1;				//总中断
#else	
	//初始化UART	
	_umd=1;				//做串口		
	_uucr1=0b10000000;
	_uucr2=0b11101100;
	_ubrg=25;				//4M时钟时波特率9600
	_usime=1;				//USIM中断使能
	// --------------HD232-----------------
	_pas07=0;			
	_pas06=1;
	_pac3=0;				//Pa3做TX,输出
//	_pas15=1;			
//	_pas14=0;			//Pa6做RX
//	_papu6=1;			//pa6上拉
//	_ifs00=1;			//RX 输入源引脚为Pa6
	_pas04 = 1;
	_papu2=1;			//pa2上拉
	_ifs00=0;			//RX 输入源引脚为Pa2
	//-------------------------------------		
	_emi=1;				//总中断
#endif
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


//void __attribute((interrupt(0x10))) UART(void)
//{
//	_usimf=0;
//	//接受中断函数
//	/*	RxFlag=1;*/
//	while(_urxif>0) {
//		temp=_utxr_rxr;
//	}
//	//WR_EE(1, temp);
//
//}

void SendByte(unsigned char l)
{
	//发送1个字节
	_utxr_rxr=l;
	while(_utxif==0);//等待数据从缓冲器加载到移位寄存器中
	while(_utidle==0);//等待数据发送完成
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
 * [__attribute description] 1S定时到
 * @param void [description]
 */
void __attribute((interrupt(0x30))) STB0(void) {
    _tb0f = 0;
    Flags.Tb0_flag = 1;
}
/**
 * [__attribute description]0.5s定时到
 * @param void [description]
 */
void __attribute((interrupt(0x34))) STB1(void) {
    _tb1f = 0;
    Flags.Tb1_flag = 1;
}

unsigned int GetADC(unsigned char c)
{
   ut r=0;
   unsigned long sum=0;		//4平均用，量程稍大
   uc i;
   //uc t1,t2;
   switch(c)
   {
      case 1:
         _sadc0=0b00000001;
         _sadc1=0b00001010;       //AD定位到AN1。温度
         break;
      case 2:	
         _sadc0=0b00000000;
         _sadc1=0b00001010;       //AD定位到AN0.电池
   	     break;
   	  default :
//   	  	 _sadc0=0b00000100;
//         _sadc1=0b00001010;       //AD定位到AN4、温度
   	  	 break;
   }
   
   //E3 DE DB D9 D8 D7 D6 D6 电池
   _adcen=1;
   for(i=0;i<4;i++){//4次平均
	   
	   _start=1;
	   _start=0;
	   while(_adbz==1);
//	   SendByte(_sadoh);
//	   SendWord(_sadol);
//	   SendWord(_sadoh<<4);
//	   SendWord(_sadol>>4);
/*无论ADRFS为何值，_sadoh始终为8位，_sadol始终为高4位*/
	   //r = (_sadoh<<4) + (_sadol>>4); //12位全部取出
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
	//电池，实测为3.53倍7.79V->C5;9.65V->D7
	ul tempAD;
	tempAD=(ul)GetADC(2);
//	tempAD = tempAD * 330;
//	tempAD = tempAD / 255;
//	tempAD = tempAD * 35;//分压
//	SendByte(tempAD >> 8);
//	SendByte(tempAD & 0XFF );
//	SendByte(((tempAD/1000)<<4) + ((tempAD%1000)/100));
//	SendByte((((tempAD%100)/10)<<4) + (tempAD%10));
	
	if(Flags.Low_battery){
		if(tempAD > BT_HIGH) Flags.Low_battery=0;
	} else{
		if(tempAD < BT_LOW) Flags.Low_battery=1;
	}
	
}
uc GET_temperature(void){
	//温度
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
			Flags.SenErr=1;               //传感器故障
	}
	//取温度
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
void LB_Alarm(void){
	OpenLED_R
	BUZZRE_ON
	DelaymS(50);
	CloseLED_R
	BUZZRE_OFF	
}
