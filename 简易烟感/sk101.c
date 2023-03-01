//SD628  独立式感烟探测器   20201009
// 4M时钟
//声音频率默认2.8K
//报警持续时间2分钟
//温度补偿，高温 1级 41度，2级 48度
//低温  1级  11度，2级 6度
//集烟盒暗电流检测故障，不报
//报警记忆标记清除
//20200908  增加电压补偿   291-282 2%,281-272 3%,271-262 4%,261-254 5%,253 8%
//20200914  退出火警时间改为60秒
//20201008  增加声音标定;测试电压40秒一次，延时5mS
//20201009  标定声音的上下0.05KHz浮动
//SD628和SK200程序合并
//20201012  需要定义是否送检版本和产品型号
//20201104  增加测试加速功能
//20201117  重新匹配放大倍数
//20201207  1200mV/nA改为1000mV/nA
//20210126  修改IR检测时序

#include "BA45F5240-2.h"

//定义送检版本
#define IsTest
//定义型号
#define SD628

//#define SK200

#ifdef SD628
   #define Ver  0x12   
   #define MyType  18  
   #define BeepHzDef    K280
   #define LowHzDef     K275
   #define HighHzDef    K285
#endif

#ifdef SK200
   #define Ver  0x31   
   #define MyType  1  
   #define BeepHzDef     K320
   #define LowHzDef      K315
   #define HighHzDef     K325
#endif

//定义声音频率
#define K270  0
#define K275  1
#define K280  2
#define K285  3
#define K290  4
#define K295  5
#define K300  6
#define K305  7
#define K310  8
#define K315  9
#define K320  10
#define K325  11
#define K330  12
#define K335  13
#define K340  14
#define K345  15

//定义放大倍数
#define nA20mV  0
#define nA30mV  1
#define nA40mV  2
#define nA60mV  3
#define nA80mV  4
#define nA100mV  5
#define nA120mV  6
#define nA160mV  7
#define nA200mV  8
#define nA240mV  9
#define nA320mV  10
#define nA400mV  11
#define nA480mV  12
#define nA640mV  13
#define nA800mV  14
#define nA1000mV  15

//定义中断向量
DEFINE_ISR(UART, 0x10);
DEFINE_ISR(STMA, 0x2c);
DEFINE_ISR(STB0, 0x30);
DEFINE_ISR(STB1, 0x34);
DEFINE_ISR(INT1, 0x0C);
      
#define IRFirDef  60	     //火警默认值
#define IRFirOutDef 40       //退出火警默认值
#define IRCompDef 100        //最大补偿默认值
#define Set1Def  0b00000101  //设置字节,默认  没有温度补偿，2.6V欠压，静音8分钟
#define IRSetDef  0x55       //红光设置  80mV/nA,150mA
#define MaxEMI  200          //蓝光最大暗电流干扰
#define SaturationDef  210   //饱和门槛
#define IRErrDef        5

#define  TempHighGrd1    184      //48度的AD
#define  TempHighGrd0    170	  //41度的AD
#define  TempLowGrd0     88      //11度的AD
#define  TempLowGrd1     75       //6度的AD

#define  EN1 _pa3	///_pa7
#define  EN2 _pb2

#define  OpenLED    {_isgenc |= 0b10000010;_isgdata1=0b00000000;}	///ISINK1 引脚灌电流使能,50mA
#define  CloseLED   _isgenc &= 0b10000001;		///ISINK1 引脚灌电流除能

//存储地址定义
#define IR0SetAdd 0x00      //IR0标
#define FirSetAdd 0x01	    //火警标定
#define FirOutAdd 0x02      //退出火警标定
#define MuteFirAdd 0x03     //报警后退出静音报警标定
#define MaxCompAdd 0x04     //最大补偿设定
#define MaxDCDarkAdd 0x05   //最大直流耦合暗电流
#define Set1Add  0x06       //设置字节1
#define Set2Add  0x07       //设置字节2
#define BeepHzAdd  0x08       //设置字节2

#define SetXorAdd  0x0f     //校验字节       

#define IDAdd 0x10          //设备ID  H+M+L+Xor
#define LifeHAdd 0x14	    //运行时间单位12小时 H+L+Xor
#define IRCurBkAdd 0x17     //红光当前背景

//位变量
volatile bit  Tb0_flg;
volatile bit  Tb1_flg;         //时基1标志
volatile bit  SetErr;          //设置有错误
//bit  OrgErr;          //0标有错误
//bit  AlmErr;          //报警标定有错误
//bit  ExitFirErr;      //退出火警标定错误
//bit  MuteFirErr;      //静音标定错误
//bit  CompensateErr;   //补偿标定错误
volatile bit  IsFireFlag;        //火灾报警标志
volatile bit  MuteFlag;          //静音标志
volatile bit  LowFlag;           //低电压标志
volatile bit  LowLEDFlag;      //用于低电压灯指示
volatile bit  GameOverFlag;    //寿命终止 
volatile bit  DirtyFlag;       //污染   
volatile bit  FautMuteFlag;    //故障消音标志
volatile bit  TestKeyFlag;     //正在测试按键
volatile bit  TestSelfFlag;    //自检标志
volatile bit  IRDarkErr;
volatile bit  IRErr;           //IR LED 失效
volatile bit  IRBkErr;         //IR当前背景错
volatile bit  FiredFlag;       //曾经火警
volatile bit  TestSelfFaultFlag;       //自检错误
volatile bit  Timer1Flag;   //时钟定时器标志
volatile bit  RxFlag;       //串口有新数据标志
volatile bit  AddTest;        //增加测试标志

volatile unsigned char   IsComming;	       //正在通信标志

volatile unsigned char MyID[3];        //产品ID

volatile unsigned int KeyCloseCnt,KeyOpenCnt;   //timer1的计数器，用于按键测试 
volatile unsigned char TestTimer,TestFun,FireFun;
volatile unsigned int LifeCount,LifeTimer;      //寿命计时，单位：12小时
volatile unsigned int LightCount;               //闪灯计数
volatile unsigned long FautMuteTimer;           //故障静音时钟
volatile unsigned int MuteTimer;      //静音计数
volatile unsigned char Fire_OutTimer;
volatile unsigned char Volume;        //音量
volatile unsigned int volt;           //电压
volatile unsigned char K3V;           //折算3V的比例因子
//volatile unsigned char LPCnt; //测试电压计数，低电压计数
volatile unsigned char IRDarkErrCnt;      //暗电流错误 
volatile unsigned char IRErrCnt;          //IR 失效计数
volatile unsigned char TestVtCnt;         //测试电压计数器
volatile unsigned char Fire_OutMuteCnt;   //火警退出静音，再次报警计数
volatile unsigned char Fire_OutFireCnt;	  //退出火警计数
volatile unsigned char MuteNumber;
volatile unsigned char BeepHz,LowHz,HighHz,dHz,TestBeepTimer;            //声音频率

volatile unsigned char ComPC;		        //发送数据指针
volatile unsigned char ComCah[24];          //发送缓冲区
volatile unsigned char ComRPC;		        //接收数据指针
volatile unsigned char ComRCah[24];         //接收缓存
//2.7;2.75;2.8;2.85;2.9;2.95;
//3.0;3.05;3.1;3.15;3.2;
//3.25;3.3;3.35,3.4,3.45
const unsigned int Hz[16]={370,364,357,351,345,339,334,328,323,318,313,308,303,299,294,290};

//volatile unsigned char dHz;

volatile unsigned char IR0;          //红外0标
volatile unsigned char IRFir;	    //火警标定
volatile unsigned char IRFirOut;     //退出火警标定
volatile unsigned char IRMuteFir;    //报警后退出静音报警标定
volatile unsigned char IRComp;       //最大补偿设定
volatile unsigned char MaxDCDark;    //最大补偿设定
volatile unsigned char Set1;         //设置字节1
volatile unsigned char Set2;         //设置字节2
//unsigned char Set3;       //设置字节3
volatile unsigned char IRCurBk;        //红光当前背景
volatile unsigned char IRDarkAD,IRAD,IRDCDarkAD;

volatile unsigned char TempAD,CurTemp;         //温度采样

volatile unsigned char IRADAve[4];     //长期平均计算
volatile unsigned char IRCnt1,IRCnt2;


//定义中断函数
void UART(void);
void STMA(void);
void STB0(void);
void STB1(void);
void INT1(void);

//定义普通函数
unsigned char RD_EE(unsigned char RD_EE_addr);                      //读EEPROM
void WR_EE(unsigned char WR_EE_addr,unsigned char WR_EE_data);      //写EEPROM
void S_ADJ_OPA(void );     //校准OP0
void S_ADJ_OPA1(void );    //校准OP1
void _INI_SM_OPA(void);    //初始化运放



void TestFlash(void);      //测试设置项是否正确
//unsigned char TestLED(void);   //测试IR和BlueIED
void SetMult(unsigned char);   //设置放大倍数
//t 持续时间mS
void Beep(unsigned char t);      
//打开报警声音 p 频率，v 音量,v=0  关闭   
void OpenSound(unsigned char p,unsigned char v);
//闪烁
void Twinkle(void);

void DelaymS(unsigned int t);

void FireBegin(void);                    //火警任务处理
void FireExit(void);                     //退出火警
unsigned char TestLED(void);             
unsigned char GetADC(unsigned char c);   //测试AD的通道，c=1 温度，c=2 OP2
void TestTmp(void);                      //测试温度
//void TestBG(void);                     //测试带隙 ///不使用带隙了，用AN0
void TestVolte(void);					 ///	代替TestBG()
void SetPack(void);

//发送缓冲数据
void NBSendData(unsigned char l);

//-15度---55度,负温度的最高位为1，如-1度=0x81;-15度=0x8f;
unsigned char GetTemp(void);

//-15度到55度
const unsigned char TMP[71]={30,32,34,35,37,39,
                             41,43,45,47,49,51,53,55,58,60,
                             62,65,67,70,72,75,77,80,83,86,
                             88,91,94,97,100,102,105,108,111,114,
                             117,120,122,125,128,131,133,136,139,141,
                             145,147,150,152,155,157,160,162,164,167,
                             170,172,174,176,178,180,182,184,186,188,
                             190,192,194,196,198};
                             
                             

void main()
{
   unsigned char i,j,k;
   unsigned int m;
   
   _scc=0x01;
   _hircc=0x05;
   while(_hircf==0);
   	   
   _mp0=0x80;
   _acc=128;
   while(_acc!=0)
   {
      _iar0=0;
	  ++_mp0;
	  _acc=_acc-1;	
   }
   
   _mp1l=0x80;
   _mp1h=1;
   _acc=128;
   while(_acc!=0)
   {
      _iar1=0;
	  ++_mp1l;
	  _acc=_acc-1;	
   }
   _mp1h=0;
   _iar1=0;
   _mp1l=128;
   
   //初始化所有IO
   
   _pac=0b10110110;   ///_pac=0b00110111;PA7用输入
   _papu=0b00000111;
   
   _pbc=0b11111011;
   _pbpu=0b11111011;
   
   //_pawu=0x10;    
   
   _sledc=0xFF;
   
   _pa=0b00000110;	
   _pb=0b00000000;		///_pb=0b00000100;
   //初始化IO复用功能
   _pas0=0b00000000;	
   _pas1=0b10010010;	///PA7用AN1 //_pas1=0b00010010;
   
   _pbs0=0b10001000;
   _pbs1=0b00000000;
   
   _ifs0=0b00000010;
   
   _wdtc=0x57;
   
   //初始化中断
   _integ=0b00001000;      //INT1 下降沿
   _intc0=0b00001000;      //INT1 使能
   
   _intc1=0b00000001;
   _intc2=0b00000000;
   _intc3=0b00000000;

   //初始化时基
   _pscr=2;
   _tb0c=0x07;
   _tb0e=1;     //时基0设置1秒定时
   _tb1c=0x06;  //时基1设置为0.5秒
   _tb1e=1;
	   
   //初始化PWM
   _ptmc0=	0b00000000;	//设置分频比 FSYS/4   4M系统时钟
   _ptmc1=	0b10101000;
   _ptmc2=	0b00000000;
   _ptmal=	0x9c;
   _ptmah=	0;	    //DUTY=50%
   _ptmrpl=	0x39;	
   _ptmrph=	0x01;	//PERIOD=3.2KHz
   
   //初始化UART
   
   _umd=1;
   _uucr1=0x80;
   _uucr2=0b11101100;
   _ubrg=25;

   //初始化ADC
   _sadc0=0b00001111;
   _sadc1=0b01101010;		///_sadc1=0b01101010;
   _vbgren=1;
             
   //初始化运放
   _INI_SM_OPA();
   DelaymS(20);   //20mS
   S_ADJ_OPA();
   S_ADJ_OPA1();
   _sds0=0;
   _sda0en=0;
   _sda1en=0;
   //关闭带隙
   _vbgren=0;
   //初始化定时器0  31.9mS
   _stmc0=0b01000000;      //fsys=32K ，
   _stmc1=0b11000001;
   _stmal=0xff;            //0xF4;
   _stmah=3;               //1;
   _stmae=1;
   _ston=0;
   
   //初始化变量
   IRErr=0;
   KeyCloseCnt=0;
   KeyOpenCnt=0;
   IRErrCnt=0;
   IRDarkErrCnt=0;
   IRDarkErr=0;
   TestVtCnt=0;
   Tb0_flg=0;
   Tb1_flg=0;
   IsFireFlag=0;     //火灾报警标志
   MuteFlag=0;       //静音标志
   LowFlag=0;        //低电压标志
   LowLEDFlag=0;
   GameOverFlag=0;   //寿命终止 
   DirtyFlag=0;      //污染   
   FautMuteFlag=0;   //故障消音标志
   TestKeyFlag=0;    //正在测试按键
   TestSelfFlag=0;   //自检标志
   FiredFlag=0;
   Timer1Flag=0;
   TestSelfFaultFlag=0;
   RxFlag=0;
   TestTimer=0;
   IRCnt1=0;
   IRCnt2=0;
   IRDarkAD=0;
   IRAD=0;
   IRDCDarkAD=0;
   LifeTimer=0;
   LightCount=0;
   MuteTimer=0;
   FautMuteTimer=0;
   Fire_OutMuteCnt=0; 
   Fire_OutFireCnt=0;
   FireFun=0;
   TestFun=0;
   Volume=0;         //音量
   volt=0;           //电压
   ComPC=0;
   IsComming=0;
   CurTemp=25;
   TempAD=128;
   MuteNumber=0;
   TestBeepTimer=0;
   BeepHz=BeepHzDef;
   LowHz=LowHzDef;
   HighHz=HighHzDef;
   dHz=LowHz;
   AddTest=0;
   TestFlash();
   //开时基0
   _tb0on=1;
   _emi=1;
    while(1)
    {
       if(TestKeyFlag==0 && IsComming==0 && IsFireFlag==0 && TestSelfFlag==0 && TestBeepTimer==0)
          _halt();
       
       if(Tb0_flg>0)
       { 
       	  Tb0_flg=0;
       	  TestTimer++;
       	  if(IsComming>0)
             IsComming--;
          if(TestBeepTimer>0)
          {
          	 TestBeepTimer--;
          	 if(TestBeepTimer==0)
          	 {
          	    if(IsFireFlag==0 && TestSelfFlag==0)
                {
                   _tb1on=0;
                   OpenSound(0,0);
                }
          	 }
          }
          //Twinkle();
       	  if(TestTimer>=TestFun)
       	  {
       	  	 if(IsFireFlag==1 && Fire_OutTimer<245)
                Fire_OutTimer=Fire_OutTimer+TestFun;

       	     TestTimer=0;
       	     if(TestVtCnt==0)
       	     {
       	     	TestTmp();
       	     	CurTemp=GetTemp();
       	     	TestVolte();///TestBG();
       	     }
       	     //管理定时器
       	     TestVtCnt=TestVtCnt+TestFun;
       	     if(TestVtCnt>40)
       	     {
       	        TestVtCnt=0;
       	     }
       	     LifeTimer=LifeTimer+TestFun;
       	     if(LifeTimer>=43200)
       	     {
       	        LifeTimer=0;
                if(SetErr==0 && (Set1&2)>0)  //12小时存储一次
                {
                   //存储背景值
                   if(IRADAve[2] != IRCurBk)
                   {
                   	  i=IRADAve[2];
                   	  k=i ^ 0xff;
                      WR_EE(IRCurBkAdd,i);
			          WR_EE(IRCurBkAdd+1,k);  
			          WR_EE(IRCurBkAdd+0x20,i);
			          WR_EE(IRCurBkAdd+0x21,k);
			          IRCurBk=IRADAve[2];
                   }
                }
	            if(GameOverFlag==0)
	            {
	               //存储运行时间，单位12小时
	               if(LifeCount>=7700)
	                  GameOverFlag=1;       //寿命到期
	               if(LifeCount<8000)
                   {
		              LifeCount++;
		              i=(unsigned char)(LifeCount>>8);
		              j=(unsigned char)(LifeCount & 0xff);
		              k=i + j;
			          WR_EE(LifeHAdd,i);
			          WR_EE(LifeHAdd+1,j);
			          WR_EE(LifeHAdd+2,k);
			          WR_EE(LifeHAdd+0x20,i);
			          WR_EE(LifeHAdd+0x21,j);
			          WR_EE(LifeHAdd+0x22,k);
		           }	
	            }
       	     }
       	     if(FautMuteFlag>0)
       	     {
       	        FautMuteTimer=FautMuteTimer+TestFun;
       	        if(FautMuteTimer>=86400)  //低电压、寿命到、LED故障、暗电流故障  消音  24小时
                {
                   FautMuteFlag=0;
	               FautMuteTimer=0;
                }
       	     }
       	     if(MuteFlag>0)
       	     {   
       	        MuteTimer=MuteTimer+TestFun;
       	        if((Set1 & 1)>0)
       	        {
       	           if(MuteTimer>=480)
       	           {
       	              MuteTimer=0;
       	              MuteFlag=0;
       	           }
       	        }
       	        else
       	        {
       	           if(MuteTimer>=72)
       	           {
       	              MuteTimer=0;
       	              MuteFlag=0;
       	           }
       	        }
       	     }	
       	     if((TestSelfFlag==0) && (TestBeepTimer==0))
       	     {
       	     	if(IsFireFlag==0)
       	        {
       	     	   LightCount=LightCount+TestFun;
       	     	   
       	     	   //闪灯管理
       	     	   if(SetErr || GameOverFlag || IRDarkErr || IRErr || LowFlag || DirtyFlag || IRBkErr)
       	     	   {
       	     	      if(LightCount>=24)
       	     	      {
       	     	         if(LowFlag)
       	     	         {
       	     	            if(LowLEDFlag==0)
       	     	            {
       	     	               LowLEDFlag=1;
       	     	               Twinkle();
       	     	               if(FautMuteFlag==0)
       	     	                  Beep(80);
       	     	            }  		
       	     	         }
       	     	      }   	
       	     	   	  if(LightCount>=48)
       	     	   	  {
       	     	   	     LightCount=0;
       	     	   	     LowLEDFlag=0;
       	     	   	  	 if(SetErr || IRDarkErr || IRErr || DirtyFlag || IRBkErr)
       	     	   	  	 {
       	     	   	  	    if(FautMuteFlag==0)	
       	     	   	  	    {
       	     	   	  	       for(i=0;i<3;i++)
       	     	   	  	       {
       	     	   	  	 	      Beep(80);
       	     	   	  	 	      Twinkle();
       	     	   	  	 	      DelaymS(80);
       	     	   	  	       }
       	     	   	  	    }
       	     	   	  	    else
       	     	   	  	    {
       	     	   	  	       for(i=0;i<3;i++)
       	     	   	  	       {
       	     	   	  	 	      Twinkle();
       	     	   	  	 	      DelaymS(80);
       	     	   	  	       }	
       	     	   	  	    }	
       	     	   	  	 }
       	     	   	  	 else
       	     	   	  	 {
       	     	   	  	    if(GameOverFlag)
       	     	   	  	    {
       	     	   	  	       if((FautMuteFlag==0) || (LifeCount>=7770))	
       	     	   	  	       {
       	     	   	  	          for(i=0;i<2;i++)
       	     	   	  	          {
       	     	   	  	 	         Beep(80);
       	     	   	  	 	         Twinkle();
       	     	   	  	 	         DelaymS(80);
       	     	   	  	          }
       	     	   	  	       }
       	     	   	  	       else
       	     	   	  	       {
       	     	   	  	          for(i=0;i<2;i++)
       	     	   	  	          {
       	     	   	  	 	         Twinkle();
       	     	   	  	 	         DelaymS(80);
       	     	   	  	          }	
       	     	   	  	       }   
       	     	   	  	    }	
       	     	   	  	 }
       	     	   	  }
       	     	   }
       	     	   else
       	     	   {
       	     	      //正常巡检
       	     	      if(MuteFlag==0)
       	     	      {  //正常状态384秒闪烁
       	     	         m=384;
       	     	      }
       	     	      else
       	     	      {//消音状态16秒闪烁
       	     	      	 m=16;
       	     	      }   
       	     	      if(LightCount>=m)   
       	     	   	  {
       	     	   	     LightCount=0;
       	     	   	     LowLEDFlag=0;
       	     	   	     Twinkle();
       	     	   	     if(FiredFlag && m==384)
       	     	   	     {
       	     	   	     	DelaymS(80);
       	     	   	     	Twinkle();
       	     	   	     }
       	     	   	  }  
       	     	   }
       	        }    
       	        IRDarkErrCnt=IRDarkErrCnt<<1;
       	        if(TestLED()>0)            //测试IR
       	        {
       	           if(IRDarkErrCnt==0)
       	              IRDarkErr=0;         //集烟盒漏光故障消失
       	           if(IsFireFlag==0)
       	           {
       	              //测试是否发射管过低
       	              IRErrCnt=IRErrCnt<<1;
       	              j=0;
       	              if(IR0>0)
       	              {
       	                 if(IRAD<(IR0>>1))
       	                 {
       	                    IRErrCnt=IRErrCnt | 1;
       	                    j=1;
       	                 }
       	              }
       	              else
       	              {
       	                 if(IRAD<IRErrDef)
       	                 {
       	                    IRErrCnt=IRErrCnt | 1;	
       	                    j=1;
       	                 }
       	              }
       	              
       	              if(IRErrCnt==255)
       	                 IRErr=1;
       	              if(IRErrCnt==0)
       	                 IRErr=0;
       	              
       	              if((j==0) && (SetErr==0) && (IRDarkErr==0) && (IRErr==0) && (IRBkErr==0))
       	              {   
       	                 if((Set1 & 2)>0)//计算漂移	
       	              	 {
       	              	    m=IRADAve[0];
       	              	    m=m<<7;
       	              	    m=m+IRADAve[1];
       	              	    m=m-IRADAve[0];
       	              	    m=m+IRAD;
       	              	    IRADAve[1]=(unsigned int)(m & 0x7f);
       	              	    IRADAve[0]=(unsigned int)(m >> 7);
       	              	    IRCnt1++;
       	              	    if(IRCnt1>=128)
       	              	    {
       	              	       IRCnt1=0;	
       	              	       m=IRADAve[2];
       	              	       m=m<<7;
       	              	       m=m+IRADAve[3];
       	              	       m=m-IRADAve[2];
       	              	       m=m+IRADAve[0];
       	              	       IRADAve[3]=(unsigned char)(m & 0x7f);
       	              	       IRADAve[2]=(unsigned char)(m >> 7);
       	              	       IRCnt2++;
       	              	  	   if(IRCnt2>=128)
       	              	  	      IRCnt2=0;
       	              	  	   //修正漂移值
       	              	  	   if(IRADAve[2]>IRComp)
       	              	  	      IRADAve[2]=IRComp;
       	              	  	   //判断污染   
       	              	  	   if(DirtyFlag)
       	              	  	   {
       	              	  	      if(IRComp>3)
       	              	  	      {
       	              	  	         if(IRADAve[2]<=(IRComp-3))
       	              	  	            DirtyFlag=0;
       	              	  	      }
       	              	  	      else
       	              	  	      {
       	              	  	         if(IRADAve[2]<IRComp)
       	              	  	            DirtyFlag=0;		
       	              	  	      }
       	              	  	   }
       	              	  	   else
       	              	  	   {
       	              	  	      if(IRADAve[2]>=IRComp)
       	              	  	         DirtyFlag=1;	
       	              	  	   }   
       	              	    }
       	              	 }
       	              	 //火警判据
       	              	 if((Set1 & 2)>0)
       	              	 {
       	              	    if(IRAD>=IRADAve[2])
       	              	 	   m=IRAD-IRADAve[2];
       	              	 	else
       	              	 	   m=0;	
       	                 }
       	              	 else
       	              	 {
       	              	    if(IRAD>=IR0)
       	              	 	   m=IRAD-IR0;
       	              	 	else
       	              	 	   m=0;	
       	              	 }
       	              	 i=0;
       	              	 if(MuteFlag &&  IRMuteFir>0)
       	              	 {
       	              	    //静音后，灵敏度降低
       	              	 	if(m>=IRMuteFir)
       	              	 	   i=1;
       	              	 }
       	              	 else
       	              	 {
       	              	    if(m>=IRFir)
       	              	 	   i=1;
       	              	 }
       	              	 if(i>0)
       	              	 {
       	              	    //大于报警门槛	
       	              	 	switch(TestFun)
       	              	 	{
       	              	 	   case 1:
       	              	 	      FireBegin();
       	              	 	      break;
       	              	 	   case 2:
       	              	 	      TestFun=1;
       	              	 	      break;
       	              	 	   default:
       	              	 	      TestFun=2;
       	              	 	      break;
       	              	 	}	
       	              	 }
       	              	 else
       	              	 	TestFun=8;
       	              }
       	              else	
       	              {
       	                 //设置有错，作为单光束使用	
       	              	 if(IRAD>=IRFirDef)
       	              	 {
       	              	    switch(TestFun)
       	              	    {
       	              	       case 1:
       	              	          FireBegin();
       	              	          break;
       	              	       case 2:
       	              	          TestFun=1;
       	              	          break;
       	              	       default:
       	              	          TestFun=2;	
       	              	    	  break;
       	              	    }
       	              	 }
       	              	 else
       	              	    TestFun=8;
       	              }
       	           }
       	           else
       	           {
       	              //火警条件下的测试
       	              Fire_OutMuteCnt=Fire_OutMuteCnt<<1;   //火警退出静音，再次报警计数
                      Fire_OutFireCnt=Fire_OutFireCnt<<1;
       	              if((SetErr==0) && (IRDarkErr==0) && (IRErr==0) && (IRBkErr==0))
       	              {   
       	                 if((Set1 & 2)>0)
       	              	 {
       	              	    if(IRAD>=IRADAve[2])
       	              	       i=IRAD-IRADAve[2];	
       	              	    else
       	              	       i=0;
       	              	 }
       	              	 else
       	              	 {
       	                    if(IRAD>=IR0)
       	              	       i=IRAD-IR0;	
       	              	    else
       	              	       i=0;
       	              	 }
       	              	 if(i<=IRFirOut)
       	              	 {
       	              	    Fire_OutFireCnt=Fire_OutFireCnt | 1; 
       	              	    if((Fire_OutFireCnt & 0xf)==0xf)
       	              	    {
       	              	       if(Fire_OutTimer>=60)
       	              	       {
       	              	          FireExit();
       	              	          MuteFlag=0;  
       	              	       }
       	              	    }
       	              	 } 
       	              	 else
       	              	 {
       	              	    if(MuteFlag && IRMuteFir>0)
       	              	    {
       	              	       if(i>IRMuteFir)
       	              	       {
       	              	          Fire_OutMuteCnt=Fire_OutMuteCnt | 1;   
       	              	          if((Fire_OutMuteCnt & 3)==3)   //退出静音模式
       	              	          {
       	              	          	 if(MuteNumber==0)
       	              	          	 {
       	              	                MuteFlag=0;	
       	              	                MuteNumber=1;
       	              	          	 }
       	              	          }
       	              	       }		
       	              	    }	
       	              	 }
       	              }
       	              else
       	              {  //设置有错，采用默认
       	                 if(IRAD<=IRFirOutDef)
       	                    Fire_OutFireCnt=Fire_OutFireCnt | 1;
       	                 if((Fire_OutFireCnt & 0xf)==0xf)
       	                 {
       	                    if(Fire_OutTimer>=60)
       	                       FireExit();   
       	                 }
       	              }
       	              TestFun=8;
       	           }
       	        }
       	        else
       	        {
       	           //IR暗电流过大
       	           #ifndef IsTest
       	              IRDarkErrCnt=IRDarkErrCnt | 1;     //送检时，此处屏蔽
                   #endif
       	     	   if(IRDarkErrCnt ==255)
       	     	      IRDarkErr=1;            //集烟盒漏光故障
       	     	   TestFun=8;
       	        }
       	     }
       	     else
       	        TestFun=8;
       	  }
       }
       
       if(Tb1_flg)
       {  //自检或者火警后管理声音与灯显示
          Tb1_flg=0;
          if(TestBeepTimer==0)
             Twinkle();
          if((MuteFlag==0 && IsFireFlag==1) || (TestSelfFlag>0) || (TestBeepTimer>0))
          {
          	 switch(FireFun)
          	 {
          	    case 0:
          	       if(Volume==0)
          	          Volume=1;
          	       FireFun=1;
          	       OpenSound(dHz,Volume);
          	       break;
          	    case 2:
          	       OpenSound(dHz,Volume);
          	       FireFun=3;
          	       break;
          	    case 4:
          	       OpenSound(dHz,Volume);
          	       FireFun=5;
          	       break;
          	    case 1:
          	       OpenSound(dHz,0);
                   FireFun=2;	
                   break;
          	    case 3:
          	       OpenSound(dHz,0);
                   FireFun=4;	
                   break;
          	    case 5:
          	       OpenSound(dHz,0);
          	       FireFun=6;
          	       break;
          	    case 6:
          	       FireFun=7;
          	       break;
          	    case 7:
          	       FireFun=8;
          	       break;
          	    case 8:
          	       FireFun=0;
          	       if(Volume<3)
             	   {
                      Volume++;
                      if(Volume==3)
                      {
                         dHz=LowHz;
                      }
             	   }
                   else
                   {
                      if(dHz==HighHz)
                         dHz=LowHz;
                      else
                         dHz++;
                   }   
                   break;
          	 }
          }	
       }
       
       if(Timer1Flag)
       {
          Timer1Flag=0;	
          if(_pa1==0)
          {
          	 KeyOpenCnt=0;
       	     if(KeyCloseCnt<132)   //4S
       	        KeyCloseCnt++;
       	     if(KeyCloseCnt>=93 && IsFireFlag==0 && TestSelfFlag==0)  //按键时间大于3秒
       	     {
       	     	//测试电池,集烟盒,温度,标定
       	     	TestSelfFlag=1;
       	     	TestSelfFaultFlag=0;
       	     	if(SetErr)
       	     	   TestSelfFaultFlag=1;
       	     	if(TestSelfFaultFlag==0)
       	     	{
                   TestVolte();///TestBG();
                   switch(Set1 & 0x0c)
                   {
   	                  case 0:   //2.6V
   	                     m=90;	///0.9V
   	                     break;
   	                  case 4:   //2.7V
   	                     m=100;	///1.0V
   	                     break;
   	                  case 8:  //2.8V
   	                     m=110;	///1.1V
   	                     break;
   	                  default:  //2.5V
   	                     m=120;	///1.2V
                         break;
                   } 
                   if(volt<m || volt>550)
                      TestSelfFaultFlag=1;
       	     	}
       	     	if(TestSelfFaultFlag==0)
       	     	{
                   TestTmp();
                   CurTemp=GetTemp();
                   if((TempAD>=218) || (TempAD<=7))   //温度在-40到70之间
                      TestSelfFaultFlag=1;
       	     	}
                if(TestSelfFaultFlag==0)
                {
                   if(TestLED()>0)
                   {
                      if(IRAD<(IR0>>1))
                         TestSelfFaultFlag=1;	
                   }	
                   else
                      TestSelfFaultFlag=1;	
                }
       	        
       	        
       	        if(TestSelfFaultFlag>0)
       	        {
       	           Twinkle();
       	     	   Beep(80);
       	     	   DelaymS(100);
       	           Twinkle();
       	     	   Beep(80);
       	     	   DelaymS(100);
       	     	   Twinkle();
       	     	   Beep(80);
       	        }
       	        else
       	        {
       	           dHz=LowHz;
       	           Volume=0;
       	           _tb1on=1;   
       	           FireFun=0;
       	        }
       	     }
          }
          else
          {
             if(KeyOpenCnt<25)
                KeyOpenCnt++;
             if(KeyOpenCnt>6)
             {
                KeyOpenCnt=0;
                TestKeyFlag=0;
                _emi=0;
                _ston=0;
                _emi=1;
                if(TestSelfFlag)
                {//关闭自检
                   TestSelfFlag=0;	
                   OpenSound(0,0);
                   _emi=0;
                   _tb1on=0;
                   _emi=1;
                }
                else
                {
                   if(KeyCloseCnt>=7 && KeyCloseCnt<30)  //按键时间大于200mS,小于1秒
                   {
                   	  i=0;
                   	  if(IsFireFlag==0)
                   	  {
                         if(FiredFlag>0)
                         {
                            Beep(150);	
                   	  	    DelaymS(150);
                   	  	    Beep(150);	
                      	    FiredFlag=0;
                      	    i=1;
                         }
                   	  }
                      else
                         FiredFlag=0;
                      if(i==0)   
                      {
                         if(MuteFlag==0)	
                   	     {
                   	        MuteFlag=1;
                   	        FiredFlag=0;
                   	        MuteTimer=0;
                   	        Beep(80);
                   	     }
                   	     else
                   	     {
                   	        MuteFlag=0;
                   	        MuteNumber=0;
                   	        Beep(80);	
                   	  	    DelaymS(80);
                   	  	    Beep(80);
                   	     }
                      }
                   } 
                   else
                   {
                      if(KeyCloseCnt>=30)	
                      {
                         FautMuteTimer=0;
                         if(FautMuteFlag==0)	
                   	     {
                   	        FautMuteFlag=1;
                   	     }
                   	     else
                   	     {
                   	        FautMuteFlag=0;
                   	        Beep(80);	
                   	  	    DelaymS(80);
                   	     }
	                     Beep(150); 
                      }
                   }
                }
                KeyCloseCnt=0;
             }
          }
       }
       if(RxFlag)
       {
          RxFlag=0;	
       	  //校验接收
       	  switch(ComRPC)
       	  {
       	     case 1:
       	        if(ComRCah[0]!=0xcc)
       	           ComRPC=0;	
       	        break;
       	  	 case 2:
       	  	    if(ComRCah[1]!=0x99)
       	           ComRPC=0;	
       	        break;
       	  	 case 3:
       	  	    if(ComRCah[2]!=0x99)
       	           ComRPC=0;	
       	        break;
       	  	 case 4:
       	  	    if(ComRCah[3]!=0xcc)
       	           ComRPC=0;	
       	        break;
       	     default:
       	        if(ComRPC>=7)
                {
	               if(ComRPC==(ComRCah[4]+6))
	               {
		              j=ComRCah[4];
		              for(k=0;k<ComRCah[4];k++)   
                         j=j ^ ComRCah[5+k];   
                      if(j==ComRCah[ComRCah[4]+5])
                      {
                         //校验正确	
                         switch(ComRCah[5])
                         {
                            case 0x80:
                               if(ComRCah[4]==1)
                               {
                   	              SetPack();
			                      ComCah[4]=1;
			                      ComCah[5]=0x80;
			                      ComCah[6]=0x81;
			                      NBSendData(7);
                               }
                   	           break;
       	                    case 0x81://写ID
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
			                      NBSendData(7);
                               }
                   	           break;
                   	        case 0x82:   //开测试报警音
                               if(ComRCah[4]==1)
                               {
                               	  if(TestBeepTimer==0 && IsFireFlag==0 && TestKeyFlag==0)
                               	  {
                               	     TestBeepTimer=40;
                               	     FireFun=0;
                               	     Volume=3;
                               	     _tb1on=1;
                               	  }
                   	              SetPack();
			                      ComCah[4]=1;
			                      ComCah[5]=0x82;
			                      ComCah[6]=0x83;
			                      NBSendData(7);
                               }
                   	           break;
                   	        case 0x83:   //关闭测试报警音
                               if(ComRCah[4]==1)
                               {
                               	  if(TestBeepTimer>0)
                               	  {
                               	     TestBeepTimer=0;
                               	     if(IsFireFlag==0 && TestSelfFlag==0)
                               	     {
                               	        _tb1on=0;
                               	        OpenSound(0,0);
                               	     }
                               	  }
                   	              SetPack();
			                      ComCah[4]=1;
			                      ComCah[5]=0x83;
			                      ComCah[6]=0x82;
			                      NBSendData(7);
                               }
                   	           break;
                   	        case 0x84://读ID
                   	           if(ComRCah[4]==1)
                               {
                   	              SetPack();
			                      ComCah[4]=4;
		                          ComCah[5]=0x84;
			                      ComCah[6]=MyID[0];
			                      ComCah[7]=MyID[1];
			                      ComCah[8]=MyID[2];
			                      ComCah[9]=ComCah[8] ^ ComCah[7] ^ ComCah[6] ^ 0x80;
			                      NBSendData(10);
                               }
                   	           break;
                   	        case 0x85://读设置
                   	           if(ComRCah[4]==1)
                               {
                   	              SetPack();
                   	              ComCah[4]=11;
                   	              ComCah[5]=0x85;
                   	              ComCah[6]=IR0;
                   	              ComCah[7]=IRFir;
                   	              ComCah[8]=IRFirOut;
                   	              ComCah[9]=IRMuteFir;
                   	              ComCah[10]=IRComp;
                   	              ComCah[11]=MaxDCDark;
                   	              ComCah[12]=IRCurBk;
                   	              ComCah[13]=Set1;
                   	              ComCah[14]=Set2;
                   	              ComCah[15]=BeepHz;
                   	              ComCah[16]=0;
                   	              for(j=4;j<16;j++)
                   	                 ComCah[16]=ComCah[j] ^ ComCah[16];
                   	              NBSendData(17);
                               }
                   	           break;
                   	        case 0x86://写设置
                   	           if(ComRCah[4]==10)
                               {
                   	              WR_EE(IR0SetAdd,ComRCah[6]);
                   	              WR_EE(FirSetAdd,ComRCah[7]);
                   	              WR_EE(FirOutAdd,ComRCah[8]);
                   	              WR_EE(MuteFirAdd,ComRCah[9]);
                   	              WR_EE(MaxCompAdd,ComRCah[10]);
                   	              WR_EE(MaxDCDarkAdd,ComRCah[11]);
                                  WR_EE(Set1Add,ComRCah[12]);
                   	              WR_EE(Set2Add,ComRCah[13]);
                   	              WR_EE(BeepHzAdd,ComRCah[14]);
                   	              WR_EE(IRCurBkAdd,ComRCah[6]);
                   	              WR_EE(IRCurBkAdd+1,ComRCah[6] ^ 0xff);
                   	              WR_EE(IRCurBkAdd+0x20,ComRCah[6]);
                   	              WR_EE(IRCurBkAdd+0x21,ComRCah[6] ^ 0xff);
                                  k=0;
                                  for(j=6;j<15;j++)
                                     k=k + ComRCah[j];
                                  //校验字节       
                                  WR_EE(SetXorAdd,k);
                                  TestFlash();
                                  SetPack();
                                  ComCah[4]=1;
                                  ComCah[5]=0x86;
                                  ComCah[6]=0x87;
                                  NBSendData(7);
                                  AddTest=1;
                               }
                   	           break;
                   	        case 0x87://读使用时间
                   	           if(ComRCah[4]==1)
                               {
                   	              SetPack();
			                      ComCah[4]=3;
			                      ComCah[5]=0x87;
			                      ComCah[6]=LifeCount	>> 8;
			                      ComCah[7]=(unsigned char)(LifeCount & 0xff);
			                      ComCah[8]=ComCah[7] ^ ComCah[6] ^ ComCah[5] ^ 3;
			                      NBSendData(9);
                               }
                   	           break;
                   	        case 0x88://写使用时间
                   	           if(ComRCah[4]==3)
                               {
                   	              WR_EE(LifeHAdd,ComRCah[6]);
                   	              WR_EE(LifeHAdd+1,ComRCah[7]);
                   	              WR_EE(LifeHAdd+2,ComRCah[6]+ComRCah[7]);
                   	              WR_EE(LifeHAdd+0x20,ComRCah[6]);
                   	              WR_EE(LifeHAdd+0x21,ComRCah[7]);
                   	              WR_EE(LifeHAdd+0x22,ComRCah[6]+ComRCah[7]);
                   	              TestFlash();
                   	              LifeTimer=0;
                                  SetPack();
                                  ComCah[4]=1;
                                  ComCah[5]=0x88;
                                  ComCah[6]=0x89;
                                  NBSendData(7);
                               }
                   	           break;
                   	        case 0x89://读状态
                   	           if(ComRCah[4]==1)
                               {
                   	              SetPack();
                   	              ComCah[4]=12;
                                  ComCah[5]=0x89;
                   	              ComCah[6]=MyType;
                                  ComCah[7]=Ver;
                                  /*ComCah[8]=(unsigned char)(volt >> 8);
                                  ComCah[9]=(unsigned char)(volt & 0xff);*/
                                  ComCah[8]=(unsigned char)(volt / 100);
                                  ComCah[9]=(unsigned char)((((volt %100)/10)<<4)+(volt%10));
                                  ComCah[10]=IRAD;
                                  ComCah[11]=IRDCDarkAD;
                                  ComCah[12]=IRDarkAD;
                                  ComCah[13]=IRADAve[2];
                   	              ComCah[14]=CurTemp; 
                   	              ComCah[15]=0;
                   	              if(SetErr)
                   	                 ComCah[15]=ComCah[15] | 1;
                   	              if(IRBkErr)
                   	                 ComCah[15]=ComCah[15] | 2;
                   	              if(IRErr)
                   	                 ComCah[15]=ComCah[15] | 4;
                   	              if(IRDarkErr)
                   	                 ComCah[15]=ComCah[15] | 8;
                   	              if(GameOverFlag)
                   	                 ComCah[15]=ComCah[15] | 16;
                   	              if(LowFlag)
                   	                 ComCah[15]=ComCah[15] | 32;
                   	              if(DirtyFlag)
                   	                 ComCah[15]=ComCah[15] | 64;
                   	              if(IsFireFlag)
                   	                 ComCah[15]=ComCah[15] | 128;
                   	              ComCah[16]=0;
                   	              if(MuteFlag)
                   	                 ComCah[16]=ComCah[16] | 1;
                   	              if(FautMuteFlag)
                   	                 ComCah[16]=ComCah[16] | 2;
                   	              if(TestSelfFlag)
                   	                 ComCah[16]=ComCah[16] | 4; 
                   	              ComCah[17]=0;
                   	              for(j=4;j<17;j++)
                   	                 ComCah[17]=ComCah[17] ^ ComCah[j];
                   	              NBSendData(18);
                   	              AddTest=1;
                               }
                   	           break;
                         }
                         if(AddTest>0)
       	                 {
       	                    AddTest=0;
       	                    TestLED();
       	                 }
                      } 
                      IsComming=0;
                      ComRPC=0;
       	           }   
       	        }
       	        break;
       	  }
       }
       GCC_CLRWDT();
    }
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

//数量  l
void NBSendData(unsigned char l)
{
   unsigned char i;
   for(i=0;i<l;i++)
   {
      _utxr_rxr=ComCah[i];
	  while(_utxif==0);
      while(_utidle==0);
   }
}

void SetPack(void)
{
   ComCah[0]=0xcc;
   ComCah[1]=0x99;
   ComCah[2]=0x99;
   ComCah[3]=0xcc;
}

unsigned char TestLED(void)
{
   unsigned char r,a,b;
   unsigned int i,j;
   r=0;
   j=0;
   _sda0en=1;	///OP0使能
   _sda1en=1;	///OP1使能
   _sds0=1;_sds4=1;_sds3=1;
   
   GCC_DELAY(200);
   _sds3=0;
   GCC_DELAY(3000);
   IRDCDarkAD=GetADC(2);
   _sds4=0;
   if(IRDCDarkAD<MaxDCDark)
   {
      r=1;
   }
   if(r>0)
   {
   	  r=0;
      GCC_DELAY(2000);
      IRDarkAD=GetADC(2);
      if(IRDarkAD<64 && IRDarkAD>11)
         r=1;
   }
   if(r>0)
   {
      _isgdata0=(Set2 & 0xf0)>>3;
      _isgenc |= 0x81;		///_isgenc=0x81;
      GCC_DELAY(100);
      
      _sadc0=0b00001111;
      _sadc1=0b01101010;       //AD定位到OP1
      _adcen=1;
      _start=1;
      _start=0;
      while(_adbz==1);  
      r=_sadoh;
      _isgenc=0;
      for(a=0;a<3;a++)
      {
      	 _start=1;
         _start=0;
         while(_adbz==1);  
         b=_sadoh;
         if(b>r)
            r=b;
      }
      _adcen=0;
      if(r>IRDarkAD)
      {
         j=r-IRDarkAD;
         //j=1;		///j=K3V;
         //j=i*j/100;	///j=i*j/100;
      }
      else
         j=0;
      r=1;
   }
   _sds0=0;
   _sda0en=0;
   _sda1en=0;
   if(r>0)
   {
   	  //温度补偿  
   	  //是否温度补偿
   	  if((Set1 & 0x30)>0)
      {
      	 i=0;
   	     switch(Set1 & 0x30)
   	     {
   	        case 0x10:
   	           if(TempAD>TempHighGrd0)
   	              i=(j>>4);
   	           break;
   	        case 0x20:
   	           if(TempAD>TempHighGrd1)
   	              i=(j>>3);
   	           else
   	           {
   	              if(TempAD>TempHighGrd0)
   	                 i=(j>>4);
   	           }
   	           break;
   	        case 0x30:	
   	           if(TempAD>TempHighGrd1)
   	              i=(j>>4)+(j>>3);
   	           else
   	           {
   	              if(TempAD>TempHighGrd0)
   	                 i=(j>>4);
   	           }
   	  	       break;
   	     }
   	     j=j+i;
      }
      if((Set1 & 0xC0)>0)
      {
      	 i=0;
   	     switch(Set1 & 0xC0)
   	     {
   	        case 0x40:
   	           if(TempAD<TempLowGrd0)
   	              i=(j>>4);
   	           break;
   	        case 0x80:
   	           if(TempAD<TempLowGrd1)
   	              i=(j>>3);
   	           else
   	           {
   	              if(TempAD<TempLowGrd0)
   	                 i=(j>>4);
   	           }
   	           break;
   	        case 0xC0:	
   	           if(TempAD<TempLowGrd1)
   	              i=(j>>4)+(j>>3);
   	           else
   	           {
   	              if(TempAD<TempLowGrd0)
   	                 i=(j>>4);
   	           }
   	  	       break;
   	     }
   	     j=j-i;
      }
   	  IRAD=(unsigned char)(j & 0xff);	
   }
   return r;
}

//测试AD的通道，c=1 电压，c=2 OP1，c=3 温度
unsigned char GetADC(unsigned char c)
{
   unsigned char r;
   switch(c)
   {
      case 1:
         _sadc0=0b00000000;
         _sadc1=0b00001010;       //AD定位到AN0
         break;
      case 2:	
         _sadc0=0b00001111;
         _sadc1=0b01101010;       //AD定位到OP1
   	     break;
   	  default :
   	  	 _sadc0=0b00000001;
         _sadc1=0b00001010;       //AD定位到AN1
   	  	 break;
   	  
   }	
   _adcen=1;
   _start=1;
   _start=0;
   while(_adbz==1);  
   if(c==1 && _sadol>=0x80)
   		r=_sadoh+1;				///如果检测电压且低四位大于0x80
   else 
   		r=_sadoh;
   _adcen=0;
   return r;	
}

void FireBegin(void)   //火警处理
{
   IsFireFlag=1;
   FiredFlag=1;
   TestFun=8;	         
   Fire_OutTimer=0;
   MuteNumber=0;
   dHz=LowHz;      
   Volume=0;
   //打开0.5S时基
   _tb1on=1;   
   FireFun=0;
   //OpenSound(0xff,0xff);
}

void FireExit(void)
{
   IsFireFlag=0;	
   Fire_OutFireCnt=0;
   _tb1on=0;
   MuteNumber=0;
   OpenSound(0,0);
}

void DelaymS(unsigned int t)
{
   unsigned int i;	
   for(i=0;i<t;i++)
      GCC_DELAY(1000);
}

//持续时间mS
void Beep(unsigned char t)
{
   _ptmrpl=	(unsigned char)(Hz[BeepHz] & 0xff);//0x65;
   _ptmrph=	(unsigned char)(Hz[BeepHz] >>8);//1;
   _ptmal=(unsigned char)((Hz[BeepHz]>>1) & 0xff);//178;
   _ptmah=(unsigned char)(Hz[BeepHz] >>9);//0;	            //DUTY=1/2
   _pton=1;
   EN1=1;
   EN2=1;
   DelaymS(t);
   _pton=0;
   EN1=0;
   EN2=0;
}
     
//打开报警声音 p 频率，v 音量,v=0  关闭   
void OpenSound(unsigned char p,unsigned char v)
{
   switch(v)
   {
      case 1:
         _ptmrpl=	(unsigned char)(Hz[BeepHz] & 0xff);
         _ptmrph=	(unsigned char)(Hz[BeepHz] >>8);
         _ptmal=(unsigned char)((Hz[BeepHz]>>4) & 0xff);
         _ptmah=(unsigned char)(Hz[BeepHz] >>12);
   
         //_ptmrpl=0x4d;
         //_ptmrph=1;	    //3K
         //_ptmal=20;
         //_ptmah=0;	    //DUTY=1/16
         _pton=1;
         EN2=1;
         EN1=0;
         break;
      case 2:
         _ptmrpl=	(unsigned char)(Hz[BeepHz] & 0xff);
         _ptmrph=	(unsigned char)(Hz[BeepHz] >>8);
         _ptmal=(unsigned char)((Hz[BeepHz]>>3) & 0xff);
         _ptmah=(unsigned char)(Hz[BeepHz] >>11);
         
         //_ptmrpl=0x4d;
         //_ptmrph=1;	     //3K
         //_ptmal=42;
         //_ptmah=0;	     //DUTY=1/16
         _pton=1;
         EN1=1;
         EN2=0;
         break;
      case 3:
         _ptmrpl=(unsigned char)(Hz[p] & 0xff);
         _ptmrph=(unsigned char)(Hz[p]>>8);	
         _ptmal=(unsigned char)((Hz[p]>>1) & 0xff);
         _ptmah=(unsigned char)(Hz[p]>>9);	       //DUTY=1/2
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

void Twinkle(void)
{
   OpenLED;
   DelaymS(5);  //5mS
   CloseLED;
}

void SetMult(unsigned char s)
{
   switch(s)
   {
      case nA20mV:   
         _sdpgac0=	10;				//R1 ;10*100K
	     _sdpgac1=	2;              //R3=10K  R2=100K
         break;	
      case nA30mV:   
         _sdpgac0=	10;				//R1 ;10*100K
	     _sdpgac1=	3;              //R3=10K  R2=300K
         break;
   	  case nA40mV:
   	     _sdpgac0=	10;				//R1 ;10*100K
	     _sdpgac1=	4;              //R3=10K  R2=400K
   	     break;
   	  //case nA50mV:   
      //   _sdpgac0=	20;				//R1 ;20*100K
	  //   _sdpgac1=	0b01000101;     //R3=20K  R2=500K
      //   break;
   	  case nA60mV:
   	     _sdpgac0=	10;				//R1 ;10*100K
	     _sdpgac1=	6;              //R3=10K  R2=600K
   	     break;
   	  case nA80mV:
   	     _sdpgac0=	10;				//R1 ;10*100K
	     _sdpgac1=	8;              //R3=10K  R2=800K
   	     break;
   	  case nA100mV:   
         _sdpgac0=	10;				//R1 ;10*100K
	     _sdpgac1=	10;             //R3=10K  R2=10*100K
         break;
   	  case nA120mV:
   	     _sdpgac0=	12;				 //R1 ;12*100K
	     _sdpgac1=	10;              //R3=10K  R2=10*100K
   	     break;
   	  case nA160mV:
   	     _sdpgac0=	16;				 //R1 ;16*100K
	     _sdpgac1=	10;              //R3=10K  R2=10*100K
   	     break;
   	  case nA200mV:
   	     _sdpgac0=	20;				 //R1 ;20*100K
	     _sdpgac1=	10;      //R3=10K  R2=1000K
   	     break;
   	  case nA240mV:
   	     _sdpgac0=	20;				 //R1 ;20*100K
	     _sdpgac1=	12;      //R3=10K  R2=1200K
   	     break;
   	  case nA320mV:
   	     _sdpgac0=	20;				//R1 ;20*100K
	     _sdpgac1=	16;     //R3=10K  R2=1600K
   	     break;
   	  case nA400mV:
   	     _sdpgac0=	20;				 //R1 ;20*100K
	     _sdpgac1=	20;              //R3=10K  R2=2000K
   	     break;
   	  case nA480mV:
   	     _sdpgac0=	20;				//R1 ;30*100K
	     _sdpgac1=	24;             //R3=10K  R2=1600K
   	     break;
   	  case nA640mV:
   	     _sdpgac0=	20;				//R1 ;20*100K
	     _sdpgac1=	32;             //R3=10K  R2=3200K
   	     break;
   	  case nA800mV:   
         _sdpgac0=	20;				//R1 ;20*100K
	     _sdpgac1=	40;             //R3=10K  R2=2000K
         break;
      case nA1000mV:   
         _sdpgac0=	20;				           //R1 ;30*100K
	     _sdpgac1=	50;             //R3=10K  R2=4000K
         break;
   }	
}



#if 0
//检测带隙
void TestBG(void)
{
   unsigned int j,k;
   unsigned long m;
   _vbgren=1;		///带隙使能
   //测试Bandgap
   _sadc0=0b00101111;
   _sadc1=0b00101010;   
   GCC_DELAY(5000);    //400      
   //测试Bandgap
   _start=1;
   _start=0;
   while(_adbz==1);
   _vbgren=0;
   _adcen=0;
   if(_sadol>=0x80)
      m=_sadoh+1;
   else
      m=_sadoh;
   volt=30600/m;      //通过带隙AD反推电池电压
   K3V=volt/3;        //判断低电压
   if((volt%3)>1)
      K3V++;
   if(K3V>130)
      K3V=130;
   //电压补偿291--282  2%,281--272 3%,271--262 4%,261--254  5%,253--  8%   
   if(volt<=291)
   {
      if(volt>=282)
         m=K3V*102;	
      else
      {
         if(volt>=272)
            m=K3V*103;     	
         else
         {
            if(volt>=262)
               m=K3V*104;
            else
            {
               if(volt>=254)
                  m=K3V*105;
               else
                  m=K3V*108;	
            }
         }
      }
      K3V=m/100;
      if((m%100)>=50)
         K3V++;
   }
   switch(Set1 & 0x0c)
   {
   	  case 4:   //2.6V
   	     j=260;
   	     k=265;
   	     break;
   	  case 8:   //2.7V
   	     j=270;
   	     k=275;
   	     break;
   	  case 12:  //2.8V
   	     j=280;
   	     k=285;
   	     break;
   	  default:  //2.5V
   	     j=250;
   	     k=255;
         break;
   } 
   if(volt>=k)
   	  LowFlag=0;	
   if(volt<j)	
   	  LowFlag=1;     
};
#endif

void TestVolte(void)	///新增的获取电压函数
{
	unsigned int j,k;
	unsigned long r;
	r=GetADC(1);
	volt=(330*r)/255;	///电压值
	switch(4)	/// Set1 & 0x0c
   {
   	  case 0:   //0.9V
   	     j=90;
   	     k=95;
   	     break;
   	  case 4:   //1.0V
   	     j=100;
   	     k=105;
   	     break;
   	  case 8:  //1.1V
   	     j=110;
   	     k=115;
   	     break;
   	  default:  //1.2V
   	     j=120;
   	     k=125;
         break;
   } 
   if(volt>=k)
   	  LowFlag=0;	
   if(volt<j)	
   	  LowFlag=1;
	
}

void TestTmp(void)
{	
   //取得温度AD
   _pa0=0;	/// OpenLED;
   GCC_DELAY(50);       //50uS延时
   TempAD=GetADC(3);    //温度AD
   _pa0=1;	/// CloseLED;
}

void TestFlash(void)
{
   unsigned char b,c,TempCah[10];
   //取ID
   MyID[0]=RD_EE(IDAdd);
   MyID[1]=RD_EE(IDAdd+1);
   MyID[2]=RD_EE(IDAdd+2);
   
   TempCah[0]=RD_EE(IDAdd+3);
   TempCah[1]=MyID[0] + MyID[1] + MyID[2];
   if(TempCah[0]!=TempCah[1])
   {
      MyID[0]=0;
      MyID[1]=0;
      MyID[2]=0;
   }
   //取运行时间
   TempCah[0]=RD_EE(LifeHAdd);
   TempCah[1]=RD_EE(LifeHAdd+1);
   TempCah[2]=RD_EE(LifeHAdd+2);
   TempCah[3]=TempCah[0] + TempCah[1];
   if(TempCah[2]==TempCah[3])
   {
      LifeCount=TempCah[0];
      LifeCount=(LifeCount<<8)+TempCah[1];
   }
   else
   {
   	  TempCah[0]=RD_EE(LifeHAdd+0x20);
      TempCah[1]=RD_EE(LifeHAdd+0x21);
      TempCah[2]=RD_EE(LifeHAdd+0x22);
      TempCah[3]=TempCah[0] + TempCah[1];
      if(TempCah[2]==TempCah[3])
      {
         LifeCount=TempCah[0];
         LifeCount=(LifeCount<<8)+TempCah[1];
      }
      else
         LifeCount=0;
   }
   
   BeepHz=BeepHzDef;
   LowHz=LowHzDef;
   HighHz=HighHzDef;
   
   //取蓝光和红光设置  
   for(b=0;b<9;b++)
      TempCah[b]=RD_EE(b);
   TempCah[9]=RD_EE(SetXorAdd);
   b=0;
   for(c=0;c<9;c++)
      b=b + TempCah[c];
   if(TempCah[9]==b)
   {
      //测试成功
   	  if(TempCah[0]>0 && TempCah[1]>0 && TempCah[2]>0 && TempCah[4]>0)
   	  {
   	  	 SetErr=0;
   	  	 IR0=TempCah[0];
   	  	 IRFir=TempCah[1];
   	  	 IRFirOut=TempCah[2];
   	  	 IRMuteFir=TempCah[3];
   	  	 IRComp=TempCah[4];
   	  	 MaxDCDark=TempCah[5];
         if(MaxDCDark==0)
            MaxDCDark=MaxEMI;
   	  	 Set1=TempCah[6];
   	  	 Set2=TempCah[7];
   
         if(TempCah[8]==0)
         {
            BeepHz=0;	
            LowHz=0;
            HighHz=1;
         }
         else
         {
   	  	    if(TempCah[8]>=15)
   	  	    {
   	  	       BeepHz=15;
   	  	       HighHz=15;
   	  	       LowHz=14;
   	  	    }
   	  	    else
   	  	    {
   	  	       BeepHz=TempCah[8];
   	  	       LowHz=BeepHz-1;
   	  	       HighHz=BeepHz+1;
   	  	    }
   	  	 }
   	  }
   	  else
   	     SetErr=1;
   }
   else
   	  SetErr=1;
   if(SetErr>0)
   {
      //全部采用默认值	
   	  IR0=0;
   	  IRFir=IRFirDef;
   	  IRFirOut=IRFirOutDef;
   	  IRMuteFir=0;
   	  IRComp=IRCompDef;
   	  MaxDCDark=MaxEMI;
   	  Set1=Set1Def;
   	  Set2=IRSetDef;
   }
   //取得当前的背景值
   if((Set1 & 2)>0)
   {
      TempCah[0]=RD_EE(IRCurBkAdd);
      TempCah[1]=RD_EE(IRCurBkAdd+1);
      IRBkErr=0;
      if((TempCah[0]^TempCah[1])!=0xff)
      {
         TempCah[0]=RD_EE(IRCurBkAdd+0x20);
         TempCah[1]=RD_EE(IRCurBkAdd+0x21);
         if((TempCah[0] ^ TempCah[1])!=0xff)
         {
         	//采用标定值作为平均值
         	IRADAve[0]=0;
         	IRADAve[1]=0;
         	IRADAve[2]=0;
         	IRADAve[3]=0;
         	IRCurBk=0;
         	IRBkErr=1;
         }
      }
      if(IRBkErr==0)
      {
      	 if(TempCah[0]==0)
      	 {
      	    //采用标定值作为平均值
         	IRADAve[0]=0;
         	IRADAve[1]=0;
         	IRADAve[2]=0;
         	IRADAve[3]=0;
         	IRCurBk=0;
         	IRBkErr=1;	
      	 }
      	 else
      	 {
      	    IRCurBk=TempCah[0];
            IRADAve[0]=IRCurBk;
            IRADAve[1]=0;
            IRADAve[2]=IRCurBk;
            IRADAve[3]=0;
      	 }	
      }
   }
   else
   {
      IRBkErr=0;
   }
   SetMult(Set2 & 0x0f);
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
    DelaymS(7);
	R_TMP2=0x00;
	if(_sda0o) 	R_TMP2=R_TMP2|0x01;	//R_TMP2.0=1;	
S_ADJ_OPA_LOOP_ADD:
	++R_TMP0;
	++_sda0vos;
    DelaymS(7);
	if(_sda0o!=(R_TMP2&0x01)) goto S_ADJ_OPA_LOOP_ADD_OK;	//R_TMP2.0
	_acc=0b00111111&_sda0vos;
	if(_acc==0b00111111) goto S_ADJ_OPA_LOOP_ADD_S;
	goto S_ADJ_OPA_LOOP_ADD;	
S_ADJ_OPA_LOOP_ADD_OK:

S_ADJ_OPA_LOOP_SUB_S:
	R_TMP1=		0b00111111;
	_sda0vos=	0b11111111;
//	OPA_ADJ_DELAY();
    DelaymS(7);
	R_TMP2=0x00;
	if(_sda0o) 	R_TMP2=R_TMP2|0x01;	//R_TMP2.0=1;
S_ADJ_OPA_LOOP_SUB:
	--R_TMP1;
	--_sda0vos;
//	OPA_ADJ_DELAY();
    DelaymS(7);
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
	DelaymS(7);
	R_TMP2=0x00;
	if(_sda1o) 	R_TMP2=R_TMP2|0x01;	//R_TMP2.0=1;	
S_ADJ_OPA1_LOOP_ADD:
	++R_TMP0;
	++_sda1vos;
	DelaymS(7);
	if(_sda1o!=(R_TMP2&0x01)) goto S_ADJ_OPA1_LOOP_ADD_OK;	//R_TMP2.0
	_acc=0b00111111&_sda1vos;
	if(_acc==0b00111111) goto S_ADJ_OPA1_LOOP_ADD_S;
	goto S_ADJ_OPA1_LOOP_ADD;	
S_ADJ_OPA1_LOOP_ADD_OK:

S_ADJ_OPA1_LOOP_SUB_S:
	R_TMP1=		0b00111111;
	_sda1vos=	0b11111111;
	DelaymS(7);
	R_TMP2=0x00;
	if(_sda1o) 	R_TMP2=R_TMP2|0x01;	//R_TMP2.0=1;
S_ADJ_OPA1_LOOP_SUB:
	--R_TMP1;
	--_sda1vos;
	DelaymS(7);
	if(_sda1o!=(R_TMP2&0x01)) goto S_ADJ_OPA1_LOOP_SUB_OK;
	_acc=0b00111111&_sda1vos;
	if(_acc==0b00000000) goto S_ADJ_OPA1_LOOP_SUB_S;
	goto S_ADJ_OPA1_LOOP_SUB;
S_ADJ_OPA1_LOOP_SUB_OK:

	_sda1vos=(R_TMP1+R_TMP0)/2;
		
}

void _INI_SM_OPA(void)
{	
//初始化Sink
	_isgenc=0x00;	//disable the sink current isgenc
	_isgdata0=11;	//current valve(mA)=50+10*N
//初始化OPA
	_sda0c=	0b01000001;		//enable OPAMP0,bandwidth=600KHz
	_sda1c=	0b01000011;		//enable OPAMP1,bandwidth=600KHz
	_sdsw=		0b01100111;		//选择开关	;---ON:SDS6 SDS5 SDS2 SDS1 SDS0
	_sdpgac0=	20;				//设置 R1 ;N*100K
	_sdpgac1=	0b01000000+20;	//bit7~bit6設置R3(00=10K,01=20K,10=30K,11=40k),bit5~bit0设置 R2 ;N*100K	
}

unsigned char RD_EE(unsigned char RD_EE_addr)
{
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

void INT1(void)
{
   _int1f=0;
   //自检按键
   if((TestKeyFlag==0) && (TestBeepTimer==0))	
   {
      TestKeyFlag=1;
      //开定时器1
      KeyOpenCnt=0;
      KeyCloseCnt=0;
      _ston=1;
   }	
}

void UART(void)
{
   _usimf=0;
   while(_urxif>0)
   {
      ComRCah[ComRPC]=_utxr_rxr;
	  if(ComRPC<24)
	     ComRPC++;
	  else
	   	 ComRPC=0;   
	  RxFlag=1;	
   }
   IsComming=3;
}

void STMA(void)
{
   _stmaf=0;	///STM 比较器 A 匹配中断请求标志位
   Timer1Flag=1;
}

void STB0(void)
{
   _tb0f=0;	///时基 0 中断请求标志位
   Tb0_flg=1;
}

void STB1(void)
{
   _tb1f=0;	///时基 1 中断请求标志位
   Tb1_flg=1;
}
