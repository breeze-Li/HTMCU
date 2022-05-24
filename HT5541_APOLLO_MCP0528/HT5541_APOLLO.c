//APOLLO协议手报  20201221
// 4M时钟
//资源分配 时基0 秒定时，时基1  灯的PWM
//闪灯频率不超过1秒
//PA1反向
//写码    7FH  55H  25H  65H  3CH  地址
//20210112  加入：读地址  7FH  55H  25H  65H  3BH   探测器在3B处回地址

#include "BA45F5541.h"

#define MyType 1

//定义中断向量

DEFINE_ISR(INT1, 0x0C);
DEFINE_ISR(CTM0A, 0x24); //边沿超时和定时
DEFINE_ISR(CTM1A, 0x28); //边沿超时和定时
DEFINE_ISR(STB0, 0x2C);  //闪灯控制
DEFINE_ISR(STB1, 0x30);  //检测秒定时

//定义中断函数
void INT1(void);
void STB0(void);
void STB1(void);
void CTM0A(void);
void CTM1A(void);

#define Key _pb0
#define KeyN _pa3
#define LED _pa6

//存储地址定义
#define IDAdd 0x00           //地址编码

//位变量
volatile bit  LedOpen;
volatile bit  JYBit;         //校验位
volatile bit  Sending;       //正在回码;
volatile bit  RstDog;
volatile bit  WriteAdd;      //写地址标志
volatile bit  LEDCmd;        //LED点亮指令
volatile bit  RevBit;
volatile bit  CanFlash;

volatile bit  NewAlarm;

volatile bit  bitstep;

volatile unsigned char MyID,NewID,MyVal;  //产品ID
volatile unsigned char ComStep,ComCah;
volatile unsigned char ComRevAdd;         //接收的地址
volatile unsigned char ComRevCmd;         //接收的指令
volatile unsigned char ComFun;            //写码序列
volatile unsigned char SendFun;
volatile unsigned char IntCount;          //中断次数
volatile unsigned char SendCah;    
volatile unsigned char CanTest;           //控制检测开关

//定义普通函数
unsigned char RD_EE(unsigned char RD_EE_addr);                      //读EEPROM
void WR_EE(unsigned char WR_EE_addr,unsigned char WR_EE_data);      //写EEPROM
void TestFlash(void);      //测试设置项是否正确

                             
void main()
{
   unsigned char i,j;
   
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
   
   _pac=0b10111111;   
   _papu=0b10110111;
   
   _pbc=0b00001111;
   _pbpu=0b00001110;
   
   _pcc=0b00001111;
   _pcpu=0b00001111;
   
   _pawu=0;    
   
   _sledc0=0xFF;
   _sledc1=0x0F;
   
   _pa=0;
   _pb=0;
   _pc=0;
   
   //初始化IO复用功能
   _pas0=0;
   _pas1=0;
   
   _pbs0=0b01010100;

   _pcs0=0;
   
   _ifs0=0;
   
   //初始化时基
   _pscr=2;
   _tb0c=0;
   _tb0e=1;     //时基0设置8mS定时
   
   _tb1c=0x07;  //时基1设置为0.5秒
   _tb1e=1;
   
   //初始化超时定时器,32K频率,统计发码宽度(5.3MS)
   _ctm0c0=0b01000000;
   _ctm0c1=0b11000001;
   _ctm0al=170;
   _ctm0ah=0;
   
   //回码宽度
   _ctm1c0=0b01000000;
   _ctm1c1=0b11000001;
   _ctm1al=11;     //等待回码延时11=330uS,回码9=270uS
   _ctm1ah=0;
   
   //PLT初始化
   
   _pltsw=0b00000000;
   _pltdacc=0;
   _pltc0c=0;  
   _pltc1c=0;  
   _pltc0vos=0b00010000;
   _pltc1vos=0b00010000;
   _pltchyc=0;
   _pltac=0b00000001;
   _pltavos=0b00100000;
   _pltda2l=8;          // 20mA
   
   _wdtc=0x56;   //看门狗周期 4秒

   //初始化中断
   _integ=0b00001100;   //int1 双延中断
   _intc0=0b00001000;   //PLT0//INT1 中断
   _intc1=0b00000000;   
   _intc2=0b00000000;   //STMA PTMA
   _intc3=0b00000000;   //TB1,TB0
   
   TestFlash();
   
   LedOpen=0;    
   CanTest=0;
   MyVal=16;
   ComStep=255;
   ComRevAdd=0;      //接收的地址
   ComRevCmd=0;
   Sending=0;
   LEDCmd=0;
   WriteAdd=0;
   SendFun=0;
   ComFun=0;
   RstDog=0;
   
   NewAlarm=0;
   
   bitstep=0;
   
   _ctm1ae=1;
   _ctm0ae=1;
   _ctm0af=0;
   _ctm1af=0;
   
   CanFlash=1;
   
   //开时基1
   _tb1on=1;
   _tb1e=1;
   _mf3e=1;
   _mf4e=1;
   _emi=1;
   /*
    LED=1;
    GCC_DELAY(5000);
    LED=0;
    while(1)
    {
    	_halt();
    };
    */
    while(1)
    {
       if(Sending==0)
       {
         _halt();
       }
       if(RstDog)
       {
          RstDog=0;
          GCC_CLRWDT();
       }
       
       if(CanTest>0)
       {
       	  CanTest=0;
          j=0;
          for(i=0;i<10;i++)
          {
             if(Key==0 && KeyN>0)
             {
               j++;	
             }
          	 GCC_DELAY(20);
          }
          if(j>8)
          {
             if(MyVal!=64)
             {
                if(LEDCmd==0)
                {
                   if(IntCount==0)
                      NewAlarm=1;	
                }
             }
             MyVal=64;
          }
          else
          {
          	 MyVal=16;
          }
       }
       
       if(WriteAdd)
       {
          WriteAdd=0;
          if(NewID>0 && NewID<127)
          {	
       	     WR_EE(IDAdd,NewID);
       	     WR_EE(IDAdd+1,~NewID);
             MyID=NewID;
          }
       }
    }
}

void DelaymS(unsigned int t)
{
   unsigned int i;	
   for(i=0;i<t;i++)
      GCC_DELAY(1000);
}

void TestFlash(void)
{
   unsigned char a,b;
   a=RD_EE(IDAdd);
   b=RD_EE(IDAdd+1);
   MyID=0;
   if((a ^ b)==0xff)
   {
      if(a>0 && a<=127)
         MyID=a;
   }
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

void CTM0A(void)
{
//超时
   _mf3f=0;
   _ctm0af=0;
   _ct0on=0;
   ComStep=255;
   ComRevAdd=0;
   ComRevCmd=0;
   
   if(LEDCmd==0)
      LED=0;
}

void CTM1A(void)
{  //管理
   _mf4f=0;
   _ctm1af=0;
   _ct1on=0;
   /*
   if(bitstep)
               {
               LED=1;
               bitstep=0;
               }
               else
               {
         	   LED=0;
               bitstep=1;
               }
                   */        
   switch(SendFun)
   {
      case 1:    //回码等待完成
         _ctm1al=9;     //等待回码延时11=330uS,回码9=270uS
         _ctm1ah=0;
         _pltaen=1;
         _pltdac2en=1;
         _plts0=1;
         _ct1on=1; 
         SendFun=2;  
         break;
   	  case 2:    //回码完成
   	     _pltaen=0;
         _pltdac2en=0;
         _plts0=0;
         SendFun=0;
   	     break;
   }
}

void STB0(void)
{
   _tb0f=0;
//火警之后的点灯,PWM周期16mS
   if(LedOpen)
   {
      LedOpen=0;
      LED=0;
   }	
   else
   {
      LedOpen=1;
      LED=1;
   }
}

void STB1(void)
{
   _tb1f=0;
   CanTest++;
   CanFlash=1;
}

void INT1(void)
{
   unsigned int i;
   _int1f=0;
   if(ComStep==255)//初始化时候/超时为255
   {
      //等待上升沿
      if(_pa1==0)//(_pa1>0)
      {
         //---开定时器并且清零-------------------//
         _ct0on=0;
         _ct0on=1;
         //--------------------------------------//
         ComStep=0;
         if(LEDCmd==0)
            LED=0;
         
      }	
   }
   else
   {
      i=_ctm0dh;
      i=i<<2;
      i=i+_ctm0dl;
      _ct0on=0;               	
   	  _ct0on=1;
   	  if(_pa1>0)//(_pa1==0)
   	  {  //下降沿
   	     if(i>35)//1.09MS
   	     {
   	        if(i<145)
   	        {
   	           ComRevAdd=0;
         	   ComRevCmd=0;
         	   ComStep=1;
         	   if(LEDCmd==0)
                  LED=0;
   	        }
   	        else//4.5MS
   	        {
   	           ComStep=0;   //等待上升沿
   	        }
   	     }
   	     else
   	     {
   	        if(ComStep>10)
   	        {  
   	           if(ComStep==11)//校验位
   	           {
   	           	  JYBit=0;
   	              if(IntCount>0)
   	              {
   	           	     _ct1on=0;
                     _ctm1al=11;     //等待回码延时11=330uS
                     _ctm1ah=0;
                     _ct1on=1;  
                     SendFun=1; 
                     if(ComRevAdd==MyID && MyID>0)
                        SendCah=MyVal;
   	              }
   	              else
   	              {
   	                 SendCah=MyVal;	
   	              }
   	           }
   	           else
   	           {
   	           	  if(ComRevAdd==MyID && MyID>0)
                  {
   	                 if(ComStep<19)
   	                 {
   	              	    if((SendCah & 64)>0)
   	              	    {
   	              	       JYBit=~JYBit;	
   	              	       _ct1on=0;
                           _ctm1al=11;     //等待回码延时11=330uS
                           _ctm1ah=0;
                           _ct1on=1;  
                           SendFun=1;
   	              	       
   	              	    }
   	              	    SendCah=SendCah<<1;
   	              	    if(ComStep==18)
   	              	    {
   	              	    	
   	              	       if(MyVal==64)
   	              	          SendCah=5;
   	              	       else
   	              	          SendCah=2;
   	              	          
   	              	    }
   	                 }
   	                 else
   	                 {
   	                 	if(ComStep<22)
   	                 	{
   	                 	   if((SendCah & 4)>0)
   	              	       {
   	              	          JYBit=~JYBit;	
   	              	          _ct1on=0;
                              _ctm1al=11;     //等待回码延时11=330uS
                              _ctm1ah=0;
                              _ct1on=1;  
                              SendFun=1;
   	              	       }
   	              	       SendCah=SendCah<<1;
   	              	       if(ComStep==21)
   	              	       {
   	              	          SendCah=MyType;	
               
   	              	       }	
   	                 	}
   	                 	else
   	                 	{
   	                 	   if(ComStep<25)
   	                 	   {
   	                 	      if((SendCah & 4)>0)
   	              	          {
   	              	             JYBit=~JYBit;	
   	              	             _ct1on=0;
                                 _ctm1al=11;     //等待回码延时11=330uS
                                 _ctm1ah=0;
                                 _ct1on=1;  
                                 SendFun=1;
   	              	          }
   	              	          SendCah=SendCah<<1;
   	              	          if(ComStep==24)
   	              	          {
   	              	             SendCah=MyID;	
   	              	             
   	              	          }	
   	                 	   }
   	                 	   else
   	                 	   {
   	                 	      if(ComStep<32)
   	                 	      {
   	                 	         if((SendCah & 64)>0)
   	              	             {
   	              	                JYBit=~JYBit;	
   	              	                _ct1on=0;
                                    _ctm1al=11;     //等待回码延时11=330uS
                                    _ctm1ah=0;
                                    _ct1on=1;  
                                    SendFun=1;
   	              	             }
   	              	             SendCah=SendCah<<1;
   	                 	      }
   	                 	      else
   	                 	      {
   	                 	         switch(ComStep)
   	                 	         {
   	                 	   	        case 32:
   	                 	   	           JYBit=~JYBit;	
   	              	                   _ct1on=0;
                                       _ctm1al=11;     //等待回码延时11=330uS
                                       _ctm1ah=0;
                                       _ct1on=1;  
                                       SendFun=1;
                                       
   	                 		           break;
   	                 		        /*
   	                 		        case 34:
   	                 	   	           JYBit=~JYBit;	
   	              	                   _ct1on=0;
                                       _ctm1al=11;     //等待回码延时11=330uS
                                       _ctm1ah=0;
                                       _ct1on=1;  
                                       SendFun=1;
                                       
   	                 		           break;
   	                 		        case 35:
   	                 	   	           JYBit=~JYBit;	
   	              	                   _ct1on=0;
                                       _ctm1al=11;     //等待回码延时11=330uS
                                       _ctm1ah=0;
                                       _ct1on=1;  
                                       SendFun=1;
                                       
   	                 		           break;
   	                 		        */
   	                 		        case 41:
   	                 		           if(JYBit>0)
   	                 		           {
   	              	                      _ct1on=0;
                                          _ctm1al=11;     //等待回码延时11=330uS
                                          _ctm1ah=0;
                                          _ct1on=1;  
                                          SendFun=1;
   	                 		           }
   	                 		           //if(IntCount>0)
   	                 		           //   SendCah=MyID;	
   	                 		           break;
   	                 	         }
   	                 	      }
   	                 	   }	
   	                 	}
   	                 }
                  }
   	              if(IntCount>0)
   	              {
   	              	 if(ComStep==41)
   	              	 {
   	              	    SendCah=MyID;
   	              	 }
   	              	 if(ComStep>41 && ComStep<49)
   	              	 {
   	              	 	
   	              	    if((SendCah & 64)>0)
   	              	 	{
   	              	 	   _ct1on=0;
                           _ctm1al=11;     //等待回码延时11=330uS
                           _ctm1ah=0;
                           _ct1on=1;  
                           SendFun=1; 	
   	              	 	}
   	              	 	SendCah=SendCah<<1;
   	              	 	if(ComStep==48)
   	              	       IntCount--;
   	              	 }
   	              }
   	           }
   	        }
   	     }
   	  }
   	  else
   	  {  //上升沿
   	     if(ComStep>0)
   	     {
   	     	if(i>80)
   	           ComStep=0;
   	        else
   	        {
   	           if(ComStep<11)
   	           {
   	              if(i<17)
                     RevBit=0;
                  else
                     RevBit=1; 
   	              if(ComStep<4)
                  {
                     ComRevCmd=ComRevCmd<<1;
                     ComRevCmd=ComRevCmd | RevBit;
                  }
                  else
                  {
                     ComRevAdd=ComRevAdd<<1;
                     ComRevAdd=ComRevAdd | RevBit;
                  }
                  ComStep++;
                  if(ComStep==11)
                  {
                  	 RstDog=1;
                  	 
                     //测试写码序列
                     if(ComRevAdd==0x7f)
                     {
                        ComFun=1;
                        ComStep=0;
                     }
                     else
                     {
                        switch(ComFun)
                        {
                           case 1:
                              if(ComRevAdd==0x55)
                                 ComFun=2;
                              else
                                 ComFun=0;
                              break;
                           case 2:
                              if(ComRevAdd==0x25)
                                 ComFun=3;
                              else
                                 ComFun=0;
                              break;
                           case 3:
                              if(ComRevAdd==0x65)
                                 ComFun=4;
                              else
                                 ComFun=0;
                              break;
                           case 4:
                              switch(ComRevAdd)
                              {
                                 case 0x3c:
                                    ComFun=5;
                                    break;
                                 case 0x3b:
                                    ComFun=6;
                                    break;
                                 default:
                              	    ComFun=0;
                              	    break;
                              }
                              break;
                           case 5:
                              //写码
                              if(ComRevAdd>0 && MyID!=ComRevAdd)
                              {
                                 WriteAdd=1;  
                                 NewID=ComRevAdd;	
                              }
                              ComFun=0;
                              ComStep=0;
                              break;
                              
                        }
                     }
                     if(ComStep>0)
                     {
                     	if(ComFun==6)
                     	{
                           if(MyID>0)
                           {
                           	  ComRevAdd=MyID;
                              if(CanFlash>0)
                           	  {
                           	     LED=1;
                           	  	 CanFlash=0;
                           	  }
                           }
                           else
                           {
                              ComStep=0;
                           }
                           ComFun=0;
                     	}
                     	else
                     	{
                     	   if(NewAlarm>0)
                     	   {
                     	      NewAlarm=0;
                     	      if(MyVal==64)
                     	         IntCount=8;
                     	   }
                           if(ComRevAdd==MyID && MyID>0)
                           {
                           	  //IntCount=0;                 //20210528
                              //准备回码，亮灯
                              if((ComRevCmd & 4)>0)
                              {
                                 if(LEDCmd==0)
                                 {
                                    LEDCmd=1;
                                    LED=1;
                                    _tb0on=0;
                                    _tb0on=1;
         	                        LedOpen=1;
                                 }
                              }
                              else
                              {
                                 if(LEDCmd>0)
                           	     {
                           	        LEDCmd=0;
                                    LED=0;
                                    _tb0on=0;
         	                        LedOpen=0;
                           	     }
                           	     else
                           	     {
                           	  	    if(CanFlash>0)
                           	  	    {
                           	  	       LED=1;
                           	  	       CanFlash=0;
                           	  	    }
                           	     }
                              }
                           }
                           else
                           {
                              //报警中断
                              if(IntCount==0)
                                 ComStep=0;
                           }
                     	}
                     }
                  }
   	           }
   	           else
   	           {
   	              if(ComStep<48)
   	                 ComStep++;
                  else
                     ComStep=0;
   	           }
   	        }	
   	     }
   	  }
   }
}
