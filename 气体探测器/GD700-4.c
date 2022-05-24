//C:\Program Files\Holtek MCU Development Tools\HT-IDE3000V8.x\BIN\HT-IDE3000.EXE
//李海斌
//2021年11月29日:串口配置完成
//2021年12月7日:温度采样及十位运放采样
//2022年1月6日：串口通讯框架
//2022年2月11日：读取AD值（使用12位运放）
//2022年3月29日：完善通讯协议（读，写，校验）
//2022年4月7日：火警及退出(声音)
//2022年4月22日：按键
//2022年4月29日：静音(了解WiFi数据上报下发使用)
//2022年5月7日：WiFi数据类型上传
//2022年5月13日：报警输出

#include "BA45F5250.h"
typedef unsigned char uc;
typedef unsigned int ut;
typedef unsigned long int ul;

#define  EN1 _pa7
#define  EN2 _pb4
#define  TEMPIO _pb2		//温度采样控制引脚
#define  R 'R'
#define  G 'G'
#define  WIFI_COM
//----------WiFi指令----------------------//
#define  test      1
#define  ch4_state 2
#define  ch4_value 5
#define  life      11
#define  fault     45
#define  mute      14

#define  MyType 9
#define  MyVer  0x10
#define  ABS(X) ((X)>=0)?(X):(-(X))

#define  OpenLED_R    _isgenc |= 0b10000010;	///ISINK1 引脚灌电流使能,50mA
#define  OpenLED_G    _isgenc |= 0b10000001;	///ISINK0 引脚灌电流使能,50mA
#define  CloseLED_R   _isgenc &= 0b10000001;	///ISINK1 引脚灌电流除能
#define  CloseLED_G   _isgenc &= 0b10000010;	///ISINK0 引脚灌电流除能
#define  ALARM_OUT 	  {_papu0 =1;_pa0 =1;}

volatile bit  Tb0_flg;
volatile bit  Tb1_flg;
//volatile bit  RxFlag;       				//串口有新数据标志
volatile bit  transfer_complete_flag;		//接收传输完成标志
volatile bit  BaseErr;
volatile bit  FireErr;
volatile bit  FullErr;
volatile bit  SenShort;
volatile bit  SenOpen;
volatile bit  IsFire;
volatile bit  Buzzerflag;
//volatile bit  tamperflag;
volatile bit  selftestflag;
volatile bit  Int0flag; 
volatile bit  Timer1Flag;
volatile uc ComPC;		   	     //发送数据指针
volatile uc ComCah[54];          //发送缓冲区
volatile uc ComRPC;		       	 //接收数据指针
volatile uc ComRCah[24];         //接收缓存
volatile uc MyID[3];
volatile uc pointer_length;		//指针长度(记录接受的字节长度)

struct  {
    ut current_OPAvalue;	//当前运放值
    ut current_refvalue;	//当前电位器值
    ut Base_value;     		// 初始值
    ut Fire_value;     		// 火警标定
    ut Full_value;     		// 100%LEL
}Valves={0,0,0,0,0};

struct  {
    uc pressdown_flag :1;	// 按下标志
    uc freed_flag :1;		// 释放标志
    uc First_blood :1;		// 按1下事件
    uc Double_kill :1;		// 2下事件
    uc Triple_kill :1;		// 3下事件
    uc Unstoppable :1;		// 长按事件
    uc double_flag :1;		// 长按标志
    uc long_flag:1;
}Key_S={0,0,0,0,0,0,0,0};	//key_status

struct  {
    uc check_result :2;	//自检状态 	0自检中1成功2失败3开机自检
    uc gas_state :1;	//燃气状态	0未报警1报警
    uc lifeOver :1;  	//寿命 		0到期  1正常使用
    uc muteflag :1;  	//静音		0未消音1消音
    uc faultflag :3;  	//故障值 	0故障  1开路2短路4标定错误
/*    uc :1;*/
}ps={3,0,0,0,0};//product_status

volatile ut OpAveVal[2] = {0};//运放平均值
volatile ut RefAveVal[2]= {0};//基准平均值
volatile uc BaseCount =0;
volatile uc ShortCount =0;
volatile uc OpenCount =0;
volatile uc FireOutTimer=0;			//火警保持计数
volatile ut LifeCount=0;
volatile uc FireCount = 0;
volatile uc Fire_OutFireCount=0;	//退出火警计数
volatile uc alarmcount=0;
volatile uc TempAD,CurTemp;         //温度采样
volatile uc OP1,OP0;
volatile uc sadoh,sadol;
volatile uc StartTimer=0;
volatile uc lightcount=0;

//----------wifi相关-------------------//
volatile bit  SetupNetState;   //曾经配网成功
volatile uc ZigBeeNetState;    //联网状态
volatile uc  ZigBeeVer;        //协议版本号；
//PID
const uc WifiPID[37]={"{\"p\":\"bmsojo0y3pmh7qwj\",\"v\":\"1.0.0\"}"};
//-15度到55度
const uc TMP[71]={30,32,34,35,37,39,
                 41,43,45,47,49,51,53,55,58,60,
                 62,65,67,70,72,75,77,80,83,86,
                 88,91,94,97,100,102,105,108,111,114,
                 117,120,122,125,128,131,133,136,139,141,
                 145,147,150,152,155,157,160,162,164,167,
                 170,172,174,176,178,180,182,184,186,188,
                 190,192,194,196,198};

//-----------存储地址定义---------------------------//
#define Base_valueAdd 0x00		//零值
#define Fire_valueAdd 0x03		//火警增量
#define Full_valueAdd 0x06		//量程
#define OPA_multipleAdd 0x09
#define IDAdd 0x10          	//设备ID  H+M+L+Xor
#define LifeHAdd 0x14	    	//运行时间单位12小时 H+L+Xor
#define SetupNetStateAdd 0x15	//曾经配网

void Stm0AInit(void);
void Stm1Init(void);
void UARTInit(void);
void OPA_int(void);
void INT0Init(void);
void S_ADJ_OPA(void );
void S_ADJ_OPA1(void );
void SendByte(uc l);
void SendWord(unsigned long Word);
void SendData(uc L);
void NBSendData(unsigned char l);
uc RD_EE(uc RD_EE_addr);
void Read_EE(uc , uc *buff, uc L);
void WR_EE(uc ,uc);
void Write_EE(uc ,uc ,uc );
void DelaymS(ut t);
void Beep(uc t);
void BuzzerOff(void);
void BuzzerOn(uc);
void Twinkle(uc);
uc GetFault(void);

ut GetADC(uc c);
int get_difference(void);
void get_temperature(void);
uc GetTemp(void);
void Com_Management(void);
void wifi_upload(uc i);
void SetPack(void);
void TestFlash(void);
void function(void);
void keyDetect(void);

void main() {
    unsigned char i, j, k;

    _scc = 0x01;
    _hircc = 0x05;
    while(_hircf == 0);

	ComRPC=0;
	ComPC=0;
	pointer_length=0;
	transfer_complete_flag=0;
	Tb1_flg=0;
	Tb0_flg=0;
	//RxFlag=0;
	SenShort =0;
	SenOpen =0;
	IsFire =0;
	Buzzerflag = 0;
	selftestflag=0;
	Int0flag = 0;
	Timer1Flag = 0;

    _mp0 = 0x80;
    _acc = 128;

    while(_acc != 0) {
        _iar0 = 0;
        ++_mp0;
        _acc = _acc - 1;
    }

    for(i = 1; i < 8; i++) {
        _mp1l = 0x80;
        _mp1h = i;
        _acc = 128;

        while(_acc != 0) {
            _iar1 = 0;
            ++_mp1l;
            _acc = _acc - 1;
        }
    }

    _mp1h = 0;
    _iar1 = 0;
    _mp1l = 128;
    
if(0){
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
   /*******变量初始化********/
		
   
    _wdtc=0x57;
    
    _pbc2=0;
    TEMPIO=1;			//温度采样控制引脚
    
    //----------ADC引脚(温度)---------------------//
    _pbc5=1;
	_pbs13=1;
	_pbs12=1;
	//----------报警输出引脚---------------------//
	_pac0=0;
	_pa0=0;
		
    //初始化时基
    _pscr = 2;		///32k
    _tb0c = 0x07;
    _tb0e = 1;   //时基0设置1秒定时
    _tb1c = 0x06; //时基1设置为0.5秒
    _tb1e = 1;

    //初始化PWM
    _pas15=0;
    _pas14=1;	//PA6为PTP
    _pac6 = 0;	//pa6输出
    _pac7 = 0;
    _pbc4 = 0;
    _ptmc0 =	0b00000000;	//设置分频比 FSYS/4   4M系统时钟
    _ptmc1 =	0b10101000;
    _ptmc2 =	0b00000000;
    _ptmal =	0x9c;
    _ptmah =	0;	   //DUTY=50%
    _ptmrpl =	0xc8;//0x39;
    _ptmrph =	0;//0x01;	//PERIOD=3.2KHz
    
   //初始化定时器1 按键 960/32000=30mS
   _stm1c0=0b01000000;      //fsys=32K，
   _stm1c1=0b11000001;
   _stm1al=0xC0;
   _stm1ah=3;
   _stm1ae=1;
   _st1on=0;
    

    //开时基0
    _tb0on = 1;
    _tb1on = 1;
    _emi = 1;
	
	UARTInit();
	Stm0AInit();
	INT0Init();
	TestFlash();
	
	OPA_int();
	DelaymS(20);
	S_ADJ_OPA();
	S_ADJ_OPA1();
//	SendByte(sizeof(char));				//1
//	SendByte(sizeof(short int));		//2
//	SendByte(sizeof(int));				//2
//	SendByte(sizeof(long));				//4

//Twinkle(G);
//DelaymS(100);
//Twinkle(G);

volatile ut count=0; 
volatile ut count1=0;
OpenLED_G
OpenLED_R 
_isgdata1=0;
    while(1) {
//        if(Tb1_flg) {
//            Tb1_flg = 0;
//        }

            if(Timer1Flag){
            	//按键处理
            	Timer1Flag=0;
            	keyDetect();
	            if(Key_S.First_blood){
	            	//if(++count1 > 3) count1=0;
	            	count1 += 10;
	            	Key_S.First_blood=0;
	            	
//					SendByte(1);
//					SendByte(ps.muteflag);
//					SendByte(alarmcount);
//					_isgenc ^= 0b00000010;
	            }
	            if(Key_S.Double_kill){
	            	if(++count > 3) count=0;
	            	SendByte(count);
	            	Key_S.Double_kill=0; 	
					
	            }
				
				if(Key_S.Triple_kill){
					Key_S.Triple_kill=0;
					ComCah[0]=0x55;
					ComCah[1]=0xaa;
					ComCah[2]=0;
					ComCah[3]=4;
					ComCah[4]=0;
					ComCah[5]=1;
					ComCah[6]=0;
                	NBSendData(7);
	            	
	            }
				
            }
			
			if(Tb0_flg){
				Tb0_flg=0;
			    GCC_CLRWDT();
//				get_temperature();
//				CurTemp=GetTemp();
//				SendByte(TempAD);
//				SendByte(((CurTemp/10)<<4)+(CurTemp%10));
/*				DelaymS(10);*/
				function();
				if(Key_S.Unstoppable){
					//Key_S.long_flag用于只做一次判断
					if(Key_S.long_flag){
						Key_S.long_flag = 0;
						ps.check_result = 0;
						wifi_upload(test);
					}
					if(IsFire && (ps.check_result ==0)){
						ps.check_result = 1;
						wifi_upload(test);
					}
					//selftestflag=!selftestflag;
	            }
//				if((++lightcount > 4) && (IsFire==0)) {
//					lightcount=0;Twinkle(G);
				}
				//Beep(80);

			if(Int0flag){
//				Int0flag=0;
//				_isgenc ^= 0b00000010;
//				_pb2 ^= 1;
//				SendByte(9);
				
			}
			if(transfer_complete_flag) Com_Management();
        	GCC_CLRWDT();
    }
}

void function(void){
	unsigned int sum, a;
	
	a = GetADC(2);
	Valves.current_OPAvalue = a;
	//----------取平均值------------------------//
	sum = (OpAveVal[0] << 2) + OpAveVal[1];  ///加上上一次的最后两位①
    sum = sum - OpAveVal[0];
    sum = sum + a;
    OpAveVal[0] = (sum >> 2); 	
    OpAveVal[1] = (sum & 3);  	///这里取最后两位(配合①使采样值平滑变化)
//	SendWord(a);
//	SendWord(OpAveVal[0]);

    a = GetADC(1); 				//AN3 基准(滑动变阻器)
    Valves.current_refvalue = a;
    sum = (RefAveVal[0] << 2) + RefAveVal[1];
    sum = sum - RefAveVal[0];
    sum = sum + a;
    RefAveVal[0] = (sum >> 2); //4平均
    RefAveVal[1] = (sum & 3);
//    SendWord(a);
//    SendWord(RefAveVal[0]);
	if(warm_up_time > 180) {        ///预热完成
		if(RefAveVal[0] > 1901 && RefAveVal[0] < 2194){		//1.3V<AN3<1.5V   电源电压2.8V
			
			//判断开路、短路。判断前逻辑左移，可在不满足条件时自动清零
            a = 0;
            ShortCount <<= 1;
            OpenCount <<= 1;
            FireCount <<= 1;
            Fire_OutFireCount <<= 1;
			
			//----------开路及短路判断--------------//
			
			if(OpAveVal[0] > (RefAveVal[0] + 1635)){ //AN0>AN3+1.11V{
                a = 1;
                ShortCount = ShortCount | 1;
            }
            if(OpAveVal[0] < (RefAveVal[0] - 731)){ //AN0<AN3-0.5V{
                a = 1;
                OpenCount = OpenCount | 1;
            }  
        }
        else{
        	a = 1;
            BaseCount = BaseCount << 1;
            BaseCount = BaseCount | 1; // 判断基准值是否正确
        }
        if(ShortCount == 0xff) SenShort = 1;

        if(ShortCount == 0	 ) SenShort = 0;

        if(OpenCount == 0xff ) SenOpen = 1;

        if(OpenCount == 0	 ) SenOpen = 0;

        if(BaseCount == 0xff ) BaseErr = 1;

        if(BaseCount == 0	 ) BaseErr = 0;
        
        if(!a){
			if( !FireErr && !FullErr && !BaseErr){
				///采样值大于等于 基准值+火警增量 ->10%的（爆炸极限）
				if(OpAveVal[0] > RefAveVal[0] + Valves.Fire_value){
					FireCount = FireCount | 1;	
				}
				///采样值小于等于 基准值+火警增量/2->10%的标定除以2得5%LEL（爆炸极限）
				else if(OpAveVal[0] < RefAveVal[0] + (Valves.Fire_value >> 1)){
					Fire_OutFireCount |= 1;
				}
				
				if(IsFire==0){
					if(FireCount == 0x0F) {
						IsFire=1; 
						Buzzerflag = 1;
						//OpenLED_R;
					}
				}
				else if(Fire_OutFireCount == 0xFF){
					IsFire=0;
					CloseLED_R
				}
			}
        }
	}
	else
	{
		warm_up_time++;
        Twinkle(1);
	}
//	SendByte(FireCount);
//	SendByte(Fire_OutFireCount);
//	SendByte(IsFire);
//	SendByte(alarmcount);
	
	//---------报警声音控制，buzzerflag控制响完三声------//
	if(Buzzerflag ){
		SendByte(alarmcount);
    	switch(++alarmcount) {
	        case 1:
	        	if(!ps.muteflag)
	            	BuzzerOn(0);
	            OpenLED_R
	            break;
	        case 2:
		        BuzzerOff();
	            CloseLED_R
	            break;
	        case 3:
		        if(!ps.muteflag)
		            BuzzerOn(1);
	            OpenLED_R
	            break;
	        case 4:
				BuzzerOff();
	            CloseLED_R
	            break;
	        case 5:
		        if(!ps.muteflag)
		            BuzzerOn(2);
	            OpenLED_R
	            break;
	        case 6:
	            BuzzerOff();
	            CloseLED_R
	            break;
	        case 7:
	            alarmcount = 0;
	            if(!IsFire) Buzzerflag=0;
	            break;
	    }
	}
}

void Com_Management(void){
	uc j=0, k=0;						
	transfer_complete_flag=0;
	if(ComRCah[0]==0xcc){	//标定通讯
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
            		case 0x81:		//写ID
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
                   	case 0x82:   //开测试报警音
                            if(ComRCah[4]==1)
                            {                            
                   	           SetPack();
			                   ComCah[4]=1;
			                   ComCah[5]=0x82;
			                   ComCah[6]=0x83;
			                   SendData(7);
			                   BaseErr=1;
			                   IsFire=1;
			                   Buzzerflag = 1;
                            }
                   	        break;
                   	case 0x83:   //关闭测试报警音
                            if(ComRCah[4]==1){
                   	           SetPack();
			                   ComCah[4]=1;
			                   ComCah[5]=0x83;
			                   ComCah[6]=0x82;
			                   SendData(7);
			                   BaseErr=0;
			                   IsFire=0;
                            }
                   	        break;
            		case 0x84:		// ID
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
					case 0x85:		//读探测器设置
						if(ComRCah[4]==1){
               	           SetPack();
		                   ComCah[4] = 8;
	                       ComCah[5] = 0x85;
		                   ComCah[6] = Valves.Base_value >> 8;
		                   ComCah[7] = Valves.Base_value & 0xFF;
		                   ComCah[8] = Valves.Fire_value >> 8;
		                   ComCah[9] = Valves.Fire_value & 0xFF;
		                   ComCah[10] = Valves.Full_value >> 8;
		                   ComCah[11] = Valves.Full_value & 0xFF;
		                   ComCah[12] = RD_EE(OPA_multipleAdd);
		                   ComCah[13] = 0x82^ComCah[6]^ComCah[7]^ComCah[8]^\
		                   				ComCah[9]^ComCah[10]^ComCah[11]^ComCah[12];//82=85^7
		                   SendData(14);
                        }
                   	    break;
					case 0x86:		//写设置
						if(ComRCah[4]==8){
							ComCah[1] = ComRCah[6];
			                ComCah[2] = ComRCah[7];
			                ComCah[3] = ComRCah[6] ^ ComRCah[7];
			                ComCah[4] = ComRCah[8];
			                ComCah[5] = ComRCah[9];
			                ComCah[6] = ComRCah[8] ^ ComRCah[9];
			                ComCah[7] = ComRCah[10];
			                ComCah[8] = ComRCah[11];
			                ComCah[9] = ComRCah[10] ^ ComRCah[11];
			                ComCah[10] = ComRCah[12];
							ComCah[11] = ComRCah[12] ^ 0xFF;
							Write_EE(Base_valueAdd, 1, 11);
							TestFlash();
							SetPack();
							ComCah[4] = 1;
							ComCah[5] = 0x86;
							ComCah[6] = 0x87;
							SendData(7);
							}
						break;
					case 0x87:		//读寿命
						if(ComRCah[4]==1){
							SetPack();
							ComCah[4] = 3;
							ComCah[5] = 0x87;
							ComCah[6] = LifeCount >> 8;
			                ComCah[7] = LifeCount & 0xFF;
			                ComCah[8] = 3 ^ 0x87 ^ ComCah[6] ^ ComCah[7];
							SendData(9);
						}
						break;
					case 0x88:		//写寿命
						if(ComRCah[4]==3){
							LifeCount = (ComRCah[6]<<8) + ComRCah[7];
							ComCah[4] = ComRCah[6];
			                ComCah[5] = ComRCah[7];
			                ComCah[6] = ComRCah[6] ^ ComRCah[7];
			                Write_EE(LifeHAdd, 4, 3);
			                TestFlash();
			                SetPack();
			                ComCah[4] = 1;
			                ComCah[5] = 0x88;
			                ComCah[6] = 0x89;
			                SendData(7);	
						}
						break;
					case 0x89:		//读采样
						if(ComRCah[4]==1){
							SetPack();
							ComCah[4] = 12;
			                ComCah[5] = 0x89;
			                ComCah[6] = MyType;
							ComCah[7] = MyVer;
							ComCah[8] = OpAveVal[0] >> 8;
							ComCah[9] = OpAveVal[0] & 0xFF;
							ComCah[10] = RefAveVal[0] >> 8;
							ComCah[11] = RefAveVal[0] & 0xFF;
							ComCah[12] = Valves.current_OPAvalue >> 8;
							ComCah[13] = Valves.current_OPAvalue & 0xFF;
							ComCah[14] = Valves.current_refvalue >> 8;
							ComCah[15] = Valves.current_refvalue & 0xFF;
							ComCah[16] = 0;
							if(SenShort) ComCah[ 16] = ComCah[ 16] | 1;
			                if(SenOpen) ComCah[ 16] = ComCah[ 16] | 2;
			                if(BaseErr) ComCah[ 16] = ComCah[ 16] | 16;
			                if(FireErr) ComCah[ 16] = ComCah[ 16] | 32;
			                if(FullErr) ComCah[ 16] = ComCah[ 16] | 64;
			                if(IsFire) ComCah[ 16] = ComCah[ 16] | 128;
							ComCah[17] =  ComCah[4] ^ ComCah[5] ^ ComCah[6] \
										^ ComCah[7] ^ ComCah[8] ^ ComCah[9] \
										^ ComCah[10] ^ ComCah[11] ^	ComCah[12] \
										^ ComCah[13] ^ ComCah[14] ^	ComCah[15] \
										^ ComCah[16];	
							SendData(18);
						}
						break;
					case 0x97:
						SendWord(Valves.Base_value);
						SendWord(Valves.Fire_value);
						SendWord(Valves.Full_value);
						break;
					case 0x98:
						//wifi_upload(0);
						if(ComRCah[6] == 0){
							if(ComRCah[7] == 0) CloseLED_G
							else _isgdata0 = ComRCah[7];
						}
						if(ComRCah[6] == 1){
							if(ComRCah[7] == 0) CloseLED_R
							else _isgdata1 = ComRCah[7];
						}
						SendByte(_isgdata0);
						SendByte(_isgdata1);
						break;
						
						
							
//					case 0x87:
//					case 0x88:
//					case 0x89:
//					case 0x90:
//					case 0x91:

            	}
            }
		}
	}
#ifdef WIFI_COM
	else{		//WiFi通讯
		//头校验
		if(ComRCah[0] == 0x55 && ComRCah[1] == 0xAA){
			//SendByte(1);SendByte(pointer_length);SendByte(ComRCah[5]);
			//长度校验
			if(pointer_length == (ComRCah[5]+7)){//SendByte(2);
				//和校验
				for(k=0; k < (pointer_length-1); k++)   
            		j=j + ComRCah[k];
            	if(j == ComRCah[pointer_length-1]){//SendByte(3);
            		
            		ZigBeeVer=ComRCah[2];
            		//选择命令字
            		switch(ComRCah[3]){
            			case 1:		//查询产品信息
            				ComCah[0]=0x55;
	       	  	             ComCah[1]=0xaa;
	       	  	             ComCah[2]=ZigBeeVer;
	       	  	       	     ComCah[3]=1;
	       	  	       	     ComCah[4]=0;
	       	  	       	     ComCah[5]=36;
	       	  	       	     for(j=0;j<36;j++)
	       	  	       	        ComCah[6+j]=WifiPID[j];
	       	  	       	     NBSendData(42);
	       	  	       	     break;
	       	  	       	case 2:   //通知配网状态
	       	  	       	     ZigBeeNetState=ComRCah[6];
	       	  	       	     ComCah[0]=0x55;
	       	  	             ComCah[1]=0xaa;
	       	  	             ComCah[2]=ZigBeeVer;
	       	  	       	     ComCah[3]=2;
	       	  	       	     ComCah[4]=0;
	       	  	       	     ComCah[5]=0;
	       	  	       	     NBSendData(6);
	       	  	       	     if(ZigBeeNetState==4)   //已经连接到云端
	       	  	       	     {   //配网成功提示
	       	  	       	        if(SetupNetState==0)
	       	  	       	        {
	       	  	       	           SetupNetState=1;
	       	  	       	           //存储状态
	       	  	       	           WR_EE(SetupNetStateAdd,1);
	                   	           WR_EE(SetupNetStateAdd+1,0xfe);
	                   	           Twinkle(R);
	       	     	               Beep(150);
	       	     	               DelaymS(100);
	       	                       Twinkle(R);
	       	     	               Beep(150);
	       	  	       	        }
	       	  	       	        DelaymS(3000);
	       	  	       	        wifi_upload(0);
	       	  	       	     }
       	  	       	     	break;
       	  	       	     case 3:    //智能配网
	       	  	       	 case 4:	//AP配网
	       	  	       	     //删除一个指令
	//       	  	       	     ZB_SubFun=3;  //等待配网成功
	//       	  	       	     ZB_TestTimer=0;     //测试计时器
	//                         ZB_ComCount=0;
	       	  	       	     //删除配网标志
	       	  	       	     WR_EE(SetupNetStateAdd,0);
				             WR_EE(SetupNetStateAdd+1,0);
				             SetupNetState=0;
	       	  	       	     break;
	       	  	       	 case 9:	//下发
	       	  	       	 	 if(ComRCah[6] == 14){
	       	  	       	 	 	ps.muteflag = ComRCah[9];
	       	  	       	 	 	ComCah[0]=0x55;
								ComCah[1]=0xaa;
								ComCah[2]=14;
								ComCah[3]=9;
								ComCah[4]=0;
								ComCah[5]=0;
								NBSendData(6);
								DelaymS(20);
								wifi_upload(mute);
	       	  	       	 	 }break;
	       	  	       	     
            		}
            	}
			}
		}
	}
#endif
}

void wifi_upload(uc i){
	ComCah[0]=0x55;
	ComCah[1]=0xaa;
	ComCah[2]=ZigBeeVer;
	ComCah[3]=5;
	ComCah[4]=0;
	switch (i){
		case test://自检 test
			ComCah[5]=5;
			ComCah[6]=1;
			ComCah[7]=4;
			ComCah[8]=0;
			ComCah[9]=1;
			ComCah[10] = ps.check_result;
			NBSendData(11);
			break;
		case ch4_state:
			ComCah[5]=5;
			ComCah[6]=2;
			ComCah[7]=4;
			ComCah[8]=0;
			ComCah[9]=1;
			ComCah[10] = IsFire ? 1:0;
			NBSendData(11);break;
		case ch4_value://燃气值ch4_value
			ComCah[5]=8;
			ComCah[6]=5;
			ComCah[7]=2;
			ComCah[8]=0;
			ComCah[9]=4;
			//只用低位，1000以内
			ComCah[10]=0;
			ComCah[11]=0;
			ComCah[12]=ComRCah[8];
			ComCah[13]=ComRCah[9];
			NBSendData(14);
			break;
		case life://寿命（正常为1，到期为0）life
			ComCah[5]=5;
			ComCah[6]=11;
			ComCah[7]=1;
			ComCah[8]=0;
			ComCah[9]=1;
			ComCah[10]=ps.lifeOver;
			NBSendData(11);break;
		case mute://消音 mute
			ComCah[5]=5;
			ComCah[6]=14;
			ComCah[7]=1;
			ComCah[8]=0;
			ComCah[9]=1;
			ComCah[10]=ps.muteflag;
			NBSendData(11);break;
		case fault:
			//故障标志（位变量->0b00000 标定 短路 开路）fault
			ComCah[5]=5;
			ComCah[6]=45;
			ComCah[7]=5;
			ComCah[8]=0;
			ComCah[9]=1;
			ComCah[10] = GetFault();
			NBSendData(11);
			break;
		default:
			//自检
			ComCah[5] = 33;	//长度 添加指令后加长
			//ComCah[6] = 5;
			ComCah[6]  = 1;
			ComCah[7]  = 4;
			ComCah[8]  = 0;
			ComCah[9] = 1;
			ComCah[10] = 1;
			//燃气值
			//ComCah[11] = 8;
			ComCah[11] = 5;
			ComCah[12] = 2;
			ComCah[13] = 0;
			ComCah[14] = 4;
			ComCah[15] = 0;
			ComCah[16] = 0;
			ComCah[17] = 0;
			ComCah[18] = 100;
			//寿命
			//ComCah[20] = 5;
			ComCah[19] = 11;
			ComCah[20] = 1;
			ComCah[21] = 0;
			ComCah[22] = 1;
			ComCah[23] = 1;
			//消音
			//ComCah[26] = 5;
			ComCah[24] = 14;
			ComCah[25] = 1;
			ComCah[26] = 0;
			ComCah[27] = 1;
			ComCah[28] = 1;
			//故障
			//ComCah[32] = 5;
			ComCah[29] = 45;
			ComCah[30] = 5;
			ComCah[31] = 0;
			ComCah[32] = 1;
			ComCah[33] = 3;
			//火警状态
			ComCah[34] = 2;
			ComCah[35] = 4;
			ComCah[36] = 0;
			ComCah[37] = 1;
			ComCah[38] = IsFire? 1:0;
			NBSendData(39);
			break;
	}
	
}

void TestFlash(void)
{
	BaseErr = 0;
	FireErr = 0;
	FullErr = 0;
	unsigned char c, b[4];
	//取ID
	Read_EE(IDAdd, b, 4);
	c = b[0] + b[1] + b[2];
	if(c != b[3]) {			//如果没有写入ID，则读出FF FF FF
		MyID[0]=255;
		MyID[1]=255;
		MyID[2]=255; 
	}else{
		MyID[0]=b[0];
		MyID[1]=b[1];
		MyID[2]=b[2]; 
	}
	//取基础值
	Read_EE(Base_valueAdd, b, 3);
    if((b[0] ^ b[1]) == b[2]) {
	    if(b[0] == 0 && b[1] == 0) {
	        b[0] = 0x08;
	        b[1] = 0x00;
	    }
		Valves.Base_value = (b[0] << 8) + b[1];
    } else {
        Valves.Base_value = 0;
        BaseErr= 1;	//未写基础值
    }
    //取火警增量
    Read_EE(Fire_valueAdd, b, 3);
    if((b[0] ^ b[1]) == b[2]) {
        if(b[0] == 0 && b[1] == 0) {
            b[0] = 0x00;            ///0xb8=184
            b[1] = 0xB8;
        }

        Valves.Fire_value = (b[0] << 8) + b[1];
    } else {
        Valves.Fire_value = 0;
        FireErr = 1; //没标定火警
    }
    //取量程
    Read_EE(Full_valueAdd, b, 3);
    if((b[0] ^ b[1]) == b[2]) {
        if(b[0] == 0 && b[1] == 0) {    ///0x600=1536
            b[0] = 0x06;
            b[1] = 0x00;
        }
        Valves.Full_value = (b[0] << 8) + b[1];
    } else {
        Valves.Full_value = 0;
        FullErr = 1;//未标定量程
    }
    //取放大倍数
    Read_EE(OPA_multipleAdd, b, 2);           //放大倍数
    if((b[0] ^ b[1]) == 0xff) {
        b[0] = b[0] & 15;

        if(b[0] > 2){
            b[0] = 1;
            WR_EE(OPA_multipleAdd, 1);
        }
    } 
    else {
        b[0] = 1;
    }
    //此处将取出的放大倍数写入寄存器
    //--------------------------------------//
    switch(b[0]){
    	case 0:_sdpgac1 = 0b11000000 + 2;break;//5
    	case 1:_sdpgac1 = 0b11000000 + 4;break;//9
    	case 2:_sdpgac1 = 0b11000000 + 8;break;//17
    }
    //--------------------------------------//
    
    //探测器累计运行时间   12小时的倍数
    Read_EE(LifeHAdd, b, 3);		 
    if((b[0] ^ b[1]) == b[2])
        LifeCount = (b[0] << 8) + b[1];
    else
        LifeCount = 0; //生命周期计数
    
}

uc GetFault(void){
	uc i=0;
	if(SenOpen) 						i |= 1;
	if(SenShort) 						i |= 2;
	if(BaseErr || FireErr || FullErr) 	i |= 4;
	return i;
}

//检测按键单击、双击、三击、长按 
void keyDetect(void){
	static uc  down_counter = 0; //按键持续按下计数器
	static uc  up_counter = 0;   //有效单击后抬起按键后的计数器
	static uc  up_flag =0;      //有效单击抬起按键后，生成按键抬起标志
	//static uc  Key_S.double_flag =0; //按下次数
	if(_pa3 == 1){
		
		
		if(down_counter <= 234)          //长按阈值 7S，跟据实际情况更改！！
		{
			down_counter++; 
		}
		else                             //按键按下到7S，就判断长按时间成立，生成长按标志 
		{ 	
			Key_S.Unstoppable = 1;        //长按键标志置位
			Key_S.long_flag = 1;
			//----------若只检测长按-----------------//
			//_st1on = 0;
			//down_counter = 0;             //清除按下计数
			////Key_S.double_flag = 0;
			//----该程序需要检测长按抬起，注释掉-----//
			return ;                       //跳出函数
		}
	}
	else
	{	
		if(down_counter > 2)        //消抖阈值：按下时间大于60ms，则为有效。可更改！！
		{
			up_flag = 1;            //有效单击抬起按键后，生成按键抬起标志 
			//距离上次单击时间在1S之间，再次单击，则认为再次发生点击事件
			if( up_counter > 1 && up_counter < 35)   //双击间隔阈值，可更改！！
			{ 
				//如果双击键标志置位
				if(Key_S.double_flag == 1){
					Key_S.double_flag = 0;
					Key_S.Triple_kill = 1;     //三击键标志置位
					_st1on = 0;
					up_flag = 0;               //清除按键抬起标志
					up_counter = 0;            //清除按键抬起计数
					
				}
				else
				{
					Key_S.double_flag=1;	//表示按下两次
					//up_flag = 0; 清除按键抬起标志(要最后才清除以确定是两次还是三次)
					up_counter = 0;      	//清除按键抬起计数(下次单击间隔计数)
				}
				//长按抬起
				if(Key_S.Unstoppable){
					Key_S.Unstoppable=0;
					ps.check_result = 0;	
					_st1on = 0;
					down_counter = 0;            //清除按下计数
				}				
			} 
		} 
        down_counter = 0;
	}
	if(up_flag) //有效单击抬起后，启动计数
       up_counter++;
	//1S内 ,没有连击，则视为点击结束;配合上面点击间隔阈值，一起更改！！
	if(up_counter > 35)      
	{ 
		if(Key_S.double_flag == 1){
			Key_S.Double_kill = 1;     //双击键标志置位
		}else{
			Key_S.First_blood = 1;    //单击键标志置位	
	  	}
	  	_st1on = 0;
		up_counter = 0;
		up_flag = 0;
		Key_S.double_flag=0;
	}
//	SendByte(down_counter);
//	SendByte(up_counter);
//	SendByte(click_counter);
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

void BuzzerOn(uc i){
	_pton = 1;
	switch(i){
		case 0:
			EN1 = 1;EN2 = 0;break;
		case 1:
			EN1 = 0;EN2 = 1;break;
		case 2:
			EN1 = 1;EN2 = 1;break;
	}   
}
void BuzzerOff(void){
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

void Twinkle(uc color)
{
	if(color=='R')
	{	OpenLED_R;
   		DelaymS(5);  //5mS
   		CloseLED_R;
   	}else{
   		OpenLED_G;
		DelaymS(5);  //5mS
		CloseLED_G;
	}
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
//数量  l
void NBSendData(unsigned char l)
{
   unsigned char i,j;
   j=0;
   for(i=0;i<l;i++)
   {
      j=j+ComCah[i];
      _txr_rxr=ComCah[i];
      while(_tidle==0);
      while(_txif==0);
	  //while((_usr & 2)==0);
   }
   _txr_rxr=j;
   while(_tidle==0);
   while(_txif==0);
   //while((_usr & 2)==0);
}

void SendByte(unsigned char l)
{
	//发送1个字节
	_txr_rxr=l;
	while(_txif==0);//等待数据从缓冲器加载到移位寄存器中
	while(_tidle==0);//等待数据发送完成
}
void SendWord(unsigned long Word){
	SendByte(Word >> 8);
    SendByte(Word & 0xFF);
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

void Read_EE(uc Address, uc *buff, uc L){
	//连续读，EE地址，存放数组，读取长度
	unsigned char i;
	for(i=0; i<L; i++) {
		buff[i] = RD_EE(Address+i);
	}
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

void Write_EE(uc WR_EE_addr,uc DATA_addr,uc L)
{	//连续写，EE地址，ComCah中待写入数据的地址，写入长度。
	unsigned char i;
	for(i=0; i<L; i++) {
		WR_EE(WR_EE_addr+i,ComCah[DATA_addr+i]);
	}
}

void __attribute((interrupt(0x08))) INT0(void){
	//int0中断
	_int0f=0;
	Int0flag=1;
	_st1on = 1;	//开stm1
	
}

void INT0Init(void){
	_pas07=0;
	_pas06=0;	//PA3/INT0/STP1I
	_ifs13 = 0;_ifs12=1;//PA3做int0
	_pac3=1;	//输入
/*	_papu3=1;	//上拉*/
	/*INT0S1~INT0S0
	00：除能
	01：上升沿
	10：下降沿
	11：双沿*/
	_int0s1=0;
	_int0s0=1;	//上升沿.
	_emi=1;
	_int0e=1;	//使能int0
}

void __attribute((interrupt(0x3c))) UART(void)
{
	_mff=0;
	_urf=0;
	//30MS定时
	if(_stm1af){
		_stm1af=0;	
		Timer1Flag=1;
	}
	else{
		//接受中断函数
		/*	RxFlag=1;*/
		while(_rxif>0) {
			ComRCah[ComRPC]=_txr_rxr;
			if(ComRPC<24)
				ComRPC++;
			else
				ComRPC=0;
		}
		//----------------清零定时器------------
		_st0on=0;
		_st0on=1;
		//--------------------------------------
	}
}
void Stm1Init(void){
	 //初始化定时器1 按键 30mS
   _stm1c0=0b01000000;      //fsys=32K，
   _stm1c1=0b11000001;
   _stm1al=0xC0;
   _stm1ah=3;
   _stm1ae=1;
   _st1on=0; 
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
	//_sime=1;			//USIM中断使能
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
unsigned int GetADC(unsigned char c)
{
   ut r=0;
   unsigned long sum=0;		//4平均用，量程稍大
   uc i;
   //uc t1,t2;
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
         _sadc1=0b00001010;       //AD定位到AN4、温度
   	  	 break;
   }
   
   for(i=0;i<4;i++){//4次平均
	   _adcen=1;
	   _start=1;
	   _start=0;
	   while(_adbz==1);
//	   SendWord(_sadoh);
//	   SendWord(_sadol);
//	   SendWord(_sadoh<<4);
//	   SendWord(_sadol>>4);
/*无论ADRFS为何值，_sadoh始终为8位，_sadol始终为高4位*/
	   r = (_sadoh<<4) + (_sadol>>4); //12位全部取出
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

int get_difference(void){  
	//获取差值
	volatile ut op0,op1;
	op0=GetADC(1);
	//SendByte(op0 >> 4);
	//SendByte(op0 & 0xFF);
	DelaymS(10);
	op1=GetADC(2);
	//SendByte(op1 >> 4);
	//SendByte(op1 & 0xFF);
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
	_pas11=1;			//A0O-PA4
	_pas12=0;
	_pas13=1;			//A1O-PA5
	_pbs01=1;			//A0PB-PB0
	_pas02=1;
	_pas03=1;			//A1PI-PA1
	_pac1= 1;
	_pac4= 0;
	_pac5= 0;
	_pbc0= 1;
	
	//初始化OPA
	_sda0c=	0b01000001;		//enable OPAMP0,bandwidth=600KHz
	_sda1c=	0b01000010;		//enable OPAMP1,bandwidth=2MHz
	_sdsw=	0b01101000;		//选择开关	;---ON:SDS3		
	_sdpgac0=20;				//设置 R1 ;N*100K
	//▲输出=(1+R2/(R3+10K))*▲输入
	//R3=40,R2=200(5倍)、400（9倍）、800（17倍）
	//bit7~bit6設置R3(00=10K,01=20K,10=30K,11=40k),bit5~bit0设置 R2 ;N*100K
	_sdpgac1= 0b11000000+4;	//R3=40K,R2=400K,放大9倍	
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
