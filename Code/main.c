/*
   FOR Charger 2018
   --Pin configure--
	串口1	串口2  Relay1	Relay2  Relay3 Relay4
		      (L-short) (ok) 	(ng)	(保护表头)


*/

/* ------ kiell 中引用基础头文件 --------*/
#include "N76E003.h"
#include "common.h"
#include "delay.h"
#include "SFR_Macro.h"
#include "Function_define.h"

/* ---------- 定义1/3(常量) --------------*/


// OK信号
#define OK_ON set_P10
#define OK_OFF clr_P10
// NG信号
#define NG_ON set_P11
#define NG_OFF clr_P11
// 短路开关
#define SHORT_ON set_P12
#define SHORT_OFF clr_P12
// 充电开关
#define CHARGE_ON set_P13
#define CHARGE_OFF clr_P13

// 定格电流待测时间 (插入几秒后开始测)
#define RATE_DELAY_TIME 7 		// unit:秒
#define RATE_MIN_CURRENT 500 	// unit:mA

// 定格电流维持时间 (电流值检出后，几称后转入短路测试)
#define SHORT_DELAY_TIME 7 		// unit:秒
#define SHORT_MAX_CURRENT 20 	// unit:mA


//判断电源有无的参考
//#define DCV_OFF 1;				// 1v以下为OFF
//#define DCV_ON 5;				// 5v以上为ON

/* ---------- 定义2/3(变量) --------------*/
static unsigned int dcv_off = 0; 		// 电压输入值
static unsigned int dcv_on = 0; 		// 电压输入值
static unsigned int  DCV_OFF = 1; 		// adc -> dc 电压输入
static unsigned int DCV_ON = 5; 		// adc -> dc 电压输入
static unsigned int adc_val = 0; 		// adc取样值
static float adc_val2 = 0; 		// adc -> dc 电压输入
static unsigned int charge_current = 0; 		// 充电电流
static unsigned int short_current = 0; 		// 充电电流
static bit pass = 1;			// 测试结果
static bit testing = 0;			// 测试中的标记
static bit power_offed= 0;			// 充电器退出标记(它仅有ADC控制)
static bit power_on= 0;			// 充电器接上标记 (表示可进入测试状态，在jugement后置0)
static bit first_boot= 1;			// 充电器接上标记
static bit get_current_ok= 0;			// 已测试电流标记
static bit recovery_ok= 0;			// 已测试电流标记
static UINT8 order = 0;
static UINT8 current_val[8] ={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

void chk_power_on_again(void);


/* ---------- 定义2/3(函数) --------------*/

	/* ---------- 函数类1/4(中断) --------------*/

void timer0_interrupt() interrupt 3 {
  // 停止ADC/串口工作，pass = 0, 设置 get_current_ok / recovery_ok = 1
  /*
  void Timer0_Delay1ms(UINT32 u32CNT)
  {
    clr_T0M;                                		//T0M=0, Timer0 Clock = Fsys/12
    TMOD |= 0x01;                           		//Timer0 is 16-bit mode
    set_TR0;                              		  //Start Timer0
    while (u32CNT != 0)
      {
        TL0 = LOBYTE(TIMER_DIV12_VALUE_1ms); 		//Find  define in "Function_define.h" "TIMER VALUE"
        TH0 = HIBYTE(TIMER_DIV12_VALUE_1ms);
        while (TF0 != 1);                   		//Check Timer0 Time-Out Flag
        clr_TF0;
        u32CNT --;
      }
    clr_TR0;                              		  //Stop Timer0
  }
  */

}

void adc_interrupt() interrupt 11 { 		// ADC 中断, 工作时机： testing = 0 时才启动
	// 转换ADC值, 怎么检测它是由低变高
	// 测试结束，要拔掉电源，才能有以下动作
	// 所以要先检查ADC 有一个低电压，之后再有一个高电压，才认为是有效的power_on
	//adc_val= ADCRH; adc_val<<= 4; adc_val|= (ADCRL & 0X0F);  // 16h -> dec
    //SBUF = ADCRH;
    //SBUF = ADCRL;
   	 //SBUF = 0x55;
   	 P12= ~P12;
   	 clr_ADCF;
   /*
   // adc_val 转数据为DEC
   adc_val2 = adc_val/5*4095;

	// ~testing 测试前 用于自动开始测试 --- 充电器再次插入
	if(~testing){
			chk_power_on_again();
	}
	// testing 测试用 用于检验OCP后，测试产品的复归
	else{
	// 在复归测试中：打开ADCEN，然后delay 若干秒检查PASS。
		if(adc_val2>DCV_ON){
			pass=1;
			clr_ADCEN;				// 	开始测试，停用ADC
		}
		else {
      pass = 0;
			clr_ADCF; set_ADCS;
		}
	}
	*/
}

// 获取电流值 UART0 中断
void uart0_interrupt() interrupt 4 {
  if(RI){
    current_val[order] = SBUF;
    if(order > 7){
      order = 0;
      get_current_ok =1;
    }
    else {
      order ++;
    }
    clr_RI;
  }
  if(TI){
  	clr_TI;
  	//clr_ADCF;
  	//set_ADCS;
  }
}
/*
void uart1_interrupt() interrupt 15 {
  if(TI) {
		P13 = ~P13;
  		//clr_TI;
  }
}
*/
	/* ---------- 函数类2/3(配置) --------------*/
void chk_power_on_again(){
	// 测试结束，要拔掉电源，才能有以下动作
	// 所以要先检查ADC 有一个低电压，之后再有一个高电压，才认为是有效的power_on
	if (first_boot && (adc_val2 > DCV_ON )){ 		//case 1: 开机时有电源 -> 可以马上开始测试
		power_on = 1;
		clr_ADCEN;			// 开始测试，停用ADC
	}
	else {										//case 2: 开机时无电源 -> 同一般情况 ->先检测有低，再检测有高
		if (adc_val2 < DCV_OFF){					//		判断poweroffed的两种情况
			power_offed = 1;
		}
		else{
			power_offed = 0;
		}

		if(power_offed && (adc_val2 > DCV_ON)){   // 		由power_offed的决定是否 power_on
			power_on = 1;
			clr_ADCEN;			// 开始测试，停用ADC	
		}

		if(~power_on){							// 以上没有产生有效的power_on, 继续ADC
			clr_ADCF; set_ADCS;
		}
	}
}

void ioConf(){
	//  准双向，已经在串口初始化定义
	//串口1  p06 / p07
	//串口2  p16 /p02

	P10_PushPull_Mode; 			// Relay 1 control - for ok -> p10
	P11_PushPull_Mode; 			// Relay 2 control - for ng -> p11
	P12_PushPull_Mode; 			// Relay 3 control - for short -> p12
	P13_PushPull_Mode; 			// Relay 4 control - for charge -> p13
	P04_PushPull_Mode; 			// ADC Triggle source

	Enable_ADC_AIN0;			// AIN0 P17

	//P03_PushPull_Mode; 			// Relay 4 control - for protect 
	//P03_PushPull_Mode; 			//  - for start test
	//P05_Input_Mode;				// 定义模拟采样输入
}


	/* ---------- 函数类3/3(功能) --------------*/

 void timer0_monitor_start(){

 }

 void timer0_monitor_stop(){

 }

void reset_judgement(){
	OK_OFF; NG_OFF;	// ok Relay =0 ; ng Relay = 0;
}

void do_a_judgement(){
	// step1 充电电流
	// step2 短路电流
	// step3 复归 (要用上ADC)
	// 1. 输出测试结果(先置0再置1, 防止两个同时有效）
	if(pass){
		NG_OFF; 
		OK_ON;
	}
	else {
		OK_OFF; 
		NG_ON;
	}
	// 2. 设测试结束标记 & 待开始新一轮充电插入检测
	first_boot = 0;
	testing = 0;
	power_on = 0; 

	// 3. 重新开始ADC侦测(开启ADC中断)
	//ADCF ADCS ADCEN 相关寄存器3个
	set_ADCEN;
	set_ADCS;
}

void send_cmd(){
}

void test_flow(){
  float current = 0;
  float re_dcv = 0;
  pass = 1;
	testing = 1;  			// 标记进入测试中
	reset_judgement();		// 进入测试状诚后，OK/NG继电器复位


  //RATE
  Timer0_Delay1ms(7000);
  get_current_ok = 0;
  send_cmd();
  timer0_monitor_start();
  while(~get_current_ok);
  timer0_monitor_stop();
  current = (1+2)/3;  // 实要用上 current_val
  if(~pass || current < 0.8){
    pass = 0; do_a_judgement(); return;
  }

  //SHORT
  Timer0_Delay1ms(1000);
  get_current_ok = 0;
  SHORT_ON;             // shorted
  Timer0_Delay1ms(2000);
  send_cmd();
  timer0_monitor_start();
  while(~get_current_ok);
  timer0_monitor_stop();
  current = (1+2)/3;  // 实要用上 current_val
  if( ~pass || current > 0.02){
    pass = 0; do_a_judgement(); SHORT_OFF; return;
  }

  //OCP
  SHORT_OFF;
  set_ADCEN; set_ADCS; // 在中断中关闭
  timer0_monitor_start();
  while(~recovery_ok);
  timer0_monitor_stop();
  re_dcv = 123;
  if(~pass || re_dcv < 5){
    pass = 0; do_a_judgement();return;
  }
	do_a_judgement();

  // 为防止以上while无限等待，while之前启用定时器，检查超时。（要用timer0)
}


/*
流程说明：
上电检查有没有输出
输出有，用relay 接上电池
等待8秒 （常量）
*/
void main(){
	// 初始化
	ioConf();	
	OK_OFF;
	NG_OFF;
	SHORT_OFF;
	CHARGE_OFF;
	P04=0;
  Timer0_Delay1ms(1000);
  /*
	set_ES_1;
*/
  /*
	set_ES;
	set_EADC;
	set_ETGSEL0;
	set_ETGSEL1;
	clr_ADCF;
	set_ADCEN;
	set_ADCS;

	set_EA;	

*/
	//InitialUART0_Timer1(115200);
	//InitialUART1_Timer3(115200);
  // 测试ADC工作状态
  // ADC完成后，检查电压的数据有效性
  // 电压有效后，插拔控制继电器的ON/OFF
  //取电流数据，用串口0
	while (1){
		/*
		 if(~testing && power_on){
		 	testing = 1;  // 标记进入测试中
		 	//test_flow();
		 }
		*/
		//SBUF = 0X33;
		//SBUF_1 = 0X34;

	//adc_val= ADCRH; adc_val<<= 4; adc_val|= (ADCRL & 0X0F);  // 16h -> dec
    //SBUF = ADCRH;
    //SBUF = ADCRL;
    Timer0_Delay1ms(1000);
    P04=1;
    P13=~P13;
    P04=0;
				
	}
}
