/*
   FOR Charger 2018
   --Pin configure--
	串口1	串口2  Relay1	Relay2  Relay3 Relay4
		      (L-short) (ok) 	(ng)	(保护表头)


*/

/* ------ kiell 中引用基础头文件 --------*/
#include "N76E003.h"
#include "Common.h"
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
#define DCV_OFF 1;				// 1v以下为OFF
#define DCV_ON 5;				// 5v以上为ON

/* ---------- 定义2/3(变量) --------------*/
static unsigned int dcv_off = 0; 		// 电压输入值
static unsigned int dcv_on = 0; 		// 电压输入值
static unsigned int adc_val = 0; 		// adc -> dc 电压输入
static unsigned int charge_current = 0; 		// 充电电流
static unsigned int short_current = 0; 		// 充电电流
static bit pass = 0;			// 测试结果
static bit testing = 0;			// 测试中的标记
static bit power_offed= 0;			// 充电器退出标记(它仅有ADC控制)
static bit power_on= 0;			// 充电器接上标记 (表示可进入测试状态，在jugement后置0)
static bit first_boot= 1;			// 充电器接上标记

/*
static unsigned int i = 0;
static unsigned char startOk = 0;
static const unsigned long pwm = ((unsigned int)((STM8_FREQ_MHZ * (unsigned long)1000000)/PWM_FREQUENCY) ); // PWM周期
const unsigned char PWM_MARSK_TABLE[6]={0x01, 0x01, 0x10, 0x10, 0x04, 0x04};
static float PP=0.5,II=0.5,DD=0;
static double SumError=0,PrevError=0,LastError=0;
static int dError=0,Error=0;
*/

/* ---------- 定义2/3(函数) --------------*/

	/* ---------- 函数类1/4(中断) --------------*/
void forDcvAdc() interrupt 16 { 		// ADC 中断, 工作时机： testing = 0 时才启动
	// 转换ADC值, 怎么检测它是由低变高
	// 测试结束，要拔掉电源，才能有以下动作
	// 所以要先检查ADC 有一个低电压，之后再有一个高电压，才认为是有效的power_on

	adc_val = ADCRH; hv <<= 4; hv |= (ADCRL & 0X0F);  // 16h -> dec

	// ~testing 测试前 用于自动开始测试 --- 充电器再次插入
	if(~testing)
		chk_power_on_again();

	// testing 测试用 用于检验OCP后，测试产品的复归
	if(testing){					// 在复归测试中：打开ADCEN，然后delay 若干秒检查PASS。
		if (adc_val > DCV_ON){
			pass =  1;
			clr_ADCEN;				// 	开始测试，停用ADC	
		}else {
			clr_ADCF; set_ADCS;
		}

	}


}

	/* ---------- 函数类2/3(配置) --------------*/
void chk_power_on_again(){
	// 测试结束，要拔掉电源，才能有以下动作
	// 所以要先检查ADC 有一个低电压，之后再有一个高电压，才认为是有效的power_on
	if (first_boot && adc_val > DCV_ON ){ 		//case 1: 开机时有电源 -> 可以马上开始测试
		power_on = 1;
		clr_ADCEN;			// 开始测试，停用ADC	
	}
	else {										//case 2: 开机时无电源 -> 同一般情况 ->先检测有低，再检测有高 
		if (adc_val < DCV_OFF){					//		判断poweroffed的两种情况
			power_offed = 1;
		}
		else{
			power_offed = 0;
		}

		if(power_offed && adc_val > DCV_ON){   // 		由power_offed的决定是否 power_on
			power_on = 1;
			clr_ADCEN;			// 开始测试，停用ADC	
		}

		if(~power_on){							// 以上没有产生有效的power_on, 继续ADC
			clr_ADCF; set_ADCS;
		}
	}
}

void clkConf(){				// 系统时钟配置: 内部16M (default)
	set_HIRCEN; 			// (sysClk = 16M)
}

void adcConf(){
	// EA 启用ADC中断
	set_ADCEN;
	set_ADCS;
}

void ioConf(){
	//  准双向，已经在串口初始化定义
	//串口1  p06 / p07
	//串口2  p16 /p02

	P10_PushPull_Mode; 			// Relay 1 control - for ok -> p10
	P11_PushPull_Mode; 			// Relay 2 control - for ng -> p11
	P12_PushPull_Mode; 			// Relay 3 control - for short -> p12
	P13_PushPull_Mode; 			// Relay 4 control - for charge -> p13

	Enable_ADC_AIN0;			// AIN0 P17

	//P03_PushPull_Mode; 			// Relay 4 control - for protect 
	//P03_PushPull_Mode; 			//  - for start test
	//P05_Input_Mode;				// 定义模拟采样输入
}


	/* ---------- 函数类3/3(功能) --------------*/

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

void test_flow(){
	testing = 1;  			// 标记进入测试中
	reset_judgement();		// 进入测试状诚后，OK/NG继电器复位

 	// 开起定时器等待7秒，定时器中断里执行电流读数
    //直接调用定时器delay函数
    // 用串口读取电流数，写入电流变量 charge_current
    // ocp 复归

    delay1ms();
    SHORT_ON;
    delay1ms();
    // chk the current when OCP
    SHORT_OFF;
    // chk dcv recove after Ocp
    set_ADCEN; set_ADCS;


	// 由以上程序为pass赋的值，决断结果
    delay1ms();
    if (~pass){
    	clr_ADCEN;
    }
	do_a_judgement();
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
	adcConf();

	// 循环测试中
	while (1){
		 if(~testing && power_on){
		 	testing = 1;  // 标记进入测试中
		 	test_flow();
		 }

	}
}
