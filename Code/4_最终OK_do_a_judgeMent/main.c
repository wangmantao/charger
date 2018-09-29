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
#define CHARGE_ON clr_P13    	// 接常闭
#define CHARGE_OFF set_P13

// 定格电流待测时间 (插入几秒后开始测)
#define RATE_DELAY_TIME 7 		// unit:秒
#define RATE_MIN_CURRENT 500 	// unit:mA

// 定格电流维持时间 (电流值检出后，几称后转入短路测试)
#define SHORT_DELAY_TIME 7 		// unit:秒
#define SHORT_MAX_CURRENT 20 	// unit:mA




/* ---------- 定义2/3(变量) --------------*/
static unsigned int DCV_OFF = 1228; 		// 电压输入有没有下限 (4.5v) (4.5/3v)/5v *4095 (因为取样分压1/3)
static unsigned int DCV_ON = 1638; 		// 电压输入有没有上限 (6v) (6/3v)/5v *4095 (因为取样分压1/3)
static unsigned int DCV_BAT = 1092; 		// battery (4v) (4/3v)/5v *4095 (因为取样分压1/3)
static unsigned int adc_val = 0; 		// adc取样值
//static float adc_val2 = 0; 		// adc -> dc 电压输入
static unsigned int charge_current = 0; 		// 充电电流
static unsigned int short_current = 0; 		// 充电电流
static bit pass = 1;			// 测试结果
static bit testing = 0;			// 测试中的标记
static bit power_offed= 0;			// 充电器退出标记(它仅有ADC控制)
static bit power_on= 0;			// 充电器接上标记 (表示可进入测试状态，在jugement后置0)
static bit first_boot= 1;			// 充电器接上标记
static bit get_current_ok= 0;			// 已测试电流标记
static bit time_out= 0;				// 等待中断取数据超时
static bit recovery_ok= 0;			// 复归确认标记
static bit adcv_ok = 0;
static bit this_comp_low= 0;
static bit last_comp_low= 0;
static bit this_comp_hi= 0;
static bit last_comp_hi= 0;
static bit bat_out_ok= 0;

static UINT8 order = 0;
static UINT8 timer0_couter= 0;
static UINT8 current_val[8] ={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
static UINT8 adc_res[3] = {0x00,0x00, 0x00};
static UINT8 cmd[6] = {0xAA,0x55,0x02,0xFE,0x01,0x00}; // 电流读取指令 AA 55 02 FE 01 00 
																		//04 55 02 FE 01 00 
static UINT32 adc_count = 0;
void chk_power_on_again(void);


/* ---------- 定义2/3(函数) --------------*/

	/* ---------- 函数类1/4(中断) --------------*/

void timer0_interrupt() interrupt 1 {
  // 停止ADC/串口工作，pass = 0, 设置 get_current_ok / recovery_ok = 1
	// timer 0 全开最大50mS；需要3秒，要60个循环。
	//TR0 = 0;
	if (timer0_couter> 59) //结束
	{
		timer0_couter = 0;
		time_out= 1;	
		clr_ET0;		//关闭中断，以免影响timer_delay()
	}
	else{
		timer0_couter ++;
 		//set_TR0;
	}
}

void adc_interrupt() interrupt 11 { 		
	adc_val= ADCRH; adc_val<<= 4; adc_val|= (ADCRL & 0X0F);  // 16h -> dec
	//F1: 未在测试状态, 充电器再次插入检测
	if(~testing){					//开始测试前，插入电源检测用此段
		chk_power_on_again();
	}
	else{							// 电池输出检测, 等待操作员按钮，没有超时限制
		if(adc_val>DCV_BAT && P13 == 1 ){			//  放电的下降值，和电池的检测值 ，用同样的4.5V。p13=CHARGE 常闭打开，不充电

			if(adc_count > 50000){					// 1.5秒持续高
				adc_count = 0;
				bat_out_ok = 1;
				clr_ADCEN;				// 	开始测试，停用ADC
			}
			else{
	      		adc_count++;
				clr_ADCF; set_ADCS;		//没有复归，接着ADC检测				
			}
		}
		else {
			clr_ADCF; set_ADCS;		//没有复归，接着ADC检测
		}

	}								
	// ADC的最后状态由if段内部case决定
}

// 获取电流值 UART0 中断
void uart0_interrupt() interrupt 4 {
  if(RI){
  	if (order > 7)
  		order = 0;
    current_val[order] = Receive_Data_From_UART0();;  // 获取电流结果
    //Send_Data_To_UART1( current_val[order]);
    if(order == 7){
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
  }
}

void uart1_interrup(void) interrupt 15 {
  if(TI_1){
    clr_TI_1;
  }
}

	/* ---------- 函数类2/3(配置) --------------*/
void chk_power_on_again(){
	// 测试结束，要拔掉电源，才能有以下动作
	// 所以要先检查ADC 有一个低电压，之后再有一个高电压，才认为是有效的power_on
	if (first_boot && (adc_val > DCV_ON )){ 		//case 1: 开机时有电源 -> 可以马上开始测试
		power_on = 1;
		CHARGE_ON;			// 上电时执行 charge
		//SBUF=0X11;
		clr_ADCEN;			// 开始测试，停用ADC
	}
	else {										//case 2: 开机时无电源 -> 同一般情况 ->先检测有低，再检测有高

		if (~power_offed && adc_val < DCV_OFF ){					//		判断poweroffed的两种情况
			this_comp_low = 1;
			if(this_comp_low != last_comp_low){ 		// 要防错抖动, 与上次不同就要重新计数
				adc_count =0;
			}
			else
				adc_count ++;
			if(adc_count > 50000){ 				// 1.5秒内一直小电压, 保证拔掉充电器时，接触不良有高电压误判而 restJugement
				power_offed = 1;	
				adc_count = 0;
			} 				
		}
		else{
			this_comp_low = 0;
		}
		last_comp_low = this_comp_low;

		// ------------ 以上为确认有效的power_offed, 供下面用-------------------

		if(power_offed && (adc_val > DCV_ON)){   // 		由power_offed的决定是否 power_on
			this_comp_hi = 1;
			if(this_comp_hi != last_comp_hi){ 		// 要防错抖动, 与上次不同就要重新计数
				adc_count =0;
			}
			else
				adc_count ++;
			if(adc_count > 50000){ 				// 1.5秒内一直小电压, 保证拔掉充电器时，接触不良有高电压误判而 restJugement
				power_on = 1;	
				CHARGE_ON;				// 上电时执行charge
				adc_count = 0;
				clr_ADCEN;			// 开始测试，停用ADC	
			} 				
		}
		else{
			this_comp_hi = 0;
		}
		last_comp_hi = this_comp_hi;

		// ------------ 以上为确认有效的power_on, 供下面用-------------------

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
	P03_Quasi_Mode;				// sys_reset (changed: 电池开信号输入)
	P04_Quasi_Mode;				// ng_reset
	set_P03;
	set_P04;


	Enable_ADC_AIN0;			// AIN0 P17

	set_PWMDIV0; 				// pwm 128分频率
	set_PWMDIV1;
	set_PWMDIV2;
}


	/* ---------- 函数类3/3(功能) --------------*/

 void timer0_monitor_start(){
 	// 配置定时期初值
 	// 开启定时器  -- > 定时器溢出时，在中断中处理：停止测试，错误输出
  	time_out=0;
  	order = 0;
    clr_T0M;                                		//T0M=0, Timer0 Clock = Fsys/12
    TMOD |= 0x01;                           		//Timer0 is 16-bit mode
    TL0 = 0x00;
    TH0 = 0x00;
    set_ET0;
    set_TR0;
 }

 void timer0_monitor_stop(){
 	// 关闭定时器
	TR0 = 0;
	clr_ET0; 	
 	// 重置定时器的值
 }

void reset_judgement(){
    SHORT_OFF;     // 最后无论OK、NG都保持短路状态，仅在此还原
	OK_OFF; NG_OFF;	// ok Relay =0 ; ng Relay = 0;
	Timer0_Delay1ms(50);
	CHARGE_ON;		// 最后无论OK、NG的判定进行复位，要进行测试，总是从充电开始
}

void do_a_judgement(){
	//CHARGE_OFF;
	// Timer0_Delay1ms(100);
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
	power_offed = 0;

	// 3. 重新开始ADC侦测(开启ADC中断)
	set_ADCEN;
	set_ADCS;


}

void send_cmd(){
  UINT8 order = 0;
  while (order <= 5){
    Send_Data_To_UART0(cmd[order]);
   order ++;
  }
}

void test_flow(){
  int tempV = 0;
  float current = 0;
  float re_dcv = 0;
  pass = 1;
	testing = 1;  			// 标记进入测试中
	reset_judgement();		// 进入测试状诚后，OK/NG/short继电器复位

  // -----------------------------------------------RATE 充电电流测试
  Timer0_Delay1ms(4000); 	// Rating current 7000->5000->3000
  get_current_ok = 0;
  timer0_monitor_start();
  send_cmd();
	while(~get_current_ok && ~time_out);	// Waiting A meter return
  timer0_monitor_stop();
  if(time_out){
  	pass= 0;
  }
  if(get_current_ok){
  	tempV = current_val[5]; tempV <<= 8; tempV |= (current_val[4] & 0xFF);
  	//current =(float) ( (current_val[5] *  256 + current_val[4])/1000 );
  }
  														// 测得电流NG时
  
/*

256+244 = 500
--------
256+?   = 300 
 ? = 44 = 0x2c
-------
0+ ?   = 200 
? = 200 = 0xc8
-------*/
  if(~pass || tempV < 0x00c8){  // < 0.2A
  //if(~pass || tempV < 0x012c){  // < 0.3A
   //if(~pass || tempV < 0x01F4){  // < 0.5A
  //if(~pass || tempV < 0x044c){  // < 1.1A
  // if(~pass || tempV < 0x03e8){  // < 1.0A
  //if(~pass || tempV < 0x0384){  // < .90A  
  	//发送定格有问题0xF1, 检测NG波形 和short波形
    pass = 0; do_a_judgement(); 
    return;   
  }
  


 
  // -----------------------------------------------SHORT 短路电流测试
														  // 切断充电器
  														  // 放电(电压没有了给出OK信号1秒钟)
														  // 等待电池开机
														  //  短路
  CHARGE_OFF;
  Timer0_Delay1ms(1000); 		 // 断开充电并延迟1.5s

  SHORT_ON;						 // 执行放电与短路
  Timer0_Delay1ms(500); 	     //	延迟500ms
  OK_ON; Timer0_Delay1ms(1000); OK_OFF; // 发出OK指示 1s
  						// waiting .... 大约6s -> 系统复位信号 P03 
  //Timer0_Delay1ms(6000); 
   while (P03);
   Timer0_Delay1ms(2000); 	     //	延迟2s 开始测电流


  /* 不进行电池输出检测
  set_ADCEN; clr_ADCF;
  set_ADCS; set_ADCEN;
  //此时电压值 > 4.8V < 8V,持续1秒钟，表示BAT,OK
  bat_out_ok =0;
  while(~bat_out_ok);   // 等待电池有电压时
  SHORT_ON;             // shorted
  Timer0_Delay1ms(2000); //保留看电流的时间
 */

  get_current_ok = 0;
  timer0_monitor_start();
  send_cmd();
	while(~get_current_ok && ~time_out);	// Waiting A meter return
  timer0_monitor_stop();
  if(time_out){
  	pass= 0;
  }
  if(get_current_ok){
  	tempV = current_val[5]; tempV <<= 8; tempV |= (current_val[4] & 0xFF);
  } 
  //tempV = 0x0003;
  //pass = 1;
  if(~pass || tempV > 0x0014){  // < 0.02A
	Timer0_Delay1ms(1500); //保留看电流的时间 1500ms *2 =3s
    pass = 0; do_a_judgement(); 
    //SHORT_OFF;                  // 无论OK、NG都保持短路状态
    Timer0_Delay1ms(1000);	
    SHORT_OFF;
  	Timer0_Delay1ms(200);	
	CHARGE_ON;				//做OK、NG的判后改变charge ON, 让输入有感知
    return; // ---> return (ng shorted cancel)
  }

   //SHORT_OFF;                  // 无论OK、NG都保持短路状态
    SHORT_OFF; Timer0_Delay1ms(100); CHARGE_ON; // Relay 复位后，再点OK灯
    do_a_judgement();
}

void sys_reset(){
	set_SWRST;	
}

void ng_reset(){
	NG_OFF;
}
/*
流程说明：
上电检查有没有输出
输出有，用relay 接上电池
等待8秒 （常量）
*/
void main(){

	 /*IAP 写保护 */
	set_IAPEN;  //启用IAP
	set_CFUEN;      // 要更新config区域
	IAPAH = 0x00; //cfg0 地址为 0000h       
	IAPAL = 0x00;
	IAPFD = 0xFD; //CFG0 的数据
	IAPCN = 0XE1; //CFG 写
	set_IAPGO; // 开始执行IAP
	clr_IAPEN;


	// 初始化
	ioConf();	
	OK_OFF;
	NG_OFF;
	SHORT_OFF;
	CHARGE_ON;				//开机default charge
   Timer0_Delay1ms(1000);

	set_ES;
	set_REN;
	set_EADC;


	clr_ADCF;
	set_ADCEN;
	set_ADCS;

	set_EA;	


	InitialUART0_Timer1(115200);
	//InitialUART1_Timer3(115200);

	while (1){
		if(power_on){
			testing = 1;  
			clr_ADCEN;
			test_flow();  //开始一轮测试
		}
		Timer0_Delay1ms(500);

		/* 两个复位信号 */
		if(P04 == 0){
			ng_reset();
		}

		/*
		if(P03 == 0) {
			sys_reset();
		}
		*/	
	}
}
