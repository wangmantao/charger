/*													  
log
2018-9-26
	断开充电同时，执行放电6.8欧姆（1A）
	测试short电流时，一秒测3次两流都是低电流才OK
		先保证电流的计算
		可能情况： 开机期间可能电流很大－〉 这是允许的
				过一段时间电流就没有了－〉 1秒测3次都是小流就对了
	充电电流测试10秒的方案改为
		-> 200mA < 电流值 > 20mA 持续2秒，则断开充电再测一次。 可重复6次

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
static unsigned int DCV_BAT = 1092; 		// battery (4v) (4/3v)/5v *4095 (因为取样分压1/3)--开启检验用 // 1092对应1.3V, 当充电电压是5v时（5/3=1.6v)
//static unsigned int DCV_BAT = 10; 		// 控制板性能实验，让它感知较低的电压开始工作
static unsigned int adc_val = 0; 		// adc取样值
//static unsigned int charge_current = 0; 		// 充电电流
//static unsigned int short_current = 0; 		// 充电电流
static bit pass = 1;			// 测试结果
static bit testing = 0;			// 测试中的标记
static bit power_offed= 0;			// 充电器退出标记(它仅有ADC控制)
static bit power_on= 0;			// 充电器接上标记 (表示可进入测试状态，在jugement后置0)
static bit first_boot= 1;			// 充电器接上标记
static bit get_current_ok= 0;			// 已测试电流标记
static bit time_out= 0;				// 等待中断取数据超时
static bit adcv_ok = 0;
static bit this_comp_low= 0;
static bit last_comp_low= 0;
static bit this_comp_hi= 0;
static bit last_comp_hi= 0;
static bit bat_out_ok= 0;
static bit bat_down_ok= 0;
static UINT8 short_ok_count	= 0;
static UINT8 short_count = 0;
static UINT8 time_out_type;

static UINT8 order = 0;
static UINT8 timer0_couter = 0;
static UINT8 current_val[8] ={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
//static UINT8 adc_res[3] = {0x00,0x00, 0x00};
static UINT8 cmd[6] = {0xAA,0x55,0x02,0xFE,0x01,0x00}; // 电流读取指令 AA 55 02 FE 01 00 
																		//04 55 02 FE 01 00 
static UINT32 adc_count = 0;
static UINT8  test_couter = 0;   // 充电电流测试次数

void chk_power_on_again(void);
void varReset(void);
void start(void);
bit short_check(void);
void ng_reset(void);
void sys_reset(void);
/* ---------- 定义2/3(函数) --------------*/
	/* ---------- 函数类1/4(中断) --------------*/

void varReset(){
	pass = 1;			// 测试结果
	testing = 0;			// 测试中的标记
	power_offed= 0;			// 充电器退出标记(它仅有ADC控制)
	power_on= 0;			// 充电器接上标记 (表示可进入测试状态，在jugement后置0)
	first_boot= 1;			// 充电器接上标记
	get_current_ok= 0;			// 已测试电流标记
	time_out= 0;				// 等待中断取数据超时
	adcv_ok = 0;
	this_comp_low= 0;
	last_comp_low= 0;
	this_comp_hi= 0;
	last_comp_hi= 0;
	bat_out_ok= 0;
	bat_down_ok= 0;
	short_ok_count	= 0;
	short_count = 0;

	order = 0;
	timer0_couter = 0;
	//current_val[8] ={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
	//cmd[6] = {0xAA,0x55,0x02,0xFE,0x01,0x00}; // 电流读取指令 AA 55 02 FE 01 00 
	adc_count = 0;
	test_couter = 0;   // 充电电流测试次数
}


/* 3秒超时定时器 */
void timer0_interrupt() interrupt 1 {
	// timer 0 全开最大50mS；需要3秒，要60个循环。
	//TR0 = 0;
	//if (timer0_couter> 59) //结束
	//if (timer0_couter> 120) //结束  ->2.5s
	UINT8 max;
	switch (time_out_type){
		case 0:
			max = 160;    // 160*0.05s=8s for 检验电池电压下降
		break;
		case 1:
			max = 40;    // 40*0.05s=2s for 读取电流值的超时 
		break;
	}

	if (timer0_couter> max) 
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

/*
用途：检查充电器接入，开始测试
*/
void ifBatOut(){
	if(adc_val > DCV_BAT && P13 == 1 ){	       //  放电时电池的检测值>4.5V。不充电,有1A的放电 
		if(adc_count > 50000){				  // 1.5秒持续高
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

/*
用途1：在断开充电后，检测电池是否还是开机状态（下降到4.5V以下）
用途2：在断开充电并短路后，电池电压不得高于1V
*/
void ifBatDown(UINT8 type){
	unsigned int DCV_BAT_DOWN;
	switch (type){
		case 0:
		DCV_BAT_DOWN = 1228;   //4.5v (检电池不应有的开机)
		break;
		case 1:
		DCV_BAT_DOWN = 273;   // 1v  (检电池放电)
		break;
	}

	if(adc_val < DCV_BAT_DOWN && P13 == 1 ){	 //放电状态下, 电池电压<1V		
		//if(adc_count > 50000){					// 1.5秒持续高
		if(adc_count > 16666){					// 0.5秒持续高
			adc_count = 0;
			bat_down_ok = 1;
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

void adc_interrupt() interrupt 11 { 		
	adc_val= ADCRH; adc_val<<= 4; adc_val|= (ADCRL & 0X0F);  // 16h -> dec
	//F1: 未在测试状态, 充电器再次插入检测
	if(~testing){			//做完 do_a_judgement后testing=0		//开始测试前，插入电源检测用此段
		// 测试完成后
		if(P11 == 1 &&  adc_val < DCV_OFF){  // NG自动复位
			ng_reset();		
		}
		chk_power_on_again();
	}
	else{		// 电池输出检测, 或电池被短路
		if(P04 == 0)      
			sys_reset();   // 当short电压不下降 系统复位
		/*
		// 用于只检测电池电压上升
		if(adc_val > DCV_BAT && P13 == 1 ){	   // 放电的下降值，和电池的检测值 ，用同样的4.5V。p13=CHARGE 常闭打开，不充电
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
		*/

		// 用于电池电压上升和下降的检测
		/*
		*/
		/*
		if(P12 == 0) 
			ifBatOut();	        // 未短路时检测电池有输出		
		else
			ifBatDown();		// 短路时检测电池有下降
		*/
		if(P12 == 0 && P13 == 1) //短路之前，检测放电
			ifBatDown(0);		
		if(P12 == 1 && P13 == 1) //短路时，检测电压有无下降
			ifBatDown(1);		
		if(P12 == 0 && P13 == 0)
			ifBatOut();	        // 未短路时检测电池有输出		

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

		if (~power_offed && adc_val < DCV_OFF ){		//		判断poweroffed的两种情况
			this_comp_low = 1;
			if(this_comp_low != last_comp_low){ 		// 要防错抖动, 与上次不同就要重新计数
				adc_count =0;
			}
			else
				adc_count ++;
			//if(adc_count > 50000){ 				// 1.5秒内一直小电压, 保证拔掉充电器时，接触不良有高电压误判而 restJugement
			if(adc_count > 33333){ 				// 1秒内一直小电压, 保证拔掉充电器时，接触不良有高电压误判而 restJugement
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
			//if(adc_count > 50000){ 				// 1.5秒内一直小电压, 保证拔掉充电器时，接触不良有高电压误判而 restJugement
			if(adc_count > 33333){ 				// 1秒内一直小电压, 保证拔掉充电器时，接触不良有高电压误判而 restJugement
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

/*
充电电流NG, 做了判定后＝整个测试过程已经完毕
当短路NG,
*/
void do_a_judgement(){
	// 1. 输出测试结果(先置0再置1, 防止两个同时有效）
	if(pass){
		NG_OFF; 
		OK_ON;
		Timer0_Delay1ms(1000); OK_OFF;  //1s OK信号复0 2019-11-13

	}
	else {                // NG状况
		OK_OFF; 
		NG_ON;Timer0_Delay1ms(1000); ng_reset();  //1s NG信号

		if(P13 == 0){    // 充电不良再发NG信号, 在11号中断ng_reset
			while (P03 == 0);   
			Timer0_Delay1ms(200);
			NG_ON;
		}
	}
	// 2. 设测试结束标记 & 待开始新一轮充电插入检测
	first_boot = 0;
	testing = 0;
	power_on = 0; 
	power_offed = 0;

	// 3. 重新开始ADC侦测(开启ADC中断)
	set_ADCEN;
	set_ADCS;
	//把OK信号点起来，表示完成这个测试（其实是为了让大系统放行）
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
	UINT8 tc;
	UINT8 test_max;
	pass = 1;
	testing = 1;  			// 标记进入测试中
	reset_judgement();		// 进入测试状诚后，OK/NG/short继电器复位

	// -----------------------------------------------RATE 充电电流测试
	// 0x00c8 <0.2A  0x012c< 0.3A 0x01F4< 0.5A 0x044c< 1.1A
	// 公式 256 * 高8位 + 244（低8位所转） = 500 mA (0x01F4)

	Timer0_Delay1ms(1000); 	// 初次等待1s

	/*
	最多取20次*0.35s/次数据，要同时制在多长时间内完成
	*/
	test_couter = 0;
	test_max = 20;
	while( test_couter < test_max ){	 
	  get_current_ok = 0;
	  /*
	  time_out_type = 1;
	  timer0_monitor_start();
	  send_cmd();
	  while(~get_current_ok && ~time_out){
	  		if(P04 == 0){
				sys_reset();   // 系统复位
		}
	  };	// 即没得到电流，又没超时一直等
	  timer0_monitor_stop();
	  */
	  send_cmd();			// 求一次数据

	  // 等一次结果, 1秒等不到，退出等待
	  tc = 0;
	  while(~get_current_ok){   
	  	Timer0_Delay1ms (10); // 1s = 0.01 * 100
	  	tc ++;
	  	if(tc >= 100)  
	  		break;	
	  }  	
	  test_couter ++;			 
	  if(tc >= 100)               //取数据失败，再取
	  	continue;
	  							 //取数据成功，值OK退出，值NG停0.35s再取
	  tempV = current_val[5]; tempV <<= 8; tempV |= (current_val[4] & 0xFF);
 	  if (tempV > 0x00c8) 
	   	break;	
	  else 
	  	Timer0_Delay1ms(350); 
	  
	  /*
	  if(get_current_ok) {					// 取得电流OK,处理电流
	  	tempV = current_val[5]; tempV <<= 8; tempV |= (current_val[4] & 0xFF);
	  	if (tempV > 0x00c8)          		// 获得了理想的电流，退出电流取值 
	    	break;	
	  }    				
	  Timer0_Delay1ms(350);   				// 下一个重复的间隔（电流表的采样率为每秒3次)
	  test_couter ++;						// 取电流成功才算累加
	  */
	}
	
	/* 故障见国 9－27 16：01
	if(~pass || tempV < 0x00c8){  
		CHARGE_OFF;	
		Timer0_Delay1ms (1000);
		CHARGE_ON;
		tempV = chargeCurrent(tempV); //第2次测试充电 
	}	
  	*/ 

	// 次数上限 或 电流不足
	if(test_couter >= test_max || tempV < 0x00c8){  // 0.2A
  		pass = 0; 				
    	do_a_judgement(); 	// NG 判定输出
    	return;   
  	}
	// -----------------------------------------------SHORT前，检测电池已关机
	CHARGE_OFF;
	bat_down_ok = 0;    
	set_ADCEN; clr_ADCF; set_ADCS; set_ADCEN;
	time_out_type = 0;
	timer0_monitor_start();
	while(~bat_down_ok && ~time_out);   //此时电压值 > 4.5V,持续.3秒钟，表示短路有效
	timer0_monitor_stop();
	if(time_out){						//超时重测
		varReset();
		start();
	}
	// ----------------------------------------------- 先短路上负载，请求开电池
	SHORT_ON;	
	OK_ON; Timer0_Delay1ms(1000); OK_OFF; 
	// ----------------------------------------------- 等待机械系统那边操作
	while (P03);
	// -----------------------------------------------SHORT 短路电流测试 连续3次电流OK，定OK
	pass = short_check();
	if(~pass){  
		SHORT_OFF; 
    	Timer0_Delay1ms(200); 					
    	CHARGE_ON;								// 为了能再次检测到上电 
    	do_a_judgement(); 						// NG 判定输出
    	return;   
  	}
	// ----------------------------------------------- OK 输出
	SHORT_OFF; Timer0_Delay1ms(100); CHARGE_ON; // Relay 复位后，再点OK灯
	do_a_judgement();
}


bit short_check(){
	float t2;
	int tempV2 = 0;
	UINT8 tc;
	bit result;
    short_ok_count	= 0;
    short_count	= 0;
	while (short_ok_count < 3 && short_count < 28){ // 要测3次OK, 总测次数不得>28次(10秒) (350mS/次) 
		get_current_ok = 0;
		send_cmd();

		// 等1秒电流值，超时不等
		tc = 0;
		while(~get_current_ok){   
		  	Timer0_Delay1ms (10); // 1s = 0.01 * 100
		  	tc ++;
		  	if(tc >= 100)  
		  		break;	
		}  	
		short_count ++;			 
		if(tc >= 100)               //取数据失败，再取
		  	continue;
		  							//取得了数据
		tempV2 = current_val[5]; tempV2 <<= 8; tempV2 |= (current_val[4] & 0xFF);
		t2 = (float) tempV2;
	  	if( t2 > -20)				// 小于20mA OK数+1 否则OK数重置0
			short_ok_count	++;
	  	else 
			short_ok_count = 0;     

		Timer0_Delay1ms(350);   	//0.35s后再测（电流表的采样率为每秒3次)
	}

	if(short_ok_count < 3)						// 没有连续3次OK
		result = 0;								//pass =0;
	else
		result = 1;							    //pass =1
	return result;
}
/*
bit short_check(){
	float t2; 									//= (float) tempV2;
	int tempV2 = 0;
	bit result;
    short_ok_count	= 0;
    short_count	= 0;
	while (short_ok_count < 3 && short_count < 28){ // 要测3次OK, 总测次数不得>28次(10秒) (350mS/次) 
		get_current_ok = 0;
		time_out_type = 1;
		timer0_monitor_start();
		send_cmd();
		while(~get_current_ok && ~time_out);	// Waiting A meter return
		timer0_monitor_stop();
		if(time_out)							// 取电流测试，接着取
			continue;								
		short_count	++;  						// 总测试数累加, 取不到电流会重取不算在内
		if(get_current_ok){						// 处理取得的数据
			tempV2 = current_val[5]; tempV2 <<= 8; tempV2 |= (current_val[4] & 0xFF);
			t2 = (float) tempV2;
	  		if( t2 > -20)						// 电流值ABS小于20mA OK
				short_ok_count	++;
	  		else 
				short_ok_count = 0;             // 电流值不OK，重新作OK计数
		}
		Timer0_Delay1ms(350);   				// 下一个重复的间隔（电流表的采样率为每秒3次)
	}
	if(short_ok_count < 3)						// 没有连续3次OK
		result = 0;								//pass =0;
	else
		result = 1;							    //pass =1
	return result;
}
*/
void sys_reset(){
	set_SWRST;	
}

void ng_reset(){
	NG_OFF;
	/*
	if(P12 == 1) {
		P12 = 0;
		Timer0_Delay1ms(50);
		P13 = 0;  
	}
	*/
}

/*IAP 写保护 */
void protect(){
	set_IAPEN;  //启用IAP
	set_CFUEN;      // 要更新config区域
	IAPAH = 0x00; //cfg0 地址为 0000h       
	IAPAL = 0x00;
	IAPFD = 0xFD; //CFG0 的数据
	IAPCN = 0XE1; //CFG 写
	set_IAPGO; // 开始执行IAP
	clr_IAPEN;
}

void start(){
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
		if(power_on && P11 == 0){  // NG信号关掉后，才能进入测试流程
			testing = 1;  
			clr_ADCEN;
			test_flow();  //开始一轮测试
		}
		Timer0_Delay1ms(500);

		/* 两个复位信号 */
		if(P04 == 0){
			sys_reset();   // 系统复位
			/*
			if(P11 ==1 )    // 当NG生效时
				ng_reset();    // NG复位
			else
				sys_reset();   // 系统复位
			*/
		}

		/*
		if(P03 == 0) {
			sys_reset();
		}
		*/
	}
}

void main(){
	protect();
	start();
}
