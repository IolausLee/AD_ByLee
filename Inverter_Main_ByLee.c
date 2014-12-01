

#include "DSP281x_Device.h"	// DSP281x Headerfile Include File
#include "DSP281x_Examples.h"	// DSP281x Examples Include File

#include "Example_DPS2812M_AD.H"
//#include "Example_DPS2812M_DA.H"

#include "IQmathLib.h"
#include "math.h"
#include "Algorithm_ByLee.h"




#define pi 3.1415926  //圆周率

//传感器参数
#define Ku 151.11		//LV25_P电压传感器放大倍数。当限流电阻为110kΩ，测量电阻270Ω
#define Kcur 6.67		//用LA55-P/SP50电流传感器，测量电阻为150Ω
#define Res 0.000305	//AD芯片的分辨率
#define U_max 20        //继保动作电压


//采样频率和延迟点数
#define fk 6.0               //采样频率,单位为kHz
//#define V_Dly 90.0              //延迟90°
#define V_T1PR 37.5*1000.0/2.0/fk   //连续增/减模式37.5*1000/2.0/fk；连续增模式37.5*1000/fk-1
//#define Dot fk*20.0          //一个周期的点数,0.02/Ts
//#define Dly V_Dly*fk*20.0/360.0    //需要延迟的点数
#define V_ACTRA 0x0666    //低有效0x0999.高有效则为0x0666
#define V_T1CON 0x0942    //定时器T1时钟脉冲：37.5MHz,连续增/减模式0942;连续增模式1142
//0x0942=0000 1001 0100 0010
//
#define V_DBTCONA 0x0AF4  //设置死区；
//0x0AF4=(1010)_(111)(1_01)(00),
//B1010=10,死区定时器周期；B111，死区定时器123使能;B101,预定标因子5；死区时间us=周期10*2^5/75MHz= 4.27us
#define Tk 1.0/fk/1000

//定向
#define f 50  //逆变器输出频率
#define T 1.0/50//周期
#define phi 0  //逆变器输出相位角
#define w 2*pi*f 
//#define w_IQ _IQmpy( _IQ(2) , _IQmpy( _IQ(pi) , _IQ(f)))

//#define count (/Tk)

//PI

#define PI_Kp 4
#define PI_Ki 150
#define PI_OutMax 300
#define PI_OutMin -300

#define Ud_Ref 5
#define Uq_Ref 0

//Uint16 EVAInterruptCount;
Uint16 x=0;//temp[128],
unsigned int ADflag=1;
unsigned int AD_corrention_flag=1;//AD校正标志位
//unsigned int channal=5;


/****测试用****/
//float Temp;
//float a[200],b[200];//,cc[200];
float temp1[128];//,temp2[128];
/****End****/

unsigned int cnt=0;//
//unsigned int xint=0,tint=0;
float sample_time=Tk;

float U1,U2;
float U1_offset=0,U2_offset=0;
float U1_offset_temp=0,U2_offset_temp=0;
float Ua_pwm,Ub_pwm,Uc_pwm;

interrupt void ADC_T1TOADC_isr(void);
interrupt void ADC_SampleINT(void);	

void inverter_pll_calc(void);
void pi_calc(PI_Ctrl *p,float Ref,float Feedback);

// ADC start parameters
//#define ADC_MODCLK 0x3	// HSPCLK = SYSCLKOUT/2*ADC_MODCLK2 = 150/(2*3) = 25MHz
// Global variable for this example
//DAC_DRV DAC=DAC_DRV_DEFAULTS;
ADC_DRV AD=ADC_DRV_DEFAULTS;
inverter_pll ip;
PLL pll;
line2phase l2p;
CLARKE c,c2;
PARK p;
ANTICLARKE ac;
ANTIPARK ap;

PI_Ctrl PI_d={
				//4.0,			// Parameter: Proportional gain  比例系数
				//150.0,			// Parameter: Integral gain  积分强度
				//Ud_Ref,   		// Input: Reference input 给定值
				//0.0,   		// Input: Feedback input 反馈值
				0.0,			// Variable: Error   误差
				0.0,			// Variable: Proportional output  比例输出 
				0.0,			// Variable: Integral output  积分输出
				0.0,		// Variable: Pre-saturated output 预输出
				//PI_OutMax,		// Parameter: Maximum output 输出最大值
				//PI_OutMin,		// Parameter: Minimum output 输出最小值
				0.0   		// Output: PID output PID输出
				};
PI_Ctrl PI_q={
				//4.0,			// Parameter: Proportional gain  
				//150.0,			// Parameter: Integral gain  
				//Uq_Ref,   		// Input: Reference input 
				//0.0,   		// Input: Feedback input 
				0.0,			// Variable: Error   
				0.0,			// Variable: Proportional output  
				0.0,			// Variable: Integral output  
				0.0,		// Variable: Pre-saturated output 
				//PI_OutMax,		// Parameter: Maximum output 
				//PI_OutMin,		// Parameter: Minimum output 
				0.0   		// Output: PID output 
				};
				
PI_Ctrl PI_default={
				//4.0,			// Parameter: Proportional gain  
				//150.0,			// Parameter: Integral gain  
				//Uq_Ref,   		// Input: Reference input 
				//0.0,   		// Input: Feedback input 
				0.0,			// Variable: Error   
				0.0,			// Variable: Proportional output  
				0.0,			// Variable: Integral output  
				0.0,		// Variable: Pre-saturated output 
				//PI_OutMax,		// Parameter: Maximum output 
				//PI_OutMin,		// Parameter: Minimum output 
				0.0   		// Output: PID output 
				};




void main(void)
{
// Step 1. Initialize System Control:
// PLL, WatchDog, enable Peripheral Clocks
// This example function is found in the DSP281x_SysCtrl.c file.
	InitSysCtrl();
	//MemCopy(&RamfuncsLoadStart, &RamfuncsLoadEnd, &RamfuncsRunStart);
	
// Step 2. Initialize GPIO: 
// This example function is found in the DSP281x_Gpio.c file and
// illustrates how to set the GPIO to it's default state.
// InitGpio();	

// Step 3. Clear all interrupts and initialize PIE vector table:
// Disable CPU interrupts 
	DINT;
	IER=0x0000;
	IFR=0x0000;

// Initialize the PIE control registers to their default state.
// The default state is all PIE interrupts disabled and flags
// are cleared.  
// This function is found in the DSP281x_PieCtrl.c file.
	InitPieCtrl();

// Disable CPU interrupts and clear all CPU interrupt flags:
	//IER = 0x0000;
	//IFR = 0x0000;

// Initialize the PIE vector table with pointers to the shell Interrupt 
// Service Routines (ISR).  
// This will populate the entire table, even if the interrupt
// is not used in this example.  This is useful for debug purposes.
// The shell ISR routines are found in DSP281x_DefaultIsr.c.
// This function is found in DSP281x_PieVect.c.
	InitPieVectTable();
	Init_Gpio();
	//MemCopy(&RamfuncsLoadStart, &RamfuncsLoadEnd, &RamfuncsRunStart);
	
// 初始化EVA定时器1	
	ADREG=0;
	EvaRegs.GPTCONA.all=0;
	
//设置通用目的定时器1的周期;
	EvaRegs.T1PR=V_T1PR;     //(0x0823,9kHz)(0x1D4B,5kHz)
	//EvaRegs.T1CMPR=0x0000;		
	
//使能通用目的定时器1的周期中断
//向上计数、预定标x128、内部时钟、使能比较、使用自己的周期值
	EvaRegs.EVAIMRA.bit.T1PINT=1;	
	EvaRegs.EVAIFRA.bit.T1PINT=1; 
	
//清除通用目的定时器1的计数器值
	EvaRegs.T1CNT=0x0000;

/**************T1CON设置*******************/
	EvaRegs.T1CON.all=0x0942;  //0x0942,
//	EvaRegs.T1CON.bit.TMODE=1;
//	EvaRegs.T1CON.bit.TPS=1;
//	EvaRegs.T1CON.bit.TENABLE=0;
//	EvaRegs.T1CON.bit.TCLKS10=0;
//	EvaRegs.T1CON.bit.TECMPR=1;
/**********************************/
	
	EvaRegs.ACTRA.all = V_ACTRA;    //低有效0x0999.高有效则为0x0666
	EvaRegs.DBTCONA.all = V_DBTCONA;   //设置死区
	EvaRegs.COMCONA.all = 0xA600;     //0xA600,时间4.27us

	
//当通用目的定时器1产生中断时启动ADC变换
	EvaRegs.GPTCONA.bit.T1TOADC=2;//0x10
		
// Interrupts that are used in this example are re-mapped to
// ISR functions found within this file.  
	EALLOW;  // This is needed to write to EALLOW protected registers
	PieVectTable.T1PINT = &ADC_T1TOADC_isr;
	PieVectTable.XINT1  = &ADC_SampleINT;  //外部中断
	EDIS;	// This is needed to disable write to EALLOW protected registers
	
	XIntruptRegs.XINT1CR.all=0x0001;
	XIntruptRegs.XINT2CR.all=0x0001;
	PieCtrlRegs.PIEIER1.bit.INTx3=1;
	PieCtrlRegs.PIEIER1.bit.INTx4=1;
	PieCtrlRegs.PIEIER2.all=M_INT4;
// Step 4. Initialize all the Device Peripherals:
// This function is found in DSP281x_InitPeripherals.c
// InitPeripherals(); 
//	DAC.DACChannelSel=0x00;
//	DAC.DACDataCycle=128;
//	DAC.DACDataOffset=0;
//	DAC.DataSel=0;
// Step 5. User specific code, enable interrupts:
	// Step 5. User specific code, enable interrupts:
	
	IER |=(M_INT2|0x0001);
	
// Enable global Interrupts and higher priority real-time debug events:
	EINT;	// Enable Global interrupt INTM
	ERTM;	// Enable Global realtime interrupt DBGM

	for(;;)
	{
		if(AD.ADCFlag.bit.ADCSampleFlag==1) {
			//if(AD.LoopVar>=128) 
				//AD.LoopVar=0;
			//AD.ADChannelSel=0;
			if(ADflag==0) {

				//GpioDataRegs.GPBCLEAR.bit.GPIOB8=1;
				//GpioDataRegs.GPBSET.bit.GPIOB9=1;

				//loopcounter++;

				ADCSmplePro(&AD);
				//AD.LoopVar++;
				
				
				/**采样完成后开始数据处理**/
				inverter_pll_calc();//计算相位
				
				
//int clarke_calc(CLARKE *c,float As,float Bs,float Cs)		
				clarke_calc(&c,l2p.Ua,l2p.Ub,l2p.Uc);//

//int park_calc(PARK *p,float Alpha,float Beta,float sina,float cosa)
				park_calc(&p,c.Alpha,c.Beta,pll.sin,pll.cos);//
				
//void pi_calc(PI_Ctrl *p,float Ref,float Feedback) 
				pi_calc(&PI_d,Ud_Ref,p.Ds);//
				pi_calc(&PI_q,Uq_Ref,p.Qs);//

//int antipark_calc(ANTIPARK *ap,float Ds,float Qs,float sina,float cosa)
				antipark_calc(&ap,PI_d.Out,PI_q.Out,pll.sin,pll.cos);//

//int anticlarke_calc(ANTICLARKE *ac,float Alpha,float Beta)
				anticlarke_calc(&ac,ap.Alpha,ap.Beta);//
				

				/******归一化*********/
				Ua_pwm=0.5+ac.As/1000;
				Ub_pwm=0.5+ac.Bs/1000;
				Uc_pwm=0.5+ac.Cs/1000;
				/********************/

				//a[cnt]=ac.As;
				

				/**设定占空比（比较中断）**/
				//EvaRegs.CMPR1=Ua_pwm*EvaRegs.T1PR;
				//EvaRegs.CMPR2=Ub_pwm*EvaRegs.T1PR;
				//EvaRegs.CMPR3=Uc_pwm*EvaRegs.T1PR;
				
				
				/**开环控制**/
				EvaRegs.CMPR1=0.5*EvaRegs.T1PR;
				EvaRegs.CMPR2=0.5*EvaRegs.T1PR;
				EvaRegs.CMPR3=0.5*EvaRegs.T1PR;
				
				//EvaRegs.CMPR1=ip.sina*EvaRegs.T1PR;
				//EvaRegs.CMPR2=ip.sinb*EvaRegs.T1PR;
				//EvaRegs.CMPR3=ip.sinc*EvaRegs.T1PR;
			}
			AD.ADCFlag.bit.ADCSampleFlag=0;
		}
	}
}

interrupt void ADC_T1TOADC_isr(void)  
{
	//tint++;
	if(ADflag==0) {
		*AD_CONVST=0;
		EvaRegs.EVAIMRA.bit.T1PINT=1;
		EvaRegs.EVAIFRA .all =BIT7;
	}//当数据处理时间大于采样间隔时间，则会数据未处理完成时发生中断，继续进行标志位置位以便之后继续中断
    if(ADflag==1) {
		ADflag=0;
		
		*AD_CONVST=0;
		EvaRegs.EVAIMRA.bit.T1PINT=1;  //
		EvaRegs.EVAIFRA .all =BIT7;    //
		//EvaRegs.EVAIFRA.bit.T1PINT=1;//效果与EvaRegs.EVAIFRA .all =BIT7语句相同
		//PieCtrlRegs.PIEACK .all =PIEACK_GROUP2;
	} 

	//EvaRegs.EVAIMRA.bit.T1PINT=1;  //
	//EvaRegs.EVAIFRA .all =BIT7;    //
	
	PieCtrlRegs.PIEACK .all =PIEACK_GROUP2;  //
	//return;
}

interrupt void ADC_SampleINT(void)
{
	//xint++;
	XIntruptRegs.XINT1CR .all =0x0000;
	if(AD.ADCFlag.bit.ADCSampleFlag==0) {
		//while(1);
		AD.ADCFlag.bit.ADCSampleFlag=1;
	}
	
	//XIntruptRegs.XINT1CR .all =0x0000;
	//AD.ADCFlag.bit.ADCSampleFlag=1;
	XIntruptRegs.XINT1CR .all =0x0001;
	PieCtrlRegs.PIEACK .all =PIEACK_GROUP1;	 //
	//return;
}

void ADCSmplePro(ADC_DRV *v)
{
	//unsigned int Temp1;
	if(v->ADChannelSel==6) 
		v->ADChannelSel=0;
	if(v->ADCFlag.bit.ADCCS0==1) {
		v->ADSampleResult0[x]=*AD_CHIPSEL0;
		v->ADSampleResult1[x]=*AD_CHIPSEL0;
		v->ADSampleResult2[x]=*AD_CHIPSEL0;
		v->ADSampleResult3[x]=*AD_CHIPSEL0;
		v->ADSampleResult4[x]=*AD_CHIPSEL0;
		v->ADSampleResult5[x]=*AD_CHIPSEL0;
	}
	if(v->ADCFlag.bit.ADCCS1==1) {
		v->ADSampleResult0[x]=*AD_CHIPSEL1;
		v->ADSampleResult1[x]=*AD_CHIPSEL1;
		v->ADSampleResult2[x]=*AD_CHIPSEL1;
		v->ADSampleResult3[x]=*AD_CHIPSEL1;
		v->ADSampleResult4[x]=*AD_CHIPSEL1;
		v->ADSampleResult5[x]=*AD_CHIPSEL1;
	}	


	U1=(signed int)v->ADSampleResult0[x]*Ku*Res-U1_offset;	
	U2=(signed int)v->ADSampleResult1[x]*Ku*Res-U2_offset;

	if(AD_corrention_flag==1) {
		
		U1_offset=U1+U1_offset;//计算当前实际偏移值
		U1_offset_temp+=U1_offset;//累计偏移值
		U1_offset=U1_offset_temp/(x+1);//计算下次估计偏移值

		//if(U1_offset>20) while(1);

		U2_offset=U2+U2_offset;//计算当前实际偏移值
		U2_offset_temp+=U2_offset;//累计偏移值
		U2_offset=U2_offset_temp/(x+1);//估算下次估计偏移值

		//if(U2_offset>20) while(1);

		if(x==50) {//计算50次后停止校正
			AD_corrention_flag=0;
			//PI_d=PI_default;
			//PI_q=PI_default;
		}
	}

/******************Fo*******************/
//	if(U1>U_max) {
//		GpioDataRegs.GPFCLEAR.bit.GPIOF4=1;
//		GpioDataRegs.GPBSET.bit.GPIOB8=1;
//		for(;;);
//	} 
//
//	if(U2>U_max) {
//		GpioDataRegs.GPFCLEAR.bit.GPIOF4=1;
//		GpioDataRegs.GPBSET.bit.GPIOB8=1;
//		for(;;);
//	}


/***int line_to_phase(line2phase *l,float Uab,float Ubc)***********/
	line_to_phase(&l2p,U1,U2);//


/******Graphic*******/
	//Temp=U1;
	temp1[x]=U1;
	//temp2[x]=l2p.Ubc;
/******Graphic End*******/

	ADflag=1;
	x++;
	if(x>=128)
		x=0;
	
}

void inverter_pll_calc(void)
{
	//_iq sin_out;
	_iq angle_IQ,delta_phi_IQ;
	_iq sina_IQ,sinb_IQ,sinc_IQ;
	
	
	if(cnt > (fk*1000/f)) //
		cnt = 0;
	else 
		cnt++;
	
	//ip.sina=(float)sin(w*Tk*cnt+phi);
	//ip.sinb=(float)sin(w*Tk*cnt+phi-0.66*pi);
	//ip.sinc=(float)sin(w*Tk*cnt+phi+0.66*pi);

	angle_IQ=_IQ(cnt * Tk * w);
	delta_phi_IQ=_IQ(0.66666 * pi);//移相120度
	
	sina_IQ=_IQsin(angle_IQ);
	sinb_IQ=_IQsin(angle_IQ - delta_phi_IQ);
	sinc_IQ=_IQsin(angle_IQ + delta_phi_IQ);
	
	ip.sina=_IQ24toF( sina_IQ );
	ip.sinb=_IQ24toF( sinb_IQ );
	ip.sinc=_IQ24toF( sinc_IQ );

	/* Graphic */
	//a[cnt]=ip.sinb;
	//b[cnt]=_IQ24toF(sin_out);
	/* Graphic end */

	if(ip.sina > 0.9999)
		ip.sina=0.9999;
	if(ip.sinb > 0.9999)
		ip.sinb=0.9999;
	if(ip.sinc > 0.9999)
		ip.sinc=0.9999;
//int clarke_calc(CLARKE *c,float As,float Bs,float Cs)
	clarke_calc(&c2,ip.sina,ip.sinb,ip.sinc);
//int pll_calc(pll *p,float Alpha,float Beta)
	pll_calc(&pll,c2.Alpha,c2.Beta);

	if(pll.sin > 0.9999)
		pll.sin=0.9999;
	if(pll.cos > 0.9999)
		pll.cos=0.9999;

	/* Graphic */
	//a[cnt]=pll.sin;
	//cc[cnt]=pll.cos;
	/* Graphic end */

}

void pi_calc(PI_Ctrl *p,float Ref,float Feedback) 
{	 
 // Compute the error
    p->Errp = Ref - Feedback;
    
    // Compute the proportional output
    p->Up = PI_Kp * p->Errp;

    // Compute the integral output
    p->Ui = (p->OutPreSat == p->Out)?(p->Ui + PI_Ki*p->Errp*sample_time):p->Ui;

    // Compute the pre-saturated output
    p->OutPreSat = p->Up + p->Ui ;     
    

    if (p->OutPreSat > PI_OutMax)                   
	{
	 p->Out =  PI_OutMax;
	}
    else if (p->OutPreSat < PI_OutMin)
	{ 
	 p->Out =  PI_OutMin; 
	}
	else
	 p->Out = p->OutPreSat;   	  
}



//===========================================================================
// No more.
//===========================================================================

