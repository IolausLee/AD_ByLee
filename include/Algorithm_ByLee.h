#ifndef ALGORITHM_BYLEE_H_
#define ALGORITHM_BYLEE_H_

//#pragma CODE_SECTION(pi_calc,"ramfuncs");
//#pragma CODE_SECTION(clarke_calc,"ramfuncs");
//#pragma CODE_SECTION(anticlarke_calc,"ramfuncs"); 
//#pragma CODE_SECTION(park_calc,"ramfuncs"); 
//#pragma CODE_SECTION(antipark_calc,"ramfuncs");
//#pragma CODE_SECTION(line_to_phase,"ramfuncs");
//#pragma CODE_SECTION(pll_calc,"ramfuncs");


typedef struct PI_CTRL_TYPEDEF
{  
	//float32  Kp;			// Parameter: Proportional gain  比例系数
	//float32  Ki;			// Parameter: Integral gain  积分强度
	//float32  Ref;   		// Input: Reference input 给定值
	//float32  Fdb;   		// Input: Feedback input 反馈值
	float32  Errp;			// Variable: Error   误差
	float32  Up;			// Variable: Proportional output  比例输出 
	float32  Ui;			// Variable: Integral output  积分输出
	float32  OutPreSat;		// Variable: Pre-saturated output 预输出
	//float32  OutMax;		// Parameter: Maximum output 输出最大值
	//float32  OutMin;		// Parameter: Minimum output 输出最小值
	float32  Out;   		// Output: PID output PID输出
} PI_Ctrl;	 
	 




typedef struct CLARKE_TYPEDEF 
{
	//float32 As;  // Input: phase-a stator variable  
	//float32 Bs;  // Input: phase-b stator variable  
	float32 Alpha;  // Output: stationary d-axis stator variable 
	float32 Beta;  // Output: stationary q-axis stator variable 
} CLARKE;

//void int clarke_calc(CLARKE *c,float As,float Bs,float Cs)
void clarke_calc(CLARKE *c,float As,float Bs)
{ 
	c->Alpha = As;//三相平衡下可用
	c->Beta = (As + 2*Bs)*0.57735026918963;  //1/sqrt(3) = 0.57735026918963 

	//c->Alpha=0.66666*As-0.33333*(Bs+Cs);
	//c->Beta=0.57735*(Bs-Cs);

	//return 0; 
} 


typedef struct ANTICLARKE_TYPEDEF
{
	//float32  Alpha;  //input 
	//float32  Beta;	//input
	float32  As;  //output
	float32  Bs;  //output
	float32  Cs;  //output
} ANTICLARKE;	 

void anticlarke_calc(ANTICLARKE *ac,float Alpha,float Beta)
{
	ac->As = Alpha;
	ac->Bs = 0.866 * Beta - 0.5 * Alpha;  //sqrt(3)/2=0.8660234038
	ac->Cs = -0.866 * Beta - 0.5 * Alpha;

} 


typedef struct PARK_TYPEDEF
{
	//float32 Alpha;  // Input: stationary d-axis stator variable 
	//float32 Beta;  // Input: stationary q-axis stator variable 
	//float32 cosa;  // Input: 
	//float32 sina;  //input;
	float32 Ds;  // Output: rotating d-axis stator variable 
	float32 Qs;  // Output: rotating q-axis stator variable 
} PARK;

void park_calc(PARK *p,float Alpha,float Beta,float sina,float cosa)
{  
	p->Ds = Alpha * cosa + Beta * sina;
	p->Qs = Beta * cosa - Alpha * sina;

	//return 0;
} 

typedef struct ANTIPARK_TYPEDEF
{
	float32  Alpha;  	// 
	float32  Beta;	//  
	//float32  cosa;	// Input: 
	//float32  sina;    //Input
	//float32  Ds;		// Input: rotating d-axis stator variable 
	//float32  Qs;		// Input: rotating q-axis stator variable  
} ANTIPARK;	
				
void antipark_calc(ANTIPARK *ap,float Ds,float Qs,float sina,float cosa)
{  
    ap->Alpha = Ds * cosa - Qs * sina;
    ap->Beta = Qs * cosa + Ds * sina;  

} 

typedef struct LINE2PHASE
{
	/* data */
	//float32 Uab;
	//float32 Ubc;

	float32 Ua;
	float32 Ub;
	float32 Uc;

} line2phase;

void line_to_phase(line2phase *l,float Uab,float Ubc)
{
	l->Ua = 0.666 * Uab + 0.333 * Ubc;
	l->Ub = 0.333 * Ubc - 0.333 * Uab;
	l->Uc = -l->Ua - l->Ub;

}


typedef struct INVERTER_PLL
{
/*	float32 w;
	float32 delta_t;
	float32 phi;*/

	 float32 sina;
	 float32 sinb;
	 float32 sinc;

//	_iq sina;
//	_iq sinb;
//	_iq sinc;
	
} inverter_pll;

void inverter_pll_calc(void);


typedef struct PLL_TYPEDEF
{
	/* data */
	//float Alpha;
	//float Beta;

	float sin;
	float cos;
} PLL;

int pll_calc(PLL *p,float Alpha,float Beta)
{
	float s;
	s=sqrt(Alpha * Alpha + Beta * Beta);

	p->sin=Beta/s;
	p->cos=Alpha/s;

	return 0;
}



#endif /*ALGORITHM_BYLEE_H_*/
