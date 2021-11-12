/*
** PID_form.h **

** This source & header are PID controller **


p.s. pray win in the RoboCup 2020..
  
*/

#ifndef PID_FORM_H_
#define PID_FORM_H_

#include "main.h"



//** Part of PID Define **//
#define ERROR_ARRAY_NUM 10
#define BLDC_PPR 409600 
#define SAMPLING_TIMER6 0.005              // 5ms = 0.005s
#define SAMPLING_TIMER7 0.025              // 25ms = 0.025s
#define SAMPLING_TIMER8 0.001              // 1ms = 0.025s
#define TIMER6 0                                // 5ms = 0.005s
#define TIMER7 1                                //25ms = 0.025

#define PPR 8192


/* Part of PID Struct */

//** PID control parameter **//
typedef struct _PID{
        float nowValue;
	float pastValue;
	
	float nowError;
	float pastError;
	float target;
	
        int nowError_int;
        
	float errorSum;
	float errorSumLimit;
	float errorDiff;

	float nowOutput;
	float pastOutput;
	float outputLimit;
	
	float underOfPoint;
	
	float kP;
	float kI;
	float kD;

        
        float error_Array[ERROR_ARRAY_NUM];
        int error_Array_Cnt;
}PID;

//** Motor Value **//
typedef struct _MOTOR{

		int MotorNum;
          
        int Encoder_Raw;
        int Encoder_Raw_past;
        
        long long nowEn;
        long long pastEn;
        long long Endif;
        long long epsilon;
        
        float RPM;
        float realRPM;
        float refRPM;
        float Degree;
        
        float TargetRPM;
        float TargetPOS;
        float TargetCUR;

        float maxRPM;
        float maxCUR;

        
        
}MOTOR;

//** BLDC Value **//
typedef struct _BLDC{
  
        int Encoder_Raw;
        int Encoder_Raw_past;
        
        long long nowEn;
        long long pastEn;
        long long Endif;
        long long epsilon;
        
        
        float RPM;
        float refRPM;

}BLDC;






/* Part of PID Function */

void PID_Control(PID* dst,
		float target,
		float input
		);



void Get_Motor_RPM(MOTOR* dst, TIM_TypeDef* TIMx);

void Get_Motor_RPM_BASE(MOTOR* dst, TIM_TypeDef* TIMx);

void MotorData_Init(MOTOR* dst, TIM_TypeDef* TIMx);

void PID_Control_Initialize(PID* dst);

void Get_BLDC_RPM(BLDC* dst, TIM_TypeDef* TIMx);

void PID_Control_FL(PID* dst, float target, float input);

#endif /* PID_CONTROL_LONG_H_ */
