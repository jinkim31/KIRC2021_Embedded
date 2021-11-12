#include "PID_form.h"

void Get_Motor_RPM(MOTOR* dst, TIM_TypeDef* TIMx)
{
      dst->Encoder_Raw = TIMx->CNT;
      
      dst->nowEn= dst->Encoder_Raw;

      dst->Endif = -dst->nowEn + dst->pastEn;


      dst->pastEn = dst->nowEn;

      dst->RPM = (60.0 * dst->Endif) / (SAMPLING_TIMER6*PPR);

      dst->Degree += dst->Endif / ((PPR) / 360.0);


      if(dst->nowEn > 60000)
      {
    	  TIMx->CNT=5000;
    	  dst->pastEn = 5000;
      }

      else if(dst->nowEn < 5000)
      {
    	  TIMx->CNT=60000;
    	  dst->pastEn = 60000;
      }  
}

void Get_Motor_RPM_BASE(MOTOR* dst, TIM_TypeDef* TIMx)
{
      dst->Encoder_Raw = TIMx->CNT;

      dst->nowEn= dst->Encoder_Raw;

        dst->Endif = -dst->nowEn + dst->pastEn;


      dst->pastEn = dst->nowEn;

      dst->RPM = (60.0 * dst->Endif) / (SAMPLING_TIMER6*PPR);

      dst->Degree += dst->Endif / ((PPR) / 360.0);

      if(dst->nowEn > 60000)
      {
    	  TIMx->CNT=5000;
    	  dst->pastEn = 5000;
      }

      else if(dst->nowEn < 5000)
      {
    	  TIMx->CNT=60000;
    	  dst->pastEn = 60000;
      }
}

void MotorData_Init(MOTOR* dst, TIM_TypeDef* TIMx){
	TIMx->CNT=5000;
	dst->Degree=0;
	dst->RPM =0 ;
}

void PID_Control(PID* dst, float target, float input)
{
   dst->nowValue = input;
   dst->target = target;
   dst->nowError =  dst->nowValue -dst->target ;
        
        dst->nowError_int = (int)dst->nowError;
        
        
//        dst->error_Array[dst->error_Array_Cnt] = dst->nowError;
//        
//        dst->error_Array_Cnt++;
//        
//        if(dst->error_Array_Cnt == ERROR_ARRAY_NUM)
//        {
//          dst->error_Array_Cnt = 0;
//        }
//        
//        float temp=0;
//        
//        for(int i=0;i<ERROR_ARRAY_NUM;i++)
//        {
//          temp += dst->error_Array[i];
//        }
//        
//        
//   dst->errorSum = temp;
        
        
        dst->errorSum += dst->nowError;

   dst->errorDiff = dst->nowError - dst->pastError;
        
        
   if(dst->errorSumLimit != 0)
   {
      if(dst->errorSum > dst->errorSumLimit)
         dst->errorSum = dst->errorSumLimit;
      else if(dst->errorSum < -dst->errorSumLimit)
         dst->errorSum = -dst->errorSumLimit;
   }
        
       
   dst->nowOutput = 
         dst->kP * dst->nowError +
         dst->kI * dst->errorSum +
         dst->kD * dst->errorDiff;
        
        
        
        
   if(dst->underOfPoint == 0) return;   // Escape Error
        
   dst->nowOutput /= dst->underOfPoint;
   dst->pastError = dst->nowError;
   
   if(dst->outputLimit != 0)
   {
      if(dst->nowOutput > dst->outputLimit) dst->nowOutput = dst->outputLimit;
      else if(dst->nowOutput < -dst->outputLimit) dst->nowOutput = -dst->outputLimit;
   }

   if(dst->nowError == 0)
   {
	   dst->errorSum = 0;
   }
}


void PID_Control_FL(PID* dst, float target, float input)
{
	dst->nowValue = input;
	dst->target = target;
	dst->nowError =  dst->nowValue -dst->target ;
        
        dst->nowError_int = (int)dst->nowError;
        
        
        dst->error_Array[dst->error_Array_Cnt] = dst->nowError;
        
        dst->error_Array_Cnt++;
        
        if(dst->error_Array_Cnt == ERROR_ARRAY_NUM)
        {
          dst->error_Array_Cnt = 0;
        }
        
        float temp=0;
        
        for(int i=0;i<ERROR_ARRAY_NUM;i++)
        {
          temp += dst->error_Array[i];
        }
        
        
	dst->errorSum = temp;
        
        
	dst->errorDiff = dst->nowError - dst->pastError;
        
        
	if(dst->errorSumLimit != 0)
	{
		if(dst->errorSum > dst->errorSumLimit)
			dst->errorSum = dst->errorSumLimit;
		else if(dst->errorSum < -dst->errorSumLimit)
			dst->errorSum = -dst->errorSumLimit;
	}
        
       
	dst->nowOutput = 
			dst->kP * dst->nowError +
			dst->kI * dst->errorSum +
			dst->kD * dst->errorDiff;
        
        
        
        
	if(dst->underOfPoint == 0) return;	// Escape Error
        
	dst->nowOutput /= dst->underOfPoint;
	dst->pastError = dst->nowError;
	
	if(dst->outputLimit != 0)
	{
		if(dst->nowOutput > dst->outputLimit) dst->nowOutput = dst->outputLimit;
		else if(dst->nowOutput < -dst->outputLimit) dst->nowOutput = -dst->outputLimit;
	}

	if(dst->nowError == 0)
	{
		dst->errorSum = 0;
	}
}


void PID_Control_Initialize(PID* dst)
{
        for(int i=0;i<ERROR_ARRAY_NUM;i++)
        {
          dst->error_Array[i] = 0;
        }
        
        dst->error_Array_Cnt=0;
}

void Get_BLDC_RPM(BLDC* dst, TIM_TypeDef* TIMx)
{
      int dir = 0;
      
      
      dst->Encoder_Raw = TIMx->CNT;
      
      dst->nowEn= dst->Encoder_Raw;
      
      if((dst->Encoder_Raw - dst->Encoder_Raw_past) < -2147483647)            //En++ overflow
      {
        dst->epsilon += 4294967295;
      }
      else if((dst->Encoder_Raw - dst->Encoder_Raw_past) > 2147483647)        //En-- overflow
      {
        dst->epsilon -= 4294967295;
      }
      
      dst->nowEn += dst->epsilon;
      dst->Endif = dst->nowEn - dst->pastEn;
      
      
      dst->pastEn = dst->nowEn;
      dst->Encoder_Raw_past = dst->Encoder_Raw;  
      
      dst->RPM = (60.0 * dst->Endif) / ((float)SAMPLING_TIMER6 * BLDC_PPR);
      
//      if(dst->nowEn > 60000)
//      {
//   TIMx->CNT=5000;
//        dst->pastEn = 5000;
//      }
//
//      else if(dst->nowEn < 5000)
//      {
//   TIMx->CNT=60000;
//        dst->pastEn = 60000;
//      }  
      
      // 50:1 512
}



