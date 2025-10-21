#include "algorithm_pid.h"


/**
 ******************************************************************************
 * @file    algorithm_pid.c
 * @author  Wang Hongxi
 * @author  Zhang Hongyu (fuzzy pid)
 * @version V1.1.3
 * @date    2024/11/10
 * @brief   DWT��ʱ�����ڼ���������� OLS������ȡ�ź�΢��
 ******************************************************************************
 */

PID_Ctrl PID;
PID_T    PID_CT;

/**
  * @brief          PID��ʼ��
	* @param[in]      pid: PID�ṹ����ָ��
  * @param[in]      PID_Coefficient[3]: PID������������ΪKp��Ki��Kd
	* @param[in]      max_Iout: ���������
	* @param[in]      max_out: PID������
	* @param[in]      band_I: ���ַ�����ֵ
  * @retval         none
  */
  
 inline float my_fabs(float x) {
    return x < 0 ? -x : x;
}
inline double my_fabs(double x) {
    return x < 0 ? -x : x;
}
/// @brief 
/// @param pid PID�ṹ����ָ��
/// @param mode PIDģʽ
/// @param Kp PID����ϵ��
/// @param Ki PID����ϵ��
/// @param Kd PID΢��ϵ��
/// @param max_out PID������
/// @param max_Iout PID���������
/// @param band_I ���ַ�����ֵ
void PID_Ctrl::Init(sPidTypeDef *pid, PidMode mode, fp32 Kp, fp32 Ki, fp32 Kd, fp32 max_out, fp32 max_Iout, fp32 band_I)
{
	if(pid == NULL)
	{
		return;
	}
	pid->mode = mode;
	pid->Kp = Kp;
	pid->Ki = Ki;
	pid->Kd = Kd;
	pid->max_out = max_out;
	pid->max_Iout = max_Iout;
	pid->band_I = band_I;
	
}

/**
  * @brief          pid������
  * @param[in]      pid: PID�ṹ����ָ��
  * @retval         none
  */
void PID_Ctrl::Clear(sPidTypeDef *pid)
{
	if(pid == NULL)
	{
		return;
	}
	pid->error[0] = pid->error[1] = pid->error[2] = 0.0f;
	pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
	pid->out = pid->P_out = pid->I_out = pid->D_out = 0.0f;
	pid->ref = pid->set = 0.0f;
}


/**
  * @brief          PID���㺯��
  * @param[in]      pid: PID�ṹ����ָ��
  * @param[in]      set: �趨ֵ
  * @param[in]      ref: ����ֵ
  * @retval         PID���ֵ
  */
fp32 PID_Ctrl::Calc(sPidTypeDef *pid, fp32 ref, fp32 set)
{
	if(pid == NULL)
	{
		return 0.0f;
	}

	pid->set = set;
	pid->ref = ref;
	pid->error[2] = pid->error[1];
	pid->error[1] = pid->error[0];
	pid->error[0] = set - ref;

		pid->P_out = pid->Kp * pid->error[0];
		if(my_fabs(pid->error[0]) < pid->band_I)
		{
			pid->I_out += pid->Ki * pid->error[0];
		}
		pid->Dbuf[2] = pid->Dbuf[1];
		pid->Dbuf[1] = pid->Dbuf[0];
		pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
		pid->D_out = pid->Kd * pid->Dbuf[0];
		
		LimitMax(pid->I_out, pid->max_Iout);
		pid->out = pid->P_out + pid->I_out + pid->D_out;
		LimitMax(pid->out, pid->max_out);
		
		
	return pid->out;
}
float sPidTypeDef::Forwardfeed(float in)
{
    float out;
    out = (in - last_in);
    last_in = in;
    return out;
}

float PID_T::Forwardfeed(PID_t *pid)
{
    static float out;
    out = (pid->Output - pid->Last_in)* pid->Fd_K + pid->Output;
    pid->Last_in = pid->Output;
		out=constrain(out,-9.5,+9.5);
    return out;
}
/*******************************************************Wang Hongxi************************************************************************/

/**
 * @brief          PID��ʼ��   PID initialize
 * @param[in]      PID�ṹ��   PID structure
 * @param[in]      ��
 * @retval         ���ؿ�      null
 */
void PID_T::Init(
			 PID_t *pid,
			float max_out,
			float intergral_limit,
			float deadband,

			float kp,
			float Ki,
			float Kd,

			float A,
			float B,

			float output_lpf_rc,
			float derivative_lpf_rc,

			uint16_t ols_order,
			uint8_t  improve
		)
{
    pid->DeadBand = deadband;
    pid->IntegralLimit = intergral_limit;
    pid->MaxOut = max_out;
    pid->Ref = 0;

    pid->Kp = kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->ITerm = 0;

    // ���ٻ��ֲ���
    // coefficient of changing integration rate
    pid->CoefA = A;
    pid->CoefB = B;

    pid->Output_LPF_RC = output_lpf_rc;

    pid->Derivative_LPF_RC = derivative_lpf_rc;

    // ��С������ȡ�ź�΢�ֳ�ʼ��
    // differential signal is distilled by OLS
    pid->OLS_Order = ols_order;
    //OLS_Init(&pid->OLS, ols_order);

    // DWT��ʱ��������������
    // reset DWT Timer count counter
    pid->DWT_CNT = 0;

    // ����PID�Ż�����
    pid->Improve = improve;

    // ����PID�쳣���� Ŀǰ�����������ת����
    pid->ERRORHandler.ERRORCount = 0;
    pid->ERRORHandler.ERRORType = PID_ERROR_NONE;

    pid->Output = 0;
}

/**
 * @brief          PID����
 * @param[in]      PID�ṹ��
 * @param[in]      ����ֵ
 * @param[in]      ����ֵ
 * @retval         ���ؿ�
 */
float PID_T::Calc(PID_t *pid, float measure, float ref)
{
    if (pid->Improve & ErrorHandle)
        f_PID_ErrorHandle(pid);

    // pid->dt = DWT_GetDeltaT((uint32_t*)&pid->DWT_CNT);
    pid->dt = 0.01f; // �̶���������10ms
    pid->Measure = measure;
    pid->Ref = ref;
    pid->Err = pid->Ref - pid->Measure;

    if (pid->User_Func1_f != NULL)
        pid->User_Func1_f(pid);
		
    if (my_fabs(pid->Err) > pid->DeadBand)
    {
		  pid->Pout  = pid->Kp * pid->Err;
			pid->ITerm = pid->Ki * pid->Err * pid->dt;
			pid->Dout  = pid->Kd * (pid->Err - pid->Last_Err) / pid->dt;

			//���λ���
			if (pid->Improve & Trapezoid_Intergral)
					f_Trapezoid_Intergral(pid);
			// ���ٻ���
			if (pid->Improve & ChangingIntegrationRate)
					f_Changing_Integration_Rate(pid);
			// ΢������
			if (pid->Improve & Derivative_On_Measurement)
					f_Derivative_On_Measurement(pid);
			// ΢���˲���
			if (pid->Improve & DerivativeFilter)
					f_Derivative_Filter(pid);
			// �����޷�
			if (pid->Improve & Integral_Limit)
					f_Integral_Limit(pid);
	
			pid->Iout   += pid->ITerm;
			pid->Output = pid->Pout + pid->Iout + pid->Dout/1000.0f;

			// ����˲�
			if (pid->Improve & OutputFilter)
			f_Output_Filter(pid);

			// ����޷�
			f_Output_Limit(pid);

			// �޹ؽ�Ҫ
			f_Proportion_Limit(pid);
    }
    pid->Last_Measure = pid->Measure;
    pid->Last_Output = pid->Output;
    pid->Last_Dout = pid->Dout;
    pid->Last_Err = pid->Err;
    pid->Last_ITerm = pid->ITerm;

    return pid->Output;
}
static void f_Trapezoid_Intergral(PID_t *pid)
{
    pid->ITerm = pid->Ki * ((pid->Err + pid->Last_Err) / 2) * pid->dt;
}

static void f_Changing_Integration_Rate(PID_t *pid)
{
	 if(my_fabs(pid->Err)>(pid->CoefA + pid->CoefB))
	 {
			pid->Iout = 0;
		  pid->ITerm =0;
	 }
   else if(pid->Err * pid->Iout > 0)
    {
        // ���ֳ��ۻ�����
        // Integral still increasing
        if (my_fabs(pid->Err) <= pid->CoefB)
            return; // Full integral
        if (my_fabs(pid->Err) <= (pid->CoefA + pid->CoefB))
            pid->ITerm *= (pid->CoefA - my_fabs(pid->Err) + pid->CoefB) / pid->CoefA;
				else
						pid->ITerm =0;
		}
}

static void f_Integral_Limit(PID_t *pid)
{
    static float temp_Output, temp_Iout;
    temp_Iout = pid->Iout + pid->ITerm;
    temp_Output = pid->Pout + pid->Iout + pid->Dout;
    if (my_fabs(temp_Output) > pid->MaxOut)
    {
        if (pid->Err * pid->Iout > 0)
        {
            // ���ֳ��ۻ�����
            // Integral still increasing
            pid->ITerm = 0;
        }
    }
    if (temp_Iout > pid->IntegralLimit)
    {
        pid->ITerm = 0;
        pid->Iout = pid->IntegralLimit;
    }
    if (temp_Iout < -pid->IntegralLimit)
    {
        pid->ITerm = 0;
        pid->Iout = -pid->IntegralLimit;
    }
}

static void f_Derivative_On_Measurement(PID_t *pid)
{
	pid->Dout = pid->Kd * (pid->Last_Measure - pid->Measure) / pid->dt;
}

static void f_Derivative_Filter(PID_t *pid)
{
    pid->Dout = pid->Dout * pid->dt / (pid->Derivative_LPF_RC + pid->dt) +
                pid->Last_Dout * pid->Derivative_LPF_RC / (pid->Derivative_LPF_RC + pid->dt);
}

static void f_Output_Filter(PID_t *pid)
{
    pid->Output = pid->Output * pid->dt / (pid->Output_LPF_RC + pid->dt) +
                  pid->Last_Output * pid->Output_LPF_RC / (pid->Output_LPF_RC + pid->dt);
}

static void f_Output_Limit(PID_t *pid)
{
    if (pid->Output > pid->MaxOut)
    {
        pid->Output = pid->MaxOut;
    }
    if (pid->Output < -(pid->MaxOut))
    {
        pid->Output = -(pid->MaxOut);
    }
}

static void f_Proportion_Limit(PID_t *pid)
{
    if (pid->Pout > pid->MaxOut)
    {
        pid->Pout = pid->MaxOut;
    }
    if (pid->Pout < -(pid->MaxOut))
    {
        pid->Pout = -(pid->MaxOut);
    }
}

// PID ERRORHandle Function
static void f_PID_ErrorHandle(PID_t *pid)
{
    /*Motor Blocked Handle*/
    if (pid->Output < pid->MaxOut * 0.001f || my_fabs(pid->Ref) < 0.0001f)
        return;

    if ((my_fabs(pid->Ref - pid->Measure) / my_fabs(pid->Ref)) > 0.95f)
    {
        // Motor blocked counting
        pid->ERRORHandler.ERRORCount++;
    }
    else
    {
        pid->ERRORHandler.ERRORCount = 0;
    }

    if (pid->ERRORHandler.ERRORCount > 500)
    {
        // Motor blocked over 1000times
        pid->ERRORHandler.ERRORType = Motor_Blocked;
    }
}

