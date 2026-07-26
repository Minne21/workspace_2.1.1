/*
 * foc.c
 *
 *  Created on: Nov 29, 2025
 *      Author: Ray
 */

#include <math.h>

#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_cordic.h"

#include "foc.h"
#include "sensor.h"

/* ---------- Globals ---------- */
FOC_t foc;
PLL_Observer pll;


float dutyA, dutyB, dutyC;
float Va, Vb, Vc;



float Vd_cmd = 0.0f;        // voltage magnitude during open-loop
float Vq_cmd = 0.0f;   		// voltage magnitude during open-loop


/* ---------------------------------------------------------------------------
 * CORDIC peripheral configuration
 *
 * Call once from your init code (e.g. after MX_CORDIC_Init or in FOC_Init).
 * --------------------------------------------------------------------------- */
void CORDIC_Config(void)
{
    /* CORDIC_FUNCTION_COSINE:
     *   Write 1 value  → angle in q1.31
     *   Read  2 values → RDATA[0] = cos(θ), RDATA[1] = sin(θ)         */
    CORDIC->CSR = CORDIC_FUNCTION_COSINE    // cosine mode: outputs cos then sin
                | CORDIC_PRECISION_6CYCLES  // 6 iterations, ~20-bit accuracy
                | CORDIC_SCALE_0            // no scaling (q = 0)
                | CORDIC_NBWRITE_1          // one input write (angle only)
                | CORDIC_NBREAD_2           // two output reads (cos, sin)
                | CORDIC_INSIZE_32BITS       // 32-bit input
                | CORDIC_OUTSIZE_32BITS;     // 32-bit output
}

/* ---------------------------------------------------------------------------
 * cordic_sincos — hardware sin/cos, replaces sinf/cosf
 *
 *  Input : theta in radians (any range — wrapped internally)
 *  Output: *s = sin(theta),  *c = cos(theta)
 *
 *  Timing: CORDIC write fires immediately; ~6 cycles later the result is
 *  ready. Polling RRDY adds no cost if called after a few instructions.
 *  In your open-loop ISR the poll will return instantly.
 * --------------------------------------------------------------------------- */
static inline void cordic_sincos(float theta, float *s, float *c)
{
    /* --- Normalise θ to q1.31 range [-1, +1) → maps to [-π, +π) ---
     *
     *  q1.31 = θ / π,  clamped to [-1, +1)
     *
     *  Fast wrap to [-π, +π) using multiply + floor instead of fmodf:
     *    1. Divide by 2π to get cycles
     *    2. Subtract integer part  → fractional part in [0, 1)
     *    3. Map [0, 1) → [-π, +π) = shift by -0.5, scale by 2π
     *    4. Then divide by π for q1.31
     *
     *  Simplified: q1.31_val = (theta / π) − 2 * floor(theta / (2π))
     *  But the CORDIC only needs [-1, +1), so:
     */

    /* Step 1: reduce to [-π, +π) */
    float t = theta * ONE_OVER_PI;          // t = θ/π,  range is unbounded
    /* Subtract even integer to bring into [-1, +1) */
    t = t - 2.0f * (float)(int32_t)(t * 0.5f);  // fast floor via truncation

    /* Edge: truncation rounds toward zero, adjust if still out of range */
    if      (t >  1.0f) t -= 2.0f;
    else if (t < -1.0f) t += 2.0f;

    /* Step 2: convert float [-1, +1) → q1.31 integer */
    int32_t angle_q31 = (int32_t)(t * Q31_SCALE);

    /* Step 3: write to CORDIC (starts computation immediately) */
    CORDIC->WDATA = (uint32_t)angle_q31;

    /* Step 4: poll RRDY — ready after ~6 clock cycles                     */
    /* In a 20 kHz ISR with a few instructions between write and read,     */
    /* RRDY is typically already set by the time we get here.              */
    while (!(CORDIC->CSR & CORDIC_CSR_RRDY));

    /* Step 5: read results — COSINE mode: first read = cos, second = sin  */
    int32_t cos_q31 = (int32_t)CORDIC->RDATA;
    int32_t sin_q31 = (int32_t)CORDIC->RDATA;

    /* Step 6: q1.31 → float */
    *c = (float)cos_q31 / Q31_SCALE;
    *s = (float)sin_q31 / Q31_SCALE;
}


static inline void Clarke(float ia, float ib, float ic, float *alpha, float *beta)
{
    *alpha = ia;
    *beta  = (ia + 2.0f * ib) * 0.57735026919f;
}

static inline void Park(float alpha, float beta, float th, float *d, float *q)
{
    float s, c;
    cordic_sincos(th, &s, &c);
    *d =  alpha * c + beta * s;
    *q = -alpha * s + beta * c;
}

/* Inverse Park: dq -> alpha-beta */
static inline void InvPark(float d, float q, float theta, float *alpha, float *beta)
{
    float s, c;
    cordic_sincos(theta, &s, &c);
    *alpha = d * c - q * s;
    *beta  = d * s + q * c;
}


/* ---------- PI ---------- */

static inline float PI_Update(PI_t *pi, float ref, float fb, float Ts)
{
    float err = ref - fb;
    pi->err = err;
    float p = pi->kp * err;
    float i_next = pi->integrator + (pi->ki * err * Ts);

    float out_raw = p + i_next;

    // Determine saturation
    float out_sat = out_raw;
    if (out_sat > pi->out_max) out_sat = pi->out_max;
    else if (out_sat < pi->out_min) out_sat = pi->out_min;

    // ANTI-WINDUP: Conditional Integration
    // Only update if not saturated OR if error is moving output back to linear range
    bool is_saturated = (out_raw != out_sat);
    bool reducing_error = (out_raw > pi->out_max && err < 0.0f) ||
                          (out_raw < pi->out_min && err > 0.0f);

    if (!is_saturated || reducing_error) {
        pi->integrator = i_next;
    }

    return out_sat;
}

// --- SVPWM ---
void SVPWM(float Valpha, float Vbeta)
{
    /* Inverse Clarke */
    float Va = Valpha;
    float Vb = -0.5f * Valpha + 0.8660254038f * Vbeta;
    float Vc = -0.5f * Valpha - 0.8660254038f * Vbeta;

    /* Zero-sequence injection */
    float vmax = fmaxf(fmaxf(Va, Vb), Vc);
    float vmin = fminf(fminf(Va, Vb), Vc);
    float voffset = 0.5f * (vmax + vmin);

    dutyA = 0.5f + (Va - voffset) / Vbus; 	// if openloop, VDC; // VBUS_VOLTAGE;
    dutyB = 0.5f + (Vb - voffset) / Vbus; 	// if openloop, VDC; // VBUS_VOLTAGE;
    dutyC = 0.5f + (Vc - voffset) / Vbus; 	// if openloop, VDC; // VBUS_VOLTAGE;

    // Limit
    if (dutyA < 0) dutyA = 0; else if (dutyA > 1) dutyA = 1;
    if (dutyB < 0) dutyB = 0; else if (dutyB > 1) dutyB = 1;
    if (dutyC < 0) dutyC = 0; else if (dutyC > 1) dutyC = 1;

    // PWM set ready
    uint32_t period = TIM1->ARR;		// period is 4250

    TIM1->CCR1 = dutyA * period;
    TIM1->CCR2 = dutyB * period;
    TIM1->CCR3 = dutyC * period;
}


#define VDQ_LIMIT   6.0f   // for 12V bus (50%)
AS5047_Enc_t motor_enc;

void FOC_Init(void)
{
    foc.aligned      = false;
    foc.align_timer  = 0.0f;
    foc.mech_offset  = 0.0f;

    foc.id_ref = 0.0f;
    foc.iq_ref = 0.5f;

    /* Current loop */

    foc.pi_id.kp = 0.05f;
    foc.pi_id.ki = 0.1f;
    foc.pi_id.out_min = -VDQ_LIMIT;
    foc.pi_id.out_max =  VDQ_LIMIT;
    foc.pi_id.integrator = 0.0f;

    foc.pi_iq.kp = 0.05f;
    foc.pi_iq.ki = 0.1f;
    foc.pi_iq.out_min = -VDQ_LIMIT;
    foc.pi_iq.out_max =  VDQ_LIMIT;
    foc.pi_iq.integrator = 0.0f;

    /* Speed loop */

    foc.speed_ref_rpm = 300.0f;   // 300 RPM target
    foc.speed_ref = foc.speed_ref_rpm * 0.10472f;  // RPM → rad/s

    foc.pi_speed.kp = 0.1f;
    foc.pi_speed.ki = 0.5f;
    foc.pi_speed.out_min = -3.0f;		// max negative torque current
    foc.pi_speed.out_max =  3.0f;
    foc.pi_speed.integrator = 0.0f;

    /* Position loop */

    foc.position_ref = 0; 				//M_PI / 3;

    foc.pi_position.kp = 0.5f;     		// START conservative
    foc.pi_position.ki = 0.0f;      	// Add later if needed, PI often causes oscillation if you jump straight to it.
    foc.pi_position.out_min = -30.0f;  	// rad/s
    foc.pi_position.out_max =  30.0f;
    foc.pi_position.integrator = 0.0f;

//    CORDIC_Config();

    AS5047_Enc_Init(&motor_enc);
}


#define SPEED_DIV 20   // 20 kHz / 20 = 1 kHz


float mech_prev;
float angle_raw;

float mech_speed = 0.0f;      // rad/s
float mech_rpm   = 0.0f;


float dtheta= 0.0f;
uint8_t speed_cnt = 0;


float filtered_speed = 0.0f;


float alpha_current = 0.1f; // Smoothing factor (0.0 to 1.0)
float alpha_1ms = 0.1f;  // Filter for 1ms task (can be slightly higher than 50us version)

float ramp_rate = 0.001f;
float iq_ref_ramp = 0.0f;


float mech_rad_s = 0.0f;
float last_mech_rad = 0.0f;


__attribute__((section(".ccmram"))) void FOC_PWM_ISR(void);

void FOC_PWM_ISR(void)
{
	/* ---------- 1. PROCESS LATEST DATA ---------- */
	    AS5047_Enc_Update(&motor_enc);

	    /* ---------- 2. Alignment ---------- */
	    if (!foc.aligned)
	    {
	        foc.align_timer += Ts_current;
	        foc.vd = ALIGN_VOLT;
	        foc.vq = 0.0f;

	        // 정렬 시에는 0.0f 각도로 고정하여 D-축을 특정 위상에 고정
	        InvPark(ALIGN_VOLT, 0.0f, 0.0f, &foc.valpha, &foc.vbeta);
	        SVPWM(foc.valpha, foc.vbeta);

	        if (foc.align_timer >= ALIGN_TIME_S)
	        {
	            // 중요: 고정된 위치의 전기적 각도를 오프셋으로 저장
	            foc.elec_offset = motor_enc.elec_rad;
//	            foc.mech_offset = motor_enc.mech_rad;
	            foc.aligned = true;

	            foc.pi_id.integrator = 0.0f;
	            foc.pi_iq.integrator = 0.0f;
	        }
	        return;
	    }
//	    foc.elec_offset = 0.3f;

	    // 정렬된 오프셋을 반영하여 실제 제어에 사용할 전기적 각도 계산
	    float control_elec_rad = motor_enc.elec_rad - foc.elec_offset;

	    // 0 ~ 2*PI 범위 유지 (빠른 조건문)
	    while (control_elec_rad < 0.0f) control_elec_rad += 6.2831853f;
	    while (control_elec_rad >= 6.2831853f) control_elec_rad -= 6.2831853f;

	    foc.mech_rad = motor_enc.mech_rad;
	    foc.elec_rad = control_elec_rad; // 이제 이 각도가 정렬된 각도입니다.



	    /* ---------- 3. Currents & Transformations ---------- */
	    foc.ia = (alpha_current * PhaseU_A) + (1.0f - alpha_current) * foc.ia;
	    foc.ib = (alpha_current * PhaseV_A) + (1.0f - alpha_current) * foc.ib;
//	    foc.ic = (alpha_current * PhaseW_A) + (1.0f - alpha_current) * foc.ic;		// wrong value, 0
	    foc.ic = -(foc.ia + foc.ib);

	    Clarke(foc.ia, foc.ib, foc.ic, &foc.ialpha, &foc.ibeta);
	    // 정렬된 elec_rad를 사용하여 Park 변환 수행 -> iq가 DC로 나옴
	    Park(foc.ialpha, foc.ibeta, foc.elec_rad, &foc.id, &foc.iq);

	    /* ---------- 4. Current PI Control ---------- */
	    foc.vd = PI_Update(&foc.pi_id, foc.id_ref, foc.id, Ts_current);
	    foc.vq = PI_Update(&foc.pi_iq, foc.iq_ref, foc.iq, Ts_current);

	    /* ---------- 5. Limit & Output ---------- */
	    float v_limit = Vbus * 0.577f;
	    float v_mag = sqrtf(foc.vd*foc.vd + foc.vq*foc.vq);

	    if (v_mag > v_limit) {
	        float scale = v_limit / v_mag;
	        foc.vd *= scale;
	        foc.vq *= scale;
	    }

	    // 테스트를 위한 강제값 해제 (이제 PI 제어가 정상 각도에서 작동함)
//	    foc.vd = 0.0f;
//	    foc.vq = 0.5f;

	    InvPark(foc.vd, foc.vq, foc.elec_rad, &foc.valpha, &foc.vbeta);
	    SVPWM(foc.valpha, foc.vbeta);


    /* ---------- Speed loop (1 ms) ---------- */
    speed_cnt++;

    if (speed_cnt >= SPEED_DIV)   // 20 * 50us = 1ms
    {
        speed_cnt = 0;

        /* --- Position difference --- */
        float angle_now = motor_enc.mech_rad;

        float delta_theta = angle_now - last_mech_rad;

        // unwrap
        if (delta_theta > M_PI)       delta_theta -= TWO_PI;
        else if (delta_theta < -M_PI) delta_theta += TWO_PI;

        last_mech_rad = angle_now;

        /* --- Speed (rad/s) --- */
        float instant_rad_s = delta_theta * 1000.0f;   // 1ms → rad/s

        /* --- Low-pass filter --- */
        mech_rad_s += alpha_1ms * (instant_rad_s - mech_rad_s);

        /* --- Convert to RPM (for debug) --- */
        mech_rpm = mech_rad_s * 9.5493f;


        /* ---------- SPEED PI ---------- */
        float iq_cmd = PI_Update(&foc.pi_speed, foc.speed_ref,     // rad/s
                                 mech_rad_s, 0.001f);           // 1ms

        /* ---------- Ramp (IMPORTANT) ---------- */
        float ramp_step = 0.02f;

        if (iq_ref_ramp < iq_cmd)
            iq_ref_ramp += ramp_step;
        else if (iq_ref_ramp > iq_cmd)
            iq_ref_ramp -= ramp_step;

        /* ---------- Clamp ---------- */
        if (iq_ref_ramp > 3.0f)  iq_ref_ramp = 3.0f;
        if (iq_ref_ramp < -3.0f) iq_ref_ramp = -3.0f;

        /* ---------- Apply ---------- */
        foc.iq_ref = iq_ref_ramp;

    }
}


#if 0
void FOC_PWM_ISR(void)
{

	/* ---------- 1. PROCESS LATEST DMA DATA ---------- */
	AS5600_Update();

#if 0
	/* ---------- Alignment ---------- */
    if (!foc.aligned)
    {
        foc.align_timer += Ts_current;
        foc.vd = ALIGN_VOLT;
        foc.vq = 0.0f;

        InvPark(foc.vd, foc.vq, 0.0f, &foc.valpha, &foc.vbeta);
        SVPWM(foc.valpha, foc.vbeta);

        if (foc.align_timer >= ALIGN_TIME_S)
        {
            foc.mech_offset = foc.mech_rad;
            foc.aligned = true;
            foc.pi_id.integrator = 0.0f;
            foc.pi_iq.integrator = 0.0f;
        }

        return;
    }

    /* ---------- Angle ---------- */
    float mech = foc.mech_rad - foc.mech_offset;
    mech = fmodf(mech, TWO_PI);
    if (mech < 0) mech += TWO_PI;

    foc.elec_rad = fmodf(mech * POLE_PAIRS, TWO_PI);
#endif


    /* ---------- Currents ---------- */
    foc.ia = PhaseU_A;
    foc.ib = PhaseV_A;
    foc.ic = PhaseW_A;

    Clarke(foc.ia, foc.ib, foc.ic, &foc.ialpha, &foc.ibeta);
    Park(foc.ialpha, foc.ibeta, foc.elec_rad, &foc.id, &foc.iq);

    /* ---------- Current PI ---------- */
    foc.vd = PI_Update(&foc.pi_id, foc.id_ref, foc.id, Ts_current);
    foc.vq = PI_Update(&foc.pi_iq, foc.iq_ref, foc.iq, Ts_current);

#if 0
    /* ---------- Voltage magnitude limit ---------- */
    float v2 = foc.vd * foc.vd + foc.vq * foc.vq;
    if (v2 > (VMAX_DQ * VMAX_DQ))
    {
        float s = VMAX_DQ / sqrtf(v2);
        foc.vd *= s;
        foc.vq *= s;
    }
#endif

    /* ---------- 6. OUTPUT ---------- */
    InvPark(foc.vd, foc.vq, foc.elec_rad, &foc.valpha, &foc.vbeta);
    SVPWM(foc.valpha, foc.vbeta);

#if speed
    speed_cnt++;

    if (speed_cnt >= SPEED_DIV)
    {
        AS5600_Update();
        angle_raw = foc.mech_rad;
        dtheta = angle_raw - mech_prev;

        // Unwrap
        if (dtheta >  M_PI) dtheta -= TWO_PI;
        if (dtheta < -M_PI) dtheta += TWO_PI;
        mech_prev = angle_raw;

        /* optional: reject reverse spike (open-loop only) */
        if (dtheta < 0.0f)
            dtheta = 0.0f;

        // 1. Calculate raw speed
        float raw_speed = dtheta / (SPEED_DIV * Ts_current);

        // 2. Exponential Moving Average (EMA)
        // Formula: Output = alpha * current + (1 - alpha) * previous
        filtered_speed = (alpha * raw_speed) + ((1.0f - alpha) * filtered_speed);

        // 3. Final Outputs
        mech_speed = filtered_speed;
        mech_rpm = mech_speed * 9.5493f;

        speed_cnt = 0;
    }
#endif

}
#endif
// in the speed loop 1kHz
// AS5600_CheckBus();
