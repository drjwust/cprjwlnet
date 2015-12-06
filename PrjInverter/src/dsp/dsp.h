#ifndef _DSP_CTRL_H
#define _DSP_CTRL_H
#include "string.h"
#include "math.h"
#include "fpu_vector.h"

#ifndef PI
#define PI					3.14159265358979f
#endif

#ifndef DSP28_DATA_TYPES
#define DSP28_DATA_TYPES
typedef char int8;
typedef int int16;
typedef long int32;
typedef long long int64;
typedef unsigned char Uint8;
typedef unsigned int Uint16;
typedef unsigned long Uint32;
typedef unsigned long long Uint64;
typedef float float32;
typedef long double float64;
#endif

/**
 * @brief Instance structure for the floating-point Biquad cascade filter.
 */
typedef struct
{
	float *pState; /**< Points to the array of state coefficients.  The array is of length 4*numStages. */
	float *pCoeffs; /**< Points to the array of coefficients.  The array is of length 5*numStages. */

} iir_filter_instance;

/**
 * @brief Instance structure for the floating-point PID Control.
 */
typedef struct
{
	float A0; /**< The derived gain, A0 = Kp + Ki + Kd . */
	float A1; /**< The derived gain, A1 = -Kp - 2Kd. */
	float A2; /**< The derived gain, A2 = Kd . */
	float state[3]; /**< The state array of length 3. */
	float Kp; /**< The proportional gain. */
	float Ki; /**< The integral gain. */
	float Kd; /**< The derivative gain. */
} pid_instance;


void iir_filter_calculate(iir_filter_instance*S, float in, float* pout);
/**
 * @brief  Initialization function for the floating-point Biquad cascade filter.
 * @param[in,out] *S           points to an instance of the floating-point Biquad cascade structure.
 * @param[in]     *pCoeffs     points to the filter coefficients.
 * @param[in]     *pState      points to the state buffer.
 * @return        none
 */
void iir_filter_init(iir_filter_instance*S, float *pCoeffs, float *pState);


/**
 * @brief  Initialization function for the floating-point PID Control.
 * @param[in,out] *S      points to an instance of the PID structure.
 * @param[in]     resetStateFlag  flag to reset the state. 0 = no change in state 1 = reset the state.
 * @return none.
 */
void pid_init(pid_instance * S, int32 resetStateFlag);

/**
 * @brief  Reset function for the floating-point PID Control.
 * @param[in,out] *S is an instance of the floating-point PID Control structure
 * @return none
 */
void pid_reset(pid_instance * S);

static __inline float pid_calculate(pid_instance * S, float in)
{
	float out;

	/* y[n] = y[n-1] + A0 * x[n] + A1 * x[n-1] + A2 * x[n-2]  */
	out = (S->A0 * in) + (S->A1 * S->state[0]) + (S->A2 * S->state[1])
			+ (S->state[2]);

	/* Update state */
	S->state[1] = S->state[0];
	S->state[0] = in;
	S->state[2] = out;

	/* return to application */
	return (out);

}

static __inline void inv_clarke(
float Ialpha,
float Ibeta,
float * pIa,
float * pIb)
{
  /* Calculating pIa from Ialpha by equation pIa = Ialpha */
  *pIa = Ialpha;

  /* Calculating pIb from Ialpha and Ibeta by equation pIb = -(1/2) * Ialpha + (sqrt(3)/2) * Ibeta */
  *pIb = -0.5 * Ialpha + (float) 0.8660254039 *Ibeta;

}

/**
  *
  * @brief  Floating-point Clarke transform
  * @param[in]       Ia       input three-phase coordinate <code>a</code>
  * @param[in]       Ib       input three-phase coordinate <code>b</code>
  * @param[out]      *pIalpha points to output two-phase orthogonal vector axis alpha
  * @param[out]      *pIbeta  points to output two-phase orthogonal vector axis beta
  * @return none.
  */

 static __inline void clarke(
 float Ia,
 float Ib,
 float * pIalpha,
 float * pIbeta)
 {
   /* Calculate pIalpha using the equation, pIalpha = Ia */
   *pIalpha = Ia;

   /* Calculate pIbeta using the equation, pIbeta = (1/sqrt(3)) * Ia + (2/sqrt(3)) * Ib */
   *pIbeta =
     ((float) 0.57735026919 * Ia + (float) 1.15470053838 * Ib);

 }

/**
 * @addtogroup park
 * @{
 */

/**
 * @brief Floating-point Park transform
 * @param[in]       Ialpha input two-phase vector coordinate alpha
 * @param[in]       Ibeta  input two-phase vector coordinate beta
 * @param[out]      *pId   points to output	rotor reference frame d
 * @param[out]      *pIq   points to output	rotor reference frame q
 * @param[in]       sinVal sine value of rotation angle theta
 * @param[in]       cosVal cosine value of rotation angle theta
 * @return none.
 *
 * The function implements the forward Park transform.
 *
 */



static inline void park(float Ialpha, float Ibeta, float * pId, float * pIq,
		float sinVal, float cosVal)
{
	/* Calculate pId using the equation, pId = Ialpha * cosVal + Ibeta * sinVal */
	*pId = Ialpha * cosVal + Ibeta * sinVal;

	/* Calculate pIq using the equation, pIq = - Ialpha * sinVal + Ibeta * cosVal */
	*pIq = -Ialpha * sinVal + Ibeta * cosVal;

}


/**
 * @brief  Floating-point Inverse Park transform
 * @param[in]       Id        input coordinate of rotor reference frame d
 * @param[in]       Iq        input coordinate of rotor reference frame q
 * @param[out]      *pIalpha  points to output two-phase orthogonal vector axis alpha
 * @param[out]      *pIbeta   points to output two-phase orthogonal vector axis beta
 * @param[in]       sinVal    sine value of rotation angle theta
 * @param[in]       cosVal    cosine value of rotation angle theta
 * @return none.
 */

static inline void inv_park(float Id, float Iq, float * pIalpha, float * pIbeta,
		float sinVal, float cosVal)
{
	/* Calculate pIalpha using the equation, pIalpha = Id * cosVal - Iq * sinVal */
	*pIalpha = Id * cosVal - Iq * sinVal;

	/* Calculate pIbeta using the equation, pIbeta = Id * sinVal + Iq * cosVal */
	*pIbeta = Id * sinVal + Iq * cosVal;

}

#endif /* _ARM_MATH_H */

/**
 *
 * End of file.
 */
