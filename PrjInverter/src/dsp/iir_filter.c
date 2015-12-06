/*
 * iir_filter.c
 *
 *  Created on: 2015-9-27
 *      Author: Administrator
 */
#include <dsp.h>

void iir_filter_init(iir_filter_instance*S, float *pCoeffs, float *pState)
{
	S->pCoeffs = pCoeffs;
	memset_fast(pState,0, 4*2);
	S->pState = pState;
}

void iir_filter_calculate(iir_filter_instance*S, float in, float* pout)
{
	float *pState = S->pState;
	float *pCoeffs = S->pCoeffs;
	float b0, b1, b2, a1, a2; /*  Filter coefficients       */
	float Xn1, Xn2, Yn1, Yn2; /*  Filter pState variables   */

	/* Reading the coefficients */
	b0 = *pCoeffs++;
	b1 = *pCoeffs++;
	b2 = *pCoeffs++;
	a1 = *pCoeffs++;
	a2 = *pCoeffs++;

	/* Reading the pState values */
	Xn1 = pState[0];
	Xn2 = pState[1];
	Yn1 = pState[2];
	Yn2 = pState[3];

	/*   acc =  b0 * x[n] + b1 * x[n-1] + b2 * x[n-2] + a1 * y[n-1] + a2 * y[n-2]   */
	*pout = (b0 * in) + (b1 * Xn1) + (b2 * Xn2) + (a1 * Yn1) + (a2 * Yn2);
}
