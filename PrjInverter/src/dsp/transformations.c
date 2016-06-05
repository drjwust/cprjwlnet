/*
 * transformations.c
 *
 *  Created on: 2016Äê1ÔÂ31ÈÕ
 *      Author: Administrator
 */

#include <dsp.h>

void inv_clarke(float Ialpha, float Ibeta, float * pIa, float * pIb)
{
	/* Calculating pIa from Ialpha by equation pIa = Ialpha */
	*pIa = Ialpha;

	/* Calculating pIb from Ialpha and Ibeta by equation pIb = -(1/2) * Ialpha + (sqrt(3)/2) * Ibeta */
	*pIb = -0.5 * Ialpha + (float) 0.8660254039 * Ibeta;

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

void clarke(float Ia, float Ib, float * pIalpha, float * pIbeta)
{
	/* Calculate pIalpha using the equation, pIalpha = Ia */
	*pIalpha = Ia;

	/* Calculate pIbeta using the equation, pIbeta = (1/sqrt(3)) * Ia + (2/sqrt(3)) * Ib */
	*pIbeta = ((float) (Ia + 2 * Ib) * 0.57735026918963);

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

void park(float Ialpha, float Ibeta, float * pId, float * pIq, float sinVal,
		float cosVal)
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

void inv_park(float Id, float Iq, float * pIalpha, float * pIbeta, float sinVal,
		float cosVal)
{
	/* Calculate pIalpha using the equation, pIalpha = Id * cosVal - Iq * sinVal */
	*pIalpha = Id * cosVal - Iq * sinVal;

	/* Calculate pIbeta using the equation, pIbeta = Id * sinVal + Iq * cosVal */
	*pIbeta = Id * sinVal + Iq * cosVal;

}
