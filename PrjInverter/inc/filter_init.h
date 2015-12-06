/*
 * iir_filter.h
 *
 *  Created on: 2015Äê2ÔÂ5ÈÕ
 *      Author: Administrator
 */

#ifndef IIR_FILTER_H_
#define IIR_FILTER_H_

void iir_filter_init(iir_filter_instance*S, float *pCoeffs, float *pState);
void iir_filter_calculate(iir_filter_instance*S, float in, float* pout);


#endif /* IIR_FILTER_H_ */
