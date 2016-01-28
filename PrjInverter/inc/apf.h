/*
 * apf.h
 *
 *  Created on: 2015-9-27
 *      Author: Administrator
 */

#ifndef APF_H_
#define APF_H_

void APF_Init(void);
interrupt void APF_Main(void);
interrupt void FaultProcess(void);
char CheckPwrState(float uab, float ubc, float uca, float udc);

#endif /* APF_H_ */
