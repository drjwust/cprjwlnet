/*
 * bsp.h
 *
 *  Created on: 2015-9-27
 *      Author: Administrator
 */

#ifndef BSP_H_
#define BSP_H_

#define GPIO_LED33		41
#define GPIO_LED34		48
#define GPIO_FAN		86
#define GPIO_PWRON		87
#define GPIO_P7			51		//LED
#define	 GPIO_P8		52		//LED
#define GPIO_P9			53
#define GPIO_P10		54

 void SetOC1Value(float i);
 void SetOC2Value(float i);
 void SetOVValue(float i);
 void BSP_Init(void);
 void ADValueConvert(void);


#endif /* BSP_H_ */
