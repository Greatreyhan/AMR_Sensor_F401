/*
 * MAX6675.h
 *
 *  Created on: May 10, 2024
 *      Author: greatreyhan
 */

#ifndef SRC_MAX6675_H_
#define SRC_MAX6675_H_
#include "main.h"

// ------------------------- Defines -------------------------
#define SSPORT GPIOB       // GPIO Port of Chip Select(Slave Select)
#define SSPIN  GPIO_PIN_10  // GPIO PIN of Chip Select(Slave Select)
// ------------------------- Functions  ----------------------
float Max6675_Read_Temp(void);

#endif /* SRC_MAX6675_H_ */
