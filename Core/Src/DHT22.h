/*
 * DHT22.h
 *
 *  Created on: Feb 12, 2024
 *      Author: greatreyhan
 */

#ifndef SRC_DHT22_H_
#define SRC_DHT22_H_

typedef struct
{
	float Temperature;
	float Humidity;
}DHT_Typedef;


void DHT_GetData (DHT_Typedef *DHT_Data);
void DHT_Start (void);

#endif /* SRC_DHT22_H_ */
