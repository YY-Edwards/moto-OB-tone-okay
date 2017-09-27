/*
 * accelerometer.h
 *
 *  Created on: Jun 11, 2009
 *      Author: cmr003
 */

#ifndef ACCELEROMETER_H_
#define ACCELEROMETER_H_
U32 my_readabyte(U32 subaddress, S8 *datareceived);
U32 my_writeabyte(U32 subaddress, U32 datatosend);
void accelerometer_init(void);
void processAccelerometer(void);
void processDoubleClick(void);
void filter(U32 index);
#endif /* ACCELEROMETER_H_ */
