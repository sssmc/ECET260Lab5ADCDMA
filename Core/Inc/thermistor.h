/*
 * thermistor.h
 *
 *  Created on: Sep 19, 2023
 *      Author: sebas
 */

#ifndef SRC_THERMISTOR_H_
#define SRC_THERMISTOR_H_

float  convertAnalogToTemperature(unsigned int analogReadValue);

float  approximateTemperatureFloat(unsigned int analogReadValue);

int approximateTemperatureInt(unsigned int analogReadValue);

#endif /* SRC_THERMISTOR_H_ */
