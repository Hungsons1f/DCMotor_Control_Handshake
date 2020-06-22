/*
 * Estimation.h
 *
 *  Created on: 5 Jan 2020
 *      Author: Hungs
 */

#include "main.h"


#ifndef SRC_APP_ESTIMATION_H_
#define SRC_APP_ESTIMATION_H_

void estimate_position( float uk[2], float yk[3], float P_[16], float Theta_[4], float P[16], float Theta[4]);
void estimate_speed(float uk, float yk[2], float P_[4], float Theta_[2], float P[4], float Theta[2]);
void STR_PID_VecController (float Theta[2], float *Kp, float *Ki, float *Kd, float times);
float STR_PosController (float Theta[4], float times, float y0, float y1, float y2, float u1, float u2, float set);
float LPF (float set);
float LPF0_5 (float set);
float LPFs (float input1, float output1);
void ClearLPF ();
void STR_PI_VecController (float Theta[2], float *Kp, float *Ki, float *Kd, float times);
float STR_MRC_PosController (float Theta[4], float uck[2], float yk[2], float uk_);

#endif /* SRC_APP_ESTIMATION_H_ */
