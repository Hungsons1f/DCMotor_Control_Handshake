/*
 * Estimation.c
 *
 *  Created on: 5 Jan 2020
 *      Author: Hungs
 */

#include "Estimation.h"
#include <math.h>


/*
 * Arguments    : const double uk[2]
 *                const double yk[3]
 *                const double P_[16]
 *                const double Theta_[4]
 *                double P[16]
 *                double Theta[4]
 * Return Type  : void
 */
void estimate_position( float uk[2], float yk[3], float P_[16], float Theta_[4], float P[16], float Theta[4])
{
  float PHI[4];
  float d0;
  int i0;
  float e;
  float y;
  float b_y;
  float b_PHI[4];
  int i1;
  float b_P_[16];
  float c_P_[16];
  int i2;
  float lamda = 0.9;

  /*  Sinh vien viet bo sung doan code nhan dang */
  PHI[0] = -yk[1];
  PHI[1] = -yk[2];
  PHI[2] = uk[0];
  PHI[3] = uk[1];
  d0 = 0.0;
  for (i0 = 0; i0 < 4; i0++) {
    d0 += PHI[i0] * Theta_[i0];
  }

  e = yk[0] - d0;
  y = 0.0;
  for (i0 = 0; i0 < 4; i0++) {
    b_PHI[i0] = 0.0;
    for (i1 = 0; i1 < 4; i1++) {
      b_PHI[i0] += PHI[i1] * P_[i1 + (i0 << 2)];
    }

    y += b_PHI[i0] * PHI[i0];
  }

  b_y = 0.0;
  for (i0 = 0; i0 < 4; i0++) {
    b_PHI[i0] = 0.0;
    for (i1 = 0; i1 < 4; i1++) {
      b_PHI[i0] += PHI[i1] * P_[i1 + (i0 << 2)];
    }

    b_y += b_PHI[i0] * PHI[i0];
  }

  for (i0 = 0; i0 < 4; i0++) {
    b_PHI[i0] = 0.0;
    for (i1 = 0; i1 < 4; i1++) {
      b_PHI[i0] += P_[i0 + (i1 << 2)] * PHI[i1];
    }

    for (i1 = 0; i1 < 4; i1++) {
      c_P_[i0 + (i1 << 2)] = b_PHI[i0] * PHI[i1];
    }

    for (i1 = 0; i1 < 4; i1++) {
      d0 = 0.0;
      for (i2 = 0; i2 < 4; i2++) {
        d0 += c_P_[i0 + (i2 << 2)] * P_[i2 + (i1 << 2)];
      }

      b_P_[i0 + (i1 << 2)] = P_[i0 + (i1 << 2)] - d0 / (lamda + b_y);
    }
  }

  for (i0 = 0; i0 < 4; i0++) {
    d0 = 0.0;
    for (i1 = 0; i1 < 4; i1++) {
      P[i1 + (i0 << 2)] = (1/lamda) * b_P_[i1 + (i0 << 2)];
      d0 += P_[i0 + (i1 << 2)] * PHI[i1];
    }

    Theta[i0] = Theta_[i0] + d0 / (lamda + y) * e;
  }
}

/*
 * Arguments    : double uk
 *                const double yk[2]
 *                const double P_[4]
 *                const double Theta_[2]
 *                double P[4]
 *                double Theta[2]
 * Return Type  : void
 */
void estimate_speed(float uk, float yk[2], float P_[4], float Theta_[2], float P[4], float Theta[2])
{
  float PHI[2];
  float d0;
  int i0;
  float e;
  float y;
  float b_y;
  float b_PHI[2];
  int i1;
  float b_P_[4];
  float c_P_[4];
  int i2;
  float lamda = 0.999;
  PHI[0] = -yk[1];
  PHI[1] = uk;
  d0 = 0.0;
  for (i0 = 0; i0 < 2; i0++) {
    d0 += PHI[i0] * Theta_[i0];
  }

  e = yk[0] - d0;
  y = 0.0;
  for (i0 = 0; i0 < 2; i0++) {
    b_PHI[i0] = 0.0;
    for (i1 = 0; i1 < 2; i1++) {
      b_PHI[i0] += PHI[i1] * P_[i1 + (i0 << 1)];
    }

    y += b_PHI[i0] * PHI[i0];
  }

  b_y = 0.0;
  for (i0 = 0; i0 < 2; i0++) {
    b_PHI[i0] = 0.0;
    for (i1 = 0; i1 < 2; i1++) {
      b_PHI[i0] += PHI[i1] * P_[i1 + (i0 << 1)];
    }

    b_y += b_PHI[i0] * PHI[i0];
  }

  for (i0 = 0; i0 < 2; i0++) {
    b_PHI[i0] = 0.0;
    for (i1 = 0; i1 < 2; i1++) {
      b_PHI[i0] += P_[i0 + (i1 << 1)] * PHI[i1];
    }

    for (i1 = 0; i1 < 2; i1++) {
      c_P_[i0 + (i1 << 1)] = b_PHI[i0] * PHI[i1];
    }

    for (i1 = 0; i1 < 2; i1++) {
      d0 = 0.0;
      for (i2 = 0; i2 < 2; i2++) {
        d0 += c_P_[i0 + (i2 << 1)] * P_[i2 + (i1 << 1)];
      }

      b_P_[i0 + (i1 << 1)] = P_[i0 + (i1 << 1)] - d0 / (lamda + b_y);
    }
  }

  for (i0 = 0; i0 < 2; i0++) {
    d0 = 0.0;
    for (i1 = 0; i1 < 2; i1++) {
      P[i1 + (i0 << 1)] = 1.0101010101010102 * b_P_[i1 + (i0 << 1)];
      d0 += P_[i0 + (i1 << 1)] * PHI[i1];
    }

    Theta[i0] = Theta_[i0] + d0 / (lamda + y) * e;
  }
}




	float xi = 1;
	float wn = 20;
	extern float anpha;
	extern float beta;
float STR_PosController (float Theta[4], float times, float y0, float y1, float y2, float u1, float u2, float set)
{
	//float T0 = times;
	/*float d1 = -2*exp(-xi*wn*T0)*cos(wn*T0*sqrt(1-xi^2));
	float d2 = exp(-2*xi*wn*T0);
	float r1 = (b1+b2)*(a1*b1*b2-a2*b1*b1-b2*b2);
	float s1 = a2*((b1+b2)*(a1*b2-a2*b1)+b2*(b1*d2-b2*d1-b2));
	float q2 = s1/r1;
	float gama = q2*b2/a2;
	float q1 = a2/b2 - q2*(b1/b2 - a1/a2 +1);
	float q0 = 1/b1*(d1+1-a1-gama);

	u = q0*e(1) + q1*e(2) + q2*e(3) + (1-gama)*u_(1) + gama*u_(2);*/


	//   *** Dat cuc mien roi rac ***
	//float anpha = 0;//0.724;
	//float beta = -0.9;//0.210;

	float a1 = Theta[0];
	float a2 = Theta[1];
	float b1 = Theta[2];
	float b2 = Theta[3];
	float x1 = -4*anpha + 1 - a1;
	float x2 = 6*anpha*anpha + beta*beta + a1 - a2;
	float x3 = 2*anpha*(2*anpha*anpha+beta*beta)-a2;
	float x4 = anpha*anpha*(anpha*anpha+beta*beta);

	float r0 = (x1+x2-x3+x4)/(b1+b2);
	float r1 = (b1+b2)*(a1*b1*b2-a2*b1*b1-b2*b2);
	float r2 = a1*b2*(b1*(x2-x3+x4)-b2*x1);
	float r3 = a2*b1*(b2*x1 - b1*(x2 - x3 + x4));
	float r4 = (b1+b2)*(b1*x4 + b2*(x3-x4));
	float r5 = b1*(x4*b1*b1 + b1*b2*x3 +x2*b2*b2)-x1*b2*b2*b2;
	float r6 = b1*b1 *(a2*x3 + a1*x4 - a2*x4);
	float r7 = b2*(b1*(a1*x4 + a2*x2 - x4) - b2*(a2*x1+x4));

	float p1 = (r5 - r1)/r1;
	float p2 = -r5/r1;
	float q0 = r0 - (r2+r3+r4)/r1;
	float q1 = (r2 + r3 + r4 - r6 -r7)/r1;
	float q2 = (r6+r7)/r1;

	float u = r0*set-q0*y0-q1*y1-q2*y2-p1*u1 -p2*u2;
	return u;
}



	float set1 = 0;
	float outLPF1 = 0;
float LPF (float set)
{
	float temp;
	temp  = 0.1813*set1 + 0.8187*outLPF1;
	set1 = set;
	outLPF1 = temp;
	return temp;
}

float LPF0_5 (float set)
{
	float temp;
	temp  = 0.0392*set1 + 0.9608*outLPF1;
	set1 = set;
	outLPF1 = temp;
	return temp;
}

float LPFs (float input1, float output1)
{
	float temp;
	temp  = 0.7154*input1 + 0.2846*output1;
	return temp;
}

void ClearLPF ()
{
	set1 = 0;
	outLPF1 = 0;
}

	extern float anphav;
	extern float betav;
void STR_PI_VecController (float Theta[2], float *Kp, float *Ki, float *Kd, float times)
{
	float b=Theta[0];
	float a=Theta[1];
	float T=times;
	float k0= anphav*anphav + betav*betav; //0.568;
	float k1= 2*anphav;//-1.447;
	*Kp= -(2*b + k0 - k1 - 1)/(2*a);
	*Ki= (k0 + k1 + 1)/(T*a);
	*Kd= 0;
}

void STR_PID_VecController (float Theta[2], float *Kp, float *Ki, float *Kd, float times)
{
	float b = Theta[0];
	float a = Theta[1];
	float k2 = 0.17;
	float k1 = 0.84;
	float k0 = 1.5;
	float T = times;
	*Kp = (-(2*b + 3*k0 + k1 - k2 - 1)/(2*a));
	*Ki = ((k0 + k1 + k2 + 1)/(T*a));
	*Kd= ((T*k0)/a);
}


	extern float paraK0;
	extern float paraK1;
	extern float paraK2;
	extern float paraK3;
float STR_MRC_PosController (float Theta[4], float uck[2], float yk[2], float uk_)
{
	float a1 = Theta[0];//-1.3978;//Theta[0];
	float a2 = Theta[1];//0.3797;//Theta[1];
	float b1 = Theta[2];//6.1830;//Theta[2];
	float b2 = Theta[3];//0.253;//Theta[3];
	//float k0=0.01967;//0.1348;//0.1302;//0.06452;
	//float k1=0.01941;//0.1017;//0.09442;//0.0521;
	//float k2=-1.922;//-1.195;//-1.158;//-1.411;
	//float k3=0.9608;//0.4317;//0.3829;//0.5273;
	float uktemp = - uk_*b2/b1 + (a1 - paraK2)/b1*yk[0] + (a2 - paraK3)/b1*yk[1] + paraK0/b1*uck[0] + paraK1/b1*uck[1];
	return uktemp;
}

