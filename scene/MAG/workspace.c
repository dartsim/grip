#include <stdio.h>
#include <math.h>

#include <stdarg.h>
#define D2R( x ) x*M_PI/180
#define CLOSE( v, t ) fabs(v - t) < 0.001

void calcEE(double T07[][4], double *jVals) {
	T07[0][0] = cos(jVals[5])*(cos(jVals[2])*cos(jVals[4])+sin(jVals[2])*sin(jVals[3])*sin(jVals[4]))-cos(jVals[3])*sin(jVals[2])*sin(jVals[5]);
	T07[0][1] = sin(jVals[6])*(sin(jVals[5])*(cos(jVals[2])*cos(jVals[4])+sin(jVals[2])*sin(jVals[3])*sin(jVals[4]))+cos(jVals[3])*cos(jVals[5])*sin(jVals[2]))-cos(jVals[6])*(cos(jVals[2])*sin(jVals[4])-cos(jVals[4])*sin(jVals[2])*sin(jVals[3]));
	T07[0][2] = cos(jVals[6])*(sin(jVals[5])*(cos(jVals[2])*cos(jVals[4])+sin(jVals[2])*sin(jVals[3])*sin(jVals[4]))+cos(jVals[3])*cos(jVals[5])*sin(jVals[2]))+sin(jVals[6])*(cos(jVals[2])*sin(jVals[4])-cos(jVals[4])*sin(jVals[2])*sin(jVals[3]));
	T07[0][3] = jVals[1]+cos(jVals[2])*(249.0/200.0)+cos(jVals[2])*cos(jVals[4])*(23.0/40.0)+cos(jVals[5])*(cos(jVals[2])*cos(jVals[4])+sin(jVals[2])*sin(jVals[3])*sin(jVals[4]))+sin(jVals[2])*sin(jVals[3])*sin(jVals[4])*(23.0/40.0)-cos(jVals[3])*sin(jVals[2])*sin(jVals[5]);
	T07[1][0] = sin(jVals[3])*sin(jVals[5])+cos(jVals[3])*cos(jVals[5])*sin(jVals[4]);
	T07[1][1] = -sin(jVals[6])*(cos(jVals[5])*sin(jVals[3])-cos(jVals[3])*sin(jVals[4])*sin(jVals[5]))+cos(jVals[3])*cos(jVals[4])*cos(jVals[6]);
	T07[1][2] = -cos(jVals[6])*(cos(jVals[5])*sin(jVals[3])-cos(jVals[3])*sin(jVals[4])*sin(jVals[5]))-cos(jVals[3])*cos(jVals[4])*sin(jVals[6]);
	T07[1][3] = jVals[0]+cos(jVals[3])*sin(jVals[4])*(23.0/40.0)+sin(jVals[3])*sin(jVals[5])+cos(jVals[3])*cos(jVals[5])*sin(jVals[4]);
	T07[2][0] = -cos(jVals[5])*(cos(jVals[4])*sin(jVals[2])-cos(jVals[2])*sin(jVals[3])*sin(jVals[4]))-cos(jVals[2])*cos(jVals[3])*sin(jVals[5]);
	T07[2][1] = -sin(jVals[6])*(sin(jVals[5])*(cos(jVals[4])*sin(jVals[2])-cos(jVals[2])*sin(jVals[3])*sin(jVals[4]))-cos(jVals[2])*cos(jVals[3])*cos(jVals[5]))+cos(jVals[6])*(sin(jVals[2])*sin(jVals[4])+cos(jVals[2])*cos(jVals[4])*sin(jVals[3]));
	T07[2][2] = -cos(jVals[6])*(sin(jVals[5])*(cos(jVals[4])*sin(jVals[2])-cos(jVals[2])*sin(jVals[3])*sin(jVals[4]))-cos(jVals[2])*cos(jVals[3])*cos(jVals[5]))-sin(jVals[6])*(sin(jVals[2])*sin(jVals[4])+cos(jVals[2])*cos(jVals[4])*sin(jVals[3]));
	T07[2][3] = sin(jVals[2])*(-249.0/200.0)-cos(jVals[4])*sin(jVals[2])*(23.0/40.0)-cos(jVals[5])*(cos(jVals[4])*sin(jVals[2])-cos(jVals[2])*sin(jVals[3])*sin(jVals[4]))-cos(jVals[2])*cos(jVals[3])*sin(jVals[5])+cos(jVals[2])*sin(jVals[3])*sin(jVals[4])*(23.0/40.0)+381.0/200.0;
	T07[3][3] = 1.0;
}

void printTrans(double T[][4]){
	printf("% 2.4lf % 2.4lf % 2.4lf % 2.4lf\n% 2.4lf % 2.4lf % 2.4lf % 2.4lf\n% 2.4lf % 2.4lf % 2.4lf % 2.4lf\n% 2.4lf % 2.4lf % 2.4lf % 2.4lf\n",
	T[0][0],T[0][1],T[0][2],T[0][3],
	T[1][0],T[1][1],T[1][2],T[1][3],
	T[2][0],T[2][1],T[2][2],T[2][3],
	T[3][0],T[3][1],T[3][2],T[3][3]);
}

int main (int argc, char const *argv[])
{
	double T[4][4];
	double jVals[7] = {0,0,0,0,0,0,0};
	
	// calcEE(T, jVals);
	// printTrans(T);
	
	double dt = 0.05;

	for (jVals[2] = D2R(-20); jVals[2]<D2R(20); jVals[2]+=dt) {
		for (jVals[3] = D2R(-115); jVals[3]<D2R(115); jVals[3]+=dt) {
			for (jVals[4] = D2R(-90); jVals[4]<D2R(90); jVals[4]+=dt) {
				for (jVals[5] = D2R(-90); jVals[5]<D2R(90); jVals[5]+=dt) {
					for (jVals[6] = D2R(-45); jVals[6]<D2R(45); jVals[6]+=dt) {
						calcEE(T, jVals);
						// Raw transformation matrix
						// printTrans(T);
						// printf("------------------------------\n");
						
						/* Check if we're at any key orientation: up,left,straight
						 * Straight: x=x, y=y, z=z (identity matrix)
						 * Up: x=z, y=y, z=-x
						 * Left: x=y, y=-z, z=z
						 * 
						 * need to check: x=x,y=y,z=z,x=z,x=y,y=-x,z=-x
						 * the logic below looks convoluted because I tried to 
						 * minimize the number of checks
						 */
						
						/* Straight:
						 * [1 0 0;
						 *  0 1 0;
						 *  0 0 1]
						 * left:
						 * [0 -1  0;
						 *  1  0  0;
						 *  0  0  1]
						 * up:
						 * [0  0 -1;
						 *  0  1  0;
						 *  1  0  0]
						 */
						int Y_Y = CLOSE(T[1][1],1); // then y is y
						if (Y_Y) {
							int X_X  = CLOSE(T[0][0],1);  // then x is x
							int Z_Z  = CLOSE(T[2][2],1);  // then z is z
							if (X_X & Z_Z) {
								// straight
								// printf("% 2.4lf % 2.4lf % 2.4lf\n", T[0][3], T[1][3], T[2][3]);
								// printTrans(T);
								continue;
							}
							int X_Z  = CLOSE(T[2][0],1);  // then x is z
							int Z_nX = CLOSE(T[0][2],-1); // then z is -x
							if (X_Z & Z_nX) {
								// up
								// printf("% 2.4lf % 2.4lf % 2.4lf\n", T[0][3], T[1][3], T[2][3]);
								// printTrans(T);
								continue;
							}
						}
						int Z_Z  = CLOSE(T[2][2],1);  // then z is z
						if (Z_Z) {
							int X_X  = CLOSE(T[0][0],1);  // then x is x
							int Z_Z  = CLOSE(T[2][2],1);  // then z is z
							if (X_X & Z_Z){
								// straight
								// printf("% 2.4lf % 2.4lf % 2.4lf\n", T[0][3], T[1][3], T[2][3]);
								// printTrans(T);
								continue;
							}
							int Y_nX = CLOSE(T[0][1],-1); // then y is -x
							int X_Y  = CLOSE(T[1][0],1);  // then x is y
							if (Y_nX & X_Y){
								// left
								printf("% 2.4lf % 2.4lf % 2.4lf\n", T[0][3], T[1][3], T[2][3]);
								//printTrans(T);
								continue;
							}
						}
						//printf("% 2.4lf % 2.4lf % 2.4lf\n", T[0][3], T[1][3], T[2][3]);
						
					}
				}
			}
		}
	}
	
	
	return 0;
}
