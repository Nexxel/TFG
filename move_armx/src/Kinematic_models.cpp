/*
 Calcula el modelo cinemático inverso del robot
*/
#include <math.h>
#include <stdio.h>

using namespace std;

void multiplyTransformations(double result[4][4], double first[4][4], double second[4][4]){
	for (int i  = 0; i < 4; i++){
		for (int j = 0;j < 4; j++){
			result[i][j] += first[i][j] * second[j][i];
		}
	}
}

/*
Modelo cinemático directo
Entradas:
q1, q2, q3, q4, q5: Ángulo de cada articulación

Salidas:
T05: Modelo cinemático directo
pr: Posición del efector final
*/
void mcd(double T05[4][4], double pr[3], double q1, double q2, double q3, double q4, double q5){
	double L3 = 0.1423;
	double d2 = 0.1528;
	double L45 = 0.07585 + 0.04625;
	double beta = 1.2496;

	double alpha[] = {0, -M_PI/2, M_PI, 0, M_PI/2};
	double a[] = {0,0,d2,L3,0};
	double d[] = {0,0,0,0,0};
	double q[] = {q1,q2-beta,q3-beta,q4+(M_PI/2),q5};

	int i  = 0;
	double T01[][4] = {
		{cos(q[i]), -sin(q[i]), 0, a[i]},
		{sin(q[i])*cos(alpha[i]), cos(q[i])*cos(alpha[i]), -0, -0*d[i]},
		{sin(q[i])*0, cos(q[i])*0, cos(alpha[i]), cos(alpha[i])*d[i]},
		{0, 0, 0, 1}
	};

	i  = 1;
	double T12[][4] = {
		{cos(q[i]), -sin(q[i]), 0, a[i]},
		{sin(q[i])*0, cos(q[i])*0, -sin(alpha[i]), -sin(alpha[i])*d[i]},
		{sin(q[i])*sin(alpha[i]), cos(q[i])*sin(alpha[i]), 0, 0*d[i]},
		{0, 0, 0, 1}
	};

	i  = 2;
	double T23[][4] = {
		{cos(q[i]), -sin(q[i]), 0, a[i]},
		{sin(q[i])*cos(alpha[i]), cos(q[i])*cos(alpha[i]), -0, -0*d[i]},
		{sin(q[i])*0, cos(q[i])*0, cos(alpha[i]), cos(alpha[i])*d[i]},
		{0, 0, 0, 1}
	};

	i  = 3;
	double T34[][4] = {
		{cos(q[i]), -sin(q[i]), 0, a[i]},
		{sin(q[i])*cos(alpha[i]), cos(q[i])*cos(alpha[i]), -0, -0*d[i]},
		{sin(q[i])*0, cos(q[i])*0, cos(alpha[i]), cos(alpha[i])*d[i]},
		{0, 0, 0, 1}
	};

	i  = 4;
	double T45[][4] = {
		{cos(q[i]), -sin(q[i]), 0, a[i]},
		{sin(q[i])*0, cos(q[i])*0, -sin(alpha[i]), -sin(alpha[i])*d[i]},
		{sin(q[i])*sin(alpha[i]), cos(q[i])*sin(alpha[i]), 0, 0*d[i]},
		{0, 0, 0, 1}
	};

	multiplyTransformations(T05, T01, T12);
	multiplyTransformations(T05, T05, T23);
	multiplyTransformations(T05, T05, T34);
	multiplyTransformations(T05, T05, T45);

	double ax = T05[0][2];
	double ay = T05[1][2];
	double az = T05[2][2];
	double px = T05[0][3];
	double py = T05[1][3];
	double pz = T05[2][3];

	pr[0] = px + L45*ax;
	pr[1] = py + L45*ay;
	pr[2] = pz + L45*az;
}


/*
Modelo cinemático inverso
Inputs:
pr: Posición del efector final

Outputs:
q: Ángulo de todas las articulaciones;
*/
void mci(double q[5], double pr[3]){

	double L3 = 0.1423;
	double d2 = 0.1528;
	double L45 = 0.07585 + 0.04625;

	int a[3] = {1, 0, 0};
	int n[3] = {0, 0, 1};
	double px = pr[0] - L45*a[0];
	double py = pr[1] - L45*a[1];
	double pz = pr[2] - L45*a[2];

	double q1 = atan2(py, px);

	double k = pow(L3, 2) + pow(px, 2) + pow(d2, 2) + pow((px * cos(q1) + py * sin(q1)), 2);
	double k1 = 2 * d2 * px * cos(q1) + 2 * py * d2 * sin(q1);
	double k2 = 2 * pz * d2;

	double theta2b = atan2(k1, k2) - atan2(k, -sqrt(pow(k1,2)+pow(k2,2)-pow(k,2)));
	double q2 = theta2b + 781/625;

	double theta23 = asin((-pz - d2*sin(theta2b))/L3);
	double q3 = q2 - theta23;

	double L = a[2]*cos(q2-q3) + a[0]*sin(q2-q3)*cos(q1) + a[1]*sin(q2-q3)*sin(q1);
	double q4 = acos(-L) - (M_PI/2);

	double q5 = asin(n[0]*sin(q1) - n[1]*cos(q1));

	q[0] = q1;
	q[1] = q2;
	q[2] = q3;
	q[3] = q4;
	q[4] = q5;
}