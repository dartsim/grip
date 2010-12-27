#include "GLTools.h"

void DrawSphere(double r, int lats, int longs) {
	int i, j;
	for(i = 0; i <= lats; i++) {
		double lat0 = PI * (-0.5f + (double) (i - 1) / lats);
		double z0  = r*sin(lat0);
		double zr0 = cos(lat0);

		double lat1 = PI * (-0.5f + (double) i / lats);
		double z1 = r*sin(lat1);
		double zr1 = cos(lat1);

		glBegin(GL_QUAD_STRIP);
		for(j = 0; j <= longs; j++) {
			double lng = 2.f * PI * (double) (j - 1) / longs;
			double x = r*cos(lng);
			double y = r*sin(lng);

			glNormal3d(x * zr0, y * zr0, z0);
			glVertex3d(x * zr0, y * zr0, z0);
			glNormal3d(x * zr1, y * zr1, z1);
			glVertex3d(x * zr1, y * zr1, z1);
		}
		glEnd();
	}
}