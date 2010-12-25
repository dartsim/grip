#pragma once
//#include "Transform.h"

class InertiaTensor{
public:
	InertiaTensor(double x_x, double x_y, double x_z, double y_x, double y_y, double y_z, double z_x, double z_y, double z_z);
	//Mat33 getInertiaTensor();
	//Mat33 Tensor;
};

class solidSphereTensor : public InertiaTensor{
public:
	solidSphereTensor(double mass, double radius);
};
