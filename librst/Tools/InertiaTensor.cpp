#include "InertiaTensor.h"

InertiaTensor::InertiaTensor(double x_x, double x_y, double x_z, double y_x, double y_y, double y_z, double z_x, double z_y, double z_z)
{
//	Tensor.e(0,0)=x_x;
//	Tensor.e(0,1)=x_y;
//	Tensor.e(0,2)=x_z;
//
//	Tensor.e(1,0)=y_x;
//	Tensor.e(1,1)=y_y;
//	Tensor.e(1,2)=y_z;
//
//	Tensor.e(2,0)=z_x;
//	Tensor.e(2,1)=z_y;
//	Tensor.e(2,2)=z_z;
}

solidSphereTensor::solidSphereTensor(double mass, double radius):
InertiaTensor((0.4*mass*radius*radius),0.0,0.0
			  ,0.0,(0.4*mass*radius*radius),0.0
			  ,0.0,0.0,(0.4*mass*radius*radius))
{}

