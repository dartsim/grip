#ifndef OBJECT_H
#define OBJECT_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Tools/Model3DS.h>
#include <Tools/Constants.h>
#include <string>

//Defined here are the different geometric primitives
enum{ NONE,SPHERE,BOX,RECT_FACE };

extern const double threshold;

class World;

class Object{
public:
	Eigen::Transform<double, 3, Eigen::Affine> absPose;
	Eigen::Matrix<double, 3, 3>	inertia;
	Eigen::Vector3d	COM;
	Eigen::Vector3d vertices[8]; //for Box(8) and Rect_Face(4) in object coordinates
	Eigen::Vector3d normal;

	World* world;
	Model3DS	*model;

	string pathname;
	string name;
	bool movable;
	int idNum;
	int eid;
	double mass;
	int primitiveType; // geometric primitive type
	double radius; //for sphere
	double d;
	double vel_t;
	bool collisionFlag;
	bool comFlag;

	Object();
	Object(Object&);
	~Object();

	Model3DS* LoadModel(string);

	void Draw();
	void DrawPrimitive(); //will draw the primitive object
};



#endif
