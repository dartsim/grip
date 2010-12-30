#ifndef WORLD_H
#define WORLD_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>
#include <queue>
#include <list>
#include <Tools/Collision/VCollide.h>
#include <Tools/Model3DS.h>
#include <Tools/Constants.h>

using namespace std;
using namespace Eigen;
class Object;
class Robot;

struct Entity{
	bool isLink;
	Object* object;
	Model3DS* model;
};

class World{
public:

	World();
	World(World &copyFrom);
	~World();

	bool flag;
	vector<Robot*>	robots;
	vector<Object*> objects;
	vector<Entity>	entities;

	// Collision detection
	VCollide vcollide;
	VCReport report;

	// Camera position
	Matrix3d camRotT;
	Vector3d worldV;
	double camRadius;
	// World colors
	Vector3d gridColor;
	Vector3d backColor;

	void Draw();
	void DeleteModels();
	void updateRobot(Robot* robot);
	int findRobot(string name);
	void updateAllCollisions();
	void updateCollision(Object *ob);
	void detectCollisions();
	void clearCollisions();
	bool checkCollisions();
	void planeDetect();
	void CreateEntity(Object* object, string path, bool link=false);

	int Save(string);
	int Load(string);
};



#endif
