#pragma once

#include "Constants.h"

// skeleton stuff
#include "kinematics/Skeleton.h"
#include "kinematics/BodyNode.h"
#include "kinematics/Joint.h"
#include "kinematics/Dof.h"
#include "kinematics/Transformation.h"
#include "kinematics/TrfmTranslate.h"
#include "kinematics/TrfmRotateExpmap.h"
#include "kinematics/TrfmRotateEuler.h"
#include "kinematics/Shape.h"
#include "kinematics/ShapeEllipsoid.h"
#include "kinematics/ShapeCube.h"
#include "kinematics/ShapeMesh.h"
#include "utils/UtilsRotation.h"
#include "utils/UtilsCode.h"

// planning stuff
#include <planning/Robot.h>
#include <planning/Model3D.h>
#include <planning/ParserUtilities.h>

// XML stuff
//#include <tinyxml2/tinyxml2.h>
#include "C:\Golems\include\grip\tinyxml2\tinyxml2.h"

#include <string>
#include <map>

using namespace std;
using namespace Eigen;
using namespace kinematics;
using namespace planning;
using namespace tinyxml2;

class ParserURDF
{

public:
	
	ParserURDF(void);
	~ParserURDF(void);

	int readURDFFile(const char* const filename, Robot *robot);

	int readLink(XMLElement *link, Robot *robot);
	int readJoint(XMLElement *joint, Robot *robot);
	int readMaterial(XMLElement *material);

	int readTransformMatrix(XMLElement *element, Matrix4d &matrix);

	int createFreeJoint(Joint *joint, Robot *robot);
	int createHingeJoint(Joint *joint, Robot *robot, Vector3d axis,
		double lower = -PI, double upper = PI);
	int createPrismaticJoint(Joint *joint, Robot *robot, Vector3d axis,
		double lower, double upper);

private:

	string dirPath;
	BodyNode *rootNode;
	map<string, BodyNode*> bodyNodes;
	map<string, Joint*> joints;
	map<BodyNode*, Model*> models;

};

