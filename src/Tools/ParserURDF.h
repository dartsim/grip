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
#include <planning/Model3DS.h>

// XML stuff
#include <ticpp/ticpp.h>

#include <string>
#include <map>

#include "ParserUtilities.h"

using namespace std;
using namespace Eigen;
using namespace kinematics;
using namespace planning;

class ParserURDF
{

public:
	
	ParserURDF(void);
	~ParserURDF(void);

	int readURDFFile(const char* const filename, Robot *robot);

	int readLink(TiXmlElement *link, Robot *robot);
	int readJoint(TiXmlElement *joint, Robot *robot);
	int readMaterial(TiXmlElement *material);

	int readTransformMatrix(TiXmlElement *element, Matrix4d &matrix);

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
	map<BodyNode*, Model3DS*> models;

};

