#include "ParserURDF.h"

ParserURDF::ParserURDF(void) :
	rootNode(NULL)
{
}

ParserURDF::~ParserURDF(void)
{
}

int ParserURDF::readURDFFile(const char* const filename, Robot *robot)
{
	string fullPath(filename);
	replace( fullPath.begin(), fullPath.end(), '\\', '/' );
	dirPath = fullPath.substr(0, fullPath.rfind("/") + 1);

	tinyxml2::XMLDocument doc;

	if(doc.LoadFile(filename))
	{
		cout << "Could not load URDF file: " << filename << endl;
		return 0;
	}
	
	cout << "Loaded URDF file: " << filename << endl;

	XMLElement *root = doc.RootElement();

	if(root == NULL)
	{
		cout << "Could not find root node." << endl;
		return 0;
	}

	// Get robot name
	const char *name = root->Attribute("name");
	if (!name)
	{	
		cout << "Could not find robot name." << endl;
		return 0;
	}
	string robotName(name);
	
	// Get links
	for (XMLElement* linkElement = root->FirstChildElement("link");
		linkElement; linkElement = linkElement->NextSiblingElement("link"))
	{
		readLink(linkElement, robot);
	}	

	// Add the world 6-DOF joint (x, y, z) and (r, p, y)
	Joint *worldJoint = new Joint(NULL, rootNode, "worldJoint");

	// Get joints
	for (XMLElement* jointElement = root->FirstChildElement("joint");
		jointElement; jointElement = jointElement->NextSiblingElement("joint"))
	{
		readJoint(jointElement, robot);
	}

	// Get materials
	for (XMLElement* materialElement = root->FirstChildElement("material");
		materialElement; materialElement = materialElement->NextSiblingElement("material"))
	{
		readMaterial(materialElement);
	}

	// add the nodes to the robot
	map<string, BodyNode*>::iterator it;
	for(it = bodyNodes.begin(); it != bodyNodes.end(); it++)
	{
		// add the node to the robot
		cout << "Adding link " << (*it).first << " to robot." << endl;
		robot->addNode((*it).second);
	}

	// Add models to the robot
	map<BodyNode*, Model3D*>::iterator it2;
	for(it2 = models.begin(); it2 != models.end(); it2++)
	{
		// add the node to the robot
		cout << "Adding link " << (*it2).first->getSkelIndex() << " with model " <<
			(*it2).second << " to robot." << endl;
		robot->addModel((*it2).second, (*it2).first->getSkelIndex());
	}

	robot->initSkel();

	return 1;
}

int ParserURDF::readLink(XMLElement *link, Robot *robot)
{
	// get link name
	const char *name = link->Attribute("name");
	if (!name)
	{
		cout << "No name given for the link." << endl;
		return 0;
	}

	string linkName(name);
	cout << "Loading data from link: " << linkName << endl;

	BodyNode *newNode = robot->createBodyNode(name);
	
	if(rootNode == NULL)
	{
		rootNode = newNode;
	}

	Vector3d position(0, 0, 0);
	Vector3d rotation(0, 0, 0);

	Shape *shape = NULL;
	double mass = 0;
	Matrix3d inertiaMatrix = Matrix3d::Identity();

	// inertia data
	XMLElement *inertiaElement = link->FirstChildElement("inertia");
	if (inertiaElement)
	{
		cout << "Link has inertial data. Loading..." << endl;

		XMLElement *originElement = inertiaElement->FirstChildElement("origin");
		if(originElement)
		{
			cout << "Link has an inertial origin element." << endl;

			const char* xyz = originElement->Attribute("xyz");
			if(xyz == NULL)
			{
				cout << "Inertial offset for link " << linkName << " was blank." << endl;
			}
			else
			{	
				stringToVector3d(string(xyz), position);
			}

			const char* rpy = originElement->Attribute("rpy");
			if(rpy == NULL)
			{
				cout << "Inertial rotation for link " << linkName << " was blank." << endl;
			}
			else
			{	
				stringToVector3d(string(rpy), rotation);
			}
		}
		else
		{
			cout << "Link has no inertial origin element. Defaulting to identities." << endl;
		}

		XMLElement *massElement = inertiaElement->FirstChildElement("mass");
		if(originElement)
		{
			cout << "Link has a mass element." << endl;

			const char* m = originElement->Attribute("xyz");
			if(m == NULL)
			{
				cout << "Mass for link " << linkName << " was blank." << endl;
			}
			else
			{	
				char *endChar;
				mass = strtod(m, &endChar);
				if(m == endChar)
				{
					cout << "Error reading mass for link " << linkName << endl;
					mass = 0.0;
				}				
			}
		}
		else
		{
			cout << "Link has no mass element." << endl;
		}

		XMLElement *inertiaElement = inertiaElement->FirstChildElement("inertia");
		if(originElement)
		{
			cout << "Link has an inertia element." << endl;

			double ixx, ixy, ixz, iyy, iyz, izz;

			const char* c = originElement->Attribute("ixx");
			if(c == NULL)
			{
				cout << "Inertia tensor ixx for link " << linkName << " was blank." << endl;
			}
			else
			{	
				char *endChar;
				ixx = strtod(c, &endChar);
				if(c == endChar)
				{
					cout << "Error reading ixx for link " << linkName << endl;
				}				
			}

			c = originElement->Attribute("ixy");
			if(c == NULL)
			{
				cout << "Inertia tensor ixy for link " << linkName << " was blank." << endl;
			}
			else
			{	
				char *endChar;
				ixy = strtod(c, &endChar);
				if(c == endChar)
				{
					cout << "Error reading ixy for link " << linkName << endl;
				}				
			}

			c = originElement->Attribute("ixz");
			if(c == NULL)
			{
				cout << "Inertia tensor ixz for link " << linkName << " was blank." << endl;
			}
			else
			{	
				char *endChar;
				ixz = strtod(c, &endChar);
				if(c == endChar)
				{
					cout << "Error reading ixz for link " << linkName << endl;
				}				
			}

			c = originElement->Attribute("iyy");
			if(c == NULL)
			{
				cout << "Inertia tensor iyy for link " << linkName << " was blank." << endl;
			}
			else
			{	
				char *endChar;
				iyy = strtod(c, &endChar);
				if(c == endChar)
				{
					cout << "Error reading iyy for link " << linkName << endl;
				}				
			}

			c = originElement->Attribute("iyz");
			if(c == NULL)
			{
				cout << "Inertia tensor iyz for link " << linkName << " was blank." << endl;
			}
			else
			{	
				char *endChar;
				iyz = strtod(c, &endChar);
				if(c == endChar)
				{
					cout << "Error reading iyx for link " << linkName << endl;
				}				
			}
						
			c = originElement->Attribute("izz");
			if(c == NULL)
			{
				cout << "Inertia tensor izz for link " << linkName << " was blank." << endl;
			}
			else
			{	
				char *endChar;
				izz = strtod(c, &endChar);
				if(c == endChar)
				{
					cout << "Error reading izz for link " << linkName << endl;
				}				
			}

			// fill inertia matrix
			inertiaMatrix(0, 0) = ixx;
			inertiaMatrix(0, 1) = ixy;
			inertiaMatrix(0, 2) = ixz;
			inertiaMatrix(1, 0) = ixy;
			inertiaMatrix(1, 1) = iyy;
			inertiaMatrix(1, 2) = iyz;
			inertiaMatrix(2, 0) = ixz;
			inertiaMatrix(2, 1) = iyz;
			inertiaMatrix(2, 2) = izz;
		}
		else
		{
			cout << "Link has no inertia element." << endl;
		}

		cout << "Finished loading inertial data." << endl;
	}
	else
	{
		cout << "Link has no inertial data." << endl;
	}

	// visual data
	XMLElement *visualElement = link->FirstChildElement("visual");
	if(visualElement)
	{
		cout << "Link has visual data. Loading..." << endl;
		
		XMLElement *originElement = visualElement->FirstChildElement("origin");
		if(originElement)
		{
			cout << "Link has a visual origin element." << endl;

			const char* xyz = originElement->Attribute("xyz");
			if(xyz == NULL)
			{
				cout << "Visual offset for link " << linkName << " was blank." << endl;
			}
			else
			{	
				stringToVector3d(string(xyz), position);
			}

			const char* rpy = originElement->Attribute("rpy");
			if(rpy == NULL)
			{
				cout << "Visual rotation for link " << linkName << " was blank." << endl;
			}
			else
			{	
				stringToVector3d(string(rpy), rotation);
			}
		}
		else
		{
			cout << "Link has no visual origin element. Defaulting to identities." << endl;
		}

		XMLElement *geometryElement = visualElement->FirstChildElement("geometry");
		if(geometryElement)
		{
			cout << "Link has a geometry element." << endl;

			XMLElement *boxElement = geometryElement->FirstChildElement("box");
			if(boxElement)
			{
				cout << "Box element found." << endl;
				const char* size = boxElement->Attribute("size");
				if(size == NULL)
				{
					cout << "Box specified but no size found." << endl;
				}
				else
				{	
					char *endChar;
					double s = strtod(size, &endChar);
					if(size == endChar)
					{
						cout << "Error reading size of box geometry for link " << linkName << endl;
						s = 1.0;
					}

					shape = new ShapeCube(Vector3d(s, s, s), mass);					
				}
			}
			else
			{
				cout << "No box element found." << endl;
			}

			XMLElement *cylinderElement = geometryElement->FirstChildElement("cylinder");
			if(cylinderElement)
			{
				cout << "Cylinder element found." << endl;

				double radius;
				double length;

				const char* r = cylinderElement->Attribute("radius");
				if(r == NULL)
				{
					cout << "Cylinder specified but no radius found." << endl;
				}
				else
				{	
					char *endChar;
					radius = strtod(r, &endChar);
					if(r == endChar)
					{
						cout << "Error reading radius of cylinder geometry for link " << linkName << endl;
						radius = 1.0;
					}										
				}

				const char* l = cylinderElement->Attribute("length");
				if(l == NULL)
				{
					cout << "Cylinder specified but no length found." << endl;
				}
				else
				{	
					char *endChar;
					length = strtod(l, &endChar);
					if(r == endChar)
					{
						cout << "Error reading length of cylinder geometry for link " << linkName << endl;
						length = 1.0;
					}										
				}

				shape = new ShapeEllipsoid(Vector3d(radius, radius, length), mass);
			}
			else
			{
				cout << "No cylinder element found." << endl;
			}

			XMLElement *sphereElement = geometryElement->FirstChildElement("sphere");
			if(sphereElement)
			{
				cout << "Sphere element found." << endl;
				const char* size = boxElement->Attribute("radius");
				if(size == NULL)
				{
					cout << "Sphere specified but no radius found." << endl;
				}
				else
				{	
					char *endChar;
					double radius = strtod(size, &endChar);
					if(size == endChar)
					{
						cout << "Error reading size of box geometry for link " << linkName << endl;
						radius = 1.0;
					}

					shape = new ShapeEllipsoid(Vector3d(radius, radius, radius), mass);					
				}
			}
			else
			{
				cout << "No Sphere element found." << endl;
			}

			Model3D* model = NULL;

			XMLElement *meshElement = geometryElement->FirstChildElement("mesh");
			if(meshElement)
			{
				cout << "Mesh element found." << endl;
				const char* filename = meshElement->Attribute("filename");
				if(filename == NULL)
				{
					cout << "Mesh specified but no filename found." << endl;
					model = 0;
				}
				else
				{	
					string p(dirPath);
					p.append(filename);
					cout << "Loading mesh: " << p << " for link: " << linkName << endl; 
					model = robot->loadModel(p);
				}
			}
			else
			{
				cout << "No mesh element found." << endl;
				model = 0;
			}

			if(model != NULL)
			{
				models[newNode] = model;
			}

			shape = new ShapeMesh( Vector3d(0, 0, 0), mass);
		}
		else
		{
			cout << "Link has no geometry element." << endl;
		}

		XMLElement *materialElement = visualElement->FirstChildElement("material");
		if(materialElement)
		{
			cout << "Link has a material element." << endl;

			const char* name = materialElement->Attribute("name");
			Vector4d rgba(1, 1, 1, 0);

			XMLElement *colorElement = materialElement->FirstChildElement("color");
			if(colorElement)
			{
				cout << "Color element found." << endl;
				const char* color = colorElement->Attribute("rgba");
				if(color == NULL)
				{
					cout << "Color specified but was blank." << endl;
				}
				else
				{	
					stringToVector4d(string(color), rgba);
				}
			}
			else
			{
				cout << "No color element found." << endl;
			}

			XMLElement *textureElement = materialElement->FirstChildElement("texture");
			if(colorElement)
			{
				cout << "Texture element found." << endl;
				const char* textureFileName = colorElement->Attribute("filename");
				if(textureFileName == NULL)
				{
					cout << "Texture specified but was blank." << endl;
				}
				else
				{	
					string p(dirPath);
					p.append(textureFileName);
				}
			}
			else
			{
				cout << "No texture element found." << endl;
			}
		}
		else
		{
			cout << "Link has no material element." << endl;
		}		
		
		if(shape != NULL)
		{
			shape->setInertia(inertiaMatrix);
			newNode->setShape(shape);
		}

		cout << "Finished loading visual data." << endl;
	}
	else
	{
		cout << "Link has no visual data." << endl;
	}
    
	// collision data
	XMLElement *collisionElement = link->FirstChildElement("collision");
	if(collisionElement)
	{
		cout << "Link has inertial data. Loading..." << endl;
		

		cout << "Finished loading collision data." << endl;
	}
	else
	{
		cout << "Link has no collision data." << endl;
	}

	cout << "Finished loading data from link: " << linkName << endl;

	// add the bodynode to our node list
	bodyNodes[name] = newNode;

	return 1;

}

int ParserURDF::readJoint(XMLElement *joint, Robot *robot)
{
	// Get Joint Name
	const char *buff = joint->Attribute("name");
	if (!buff)
	{
		cout << "Joint has no name." << endl;
		return 0;
	}
	string jointName(buff);
	cout << "Loading data from joint: " << jointName << endl;

	// Get transform from data
	Matrix4d transformMatrix = Matrix4d::Identity();
	Vector3d position(0, 0, 0);
	Vector3d orientation(0, 0, 0);

	XMLElement *originElement = joint->FirstChildElement("origin");
	if (!originElement)
	{
		cout << "Joint has no origin, using identity transform." << endl;
	}
	else
	{		
		const char* xyz = originElement->Attribute("xyz");
		if(xyz == NULL)
		{
			cout << "No xyz values. Using empty vector." << endl;
		}
		else
		{
			stringToVector3d(string(xyz), position);
		}

		Vector3d orientation(0, 0, 0);
		const char* rpy = originElement->Attribute("rpy");
		if(rpy == NULL)
		{
			cout << "No rpy values. Using empty vector." << endl;
		}
		else
		{
			stringToVector3d(string(rpy), orientation);
		}
	}

	// Get Parent Link
	string parentName("");
	XMLElement *parentElement = joint->FirstChildElement("parent");
	if (parentElement)
	{
		buff = parentElement->Attribute("link");
		if (!buff)
		{
			cout << "No parent link name specified for Joint link " << jointName << endl;
		}
		else
		{
			parentName = std::string(buff);
		}
	}

	// Get Child Link
	string childName("");
	XMLElement *childElement = joint->FirstChildElement("child");
	if(childElement)
	{
		buff = childElement->Attribute("link");
		if (!buff)
		{
			cout << "No child link name specified for Joint link " << jointName << endl;
		}
		else
		{
			childName = std::string(buff);
		}
	}

	// Create joint
	BodyNode *parent;
	if(parentName == "")
	{
		parent = robot->getRoot();
	}
	else
	{
		parent = bodyNodes[parentName.c_str()];
	}

	BodyNode *child = bodyNodes[childName.c_str()];

	if(parent == NULL)
	{
		cout << "Joint " << jointName << " has no parent link." << endl;
	}
	if(child == NULL)
	{
		cout << "Joint " << jointName << " has no child link." << endl;
	}

	cout << "Creating joint with parent: " << parentName.c_str() << " and child: " <<
		childName.c_str() << endl;

	Joint *newJoint = new Joint(parent, child, jointName.c_str());

	// Add the transform for translation
	Transformation * translation = new TrfmTranslate( 
		new kinematics::Dof(position[0]),
		new kinematics::Dof(position[1]),
		new kinematics::Dof(position[2]),
		"Translation" );
	newJoint->addTransform(translation, false);

	// Add the transforms for rotation
	Transformation * rotationX = new TrfmRotateEulerX(new Dof(DEG2RAD(orientation[0]), "RotationX"));
    newJoint->addTransform(rotationX, false);

	Transformation * rotationY = new TrfmRotateEulerY(new Dof(DEG2RAD(orientation[1]), "RotationY"));
    newJoint->addTransform(rotationY, false);

	Transformation * rotationZ = new TrfmRotateEulerZ(new Dof(DEG2RAD(orientation[2]), "RotationZ"));
    newJoint->addTransform(rotationZ, false);

	// Get Joint type
	buff = joint->Attribute("type");
	if (!buff)
	{
		cout << "Joint " << jointName << " has no type." << endl;
		return 0;
	}
	std::string jointTypeStr = buff;
	
	Joint::JointType jointType;
	if (jointTypeStr == "planar") // NA - not supported
	{
		jointType = Joint::J_UNKNOWN;
		cout << "Joint " << jointName << " is of type J_UNKNOWN" << endl;
	}
	else if (jointTypeStr == "floating") // J_FREEEXPMAP - free
	{
		jointType = Joint::J_FREEEXPMAP;
		cout << "Joint " << jointName << " is of type J_FREEEXPMAP" << endl;
	}
	else if (jointTypeStr == "revolute") // J_HINGE (limits) - hinge joint
	{
		jointType = Joint::J_HINGE;
		cout << "Joint " << jointName << " is of type J_HINGE (limited)" << endl;
	}
	else if (jointTypeStr == "continuous") // J_HINGE (no limits) - hinge joint
	{
		jointType = Joint::J_HINGE;
		cout << "Joint " << jointName << " is of type J_HINGE (continuous)" << endl;
	}
	else if (jointTypeStr == "prismatic") // J_TRANS - translation
	{
		jointType = Joint::J_TRANS;
		cout << "Joint " << jointName << " is of type J_TRANS" << endl;
	}
	else if (jointTypeStr == "fixed") // NA - not supported
	{
		jointType = Joint::J_UNKNOWN;
		cout << "Joint " << jointName << " is of type J_UNKNOWN" << endl;
	}
	else
	{
		jointType = Joint::J_UNKNOWN; // unknown type	
		cout << "Joint " << jointName << " is of type J_UNKNOWN" << endl;
	}

	// Get Joint Axis
	Vector3d axis(0, 0, 0);

	// axis
	XMLElement *axisElement = joint->FirstChildElement("axis");
	if (!axisElement)
	{
		cout << "No axis element for Joint link " << jointName << " defaulting to (1,0,0) axis." << endl;
		axis = Vector3d(1.0, 0.0, 0.0);
	}
	else
	{
		if (!axisElement->Attribute("xyz"))
		{
			cout << "No xyz attribute for axis element for Joint link " << jointName << endl;
		}
		else
		{
			const char *axisXYZ = axisElement->Attribute("xyz");
			if (!axisXYZ)
			{
				cout << "Malformed axis element for joint " << jointName << endl;					
				return 0;
			}
			else
			{
				stringToVector3d(string(axisXYZ), axis);
			}			
		}
	}

	// Joint limits
	bool foundLower = false;
	bool foundUpper = false;
	double lower = 0.0;
	double upper = 0.0;

	// Get limit
	XMLElement *limitElement = joint->FirstChildElement("limit");
	if (limitElement)
	{
		// Get lower joint limit
		buff = limitElement->Attribute("lower");
		if (buff == NULL)
		{
			cout << "Joint " << jointName << " has no lower limit, defaults to 0.0" << endl;
		}
		else
		{
			char *endChar;
			lower = strtod(buff, &endChar);
			if(buff == endChar)
			{
				cout << "Error reading upper limit for joint " << jointName << endl;
			}
			else
			{
				foundLower = true;
			}
		}

		// Get upper joint limit
		buff = limitElement->Attribute("upper");
		if (buff == NULL)
		{
			cout << "Joint " << jointName << " has no upper limit, defaults to 0.0" << endl;
		}
		else
		{
			char *endChar;
			upper = strtod(buff, &endChar);
			if(buff == endChar)
			{
				cout << "Error reading upper limit for joint " << jointName << endl;
			}
			else
			{
				foundUpper = true;
			}
		}

	//	// Get joint effort limit
	//	const char* effort_str = config->Attribute("effort");
	//	if (effort_str == NULL){
	//	ROS_ERROR("joint limit: no effort");
	//	return false;
	//	}
	//	else
	//	{
	//	try
	//	{
	//	this->effort = boost::lexical_cast<double>(effort_str);
	//	}
	//	catch (boost::bad_lexical_cast &e)
	//	{
	//	ROS_ERROR("effort value (%s) is not a float",effort_str);
	//	return false;
	//	}
	//	}

	//	// Get joint velocity limit
	//	const char* velocity_str = config->Attribute("velocity");
	//	if (velocity_str == NULL){
	//	ROS_ERROR("joint limit: no velocity");
	//	return false;
	//	}
	//	else
	//	{
	//	try
	//	{
	//	this->velocity = boost::lexical_cast<double>(velocity_str);
	//	}
	//	catch (boost::bad_lexical_cast &e)
	//	{
	//	ROS_ERROR("velocity value (%s) is not a float",velocity_str);
	//	return false;
	//	}
	//}
	//else if (this->type == REVOLUTE)
	//{
	//	ROS_ERROR("Joint '%s' is of type REVOLUTE but it does not specify limits", this->name.c_str());
	//	return false;
	//}
	//else if (this->type == PRISMATIC)
	//{
	//	ROS_INFO("Joint '%s' is of type PRISMATIC without limits", this->name.c_str());
	//	limits.reset();
	//}

	//// Get safety
	//XMLElement *safety_xml = config->FirstChildElement("safety_controller");
	//if (safety_xml)
	//{
	//	safety.reset(new JointSafety);
	//	if (!safety->initXml(safety_xml))
	//	{
	//		ROS_ERROR("Could not parse safety element for joint '%s'", this->name.c_str());
	//		safety.reset();
	//		return false;
	//	}
	//}

	//// Get calibration
	//XMLElement *calibration_xml = config->FirstChildElement("calibration");
	//if (calibration_xml)
	//{
	//	calibration.reset(new JointCalibration);
	//	if (!calibration->initXml(calibration_xml))
	//	{
	//		ROS_ERROR("Could not parse calibration element for joint  '%s'", this->name.c_str());
	//		calibration.reset();
	//		return false;
	//	}
	//}

	//// Get Joint Mimic
	//XMLElement *mimic_xml = config->FirstChildElement("mimic");
	//if (mimic_xml)
	//{
	//	mimic.reset(new JointMimic);
	//	if (!mimic->initXml(mimic_xml))
	//	{
	//		ROS_ERROR("Could not parse mimic element for joint  '%s'", this->name.c_str());
	//		mimic.reset();
	//		return false;
	//	}
	//}

	//// Get Dynamics
	//XMLElement *prop_xml = config->FirstChildElement("dynamics");
	//if (prop_xml)
	//{
	//	dynamics.reset(new JointDynamics);
	//	if (!dynamics->initXml(prop_xml))
	//	{
	//		ROS_ERROR("Could not parse joint_dynamics element for joint '%s'", this->name.c_str());
	//		dynamics.reset();
	//		return false;
	//	}
	//}

	}

	// If joint has no axis
	//if (jointTypeStr != "floating" && jointTypeStr != "fixed")
	if(jointType == Joint::J_FREEEXPMAP)
	{
		createFreeJoint(newJoint, robot);
	}
	// We should have read axis for the joint
	else
	{
		if (jointType == Joint::J_HINGE)  
		{
			if(jointTypeStr == "revolute")
			{
				if(!foundLower || !foundUpper)
				{
					cout << "Joint " << jointName << " was specified as a revolute joint but no " <<
						"upper or lower limits were found." << endl;
				}
				createHingeJoint(newJoint, robot, axis, lower, upper);
			}	
			else if (jointTypeStr == "continuous")
			{
				if(!foundLower || !foundUpper)
				{
					cout << "Joint " << jointName << " was specified as a continuous joint but " <<
						"an upper or lower limit was found." << endl;
				}
				createHingeJoint(newJoint, robot, axis);
			}
		}
		else if (jointType == Joint::J_TRANS)
		{
			if(!foundLower || !foundUpper)
			{
				cout << "Joint " << jointName << " was specified as a prismatic joint but no " <<
					"upper or lower limits were found." << endl;
			}
			createPrismaticJoint(newJoint, robot, axis, lower, upper);
		}
		else
		{
			cout << "Joint " << jointName << " has an unknown or unsupported joint with an axis." << endl;
		}		
	}
	
	// Add joint to robot
	//robot->addJoint(newJoint);

	cout << "Finished loading data from joint: " << jointName << endl;

	return 1;
}

int ParserURDF::readMaterial(XMLElement *material)
{
	return 1;
}

int ParserURDF::readTransformMatrix(XMLElement *element, Matrix4d &matrix)
{
	if(!element)
	{
		cout << "Transform matrix is empty." << endl;
		return 0;
	}

	Vector3d position(0, 0, 0);
	const char* xyz = element->Attribute("xyz");
	if(xyz == NULL)
	{
		cout << "No xyz values. Using empty vector." << endl;
	}
	else
	{
		stringToVector3d(string(xyz), position);
	}

	Vector3d orientation(0, 0, 0);
	const char* rpy = element->Attribute("rpy");
	if(rpy == NULL)
	{
		cout << "No rpy values. Using empty vector." << endl;
	}
	else
	{
		stringToVector3d(string(rpy), orientation);
	}

	matrix = buildTransformationMatrix(orientation, position);

	return 1;
}

int ParserURDF::createFreeJoint(Joint *joint, Robot *robot)
{
	// create new transformation
    string tname1 = string(joint->getName()) + "_t";
    string tname1_0 = tname1 + "Free0";
    string tname1_1 = tname1 + "Free1";
    string tname1_2 = tname1 + "Free2";
    vector<Dof*> dofs1;
    dofs1.resize(3);
    dofs1[0] = new Dof(0.0, tname1_0.c_str(), -100.0, 100.0);
    dofs1[1] = new Dof(0.0, tname1_1.c_str(), -100.0, 100.0);
    dofs1[2] = new Dof(0.0, tname1_2.c_str(), -100.0, 100.0);

    // add transformation to joint
    TrfmTranslate* trans = new TrfmTranslate(dofs1[0], dofs1[1], dofs1[2], tname1.c_str()); 
    joint->addTransform(trans);	
    // add transformation to model because it's a variable dof
    robot->addTransform(trans);

    string tname2 = string(joint->getName()) + "_a";
    string tname2_0 = tname2 + "Free3";
    string tname2_1 = tname2 + "Free4";
    string tname2_2 = tname2 + "Free5";
    vector<Dof*> dofs2;
    dofs2.resize(3);
    dofs2[0] = new Dof(0.0, tname2_0.c_str(), -3.1415, 3.1415);
    dofs2[1] = new Dof(0.0, tname2_1.c_str(), -3.1415, 3.1415);
    dofs2[2] = new Dof(0.0, tname2_2.c_str(), -3.1415, 3.1415);

    // add transformation to joint
    TrfmRotateExpMap* expmap= new TrfmRotateExpMap(dofs2[0], dofs2[1], dofs2[2], tname2.c_str());
    joint->addTransform(expmap);	
    // add transformation to model because it's a variable dof
    robot->addTransform(expmap);

	return 1;
}

int ParserURDF::createHingeJoint(Joint *joint, Robot *robot, Vector3d axis,
								 double lower, double upper)
{
	string tname = string(joint->getName()) + "_a";
    tname += "Hinge0";
    const char* pTname = tname.c_str();

    // Read axes data
    Transformation *r1 = NULL;

	axis.normalize();

	// *Note - assuming only X, Y, or Z axis...

	// X-Axis
    if((axis - Vector3d(1.0, 0.0, 0.0)).norm() < 0.01)
	{
        r1 = new TrfmRotateEulerX(new Dof(0.0, pTname, lower, upper));
    }
	// Y-Axis
    else if((axis - Vector3d(0.0, 1.0, 0.0)).norm() < 0.01)
	{
        r1 = new TrfmRotateEulerY(new Dof(0.0, pTname, lower, upper));
    }
	// Z-Axis
    else if((axis - Vector3d(0.0, 0.0, 1.0)).norm() < 0.01)
	{
        r1 = new TrfmRotateEulerZ(new Dof(0.0, pTname, lower, upper)); 
    }

    if(r1 == NULL)
	{
		cout << "Error creating hinge joint for Joint " << joint->getName() << endl;
		return 0;
	}

    joint->addTransform(r1);	
    robot->addTransform(r1);

	return 1;
}

int ParserURDF::createPrismaticJoint(Joint *joint, Robot *robot, Vector3d axis,
									 double lower, double upper)
{
	string tname = string(joint->getName()) + "_t";
    tname += "Prismatic0";
    const char* pTname = tname.c_str();

	axis.normalize();

	// *Note - assuming only X, Y, or Z axis...

	Dof* dofX = new Dof(0.0, "DOFX", 0.0, 0.0);
    Dof* dofY = new Dof(0.0, "DOFY", 0.0, 0.0);
    Dof* dofZ = new Dof(0.0, "DOFX", 0.0, 0.0);

	// X-Axis
    if((axis - Vector3d(1.0, 0.0, 0.0)).norm() < 0.01)
	{
        dofX->setMin(lower);
		dofX->setMax(upper);
    }
	// Y-Axis
    else if((axis - Vector3d(0.0, 1.0, 0.0)).norm() < 0.01)
	{
        dofY->setMin(lower);
		dofY->setMax(upper);
    }
	// Z-Axis
    else if((axis - Vector3d(0.0, 0.0, 1.0)).norm() < 0.01)
	{
        dofZ->setMin(lower);
		dofZ->setMax(upper);
    }

	TrfmTranslate* t = new TrfmTranslate(dofX, dofY, dofX, pTname);

    if(t == NULL)
	{
		cout << "Error creating prismatic joint for Joint " << joint->getName() << endl;
		return 0;
	}

    joint->addTransform(t);	
    robot->addTransform(t);

	return 1;
}
