#include <Tools/Link.h>
#include <Tools/Robot.h>
#include <Tools/World.h>

using namespace Eigen;

Link::Link()
{
}

Link::~Link()
{
}

Link::Link(Link &copyFrom): Object(copyFrom)
{
	this->absPose = copyFrom.absPose;
	this->COM = copyFrom.COM;
	this->idNum = copyFrom.idNum;
	this->inertia = copyFrom.inertia;
	this->jAxis = copyFrom.jAxis;
	this->jMax = copyFrom.jMax;
	this->jMin = copyFrom.jMin;
	this->jTrans = copyFrom.jTrans;
	this->jType = copyFrom.jType;
	this->jVal = copyFrom.jVal;
	this->mass = copyFrom.mass;
	this->pose = copyFrom.pose;
	this->model = copyFrom.model;
	this->movable = copyFrom.movable;
	this->name = copyFrom.name;
	this->index = copyFrom.index;
	this->parent = NULL;
	this->robot = NULL;
	this->children.clear();
}

void Link::recursiveSetAncestry(Robot *rootRobot, Link *parentLink)
{
	this->parent = parentLink;
	this->robot = rootRobot;
	this->robot->links.push_back(this);
	for(unsigned int i = 0; i < this->children.size(); i++)
		this->children[i]->recursiveSetAncestry(rootRobot, this);
}

// Updates relative position based on updated joint value
void Link::updateRelPose(){
	Transform<double, 3, Eigen::Affine> jT;
	jT.setIdentity();
	if(jType == PRISM) jT.translate(jAxis*jVal);
	if(jType == REVOL) jT = AngleAxisd(jVal,jAxis);
	pose = jTrans*jT;
}

// Updates absolute pose from relative pose and parent absolute pose
void Link::updateAbsPose(){
	if(parent != NULL){
		absPose = parent->absPose*pose;
	}
}

// Given a new absolute position, propogates effects down the tree
void Link::updateRecursive(bool fromJoints, bool collisions){
	if(collisions)
		this->robot->world->updateCollision(this);

	for(unsigned int i=0; i<children.size(); i++){
		if(fromJoints)
			children[i]->updateRelPose();
		children[i]->updateAbsPose();
		children[i]->updateRecursive(fromJoints, collisions);
	}
}

// Updates absolute parent position based on child absolute position
void Link::updateParentPose(){
	if(parent != NULL) {
		updateRelPose();
		parent->absPose = absPose*pose.inverse(Eigen::Affine);
	}
}

//Same as above but recurses effects throughout the robot
void Link::updateParentPoseRecursive(bool fromJoints, bool collisions){
	if(parent != NULL) {
		updateRelPose();
		parent->absPose = absPose*pose.inverse(Eigen::Affine);
		parent->updateParentPoseRecursive(fromJoints, collisions);
	}else{
		// When all the way up to the base link, come back down
		updateRecursive(fromJoints, collisions);
	}
}
