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
	this->treeCOM = copyFrom.treeCOM;
	this->treeMass = copyFrom.treeMass;
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
	//for(int i = 0; i < 100; i++)
	this->name = copyFrom.name;

	this->index = copyFrom.index;
	//this->parent = copyFrom.parent;
	//this->robot = copyFrom.robot;
	this->parent = NULL;
	this->robot = NULL;

	this->children.clear();


	//for(unsigned int i = 0; i < copyFrom.children.size(); i++)
	//{
	//	this->children.push_back(new Link(*copyFrom.children[i]));
	//	this->children[i]->parent = this;
	//	this->children[i]->robot = this->robot;
	//}
}



void Link::recursiveSetAncestry(Robot *rootRobot, Link *parentLink)
{
	this->parent = parentLink;
	this->robot = rootRobot;
	this->robot->links.push_back(this);
	for(unsigned int i = 0; i < this->children.size(); i++)
		this->children[i]->recursiveSetAncestry(rootRobot, this);
}

void Link::updateRelPose(){
	Transform<double, 3, Eigen::Affine> jT;
	jT.setIdentity();
	//TODO
	//if(jType == PRISM) jT.translate(jT.linear() + jAxis*jVal);
	//if(jType == REVOL){ jT.rot.set_uniaxis_rot(jVal,jAxis); }
	pose = jTrans*jT;
}

void Link::updateAbsPose(){
	if(parent != NULL){
		absPose = parent->absPose*pose;
	}
}

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

//Gives pose of parent based on child position
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

void Link::updateRecursiveCOM()
{
	if(children.size()!=0)
	{
		if(this->mass<=0.0)
		{
			this->mass=0.0;
			this->COM(0) = 0.0f;
			this->COM(1) = 0.0f;
			this->COM(2) = 0.0f;
		}

		this->treeCOM=this->COM*this->mass;
		this->treeMass=mass;

		for(unsigned int i=0; i<children.size();i++)
		{
			children[i]->updateRecursiveCOM();
			Transform<double, 3, Eigen::Affine> jT;
			jT.setIdentity();
			//this->treeCOM+=children[i]->treeMass*((jTrans.pos+(children[i]->jAxis*children[i]->jVal))*children[i]->treeCOM);
			this->treeCOM+=children[i]->pose*children[i]->treeCOM*children[i]->treeMass;
			this->treeMass+=children[i]->treeMass;
		}
		this->treeCOM=this->treeCOM/this->treeMass;
		//cout<<endl<<" tree "<<this->name<<" Mass: "<<this->treeMass<<" X "<<this->treeCOM.x<<" Y "<< this->treeCOM.y<<" Z "<< this->treeCOM.z<<endl;
	}
	else
	{
		this->treeMass=this->mass;
		this->treeCOM=this->COM; //If no children then the treeCOM starts with this link's COM
		//cout<<endl<<" No kids tree "<<this->name<<" Mass: "<<this->treeMass<<" X "<<this->treeCOM.x<<" Y "<< this->treeCOM.y<<" Z "<< this->treeCOM.z<<endl;
	}

}

