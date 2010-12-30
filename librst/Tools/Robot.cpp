#include <fstream>
#include <iostream>
#include <Tools/World.h>
#include <Tools/Robot.h>
#include <Tools/GLTools.h>

using namespace Eigen;

Robot::Robot()
{
		activeLinks.clear();
		links.clear();
}

Robot::Robot(Robot &copyFrom)
{
	links.clear();
	activeLinks.clear();
	//this->world = NULL; // good for safety, but we may want to clone robots w/in a world

	this->name = copyFrom.name;
	this->idNum = copyFrom.idNum;
	this->links.clear();

	for(unsigned int i = 0; i < copyFrom.links.size(); i++){
		this->links.push_back(new Link(*copyFrom.links[i]));
	}

	for(unsigned int i = 0; i < copyFrom.links.size(); i++){
		if(copyFrom.links[i]->parent == NULL){
			this->links[i]->parent = NULL;
			this->baseLink = this->links[i];
		}
		for(unsigned int j = 0; j < copyFrom.links[i]->children.size(); j++){
			int childId = copyFrom.links[i]->children[j]->index;
			Link* childLink = this->links[childId];
			this->links[i]->children.push_back(childLink);
			childLink->parent = this->links[i];
		}
	}

	int numRootLinks = (int)this->links.size();
	for(int i = 0; i < numRootLinks; i++)
		this->links[i]->recursiveSetAncestry(this, NULL);

	// Added copy of active links by jon in rst2.1
	for(unsigned int i = 0; i < copyFrom.activeLinks.size(); i++){
		int fromIDX = copyFrom.activeLinks[i]->index;
		this->activeLinks.push_back(this->links[fromIDX]);
	}
}

Robot::~Robot()
{
	for(unsigned int i = 0; i < this->links.size(); i++)
		delete this->links[i]; // this caused problems when i deleted from planner...
	this->links.clear();
}


void Robot::drawCOM()
{
			glPushMatrix();
			glColor3f(1.0f,0.0f,0.0f);
			glTranslated(this->COM(0, 3),this->COM(1, 3),0.0);
			DrawSphere(0.02f,10,10);
			glPopMatrix();
}

void Robot::Draw(){
	for(unsigned int i=0; i<links.size(); i++)
		links[i]->Draw();
}

int Robot::findLink(string name){
	for(unsigned int i=0; i<links.size(); i++)
		if(links[i]->name == name) return i;
	return -1;
}

void Robot::setConf(VectorXd conf, bool collision){
	for(unsigned int i=0; i<activeLinks.size(); i++){
		activeLinks[i]->jVal = conf[i];
	}
	baseLink->updateRecursive(true,collision);
}

void Robot::getConf(VectorXd conf){
	for(unsigned int i=0; i<activeLinks.size(); i++){
		conf[i] = activeLinks[i]->jVal;
	}
}

void Robot::updateCOM()
{
	for(unsigned int i=0; i<links.size();i++)
	{
		Link* link = links[i];
		if(link->parent==NULL)
		{
//			link->updateRecursiveCOM();
//			this->mass=link->treeMass;
//			this->COM=link->treeCOM;
		}
	}
}

int Robot::Load(string fullname, World* w){
	string path,line,str,filename;
	world = w;

	cout << "  LOADING ROBOT" << endl << "  " << fullname << endl;

	path = fullname.substr( 0, fullname.rfind("/")+1 );

	fstream rstream(fullname.c_str(),ios::in);

	int fpos;
	int lnum=0;

	while(!rstream.eof()){
		fpos = rstream.tellg();
		lnum++;

		rstream >> str;

		if(str[0] != '>'){
			rstream.seekg(fpos);
			getline(rstream,line);
			continue;
		}

		rstream >> str;

		Link* link;
		if(str == "LINK"){
				link = new Link();
				link->parent = NULL;
				link->robot = this; // this is the weird step that explains why the structure seems to recurse forever in debug view
				rstream >> link->name;
				cout << "Link: " << link->name << endl;
				rstream >> filename;

				if(filename != "NOMODEL"){
					string fullpath(path);
					fullpath.append(filename);
					world->CreateEntity(link,fullpath,true);
				}

				streampos curpos = rstream.tellg();
				rstream >> str;
				if(str == "NOCOLLISION"){
					cout << "NOCOL " << endl;
					world->vcollide.DeactivateObject(link->eid);
				}
				rstream.seekg(curpos);

				links.push_back(link);
				link->index = (int)links.size()-1;

		}
		//NEW ELSE IF FOR COM
		else if(str == "COM"){
				char cname[100];
				rstream >> cname;
				int cnum;

				cnum = findLink(cname);		// find index of link name

				link = links[cnum];

				double mass;
				rstream >> mass;
				link->mass= mass;

				Vector3d pos;
				rstream >> pos(0);
				rstream >> pos(1);
				rstream >> pos(2);
				link->COM = pos;

				cout<<endl<<"Link: "<<link->name<<" mass: "<<link->mass <<" COM X: "<<link->COM(0) <<" Y: "<<link->COM(1) <<" Z: "<< link->COM(2) <<endl;
		}
		else if(str == "CON"){
				string pname, cname;
				rstream >> pname;
				rstream >> cname;
				cout << pname + " " + cname << endl;
				int pnum, cnum;
				if(pname == "WORLD"){
					pnum = -2;
				}else{
					pnum = findLink(pname); // find index of parent name
				}
				cnum = findLink(cname);		// find index of link name

				//cerr << pnum << " " << cnum << endl;

				if(pnum == -1){ cerr << "Non-existant parent: " << pname << endl; return 1; }
				if(cnum == -1){ cerr << "Non-existant child: " << cname << endl; return 1; }

				link = links[cnum];
				if(pnum == -2){
					link->parent = NULL;
					this->baseLink = link;
				}else{
					link->parent = links[pnum];
					links[pnum]->children.push_back(link);
				}

				Vector3d pos;
				rstream >> pos(0);
				rstream >> pos(1);
				rstream >> pos(2);
				//link->jTrans.pos = pos;

				double roll,pitch,yaw;
				rstream >> roll;
				rstream >> pitch;
				rstream >> yaw;
				Matrix3d rot;
				rot = AngleAxisd(DEG2RAD(yaw), Vector3d::UnitZ())
				  * AngleAxisd(DEG2RAD(pitch), Vector3d::UnitY())
				  * AngleAxisd(DEG2RAD(roll), Vector3d::UnitX());
				link->jTrans=rot;

				link->jTrans.translation() = pos;

				string buf;
				rstream >> buf;
				if(buf == "PZ") link->jAxis << 0,0,1;
				if(buf == "NZ") link->jAxis << 0,0,-1;
				if(buf == "PX") link->jAxis << 1,0,0;
				if(buf == "NX") link->jAxis << -1,0,0;
				if(buf == "PY") link->jAxis << 0,1,0;
				if(buf == "NY") link->jAxis << 0,-1,0;

				rstream >> buf;
				if(buf == "FIXED") link->jType = Link::FIXED;
				if(buf == "REVOL") link->jType = Link::REVOL;
				if(buf == "PRISM") link->jType = Link::PRISM;
				if(buf == "FREE") link->jType = Link::FREE;

				if(link->jType == Link::REVOL || link->jType == Link::PRISM){
					rstream >> link->jMin;
					rstream >> link->jMax;

					activeLinks.push_back(link);
				}
				link->updateRelPose();

				link->jVal = 0.0;

		}
		else{
				cerr << "BAD LINE in Robot File" << endl;
		}
		getline(rstream,line);
	}
	rstream.close();

	for(unsigned int i=0; i<links.size(); i++){
		cerr << links[i]->name << ": ";
		for(unsigned int j=0; j<links[i]->children.size(); j++){
		  //cerr << links[i]->children[j]->name << " ";
		}
		cerr << endl;
	}

	cout << "." << endl;
	return 0;
}


void Robot::Info() {
	cout << "Robot: " << endl;
	cout << "name: " << name << endl;
	cout << "pathname: " << pathname << endl;
	cout << "idNum = " << idNum << endl;
	//cout << "COM = " << COM << endl;
	cout << "mass = " << mass << endl;
	cout << "links.size() = " << links.size() << endl;
	for(unsigned int i = 0; i < links.size(); i++) {
		cout << i << " ------------------------------- " << endl;
		cout << "  Link " << i << " Ptr " << links[i]  << " Parent " << links[i]->parent << endl;
		cout << "  Number of children = " << links[i]->children.size() << endl;
		//cout << "  pose = " << links[i]->pose << endl;
		//cout << "  jTrans = " << links[i]->jTrans << endl;
		//cout << "  jAxis = " << links[i]->jAxis << endl;
	}
	cout << "activeLinks.size() = " << activeLinks.size() << endl;

//	cout << "world " << world->robots[0]->links[4] << " " << world->robots[0]->links[4]->name << " " << world->robots[0]->links[4]->parent << endl;

}
