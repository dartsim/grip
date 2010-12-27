#include <algorithm>
#include <fstream>
#include <iostream>
#include <vector>
#include <math.h>
#include <Tools/Model3DS.h>
#include <Tools/World.h>
#include <Tools/Robot.h>
#include <Tools/Object.h>

#define BSTATE 0
#define RSTATE 1
#define OSTATE 2

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace Eigen;

World::World()
{
}

World::World(World &copyFrom)
{
	this->entities.clear();
	this->objects.clear();
	this->robots.clear();

	for(unsigned int i = 0; i < copyFrom.objects.size(); i++) {
		this->objects.push_back(new Object(*copyFrom.objects[i]));
		this->objects.back()->world = this; // don't forget to update the world ptr...
	}

	for(unsigned int i = 0; i < copyFrom.robots.size(); i++) {
		this->robots.push_back(new Robot(*copyFrom.robots[i]));
		this->robots.back()->world = this; // don't forget to update the world ptr...
	}

	for(unsigned int i = 0; i < copyFrom.entities.size(); i++)
		this->entities.push_back(copyFrom.entities[i]);

	this->vcollide = copyFrom.vcollide; // just point to collision models originally created when 1st world was loaded
}

World::~World()
{
	for(unsigned int i = 0; i < this->objects.size(); i++)
		delete this->objects[i];
	this->objects.clear();
	for(unsigned int i = 0; i < this->robots.size(); i++)
		delete this->robots[i];
	this->robots.clear();
}

void World::DeleteModels()
{
	cout << "deleting models... " << endl;
	for(unsigned int i = 0; i < this->entities.size(); i++)
	{
		cout << "Deleting model for entity: " << this->entities[i].model->modelname << endl;
		delete this->entities[i].model;
	}
	this->entities.clear();
	cout << "done deleting models" << endl;
}

void World::Draw(){
	for(unsigned int i=0; i<objects.size(); i++){
		objects[i]->Draw();
	}
	for(unsigned int i=0; i<robots.size(); i++){
		robots[i]->Draw();
	}
}

void World::CreateEntity(Object* object, string path, bool link){
	Entity ent;
	ent.object = object;
	ent.isLink = link;

	object->world = this;

	Model3DS *model = object->LoadModel(path);
	ent.model = model;

	vector<Model3DS::Triangle> *trigs = new vector<Model3DS::Triangle>;
	object->model->ReportTriangles(trigs);
	int collCounter;
    flag=false;
	vcollide.NewObject(&collCounter);
	cerr << "Creating COLLISION MODEL: " << collCounter << endl;// << " for " << object->name << " # " << trigs->size() << endl;
	for(unsigned int i = 0; i < trigs->size(); i++){
		vcollide.AddTri((*trigs)[i].v1,(*trigs)[i].v2,(*trigs)[i].v3,collCounter);
	}
	vcollide.EndObject();
	vcollide.ActivateObject(collCounter);
	trigs->clear();
	delete trigs;

	//MUST BE TRUE
	assert((int)entities.size() == collCounter);

	object->eid = (int)entities.size();

	entities.push_back(ent);
}

// Writes the scene state to a .rscene file
int World::Save(string filename) {
	ofstream wstream(filename.c_str());

	if (!wstream.is_open()) return 1;

	if(robots.size() > 0) {
		wstream << "##### ROBOTS #####" << endl;
		wstream << endl;
		unsigned int i = 0;
		while(i < robots.size()) {
			wstream << "##### ROBOT " << i+1 << " #####" << endl;
			wstream << endl;
			wstream << "> \t" << "ROBOT " << robots[i]->name << " " << robots[i]->pathname << endl;
			wstream << "> \t" << "POSITION " << robots[i]->baseLink->absPose(0,3) << " " << robots[i]->baseLink->absPose(1,3) << " " << robots[i]->baseLink->absPose(2,3) << endl;
			//wstream << "> \t" << "ORIENTATION " << robots[i]->baseLink->absPose.rot.roll()*180/M_PI << " " << robots[i]->baseLink->absPose.rot.pitch()*180/M_PI << " " << robots[i]->baseLink->absPose.rot.yaw()*180/M_PI << endl;
			double roll=atan2(robots[i]->baseLink->absPose(2,1), robots[i]->baseLink->absPose(2,2));
			double pitch=-asin(robots[i]->baseLink->absPose(2,0));
			double yaw=atan2(robots[i]->baseLink->absPose(1,0), robots[i]->baseLink->absPose(0,0));
			wstream << "> \t" << "ORIENTATION " << RAD2DEG(roll) << " " << RAD2DEG(pitch) << " " << RAD2DEG(yaw) << endl;
			wstream << endl;
			wstream << "##### INITIAL ANGLES #####" << endl;
			wstream << endl;
			unsigned int lnum = 0;
			while(lnum < robots[i]->links.size()) {
				wstream << "> \t" << "INIT " << robots[i]->links[lnum]->name << " " << robots[i]->links[lnum]->jVal << endl;
				lnum++;
			}
			wstream << endl;
			i++;
		}
	}

	if(objects.size() > 0) {
		wstream << "##### OBJECTS #####" << endl;
		wstream << endl;
		unsigned int i = 0;
		while(i < objects.size()) {
			wstream << "##### OBJECT " << i+1 << " #####" << endl;
			wstream << endl;
			wstream << "> \t" << "OBJECT " << objects[i]->name << " " << objects[i]->pathname << endl;
			wstream << "> \t" << "POSITION " << objects[i]->absPose(0,3) << " " << objects[i]->absPose(1,3) << " " << objects[i]->absPose(2,3) << endl;
			double roll=atan2(objects[i]->absPose(2,1), objects[i]->absPose(2,2));
			double pitch=-asin(objects[i]->absPose(2,0));
			double yaw=atan2(objects[i]->absPose(1,0), objects[i]->absPose(0,0));
			wstream << "> \t" << "ORIENTATION " << RAD2DEG(roll) << " " << RAD2DEG(pitch) << " " << RAD2DEG(yaw) << endl;
			wstream << endl;
			i++;
		}
	}

	wstream.close();
	return 0;
}

int World::Load(string fullname) {
	string worldPath(fullname);

	// Change path to a Unix-style path if given a windows one:
	// windows can handle Unix-style paths.
	replace(worldPath.begin(), worldPath.end(), '\\', '/');
	string path = worldPath.substr(0, worldPath.rfind("/") + 1);

	string line, str, filename, fullpath;
	fstream wstream(fullname.c_str(), ios::in);

	int fpos;
	int lnum = 0;

	int state = BSTATE;

	Robot * robot;
	Object* object;

	while (!wstream.eof()) {
		fpos = wstream.tellg();
		lnum++;

		wstream >> str;
		if (str[0] != '>') {
			wstream.seekg(fpos);
			getline(wstream, line);
			continue;
		}

		wstream >> str;
		if (str == "ROBOT") {
			robot = new Robot();
			cout << "robot handle = " << robot << endl;
			wstream >> robot->name;
			cout << "Loading: " << robot->name << endl;
			wstream >> robot->pathname;

			fullpath = path;
			fullpath.append(robot->pathname);

			robot->Load(fullpath, this);

			robots.push_back(robot);
			state = RSTATE;
		} else if (str == "OBJECT") {
			object = new Object();
			cout << "object handle = " << object << endl;
			wstream >> object->name;
			cout << "Loading: " << object->name << endl;
			wstream >> object->pathname;
			object->movable = false;

			if (object->pathname != "NOMODEL") {
				fullpath = path;
				fullpath.append(object->pathname);

				CreateEntity(object, fullpath.c_str(), false);
			}

			object->absPose.Identity();
			objects.push_back(object);
			state = OSTATE;
		}
		///////////////////////////////////
		// READ ROBOT SPEC
		///////////////////////////////////
		else if (state == RSTATE) {
			if (str == "POSITION") {
				Vector3d pos;
				wstream >> pos(0);
				wstream >> pos(1);
				wstream >> pos(2);
				robot->baseLink->absPose.translation() = pos;
			} else if (str == "ORIENTATION") {
				double roll, pitch, yaw;
				wstream >> roll;
				wstream >> pitch;
				wstream >> yaw;
				Matrix3d rot;
				rot = AngleAxisd(DEG2RAD(yaw), Vector3d::UnitZ())
				  * AngleAxisd(DEG2RAD(pitch), Vector3d::UnitY())
				  * AngleAxisd(DEG2RAD(roll), Vector3d::UnitX());
				Vector3d temp = robot->baseLink->absPose.translation();
				robot->baseLink->absPose = rot;
				robot->baseLink->absPose.translation() = temp;
//				robot->baseLink->absPose.rot = rot * robot->baseLink->absPose.rot;
			} else if (str == "INIT") {
				string buf;
				wstream >> buf;
				int lnum = robot->findLink(buf.c_str());
				if (lnum == -1) {
					cout << " ";
				} else {
					wstream >> robot->links[lnum]->jVal;
					cout << "Init: Link" << lnum << " to " << robot->links[lnum]->jVal << endl;
				}
			}

		}
		///////////////////////////////////
		// READ OBJECT SPEC
		///////////////////////////////////
		else if (state == OSTATE) {
			if (str == "POSITION") {
				Vector3d pos;
				wstream >> pos(0);
				wstream >> pos(1);
				wstream >> pos(2);
				object->absPose.translation() = pos;
			} else if (str == "ORIENTATION") {
				double roll, pitch, yaw;
				wstream >> roll;
				wstream >> pitch;
				wstream >> yaw;
				Matrix3d rot;
				rot = AngleAxisd(DEG2RAD(yaw), Vector3d::UnitZ())
				  * AngleAxisd(DEG2RAD(pitch), Vector3d::UnitY())
				  * AngleAxisd(DEG2RAD(roll), Vector3d::UnitX());
				Vector3d temp = object->absPose.translation();
				object->absPose = rot;
				object->absPose.translation() = temp;
			} else if (str == "TYPE") {
				string buf;
				wstream >> buf;
				if (buf == "MOVABLE")
					object->movable = true;
			}
		} else {
			cerr << "ERROR READING WORLD FILE AT LINE " << lnum << endl;
			wstream.close();
			return 1;
		}
		getline(wstream, line);
	}
	wstream.close();
	for (unsigned int j = 0; j < robots.size(); j++) {
		updateRobot(robots[j]);
	}

	updateAllCollisions();
	cout << "Finished Loading!" << endl;

	detectCollisions();

	return 0;
}

int World::findRobot(string name){
	for(unsigned int i=0; i<robots.size(); i++)
		if(robots[i]->name == name) return i;
	return -1;
}

bool World::checkCollisions(){
	VCReport report; // jon: why declaring internal report here???
    vcollide.Collide( &report, VC_FIRST_CONTACT);  //perform collision test.
	if(report.numObjPairs() > 0) return false;
	return true;
}

void World::clearCollisions(){
    for (unsigned int i = 0; i < entities.size(); i++)
    {
		entities[i].object->collisionFlag = false;
    }
}

void World::detectCollisions(){
    vcollide.Collide( &report, VC_FIRST_CONTACT);  //perform collision test.
	for(unsigned int i=0; i<entities.size(); i++){
		entities[i].object->collisionFlag = false;

	}
    for (int j = 0; j < report.numObjPairs(); j++)
    {
        flag=true;
		entities[report.obj1ID(j)].object->collisionFlag = true;
		entities[report.obj2ID(j)].object->collisionFlag = true;
		//cout << "COLL: "   << entities[report.obj1ID(j)].object->name<< " : " << entities[report.obj2ID(j)].object->name<<endl;
    }
}

void World::updateAllCollisions(){
	for(unsigned int i=0; i<entities.size(); i++){
		updateCollision(entities[i].object);
		entities[i].object->collisionFlag = false;
	}
	for(unsigned int i=0; i<robots.size(); i++){
		Robot* r = robots[i];
		for(unsigned int j=0; j<r->links.size(); j++){
			for(unsigned int k=j; k<r->links.size(); k++){
				Link* l1 = r->links[j];
				Link* l2 = r->links[k];
				if(l1->model != NULL && l2->model != NULL){
					vcollide.DeactivatePair(l1->eid,l2->eid);
				}
			}
		}
	}
}

void World::updateCollision(Object* ob){
	if(ob->model == NULL) return;
	int eid = ob->eid;
	//Mat33 r = ob->absPose.rot;
	//Vec3  t = ob->absPose.pos;

	double newTrans[4][4] =
	{{ob->absPose(0,0), ob->absPose(0,1), ob->absPose(0,2), ob->absPose(0,3)},
	{ob->absPose(1,0), ob->absPose(1,1), ob->absPose(1,2), ob->absPose(1,3)},
	{ob->absPose(2,0), ob->absPose(2,1), ob->absPose(2,2), ob->absPose(2,3)},
	{0, 0, 0, 1}};
	//double newTrans[4][4] = ob->absPose.data();
	//double newTrans[4][4];
	//memcpy(&newTrans,ob->absPose.data(),16*sizeof(double));

	vcollide.UpdateTrans(eid,newTrans);
}

void World::updateRobot(Robot* robot)
{
	for(unsigned int i=0; i < robot->links.size(); i++){
		//cerr << "LS - " << robot->links.size() << " ";
		Link* link = robot->links[i];
		if(link->parent == NULL){
			//cerr << "ROOT: " << link->name;
			//link->absPose = robot->absPose*link->pose;
			link->updateRecursive(true, true); // right?? changed by jon to test
		}
	}
}


