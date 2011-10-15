/**
 * @file Parser.h
 * @brief Reads a RSDH file and spit out a World object
 * @author A. Huaman
 */

#include "kinematics/Skeleton.h"
#include "kinematics/BodyNode.h"
#include "kinematics/Joint.h"
#include "kinematics/Dof.h"
#include "kinematics/TrfmRotateEuler.h"
#include "kinematics/TrfmTranslate.h"

#include "GUI.h"
#include "Parser.h"

#define BSTATE 0
#define RSTATE 1
#define OSTATE 2

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace std;
using namespace Eigen;

/**
 * @function ParseWorld
 * @brief Read a RSDH file. Spits out a World object
 */
planning::World* parseWorld( std::string _fullname )
{
    planning::World *world = new planning::World();

    string worldPath( _fullname );

    // Change path to a Unix-style path if given a windows one:
    // windows can handle Unix-style paths.
    replace( worldPath.begin(), worldPath.end(), '\\', '/' );
    string path = worldPath.substr(0, worldPath.rfind("/") + 1);

    string line, str, filename, fullpath;
    fstream wstream( _fullname.c_str(), ios::in );

    int lnum = 0;

    int state = BSTATE;

    planning::Robot *robot;
    planning::Object* object;

    while (!wstream.eof()) {
        lnum++;

        wstream >> str;
	if ( str[0] != '>' ) {
	    getline( wstream, line );
	    continue;
        }

	wstream >> str;

	if (str == "ROBOT") {
	    robot = new planning::Robot();
	    wstream >> robot->mName;
	    std::cout << "-- Loading: " << robot->mName << std::endl;
	    wstream >> robot->mPathName;
	    fullpath = path;
	    fullpath.append( robot->mPathName );
	    parseRobot( fullpath, robot );
	    world->addRobot( robot );
	    state = RSTATE;

        } else if (str == "OBJECT") {
            object = new planning::Object();
	    wstream >> object->mName;
	    std::cout << "-- Loading: " << object->mName << std::endl;
	    wstream >> object->mPathName;
 
            fullpath = path;
            fullpath.append( object->mPathName );
            parseObject( fullpath, object );
            world->addObject( object );
	    state = OSTATE;

	} else if (str == "CAMERA") {
            
	    double roll, pitch, yaw, tx, ty, tz, rad;
	    string space;
	    wstream >> space >> roll >> pitch >> yaw;
	    wstream >> space >> tx >> ty >> tz;
	    wstream >> space >> rad;
	    std::cout << "-- Loading Camera" << std::endl;

            ///-- Loading Viewer parameters
	    mCamRotT = AngleAxisd(DEG2RAD(yaw), Vector3d::UnitZ())
				  * AngleAxisd(DEG2RAD(pitch), Vector3d::UnitY())
				  * AngleAxisd(DEG2RAD(roll), Vector3d::UnitX());
	    mWorldV = Vector3d( tx, ty, tz );
	    mCamRadius = rad;
          
	} else if (str == "COLORS") {
          
	    string space;
	    wstream >> space >> mBackColor[0] >> mBackColor[1] >> mBackColor[2];
	    wstream >> space >> mGridColor[0] >> mGridColor[1] >> mGridColor[2];
	    std::cout << "Loading Colors" << std::endl;
          
	}
	///////////////////////////////////
	// READ ROBOT SPEC
	///////////////////////////////////
	else if (state == RSTATE) {
        /*
	    if (str == "POSITION") {
	        Vector3d pos;
		wstream >> pos(0) >> pos(1) >> pos(2);
		robot->baseLink->absPose.translation() = pos;
	    } else if (str == "ORIENTATION") {
		double roll, pitch, yaw;
		wstream >> roll >> pitch >> yaw;
		Matrix3d rot;
		rot = AngleAxisd(DEG2RAD(yaw), Vector3d::UnitZ())
				  * AngleAxisd(DEG2RAD(pitch), Vector3d::UnitY())
				  * AngleAxisd(DEG2RAD(roll), Vector3d::UnitX());
		Vector3d temp = robot->baseLink->absPose.translation();
		robot->baseLink->absPose = rot;
		robot->baseLink->absPose.translation() = temp;
	    } else if (str == "INIT") {
	        string buf;
		wstream >> buf;
		int lnum = robot->findLink(buf.c_str());
		if (lnum == -1) {
		    cout << " ";
		} else {
		    wstream >> robot->links[lnum]->jVal;
		    std::cout << "Init: Link" << lnum << " to " << robot->links[lnum]->jVal << std::endl;
		}
	    }
        */
        }
	///////////////////////////////////
        // READ OBJECT SPEC
	///////////////////////////////////
	else if (state == OSTATE) {
           /*
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
		if (buf == "MOVABLE") {  object->movable = true; }
	    }
           */
	} else {
            /*
	    std::cerr << "ERROR READING WORLD FILE AT LINE " << lnum << std::endl;
	    wstream.close();
	    return 1;
            */
	}
        getline(wstream, line);
    } /// end of while
    wstream.close();
    /*
    for (unsigned int j = 0; j < robots.size(); j++) {
        robots[j]->baseLink->updateAbsPose();
    }
    */
    //updateAllCollisionModels();
    cout << "Finished Loading!" << endl;

    //detectCollisions();

    return world;
}


/**
 * @function parseRobot
 * @brief Load robot from RSDH file
 */
int parseRobot( string _fullname, planning::Robot *_robot ) {

    std::cout<< "--> Parsing robot "<< _robot->mName << std::endl;
    std::string path, line, str, filename;
    
    path = _fullname.substr( 0, _fullname.rfind("/")+1 );

    fstream rstream( _fullname.c_str(),ios::in );

    int fpos;
    int lnum=0;
    int rootnum = -1;

    std::vector< kinematics::BodyNode* > bodyNodes;
    std::vector< kinematics::Joint* > joints;

    while(!rstream.eof()) {
	lnum++;

	rstream >> str;

	if(str[0] != '>') {
	    getline(rstream,line);
	    continue;
	}

	rstream >> str;

	if( str == "LINK" ) {
            char name[30];
	    rstream >> name; 
            kinematics::BodyNode *bodyNode = _robot->createBodyNode( name );
	    rstream >> filename;

	    if(filename != "NOMODEL") {
	        string fullpath( path );
		fullpath.append( filename );
                //-- TODO Create a PrimitiveMesh from the 3DS Model 
                kinematics::PrimitiveMesh *p = new kinematics::PrimitiveMesh( Vector3d(0, 0, 0), 0 );
		bodyNode->setPrimitive( p );
	    }

	    // Does not work under Windows if file uses Linux line breaks
            // Current solution: NOCOLLISION is not supported anymore
	    //streampos curpos = rstream.tellg();
	    //rstream >> str;
	    //if(str == "NOCOLLISION"){
	    //	cout << "NOCOL " << endl;
	    //	world->vcollide.DeactivateObject(link->eid);
	    //}
	    //rstream.seekg(curpos);

	    bodyNodes.push_back( bodyNode );
	}

	else if(str == "COM") {
	    char cname[100];
	    rstream >> cname;

	    int cnum = -1;

            for( unsigned int i = 0; i < bodyNodes.size(); i++ ) {    
                if( bodyNodes[i]->getName() == cname ) { cnum = i; break; } 
            }

            if( cnum >= 0 ) {
	        double mass;
	        rstream >> mass;
                
	        kinematics::Primitive* p = bodyNodes[cnum]->getPrimitive(); 
                p->setMass( mass );

	        Vector3d pos;
	        rstream >> pos(0);
	        rstream >> pos(1);
	        rstream >> pos(2);
	        bodyNodes[cnum]->setLocalCOM( pos );
            }

        } 

        else if(str == "CON") {
            kinematics::Joint* joint;
            kinematics::Transformation* trans;
	    string pname, cname;
	    rstream >> pname;
	    rstream >> cname;

	    int pnum, cnum;

            pnum = -1;

	    if( pname == "WORLD" ) {
	        pnum = -2;  // and it is parentNode
	    } else {
                for( unsigned int i = 0; i < bodyNodes.size(); i++ ) {    
                    if( bodyNodes[i]->getName() == pname ) { pnum = i; break; } 
                }
	    }

            for( unsigned int i = 0; i < bodyNodes.size(); i++ ) {    
                if( bodyNodes[i]->getName() == cname ) { cnum = i; break; } 
                }

	    if(pnum == -1){  std::cerr << "--(!) Non-existant parent: " << pname << std::endl; return 1;  }
	    if(cnum == -1){  std::cerr << "--(!) Non-existant child: " << cname << std::endl; return 1;  }

	    if(pnum == -2){
                joint = new kinematics::Joint( NULL, bodyNodes[cnum], "Joint" );
                rootnum = cnum;
	    } else {
                joint = new kinematics::Joint( bodyNodes[pnum], bodyNodes[cnum], "Joint" );
            }

            
	    Vector3d pos;
	    rstream >> pos(0);
	    rstream >> pos(1);
	    rstream >> pos(2);
            
	    double roll, pitch, yaw;
	    rstream >> roll;
	    rstream >> pitch;
	    rstream >> yaw;
 
            /** Add Transform for rotation and translation. No DOF */ 

            trans = new kinematics::TrfmTranslate( new kinematics::Dof(pos(0)),
						   new kinematics::Dof(pos(1)),
						   new kinematics::Dof(pos(2)),
					           "Translate" );
	    joint->addTransform( trans, false ); 

            trans = new kinematics::TrfmRotateEulerZ( new::kinematics::Dof(yaw) );
            joint->addTransform( trans, false );  

            trans = new kinematics::TrfmRotateEulerY( new::kinematics::Dof(pitch) );
            joint->addTransform( trans, false );  

            trans = new kinematics::TrfmRotateEulerX( new::kinematics::Dof(roll) );
            joint->addTransform( trans, false );  


	    string bufAxis;
	    rstream >> bufAxis;

            string bufType;
	    rstream >> bufType;

            int type = -1; // -1: NONE 0: REVOL 1: PRISM
 
	    if( bufType == "FIXED" ) 
            {  }

	    if( bufType == "REVOL" ) 
            {
                type = 0;
         
                if( bufAxis == "PZ" ) 
                {  trans = new kinematics::TrfmRotateEulerZ( new kinematics::Dof(0) );  }
                if( bufAxis == "NZ" ) 
                {  trans = new kinematics::TrfmRotateEulerZ( new kinematics::Dof(0) );  }
                if( bufAxis == "PY" ) 
                {  trans = new kinematics::TrfmRotateEulerY( new kinematics::Dof(0) );  }
                if( bufAxis == "NY" ) 
                {  trans = new kinematics::TrfmRotateEulerY( new kinematics::Dof(0) );  }
                if( bufAxis == "PX" ) 
                {  trans = new kinematics::TrfmRotateEulerX( new kinematics::Dof(0) );  }
                if( bufAxis == "NX" ) 
                {  trans = new kinematics::TrfmRotateEulerX( new kinematics::Dof(0) );  }
                  
                joint->addTransform( trans, true );
                _robot->addTransform( trans ); 
            }

	    if(bufType == "PRISM")
            {
                type = 1;

                if( bufAxis == "PZ" ) 
                {  trans = new kinematics::TrfmTranslateZ( new kinematics::Dof(0) );  }
                if( bufAxis == "NZ" ) 
                {  trans = new kinematics::TrfmTranslateZ( new kinematics::Dof(0) );  }
                if( bufAxis == "PY" ) 
                {  trans = new kinematics::TrfmTranslateY( new kinematics::Dof(0) );  }
                if( bufAxis == "NY" ) 
                {  trans = new kinematics::TrfmTranslateY( new kinematics::Dof(0) );  }
                if( bufAxis == "PX" ) 
                {  trans = new kinematics::TrfmTranslateX( new kinematics::Dof(0) );  }
                if( bufAxis == "NX" ) 
                {  trans = new kinematics::TrfmTranslateX( new kinematics::Dof(0) );  }
                  
                joint->addTransform( trans, true );
                _robot->addTransform( trans ); 
            }

	    if(bufType == "FREE") 
            {  }

	    if( type == 0 || type == 1 ) {
                double min, max;
		rstream >> min;
		rstream >> max;
		if( type == 0) {
                    joint->getDof(0)->setMin( DEG2RAD(min) );
                    joint->getDof(0)->setMax( DEG2RAD(max) );
		} else {
                    joint->getDof(0)->setMin( min );
                    joint->getDof(0)->setMax( max );
		}
	    }
            
	} else{
	    std::cerr << "BAD LINE in Robot File" << std::endl;
	}
		
        getline(rstream,line);
    }
    rstream.close();

    for( unsigned int i=0; i < bodyNodes.size(); i++ ) {
        _robot->addNode( bodyNodes[i] ); 
    	std::cerr << std::endl;
    }

    return 0;
}

/**
 * @function parseObject
 * @brief Load object
 */
int parseObject( string _fullpath, planning::Object *_object )
{
    _object->mMovable = false;

    kinematics::BodyNode *bodyNode = _object->createBodyNode( "object" );

    kinematics::Joint *joint = new kinematics::Joint( NULL, bodyNode );
    _object->addNode( bodyNode );

   return 0;
}

