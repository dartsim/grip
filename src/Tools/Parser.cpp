/**
 * @file Parser.h
 * @brief Reads a RSDH file and spit out a DART World object
 * @author A. Huaman
 */

#include "dynamics/SkeletonDynamics.h"
#include "dynamics/BodyNodeDynamics.h"
#include "kinematics/Joint.h"
#include "kinematics/Dof.h"
#include "kinematics/TrfmRotateEuler.h"
#include "kinematics/TrfmTranslate.h"

#include "GUI/GUI.h"
#include "Parser.h"

// PARSER_URDF is defined in CMakeList
#ifdef PARSER_URDF
#include "ParserURDF.h"
#endif

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
robotics::World* parseWorld( std::string _fullname )
{
    robotics::World *world = new robotics::World();

    string worldPath( _fullname );

    // Change path to a Unix-style path if given a windows one:
    // windows can handle Unix-style paths.
    replace( worldPath.begin(), worldPath.end(), '\\', '/' );
    string path = worldPath.substr(0, worldPath.rfind("/") + 1);

    string line, str, filename, fullpath;
    fstream wstream( _fullname.c_str(), ios::in );

    int lnum = 0;

    int state = BSTATE;

    robotics::Robot *robot;
    robotics::Object* object;

    /// Read .rscene file
    while (!wstream.eof()) {
        lnum++;

        wstream >> str;
	if ( str[0] != '>' ) {
	    getline( wstream, line );
	    continue;
        }

	wstream >> str;

        /// Read .rsdh Robot description file
	if (str == "ROBOT") {
	    std::string temp;
	    robot = new robotics::Robot();
	    wstream >> temp;
		  robot->setName(temp);
	    std::cout << "--> Loading: " << robot->getName() << std::endl;
	    wstream >> temp;
      robot->setPathName(temp);
	    fullpath = path;
	    fullpath.append( robot->getPathName() );

		string ext = fullpath.substr(fullpath.find_last_of(".") + 1);

		std::cout << "Robot extension: " << ext << endl;

		if(ext == "rsdh")
		{
			std::cout << "Parsing with rhsd parser" << endl;		
			parseRobot( fullpath, robot );
		}
		else if(ext == "urdf")
		{
			#ifdef PARSER_URDF
			std::cout << "Parsing with URDF parser" << endl;
			ParserURDF parser;
			parser.readURDFFile(fullpath.c_str(), robot);
			#endif
		}	    
		
		world->addRobot( robot );
	    state = RSTATE;

        /// Read object specification
        } else if (str == "OBJECT") {
            object = new robotics::Object();
						std::string temp;
	    wstream >> temp;
      object->setName(temp);
	    std::cout << "--> Loading: " << object->getName() << std::endl;
	    wstream >> temp;
       object->setPathName(temp);
 
            fullpath = path;
            fullpath.append( object->getPathName() );
            parseObject( fullpath, object );
            world->addObject( object );
	    state = OSTATE;
         
        /// Read camera specification
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
          
        /// Read background and grid colors
	} else if (str == "COLORS") {
          
	    string space;
	    wstream >> space >> mBackColor[0] >> mBackColor[1] >> mBackColor[2];
	    wstream >> space >> mGridColor[0] >> mGridColor[1] >> mGridColor[2];
	    std::cout << "-- Loading Colors" << std::endl;
          
	}
	/** --------------------------------- */
	/**  READ ROBOT SPEC                  */
	/** --------------------------------- */
	else if (state == RSTATE) {

	    if (str == "POSITION") {
	        Vector3d pos;
		wstream >> pos(0) >> pos(1) >> pos(2);

                robot->setPositionX( pos(0) );
                robot->setPositionY( pos(1) );
                robot->setPositionZ( pos(2) );

	    } else if (str == "ORIENTATION") {

		double roll, pitch, yaw;
		wstream >> roll >> pitch >> yaw;

                robot->setRotationRPY( DEG2RAD(roll), DEG2RAD(pitch), DEG2RAD(yaw) );

	    } else if (str == "INIT") {

	        string buf;
		wstream >> buf;
                char* name = (char*) buf.c_str();
                cout<< "Name:" << name << endl;
		dynamics::BodyNodeDynamics *node = (dynamics::BodyNodeDynamics* )robot->getNode( name );

                if( node )
                { 
                    double val;
 		    wstream >> val;

                    if( node->getParentJoint()->getNumDofs() == 0 ) continue;
 
                    node->getParentJoint()->getDof(0)->setValue(val);
		    std::cout << "Init: node" << node->getName() << " to " << node->getParentJoint()->getDof(0)->getValue() << std::endl;
		}

	    }
        
        }
	/** --------------------------------- */
        /** READ OBJECT SPEC                  */
	/** --------------------------------- */
	else if (state == OSTATE) {
            kinematics::Joint *joint;
	    if (str == "POSITION") {
	        Vector3d pos;
		wstream >> pos(0);
		wstream >> pos(1);
		wstream >> pos(2);

                object->setPositionX( pos(0) );
                object->setPositionY( pos(1) );
                object->setPositionZ( pos(2) );

	    } else if (str == "ORIENTATION") {
		double roll, pitch, yaw;
		wstream >> roll;
		wstream >> pitch;
		wstream >> yaw;

                object->setRotationRPY( DEG2RAD(roll), DEG2RAD(pitch), DEG2RAD(yaw) );
 
	    } else if (str == "TYPE") {
		string buf;
		wstream >> buf;
		if (buf == "MOVABLE") {  object->setMovable( true ); }
	    }
           
	} else {
            
	    std::cerr << "ERROR READING WORLD FILE AT LINE " << lnum << std::endl;
	    wstream.close();
            return NULL;
	}
        getline(wstream, line);
    } /// end of while
    wstream.close();

    for (unsigned int i = 0; i < world->getNumRobots(); i++) {
        world->getRobot(i)->initSkel();
    }

    for (unsigned int i = 0; i < world->getNumObjects(); i++) {
        world->getObject(i)->initSkel();
    }

    cout << "--(v) Finished Loading!" << endl;

    return world;
}


/**
 * @function parseRobot
 * @brief Load robot from RSDH file
 */
int parseRobot( string _fullname, robotics::Robot *_robot ) {

    kinematics::Joint* joint;
    dynamics::BodyNodeDynamics *node;
    kinematics::Transformation* trans;

    std::vector<const aiScene*> models;
    std::vector<int> modelsInd;
    const aiScene* model;

    std::cout<< "--> Parsing robot "<< _robot->getName() << std::endl;

    std::string path, line, str, filename;    
    path = _fullname.substr( 0, _fullname.rfind("/") + 1 );

    fstream rstream( _fullname.c_str(),ios::in );

    //int fpos;
    int lnum = 0;

    //-- Set the containers for the bodyNodes and joints for the robot
    std::vector< dynamics::BodyNodeDynamics* > bodyNodes(0);
    std::vector< kinematics::Joint* > joints(0);


    //-- Read the RSDH file
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
          node = (dynamics::BodyNodeDynamics *)_robot->createBodyNode( name );
	        rstream >> filename;

  	      if(filename != "NOMODEL") {
	            string fullpath( path );
		          fullpath.append( filename );
              //-- TODO Create a ShapeMesh from the 3DS Model
              model = _robot->loadModel( fullpath );
                
              kinematics::ShapeMesh *p = new kinematics::ShapeMesh( Vector3d(1, 1, 1), 0, NULL );
	      node->setShape( p );
	        } else {
	            model = 0;
	        }
            
          models.push_back( model );
	        bodyNodes.push_back( node );
          modelsInd.push_back( bodyNodes.size() );
	    }

      /// Set Center of Mass of the Node
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
                
	            kinematics::Shape* p = bodyNodes[cnum]->getShape();
              p->setMass( mass );

	            Vector3d pos;
	            rstream >> pos(0);
	            rstream >> pos(1);
	            rstream >> pos(2);
	            bodyNodes[cnum]->setLocalCOM( pos );
          }

      } 

      /// Create the Joints and/or DOF
      else if(str == "CON") {

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
              joint = new kinematics::Joint( _robot->getRoot(), bodyNodes[cnum], "Joint" );
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

          trans = new kinematics::TrfmRotateEulerZ( new::kinematics::Dof( DEG2RAD(yaw) ) );
          joint->addTransform( trans, false );  

          trans = new kinematics::TrfmRotateEulerY( new::kinematics::Dof( DEG2RAD(pitch) ) );
          joint->addTransform( trans, false );  

          trans = new kinematics::TrfmRotateEulerX( new::kinematics::Dof( DEG2RAD(roll)) );
          joint->addTransform( trans, false );  

	        string bufAxis;
	        rstream >> bufAxis;

          string bufType;
	        rstream >> bufType;

          int type = -1; // -1: NONE 0: REVOL 1: PRISM
 
	        if( bufType == "FIXED" ) 
            {  }

	        if( bufType == "REVOL" ) {
              type = 0;
         
              if( bufAxis == "PZ" ) 
              {  trans = new kinematics::TrfmRotateEulerZ( new kinematics::Dof(0) );  }
              if( bufAxis == "PY" ) 
              {  trans = new kinematics::TrfmRotateEulerY( new kinematics::Dof(0) );  }
              if( bufAxis == "PX" ) 
              {  trans = new kinematics::TrfmRotateEulerX( new kinematics::Dof(0) );  }
                  
              joint->addTransform( trans, true );
              _robot->addTransform( trans ); 
          }

	        if(bufType == "PRISM") {
              type = 1;

              if( bufAxis == "PZ" ) 
              {  trans = new kinematics::TrfmTranslateZ( new kinematics::Dof(0) );  }
              if( bufAxis == "PY" ) 
              {  trans = new kinematics::TrfmTranslateY( new kinematics::Dof(0) );  }
              if( bufAxis == "PX" ) 
              {  trans = new kinematics::TrfmTranslateX( new kinematics::Dof(0) );  }
                  
              joint->addTransform( trans, true );
              _robot->addTransform( trans ); 
          }

	        if(bufType == "FREE") 
          { /** THIS SHOULD BE ERASED */ }

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
		
      getline( rstream, line );

    } // end of while
    rstream.close();

    for( unsigned int i=0; i < bodyNodes.size(); i++ ) {
        _robot->addNode( bodyNodes[i] ); 
    }

    for( unsigned int i = 0; i < models.size(); i++ ) {
      if( models[i] != 0 ) {
          _robot->addModel( models[i], modelsInd[i] );
          printf("Mode: %d Model Ind: %d \n", i, modelsInd[i]);
      }
    }


    //-- Init the object
    _robot->initSkel();

    return 0;
}

/**
 * @function parseObject
 * @brief Load object
 */
int parseObject( string _filename, robotics::Object *_object )
{
    const aiScene* model;
    _object->setMovable( false );

    if( _filename != "NOMODEL" ) {

      //-- TODO Create a ShapeMesh from the 3DS Model
      model = _object->loadModel( _filename );                
      kinematics::ShapeMesh *p = new kinematics::ShapeMesh( Vector3d(1, 1, 1), 0, NULL );
      _object->getRoot()->setShape( p );
    }
            
    //-- Assume an object : a unique body node
    _object->addModel( model, 0 );

    //-- Init the object
    _object->initSkel();

    return 0;
}

