/**
 * @file utils.cpp
 * @brief Implementation of utility functions
 * @author A. Huaman
 * @date 2011-10-25
 */

#include "utils.h"
#include <fstream>
#include "../GUI/GUI.h"
#include <robotics/Robot.h>
#include <robotics/Object.h>
#include <dynamics/BodyNodeDynamics.h>
#include <kinematics/Joint.h>
#include <kinematics/Dof.h>
#include <Tools/Constants.h>

using namespace std;

/**
 * @function saveRscene
 * @brief Save your world in its current state in a .rscene file
 */
int saveRscene( std::string filename ) {

    ofstream wstream( filename.c_str() );

    if (!wstream.is_open()) return 1;


    /// Save Robots
    if( mWorld->getNumRobots() > 0 ) {

        wstream << "##### ROBOTS #####" << endl;
	wstream << endl;
	unsigned int i = 0;
	while( i < mWorld->getNumRobots() ) {

	    wstream << "##### ROBOT " << i+1 << " #####" << endl;
	    wstream << endl;
            wstream << "> \t" << "ROBOT " << mWorld->getRobot(i)->getName() << " " << mWorld->getRobot(i)->getPathName() << endl;
           
            double x; double y; double z;
            mWorld->getRobot(i)->getPositionXYZ( x, y, z);
	    wstream << "> \t" << "POSITION " << x << " " << y << " " << z << endl;
		
            double roll; double pitch; double yaw;
            mWorld->getRobot(i)->getRotationRPY( roll, pitch, yaw);	
            wstream << "> \t" << "ORIENTATION " << RAD2DEG(roll) << " " << RAD2DEG(pitch) << " " << RAD2DEG(yaw) << endl;
	    wstream << endl;

	    wstream << "##### INITIAL ANGLES #####" << endl;
	    wstream << endl;
           
            Eigen::VectorXd dof_vals;
            dof_vals = mWorld->getRobot(i)->getQuickDofs();

            Eigen::VectorXi dof_ind;
            dof_ind = mWorld->getRobot(i)->getQuickDofsIndices();

            if( dof_vals.size() != dof_ind.size() ) {
                printf(" --(!) Something really wrong is going on here. Check Robot class in DART...Exiting \n");
                return 1;
            }

            int lnum = 0;
	    while( lnum < dof_vals.size() ) {
	        wstream << "> \t" << "INIT " << mWorld->getRobot(i)->getDof(dof_ind(lnum))->getJoint()->getChildNode()->getName() << " " << dof_vals(lnum) << endl;
		lnum++;
	    }
	    wstream << endl;
	    i++;
	}
    }

    /// Save Objects
    if( mWorld->getNumObjects() > 0 ) {
 
        wstream << "##### OBJECTS #####" << endl;
	wstream << endl;
	unsigned int i = 0;
	while( i < mWorld->getNumObjects() ) {

	    wstream << "##### OBJECT " << i+1 << " #####" << endl;
	    wstream << endl;
	    wstream << "> \t" << "OBJECT " << mWorld->getObject(i)->getName() << " " << mWorld->getObject(i)->getPathName() << endl;

            double x; double y; double z;
            mWorld->getObject(i)->getPositionXYZ( x, y, z);
	    wstream << "> \t" << "POSITION " << x << " " << y << " " << z << endl;

            double roll; double pitch; double yaw;
            mWorld->getObject(i)->getRotationRPY( roll, pitch, yaw);	 
            wstream << "> \t" << "ORIENTATION " << RAD2DEG(roll) << " " << RAD2DEG(pitch) << " " << RAD2DEG(yaw) << endl;
	    wstream << endl;
	    i++;
	}
    }

    /// Output Camera and Colors
    wstream << "##### SCENE INFO" << endl << endl;
    wstream << "> CAMERA" << endl;
    double roll = atan2( mCamRotT(2,1), mCamRotT(2,2) );
    double pitch = -asin( mCamRotT(2,0) );
    double yaw = atan2( mCamRotT(1,0), mCamRotT(0,0) );

    wstream << "ROT " << RAD2DEG(roll) << " " << RAD2DEG(pitch) << " " << RAD2DEG(yaw) << endl;
    wstream << "WRT " << mWorldV[0] << " " << mWorldV[1] << " " << mWorldV[2] << endl;
    wstream << "RAD " << mCamRadius << endl << endl;

    wstream << "> COLORS" << endl;
    wstream << "BACK " << mBackColor[0] << " " << mBackColor[1] << " " << mBackColor[2] << endl;
    wstream << "GRID " << mGridColor[0] << " " << mGridColor[1] << " " << mGridColor[2] << endl;

    wstream.close();
    return 0;
}
