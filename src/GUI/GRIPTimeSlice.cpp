/*
 * Copyright (c) 2010, Georgia Tech Research Corporation
 * 
 * Humanoid Robotics Lab      Georgia Institute of Technology
 * Director: Mike Stilman     http://www.golems.org
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *     * Redistributions of source code must retain the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials
 *       provided with the distribution.
 *     * Neither the name of the Georgia Tech Research Corporation nor
 *       the names of its contributors may be used to endorse or
 *       promote products derived from this software without specific
 *       prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY GEORGIA TECH RESEARCH CORPORATION ''AS
 * IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL GEORGIA
 * TECH RESEARCH CORPORATION BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "GRIPTimeSlice.h"
#include <iostream>

/**
 * @function GRIPTimeSlice
 * @brief Constructor
 */
GRIPTimeSlice::GRIPTimeSlice( planning::World *_w ) {
	
	objectsXYZ.clear(); objectsRPY.clear();
	robotsXYZ.clear(); robotsRPY.clear();
        robotBodyNodesPoses.clear(); 

        double x, y, z;
        double roll, pitch, yaw;
     
        /// Get Objects Poses
	for(unsigned int i = 0; i < _w->mObjects.size(); i++ ) {
            _w->mObjects[i]->getPositionXYZ( x, y, z);                
	    objectsXYZ.push_back( Eigen::Vector3d( x, y, z ) );

            _w->mObjects[i]->getRotationRPY( roll, pitch, yaw );                
	    objectsRPY.push_back( Eigen::Vector3d( roll, pitch, yaw ) );
	}
   
        /// Get Robot Poses
	for(unsigned int i = 0; i < _w->mRobots.size(); i++ ) {
            _w->mRobots[i]->getPositionXYZ( x, y, z);                
	    robotsXYZ.push_back( Eigen::Vector3d( x, y, z ) );

            _w->mRobots[i]->getRotationRPY( roll, pitch, yaw );                
	    robotsRPY.push_back( Eigen::Vector3d( roll, pitch, yaw ) );

            /// Get Node DOFs values
	    Eigen::VectorXd RJointVec;
            RJointVec = _w->mRobots[i]->getQuickDofs();
	    robotBodyNodesPoses.push_back( RJointVec );

	}

}

/**
 * @function ~GRIPTimeSlice
 * @brief Destructor
 */
GRIPTimeSlice::~GRIPTimeSlice() {

}

/**
 * @function SetToWorld
 */
void GRIPTimeSlice::SetToWorld( planning::World* _w ) {
	
    for(unsigned int i = 0; i < _w->mObjects.size(); i++ ) {

        _w->mObjects[i]->setPositionXYZ( objectsXYZ[i](0), objectsXYZ[i](1), objectsXYZ[i](2) );
        _w->mObjects[i]->setRotationRPY( objectsRPY[i](0), objectsRPY[i](1), objectsRPY[i](2) );
    }
    for(unsigned int i = 0; i < _w->mRobots.size(); i++ ) {

        _w->mRobots[i]->setPositionXYZ( robotsXYZ[i](0), robotsXYZ[i](1), robotsXYZ[i](2) );
        _w->mRobots[i]->setRotationRPY( robotsRPY[i](0), robotsRPY[i](1), robotsRPY[i](2) );
        _w->mRobots[i]->setQuickDofs( robotBodyNodesPoses[i] );

    }
}

