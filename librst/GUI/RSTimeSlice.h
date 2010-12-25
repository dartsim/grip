//---------------------------------------------------------------------
//  Copyright (c) 2009 Mike Stilman
//  All Rights Reserved.
//
//  Permission to duplicate or use this software in whole or in part
//  is only granted by consultation with the author.
//
//    Mike Stilman              mstilman@cc.gatech.edu
//
//	  Robotics and Intelligent Machines
//    Georgia Tech
//--------------------------------------------------------------------

#ifndef RSTIME_SLICE_H
#define RSTIME_SLICE_H

#include <vector>

class World;
class Object;
class Link;


using namespace std;

class RSTimeSlice
{

public:
	RSTimeSlice(World*);
	~RSTimeSlice();

	void SetToWorld(World*);

	//vector<Transform> oPose;
	//vector<Transform> rPose;
	vector< vector<double> > rJoints;
};


#endif
