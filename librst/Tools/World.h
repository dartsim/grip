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

#ifndef WORLD_H
#define WORLD_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>
#include <Tools/Collision/VCollide.h>
#include <Tools/Model3DS.h>
#include <Tools/Constants.h>

class Object;
class Robot;

class World{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	World();
	World(const World &copyFrom);
	~World();

	bool flag;
	std::vector<Robot*>	robots;
	std::vector<Object*> objects;
	std::vector<Object*> entities;

	// Collision detection
	VCollide vcollide;
	VCReport report;

	// Camera position
	Eigen::Matrix3d camRotT;
	Eigen::Vector3d worldV;
	double camRadius;
	// World colors
	Eigen::Vector3d gridColor;
	Eigen::Vector3d backColor;

	void Draw();
	void DeleteModels();
	void updateRobot(Robot* robot);
	int findRobot(string name);
	void updateAllCollisions();
	void updateCollision(Object *ob);
	void detectCollisions();
	void clearCollisions();
	bool checkCollisions();
	void planeDetect();
	void CreateEntity(Object* object);

	int Save(string);
	int Load(string);
};



#endif
