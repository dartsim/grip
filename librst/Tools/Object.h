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

#ifndef OBJECT_H
#define OBJECT_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Tools/Model3DS.h>
#include <Tools/Constants.h>
#include <string>

//Defined here are the different geometric primitives
enum{ NONE,SPHERE,BOX,RECT_FACE };

extern const double threshold;

class World;

class Object{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	Eigen::Transform<double, 3, Eigen::Affine> absPose;
	Eigen::Matrix<double, 3, 3>	inertia;
	Eigen::Vector3d	COM;
	Eigen::Vector3d vertices[8]; //for Box(8) and Rect_Face(4) in object coordinates
	Eigen::Vector3d normal;

	World* world;
	Model3DS	*model;

	string pathname;
	string name;
	bool movable;
	int idNum;
	int eid;
	double mass;
	int primitiveType; // geometric primitive type
	double radius; //for sphere
	double d;
	double vel_t;
	bool collisionFlag;
	bool comFlag;

	Object();
	~Object();

	Model3DS* LoadModel(string);

	void Draw();
	void DrawPrimitive(); //will draw the primitive object
};



#endif
