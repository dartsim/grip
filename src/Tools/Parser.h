/**
 * @file Parser.h
 * @author A. Huaman
 * @date 2011-10-06
 * @brief Load from RSDH to dart
 */

#ifndef PARSER_H
#define PARSER_H

#include <cstddef>
#include <fstream>
#include <iostream>
#include <cstdio>
#include <string>

#include <Tools/Constants.h>
#include <kinematics/ShapeMesh.h>
#include <robotics/Robot.h>
#include <robotics/Object.h>
#include <robotics/World.h>
#include <robotics/Model3D.h>


robotics::World* parseWorld( std::string filename );
int parseRobot( std::string _filename, robotics::Robot *_robot );
int parseObject( std::string _filename, robotics::Object *_object );

#endif /** PARSER_H */
