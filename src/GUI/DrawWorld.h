/**
 * @file DrawWorld.h
 */
#include "GUI.h"
#include <Eigen/Geometry>
#include <planning/Model3DS.h>

void drawWorld(); 
void drawModel( Model3DS* _model, Eigen::Transform<double, 3, Eigen::Affine> _pose );
