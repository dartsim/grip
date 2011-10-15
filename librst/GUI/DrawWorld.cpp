/**
 * @file DrawWorld.cpp
 */
#include <kinematics/Primitive.h>
#include <kinematics/BodyNode.h>
#include <kinematics/PrimitiveMesh.h>
//#include <planning/>
#include "DrawWorld.h"

/**
 * @function DrawWorld
 */
void drawWorld() {
    printf("Drawing world \n");
    for( unsigned int i = 0; i < mWorld->mObjects.size(); i++ ) {  

        for( unsigned int j = 0; j < mWorld->mObjects[i]->getNumNodes(); j++ ) {
    
        } 
    }

}
