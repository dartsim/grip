/**
 * @file Collision.cpp
 */

#include "Collision.h"
#include <kinematics/BodyNode.h>

/**
 * @function Collision
 * @brief Constructor
 */
Collision::Collision() {

}

/**
 * @function ~Collision
 * @brief Destructor
 */
Collision::~Collision() {

}


/**
 * @function InitFromWorld
 */  
void Collision::InitFromWorld( planning::World* _world ) {

  printf("Reading robots Collision \n");
  // Reading robots
  for( unsigned int i = 0; i < _world->mRobots.size(); i++ ) {

    for( unsigned int j = 0; j < _world->mRobots[i]->mModels.size(); j++ ) {
       
      int ind = _world->mRobots[i]->mModelIndices[j]; 
      Eigen::MatrixXd poseMatrix = _world->mRobots[i]->getNode( ind )->getWorldTransform();   

      CreateCollisionEntity( COLLISION_ROBOT, i, ind, _world->mRobots[i]->mModels[j], poseMatrix );
    }

  }

  printf("Reading objects Collision \n");
  // Reading objects
  for( unsigned int i = 0; i < _world->mObjects.size(); i++ ) {
    
    for( unsigned int j = 0; j < _world->mObjects[i]->mModels.size(); j++ ) {
  
      int ind = _world->mObjects[i]->mModelIndices[j];        
      Eigen::MatrixXd poseMatrix = _world->mObjects[i]->getNode( ind )->getWorldTransform();         

      CreateCollisionEntity( COLLISION_OBJECT, i, ind, _world->mObjects[i]->mModels[j], poseMatrix );
    }

  }


}

/**
 * @function CreateCollisionEntity
 */
void Collision::CreateCollisionEntity( CollisionType _type, 
                                       int _id, 
                                       int _nodeId, 
                                       Model3DS *_model, 
                                       const Eigen::MatrixXd &_pose ) {

  printf("Create Collision Entity \n");
  CollisionEntity coll;

  coll.mType = _type;
  coll.mId = _id;
  coll.mBodyNodeId = _nodeId;
  coll.mCollisionFlag = true;

  coll.mTrans[0][0] = _pose(0,0); coll.mTrans[0][1] = _pose(0,1); coll.mTrans[0][2] = _pose(0,2); coll.mTrans[0][3] = _pose(0,3);
  coll.mTrans[1][0] = _pose(1,0); coll.mTrans[1][1] = _pose(1,1); coll.mTrans[1][2] = _pose(1,2); coll.mTrans[1][3] = _pose(1,3);
  coll.mTrans[2][0] = _pose(2,0); coll.mTrans[2][1] = _pose(2,1); coll.mTrans[2][2] = _pose(2,2); coll.mTrans[2][3] = _pose(2,3);
  coll.mTrans[3][0] = _pose(3,0); coll.mTrans[3][1] = _pose(3,1); coll.mTrans[3][2] = _pose(3,2); coll.mTrans[3][3] = _pose(3,3);
  
  std::vector<Model3DS::Triangle> *trigs = new std::vector<Model3DS::Triangle>;
  _model->ReportTriangles( trigs );

  printf("Num trigs: %d \n", trigs->size() );
/*
  printf("Here 3 \n");
  int collCounter;

  printf("Here 4 \n");
  int ana;
  int a = vcollide.NewObject( &ana );

  collCounter = 0;

  printf("a: %d \n", a);

  printf("Here 5 \n");
	for( unsigned int i = 0; i < trigs->size(); i++ ) {
	  vcollide.AddTri( (*trigs)[i].v1,(*trigs)[i].v2,(*trigs)[i].v3, collCounter );
	}
  printf("Here 6 \n");

	vcollide.EndObject();
	vcollide.ActivateObject( collCounter );
*/
	trigs->clear();
	delete trigs;
/*
  printf("Here 7 \n");
  printf("--> CollCounter: %d \n", collCounter );

	//MUST BE TRUE
	assert( (int)mEntities.size() == collCounter);

	coll.mEid = collCounter;

	mEntities.push_back( coll );  
*/
}

/**
 * @function CollisionEntity
 * @brief Constructor
 */
CollisionEntity::CollisionEntity() {

}

/**
 * @function ~CollisionEntity
 * @brief Destructor
 */
CollisionEntity::~CollisionEntity() {

}


