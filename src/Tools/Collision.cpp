/**
 * @file Collision.cpp
 */


#include "Collision.h"
#include <GUI/GUI.h>
#include <dynamics/BodyNodeDynamics.h>
#include <assimp/scene.h>

/**
 * @function Collision
 * @brief Constructor
 */
Collision::Collision() {

  mFlag = true;
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
void Collision::InitFromWorld( robotics::World* _world ) {

  printf("Reading robots Collision \n");

  // Reading robots
  mRobotsEid.resize( _world->getNumRobots() );

  /// For every robot
  for( unsigned int i = 0; i < _world->getNumRobots(); i++ ) {

    std::vector<int> mod( _world->getRobot(i)->getNumModels() );
    /// Read every model into a CollisionEntity
    for( unsigned int j = 0; j < _world->getRobot(i)->getNumModels(); j++ ) {
       
      int ind = _world->getRobot(i)->getModelIndex(j); 
      Eigen::MatrixXd poseMatrix = _world->getRobot(i)->getNode( ind )->getWorldTransform();   

      int k = CreateCollisionEntity( COLLISION_ROBOT, i, ind, _world->getRobot(i)->getModel(j), poseMatrix );
      mod[j] = k;      
    }
    /// Save the indices
    mRobotsEid[i] = mod;
  }

  printf("Reading objects Collision \n");


  // Reading objects
  mObjectsEid.resize( _world->getNumObjects() );

  for( unsigned int i = 0; i < _world->getNumObjects(); i++ ) {
    
    std::vector<int> mod( _world->getObject(i)->getNumModels() );
    /// Read every model into a CollisionEntity
    for( unsigned int j = 0; j < _world->getObject(i)->getNumModels(); j++ ) {
  
      int ind = _world->getObject(i)->getModelIndex(j);        
      Eigen::MatrixXd poseMatrix = _world->getObject(i)->getNode( ind )->getWorldTransform();         

      int k = CreateCollisionEntity( COLLISION_OBJECT, i, ind, _world->getObject(i)->getModel(j), poseMatrix );
      mod[j] = k;
    }
    /// Save the indices
    mObjectsEid[i] = mod;
  }

}

/**
 * @function CheckCollisions
 * @brief Returns true iff collision
 */
bool Collision::CheckCollisions() {
    UpdateAllCollisionModels();
    VCReport report;
    vcollide.Collide( &report, VC_FIRST_CONTACT);  /// Perform collision test.

	for (int j = 0; j < report.numObjPairs(); j++) {
        mFlag = true;
		    // Object* object1 = entities[report.obj1ID(j)];
		    // Object* object2 = entities[report.obj2ID(j)];
		    mEntities[report.obj1ID(j)]->mCollisionFlag = true;
		    mEntities[report.obj2ID(j)]->mCollisionFlag = true;
			//cout << "COLLIDED " <<  mEntities[report.obj1ID(j)]->mId << " " <<  mEntities[report.obj2ID(j)]->mId << endl;
		    //cout << "COLL: "   << entities[report.obj1ID(j)].object->name<< " : " << entities[report.obj2ID(j)].object->name<<endl;
    }
	return report.numObjPairs() > 0;
}

/**
 * @function ClearCollision
 */

void Collision::ClearCollisions() {
    for (unsigned int i = 0; i < mEntities.size(); i++) {
		    mEntities[i]->mCollisionFlag = false;
    }
}

/**
 * @function DetectCollisions
 */
void Collision::DetectCollisions() {

    VCReport report;
    vcollide.Collide(&report, VC_FIRST_CONTACT); /// Perform collision test.

	  ClearCollisions();

    for (int j = 0; j < report.numObjPairs(); j++) {
        mFlag = true;
		    // Object* object1 = entities[report.obj1ID(j)];
		    // Object* object2 = entities[report.obj2ID(j)];
		    mEntities[report.obj1ID(j)]->mCollisionFlag = true;
		    mEntities[report.obj2ID(j)]->mCollisionFlag = true;
		    //cout << "COLL: "   << entities[report.obj1ID(j)].object->name<< " : " << entities[report.obj2ID(j)].object->name<<endl;
    }
}


/**
 * @function UpdateAllCollisionModels
 */
void Collision::UpdateAllCollisionModels() {

    for( unsigned int i = 0; i < mWorld->getNumRobots(); i++ )
    { mWorld->getRobot(i)->update(); }

    /// Update all the models ( objects + robots )
    for( unsigned int i = 0; i < mEntities.size(); i++ ) {
		    UpdateCollisionModel( i );
	  }

    /// Deactivate collisions between neighboring nodes in the robot
    for( unsigned int i = 0; i < mRobotsEid.size(); i++ ) {
		
		  for( unsigned int j = 0; j< mRobotsEid[i].size(); j++) {
			  for( unsigned int k = j; k < mRobotsEid[i].size(); k++) {
				  vcollide.DeactivatePair( mRobotsEid[i][j], mRobotsEid[i][k] );
				}
			}
		}
      
}

/**
 * @function UpdateCollisionModel
 */
void Collision::UpdateCollisionModel( int _mEntityIndex ) {


  int eid = mEntities[_mEntityIndex]->mEid;
  int mind = mEntities[_mEntityIndex]->mId;
  int bind = mEntities[_mEntityIndex]->mBodyNodeId;
 
  Eigen::MatrixXd pose;

  if( mEntities[_mEntityIndex]->mType == COLLISION_ROBOT )
  {  pose = mWorld->getRobot(mind)->getNode(bind)->getWorldTransform(); }

  else
  {  pose = mWorld->getObject(mind)->getNode(bind)->getWorldTransform(); }

	double newTrans[4][4] =
	{ {pose(0,0), pose(0,1), pose(0,2), pose(0,3)},
	  {pose(1,0), pose(1,1), pose(1,2), pose(1,3)},
	  {pose(2,0), pose(2,1), pose(2,2), pose(2,3)},
	  {0, 0, 0, 1} };

	vcollide.UpdateTrans( eid, newTrans );
	mEntities[_mEntityIndex]->mCollisionFlag = false;
}


/**
 * @function CreateCollisionEntity
 */
int Collision::CreateCollisionEntity( CollisionType _type, 
                                       int _id, 
                                       int _nodeId, 
                                       const aiScene *_model, 
                                       const Eigen::MatrixXd &_pose ) {

  CollisionEntity *coll = new CollisionEntity();

  coll->mType = _type;
  coll->mId = _id;
  coll->mBodyNodeId = _nodeId;
  coll->mCollisionFlag = true;
  coll->model = _model;

  coll->mTrans[0][0] = _pose(0,0); coll->mTrans[0][1] = _pose(0,1); coll->mTrans[0][2] = _pose(0,2); coll->mTrans[0][3] = _pose(0,3);
  coll->mTrans[1][0] = _pose(1,0); coll->mTrans[1][1] = _pose(1,1); coll->mTrans[1][2] = _pose(1,2); coll->mTrans[1][3] = _pose(1,3);
  coll->mTrans[2][0] = _pose(2,0); coll->mTrans[2][1] = _pose(2,1); coll->mTrans[2][2] = _pose(2,2); coll->mTrans[2][3] = _pose(2,3);
  coll->mTrans[3][0] = _pose(3,0); coll->mTrans[3][1] = _pose(3,1); coll->mTrans[3][2] = _pose(3,2); coll->mTrans[3][3] = _pose(3,3);
  
  int collCounter;
  vcollide.NewObject( &collCounter );
	printf( "--> Creating Collision entity: %d \n", collCounter );

	for(unsigned int i = 0; i < _model->mNumMeshes; i++) {
		for(unsigned int j = 0; j < _model->mMeshes[i]->mNumFaces; j++) {
			double vertices[3][3];
			for(unsigned int k = 0; k < 3; k++) {
				const aiVector3D& vertex = _model->mMeshes[i]->mVertices[_model->mMeshes[i]->mFaces[j].mIndices[k]];
				vertices[k][0] = vertex.x;
				vertices[k][1] = vertex.y;
				vertices[k][2] = vertex.z;
			}
			vcollide.AddTri(vertices[0], vertices[1], vertices[2]);
		}
	}

	vcollide.EndObject();
	vcollide.ActivateObject( collCounter );

	//MUST BE TRUE
	assert( (int)mEntities.size() == collCounter);

	coll->mEid = collCounter;

	mEntities.push_back( coll );  

  return collCounter;
}

/////////////////////////////////////////////////////

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


