/**
 * @file Collision.cpp
 */

#include "Collision.h"
#include <GUI/GUI.h>
#include <kinematics/BodyNode.h>

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
void Collision::InitFromWorld( planning::World* _world ) {

  printf("Reading robots Collision \n");

  // Reading robots
  mRobotsEid.resize( _world->mRobots.size() );

  /// For every robot
  for( unsigned int i = 0; i < _world->mRobots.size(); i++ ) {

    std::vector<int> mod( _world->mRobots[i]->mModels.size() );
    /// Read every model into a CollisionEntity
    for( unsigned int j = 0; j < _world->mRobots[i]->mModels.size(); j++ ) {
       
      int ind = _world->mRobots[i]->mModelIndices[j]; 
      Eigen::MatrixXd poseMatrix = _world->mRobots[i]->getNode( ind )->getWorldTransform();   

      int k = CreateCollisionEntity( COLLISION_ROBOT, i, ind, _world->mRobots[i]->mModels[j], poseMatrix );
      mod[j] = k;      
    }
    /// Save the indices
    mRobotsEid[i] = mod;
  }

  printf("Reading objects Collision \n");


  // Reading objects
  mObjectsEid.resize( _world->mObjects.size() );

  for( unsigned int i = 0; i < _world->mObjects.size(); i++ ) {
    
    std::vector<int> mod( _world->mObjects[i]->mModels.size() );
    /// Read every model into a CollisionEntity
    for( unsigned int j = 0; j < _world->mObjects[i]->mModels.size(); j++ ) {
  
      int ind = _world->mObjects[i]->mModelIndices[j];        
      Eigen::MatrixXd poseMatrix = _world->mObjects[i]->getNode( ind )->getWorldTransform();         

      int k = CreateCollisionEntity( COLLISION_OBJECT, i, ind, _world->mObjects[i]->mModels[j], poseMatrix );
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

    for( int i = 0; i < mWorld->mRobots.size(); i++ )
    { mWorld->mRobots[i]->update(); }

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
  {  pose = mWorld->mRobots[mind]->getNode(bind)->getWorldTransform(); }

  else
  {  pose = mWorld->mObjects[mind]->getNode(bind)->getWorldTransform(); }

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
                                       Model3DS *_model, 
                                       const Eigen::MatrixXd &_pose ) {

  CollisionEntity *coll = new CollisionEntity();

  coll->mType = _type;
  coll->mId = _id;
  coll->mBodyNodeId = _nodeId;
  coll->mCollisionFlag = true;

  coll->mTrans[0][0] = _pose(0,0); coll->mTrans[0][1] = _pose(0,1); coll->mTrans[0][2] = _pose(0,2); coll->mTrans[0][3] = _pose(0,3);
  coll->mTrans[1][0] = _pose(1,0); coll->mTrans[1][1] = _pose(1,1); coll->mTrans[1][2] = _pose(1,2); coll->mTrans[1][3] = _pose(1,3);
  coll->mTrans[2][0] = _pose(2,0); coll->mTrans[2][1] = _pose(2,1); coll->mTrans[2][2] = _pose(2,2); coll->mTrans[2][3] = _pose(2,3);
  coll->mTrans[3][0] = _pose(3,0); coll->mTrans[3][1] = _pose(3,1); coll->mTrans[3][2] = _pose(3,2); coll->mTrans[3][3] = _pose(3,3);
  
  std::vector<Model3DS::Triangle> *trigs = new std::vector<Model3DS::Triangle>;
  _model->ReportTriangles( trigs );


  int collCounter;
  vcollide.NewObject( &collCounter );
	printf( "--> Creating Collision entity: %d \n", collCounter );

	for(unsigned int i = 0; i < trigs->size(); i++) {
		vcollide.AddTri((*trigs)[i].v1,(*trigs)[i].v2,(*trigs)[i].v3,collCounter);
	}
	vcollide.EndObject();
	vcollide.ActivateObject( collCounter );

	trigs->clear();
	delete trigs;

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


