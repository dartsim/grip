/**
 * @file Collision.h
 */

#ifndef _GRIP_COLLISION_
#define _GRIP_COLLISION_

#include <robotics/World.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>
#include <VCollide/VCollide.h>
#include <Tools/Constants.h>

class aiScene;

enum CollisionType {
    COLLISION_OBJECT = 0,
    COLLISION_ROBOT = 1
};


/**
 * @class CollisionEntity
 */
class CollisionEntity {

public:
  CollisionEntity();
  ~CollisionEntity();

  int mType; /**< Type: Robot or Object */
  int mId;  /**< ID of mRobot or mObject in mWorld */
  int mBodyNodeId; /**< Index of BodyNode in Skeleton (again, of Robot or Object) */
  double mTrans[4][4]; /**< Transformation */
  bool mCollisionFlag; /**< Collision Flag */
  const aiScene* model;

  int mEid; /** Index in vcollide */

};


/**
 * @class Collision
 */
class Collision {

public:

  Collision();
  ~Collision();

  // Member variables
  std::vector<CollisionEntity*> mEntities;
  std::vector< std::vector<int> >mRobotsEid;
  std::vector< std::vector<int> >mObjectsEid;

  VCollide vcollide;
  VCReport vreport;
  bool mFlag;

  // Member functions
  void InitFromWorld( robotics::World* _world );
  int CreateCollisionEntity( CollisionType _type, 
                              int _id, 
                              int _nodeId, 
                              const aiScene *_model, 
                              const Eigen::MatrixXd &_pose );
  bool CheckCollisions();
  void ClearCollisions();
  void DetectCollisions();
  void UpdateAllCollisionModels();
  void UpdateCollisionModel( int _eid );
  void updateAllCollisionModels();
};

#endif /** _GRIP_COLLISION_ */
