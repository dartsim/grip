/**
 * @file Collision.h
 */

#ifndef _GRIP_COLLISION_
#define _GRIP_COLLISION_

#include <vector>
#include <Eigen/Core>
#include <iostream>
#include <stdlib.h>
#include <VCollide/VCollide.h>
#include <planning/Model3DS.h>
#include <Tools/Constants.h>
#include <planning/World.h>

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
  std::vector<CollisionEntity> mEntities;

  VCollide vcollide;
  VCReport report;

  // Member functions
  void InitFromWorld( planning::World* _world );
  void CreateCollisionEntity( CollisionType _type, 
                              int _id, 
                              int _nodeId, 
                              Model3DS *_model, 
                              const Eigen::MatrixXd &_pose );

};

#endif /** _GRIP_COLLISION_ */
