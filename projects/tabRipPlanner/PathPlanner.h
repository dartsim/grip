/**
 * @file PathPlanner.h
 * @brief 
 * @datee 2011-10-24
 */
#ifndef _PATH_PLANNER_
#define _PATH_PLANNER_

#include <iostream>
#include <Eigen/Core>
#include <vector>
#include <planning/World.h>
#include "RRT.h"

/**
 * @class PathPlanner
 * @brief Basic Planner RRT-based
 */
class PathPlanner {

public:

    /// Member variables
    double stepSize;
    planning::World* world;
    bool solved;
    std::list<Eigen::VectorXd> path;
    

    /// Constructor
    PathPlanner();
    PathPlanner( planning::World &_world, 
                 bool _copyWorld = true, 
                 double _stepSize = 0.1 )
    /// Destructor
    ~PathPlanner();  

    /// planPath
    bool planPath( int _robotId, 
                   const Eigen::VectorXi &_links, 
                   const Eigen::VectorXd &_start, 
                   const Eigen::VectorXd &_goal, 
                   std::list<Eigen::VectorXd> &_path, 
                   bool _bidirectional = true, 
                   bool _connect = true, 
                   bool _smooth = true, 
                   unsigned int _maxNodes = 0 ) const;

    bool checkPathSegment(int robotId, const std::vector<int> &linkIds, const Eigen::VectorXd &config1, const Eigen::VectorXd &config2) const;
    void smoothPath(int robotId, std::vector<int> links, std::list<Eigen::VectorXd> &path) const;
    void smoothPath2( int robotId, std::vector<int> linkIds, std::list<Eigen::VectorXd> &path ) const;

private:
    /// Member variables
    bool copyWorld;

    /// Finds a plan using a standard RRT
    bool planSingleTreeRrt( int _robotId, 
                            const Eigen::VectorXi &_links, 
                            const Eigen::VectorXd &_start, 
                            const Eigen::VectorXd &_goal, 
                            std::list<Eigen::VectorXd> &_path, 
                            bool _connect, 
                            unsigned int _maxNodes ) const;

    /// Grows 2 RRT (Start and Goal)
    bool planBidirectionalRrt( int _robotId, 
                               const Eigen::VectorXi &_links, 
                               const Eigen::VectorXd &_start, 
                               const Eigen::VectorXd &_goal, 
                               std::list<Eigen::VectorXd> &_path, 
                               bool _connect, 
                               unsigned int _maxNodes ) const;

};

#endif /** _PATH_PLANNER */
