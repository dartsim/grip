/**
 * @file PathPlanner.cpp
 */
#include "PathPlanner.h"

/**
 * @function PathPlanner
 * @brief Constructor
 */
PathPlanner::PathPlanner() {
    copyWorld = false;
    world = NULL;
}

/**
 * @function PathPlanner
 * @brief Constructor
 */
PathPlanner::PathPlanner( planning::World &_world, bool _copyWorld, double _stepSize ) {

    copyWorld = _copyWorld;

    if( copyWorld ) {
        world = new planning::World( _world ); // Not sure if this will work
    } else {
        world = &_world;
    }

    stepsize = _stepSize;
}

/**
 * @function ~PathPlanner
 * @brief Destructor
 */
PathPlanner::~PathPlanner() {

    if( copyWorld ) {
        delete world;
    }
}

/**
 * @function planPath
 * @brief Main function
 */
bool PathPlanner::planPath( int _robotId, 
               		    const std::vector<int> &_links, 
                            const Eigen::VectorXd &_start, 
                            const Eigen::VectorXd &_goal, 
                            std::list<Eigen::VectorXd> &_path, 
                            bool _bidirectional = true, 
                            bool _connect = true, 
                            bool _greedy = false;
                            bool _smooth = true, 
                            unsigned int _maxNodes = 0 ) {

    world->mRobots[_robotId]->setConf( _links, _start );

    if( world->checkCollisions() )
        return false;

    world->mRobots[_robotId]->setConf( _links, _goal );

    if( world->checkCollisions() )
        return false;
	
    bool result;
    if( bidirectional ) { 
        result = planBidirectionalRrt(robotId, links, start, goal, path, connect, maxNodes); 
    } else {
        result = planSingleTreeRrt(robotId, links, start, goal, path, connect, maxNodes);
    }

    if( result && _smooth ) {
        smoothPath( _robotId, _links, _path );
    }

    return result;
}

/**
 * @function checkPathSegment
 * @brief True iff collision-free
 */
bool PathPlanner::checkPathSegment( int _robotId, 
                                    const std::vector<int> &_linkIds, 
                                    const Eigen::VectorXd &_config1, 
                                    const Eigen::VectorXd &_config2 ) {

    int n = (int)((_config2 - _config1).norm() / stepSize );

    for( int i = 0; i < n; i++ ) {
        VectorXd conf = (double)(n - i)/(double)n * _config1 + (double)(i)/(double)n * _config2;
	world->robots[robotId]->setConf(linkIds, conf, true);
	if( world->checkCollisions() ) {
	    return false;
	}
    }

    return true;
}

/**
 * @function smoothPath
 */
void PathPlanner::smoothPath( int _robotId, 
                              std::vector<int> _links, 
                              std::list<Eigen::VectorXd> &_path ) {

    list<VectorXd>::iterator config1, config2;
    list<VectorXd>::iterator temp = path.begin();
    if(temp == path.end()) return;

    while(true) {
        config1 = temp;
	temp++;
	if(temp == path.end()) return;
	config2 = temp;
	config2++;
	if(config2 == path.end()) return;
		
	while(checkPathSegment(robotId, linkIds, *config1, *config2)) {
	    path.erase(temp);
	    temp = config2;
	    config2++;
	    if(config2 == path.end()) return;
	}
    }

}

/**
 * @function smoothPath2
 */
void PathPlanner::smoothPath2( int _robotId, 
                               std::vector<int> _linkIds, 
                               std::list<Eigen::VectorXd> &_path ) {

    srand(time(NULL));

    int node_1; int node_2; int aux_node;

    int num_points = path.size();
    int num_checks = (int) num_points*1;

    // Number of checks
    for( int i = 0; i < num_checks; i++ )
    {
       if( path.size() < 5 ) { return; } //-- No way we can reduce something leaving out the extremes

       int minNode = 0;
       int maxNode = path.size() - 1;

       node_1 = (int) randomInRange( minNode + 1, maxNode - 1 );

       do{ node_2 = (int) randomInRange( minNode + 1, maxNode - 1 ); } while( node_2 == node_1 );

       if( node_2 < node_1 ) 
       {  aux_node = node_1;
          node_1 = node_2;
          node_2 = aux_node; }
      
       //-- Check
       list<Eigen::VectorXd>::iterator n1 = path.begin();
       list<Eigen::VectorXd>::iterator n2 = path.begin();
       advance( n1, node_1 - 1 );
       advance( n2, node_2 - 1 );

       bool result = checkPathSegment( robotId, linkIds, *n1, *n2 );
       if( result == true )
       { int times = node_2 - node_1 - 1;
         for( int j = 0; j < times; j++ )
         { list<Eigen::VectorXd>::iterator temp = path.begin(); 
           advance( temp, node_1 + 1 );
           path.erase( temp );  }
       }
    }   

}

/**
 * @function planSingleRRT
 * @brief Finds a plan using a standard RRT
 */
bool PathPlanner::planSingleTreeRrt( int _robotId, 
                                     const Eigen::VectorXi &_links, 
                                     const Eigen::VectorXd &_start, 
                                     const Eigen::VectorXd &_goal, 
                                     std::list<Eigen::VectorXd> &_path, 
                                     bool _connect, 
                                     unsigned int _maxNodes ) {

    RRT rrt( world, _robotId, _links, _start, stepSize );
    RRT::StepResult result = RRT::STEP_PROGRESS;

    double smallestGap = DBL_MAX;

    while ( result != RRT::STEP_REACHED ) {

        if( _connect ) {
	    rrt.connect();
	    if( rrt.connect(_goal) ) {
	        result = RRT::STEP_REACHED;
	    }

	} else {
	    rrt.tryStep();
	    result = rrt.tryStep(goal);
	}

	if( _maxNodes > 0 && rrt.getSize() > _maxNodes ) {
	    return false;
        }

	double gap = rrt.getGap(goal);
	if( gap < smallestGap ) {
	    smallestGap = gap;
	    cout << "Gap: " << smallestGap << "    Tree size: " << rrt.configVector.size() << endl;
	}
    }

    rrt.tracePath( rrt.activeNode, _path );
    return true;
}

/**
 * @function planBidirectionalRRT
 * @brief Grows 2 RRT (Start and Goal)
 */
bool PathPlanner::planBidirectionalRrt( int _robotId, 
                                        const Eigen::VectorXi &_links, 
                                        const Eigen::VectorXd &_start, 
                                        const Eigen::VectorXd &_goal, 
                                        std::list<Eigen::VectorXd> &_path, 
                                        bool _connect, 
                                        unsigned int _maxNodes ) {

    RRT start_rrt( world, _robotId, _links, _start, stepSize );
    RRT goal_rrt( world, _robotId, _links, _goal, stepSize );

    RRT* rrt1 = &start_rrt;
    RRT* rrt2 = &goal_rrt;
	
    double smallestGap = DBL_MAX;
    bool connected = false;
    while(!connected) {
        RRT* temp = rrt1;
	rrt1 = rrt2;
	rrt2 = temp;

	if( connect ) {
	    rrt1->connect();
	    //rrt1->tryStep();
	    connected = rrt2->connect( rrt1->configVector[rrt1->activeNode] );
	} else {
	    rrt1->tryStep();
	    connected = (RRT::STEP_REACHED == rrt2->tryStep(rrt1->configVector[rrt1->activeNode]));
	}

	if( _maxNodes > 0 && rrt1->getSize() + rrt2->getSize() > _maxNodes ) {
	    return false; }

	double gap = rrt2->getGap( rrt1->configVector[rrt1->activeNode] );
	if( gap < smallestGap ) {
	    smallestGap = gap;
	    cout << "Gap: " << smallestGap << "    Tree sizes: " << start_rrt.configVector.size() << "/" << goal_rrt.configVector.size() << endl;
	}
    }
	
    start_rrt.tracePath( start_rrt.activeNode, path );
    goal_rrt.tracePath( goal_rrt.activeNode, path, true );
	
    return true;
}
