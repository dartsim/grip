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
       printf( "Not implemented yet. Is this necesary? \n" );
    } else {
        world = &_world;
    }

    stepSize = _stepSize;
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
               		    const Eigen::VectorXi &_links, 
                            const Eigen::VectorXd &_start, 
                            const Eigen::VectorXd &_goal, 
                            bool _bidirectional, 
                            bool _connect, 
                            bool _greedy,
                            bool _smooth, 
                            unsigned int _maxNodes ) {

    printf( "Planner Summary: \n" );
    printf("Links Index: \n");
    for( int i = 0; i < _links.size(); i++ )
    { printf(" %d ", _links(i) ); }
    printf("\n");
    
    printf("Start: \n");
    for( int i = 0; i < _start.size(); i++ )
    { printf(" %.3f ", _start(i) ); }
    printf("\n");

    printf("Goal: \n");
    for( int i = 0; i < _goal.size(); i++ )
    { printf(" %.3f ", _goal(i) ); }
    printf("\n");

    printf(" Bidirectional: %d \n", _bidirectional );
    printf(" Connect: %d \n", _connect );
    printf(" Greedy: %d \n", _greedy );
    printf(" Smooth: %d \n", _smooth );
    printf("  MAX nodes: %d \n", _maxNodes );

    //world->mRobots[_robotId]->setQuickDofs( _start ); // Other quick way
    world->mRobots[_robotId]->setDofs( _start, _links );
    if( world->checkCollisions() )
        return false;

    world->mRobots[_robotId]->setDofs( _goal, _links );
    if( world->checkCollisions() )
        return false;
	
    bool result;
    if( _bidirectional ) { 
        result = planBidirectionalRrt( _robotId, _links, _start, _goal, _connect, _greedy, _maxNodes ); 
    } else {
        result = planSingleTreeRrt( _robotId, _links, _start, _goal, _connect, _greedy, _maxNodes );
    }

    if( result && _smooth ) {
        smoothPath( _robotId, _links, path );
    }

    return result;
}


/**
 * @function planSingleRRT
 * @brief Finds a plan using a standard RRT
 */
bool PathPlanner::planSingleTreeRrt( int _robotId, 
                                     const Eigen::VectorXi &_links, 
                                     const Eigen::VectorXd &_start, 
                                     const Eigen::VectorXd &_goal, 
                                     bool _connect, 
                                     bool _greedy,
                                     unsigned int _maxNodes ) {

    RRT rrt( world, _robotId, _links, _start, stepSize );
    RRT::StepResult result = RRT::STEP_PROGRESS;

    double smallestGap = DBL_MAX;

    while ( result != RRT::STEP_REACHED ) {

        if( _connect ) {
            printf("No goal \n");
	    rrt.connect();
	    if( rrt.connect( _goal ) ) {
                printf("Trying goal \n");
	        result = RRT::STEP_REACHED;
	    }

	} else {
            printf("Common \n");
	    rrt.tryStep();
	    result = rrt.tryStep( _goal );
	}

	if( _maxNodes > 0 && rrt.getSize() > _maxNodes ) {
	    return false;
        }

	double gap = rrt.getGap( _goal );
	if( gap < smallestGap ) {
	    smallestGap = gap;
	    cout << "Gap: " << smallestGap << "    Tree size: " << rrt.configVector.size() << endl;
	}
    }
    /// Save path to path member variable of PathPlanner 
    printf(" Reached path! Gap: %.3f \n", rrt.getGap( _goal ) );
    rrt.tracePath( rrt.activeNode, path, false );
   
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
                                        bool _connect,
                                        bool _greedy, 
                                        unsigned int _maxNodes ) {
    // ================================================//
    // YOUR CODE HERE                                  //
    // ================================================//	
    RRT rrt_a( world, _robotId, _links, _start, stepSize );
    RRT rrt_b( world, _robotId, _links, _goal, stepSize );
    return true;
}


/**
 * @function checkPathSegment
 * @brief True iff collision-free
 */
bool PathPlanner::checkPathSegment( int _robotId, 
                                    const Eigen::VectorXi &_links, 
                                    const Eigen::VectorXd &_config1, 
                                    const Eigen::VectorXd &_config2 ) const {

    int n = (int)((_config2 - _config1).norm() / stepSize );

    for( int i = 0; i < n; i++ ) {
        Eigen::VectorXd conf = (double)(n - i)/(double)n * _config1 + (double)(i)/(double)n * _config2;
        world->mRobots[_robotId]->setDofs( conf, _links );
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
                              const Eigen::VectorXi &_links, 
                              std::list<Eigen::VectorXd> &_path ) {

    std::list<Eigen::VectorXd>::iterator config1, config2;
    std::list<Eigen::VectorXd>::iterator temp = _path.begin();

    if( temp == _path.end() ) return;

    while(true) {
        config1 = temp;
	temp++;
	if(temp == _path.end()) return;
	config2 = temp;
	config2++;
	if(config2 == _path.end()) return;
		
	while( checkPathSegment( _robotId, _links, *config1, *config2) ) {
	    _path.erase( temp );
	    temp = config2;
	    config2++;
	    if( config2 == _path.end() ) return;
	}
    }

}

/**
 * @function smoothPath2
 */
void PathPlanner::smoothPath2( int _robotId, 
                               const Eigen::VectorXi &_links, 
                               std::list<Eigen::VectorXd> &_path ) {

    srand(time(NULL));

    int node_1; int node_2; int aux_node;

    int num_points = _path.size();
    int num_checks = (int) num_points*1;

    // Number of checks
    for( int i = 0; i < num_checks; i++ )
    {
       if( _path.size() < 5 ) { return; } //-- No way we can reduce something leaving out the extremes

       int minNode = 0;
       int maxNode = _path.size() - 1;

       node_1 = (int) randomInRange( minNode + 1, maxNode - 1 );

       do{ node_2 = randomInRange( minNode + 1, maxNode - 1 ); } while( node_2 == node_1 );

       if( node_2 < node_1 ) 
       {  aux_node = node_1;
          node_1 = node_2;
          node_2 = aux_node; }
      
       //-- Check
       list<Eigen::VectorXd>::iterator n1 = _path.begin();
       list<Eigen::VectorXd>::iterator n2 = _path.begin();
       advance( n1, node_1 - 1 );
       advance( n2, node_2 - 1 );

       bool result = checkPathSegment( _robotId, _links, *n1, *n2 );
       if( result == true )
       { int times = node_2 - node_1 - 1;
         for( int j = 0; j < times; j++ )
         { std::list<Eigen::VectorXd>::iterator temp = _path.begin(); 
           advance( temp, node_1 + 1 );
           _path.erase( temp );  }
       }
    }   

}
