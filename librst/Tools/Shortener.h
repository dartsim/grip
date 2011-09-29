/*
 * Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *     * Redistributions of source code must retain the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials
 *       provided with the distribution.
 *     * Neither the name of the Georgia Tech Research Corporation nor
 *       the names of its contributors may be used to endorse or
 *       promote products derived from this software without specific
 *       prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY GEORGIA TECH RESEARCH CORPORATION ''AS
 * IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL GEORGIA
 * TECH RESEARCH CORPORATION BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

/**
 * @file Shortener.h
 * @brief Shortener class for path planner
 * @author Ana Huaman
 * @date 2011-09-28
 */

#include <iostream>
#include <Tools/Robot.h>
#include <ctime>
#include <list>
#include <vector>

#ifndef _SHORTENER_
#define _SHORTENER_

#define RAND12(N1,N2) N1 + ((N2-N1) * ((double)rand() / ((double)RAND_MAX + 1))) // random # between N&(M-1)

/**
 * @class Shortener
 */
class Shortener
{
   public:
     EIGEN_MAKE_ALIGNED_OPERATOR_NEW

     Shortener( World &world,
                int robotId,
                std::vector<int> linksId );
     ~Shortener();
     void Shorten( std::list< Eigen::VectorXd > &path, double stepSize );

   private:
     class GroupInfo
     {
        public:
        int group;
        std::vector<int> nodes;
        GroupInfo( int group, std::vector<int> nodes )
        {
           this->group = group;
           this->nodes = nodes;
        }
       ~GroupInfo()
       {}   
     };

     World* mWorld;
     int mRobotId;     
     std::vector<int> mLinksId;

     Eigen::VectorXd mStartConfig;
     Eigen::VectorXd mGoalConfig;   
     Eigen::Vector3d mStartPos;
     Eigen::Vector3d mGoalPos;   

     double mStepSize;

     //-- General
     bool CheckPathSegment( Eigen::VectorXd config1, Eigen::VectorXd config2 ) const;

     //-- Grouping stuff
     std::vector< GroupInfo > mGroupInfo;
     std::vector<int> mGroupID;

     void InitGroups( const std::list< Eigen::VectorXd > &path );
     void CheckGroups( );
     std::vector<Eigen::VectorXd>GetNewNodes( Eigen::VectorXd config1, Eigen::VectorXd config2, double stepSize );

};

#endif /** _SHORTENER_ */
