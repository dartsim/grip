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
 * @file Shortener.cpp
 * @brief Implementation of Shortener class 
 * @author Ana Huaman
 * @date 2011-09-28
 */
#include "Shortener.h"

/**
 * @function Shortener
 */
Shortener::Shortener( World &world, 
                      int robotId,
                      std::vector<int> linksId )
{
   mWorld = &world;
   mRobotId = robotId;
   mLinksId = linksId;
}

/**
 * @function ~Shortener
 * @brief Destructor
 */
Shortener::~Shortener()
{}

/**
 * @function CheckPathSegment
 */
bool Shortener::CheckPathSegment( Eigen::VectorXd config1, Eigen::VectorXd config2 ) const {
	int n = (int)((config2 - config1).norm() / mStepSize);
	for(int i = 0; i < n; i++) {
		Eigen::VectorXd conf = (double)(n - i)/(double)n * config1 + (double)(i)/(double)n * config2;
		mWorld->robots[mRobotId]->setConf( mLinksId, conf, true);
		if( mWorld->checkCollisions()) {
			return false;
		}
	}
	return true;
}


/**
 * @function Shorten
 * @brief Plain explanation. It shortens
 */
void Shortener::Shorten( std::list< Eigen::VectorXd > &path, double stepSize )
{
   int g1; int g2;
   int gInd;
   int numPoints; int numChecks;

   int n1; int n2; int naux;

   //-- 0. Initialize stuff
   srand(time(NULL));
   mStepSize = stepSize;
   numPoints = path.size();
   numChecks = (int) numPoints*1.0;

   //-- 1. Init the groups and check them
   InitGroups( path );
   gInd = 0; 

   //-- 2. Check numCheck times
   for( int checks = 0; checks < numChecks; checks++ )
   {

      CheckGroups();

      //-- The path is a line, nothing else to do
      if( mGroupInfo.size() == 1 && mGroupInfo[0].group != 0 )
      { printf("--oo Straight line \n"); break; }

      do
      {  g1 = RAND12( 0, mGroupInfo.size() );
         g2 = RAND12( 0, mGroupInfo.size() );
      }while( mGroupInfo[g1].group == mGroupInfo[g2].group && 
              (mGroupInfo[g1].group != 0 || mGroupInfo[g2].group != 0 || mGroupInfo[g2].nodes.size() <= 1 ) );

      do
      {
         n1 = mGroupInfo[g1].nodes[ RAND12( 0, mGroupInfo[g1].nodes.size() ) ];
         n2 = mGroupInfo[g2].nodes[ RAND12( 0, mGroupInfo[g2].nodes.size() ) ];
      }while( n1 == n2 );

      if( n2 < n1 ){ naux = n1; n1 = n2; n2 = naux; }

      std::list<Eigen::VectorXd>::iterator ip1 = path.begin(); advance( ip1, n1 );
      std::list<Eigen::VectorXd>::iterator ip2 = path.begin(); advance( ip2, n2 );
      std::list<Eigen::VectorXd>::iterator ip;
     
      //-- If no collisions  
      if( CheckPathSegment(*ip1, *ip2) == true )
      {
         //-- Get new nodes
         std::vector<Eigen::VectorXd> newNodes = GetNewNodes( *ip1, *ip2, mStepSize );
         std::vector<Eigen::VectorXd>::iterator inn1 = newNodes.begin();
         std::vector<Eigen::VectorXd>::iterator inn2 = newNodes.end();

         //-- Update GroupID 
         gInd++;
         std::vector<int>::iterator ig1 = mGroupID.begin(); advance( ig1, n1 );
         std::vector<int>::iterator ig2 = mGroupID.begin(); advance( ig2, n2 );
         std::vector<int>::iterator ig;
         //-- Erase old nodes
         ig2++;
         ig = mGroupID.erase( ig1, ig2 );
      
         //-- Add nodes on segment
         mGroupID.insert( ig, newNodes.size(), gInd );

         //-- Update Path
         ip2++;
         ip = path.erase( ip1, ip2 );
         path.insert( ip, inn1, inn2 );

      }

   }

}

/**
 * @function InitGroups
 */
void Shortener::InitGroups( const std::list< Eigen::VectorXd > &path )
{
   mGroupID.resize(0);
   if( mGroupInfo.size() != 0 ) { mGroupInfo.clear(); }

   for( int i = 0; i < path.size(); i++ )
   {  mGroupID.push_back(0);  }
}

/**
 * @function CheckGroups
 */
void Shortener::CheckGroups( )
{
   //-- Start by resetting the whole thing
   if( mGroupInfo.size() != 0 ) { mGroupInfo.clear(); }

   //-- Proceed
   int group;
   std::vector<int> nodes(0);

   group = mGroupID[0];
   nodes.push_back(0);

   bool alreadyThere;

   for( int i = 1; i < mGroupID.size(); i++ )
   {
      if( mGroupID[i] != mGroupID[i-1] )
      {
         alreadyThere = false;
         //-- Check if this is part of an existent group
         for( int j = 0; j < mGroupInfo.size(); j++ )
         {
            if( group == mGroupInfo[j].group )
            {  int oldSize = mGroupInfo[j].nodes.size();
               mGroupInfo[j].nodes.resize( oldSize + nodes.size() );
               std::copy( nodes.begin(), nodes.end(), mGroupInfo[j].nodes.begin() + oldSize );
               alreadyThere = true;
               break;
            }  
         }
         
         if( alreadyThere == false )
         { mGroupInfo.push_back( GroupInfo(group, nodes) ); }
       
         nodes.clear();
         group = mGroupID[i]; nodes.push_back(i);
         
      } //-- end if
      else
      {  nodes.push_back(i); }
   }
 
   //-- For the last group
   alreadyThere = false;
   for( int j = 0; j < mGroupInfo.size(); j++ )
   {
      if( group == mGroupInfo[j].group )
      {  int oldSize = mGroupInfo[j].nodes.size();
         mGroupInfo[j].nodes.resize( oldSize + nodes.size() );
         std::copy( nodes.begin(), nodes.end(), mGroupInfo[j].nodes.begin() + oldSize );
         alreadyThere = true;
         break;
      }  
   }

   if( alreadyThere == false )
   { mGroupInfo.push_back( GroupInfo(group, nodes) ); }
}

/**
 * @function GetNewNodes
 */
std::vector<Eigen::VectorXd> Shortener::GetNewNodes( Eigen::VectorXd config1, Eigen::VectorXd config2, double stepSize )
{
   std::vector<Eigen::VectorXd> newNodes;

   int n = ceil((config2 - config1).norm() / stepSize ) + 1;
   for( int i = 0; i < n; i++ ) 
   {  Eigen::VectorXd conf = (double)(n - i)/(double)n * config1 + (double)(i)/(double)n * config2;
      newNodes.push_back( conf ); 
   }
   return newNodes;
}
