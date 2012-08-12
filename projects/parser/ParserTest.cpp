/**
 * @file test0.cpp
 */
#include <robotics/Robot.h>
#include <robotics/ParserHUBO.h>
#include <stdio.h>
#include <iostream>


// path: /home/ana/Research/Hubo-Project/openHubo/jaemiHubo.stairClimbing.env.xml
// path: /home/ana/Research/Hubo-Project/openHubo/leftleg.kinbody.xml
/**
 * @function main
 */
int main( int argc, char* argv[] ) {

  boost::filesystem::path p(argv[1]);
   printf("Path loaded: \n");
   std::cout << p << std::endl;
  ParseFile( p );

  return 0;
}
