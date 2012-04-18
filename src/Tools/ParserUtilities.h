#pragma once

#include <string>
#include <vector>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

void stringToVector3d(string str, Vector3d &vector);
void stringToVector4d(string str, Vector4d &vector);

std::vector<std::string> &split(const std::string &s, char delim, std::vector<std::string> &elems);
std::vector<std::string> split(const std::string &s, char delim);

Matrix4d buildTransformationMatrix(Vector3d rotation, Vector3d translation);

