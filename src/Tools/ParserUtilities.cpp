#include "ParserUtilities.h"

#include <iostream>
#include <sstream>

void stringToVector3d(string &str, Vector3d &vector)
{
	std::vector<string> substrings = split(str, ' ');

	if(substrings.size() != 3)
	{
		// 
	}
	else
	{
		for(unsigned int i = 0; i < 3; i++)
		{
			vector[i] = atof(substrings.at(i).c_str());
		}
	}
}

void stringToVector4d(string &str, Vector4d &vector)
{
	std::vector<string> substrings = split(str, ' ');

	if(substrings.size() != 4)
	{
		// 
	}
	else
	{
		for(unsigned int i = 0; i < 4; i++)
		{
			vector[i] = atof(substrings.at(i).c_str());
		}
	}
}

vector<string> &split(const string &s, char delim, vector<string> &elems)
{
    stringstream ss(s);
    string item;
    while(getline(ss, item, delim))
	{
        elems.push_back(item);
    }
    return elems;
}

vector<string> split(const string &s, char delim)
{
    vector<string> elems;
    return split(s, delim, elems);
}

Matrix4d buildTransformationMatrix(Vector3d rotation, Vector3d translation)
{
	Matrix4d mat = Matrix4d::Zero(4, 4);

	double yaw = rotation[0];
	double pitch = rotation[1];
	double roll = rotation[2];

	// Assuming the angles are in degrees.
    double ch = cos(yaw * 0.0174532925);
    double sh = sin(yaw * 0.0174532925);
    double ca = cos(pitch * 0.0174532925);
    double sa = sin(pitch * 0.0174532925);
    double cb = cos(roll * 0.0174532925);
    double sb = sin(roll * 0.0174532925);

    mat(0, 0) = ch * ca;
    mat(0, 1) = sh*sb - ch*sa*cb;
	mat(0, 2) = ch*sa*sb + sh*cb;

    mat(1, 0) = sa;
    mat(1, 1) = ca*cb;
    mat(1, 2) = -ca*sb;

    mat(2, 0) = -sh*ca;
	mat(2, 1) = sh*sa*cb + ch*sb;
    mat(2, 2) = -sh*sa*sb + ch*cb;

	// add translation
	mat(3, 0) = translation[0];
	mat(3, 1) = translation[1];
	mat(3, 2) = translation[2];

	// add homogenous coordinate
	mat(3, 3) = 1.0;
	
	return mat;
}

