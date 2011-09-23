#include "Path.h"

using namespace std;
using namespace Eigen;

Path::Path(const list<VectorXd> &path, double maxDeviation) :
	length(0.0)
{
	if(path.size() < 2)
		return;
	list<VectorXd>::const_iterator config1 = path.begin();
	list<VectorXd>::const_iterator config2 = config1;
	config2++;
	list<VectorXd>::const_iterator config3;
	VectorXd startConfig = *config1;
	while(config2 != path.end()) {
		VectorXd endConfig = *config2;
		config3 = config2;
		config3++;
		if(maxDeviation > 0.0 && config3 != path.end()) {
			CircularPathSegment* blendSegment = new CircularPathSegment(0.5 * (*config1 + *config2), *config2, 0.5 * (*config2 + *config3), maxDeviation);
			endConfig = blendSegment->getConfiguration(0.0);
			pathSegments.push_back(new LinearPathSegment(startConfig, endConfig));
			pathSegments.push_back(blendSegment);
			startConfig = blendSegment->getConfiguration(blendSegment->getLength());
		}
		else {
			pathSegments.push_back(new LinearPathSegment(startConfig, *config2));
			startConfig = *config2;
		}
		config1 = config2;
		config2++;
	}
	for(list<PathSegment*>::const_iterator it = pathSegments.begin(); it != pathSegments.end(); it++) {
		length += (*it)->getLength();
	}
}


Path::~Path() {
	for(list<PathSegment*>::iterator it = pathSegments.begin(); it != pathSegments.end(); it++) {
		delete *it;
	}
}


double Path::getLength() const {
	return length;
}


PathSegment* Path::getPathSegment(double &s) {
	list<PathSegment*>::iterator it = pathSegments.begin();
	list<PathSegment*>::iterator next = it;
	next++;
	while(next != pathSegments.end() && s > (*it)->getLength()) {
		s -= (*it)->getLength();

		it = next;
		next++;
	}
	return *it;
}


VectorXd Path::getConfiguration(double s) {
	PathSegment* pathSegment = getPathSegment(s);
	return pathSegment->getConfiguration(s);
}

VectorXd Path::getPathVelocity(double s) {
	PathSegment* pathSegment = getPathSegment(s);
	return pathSegment->getPathVelocity(s);
}

VectorXd Path::getPathAcceleration(double s) {
	PathSegment* pathSegment = getPathSegment(s);
	return pathSegment->getPathAcceleration(s);
}