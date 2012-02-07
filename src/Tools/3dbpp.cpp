#include <iostream>
#include <sstream>
#include "3dbpp.h"

using namespace std;

config_t::config_t() {
	reset();
}

void config_t::reset() {
	key.clear();
	pattern.clear();
	n_packs = 0;
	n_area = 0;
	n_maxh = 0;
	n_maxw = 0;
	n_maxd = 0;
	n_tvolume = 0;
	n_density = 0.0f;
	n_packvolume = 0;
	n_packweight = 0;
	origin.assign(3, 0);
	corner1.assign(3, 0);
	corner2.assign(3, 0);
	corner3.assign(3, 0);
}

void config_t::set(database* db, key_ k, pattern_ p) {
	d = db;
	key = k;
	pattern = p;

	int c = 0;
	for (uint i = 0; i < key.size(); i++) {
		for (uint j = 0; j < key[i]; j++) {
			int xn = pattern[c*3 + 0] + d->package[i].w;
			int yn = pattern[c*3 + 1] + d->package[i].h;
			int zn = pattern[c*3 + 2] + d->package[i].d;
			if (n_maxw < xn)	n_maxw = xn;
			if (n_maxh < yn)	n_maxh = yn;
			if (n_maxd < zn)	n_maxd = zn;
			n_packvolume += db->package[i].volume();
			n_packweight += db->package[i].get_weight();
			c++;
		}
	}

	n_packs = c;
	n_tvolume = n_maxw * n_maxh * n_maxd;
	if (n_tvolume != 0)
		n_density = (float) ((n_packvolume) / (n_tvolume));
	n_area = n_maxw * n_maxh;

	corner1[0] = n_maxw;
	corner2[1] = n_maxh;
	corner3[2] = n_maxd;
}

vector<int> config_t::get_origin() {
	return origin;
}

void config_t::set_origin(vector<int> o) {
	origin = o;

	for (uint i = 0; i < pattern.size(); i++) {
		pattern[i] += origin[i%3];
	}
}

bool config_t::operator == (const config_t &c) {
	if (c.key == key && n_maxw == c.n_maxw && n_maxh == c.n_maxh && n_maxd == c.n_maxd)
		return true;
	return false;
}

bool config_t::is_bound() {
	return (n_maxw <= d->bin.w && n_maxh <= d->bin.h && n_maxd <= d->bin.d);
}

bool config_t::is_layer() {
	int area = d->bin.w * d->bin.h;
	return(((float)(area - get_area())/(float)area) < 0.05);
}

ostream & operator << (ostream &o, const config_t &c) {
	vector<int> key = c.key;
	vector<int> pattern = c.pattern;
	o << "Layer " << endl;
	for (uint i = 0; i < key.size(); i++) o << key[i] << " ";
	o << endl;
	for (uint i = 0; i < pattern.size(); i++) o << pattern[i] << " ";
	o << endl;
	o << "Total packages: " << c.n_packs << endl;
	o << "Total packvolume: " << c.n_packvolume << endl;
	o << "Total volume: " << c.n_tvolume << endl;
	o << "Total area: " << c.n_area << endl;
	o << "Max height: " << c.n_maxd << endl;
	o << "Density: " << c.n_density << endl;
	o << endl;
	return o;
}

void config_t::add(const config_t c) {
	vector<int> p;
	vector<int> k(key.size(), 0);
	int c1 = 0, c2 = 0;

	for (uint i = 0; i < key.size(); i++) {
		for (uint j = 0; j < key[i]; j++) {
			p.push_back(pattern[c1*3 + 0]);
			p.push_back(pattern[c1*3 + 1]);
			p.push_back(pattern[c1*3 + 2]);
			c1++;
		}

		for (uint j = 0; j < c.key[i]; j++) {
			p.push_back(c.pattern[c2*3 + 0]);
			p.push_back(c.pattern[c2*3 + 1]);
			p.push_back(c.pattern[c2*3 + 2]);
			c2++;
		}
		k[i] = key[i] + c.key[i];
	}

	reset();
	set(this->d, k, p);
}

int config_t::get_height() {
	return n_maxh;
}

int config_t::get_area() {
	return n_area;
}

vector<int> config_t::get_corner(int i) {
	switch(i) {
	case 0: return corner1;
	case 1: return corner2;
	case 2: return corner3;
	}
}

key_ config_t::get_key() {
	return key;
}

pattern_ config_t::get_pattern() {
	return pattern;
}

dimensions_ config_t::get_dimensions() {
	dimensions_ dims;
	dims.push_back(n_maxw);
	dims.push_back(n_maxh);
	dims.push_back(n_maxd);
	return dims;
}

int config_t::get_weight() {
	return n_packweight;
}

float config_t::density() {
	return n_density;
}

string config_t::key_s() {
	stringstream s(stringstream::out);
	for (uint i = 0; i < key.size(); i++) {

		s << key[i] << " ";
	}
	return s.str();
}

string config_t::pattern_s() {
	stringstream s(stringstream::out);
	for (uint i = 0; i < pattern.size(); i++) {
		s << pattern[i] << " ";
	}
	return s.str();
}

string config_t::dimensions_s() {
	stringstream s(stringstream::out);
	s << n_maxw << " " << n_maxh << " " << n_maxd << " ";
	return s.str();
}

string config_t::cost_s() {
	stringstream s(stringstream::out);
	s << n_density;
	return s.str();
}
