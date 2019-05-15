#include "filter.h"

using namespace std;

Filter::Filter() {
	width = height = 1;
	kernel = {1.f};
}

Filter::Filter(const std::string& filename) {

	read(filename);
}

void Filter::read(std::string filename) {

	ifstream mystream(filename);
	mystream >> width >> height;
	float params;
	while (mystream >> params)
	{
		kernel.push_back(params);
	}
	normalize();
}

void Filter::normalize() {
	int nums = kernel.size();
	int sum = 0;
	for (int i = 0; i < nums; i++)
		sum += kernel[i];

	for (int i = 0; i < nums; i++)
		kernel[i] /= sum;
}

const float& Filter::at(int x, int y) const {

	int idx = y * width + x;
	return kernel[idx];
}

