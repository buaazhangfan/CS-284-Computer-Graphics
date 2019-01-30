#include "image.h"
#include "lodepng.h"


Image::Image() {
	width = height = 0;
}

Image::Image(const std::string& filename) {
	read(filename);
}

Image::Image(const unsigned& w, const unsigned& h)
: width(w), height(h), data(4 * w * h) {

}

void Image::read(std::string filename) {

	lodepng::decode(data, width, height, filename);
}

void Image::write(std::string filename) const {

	lodepng::encode(filename, data, width, height);
}

uint8_t* Image::at(int x, int y) {
	// FIXME

	int idx = y * width + x;
	return &data[4 * idx];

}

Image Image::operator*(const Filter& filter) {
	// FIXME
	int nums_f = filter.kernel.size();
	int nums_i = data.size();
	Image newImage(width, height);
	// std::vector<uint8_t>newimage(nums_i);
	int f_w = filter.width;
	int f_h = filter.height;

	for (int c = 0; c < 4; c++)
	{
		for (int y = 0; y < height; y++)
		{
			for (int x = 0; x < width; x++)
			{
				for (int u = - f_h / 2; u <= f_h / 2; u++)
				{
					for (int v = - f_w / 2; v <= f_w / 2; v++)
					{
						uint8_t image_pi;
						if (x + v < 0 || y + u < 0 || x + v >= width || y + u >= height)
							image_pi = 0;
						else
							image_pi = *(this -> at(x + v, y + u) + c);
						*(newImage.at(x, y) + c) += filter.at(v + f_w / 2, u + f_h / 2) * image_pi + c;
					}
				}
			}
		}
	}

	return newImage;

}