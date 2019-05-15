#include "texture.h"
#include "CGL/color.h"

#include <cmath>
#include <algorithm>

namespace CGL {

Color Texture::sample(const SampleParams &sp) {
  
  int level = 0;
  if (sp.lsm == 0)
	{
		level = 0;
		if (sp.psm == 0)
			return sample_nearest(sp.p_uv, level);
		else
			return sample_bilinear(sp.p_uv, level);
	}
	else if (sp.lsm == 1)
	{
		level = (int)round(get_level(sp));
		if (sp.psm == 0)
			return sample_nearest(sp.p_uv, level);
		else
			return sample_bilinear(sp.p_uv, level);
	}
	else
	{
		double level = get_level(sp);
		int low_level = (int)floor(level);
		int high_level = low_level + 1;
		double s = level - low_level;
		Color low_color = Color();
		Color high_color = Color();
		
		if (sp.psm == 0)
		{
			low_color = sample_nearest(sp.p_uv, low_level);
			high_color = sample_nearest(sp.p_uv, high_level);
		}
		else
		{
			low_color = sample_bilinear(sp.p_uv, low_level);
			high_color = sample_bilinear(sp.p_uv, high_level);
		}
		
		Color res = (1 - s) * low_color + s * high_color;
		return res;
		// level = //
	}

}

float Texture::get_level(const SampleParams &sp) {

	Vector2D dx_duv = sp.p_dx_uv - sp.p_uv;
	Vector2D dy_duv = sp.p_dy_uv - sp.p_uv;
	
	dx_duv.x *= width;
	dx_duv.y *= height;
	dy_duv.x *= width;
	dy_duv.y *= height;
	
	double L = max(sqrt(dx_duv.x * dx_duv.x + dx_duv.y * dx_duv.y), sqrt(dy_duv.x * dy_duv.x + dy_duv.y + dy_duv.y));
	double D = log2(L);
  // Optional helper function for Parts 5 and 6
  return D;
}

// Returns the nearest sample given a particular level and set of uv coords
Color Texture::sample_nearest(Vector2D uv, int level) {
  // Optional helper function for Parts 5 and 6
  // Feel free to ignore or create your own
		int nst_x = floor(uv.x * mipmap[level].width);
		int nst_y = floor(uv.y * mipmap[level].height);
		
		return mipmap[level].get_texel(nst_x, nst_y);

}
Color MipLevel::get_texel(int tx, int ty) {
	unsigned char	text_col[3];

	text_col[0] = texels[3 * (ty * width + tx)];
	text_col[1] = texels[3 * (ty * width + tx) + 1];
	text_col[2] = texels[3 * (ty * width + tx) + 2];
	
	return Color(text_col);
}
// Returns the bilinear sample given a particular level and set of uv coords
Color Texture::sample_bilinear(Vector2D uv, int level) {
  // Optional helper function for Parts 5 and 6
  // Feel free to ignore or create your own
	double x = uv.x * mipmap[level].width;
	double y = uv.y * mipmap[level].height;
	
	Vector2D center = Vector2D(round(x), round(y));
	Vector2D u00 = Vector2D(center.x - 0.5, center.y + 0.5);
	// Vector2D u01 = Vector2D(center.x - 0.5, center.y - 0.5);
	// Vector2D u11 = Vector2D(center.x + 0.5, center.y - 0.5);
	// Vector2D u10 = Vector2D(center.x + 0.5, center.y + 0.5);
	double s = abs(x - u00.x);
	double t = abs(y - u00.y);
	/*
	Color u00_ = mipmap[level].get_texel((int)floor(u00.x), (int)floor(u00.y));
	Color u01_ = mipmap[level].get_texel((int)floor(u01.x), (int)floor(u01.y));
	Color u11_ = mipmap[level].get_texel((int)floor(u11.x), (int)floor(u11.y));
	Color u10_ = mipmap[level].get_texel((int)floor(u10.x), (int)floor(u10.y));
	*/
	
	Color u00_ = mipmap[level].get_texel((int)(center.x - 1), (int)(center.y));
	Color u01_ = mipmap[level].get_texel((int)(center.x - 1), (int)(center.y - 1));
	Color u11_ = mipmap[level].get_texel((int)(center.x), (int)(center.y - 1));
	Color u10_ = mipmap[level].get_texel((int)(center.x), (int)(center.y));
	
	Color res = u00_ * (s * t - s - t + 1) + u10_ * (s - s * t) + u01_ * (t - s * t) + u11_ * (s * t);
		
  return res;
}



/****************************************************************************/



inline void uint8_to_float(float dst[3], unsigned char *src) {
  uint8_t *src_uint8 = (uint8_t *)src;
  dst[0] = src_uint8[0] / 255.f;
  dst[1] = src_uint8[1] / 255.f;
  dst[2] = src_uint8[2] / 255.f;
}

inline void float_to_uint8(unsigned char *dst, float src[3]) {
  uint8_t *dst_uint8 = (uint8_t *)dst;
  dst_uint8[0] = (uint8_t)(255.f * max(0.0f, min(1.0f, src[0])));
  dst_uint8[1] = (uint8_t)(255.f * max(0.0f, min(1.0f, src[1])));
  dst_uint8[2] = (uint8_t)(255.f * max(0.0f, min(1.0f, src[2])));
}

void Texture::generate_mips(int startLevel) {

  // make sure there's a valid texture
  if (startLevel >= mipmap.size()) {
    std::cerr << "Invalid start level";
  }

  // allocate sublevels
  int baseWidth = mipmap[startLevel].width;
  int baseHeight = mipmap[startLevel].height;
  int numSubLevels = (int)(log2f((float)max(baseWidth, baseHeight)));

  numSubLevels = min(numSubLevels, kMaxMipLevels - startLevel - 1);
  mipmap.resize(startLevel + numSubLevels + 1);

  int width = baseWidth;
  int height = baseHeight;
  for (int i = 1; i <= numSubLevels; i++) {

    MipLevel &level = mipmap[startLevel + i];

    // handle odd size texture by rounding down
    width = max(1, width / 2);
    //assert (width > 0);
    height = max(1, height / 2);
    //assert (height > 0);

    level.width = width;
    level.height = height;
    level.texels = vector<unsigned char>(3 * width * height);
  }

  // create mips
  int subLevels = numSubLevels - (startLevel + 1);
  for (int mipLevel = startLevel + 1; mipLevel < startLevel + subLevels + 1;
       mipLevel++) {

    MipLevel &prevLevel = mipmap[mipLevel - 1];
    MipLevel &currLevel = mipmap[mipLevel];

    int prevLevelPitch = prevLevel.width * 3; // 32 bit RGB
    int currLevelPitch = currLevel.width * 3; // 32 bit RGB

    unsigned char *prevLevelMem;
    unsigned char *currLevelMem;

    currLevelMem = (unsigned char *)&currLevel.texels[0];
    prevLevelMem = (unsigned char *)&prevLevel.texels[0];

    float wDecimal, wNorm, wWeight[3];
    int wSupport;
    float hDecimal, hNorm, hWeight[3];
    int hSupport;

    float result[3];
    float input[3];

    // conditional differentiates no rounding case from round down case
    if (prevLevel.width & 1) {
      wSupport = 3;
      wDecimal = 1.0f / (float)currLevel.width;
    } else {
      wSupport = 2;
      wDecimal = 0.0f;
    }

    // conditional differentiates no rounding case from round down case
    if (prevLevel.height & 1) {
      hSupport = 3;
      hDecimal = 1.0f / (float)currLevel.height;
    } else {
      hSupport = 2;
      hDecimal = 0.0f;
    }

    wNorm = 1.0f / (2.0f + wDecimal);
    hNorm = 1.0f / (2.0f + hDecimal);

    // case 1: reduction only in horizontal size (vertical size is 1)
    if (currLevel.height == prevLevel.height) {
      //assert (currLevel.height == 1);

      for (int i = 0; i < currLevel.width; i++) {
        wWeight[0] = wNorm * (1.0f - wDecimal * i);
        wWeight[1] = wNorm * 1.0f;
        wWeight[2] = wNorm * wDecimal * (i + 1);

        result[0] = result[1] = result[2] = 0.0f;

        for (int ii = 0; ii < wSupport; ii++) {
          uint8_to_float(input, prevLevelMem + 3 * (2 * i + ii));
          result[0] += wWeight[ii] * input[0];
          result[1] += wWeight[ii] * input[1];
          result[2] += wWeight[ii] * input[2];
        }

        // convert back to format of the texture
        float_to_uint8(currLevelMem + (3 * i), result);
      }

      // case 2: reduction only in vertical size (horizontal size is 1)
    } else if (currLevel.width == prevLevel.width) {
      //assert (currLevel.width == 1);

      for (int j = 0; j < currLevel.height; j++) {
        hWeight[0] = hNorm * (1.0f - hDecimal * j);
        hWeight[1] = hNorm;
        hWeight[2] = hNorm * hDecimal * (j + 1);

        result[0] = result[1] = result[2] = 0.0f;
        for (int jj = 0; jj < hSupport; jj++) {
          uint8_to_float(input, prevLevelMem + prevLevelPitch * (2 * j + jj));
          result[0] += hWeight[jj] * input[0];
          result[1] += hWeight[jj] * input[1];
          result[2] += hWeight[jj] * input[2];
        }

        // convert back to format of the texture
        float_to_uint8(currLevelMem + (currLevelPitch * j), result);
      }

      // case 3: reduction in both horizontal and vertical size
    } else {

      for (int j = 0; j < currLevel.height; j++) {
        hWeight[0] = hNorm * (1.0f - hDecimal * j);
        hWeight[1] = hNorm;
        hWeight[2] = hNorm * hDecimal * (j + 1);

        for (int i = 0; i < currLevel.width; i++) {
          wWeight[0] = wNorm * (1.0f - wDecimal * i);
          wWeight[1] = wNorm * 1.0f;
          wWeight[2] = wNorm * wDecimal * (i + 1);

          result[0] = result[1] = result[2] = 0.0f;

          // convolve source image with a trapezoidal filter.
          // in the case of no rounding this is just a box filter of width 2.
          // in the general case, the support region is 3x3.
          for (int jj = 0; jj < hSupport; jj++)
            for (int ii = 0; ii < wSupport; ii++) {
              float weight = hWeight[jj] * wWeight[ii];
              uint8_to_float(input, prevLevelMem +
                                        prevLevelPitch * (2 * j + jj) +
                                        3 * (2 * i + ii));
              result[0] += weight * input[0];
              result[1] += weight * input[1];
              result[2] += weight * input[2];
            }

          // convert back to format of the texture
          float_to_uint8(currLevelMem + currLevelPitch * j + 3 * i, result);
        }
      }
    }
  }
}

}
