#ifndef __VCP_IMUTILS_CONSTANTS_H__
#define __VCP_IMUTILS_CONSTANTS_H__

#include <opencv2/core/version.hpp>
#if CV_VERSION_MAJOR < 3
    #include <opencv2/core/core.hpp>
#else
    #include <opencv2/core.hpp>
#endif

namespace vcp
{
namespace imutils
{
// To prepare the color name lookup tables, use the following MATLAB snippet
// in combination with the official w2c.mat file. This will generate the
// constants_colornames.cpp file.
//%% ACT tracker uses a modified (10 color weights, not normalized) color discretization schema:
//% load('w2crs.mat')
//% w2c = w2crs;
//%% Use this for the original color names/color attributes:
//load('w2c.mat');
//fid = fopen('constants_colornames.cpp', 'w');
//fprintf(fid, '#include "../constants.h"\n');
//fprintf(fid, 'namespace vcp {\n');
//fprintf(fid, 'namespace imutils {\n');
//rows = size(w2c, 1);
//cols = size(w2c, 2);
//% order of color names: black ,   blue   , brown       , grey       , green   , orange   , pink     , purple  , red     , white    , yellow
//color_values =     {  [0 0 0] , [0 0 1] , [.5 .4 .25] , [.5 .5 .5] , [0 1 0] , [1 .8 0] , [1 .5 1] , [1 0 1] , [1 0 0] , [1 1 1 ] , [ 1 1 0 ] };
//% Store the 11 base colors
//fprintf(fid, 'const cv::Vec3b kColorNameColorsRgb[] = {\n');
//for i = 1:length(color_values)
//  cdbl = color_values{i};
//  cuint8 = uint8(cdbl .* 255);
//  fprintf(fid, 'cv::Vec3b(%d, %d, %d)', cuint8(1), cuint8(2), cuint8(3));
//  if i < length(color_values), fprintf(fid, ','); end
//%   if mod(i, 3) == 0, fprintf(fid, '\n  '); end
//end
//fprintf(fid, '\n};\n');
//% Store a lookup table to the corresponding base color
//fprintf(fid, 'const uchar kColorNameColorLookup[] = {\n');
//for r = 1:rows
//  [~, idx] = max(w2c(r,:));
//  fprintf(fid, '%d', idx-1);
//  if r < rows, fprintf(fid, ','); end
//  if mod(r, 256) == 0, fprintf(fid, '\n'); end
//end
//fprintf(fid, '\n};\n');
//% Store the precomputed color probabilities
//fprintf('For header:\nextern const float kColorNameProbabilitiesRgb[][%d]\n', cols);
//fprintf(fid, 'const float kColorNameProbabilitiesRgb[][%d] = {\n', cols);
//for r = 1:rows
//  fprintf(fid, '{');
//  for c = 1:cols
//    if c > 1, fprintf(fid, ','); end
//    fprintf(fid, '%6ff', w2c(r, c));
//  end
//  fprintf(fid, '}');
//  if r < rows, fprintf(fid, ','); end
//  fprintf(fid, '\n');
//end
//fprintf(fid, '};\n');
//fprintf(fid, '} // namespace imutils\n');
//fprintf(fid, '} // namespace vcp\n');
//fclose(fid);


/** @brief The 11 base color names as RGB values. */
extern const cv::Vec3b kColorNameColorsRgb[];
/** @brief 32k lookup table to map RGB value to base color name. */
extern const uchar kColorNameColorLookup[];
/** @brief The base color name/attribute probabilities. */
extern const float kColorNameProbabilitiesRgb[][11];
} // namespace imutils
} // namespace vcp
#endif // __VCP_IMUTILS_CONSTANTS_H__
