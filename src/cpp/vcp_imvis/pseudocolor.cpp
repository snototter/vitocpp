#include "pseudocolor.h"

#include <algorithm>

#include <sstream>
#include <opencv2/core/core.hpp>

#include <vcp_utils/vcp_error.h>
#include <vcp_utils/vcp_logging.h>
#include <vcp_imutils/imutils.h>
#include <vcp_imutils/matutils.h>

#undef VCP_LOGGING_COMPONENT
#define VCP_LOGGING_COMPONENT "vcp::imvis::pseudocolor"
namespace vcp
{
namespace imvis
{
namespace pseudocolor
{
const cv::Scalar kColormapAutumn[] =
{
  cv::Scalar(  0,   0, 255), cv::Scalar(  0,   1, 255), cv::Scalar(  0,   2, 255), cv::Scalar(  0,   3, 255), cv::Scalar(  0,   4, 255), cv::Scalar(  0,   5, 255), cv::Scalar(  0,   6, 255), cv::Scalar(  0,   7, 255),
  cv::Scalar(  0,   8, 255), cv::Scalar(  0,   9, 255), cv::Scalar(  0,  10, 255), cv::Scalar(  0,  11, 255), cv::Scalar(  0,  12, 255), cv::Scalar(  0,  13, 255), cv::Scalar(  0,  14, 255), cv::Scalar(  0,  15, 255),
  cv::Scalar(  0,  16, 255), cv::Scalar(  0,  17, 255), cv::Scalar(  0,  18, 255), cv::Scalar(  0,  19, 255), cv::Scalar(  0,  20, 255), cv::Scalar(  0,  21, 255), cv::Scalar(  0,  22, 255), cv::Scalar(  0,  23, 255),
  cv::Scalar(  0,  24, 255), cv::Scalar(  0,  25, 255), cv::Scalar(  0,  26, 255), cv::Scalar(  0,  27, 255), cv::Scalar(  0,  28, 255), cv::Scalar(  0,  29, 255), cv::Scalar(  0,  30, 255), cv::Scalar(  0,  31, 255),
  cv::Scalar(  0,  32, 255), cv::Scalar(  0,  33, 255), cv::Scalar(  0,  34, 255), cv::Scalar(  0,  35, 255), cv::Scalar(  0,  36, 255), cv::Scalar(  0,  37, 255), cv::Scalar(  0,  38, 255), cv::Scalar(  0,  39, 255),
  cv::Scalar(  0,  40, 255), cv::Scalar(  0,  41, 255), cv::Scalar(  0,  42, 255), cv::Scalar(  0,  43, 255), cv::Scalar(  0,  44, 255), cv::Scalar(  0,  45, 255), cv::Scalar(  0,  46, 255), cv::Scalar(  0,  47, 255),
  cv::Scalar(  0,  48, 255), cv::Scalar(  0,  49, 255), cv::Scalar(  0,  50, 255), cv::Scalar(  0,  51, 255), cv::Scalar(  0,  52, 255), cv::Scalar(  0,  53, 255), cv::Scalar(  0,  54, 255), cv::Scalar(  0,  55, 255),
  cv::Scalar(  0,  56, 255), cv::Scalar(  0,  57, 255), cv::Scalar(  0,  58, 255), cv::Scalar(  0,  59, 255), cv::Scalar(  0,  60, 255), cv::Scalar(  0,  61, 255), cv::Scalar(  0,  62, 255), cv::Scalar(  0,  63, 255),
  cv::Scalar(  0,  64, 255), cv::Scalar(  0,  65, 255), cv::Scalar(  0,  66, 255), cv::Scalar(  0,  67, 255), cv::Scalar(  0,  68, 255), cv::Scalar(  0,  69, 255), cv::Scalar(  0,  70, 255), cv::Scalar(  0,  71, 255),
  cv::Scalar(  0,  72, 255), cv::Scalar(  0,  73, 255), cv::Scalar(  0,  74, 255), cv::Scalar(  0,  75, 255), cv::Scalar(  0,  76, 255), cv::Scalar(  0,  77, 255), cv::Scalar(  0,  78, 255), cv::Scalar(  0,  79, 255),
  cv::Scalar(  0,  80, 255), cv::Scalar(  0,  81, 255), cv::Scalar(  0,  82, 255), cv::Scalar(  0,  83, 255), cv::Scalar(  0,  84, 255), cv::Scalar(  0,  85, 255), cv::Scalar(  0,  86, 255), cv::Scalar(  0,  87, 255),
  cv::Scalar(  0,  88, 255), cv::Scalar(  0,  89, 255), cv::Scalar(  0,  90, 255), cv::Scalar(  0,  91, 255), cv::Scalar(  0,  92, 255), cv::Scalar(  0,  93, 255), cv::Scalar(  0,  94, 255), cv::Scalar(  0,  95, 255),
  cv::Scalar(  0,  96, 255), cv::Scalar(  0,  97, 255), cv::Scalar(  0,  98, 255), cv::Scalar(  0,  99, 255), cv::Scalar(  0, 100, 255), cv::Scalar(  0, 101, 255), cv::Scalar(  0, 102, 255), cv::Scalar(  0, 103, 255),
  cv::Scalar(  0, 104, 255), cv::Scalar(  0, 105, 255), cv::Scalar(  0, 106, 255), cv::Scalar(  0, 107, 255), cv::Scalar(  0, 108, 255), cv::Scalar(  0, 109, 255), cv::Scalar(  0, 110, 255), cv::Scalar(  0, 111, 255),
  cv::Scalar(  0, 112, 255), cv::Scalar(  0, 113, 255), cv::Scalar(  0, 114, 255), cv::Scalar(  0, 115, 255), cv::Scalar(  0, 116, 255), cv::Scalar(  0, 117, 255), cv::Scalar(  0, 118, 255), cv::Scalar(  0, 119, 255),
  cv::Scalar(  0, 120, 255), cv::Scalar(  0, 121, 255), cv::Scalar(  0, 122, 255), cv::Scalar(  0, 123, 255), cv::Scalar(  0, 124, 255), cv::Scalar(  0, 125, 255), cv::Scalar(  0, 126, 255), cv::Scalar(  0, 127, 255),
  cv::Scalar(  0, 128, 255), cv::Scalar(  0, 129, 255), cv::Scalar(  0, 130, 255), cv::Scalar(  0, 131, 255), cv::Scalar(  0, 132, 255), cv::Scalar(  0, 133, 255), cv::Scalar(  0, 134, 255), cv::Scalar(  0, 135, 255),
  cv::Scalar(  0, 136, 255), cv::Scalar(  0, 137, 255), cv::Scalar(  0, 138, 255), cv::Scalar(  0, 139, 255), cv::Scalar(  0, 140, 255), cv::Scalar(  0, 141, 255), cv::Scalar(  0, 142, 255), cv::Scalar(  0, 143, 255),
  cv::Scalar(  0, 144, 255), cv::Scalar(  0, 145, 255), cv::Scalar(  0, 146, 255), cv::Scalar(  0, 147, 255), cv::Scalar(  0, 148, 255), cv::Scalar(  0, 149, 255), cv::Scalar(  0, 150, 255), cv::Scalar(  0, 151, 255),
  cv::Scalar(  0, 152, 255), cv::Scalar(  0, 153, 255), cv::Scalar(  0, 154, 255), cv::Scalar(  0, 155, 255), cv::Scalar(  0, 156, 255), cv::Scalar(  0, 157, 255), cv::Scalar(  0, 158, 255), cv::Scalar(  0, 159, 255),
  cv::Scalar(  0, 160, 255), cv::Scalar(  0, 161, 255), cv::Scalar(  0, 162, 255), cv::Scalar(  0, 163, 255), cv::Scalar(  0, 164, 255), cv::Scalar(  0, 165, 255), cv::Scalar(  0, 166, 255), cv::Scalar(  0, 167, 255),
  cv::Scalar(  0, 168, 255), cv::Scalar(  0, 169, 255), cv::Scalar(  0, 170, 255), cv::Scalar(  0, 171, 255), cv::Scalar(  0, 172, 255), cv::Scalar(  0, 173, 255), cv::Scalar(  0, 174, 255), cv::Scalar(  0, 175, 255),
  cv::Scalar(  0, 176, 255), cv::Scalar(  0, 177, 255), cv::Scalar(  0, 178, 255), cv::Scalar(  0, 179, 255), cv::Scalar(  0, 180, 255), cv::Scalar(  0, 181, 255), cv::Scalar(  0, 182, 255), cv::Scalar(  0, 183, 255),
  cv::Scalar(  0, 184, 255), cv::Scalar(  0, 185, 255), cv::Scalar(  0, 186, 255), cv::Scalar(  0, 187, 255), cv::Scalar(  0, 188, 255), cv::Scalar(  0, 189, 255), cv::Scalar(  0, 190, 255), cv::Scalar(  0, 191, 255),
  cv::Scalar(  0, 192, 255), cv::Scalar(  0, 193, 255), cv::Scalar(  0, 194, 255), cv::Scalar(  0, 195, 255), cv::Scalar(  0, 196, 255), cv::Scalar(  0, 197, 255), cv::Scalar(  0, 198, 255), cv::Scalar(  0, 199, 255),
  cv::Scalar(  0, 200, 255), cv::Scalar(  0, 201, 255), cv::Scalar(  0, 202, 255), cv::Scalar(  0, 203, 255), cv::Scalar(  0, 204, 255), cv::Scalar(  0, 205, 255), cv::Scalar(  0, 206, 255), cv::Scalar(  0, 207, 255),
  cv::Scalar(  0, 208, 255), cv::Scalar(  0, 209, 255), cv::Scalar(  0, 210, 255), cv::Scalar(  0, 211, 255), cv::Scalar(  0, 212, 255), cv::Scalar(  0, 213, 255), cv::Scalar(  0, 214, 255), cv::Scalar(  0, 215, 255),
  cv::Scalar(  0, 216, 255), cv::Scalar(  0, 217, 255), cv::Scalar(  0, 218, 255), cv::Scalar(  0, 219, 255), cv::Scalar(  0, 220, 255), cv::Scalar(  0, 221, 255), cv::Scalar(  0, 222, 255), cv::Scalar(  0, 223, 255),
  cv::Scalar(  0, 224, 255), cv::Scalar(  0, 225, 255), cv::Scalar(  0, 226, 255), cv::Scalar(  0, 227, 255), cv::Scalar(  0, 228, 255), cv::Scalar(  0, 229, 255), cv::Scalar(  0, 230, 255), cv::Scalar(  0, 231, 255),
  cv::Scalar(  0, 232, 255), cv::Scalar(  0, 233, 255), cv::Scalar(  0, 234, 255), cv::Scalar(  0, 235, 255), cv::Scalar(  0, 236, 255), cv::Scalar(  0, 237, 255), cv::Scalar(  0, 238, 255), cv::Scalar(  0, 239, 255),
  cv::Scalar(  0, 240, 255), cv::Scalar(  0, 241, 255), cv::Scalar(  0, 242, 255), cv::Scalar(  0, 243, 255), cv::Scalar(  0, 244, 255), cv::Scalar(  0, 245, 255), cv::Scalar(  0, 246, 255), cv::Scalar(  0, 247, 255),
  cv::Scalar(  0, 248, 255), cv::Scalar(  0, 249, 255), cv::Scalar(  0, 250, 255), cv::Scalar(  0, 251, 255), cv::Scalar(  0, 252, 255), cv::Scalar(  0, 253, 255), cv::Scalar(  0, 254, 255), cv::Scalar(  0, 255, 255)
};

const cv::Scalar kColormapBone[] =
{
  cv::Scalar(  0,   0,   0), cv::Scalar(  1,   1,   1), cv::Scalar(  2,   2,   2), cv::Scalar(  4,   3,   3), cv::Scalar(  5,   4,   4), cv::Scalar(  6,   4,   4), cv::Scalar(  7,   5,   5), cv::Scalar(  8,   6,   6),
  cv::Scalar( 10,   7,   7), cv::Scalar( 11,   8,   8), cv::Scalar( 12,   9,   9), cv::Scalar( 13,  10,  10), cv::Scalar( 15,  11,  11), cv::Scalar( 16,  11,  11), cv::Scalar( 17,  12,  12), cv::Scalar( 18,  13,  13),
  cv::Scalar( 19,  14,  14), cv::Scalar( 21,  15,  15), cv::Scalar( 22,  16,  16), cv::Scalar( 23,  17,  17), cv::Scalar( 24,  17,  17), cv::Scalar( 25,  18,  18), cv::Scalar( 27,  19,  19), cv::Scalar( 28,  20,  20),
  cv::Scalar( 29,  21,  21), cv::Scalar( 30,  22,  22), cv::Scalar( 31,  23,  23), cv::Scalar( 33,  24,  24), cv::Scalar( 34,  24,  24), cv::Scalar( 35,  25,  25), cv::Scalar( 36,  26,  26), cv::Scalar( 37,  27,  27),
  cv::Scalar( 39,  28,  28), cv::Scalar( 40,  29,  29), cv::Scalar( 41,  30,  30), cv::Scalar( 42,  31,  31), cv::Scalar( 44,  32,  32), cv::Scalar( 45,  32,  32), cv::Scalar( 46,  33,  33), cv::Scalar( 47,  34,  34),
  cv::Scalar( 48,  35,  35), cv::Scalar( 50,  36,  36), cv::Scalar( 51,  37,  37), cv::Scalar( 52,  38,  38), cv::Scalar( 53,  39,  39), cv::Scalar( 54,  39,  39), cv::Scalar( 56,  40,  40), cv::Scalar( 57,  41,  41),
  cv::Scalar( 58,  42,  42), cv::Scalar( 59,  43,  43), cv::Scalar( 60,  44,  44), cv::Scalar( 62,  45,  45), cv::Scalar( 63,  45,  45), cv::Scalar( 64,  46,  46), cv::Scalar( 65,  47,  47), cv::Scalar( 66,  48,  48),
  cv::Scalar( 68,  49,  49), cv::Scalar( 69,  50,  50), cv::Scalar( 70,  51,  51), cv::Scalar( 71,  52,  52), cv::Scalar( 73,  53,  53), cv::Scalar( 74,  53,  53), cv::Scalar( 75,  54,  54), cv::Scalar( 76,  55,  55),
  cv::Scalar( 77,  56,  56), cv::Scalar( 79,  57,  57), cv::Scalar( 80,  58,  58), cv::Scalar( 81,  59,  59), cv::Scalar( 82,  59,  59), cv::Scalar( 83,  60,  60), cv::Scalar( 85,  61,  61), cv::Scalar( 86,  62,  62),
  cv::Scalar( 87,  63,  63), cv::Scalar( 88,  64,  64), cv::Scalar( 89,  65,  65), cv::Scalar( 91,  66,  66), cv::Scalar( 92,  66,  66), cv::Scalar( 93,  67,  67), cv::Scalar( 94,  68,  68), cv::Scalar( 95,  69,  69),
  cv::Scalar( 97,  70,  70), cv::Scalar( 98,  71,  71), cv::Scalar( 99,  72,  72), cv::Scalar(100,  73,  73), cv::Scalar(102,  74,  74), cv::Scalar(103,  74,  74), cv::Scalar(104,  75,  75), cv::Scalar(105,  76,  76),
  cv::Scalar(106,  77,  77), cv::Scalar(108,  78,  78), cv::Scalar(109,  79,  79), cv::Scalar(110,  80,  80), cv::Scalar(111,  81,  81), cv::Scalar(112,  81,  81), cv::Scalar(114,  82,  82), cv::Scalar(115,  83,  83),
  cv::Scalar(116,  84,  84), cv::Scalar(117,  85,  85), cv::Scalar(118,  87,  86), cv::Scalar(119,  88,  87), cv::Scalar(119,  89,  88), cv::Scalar(120,  90,  88), cv::Scalar(121,  91,  89), cv::Scalar(122,  93,  90),
  cv::Scalar(123,  94,  91), cv::Scalar(124,  95,  92), cv::Scalar(125,  96,  93), cv::Scalar(126,  97,  94), cv::Scalar(126,  99,  94), cv::Scalar(127, 100,  95), cv::Scalar(128, 101,  96), cv::Scalar(129, 102,  97),
  cv::Scalar(130, 103,  98), cv::Scalar(131, 105,  99), cv::Scalar(132, 106, 100), cv::Scalar(133, 107, 101), cv::Scalar(133, 108, 102), cv::Scalar(134, 110, 102), cv::Scalar(135, 111, 103), cv::Scalar(136, 112, 104),
  cv::Scalar(137, 113, 105), cv::Scalar(138, 114, 106), cv::Scalar(139, 116, 107), cv::Scalar(140, 117, 108), cv::Scalar(140, 118, 109), cv::Scalar(141, 119, 109), cv::Scalar(142, 120, 110), cv::Scalar(143, 122, 111),
  cv::Scalar(144, 123, 112), cv::Scalar(145, 124, 113), cv::Scalar(146, 125, 114), cv::Scalar(147, 126, 115), cv::Scalar(147, 128, 115), cv::Scalar(148, 129, 116), cv::Scalar(149, 130, 117), cv::Scalar(150, 131, 118),
  cv::Scalar(151, 132, 119), cv::Scalar(152, 134, 120), cv::Scalar(153, 135, 121), cv::Scalar(153, 136, 122), cv::Scalar(154, 137, 123), cv::Scalar(155, 139, 123), cv::Scalar(156, 140, 124), cv::Scalar(157, 141, 125),
  cv::Scalar(158, 142, 126), cv::Scalar(159, 143, 127), cv::Scalar(160, 145, 128), cv::Scalar(161, 146, 129), cv::Scalar(161, 147, 130), cv::Scalar(162, 148, 130), cv::Scalar(163, 149, 131), cv::Scalar(164, 151, 132),
  cv::Scalar(165, 152, 133), cv::Scalar(166, 153, 134), cv::Scalar(167, 154, 135), cv::Scalar(167, 155, 136), cv::Scalar(168, 157, 137), cv::Scalar(169, 158, 137), cv::Scalar(170, 159, 138), cv::Scalar(171, 160, 139),
  cv::Scalar(172, 161, 140), cv::Scalar(173, 163, 141), cv::Scalar(174, 164, 142), cv::Scalar(174, 165, 143), cv::Scalar(175, 166, 144), cv::Scalar(176, 168, 144), cv::Scalar(177, 169, 145), cv::Scalar(178, 170, 146),
  cv::Scalar(179, 171, 147), cv::Scalar(180, 172, 148), cv::Scalar(181, 174, 149), cv::Scalar(182, 175, 150), cv::Scalar(182, 176, 151), cv::Scalar(183, 177, 151), cv::Scalar(184, 178, 152), cv::Scalar(185, 180, 153),
  cv::Scalar(186, 181, 154), cv::Scalar(187, 182, 155), cv::Scalar(188, 183, 156), cv::Scalar(188, 184, 157), cv::Scalar(189, 186, 157), cv::Scalar(190, 187, 158), cv::Scalar(191, 188, 159), cv::Scalar(192, 189, 160),
  cv::Scalar(193, 190, 161), cv::Scalar(194, 192, 162), cv::Scalar(195, 193, 163), cv::Scalar(195, 194, 164), cv::Scalar(196, 195, 165), cv::Scalar(197, 197, 165), cv::Scalar(198, 198, 166), cv::Scalar(199, 199, 167),
  cv::Scalar(200, 200, 168), cv::Scalar(201, 201, 170), cv::Scalar(202, 202, 171), cv::Scalar(203, 203, 172), cv::Scalar(203, 203, 174), cv::Scalar(204, 204, 175), cv::Scalar(205, 205, 177), cv::Scalar(206, 206, 178),
  cv::Scalar(207, 207, 179), cv::Scalar(208, 208, 181), cv::Scalar(209, 209, 182), cv::Scalar(210, 210, 183), cv::Scalar(210, 210, 185), cv::Scalar(211, 211, 186), cv::Scalar(212, 212, 188), cv::Scalar(213, 213, 189),
  cv::Scalar(214, 214, 190), cv::Scalar(215, 215, 192), cv::Scalar(216, 216, 193), cv::Scalar(217, 217, 195), cv::Scalar(217, 217, 196), cv::Scalar(218, 218, 197), cv::Scalar(219, 219, 199), cv::Scalar(220, 220, 200),
  cv::Scalar(221, 221, 201), cv::Scalar(222, 222, 203), cv::Scalar(223, 223, 204), cv::Scalar(224, 224, 205), cv::Scalar(224, 224, 207), cv::Scalar(225, 225, 208), cv::Scalar(226, 226, 210), cv::Scalar(227, 227, 211),
  cv::Scalar(228, 228, 212), cv::Scalar(229, 229, 214), cv::Scalar(230, 230, 215), cv::Scalar(230, 230, 216), cv::Scalar(231, 231, 218), cv::Scalar(232, 232, 219), cv::Scalar(233, 233, 221), cv::Scalar(234, 234, 222),
  cv::Scalar(235, 235, 223), cv::Scalar(236, 236, 225), cv::Scalar(237, 237, 226), cv::Scalar(238, 238, 227), cv::Scalar(238, 238, 229), cv::Scalar(239, 239, 230), cv::Scalar(240, 240, 232), cv::Scalar(241, 241, 233),
  cv::Scalar(242, 242, 234), cv::Scalar(243, 243, 236), cv::Scalar(244, 244, 237), cv::Scalar(244, 244, 238), cv::Scalar(245, 245, 240), cv::Scalar(246, 246, 241), cv::Scalar(247, 247, 243), cv::Scalar(248, 248, 244),
  cv::Scalar(249, 249, 245), cv::Scalar(250, 250, 247), cv::Scalar(251, 251, 248), cv::Scalar(251, 251, 250), cv::Scalar(252, 252, 251), cv::Scalar(253, 253, 252), cv::Scalar(254, 254, 254), cv::Scalar(255, 255, 255)
};

const cv::Scalar kColormapCold[] =
{
  cv::Scalar(  0,   0,   0), cv::Scalar(  3,   0,   0), cv::Scalar(  6,   0,   0), cv::Scalar(  9,   0,   0), cv::Scalar( 12,   0,   0), cv::Scalar( 15,   0,   0), cv::Scalar( 18,   0,   0), cv::Scalar( 21,   0,   0),
  cv::Scalar( 24,   0,   0), cv::Scalar( 27,   0,   0), cv::Scalar( 30,   0,   0), cv::Scalar( 33,   0,   0), cv::Scalar( 36,   0,   0), cv::Scalar( 39,   0,   0), cv::Scalar( 42,   0,   0), cv::Scalar( 45,   0,   0),
  cv::Scalar( 48,   0,   0), cv::Scalar( 51,   0,   0), cv::Scalar( 54,   0,   0), cv::Scalar( 57,   0,   0), cv::Scalar( 60,   0,   0), cv::Scalar( 63,   0,   0), cv::Scalar( 66,   0,   0), cv::Scalar( 69,   0,   0),
  cv::Scalar( 72,   0,   0), cv::Scalar( 75,   0,   0), cv::Scalar( 78,   0,   0), cv::Scalar( 81,   0,   0), cv::Scalar( 84,   0,   0), cv::Scalar( 87,   0,   0), cv::Scalar( 90,   0,   0), cv::Scalar( 93,   0,   0),
  cv::Scalar( 96,   0,   0), cv::Scalar( 99,   0,   0), cv::Scalar(102,   0,   0), cv::Scalar(105,   0,   0), cv::Scalar(108,   0,   0), cv::Scalar(111,   0,   0), cv::Scalar(114,   0,   0), cv::Scalar(117,   0,   0),
  cv::Scalar(120,   0,   0), cv::Scalar(123,   0,   0), cv::Scalar(126,   0,   0), cv::Scalar(129,   0,   0), cv::Scalar(132,   0,   0), cv::Scalar(135,   0,   0), cv::Scalar(138,   0,   0), cv::Scalar(141,   0,   0),
  cv::Scalar(144,   0,   0), cv::Scalar(147,   0,   0), cv::Scalar(150,   0,   0), cv::Scalar(153,   0,   0), cv::Scalar(156,   0,   0), cv::Scalar(159,   0,   0), cv::Scalar(162,   0,   0), cv::Scalar(165,   0,   0),
  cv::Scalar(168,   0,   0), cv::Scalar(171,   0,   0), cv::Scalar(174,   0,   0), cv::Scalar(177,   0,   0), cv::Scalar(180,   0,   0), cv::Scalar(183,   0,   0), cv::Scalar(186,   0,   0), cv::Scalar(189,   0,   0),
  cv::Scalar(192,   0,   0), cv::Scalar(195,   0,   0), cv::Scalar(198,   0,   0), cv::Scalar(201,   0,   0), cv::Scalar(204,   0,   0), cv::Scalar(207,   0,   0), cv::Scalar(210,   0,   0), cv::Scalar(213,   0,   0),
  cv::Scalar(216,   0,   0), cv::Scalar(219,   0,   0), cv::Scalar(222,   0,   0), cv::Scalar(225,   0,   0), cv::Scalar(228,   0,   0), cv::Scalar(231,   0,   0), cv::Scalar(234,   0,   0), cv::Scalar(237,   0,   0),
  cv::Scalar(240,   0,   0), cv::Scalar(243,   0,   0), cv::Scalar(246,   0,   0), cv::Scalar(249,   0,   0), cv::Scalar(252,   0,   0), cv::Scalar(255,   0,   0), cv::Scalar(255,   3,   0), cv::Scalar(255,   6,   0),
  cv::Scalar(255,   9,   0), cv::Scalar(255,  12,   0), cv::Scalar(255,  15,   0), cv::Scalar(255,  18,   0), cv::Scalar(255,  21,   0), cv::Scalar(255,  24,   0), cv::Scalar(255,  27,   0), cv::Scalar(255,  30,   0),
  cv::Scalar(255,  33,   0), cv::Scalar(255,  36,   0), cv::Scalar(255,  39,   0), cv::Scalar(255,  42,   0), cv::Scalar(255,  45,   0), cv::Scalar(255,  48,   0), cv::Scalar(255,  51,   0), cv::Scalar(255,  54,   0),
  cv::Scalar(255,  57,   0), cv::Scalar(255,  60,   0), cv::Scalar(255,  63,   0), cv::Scalar(255,  66,   0), cv::Scalar(255,  69,   0), cv::Scalar(255,  72,   0), cv::Scalar(255,  75,   0), cv::Scalar(255,  78,   0),
  cv::Scalar(255,  81,   0), cv::Scalar(255,  84,   0), cv::Scalar(255,  87,   0), cv::Scalar(255,  90,   0), cv::Scalar(255,  93,   0), cv::Scalar(255,  96,   0), cv::Scalar(255,  99,   0), cv::Scalar(255, 102,   0),
  cv::Scalar(255, 105,   0), cv::Scalar(255, 108,   0), cv::Scalar(255, 111,   0), cv::Scalar(255, 114,   0), cv::Scalar(255, 117,   0), cv::Scalar(255, 120,   0), cv::Scalar(255, 123,   0), cv::Scalar(255, 126,   0),
  cv::Scalar(255, 129,   0), cv::Scalar(255, 132,   0), cv::Scalar(255, 135,   0), cv::Scalar(255, 138,   0), cv::Scalar(255, 141,   0), cv::Scalar(255, 144,   0), cv::Scalar(255, 147,   0), cv::Scalar(255, 150,   0),
  cv::Scalar(255, 153,   0), cv::Scalar(255, 156,   0), cv::Scalar(255, 159,   0), cv::Scalar(255, 162,   0), cv::Scalar(255, 165,   0), cv::Scalar(255, 168,   0), cv::Scalar(255, 171,   0), cv::Scalar(255, 174,   0),
  cv::Scalar(255, 177,   0), cv::Scalar(255, 180,   0), cv::Scalar(255, 183,   0), cv::Scalar(255, 186,   0), cv::Scalar(255, 189,   0), cv::Scalar(255, 192,   0), cv::Scalar(255, 195,   0), cv::Scalar(255, 198,   0),
  cv::Scalar(255, 201,   0), cv::Scalar(255, 204,   0), cv::Scalar(255, 207,   0), cv::Scalar(255, 210,   0), cv::Scalar(255, 213,   0), cv::Scalar(255, 216,   0), cv::Scalar(255, 219,   0), cv::Scalar(255, 222,   0),
  cv::Scalar(255, 225,   0), cv::Scalar(255, 228,   0), cv::Scalar(255, 231,   0), cv::Scalar(255, 234,   0), cv::Scalar(255, 237,   0), cv::Scalar(255, 240,   0), cv::Scalar(255, 243,   0), cv::Scalar(255, 246,   0),
  cv::Scalar(255, 249,   0), cv::Scalar(255, 252,   0), cv::Scalar(255, 255,   0), cv::Scalar(255, 255,   3), cv::Scalar(255, 255,   6), cv::Scalar(255, 255,   9), cv::Scalar(255, 255,  12), cv::Scalar(255, 255,  15),
  cv::Scalar(255, 255,  18), cv::Scalar(255, 255,  21), cv::Scalar(255, 255,  24), cv::Scalar(255, 255,  27), cv::Scalar(255, 255,  30), cv::Scalar(255, 255,  33), cv::Scalar(255, 255,  36), cv::Scalar(255, 255,  39),
  cv::Scalar(255, 255,  42), cv::Scalar(255, 255,  45), cv::Scalar(255, 255,  48), cv::Scalar(255, 255,  51), cv::Scalar(255, 255,  54), cv::Scalar(255, 255,  57), cv::Scalar(255, 255,  60), cv::Scalar(255, 255,  63),
  cv::Scalar(255, 255,  66), cv::Scalar(255, 255,  69), cv::Scalar(255, 255,  72), cv::Scalar(255, 255,  75), cv::Scalar(255, 255,  78), cv::Scalar(255, 255,  81), cv::Scalar(255, 255,  84), cv::Scalar(255, 255,  87),
  cv::Scalar(255, 255,  90), cv::Scalar(255, 255,  93), cv::Scalar(255, 255,  96), cv::Scalar(255, 255,  99), cv::Scalar(255, 255, 102), cv::Scalar(255, 255, 105), cv::Scalar(255, 255, 108), cv::Scalar(255, 255, 111),
  cv::Scalar(255, 255, 114), cv::Scalar(255, 255, 117), cv::Scalar(255, 255, 120), cv::Scalar(255, 255, 123), cv::Scalar(255, 255, 126), cv::Scalar(255, 255, 129), cv::Scalar(255, 255, 132), cv::Scalar(255, 255, 135),
  cv::Scalar(255, 255, 138), cv::Scalar(255, 255, 141), cv::Scalar(255, 255, 144), cv::Scalar(255, 255, 147), cv::Scalar(255, 255, 150), cv::Scalar(255, 255, 153), cv::Scalar(255, 255, 156), cv::Scalar(255, 255, 159),
  cv::Scalar(255, 255, 162), cv::Scalar(255, 255, 165), cv::Scalar(255, 255, 168), cv::Scalar(255, 255, 171), cv::Scalar(255, 255, 174), cv::Scalar(255, 255, 177), cv::Scalar(255, 255, 180), cv::Scalar(255, 255, 183),
  cv::Scalar(255, 255, 186), cv::Scalar(255, 255, 189), cv::Scalar(255, 255, 192), cv::Scalar(255, 255, 195), cv::Scalar(255, 255, 198), cv::Scalar(255, 255, 201), cv::Scalar(255, 255, 204), cv::Scalar(255, 255, 207),
  cv::Scalar(255, 255, 210), cv::Scalar(255, 255, 213), cv::Scalar(255, 255, 216), cv::Scalar(255, 255, 219), cv::Scalar(255, 255, 222), cv::Scalar(255, 255, 225), cv::Scalar(255, 255, 228), cv::Scalar(255, 255, 231),
  cv::Scalar(255, 255, 234), cv::Scalar(255, 255, 237), cv::Scalar(255, 255, 240), cv::Scalar(255, 255, 243), cv::Scalar(255, 255, 246), cv::Scalar(255, 255, 249), cv::Scalar(255, 255, 252), cv::Scalar(255, 255, 255)
};

const cv::Scalar kColormapDisparity[] =
{
  cv::Scalar(  0,   0,   0), cv::Scalar(  9,   0,   0), cv::Scalar( 18,   0,   0), cv::Scalar( 26,   0,   0), cv::Scalar( 35,   0,   0), cv::Scalar( 44,   0,   0), cv::Scalar( 53,   0,   0), cv::Scalar( 61,   0,   0),
  cv::Scalar( 70,   0,   0), cv::Scalar( 79,   0,   0), cv::Scalar( 88,   0,   0), cv::Scalar( 96,   0,   0), cv::Scalar(105,   0,   0), cv::Scalar(114,   0,   0), cv::Scalar(123,   0,   0), cv::Scalar(132,   0,   0),
  cv::Scalar( 70,   0,  27), cv::Scalar( 80,   0,  26), cv::Scalar( 91,   0,  25), cv::Scalar(102,   0,  25), cv::Scalar(113,   0,  24), cv::Scalar(123,   0,  23), cv::Scalar(134,   0,  22), cv::Scalar(145,   0,  22),
  cv::Scalar(155,   0,  21), cv::Scalar(166,   0,  20), cv::Scalar(177,   0,  20), cv::Scalar(188,   0,  19), cv::Scalar(198,   0,  18), cv::Scalar(209,   0,  17), cv::Scalar(220,   0,  17), cv::Scalar(230,   0,  16),
  cv::Scalar(167,   1,  42), cv::Scalar(163,   2,  45), cv::Scalar(158,   3,  48), cv::Scalar(153,   3,  52), cv::Scalar(149,   4,  55), cv::Scalar(144,   5,  58), cv::Scalar(140,   6,  62), cv::Scalar(135,   7,  65),
  cv::Scalar(131,   8,  68), cv::Scalar(126,   9,  71), cv::Scalar(122,  10,  75), cv::Scalar(117,  11,  78), cv::Scalar(113,  12,  81), cv::Scalar(108,  13,  84), cv::Scalar(104,  14,  88), cv::Scalar( 99,  15,  91),
  cv::Scalar(153,   0, 102), cv::Scalar(147,   0, 108), cv::Scalar(142,   0, 113), cv::Scalar(136,   0, 119), cv::Scalar(131,   0, 124), cv::Scalar(126,   0, 129), cv::Scalar(120,   0, 135), cv::Scalar(115,   0, 140),
  cv::Scalar(109,   0, 146), cv::Scalar(104,   0, 151), cv::Scalar( 99,   0, 156), cv::Scalar( 93,   0, 162), cv::Scalar( 88,   0, 167), cv::Scalar( 82,   0, 173), cv::Scalar( 77,   0, 178), cv::Scalar( 72,   0, 183),
  cv::Scalar( 83,  12, 159), cv::Scalar( 80,  11, 166), cv::Scalar( 76,  10, 172), cv::Scalar( 73,   9, 178), cv::Scalar( 69,   9, 184), cv::Scalar( 66,   8, 190), cv::Scalar( 63,   7, 196), cv::Scalar( 59,   6, 202),
  cv::Scalar( 56,   6, 208), cv::Scalar( 53,   5, 214), cv::Scalar( 49,   4, 220), cv::Scalar( 46,   3, 226), cv::Scalar( 43,   3, 233), cv::Scalar( 39,   2, 239), cv::Scalar( 36,   1, 245), cv::Scalar( 33,   1, 251),
  cv::Scalar( 79,   7, 224), cv::Scalar( 88,   7, 224), cv::Scalar( 98,   7, 224), cv::Scalar(107,   7, 224), cv::Scalar(116,   6, 224), cv::Scalar(125,   6, 224), cv::Scalar(135,   6, 224), cv::Scalar(144,   6, 224),
  cv::Scalar(153,   6, 224), cv::Scalar(162,   6, 224), cv::Scalar(171,   6, 224), cv::Scalar(181,   6, 224), cv::Scalar(190,   6, 224), cv::Scalar(199,   6, 224), cv::Scalar(208,   5, 224), cv::Scalar(217,   5, 224),
  cv::Scalar(171,   1, 254), cv::Scalar(174,   3, 252), cv::Scalar(177,   5, 250), cv::Scalar(180,   8, 247), cv::Scalar(183,  10, 245), cv::Scalar(187,  12, 243), cv::Scalar(190,  14, 241), cv::Scalar(193,  17, 238),
  cv::Scalar(196,  19, 236), cv::Scalar(199,  21, 234), cv::Scalar(202,  23, 232), cv::Scalar(205,  26, 229), cv::Scalar(208,  28, 227), cv::Scalar(211,  30, 225), cv::Scalar(214,  32, 223), cv::Scalar(217,  35, 220),
  cv::Scalar(178,  54, 201), cv::Scalar(175,  59, 196), cv::Scalar(171,  63, 192), cv::Scalar(168,  68, 187), cv::Scalar(165,  73, 182), cv::Scalar(162,  78, 177), cv::Scalar(158,  82, 173), cv::Scalar(155,  87, 168),
  cv::Scalar(152,  92, 163), cv::Scalar(149,  97, 158), cv::Scalar(145, 101, 154), cv::Scalar(142, 106, 149), cv::Scalar(139, 111, 144), cv::Scalar(136, 116, 139), cv::Scalar(132, 120, 135), cv::Scalar(129, 125, 130),
  cv::Scalar( 50, 133, 147), cv::Scalar( 49, 139, 140), cv::Scalar( 48, 145, 133), cv::Scalar( 47, 150, 126), cv::Scalar( 46, 156, 118), cv::Scalar( 45, 161, 111), cv::Scalar( 44, 167, 104), cv::Scalar( 43, 172,  97),
  cv::Scalar( 43, 178,  90), cv::Scalar( 42, 183,  82), cv::Scalar( 41, 189,  75), cv::Scalar( 40, 194,  68), cv::Scalar( 39, 200,  61), cv::Scalar( 38, 206,  53), cv::Scalar( 37, 211,  46), cv::Scalar( 36, 217,  39),
  cv::Scalar( 48, 206,  58), cv::Scalar( 51, 209,  55), cv::Scalar( 53, 213,  51), cv::Scalar( 56, 216,  47), cv::Scalar( 58, 219,  43), cv::Scalar( 60, 222,  39), cv::Scalar( 63, 225,  36), cv::Scalar( 65, 228,  32),
  cv::Scalar( 67, 232,  28), cv::Scalar( 70, 235,  24), cv::Scalar( 72, 238,  20), cv::Scalar( 75, 241,  17), cv::Scalar( 77, 244,  13), cv::Scalar( 79, 247,   9), cv::Scalar( 82, 251,   5), cv::Scalar( 84, 254,   1),
  cv::Scalar( 98, 241,  24), cv::Scalar(106, 242,  23), cv::Scalar(114, 243,  21), cv::Scalar(123, 244,  20), cv::Scalar(131, 245,  18), cv::Scalar(139, 246,  16), cv::Scalar(147, 247,  15), cv::Scalar(156, 247,  13),
  cv::Scalar(164, 248,  12), cv::Scalar(172, 249,  10), cv::Scalar(181, 250,   8), cv::Scalar(189, 251,   7), cv::Scalar(197, 252,   5), cv::Scalar(206, 253,   4), cv::Scalar(214, 254,   2), cv::Scalar(222, 255,   0),
  cv::Scalar(186, 247,  33), cv::Scalar(186, 248,  35), cv::Scalar(186, 248,  37), cv::Scalar(186, 249,  40), cv::Scalar(186, 249,  42), cv::Scalar(187, 250,  44), cv::Scalar(187, 250,  46), cv::Scalar(187, 251,  49),
  cv::Scalar(187, 251,  51), cv::Scalar(187, 252,  53), cv::Scalar(187, 252,  56), cv::Scalar(187, 253,  58), cv::Scalar(187, 253,  60), cv::Scalar(187, 254,  62), cv::Scalar(187, 254,  65), cv::Scalar(187, 255,  67),
  cv::Scalar(183, 242,  97), cv::Scalar(177, 243, 101), cv::Scalar(172, 244, 105), cv::Scalar(167, 245, 109), cv::Scalar(161, 245, 112), cv::Scalar(156, 246, 116), cv::Scalar(151, 247, 120), cv::Scalar(145, 248, 123),
  cv::Scalar(140, 249, 127), cv::Scalar(134, 250, 131), cv::Scalar(129, 251, 134), cv::Scalar(124, 251, 138), cv::Scalar(118, 252, 142), cv::Scalar(113, 253, 146), cv::Scalar(108, 254, 149), cv::Scalar(102, 255, 153),
  cv::Scalar(155, 241, 164), cv::Scalar(146, 242, 169), cv::Scalar(137, 243, 174), cv::Scalar(127, 244, 179), cv::Scalar(118, 245, 184), cv::Scalar(109, 245, 189), cv::Scalar(100, 246, 194), cv::Scalar( 90, 247, 199),
  cv::Scalar( 81, 248, 204), cv::Scalar( 72, 249, 209), cv::Scalar( 63, 250, 214), cv::Scalar( 53, 251, 219), cv::Scalar( 44, 252, 224), cv::Scalar( 35, 253, 229), cv::Scalar( 26, 254, 234), cv::Scalar( 16, 255, 239),
  cv::Scalar( 95, 253, 216), cv::Scalar(101, 253, 216), cv::Scalar(107, 254, 217), cv::Scalar(113, 254, 218), cv::Scalar(119, 254, 219), cv::Scalar(125, 254, 220), cv::Scalar(131, 254, 221), cv::Scalar(137, 254, 221),
  cv::Scalar(143, 254, 222), cv::Scalar(149, 254, 223), cv::Scalar(155, 254, 224), cv::Scalar(162, 255, 225), cv::Scalar(168, 255, 225), cv::Scalar(174, 255, 226), cv::Scalar(180, 255, 227), cv::Scalar(186, 255, 228),
  cv::Scalar(123, 255, 255), cv::Scalar(132, 255, 255), cv::Scalar(141, 255, 255), cv::Scalar(150, 255, 255), cv::Scalar(159, 255, 255), cv::Scalar(167, 255, 255), cv::Scalar(176, 255, 255), cv::Scalar(185, 255, 255),
  cv::Scalar(194, 255, 255), cv::Scalar(202, 255, 255), cv::Scalar(211, 255, 255), cv::Scalar(220, 255, 255), cv::Scalar(229, 255, 255), cv::Scalar(237, 255, 255), cv::Scalar(246, 255, 255), cv::Scalar(255, 255, 255)
};

const cv::Scalar kColormapEarth[] =
{
  cv::Scalar(  0,   0,   0), cv::Scalar(  1,   2,   0), cv::Scalar(  2,   3,   0), cv::Scalar(  2,   5,   0), cv::Scalar(  3,   6,   0), cv::Scalar(  4,   8,   0), cv::Scalar(  5,   9,   0), cv::Scalar(  6,  11,   0),
  cv::Scalar(  7,  12,   0), cv::Scalar(  7,  14,   0), cv::Scalar(  8,  15,   0), cv::Scalar(  9,  17,   0), cv::Scalar( 10,  19,   0), cv::Scalar( 11,  20,   0), cv::Scalar( 11,  22,   0), cv::Scalar( 12,  23,   0),
  cv::Scalar( 13,  25,   0), cv::Scalar( 14,  26,   0), cv::Scalar( 15,  28,   0), cv::Scalar( 14,  29,   2), cv::Scalar( 14,  29,   4), cv::Scalar( 14,  30,   6), cv::Scalar( 13,  30,   9), cv::Scalar( 13,  31,  11),
  cv::Scalar( 12,  32,  13), cv::Scalar( 12,  32,  16), cv::Scalar( 11,  33,  18), cv::Scalar( 11,  34,  20), cv::Scalar( 10,  34,  23), cv::Scalar( 10,  35,  25), cv::Scalar(  9,  35,  27), cv::Scalar(  9,  36,  30),
  cv::Scalar(  8,  37,  32), cv::Scalar(  8,  37,  34), cv::Scalar(  7,  38,  37), cv::Scalar(  7,  38,  39), cv::Scalar(  7,  39,  41), cv::Scalar(  7,  40,  42), cv::Scalar(  9,  42,  41), cv::Scalar( 10,  44,  40),
  cv::Scalar( 12,  46,  39), cv::Scalar( 13,  48,  39), cv::Scalar( 14,  49,  38), cv::Scalar( 16,  51,  37), cv::Scalar( 17,  53,  36), cv::Scalar( 19,  55,  35), cv::Scalar( 20,  57,  35), cv::Scalar( 22,  59,  34),
  cv::Scalar( 23,  60,  33), cv::Scalar( 25,  62,  32), cv::Scalar( 26,  64,  31), cv::Scalar( 27,  66,  31), cv::Scalar( 29,  68,  30), cv::Scalar( 30,  70,  29), cv::Scalar( 32,  71,  28), cv::Scalar( 33,  73,  28),
  cv::Scalar( 32,  74,  31), cv::Scalar( 32,  74,  33), cv::Scalar( 31,  75,  35), cv::Scalar( 31,  76,  37), cv::Scalar( 30,  76,  39), cv::Scalar( 30,  77,  42), cv::Scalar( 29,  78,  44), cv::Scalar( 29,  78,  46),
  cv::Scalar( 28,  79,  48), cv::Scalar( 28,  80,  50), cv::Scalar( 27,  80,  52), cv::Scalar( 27,  81,  55), cv::Scalar( 26,  82,  57), cv::Scalar( 26,  82,  59), cv::Scalar( 25,  83,  61), cv::Scalar( 25,  84,  63),
  cv::Scalar( 24,  85,  66), cv::Scalar( 24,  85,  67), cv::Scalar( 25,  87,  67), cv::Scalar( 27,  88,  67), cv::Scalar( 28,  90,  67), cv::Scalar( 29,  91,  67), cv::Scalar( 30,  93,  67), cv::Scalar( 31,  94,  67),
  cv::Scalar( 33,  96,  67), cv::Scalar( 34,  97,  68), cv::Scalar( 35,  98,  68), cv::Scalar( 36, 100,  68), cv::Scalar( 38, 101,  68), cv::Scalar( 39, 103,  68), cv::Scalar( 40, 104,  68), cv::Scalar( 41, 106,  68),
  cv::Scalar( 42, 107,  68), cv::Scalar( 44, 109,  68), cv::Scalar( 45, 110,  68), cv::Scalar( 46, 112,  68), cv::Scalar( 48, 113,  68), cv::Scalar( 50, 114,  69), cv::Scalar( 52, 114,  70), cv::Scalar( 54, 115,  71),
  cv::Scalar( 56, 116,  71), cv::Scalar( 58, 117,  72), cv::Scalar( 60, 118,  73), cv::Scalar( 62, 119,  73), cv::Scalar( 64, 120,  74), cv::Scalar( 66, 121,  75), cv::Scalar( 68, 122,  76), cv::Scalar( 70, 123,  76),
  cv::Scalar( 72, 124,  77), cv::Scalar( 74, 125,  78), cv::Scalar( 76, 126,  79), cv::Scalar( 78, 127,  79), cv::Scalar( 80, 128,  80), cv::Scalar( 82, 129,  81), cv::Scalar( 83, 129,  83), cv::Scalar( 83, 130,  85),
  cv::Scalar( 83, 130,  87), cv::Scalar( 83, 131,  90), cv::Scalar( 84, 131,  92), cv::Scalar( 84, 131,  94), cv::Scalar( 84, 132,  97), cv::Scalar( 84, 132,  99), cv::Scalar( 85, 133, 102), cv::Scalar( 85, 133, 104),
  cv::Scalar( 85, 134, 106), cv::Scalar( 85, 134, 109), cv::Scalar( 86, 135, 111), cv::Scalar( 86, 135, 113), cv::Scalar( 86, 136, 116), cv::Scalar( 86, 136, 118), cv::Scalar( 87, 137, 120), cv::Scalar( 87, 137, 123),
  cv::Scalar( 88, 137, 125), cv::Scalar( 90, 138, 126), cv::Scalar( 92, 138, 128), cv::Scalar( 94, 139, 129), cv::Scalar( 96, 140, 131), cv::Scalar( 98, 140, 133), cv::Scalar( 99, 141, 134), cv::Scalar(101, 141, 136),
  cv::Scalar(103, 142, 138), cv::Scalar(105, 142, 139), cv::Scalar(107, 143, 141), cv::Scalar(109, 143, 142), cv::Scalar(111, 144, 144), cv::Scalar(113, 144, 146), cv::Scalar(115, 145, 147), cv::Scalar(117, 145, 149),
  cv::Scalar(119, 146, 150), cv::Scalar(120, 146, 152), cv::Scalar(122, 147, 153), cv::Scalar(125, 148, 153), cv::Scalar(127, 150, 152), cv::Scalar(129, 151, 152), cv::Scalar(132, 153, 151), cv::Scalar(134, 154, 151),
  cv::Scalar(136, 156, 150), cv::Scalar(139, 157, 150), cv::Scalar(141, 159, 150), cv::Scalar(143, 160, 149), cv::Scalar(145, 162, 149), cv::Scalar(148, 163, 148), cv::Scalar(150, 165, 148), cv::Scalar(152, 166, 147),
  cv::Scalar(155, 168, 147), cv::Scalar(157, 169, 147), cv::Scalar(159, 171, 146), cv::Scalar(161, 172, 146), cv::Scalar(164, 174, 145), cv::Scalar(165, 175, 145), cv::Scalar(165, 177, 145), cv::Scalar(166, 178, 145),
  cv::Scalar(167, 180, 145), cv::Scalar(168, 181, 145), cv::Scalar(169, 183, 145), cv::Scalar(170, 185, 145), cv::Scalar(171, 186, 145), cv::Scalar(172, 188, 145), cv::Scalar(172, 189, 144), cv::Scalar(173, 191, 144),
  cv::Scalar(174, 193, 144), cv::Scalar(175, 194, 144), cv::Scalar(176, 196, 144), cv::Scalar(177, 197, 144), cv::Scalar(178, 199, 144), cv::Scalar(179, 200, 144), cv::Scalar(179, 202, 144), cv::Scalar(179, 203, 145),
  cv::Scalar(179, 204, 147), cv::Scalar(179, 205, 148), cv::Scalar(179, 206, 150), cv::Scalar(179, 207, 151), cv::Scalar(179, 208, 152), cv::Scalar(178, 209, 154), cv::Scalar(178, 210, 155), cv::Scalar(178, 211, 157),
  cv::Scalar(178, 212, 158), cv::Scalar(178, 213, 160), cv::Scalar(178, 214, 161), cv::Scalar(178, 215, 163), cv::Scalar(177, 216, 164), cv::Scalar(177, 217, 166), cv::Scalar(177, 218, 167), cv::Scalar(177, 219, 169),
  cv::Scalar(177, 220, 170), cv::Scalar(176, 220, 173), cv::Scalar(176, 221, 175), cv::Scalar(175, 221, 178), cv::Scalar(175, 222, 180), cv::Scalar(174, 222, 183), cv::Scalar(174, 223, 185), cv::Scalar(174, 223, 188),
  cv::Scalar(173, 224, 191), cv::Scalar(173, 224, 193), cv::Scalar(172, 225, 196), cv::Scalar(172, 225, 198), cv::Scalar(171, 226, 201), cv::Scalar(171, 226, 204), cv::Scalar(170, 226, 206), cv::Scalar(170, 227, 209),
  cv::Scalar(169, 227, 211), cv::Scalar(169, 228, 214), cv::Scalar(168, 228, 217), cv::Scalar(169, 229, 219), cv::Scalar(170, 229, 221), cv::Scalar(172, 229, 223), cv::Scalar(174, 230, 225), cv::Scalar(175, 230, 227),
  cv::Scalar(177, 231, 229), cv::Scalar(179, 231, 231), cv::Scalar(181, 231, 233), cv::Scalar(182, 232, 235), cv::Scalar(184, 232, 237), cv::Scalar(186, 232, 239), cv::Scalar(187, 233, 241), cv::Scalar(189, 233, 243),
  cv::Scalar(191, 233, 245), cv::Scalar(193, 234, 247), cv::Scalar(194, 234, 249), cv::Scalar(196, 234, 251), cv::Scalar(198, 235, 253), cv::Scalar(200, 235, 255), cv::Scalar(203, 236, 255), cv::Scalar(206, 237, 255),
  cv::Scalar(209, 238, 255), cv::Scalar(212, 240, 255), cv::Scalar(215, 241, 255), cv::Scalar(218, 242, 255), cv::Scalar(221, 243, 255), cv::Scalar(224, 244, 255), cv::Scalar(227, 245, 255), cv::Scalar(230, 246, 255),
  cv::Scalar(233, 247, 255), cv::Scalar(237, 248, 255), cv::Scalar(240, 249, 255), cv::Scalar(243, 251, 255), cv::Scalar(246, 252, 255), cv::Scalar(249, 253, 255), cv::Scalar(252, 254, 255), cv::Scalar(255, 255, 255)
};

const cv::Scalar kColormapGray[] =
{
  cv::Scalar(  0,   0,   0), cv::Scalar(  1,   1,   1), cv::Scalar(  2,   2,   2), cv::Scalar(  3,   3,   3), cv::Scalar(  4,   4,   4), cv::Scalar(  5,   5,   5), cv::Scalar(  6,   6,   6), cv::Scalar(  7,   7,   7),
  cv::Scalar(  8,   8,   8), cv::Scalar(  9,   9,   9), cv::Scalar( 10,  10,  10), cv::Scalar( 11,  11,  11), cv::Scalar( 12,  12,  12), cv::Scalar( 13,  13,  13), cv::Scalar( 14,  14,  14), cv::Scalar( 15,  15,  15),
  cv::Scalar( 16,  16,  16), cv::Scalar( 17,  17,  17), cv::Scalar( 18,  18,  18), cv::Scalar( 19,  19,  19), cv::Scalar( 20,  20,  20), cv::Scalar( 21,  21,  21), cv::Scalar( 22,  22,  22), cv::Scalar( 23,  23,  23),
  cv::Scalar( 24,  24,  24), cv::Scalar( 25,  25,  25), cv::Scalar( 26,  26,  26), cv::Scalar( 27,  27,  27), cv::Scalar( 28,  28,  28), cv::Scalar( 29,  29,  29), cv::Scalar( 30,  30,  30), cv::Scalar( 31,  31,  31),
  cv::Scalar( 32,  32,  32), cv::Scalar( 33,  33,  33), cv::Scalar( 34,  34,  34), cv::Scalar( 35,  35,  35), cv::Scalar( 36,  36,  36), cv::Scalar( 37,  37,  37), cv::Scalar( 38,  38,  38), cv::Scalar( 39,  39,  39),
  cv::Scalar( 40,  40,  40), cv::Scalar( 41,  41,  41), cv::Scalar( 42,  42,  42), cv::Scalar( 43,  43,  43), cv::Scalar( 44,  44,  44), cv::Scalar( 45,  45,  45), cv::Scalar( 46,  46,  46), cv::Scalar( 47,  47,  47),
  cv::Scalar( 48,  48,  48), cv::Scalar( 49,  49,  49), cv::Scalar( 50,  50,  50), cv::Scalar( 51,  51,  51), cv::Scalar( 52,  52,  52), cv::Scalar( 53,  53,  53), cv::Scalar( 54,  54,  54), cv::Scalar( 55,  55,  55),
  cv::Scalar( 56,  56,  56), cv::Scalar( 57,  57,  57), cv::Scalar( 58,  58,  58), cv::Scalar( 59,  59,  59), cv::Scalar( 60,  60,  60), cv::Scalar( 61,  61,  61), cv::Scalar( 62,  62,  62), cv::Scalar( 63,  63,  63),
  cv::Scalar( 64,  64,  64), cv::Scalar( 65,  65,  65), cv::Scalar( 66,  66,  66), cv::Scalar( 67,  67,  67), cv::Scalar( 68,  68,  68), cv::Scalar( 69,  69,  69), cv::Scalar( 70,  70,  70), cv::Scalar( 71,  71,  71),
  cv::Scalar( 72,  72,  72), cv::Scalar( 73,  73,  73), cv::Scalar( 74,  74,  74), cv::Scalar( 75,  75,  75), cv::Scalar( 76,  76,  76), cv::Scalar( 77,  77,  77), cv::Scalar( 78,  78,  78), cv::Scalar( 79,  79,  79),
  cv::Scalar( 80,  80,  80), cv::Scalar( 81,  81,  81), cv::Scalar( 82,  82,  82), cv::Scalar( 83,  83,  83), cv::Scalar( 84,  84,  84), cv::Scalar( 85,  85,  85), cv::Scalar( 86,  86,  86), cv::Scalar( 87,  87,  87),
  cv::Scalar( 88,  88,  88), cv::Scalar( 89,  89,  89), cv::Scalar( 90,  90,  90), cv::Scalar( 91,  91,  91), cv::Scalar( 92,  92,  92), cv::Scalar( 93,  93,  93), cv::Scalar( 94,  94,  94), cv::Scalar( 95,  95,  95),
  cv::Scalar( 96,  96,  96), cv::Scalar( 97,  97,  97), cv::Scalar( 98,  98,  98), cv::Scalar( 99,  99,  99), cv::Scalar(100, 100, 100), cv::Scalar(101, 101, 101), cv::Scalar(102, 102, 102), cv::Scalar(103, 103, 103),
  cv::Scalar(104, 104, 104), cv::Scalar(105, 105, 105), cv::Scalar(106, 106, 106), cv::Scalar(107, 107, 107), cv::Scalar(108, 108, 108), cv::Scalar(109, 109, 109), cv::Scalar(110, 110, 110), cv::Scalar(111, 111, 111),
  cv::Scalar(112, 112, 112), cv::Scalar(113, 113, 113), cv::Scalar(114, 114, 114), cv::Scalar(115, 115, 115), cv::Scalar(116, 116, 116), cv::Scalar(117, 117, 117), cv::Scalar(118, 118, 118), cv::Scalar(119, 119, 119),
  cv::Scalar(120, 120, 120), cv::Scalar(121, 121, 121), cv::Scalar(122, 122, 122), cv::Scalar(123, 123, 123), cv::Scalar(124, 124, 124), cv::Scalar(125, 125, 125), cv::Scalar(126, 126, 126), cv::Scalar(127, 127, 127),
  cv::Scalar(128, 128, 128), cv::Scalar(129, 129, 129), cv::Scalar(130, 130, 130), cv::Scalar(131, 131, 131), cv::Scalar(132, 132, 132), cv::Scalar(133, 133, 133), cv::Scalar(134, 134, 134), cv::Scalar(135, 135, 135),
  cv::Scalar(136, 136, 136), cv::Scalar(137, 137, 137), cv::Scalar(138, 138, 138), cv::Scalar(139, 139, 139), cv::Scalar(140, 140, 140), cv::Scalar(141, 141, 141), cv::Scalar(142, 142, 142), cv::Scalar(143, 143, 143),
  cv::Scalar(144, 144, 144), cv::Scalar(145, 145, 145), cv::Scalar(146, 146, 146), cv::Scalar(147, 147, 147), cv::Scalar(148, 148, 148), cv::Scalar(149, 149, 149), cv::Scalar(150, 150, 150), cv::Scalar(151, 151, 151),
  cv::Scalar(152, 152, 152), cv::Scalar(153, 153, 153), cv::Scalar(154, 154, 154), cv::Scalar(155, 155, 155), cv::Scalar(156, 156, 156), cv::Scalar(157, 157, 157), cv::Scalar(158, 158, 158), cv::Scalar(159, 159, 159),
  cv::Scalar(160, 160, 160), cv::Scalar(161, 161, 161), cv::Scalar(162, 162, 162), cv::Scalar(163, 163, 163), cv::Scalar(164, 164, 164), cv::Scalar(165, 165, 165), cv::Scalar(166, 166, 166), cv::Scalar(167, 167, 167),
  cv::Scalar(168, 168, 168), cv::Scalar(169, 169, 169), cv::Scalar(170, 170, 170), cv::Scalar(171, 171, 171), cv::Scalar(172, 172, 172), cv::Scalar(173, 173, 173), cv::Scalar(174, 174, 174), cv::Scalar(175, 175, 175),
  cv::Scalar(176, 176, 176), cv::Scalar(177, 177, 177), cv::Scalar(178, 178, 178), cv::Scalar(179, 179, 179), cv::Scalar(180, 180, 180), cv::Scalar(181, 181, 181), cv::Scalar(182, 182, 182), cv::Scalar(183, 183, 183),
  cv::Scalar(184, 184, 184), cv::Scalar(185, 185, 185), cv::Scalar(186, 186, 186), cv::Scalar(187, 187, 187), cv::Scalar(188, 188, 188), cv::Scalar(189, 189, 189), cv::Scalar(190, 190, 190), cv::Scalar(191, 191, 191),
  cv::Scalar(192, 192, 192), cv::Scalar(193, 193, 193), cv::Scalar(194, 194, 194), cv::Scalar(195, 195, 195), cv::Scalar(196, 196, 196), cv::Scalar(197, 197, 197), cv::Scalar(198, 198, 198), cv::Scalar(199, 199, 199),
  cv::Scalar(200, 200, 200), cv::Scalar(201, 201, 201), cv::Scalar(202, 202, 202), cv::Scalar(203, 203, 203), cv::Scalar(204, 204, 204), cv::Scalar(205, 205, 205), cv::Scalar(206, 206, 206), cv::Scalar(207, 207, 207),
  cv::Scalar(208, 208, 208), cv::Scalar(209, 209, 209), cv::Scalar(210, 210, 210), cv::Scalar(211, 211, 211), cv::Scalar(212, 212, 212), cv::Scalar(213, 213, 213), cv::Scalar(214, 214, 214), cv::Scalar(215, 215, 215),
  cv::Scalar(216, 216, 216), cv::Scalar(217, 217, 217), cv::Scalar(218, 218, 218), cv::Scalar(219, 219, 219), cv::Scalar(220, 220, 220), cv::Scalar(221, 221, 221), cv::Scalar(222, 222, 222), cv::Scalar(223, 223, 223),
  cv::Scalar(224, 224, 224), cv::Scalar(225, 225, 225), cv::Scalar(226, 226, 226), cv::Scalar(227, 227, 227), cv::Scalar(228, 228, 228), cv::Scalar(229, 229, 229), cv::Scalar(230, 230, 230), cv::Scalar(231, 231, 231),
  cv::Scalar(232, 232, 232), cv::Scalar(233, 233, 233), cv::Scalar(234, 234, 234), cv::Scalar(235, 235, 235), cv::Scalar(236, 236, 236), cv::Scalar(237, 237, 237), cv::Scalar(238, 238, 238), cv::Scalar(239, 239, 239),
  cv::Scalar(240, 240, 240), cv::Scalar(241, 241, 241), cv::Scalar(242, 242, 242), cv::Scalar(243, 243, 243), cv::Scalar(244, 244, 244), cv::Scalar(245, 245, 245), cv::Scalar(246, 246, 246), cv::Scalar(247, 247, 247),
  cv::Scalar(248, 248, 248), cv::Scalar(249, 249, 249), cv::Scalar(250, 250, 250), cv::Scalar(251, 251, 251), cv::Scalar(252, 252, 252), cv::Scalar(253, 253, 253), cv::Scalar(254, 254, 254), cv::Scalar(255, 255, 255)
};

const cv::Scalar kColormapHot[] =
{
  cv::Scalar(  0,   0,   0), cv::Scalar(  0,   0,   3), cv::Scalar(  0,   0,   5), cv::Scalar(  0,   0,   8), cv::Scalar(  0,   0,  11), cv::Scalar(  0,   0,  13), cv::Scalar(  0,   0,  16), cv::Scalar(  0,   0,  19),
  cv::Scalar(  0,   0,  21), cv::Scalar(  0,   0,  24), cv::Scalar(  0,   0,  27), cv::Scalar(  0,   0,  29), cv::Scalar(  0,   0,  32), cv::Scalar(  0,   0,  35), cv::Scalar(  0,   0,  37), cv::Scalar(  0,   0,  40),
  cv::Scalar(  0,   0,  43), cv::Scalar(  0,   0,  45), cv::Scalar(  0,   0,  48), cv::Scalar(  0,   0,  51), cv::Scalar(  0,   0,  53), cv::Scalar(  0,   0,  56), cv::Scalar(  0,   0,  59), cv::Scalar(  0,   0,  61),
  cv::Scalar(  0,   0,  64), cv::Scalar(  0,   0,  67), cv::Scalar(  0,   0,  69), cv::Scalar(  0,   0,  72), cv::Scalar(  0,   0,  75), cv::Scalar(  0,   0,  77), cv::Scalar(  0,   0,  80), cv::Scalar(  0,   0,  83),
  cv::Scalar(  0,   0,  85), cv::Scalar(  0,   0,  88), cv::Scalar(  0,   0,  91), cv::Scalar(  0,   0,  93), cv::Scalar(  0,   0,  96), cv::Scalar(  0,   0,  99), cv::Scalar(  0,   0, 101), cv::Scalar(  0,   0, 104),
  cv::Scalar(  0,   0, 107), cv::Scalar(  0,   0, 109), cv::Scalar(  0,   0, 112), cv::Scalar(  0,   0, 115), cv::Scalar(  0,   0, 117), cv::Scalar(  0,   0, 120), cv::Scalar(  0,   0, 123), cv::Scalar(  0,   0, 125),
  cv::Scalar(  0,   0, 128), cv::Scalar(  0,   0, 131), cv::Scalar(  0,   0, 133), cv::Scalar(  0,   0, 136), cv::Scalar(  0,   0, 139), cv::Scalar(  0,   0, 141), cv::Scalar(  0,   0, 144), cv::Scalar(  0,   0, 147),
  cv::Scalar(  0,   0, 149), cv::Scalar(  0,   0, 152), cv::Scalar(  0,   0, 155), cv::Scalar(  0,   0, 157), cv::Scalar(  0,   0, 160), cv::Scalar(  0,   0, 163), cv::Scalar(  0,   0, 165), cv::Scalar(  0,   0, 168),
  cv::Scalar(  0,   0, 171), cv::Scalar(  0,   0, 173), cv::Scalar(  0,   0, 176), cv::Scalar(  0,   0, 179), cv::Scalar(  0,   0, 181), cv::Scalar(  0,   0, 184), cv::Scalar(  0,   0, 187), cv::Scalar(  0,   0, 189),
  cv::Scalar(  0,   0, 192), cv::Scalar(  0,   0, 195), cv::Scalar(  0,   0, 197), cv::Scalar(  0,   0, 200), cv::Scalar(  0,   0, 203), cv::Scalar(  0,   0, 205), cv::Scalar(  0,   0, 208), cv::Scalar(  0,   0, 211),
  cv::Scalar(  0,   0, 213), cv::Scalar(  0,   0, 216), cv::Scalar(  0,   0, 219), cv::Scalar(  0,   0, 221), cv::Scalar(  0,   0, 224), cv::Scalar(  0,   0, 227), cv::Scalar(  0,   0, 229), cv::Scalar(  0,   0, 232),
  cv::Scalar(  0,   0, 235), cv::Scalar(  0,   0, 237), cv::Scalar(  0,   0, 240), cv::Scalar(  0,   0, 243), cv::Scalar(  0,   0, 245), cv::Scalar(  0,   0, 248), cv::Scalar(  0,   0, 251), cv::Scalar(  0,   0, 253),
  cv::Scalar(  0,   1, 255), cv::Scalar(  0,   4, 255), cv::Scalar(  0,   6, 255), cv::Scalar(  0,   9, 255), cv::Scalar(  0,  12, 255), cv::Scalar(  0,  14, 255), cv::Scalar(  0,  17, 255), cv::Scalar(  0,  20, 255),
  cv::Scalar(  0,  22, 255), cv::Scalar(  0,  25, 255), cv::Scalar(  0,  28, 255), cv::Scalar(  0,  30, 255), cv::Scalar(  0,  33, 255), cv::Scalar(  0,  36, 255), cv::Scalar(  0,  38, 255), cv::Scalar(  0,  41, 255),
  cv::Scalar(  0,  44, 255), cv::Scalar(  0,  46, 255), cv::Scalar(  0,  49, 255), cv::Scalar(  0,  52, 255), cv::Scalar(  0,  54, 255), cv::Scalar(  0,  57, 255), cv::Scalar(  0,  60, 255), cv::Scalar(  0,  62, 255),
  cv::Scalar(  0,  65, 255), cv::Scalar(  0,  68, 255), cv::Scalar(  0,  70, 255), cv::Scalar(  0,  73, 255), cv::Scalar(  0,  76, 255), cv::Scalar(  0,  78, 255), cv::Scalar(  0,  81, 255), cv::Scalar(  0,  84, 255),
  cv::Scalar(  0,  86, 255), cv::Scalar(  0,  89, 255), cv::Scalar(  0,  92, 255), cv::Scalar(  0,  94, 255), cv::Scalar(  0,  97, 255), cv::Scalar(  0, 100, 255), cv::Scalar(  0, 102, 255), cv::Scalar(  0, 105, 255),
  cv::Scalar(  0, 108, 255), cv::Scalar(  0, 110, 255), cv::Scalar(  0, 113, 255), cv::Scalar(  0, 116, 255), cv::Scalar(  0, 118, 255), cv::Scalar(  0, 121, 255), cv::Scalar(  0, 124, 255), cv::Scalar(  0, 126, 255),
  cv::Scalar(  0, 129, 255), cv::Scalar(  0, 132, 255), cv::Scalar(  0, 134, 255), cv::Scalar(  0, 137, 255), cv::Scalar(  0, 140, 255), cv::Scalar(  0, 142, 255), cv::Scalar(  0, 145, 255), cv::Scalar(  0, 148, 255),
  cv::Scalar(  0, 150, 255), cv::Scalar(  0, 153, 255), cv::Scalar(  0, 156, 255), cv::Scalar(  0, 158, 255), cv::Scalar(  0, 161, 255), cv::Scalar(  0, 164, 255), cv::Scalar(  0, 166, 255), cv::Scalar(  0, 169, 255),
  cv::Scalar(  0, 172, 255), cv::Scalar(  0, 174, 255), cv::Scalar(  0, 177, 255), cv::Scalar(  0, 180, 255), cv::Scalar(  0, 182, 255), cv::Scalar(  0, 185, 255), cv::Scalar(  0, 188, 255), cv::Scalar(  0, 190, 255),
  cv::Scalar(  0, 193, 255), cv::Scalar(  0, 196, 255), cv::Scalar(  0, 198, 255), cv::Scalar(  0, 201, 255), cv::Scalar(  0, 204, 255), cv::Scalar(  0, 206, 255), cv::Scalar(  0, 209, 255), cv::Scalar(  0, 212, 255),
  cv::Scalar(  0, 214, 255), cv::Scalar(  0, 217, 255), cv::Scalar(  0, 220, 255), cv::Scalar(  0, 222, 255), cv::Scalar(  0, 225, 255), cv::Scalar(  0, 228, 255), cv::Scalar(  0, 230, 255), cv::Scalar(  0, 233, 255),
  cv::Scalar(  0, 236, 255), cv::Scalar(  0, 238, 255), cv::Scalar(  0, 241, 255), cv::Scalar(  0, 244, 255), cv::Scalar(  0, 246, 255), cv::Scalar(  0, 249, 255), cv::Scalar(  0, 252, 255), cv::Scalar(  0, 254, 255),
  cv::Scalar(  3, 255, 255), cv::Scalar(  7, 255, 255), cv::Scalar( 11, 255, 255), cv::Scalar( 15, 255, 255), cv::Scalar( 19, 255, 255), cv::Scalar( 23, 255, 255), cv::Scalar( 27, 255, 255), cv::Scalar( 31, 255, 255),
  cv::Scalar( 35, 255, 255), cv::Scalar( 39, 255, 255), cv::Scalar( 43, 255, 255), cv::Scalar( 47, 255, 255), cv::Scalar( 51, 255, 255), cv::Scalar( 55, 255, 255), cv::Scalar( 59, 255, 255), cv::Scalar( 63, 255, 255),
  cv::Scalar( 67, 255, 255), cv::Scalar( 71, 255, 255), cv::Scalar( 75, 255, 255), cv::Scalar( 79, 255, 255), cv::Scalar( 83, 255, 255), cv::Scalar( 87, 255, 255), cv::Scalar( 91, 255, 255), cv::Scalar( 95, 255, 255),
  cv::Scalar( 99, 255, 255), cv::Scalar(103, 255, 255), cv::Scalar(107, 255, 255), cv::Scalar(111, 255, 255), cv::Scalar(115, 255, 255), cv::Scalar(119, 255, 255), cv::Scalar(123, 255, 255), cv::Scalar(127, 255, 255),
  cv::Scalar(131, 255, 255), cv::Scalar(135, 255, 255), cv::Scalar(139, 255, 255), cv::Scalar(143, 255, 255), cv::Scalar(147, 255, 255), cv::Scalar(151, 255, 255), cv::Scalar(155, 255, 255), cv::Scalar(159, 255, 255),
  cv::Scalar(163, 255, 255), cv::Scalar(167, 255, 255), cv::Scalar(171, 255, 255), cv::Scalar(175, 255, 255), cv::Scalar(179, 255, 255), cv::Scalar(183, 255, 255), cv::Scalar(187, 255, 255), cv::Scalar(191, 255, 255),
  cv::Scalar(195, 255, 255), cv::Scalar(199, 255, 255), cv::Scalar(203, 255, 255), cv::Scalar(207, 255, 255), cv::Scalar(211, 255, 255), cv::Scalar(215, 255, 255), cv::Scalar(219, 255, 255), cv::Scalar(223, 255, 255),
  cv::Scalar(227, 255, 255), cv::Scalar(231, 255, 255), cv::Scalar(235, 255, 255), cv::Scalar(239, 255, 255), cv::Scalar(243, 255, 255), cv::Scalar(247, 255, 255), cv::Scalar(251, 255, 255), cv::Scalar(255, 255, 255)
};

const cv::Scalar kColormapHSV[] =
{
  cv::Scalar(  0,   0, 255), cv::Scalar(  0,   6, 255), cv::Scalar(  0,  12, 255), cv::Scalar(  0,  18, 255), cv::Scalar(  0,  24, 255), cv::Scalar(  0,  30, 255), cv::Scalar(  0,  36, 255), cv::Scalar(  0,  42, 255),
  cv::Scalar(  0,  48, 255), cv::Scalar(  0,  54, 255), cv::Scalar(  0,  60, 255), cv::Scalar(  0,  66, 255), cv::Scalar(  0,  72, 255), cv::Scalar(  0,  78, 255), cv::Scalar(  0,  84, 255), cv::Scalar(  0,  90, 255),
  cv::Scalar(  0,  96, 255), cv::Scalar(  0, 102, 255), cv::Scalar(  0, 108, 255), cv::Scalar(  0, 114, 255), cv::Scalar(  0, 120, 255), cv::Scalar(  0, 126, 255), cv::Scalar(  0, 132, 255), cv::Scalar(  0, 138, 255),
  cv::Scalar(  0, 144, 255), cv::Scalar(  0, 150, 255), cv::Scalar(  0, 156, 255), cv::Scalar(  0, 162, 255), cv::Scalar(  0, 168, 255), cv::Scalar(  0, 174, 255), cv::Scalar(  0, 180, 255), cv::Scalar(  0, 186, 255),
  cv::Scalar(  0, 192, 255), cv::Scalar(  0, 198, 255), cv::Scalar(  0, 204, 255), cv::Scalar(  0, 210, 255), cv::Scalar(  0, 216, 255), cv::Scalar(  0, 222, 255), cv::Scalar(  0, 228, 255), cv::Scalar(  0, 234, 255),
  cv::Scalar(  0, 240, 255), cv::Scalar(  0, 246, 255), cv::Scalar(  0, 252, 255), cv::Scalar(  0, 255, 252), cv::Scalar(  0, 255, 246), cv::Scalar(  0, 255, 240), cv::Scalar(  0, 255, 234), cv::Scalar(  0, 255, 228),
  cv::Scalar(  0, 255, 222), cv::Scalar(  0, 255, 216), cv::Scalar(  0, 255, 210), cv::Scalar(  0, 255, 204), cv::Scalar(  0, 255, 198), cv::Scalar(  0, 255, 192), cv::Scalar(  0, 255, 186), cv::Scalar(  0, 255, 180),
  cv::Scalar(  0, 255, 174), cv::Scalar(  0, 255, 168), cv::Scalar(  0, 255, 162), cv::Scalar(  0, 255, 156), cv::Scalar(  0, 255, 150), cv::Scalar(  0, 255, 144), cv::Scalar(  0, 255, 138), cv::Scalar(  0, 255, 132),
  cv::Scalar(  0, 255, 126), cv::Scalar(  0, 255, 120), cv::Scalar(  0, 255, 114), cv::Scalar(  0, 255, 108), cv::Scalar(  0, 255, 102), cv::Scalar(  0, 255,  96), cv::Scalar(  0, 255,  90), cv::Scalar(  0, 255,  84),
  cv::Scalar(  0, 255,  78), cv::Scalar(  0, 255,  72), cv::Scalar(  0, 255,  66), cv::Scalar(  0, 255,  60), cv::Scalar(  0, 255,  54), cv::Scalar(  0, 255,  48), cv::Scalar(  0, 255,  42), cv::Scalar(  0, 255,  36),
  cv::Scalar(  0, 255,  30), cv::Scalar(  0, 255,  24), cv::Scalar(  0, 255,  18), cv::Scalar(  0, 255,  12), cv::Scalar(  0, 255,   6), cv::Scalar(  0, 255,   0), cv::Scalar(  6, 255,   0), cv::Scalar( 12, 255,   0),
  cv::Scalar( 18, 255,   0), cv::Scalar( 24, 255,   0), cv::Scalar( 30, 255,   0), cv::Scalar( 36, 255,   0), cv::Scalar( 42, 255,   0), cv::Scalar( 48, 255,   0), cv::Scalar( 54, 255,   0), cv::Scalar( 60, 255,   0),
  cv::Scalar( 66, 255,   0), cv::Scalar( 72, 255,   0), cv::Scalar( 78, 255,   0), cv::Scalar( 84, 255,   0), cv::Scalar( 90, 255,   0), cv::Scalar( 96, 255,   0), cv::Scalar(102, 255,   0), cv::Scalar(108, 255,   0),
  cv::Scalar(114, 255,   0), cv::Scalar(120, 255,   0), cv::Scalar(126, 255,   0), cv::Scalar(132, 255,   0), cv::Scalar(138, 255,   0), cv::Scalar(144, 255,   0), cv::Scalar(150, 255,   0), cv::Scalar(156, 255,   0),
  cv::Scalar(162, 255,   0), cv::Scalar(168, 255,   0), cv::Scalar(174, 255,   0), cv::Scalar(180, 255,   0), cv::Scalar(186, 255,   0), cv::Scalar(192, 255,   0), cv::Scalar(198, 255,   0), cv::Scalar(204, 255,   0),
  cv::Scalar(210, 255,   0), cv::Scalar(216, 255,   0), cv::Scalar(222, 255,   0), cv::Scalar(228, 255,   0), cv::Scalar(234, 255,   0), cv::Scalar(240, 255,   0), cv::Scalar(246, 255,   0), cv::Scalar(252, 255,   0),
  cv::Scalar(255, 252,   0), cv::Scalar(255, 246,   0), cv::Scalar(255, 240,   0), cv::Scalar(255, 234,   0), cv::Scalar(255, 228,   0), cv::Scalar(255, 222,   0), cv::Scalar(255, 216,   0), cv::Scalar(255, 210,   0),
  cv::Scalar(255, 204,   0), cv::Scalar(255, 198,   0), cv::Scalar(255, 192,   0), cv::Scalar(255, 186,   0), cv::Scalar(255, 180,   0), cv::Scalar(255, 174,   0), cv::Scalar(255, 168,   0), cv::Scalar(255, 162,   0),
  cv::Scalar(255, 156,   0), cv::Scalar(255, 150,   0), cv::Scalar(255, 144,   0), cv::Scalar(255, 138,   0), cv::Scalar(255, 132,   0), cv::Scalar(255, 126,   0), cv::Scalar(255, 120,   0), cv::Scalar(255, 114,   0),
  cv::Scalar(255, 108,   0), cv::Scalar(255, 102,   0), cv::Scalar(255,  96,   0), cv::Scalar(255,  90,   0), cv::Scalar(255,  84,   0), cv::Scalar(255,  78,   0), cv::Scalar(255,  72,   0), cv::Scalar(255,  66,   0),
  cv::Scalar(255,  60,   0), cv::Scalar(255,  54,   0), cv::Scalar(255,  48,   0), cv::Scalar(255,  42,   0), cv::Scalar(255,  36,   0), cv::Scalar(255,  30,   0), cv::Scalar(255,  24,   0), cv::Scalar(255,  18,   0),
  cv::Scalar(255,  12,   0), cv::Scalar(255,   6,   0), cv::Scalar(255,   0,   0), cv::Scalar(255,   0,   6), cv::Scalar(255,   0,  12), cv::Scalar(255,   0,  18), cv::Scalar(255,   0,  24), cv::Scalar(255,   0,  30),
  cv::Scalar(255,   0,  36), cv::Scalar(255,   0,  42), cv::Scalar(255,   0,  48), cv::Scalar(255,   0,  54), cv::Scalar(255,   0,  60), cv::Scalar(255,   0,  66), cv::Scalar(255,   0,  72), cv::Scalar(255,   0,  78),
  cv::Scalar(255,   0,  84), cv::Scalar(255,   0,  90), cv::Scalar(255,   0,  96), cv::Scalar(255,   0, 102), cv::Scalar(255,   0, 108), cv::Scalar(255,   0, 114), cv::Scalar(255,   0, 120), cv::Scalar(255,   0, 126),
  cv::Scalar(255,   0, 132), cv::Scalar(255,   0, 138), cv::Scalar(255,   0, 144), cv::Scalar(255,   0, 150), cv::Scalar(255,   0, 156), cv::Scalar(255,   0, 162), cv::Scalar(255,   0, 168), cv::Scalar(255,   0, 174),
  cv::Scalar(255,   0, 180), cv::Scalar(255,   0, 186), cv::Scalar(255,   0, 192), cv::Scalar(255,   0, 198), cv::Scalar(255,   0, 204), cv::Scalar(255,   0, 210), cv::Scalar(255,   0, 216), cv::Scalar(255,   0, 222),
  cv::Scalar(255,   0, 228), cv::Scalar(255,   0, 234), cv::Scalar(255,   0, 240), cv::Scalar(255,   0, 246), cv::Scalar(255,   0, 252), cv::Scalar(252,   0, 255), cv::Scalar(246,   0, 255), cv::Scalar(240,   0, 255),
  cv::Scalar(234,   0, 255), cv::Scalar(228,   0, 255), cv::Scalar(222,   0, 255), cv::Scalar(216,   0, 255), cv::Scalar(210,   0, 255), cv::Scalar(204,   0, 255), cv::Scalar(198,   0, 255), cv::Scalar(192,   0, 255),
  cv::Scalar(186,   0, 255), cv::Scalar(180,   0, 255), cv::Scalar(174,   0, 255), cv::Scalar(168,   0, 255), cv::Scalar(162,   0, 255), cv::Scalar(156,   0, 255), cv::Scalar(150,   0, 255), cv::Scalar(144,   0, 255),
  cv::Scalar(138,   0, 255), cv::Scalar(132,   0, 255), cv::Scalar(126,   0, 255), cv::Scalar(120,   0, 255), cv::Scalar(114,   0, 255), cv::Scalar(108,   0, 255), cv::Scalar(102,   0, 255), cv::Scalar( 96,   0, 255),
  cv::Scalar( 90,   0, 255), cv::Scalar( 84,   0, 255), cv::Scalar( 78,   0, 255), cv::Scalar( 72,   0, 255), cv::Scalar( 66,   0, 255), cv::Scalar( 60,   0, 255), cv::Scalar( 54,   0, 255), cv::Scalar( 48,   0, 255),
  cv::Scalar( 42,   0, 255), cv::Scalar( 36,   0, 255), cv::Scalar( 30,   0, 255), cv::Scalar( 24,   0, 255), cv::Scalar( 18,   0, 255), cv::Scalar( 12,   0, 255), cv::Scalar(  6,   0, 255), cv::Scalar(  0,   0, 255)
};

const cv::Scalar kColormapInferno[] =
{
  cv::Scalar(  4,   0,   0), cv::Scalar(  5,   0,   1), cv::Scalar(  6,   1,   1), cv::Scalar(  8,   1,   1), cv::Scalar( 10,   1,   2), cv::Scalar( 12,   2,   2), cv::Scalar( 14,   2,   2), cv::Scalar( 16,   2,   3),
  cv::Scalar( 18,   3,   4), cv::Scalar( 20,   3,   4), cv::Scalar( 23,   4,   5), cv::Scalar( 25,   4,   6), cv::Scalar( 27,   5,   7), cv::Scalar( 29,   5,   8), cv::Scalar( 31,   6,   9), cv::Scalar( 34,   7,  10),
  cv::Scalar( 36,   7,  11), cv::Scalar( 38,   8,  12), cv::Scalar( 41,   8,  13), cv::Scalar( 43,   9,  14), cv::Scalar( 45,   9,  16), cv::Scalar( 48,  10,  17), cv::Scalar( 50,  10,  18), cv::Scalar( 52,  11,  20),
  cv::Scalar( 55,  11,  21), cv::Scalar( 57,  11,  22), cv::Scalar( 60,  12,  24), cv::Scalar( 62,  12,  25), cv::Scalar( 65,  12,  27), cv::Scalar( 67,  12,  28), cv::Scalar( 69,  12,  30), cv::Scalar( 72,  12,  31),
  cv::Scalar( 74,  12,  33), cv::Scalar( 76,  12,  35), cv::Scalar( 79,  12,  36), cv::Scalar( 81,  12,  38), cv::Scalar( 83,  11,  40), cv::Scalar( 85,  11,  41), cv::Scalar( 87,  11,  43), cv::Scalar( 89,  11,  45),
  cv::Scalar( 91,  10,  47), cv::Scalar( 92,  10,  49), cv::Scalar( 94,  10,  50), cv::Scalar( 95,  10,  52), cv::Scalar( 97,   9,  54), cv::Scalar( 98,   9,  56), cv::Scalar( 99,   9,  57), cv::Scalar(100,   9,  59),
  cv::Scalar(101,   9,  61), cv::Scalar(102,   9,  62), cv::Scalar(103,  10,  64), cv::Scalar(104,  10,  66), cv::Scalar(104,  10,  68), cv::Scalar(105,  10,  69), cv::Scalar(106,  11,  71), cv::Scalar(106,  11,  73),
  cv::Scalar(107,  12,  74), cv::Scalar(107,  12,  76), cv::Scalar(108,  13,  77), cv::Scalar(108,  13,  79), cv::Scalar(108,  14,  81), cv::Scalar(109,  14,  82), cv::Scalar(109,  15,  84), cv::Scalar(109,  15,  85),
  cv::Scalar(110,  16,  87), cv::Scalar(110,  16,  89), cv::Scalar(110,  17,  90), cv::Scalar(110,  18,  92), cv::Scalar(110,  18,  93), cv::Scalar(110,  19,  95), cv::Scalar(110,  19,  97), cv::Scalar(110,  20,  98),
  cv::Scalar(110,  21, 100), cv::Scalar(110,  21, 101), cv::Scalar(110,  22, 103), cv::Scalar(110,  22, 105), cv::Scalar(110,  23, 106), cv::Scalar(110,  24, 108), cv::Scalar(110,  24, 109), cv::Scalar(110,  25, 111),
  cv::Scalar(110,  25, 113), cv::Scalar(110,  26, 114), cv::Scalar(110,  26, 116), cv::Scalar(110,  27, 117), cv::Scalar(109,  28, 119), cv::Scalar(109,  28, 120), cv::Scalar(109,  29, 122), cv::Scalar(109,  29, 124),
  cv::Scalar(109,  30, 125), cv::Scalar(108,  30, 127), cv::Scalar(108,  31, 128), cv::Scalar(108,  32, 130), cv::Scalar(107,  32, 132), cv::Scalar(107,  33, 133), cv::Scalar(107,  33, 135), cv::Scalar(106,  34, 136),
  cv::Scalar(106,  34, 138), cv::Scalar(105,  35, 140), cv::Scalar(105,  35, 141), cv::Scalar(105,  36, 143), cv::Scalar(104,  37, 144), cv::Scalar(104,  37, 146), cv::Scalar(103,  38, 147), cv::Scalar(103,  38, 149),
  cv::Scalar(102,  39, 151), cv::Scalar(102,  39, 152), cv::Scalar(101,  40, 154), cv::Scalar(100,  41, 155), cv::Scalar(100,  41, 157), cv::Scalar( 99,  42, 159), cv::Scalar( 99,  42, 160), cv::Scalar( 98,  43, 162),
  cv::Scalar( 97,  44, 163), cv::Scalar( 96,  44, 165), cv::Scalar( 96,  45, 166), cv::Scalar( 95,  46, 168), cv::Scalar( 94,  46, 169), cv::Scalar( 94,  47, 171), cv::Scalar( 93,  48, 173), cv::Scalar( 92,  48, 174),
  cv::Scalar( 91,  49, 176), cv::Scalar( 90,  50, 177), cv::Scalar( 90,  50, 179), cv::Scalar( 89,  51, 180), cv::Scalar( 88,  52, 182), cv::Scalar( 87,  53, 183), cv::Scalar( 86,  53, 185), cv::Scalar( 85,  54, 186),
  cv::Scalar( 84,  55, 188), cv::Scalar( 83,  56, 189), cv::Scalar( 82,  57, 191), cv::Scalar( 81,  58, 192), cv::Scalar( 80,  58, 193), cv::Scalar( 79,  59, 195), cv::Scalar( 78,  60, 196), cv::Scalar( 77,  61, 198),
  cv::Scalar( 76,  62, 199), cv::Scalar( 75,  63, 200), cv::Scalar( 74,  64, 202), cv::Scalar( 73,  65, 203), cv::Scalar( 72,  66, 204), cv::Scalar( 71,  67, 206), cv::Scalar( 70,  68, 207), cv::Scalar( 69,  69, 208),
  cv::Scalar( 68,  70, 210), cv::Scalar( 67,  71, 211), cv::Scalar( 66,  72, 212), cv::Scalar( 65,  74, 213), cv::Scalar( 63,  75, 215), cv::Scalar( 62,  76, 216), cv::Scalar( 61,  77, 217), cv::Scalar( 60,  78, 218),
  cv::Scalar( 59,  80, 219), cv::Scalar( 58,  81, 221), cv::Scalar( 56,  82, 222), cv::Scalar( 55,  83, 223), cv::Scalar( 54,  85, 224), cv::Scalar( 53,  86, 225), cv::Scalar( 52,  87, 226), cv::Scalar( 51,  89, 227),
  cv::Scalar( 49,  90, 228), cv::Scalar( 48,  92, 229), cv::Scalar( 47,  93, 230), cv::Scalar( 46,  94, 231), cv::Scalar( 45,  96, 232), cv::Scalar( 43,  97, 233), cv::Scalar( 42,  99, 234), cv::Scalar( 41, 100, 235),
  cv::Scalar( 40, 102, 235), cv::Scalar( 38, 103, 236), cv::Scalar( 37, 105, 237), cv::Scalar( 36, 106, 238), cv::Scalar( 35, 108, 239), cv::Scalar( 33, 110, 239), cv::Scalar( 32, 111, 240), cv::Scalar( 31, 113, 241),
  cv::Scalar( 29, 115, 241), cv::Scalar( 28, 116, 242), cv::Scalar( 27, 118, 243), cv::Scalar( 25, 120, 243), cv::Scalar( 24, 121, 244), cv::Scalar( 23, 123, 245), cv::Scalar( 21, 125, 245), cv::Scalar( 20, 126, 246),
  cv::Scalar( 19, 128, 246), cv::Scalar( 18, 130, 247), cv::Scalar( 16, 132, 247), cv::Scalar( 15, 133, 248), cv::Scalar( 14, 135, 248), cv::Scalar( 12, 137, 248), cv::Scalar( 11, 139, 249), cv::Scalar( 10, 140, 249),
  cv::Scalar(  9, 142, 249), cv::Scalar(  8, 144, 250), cv::Scalar(  7, 146, 250), cv::Scalar(  7, 148, 250), cv::Scalar(  6, 150, 251), cv::Scalar(  6, 151, 251), cv::Scalar(  6, 153, 251), cv::Scalar(  6, 155, 251),
  cv::Scalar(  7, 157, 251), cv::Scalar(  7, 159, 252), cv::Scalar(  8, 161, 252), cv::Scalar(  9, 163, 252), cv::Scalar( 10, 165, 252), cv::Scalar( 12, 166, 252), cv::Scalar( 13, 168, 252), cv::Scalar( 15, 170, 252),
  cv::Scalar( 17, 172, 252), cv::Scalar( 18, 174, 252), cv::Scalar( 20, 176, 252), cv::Scalar( 22, 178, 252), cv::Scalar( 24, 180, 252), cv::Scalar( 26, 182, 251), cv::Scalar( 29, 184, 251), cv::Scalar( 31, 186, 251),
  cv::Scalar( 33, 188, 251), cv::Scalar( 35, 190, 251), cv::Scalar( 38, 192, 250), cv::Scalar( 40, 194, 250), cv::Scalar( 42, 196, 250), cv::Scalar( 45, 198, 250), cv::Scalar( 47, 199, 249), cv::Scalar( 50, 201, 249),
  cv::Scalar( 53, 203, 249), cv::Scalar( 55, 205, 248), cv::Scalar( 58, 207, 248), cv::Scalar( 61, 209, 247), cv::Scalar( 64, 211, 247), cv::Scalar( 67, 213, 246), cv::Scalar( 70, 215, 246), cv::Scalar( 73, 217, 245),
  cv::Scalar( 76, 219, 245), cv::Scalar( 79, 221, 244), cv::Scalar( 83, 223, 244), cv::Scalar( 86, 225, 244), cv::Scalar( 90, 227, 243), cv::Scalar( 93, 229, 243), cv::Scalar( 97, 230, 242), cv::Scalar(101, 232, 242),
  cv::Scalar(105, 234, 242), cv::Scalar(109, 236, 241), cv::Scalar(113, 237, 241), cv::Scalar(117, 239, 241), cv::Scalar(121, 241, 241), cv::Scalar(125, 242, 242), cv::Scalar(130, 244, 242), cv::Scalar(134, 245, 243),
  cv::Scalar(138, 246, 243), cv::Scalar(142, 248, 244), cv::Scalar(146, 249, 245), cv::Scalar(150, 250, 246), cv::Scalar(154, 251, 248), cv::Scalar(157, 252, 249), cv::Scalar(161, 253, 250), cv::Scalar(164, 255, 252)
};

const cv::Scalar kColormapJet[] =
{
  cv::Scalar(128,   0,   0), cv::Scalar(132,   0,   0), cv::Scalar(136,   0,   0), cv::Scalar(140,   0,   0), cv::Scalar(144,   0,   0), cv::Scalar(147,   0,   0), cv::Scalar(152,   0,   0), cv::Scalar(156,   0,   0),
  cv::Scalar(160,   0,   0), cv::Scalar(163,   0,   0), cv::Scalar(168,   0,   0), cv::Scalar(172,   0,   0), cv::Scalar(176,   0,   0), cv::Scalar(179,   0,   0), cv::Scalar(184,   0,   0), cv::Scalar(188,   0,   0),
  cv::Scalar(192,   0,   0), cv::Scalar(195,   0,   0), cv::Scalar(200,   0,   0), cv::Scalar(204,   0,   0), cv::Scalar(208,   0,   0), cv::Scalar(211,   0,   0), cv::Scalar(216,   0,   0), cv::Scalar(220,   0,   0),
  cv::Scalar(224,   0,   0), cv::Scalar(227,   0,   0), cv::Scalar(232,   0,   0), cv::Scalar(236,   0,   0), cv::Scalar(240,   0,   0), cv::Scalar(243,   0,   0), cv::Scalar(248,   0,   0), cv::Scalar(252,   0,   0),
  cv::Scalar(255,   0,   0), cv::Scalar(255,   4,   0), cv::Scalar(255,   8,   0), cv::Scalar(255,  13,   0), cv::Scalar(255,  16,   0), cv::Scalar(255,  20,   0), cv::Scalar(255,  24,   0), cv::Scalar(255,  29,   0),
  cv::Scalar(255,  33,   0), cv::Scalar(255,  36,   0), cv::Scalar(255,  41,   0), cv::Scalar(255,  45,   0), cv::Scalar(255,  49,   0), cv::Scalar(255,  52,   0), cv::Scalar(255,  57,   0), cv::Scalar(255,  61,   0),
  cv::Scalar(255,  65,   0), cv::Scalar(255,  68,   0), cv::Scalar(255,  73,   0), cv::Scalar(255,  77,   0), cv::Scalar(255,  81,   0), cv::Scalar(255,  84,   0), cv::Scalar(255,  89,   0), cv::Scalar(255,  93,   0),
  cv::Scalar(255,  97,   0), cv::Scalar(255, 100,   0), cv::Scalar(255, 105,   0), cv::Scalar(255, 109,   0), cv::Scalar(255, 113,   0), cv::Scalar(255, 116,   0), cv::Scalar(255, 121,   0), cv::Scalar(255, 125,   0),
  cv::Scalar(255, 129,   0), cv::Scalar(255, 132,   0), cv::Scalar(255, 136,   0), cv::Scalar(255, 141,   0), cv::Scalar(255, 145,   0), cv::Scalar(255, 148,   0), cv::Scalar(255, 153,   0), cv::Scalar(255, 157,   0),
  cv::Scalar(255, 161,   0), cv::Scalar(255, 164,   0), cv::Scalar(255, 168,   0), cv::Scalar(255, 173,   0), cv::Scalar(255, 177,   0), cv::Scalar(255, 180,   0), cv::Scalar(255, 185,   0), cv::Scalar(255, 189,   0),
  cv::Scalar(255, 193,   0), cv::Scalar(255, 196,   0), cv::Scalar(255, 200,   0), cv::Scalar(255, 205,   0), cv::Scalar(255, 209,   0), cv::Scalar(255, 212,   0), cv::Scalar(255, 217,   0), cv::Scalar(255, 221,   0),
  cv::Scalar(255, 225,   0), cv::Scalar(255, 228,   0), cv::Scalar(255, 232,   0), cv::Scalar(255, 237,   0), cv::Scalar(255, 241,   0), cv::Scalar(255, 244,   0), cv::Scalar(255, 249,   0), cv::Scalar(255, 253,   0),
  cv::Scalar(254, 255,   1), cv::Scalar(250, 255,   5), cv::Scalar(246, 255,   9), cv::Scalar(242, 255,  14), cv::Scalar(238, 255,  17), cv::Scalar(234, 255,  21), cv::Scalar(229, 255,  26), cv::Scalar(226, 255,  30),
  cv::Scalar(222, 255,  33), cv::Scalar(218, 255,  37), cv::Scalar(214, 255,  41), cv::Scalar(210, 255,  46), cv::Scalar(206, 255,  49), cv::Scalar(202, 255,  53), cv::Scalar(197, 255,  58), cv::Scalar(194, 255,  62),
  cv::Scalar(190, 255,  66), cv::Scalar(186, 255,  69), cv::Scalar(182, 255,  73), cv::Scalar(178, 255,  78), cv::Scalar(174, 255,  82), cv::Scalar(170, 255,  85), cv::Scalar(165, 255,  90), cv::Scalar(162, 255,  94),
  cv::Scalar(158, 255,  98), cv::Scalar(154, 255, 101), cv::Scalar(150, 255, 105), cv::Scalar(146, 255, 110), cv::Scalar(142, 255, 114), cv::Scalar(138, 255, 117), cv::Scalar(133, 255, 122), cv::Scalar(130, 255, 126),
  cv::Scalar(126, 255, 130), cv::Scalar(122, 255, 133), cv::Scalar(118, 255, 137), cv::Scalar(114, 255, 141), cv::Scalar(110, 255, 145), cv::Scalar(105, 255, 150), cv::Scalar(101, 255, 154), cv::Scalar( 98, 255, 158),
  cv::Scalar( 94, 255, 162), cv::Scalar( 90, 255, 165), cv::Scalar( 86, 255, 169), cv::Scalar( 82, 255, 173), cv::Scalar( 77, 255, 178), cv::Scalar( 73, 255, 182), cv::Scalar( 69, 255, 186), cv::Scalar( 66, 255, 190),
  cv::Scalar( 62, 255, 194), cv::Scalar( 58, 255, 197), cv::Scalar( 54, 255, 201), cv::Scalar( 50, 255, 205), cv::Scalar( 46, 255, 209), cv::Scalar( 41, 255, 214), cv::Scalar( 37, 255, 218), cv::Scalar( 33, 255, 222),
  cv::Scalar( 30, 255, 226), cv::Scalar( 26, 255, 229), cv::Scalar( 22, 255, 233), cv::Scalar( 18, 255, 237), cv::Scalar( 13, 255, 242), cv::Scalar(  9, 255, 246), cv::Scalar(  5, 255, 250), cv::Scalar(  1, 255, 254),
  cv::Scalar(  0, 253, 255), cv::Scalar(  0, 249, 255), cv::Scalar(  0, 245, 255), cv::Scalar(  0, 241, 255), cv::Scalar(  0, 237, 255), cv::Scalar(  0, 232, 255), cv::Scalar(  0, 228, 255), cv::Scalar(  0, 225, 255),
  cv::Scalar(  0, 221, 255), cv::Scalar(  0, 217, 255), cv::Scalar(  0, 213, 255), cv::Scalar(  0, 209, 255), cv::Scalar(  0, 204, 255), cv::Scalar(  0, 200, 255), cv::Scalar(  0, 196, 255), cv::Scalar(  0, 193, 255),
  cv::Scalar(  0, 189, 255), cv::Scalar(  0, 185, 255), cv::Scalar(  0, 181, 255), cv::Scalar(  0, 177, 255), cv::Scalar(  0, 173, 255), cv::Scalar(  0, 168, 255), cv::Scalar(  0, 164, 255), cv::Scalar(  0, 161, 255),
  cv::Scalar(  0, 157, 255), cv::Scalar(  0, 153, 255), cv::Scalar(  0, 149, 255), cv::Scalar(  0, 145, 255), cv::Scalar(  0, 140, 255), cv::Scalar(  0, 136, 255), cv::Scalar(  0, 132, 255), cv::Scalar(  0, 129, 255),
  cv::Scalar(  0, 125, 255), cv::Scalar(  0, 121, 255), cv::Scalar(  0, 117, 255), cv::Scalar(  0, 113, 255), cv::Scalar(  0, 109, 255), cv::Scalar(  0, 104, 255), cv::Scalar(  0, 100, 255), cv::Scalar(  0,  97, 255),
  cv::Scalar(  0,  93, 255), cv::Scalar(  0,  89, 255), cv::Scalar(  0,  85, 255), cv::Scalar(  0,  81, 255), cv::Scalar(  0,  76, 255), cv::Scalar(  0,  72, 255), cv::Scalar(  0,  68, 255), cv::Scalar(  0,  65, 255),
  cv::Scalar(  0,  61, 255), cv::Scalar(  0,  57, 255), cv::Scalar(  0,  53, 255), cv::Scalar(  0,  49, 255), cv::Scalar(  0,  45, 255), cv::Scalar(  0,  40, 255), cv::Scalar(  0,  36, 255), cv::Scalar(  0,  33, 255),
  cv::Scalar(  0,  29, 255), cv::Scalar(  0,  25, 255), cv::Scalar(  0,  21, 255), cv::Scalar(  0,  17, 255), cv::Scalar(  0,  12, 255), cv::Scalar(  0,   8, 255), cv::Scalar(  0,   4, 255), cv::Scalar(  0,   0, 255),
  cv::Scalar(  0,   0, 252), cv::Scalar(  0,   0, 248), cv::Scalar(  0,   0, 244), cv::Scalar(  0,   0, 240), cv::Scalar(  0,   0, 236), cv::Scalar(  0,   0, 231), cv::Scalar(  0,   0, 227), cv::Scalar(  0,   0, 224),
  cv::Scalar(  0,   0, 220), cv::Scalar(  0,   0, 216), cv::Scalar(  0,   0, 212), cv::Scalar(  0,   0, 208), cv::Scalar(  0,   0, 203), cv::Scalar(  0,   0, 199), cv::Scalar(  0,   0, 195), cv::Scalar(  0,   0, 192),
  cv::Scalar(  0,   0, 188), cv::Scalar(  0,   0, 184), cv::Scalar(  0,   0, 180), cv::Scalar(  0,   0, 176), cv::Scalar(  0,   0, 172), cv::Scalar(  0,   0, 167), cv::Scalar(  0,   0, 163), cv::Scalar(  0,   0, 160),
  cv::Scalar(  0,   0, 156), cv::Scalar(  0,   0, 152), cv::Scalar(  0,   0, 148), cv::Scalar(  0,   0, 144), cv::Scalar(  0,   0, 139), cv::Scalar(  0,   0, 135), cv::Scalar(  0,   0, 132), cv::Scalar(  0,   0, 128)
};

const cv::Scalar kColormapMagma[] =
{
  cv::Scalar(  4,   0,   0), cv::Scalar(  5,   0,   1), cv::Scalar(  6,   1,   1), cv::Scalar(  8,   1,   1), cv::Scalar(  9,   1,   2), cv::Scalar( 11,   2,   2), cv::Scalar( 13,   2,   2), cv::Scalar( 15,   3,   3),
  cv::Scalar( 18,   3,   3), cv::Scalar( 20,   4,   4), cv::Scalar( 22,   4,   5), cv::Scalar( 24,   5,   6), cv::Scalar( 26,   5,   6), cv::Scalar( 28,   6,   7), cv::Scalar( 30,   7,   8), cv::Scalar( 32,   7,   9),
  cv::Scalar( 34,   8,  10), cv::Scalar( 36,   9,  11), cv::Scalar( 38,   9,  12), cv::Scalar( 41,  10,  13), cv::Scalar( 43,  11,  14), cv::Scalar( 45,  11,  16), cv::Scalar( 47,  12,  17), cv::Scalar( 49,  13,  18),
  cv::Scalar( 52,  13,  19), cv::Scalar( 54,  14,  20), cv::Scalar( 56,  14,  21), cv::Scalar( 59,  15,  22), cv::Scalar( 61,  15,  24), cv::Scalar( 63,  16,  25), cv::Scalar( 66,  16,  26), cv::Scalar( 68,  16,  28),
  cv::Scalar( 71,  17,  29), cv::Scalar( 73,  17,  30), cv::Scalar( 75,  17,  32), cv::Scalar( 78,  17,  33), cv::Scalar( 80,  17,  34), cv::Scalar( 83,  18,  36), cv::Scalar( 85,  18,  37), cv::Scalar( 88,  18,  39),
  cv::Scalar( 90,  17,  41), cv::Scalar( 92,  17,  42), cv::Scalar( 95,  17,  44), cv::Scalar( 97,  17,  45), cv::Scalar( 99,  17,  47), cv::Scalar(101,  17,  49), cv::Scalar(103,  16,  51), cv::Scalar(105,  16,  52),
  cv::Scalar(107,  16,  54), cv::Scalar(108,  16,  56), cv::Scalar(110,  15,  57), cv::Scalar(112,  15,  59), cv::Scalar(113,  15,  61), cv::Scalar(114,  15,  63), cv::Scalar(116,  15,  64), cv::Scalar(117,  15,  66),
  cv::Scalar(118,  15,  68), cv::Scalar(119,  16,  69), cv::Scalar(120,  16,  71), cv::Scalar(120,  16,  73), cv::Scalar(121,  16,  74), cv::Scalar(122,  17,  76), cv::Scalar(123,  17,  78), cv::Scalar(123,  18,  79),
  cv::Scalar(124,  18,  81), cv::Scalar(124,  19,  82), cv::Scalar(125,  19,  84), cv::Scalar(125,  20,  86), cv::Scalar(126,  21,  87), cv::Scalar(126,  21,  89), cv::Scalar(126,  22,  90), cv::Scalar(127,  22,  92),
  cv::Scalar(127,  23,  93), cv::Scalar(127,  24,  95), cv::Scalar(128,  24,  96), cv::Scalar(128,  25,  98), cv::Scalar(128,  26, 100), cv::Scalar(128,  26, 101), cv::Scalar(128,  27, 103), cv::Scalar(129,  28, 104),
  cv::Scalar(129,  28, 106), cv::Scalar(129,  29, 107), cv::Scalar(129,  29, 109), cv::Scalar(129,  30, 110), cv::Scalar(129,  31, 112), cv::Scalar(129,  31, 114), cv::Scalar(129,  32, 115), cv::Scalar(129,  33, 117),
  cv::Scalar(129,  33, 118), cv::Scalar(129,  34, 120), cv::Scalar(130,  34, 121), cv::Scalar(130,  35, 123), cv::Scalar(130,  35, 124), cv::Scalar(130,  36, 126), cv::Scalar(130,  37, 128), cv::Scalar(129,  37, 129),
  cv::Scalar(129,  38, 131), cv::Scalar(129,  38, 132), cv::Scalar(129,  39, 134), cv::Scalar(129,  39, 136), cv::Scalar(129,  40, 137), cv::Scalar(129,  41, 139), cv::Scalar(129,  41, 140), cv::Scalar(129,  42, 142),
  cv::Scalar(129,  42, 144), cv::Scalar(129,  43, 145), cv::Scalar(128,  43, 147), cv::Scalar(128,  44, 148), cv::Scalar(128,  44, 150), cv::Scalar(128,  45, 152), cv::Scalar(128,  45, 153), cv::Scalar(127,  46, 155),
  cv::Scalar(127,  46, 156), cv::Scalar(127,  47, 158), cv::Scalar(127,  47, 160), cv::Scalar(126,  48, 161), cv::Scalar(126,  48, 163), cv::Scalar(126,  49, 165), cv::Scalar(125,  49, 166), cv::Scalar(125,  50, 168),
  cv::Scalar(125,  51, 170), cv::Scalar(124,  51, 171), cv::Scalar(124,  52, 173), cv::Scalar(123,  52, 174), cv::Scalar(123,  53, 176), cv::Scalar(123,  53, 178), cv::Scalar(122,  54, 179), cv::Scalar(122,  54, 181),
  cv::Scalar(121,  55, 183), cv::Scalar(121,  55, 184), cv::Scalar(120,  56, 186), cv::Scalar(120,  57, 188), cv::Scalar(119,  57, 189), cv::Scalar(119,  58, 191), cv::Scalar(118,  58, 192), cv::Scalar(117,  59, 194),
  cv::Scalar(117,  60, 196), cv::Scalar(116,  60, 197), cv::Scalar(115,  61, 199), cv::Scalar(115,  62, 200), cv::Scalar(114,  62, 202), cv::Scalar(113,  63, 204), cv::Scalar(113,  64, 205), cv::Scalar(112,  64, 207),
  cv::Scalar(111,  65, 208), cv::Scalar(111,  66, 210), cv::Scalar(110,  67, 211), cv::Scalar(109,  68, 213), cv::Scalar(108,  69, 214), cv::Scalar(108,  69, 216), cv::Scalar(107,  70, 217), cv::Scalar(106,  71, 219),
  cv::Scalar(105,  72, 220), cv::Scalar(104,  73, 222), cv::Scalar(104,  74, 223), cv::Scalar(103,  76, 224), cv::Scalar(102,  77, 226), cv::Scalar(101,  78, 227), cv::Scalar(100,  79, 228), cv::Scalar(100,  80, 229),
  cv::Scalar( 99,  82, 231), cv::Scalar( 98,  83, 232), cv::Scalar( 98,  84, 233), cv::Scalar( 97,  86, 234), cv::Scalar( 96,  87, 235), cv::Scalar( 96,  88, 236), cv::Scalar( 95,  90, 237), cv::Scalar( 94,  91, 238),
  cv::Scalar( 94,  93, 239), cv::Scalar( 94,  95, 240), cv::Scalar( 93,  96, 241), cv::Scalar( 93,  98, 242), cv::Scalar( 92, 100, 242), cv::Scalar( 92, 101, 243), cv::Scalar( 92, 103, 244), cv::Scalar( 92, 105, 244),
  cv::Scalar( 92, 107, 245), cv::Scalar( 92, 108, 246), cv::Scalar( 92, 110, 246), cv::Scalar( 92, 112, 247), cv::Scalar( 92, 114, 247), cv::Scalar( 92, 116, 248), cv::Scalar( 92, 118, 248), cv::Scalar( 93, 120, 249),
  cv::Scalar( 93, 121, 249), cv::Scalar( 93, 123, 249), cv::Scalar( 94, 125, 250), cv::Scalar( 94, 127, 250), cv::Scalar( 95, 129, 250), cv::Scalar( 95, 131, 251), cv::Scalar( 96, 133, 251), cv::Scalar( 97, 135, 251),
  cv::Scalar( 97, 137, 252), cv::Scalar( 98, 138, 252), cv::Scalar( 99, 140, 252), cv::Scalar(100, 142, 252), cv::Scalar(101, 144, 252), cv::Scalar(102, 146, 253), cv::Scalar(103, 148, 253), cv::Scalar(104, 150, 253),
  cv::Scalar(105, 152, 253), cv::Scalar(106, 154, 253), cv::Scalar(107, 155, 253), cv::Scalar(108, 157, 254), cv::Scalar(109, 159, 254), cv::Scalar(110, 161, 254), cv::Scalar(111, 163, 254), cv::Scalar(113, 165, 254),
  cv::Scalar(114, 167, 254), cv::Scalar(115, 169, 254), cv::Scalar(116, 170, 254), cv::Scalar(118, 172, 254), cv::Scalar(119, 174, 254), cv::Scalar(120, 176, 254), cv::Scalar(122, 178, 254), cv::Scalar(123, 180, 254),
  cv::Scalar(124, 182, 254), cv::Scalar(126, 183, 254), cv::Scalar(127, 185, 254), cv::Scalar(129, 187, 254), cv::Scalar(130, 189, 254), cv::Scalar(132, 191, 254), cv::Scalar(133, 193, 254), cv::Scalar(135, 194, 254),
  cv::Scalar(136, 196, 254), cv::Scalar(138, 198, 254), cv::Scalar(140, 200, 254), cv::Scalar(141, 202, 254), cv::Scalar(143, 204, 254), cv::Scalar(144, 205, 254), cv::Scalar(146, 207, 254), cv::Scalar(148, 209, 254),
  cv::Scalar(149, 211, 254), cv::Scalar(151, 213, 254), cv::Scalar(153, 215, 254), cv::Scalar(154, 216, 254), cv::Scalar(156, 218, 253), cv::Scalar(158, 220, 253), cv::Scalar(160, 222, 253), cv::Scalar(161, 224, 253),
  cv::Scalar(163, 226, 253), cv::Scalar(165, 227, 253), cv::Scalar(167, 229, 253), cv::Scalar(169, 231, 253), cv::Scalar(170, 233, 253), cv::Scalar(172, 235, 253), cv::Scalar(174, 236, 252), cv::Scalar(176, 238, 252),
  cv::Scalar(178, 240, 252), cv::Scalar(180, 242, 252), cv::Scalar(182, 244, 252), cv::Scalar(184, 246, 252), cv::Scalar(185, 247, 252), cv::Scalar(187, 249, 252), cv::Scalar(189, 251, 252), cv::Scalar(191, 253, 252)
};

const cv::Scalar kColormapParula[] =
{
  cv::Scalar(168,  38,  62), cv::Scalar(172,  39,  62), cv::Scalar(175,  40,  63), cv::Scalar(178,  41,  63), cv::Scalar(180,  42,  64), cv::Scalar(183,  43,  64), cv::Scalar(186,  44,  65), cv::Scalar(189,  45,  65),
  cv::Scalar(191,  46,  66), cv::Scalar(194,  47,  66), cv::Scalar(197,  48,  67), cv::Scalar(200,  49,  67), cv::Scalar(202,  50,  67), cv::Scalar(205,  51,  68), cv::Scalar(208,  52,  68), cv::Scalar(210,  53,  69),
  cv::Scalar(213,  55,  69), cv::Scalar(215,  56,  69), cv::Scalar(217,  57,  70), cv::Scalar(220,  58,  70), cv::Scalar(222,  59,  70), cv::Scalar(224,  61,  70), cv::Scalar(225,  62,  71), cv::Scalar(227,  63,  71),
  cv::Scalar(229,  65,  71), cv::Scalar(230,  66,  71), cv::Scalar(232,  68,  71), cv::Scalar(233,  69,  71), cv::Scalar(235,  70,  71), cv::Scalar(236,  72,  72), cv::Scalar(237,  73,  72), cv::Scalar(238,  75,  72),
  cv::Scalar(240,  76,  72), cv::Scalar(241,  78,  72), cv::Scalar(242,  79,  72), cv::Scalar(243,  80,  72), cv::Scalar(244,  82,  72), cv::Scalar(245,  83,  72), cv::Scalar(246,  84,  72), cv::Scalar(247,  86,  71),
  cv::Scalar(247,  87,  71), cv::Scalar(248,  89,  71), cv::Scalar(249,  90,  71), cv::Scalar(250,  91,  71), cv::Scalar(250,  93,  71), cv::Scalar(251,  94,  70), cv::Scalar(251,  96,  70), cv::Scalar(252,  97,  70),
  cv::Scalar(252,  98,  69), cv::Scalar(253, 100,  69), cv::Scalar(253, 101,  68), cv::Scalar(253, 103,  67), cv::Scalar(254, 104,  67), cv::Scalar(254, 106,  66), cv::Scalar(254, 107,  65), cv::Scalar(254, 109,  64),
  cv::Scalar(255, 110,  63), cv::Scalar(255, 112,  62), cv::Scalar(255, 113,  60), cv::Scalar(255, 115,  59), cv::Scalar(255, 116,  57), cv::Scalar(254, 118,  56), cv::Scalar(254, 119,  54), cv::Scalar(253, 121,  53),
  cv::Scalar(253, 122,  51), cv::Scalar(252, 124,  50), cv::Scalar(252, 125,  49), cv::Scalar(251, 127,  48), cv::Scalar(250, 128,  47), cv::Scalar(250, 130,  47), cv::Scalar(249, 131,  46), cv::Scalar(248, 132,  46),
  cv::Scalar(248, 134,  46), cv::Scalar(247, 135,  46), cv::Scalar(246, 136,  45), cv::Scalar(245, 138,  45), cv::Scalar(244, 139,  45), cv::Scalar(243, 140,  45), cv::Scalar(242, 142,  45), cv::Scalar(241, 143,  44),
  cv::Scalar(240, 144,  44), cv::Scalar(239, 145,  43), cv::Scalar(238, 147,  42), cv::Scalar(237, 148,  41), cv::Scalar(236, 149,  40), cv::Scalar(235, 151,  39), cv::Scalar(234, 152,  39), cv::Scalar(233, 153,  38),
  cv::Scalar(232, 154,  38), cv::Scalar(232, 155,  37), cv::Scalar(231, 156,  37), cv::Scalar(230, 158,  36), cv::Scalar(229, 159,  36), cv::Scalar(229, 160,  35), cv::Scalar(228, 161,  35), cv::Scalar(228, 162,  34),
  cv::Scalar(227, 163,  33), cv::Scalar(227, 165,  32), cv::Scalar(226, 166,  31), cv::Scalar(225, 167,  30), cv::Scalar(225, 168,  29), cv::Scalar(224, 169,  29), cv::Scalar(223, 170,  28), cv::Scalar(222, 171,  27),
  cv::Scalar(221, 172,  26), cv::Scalar(220, 173,  25), cv::Scalar(218, 174,  23), cv::Scalar(217, 175,  22), cv::Scalar(216, 176,  20), cv::Scalar(214, 177,  18), cv::Scalar(213, 178,  16), cv::Scalar(212, 179,  14),
  cv::Scalar(210, 179,  11), cv::Scalar(209, 180,   8), cv::Scalar(207, 181,   6), cv::Scalar(206, 182,   4), cv::Scalar(204, 183,   2), cv::Scalar(202, 183,   1), cv::Scalar(201, 184,   0), cv::Scalar(199, 185,   0),
  cv::Scalar(198, 186,   0), cv::Scalar(196, 186,   1), cv::Scalar(194, 187,   2), cv::Scalar(193, 187,   4), cv::Scalar(191, 188,   6), cv::Scalar(189, 189,   9), cv::Scalar(188, 189,  13), cv::Scalar(186, 190,  16),
  cv::Scalar(184, 190,  20), cv::Scalar(182, 191,  23), cv::Scalar(181, 192,  26), cv::Scalar(179, 192,  29), cv::Scalar(177, 193,  32), cv::Scalar(175, 193,  35), cv::Scalar(174, 194,  37), cv::Scalar(172, 194,  39),
  cv::Scalar(170, 195,  41), cv::Scalar(168, 195,  43), cv::Scalar(166, 196,  44), cv::Scalar(165, 196,  46), cv::Scalar(163, 197,  47), cv::Scalar(161, 197,  49), cv::Scalar(159, 198,  50), cv::Scalar(157, 199,  51),
  cv::Scalar(155, 199,  53), cv::Scalar(153, 200,  54), cv::Scalar(150, 200,  56), cv::Scalar(148, 201,  57), cv::Scalar(146, 201,  59), cv::Scalar(144, 202,  61), cv::Scalar(141, 202,  64), cv::Scalar(139, 202,  66),
  cv::Scalar(137, 203,  69), cv::Scalar(134, 203,  72), cv::Scalar(132, 203,  75), cv::Scalar(129, 204,  78), cv::Scalar(127, 204,  81), cv::Scalar(124, 204,  84), cv::Scalar(122, 204,  87), cv::Scalar(119, 204,  90),
  cv::Scalar(116, 205,  94), cv::Scalar(114, 205,  97), cv::Scalar(111, 205, 100), cv::Scalar(108, 205, 103), cv::Scalar(105, 205, 107), cv::Scalar(102, 205, 110), cv::Scalar(100, 205, 114), cv::Scalar( 97, 204, 118),
  cv::Scalar( 94, 204, 121), cv::Scalar( 91, 204, 125), cv::Scalar( 89, 204, 129), cv::Scalar( 86, 204, 132), cv::Scalar( 83, 203, 136), cv::Scalar( 81, 203, 139), cv::Scalar( 78, 203, 143), cv::Scalar( 75, 202, 147),
  cv::Scalar( 72, 202, 150), cv::Scalar( 70, 201, 154), cv::Scalar( 67, 201, 157), cv::Scalar( 64, 200, 161), cv::Scalar( 62, 200, 164), cv::Scalar( 59, 199, 167), cv::Scalar( 57, 199, 171), cv::Scalar( 55, 198, 174),
  cv::Scalar( 53, 198, 178), cv::Scalar( 51, 197, 181), cv::Scalar( 49, 196, 184), cv::Scalar( 47, 196, 187), cv::Scalar( 45, 195, 190), cv::Scalar( 44, 195, 194), cv::Scalar( 42, 194, 197), cv::Scalar( 41, 193, 200),
  cv::Scalar( 40, 193, 203), cv::Scalar( 39, 192, 206), cv::Scalar( 39, 191, 208), cv::Scalar( 39, 191, 211), cv::Scalar( 39, 190, 214), cv::Scalar( 40, 190, 217), cv::Scalar( 40, 189, 219), cv::Scalar( 41, 188, 222),
  cv::Scalar( 42, 188, 225), cv::Scalar( 43, 188, 227), cv::Scalar( 45, 187, 230), cv::Scalar( 46, 187, 232), cv::Scalar( 48, 186, 234), cv::Scalar( 50, 186, 236), cv::Scalar( 53, 186, 239), cv::Scalar( 55, 186, 241),
  cv::Scalar( 57, 186, 243), cv::Scalar( 59, 186, 245), cv::Scalar( 61, 186, 247), cv::Scalar( 62, 186, 249), cv::Scalar( 62, 187, 251), cv::Scalar( 62, 188, 252), cv::Scalar( 61, 189, 254), cv::Scalar( 60, 190, 254),
  cv::Scalar( 59, 192, 254), cv::Scalar( 58, 193, 254), cv::Scalar( 57, 194, 254), cv::Scalar( 56, 196, 254), cv::Scalar( 55, 197, 254), cv::Scalar( 53, 199, 254), cv::Scalar( 52, 200, 254), cv::Scalar( 51, 202, 254),
  cv::Scalar( 50, 203, 253), cv::Scalar( 49, 205, 253), cv::Scalar( 49, 206, 253), cv::Scalar( 48, 208, 252), cv::Scalar( 47, 210, 251), cv::Scalar( 46, 211, 251), cv::Scalar( 46, 213, 250), cv::Scalar( 45, 214, 249),
  cv::Scalar( 44, 216, 249), cv::Scalar( 43, 217, 248), cv::Scalar( 42, 219, 247), cv::Scalar( 42, 221, 247), cv::Scalar( 41, 222, 246), cv::Scalar( 40, 224, 246), cv::Scalar( 40, 225, 245), cv::Scalar( 39, 227, 245),
  cv::Scalar( 38, 229, 245), cv::Scalar( 38, 230, 245), cv::Scalar( 37, 232, 245), cv::Scalar( 36, 233, 245), cv::Scalar( 35, 235, 245), cv::Scalar( 34, 236, 245), cv::Scalar( 33, 238, 245), cv::Scalar( 32, 239, 246),
  cv::Scalar( 31, 241, 246), cv::Scalar( 30, 242, 246), cv::Scalar( 28, 244, 247), cv::Scalar( 27, 245, 247), cv::Scalar( 26, 247, 248), cv::Scalar( 24, 248, 248), cv::Scalar( 22, 249, 249), cv::Scalar( 21, 251, 249)
};

const cv::Scalar kColormapPastel[] =
{
  cv::Scalar(  0,   0,   0), cv::Scalar(  0,   0,   3), cv::Scalar(  0,   0,   7), cv::Scalar(  0,   0,  10), cv::Scalar(  1,   0,  13), cv::Scalar(  1,   0,  16), cv::Scalar(  1,   0,  20), cv::Scalar(  1,   0,  23),
  cv::Scalar(  1,   0,  26), cv::Scalar(  1,   0,  30), cv::Scalar(  1,   0,  33), cv::Scalar(  1,   0,  36), cv::Scalar(  2,   0,  40), cv::Scalar(  2,   0,  43), cv::Scalar(  2,   0,  46), cv::Scalar(  2,   0,  49),
  cv::Scalar(  2,   0,  53), cv::Scalar(  2,   0,  56), cv::Scalar(  2,   0,  59), cv::Scalar(  2,   0,  63), cv::Scalar(  3,   0,  66), cv::Scalar(  3,   0,  69), cv::Scalar(  3,   0,  73), cv::Scalar(  3,   0,  76),
  cv::Scalar(  3,   0,  79), cv::Scalar(  3,   0,  82), cv::Scalar(  3,   0,  86), cv::Scalar(  3,   0,  89), cv::Scalar(  4,   0,  92), cv::Scalar(  4,   0,  96), cv::Scalar(  4,   0,  99), cv::Scalar(  4,   0, 102),
  cv::Scalar(  4,   0, 105), cv::Scalar(  4,   0, 109), cv::Scalar(  4,   0, 112), cv::Scalar(  4,   0, 115), cv::Scalar(  5,   0, 119), cv::Scalar(  7,   1, 118), cv::Scalar( 12,   4, 115), cv::Scalar( 16,   6, 112),
  cv::Scalar( 21,   9, 108), cv::Scalar( 26,  11, 105), cv::Scalar( 30,  14, 102), cv::Scalar( 35,  16,  98), cv::Scalar( 39,  19,  95), cv::Scalar( 44,  21,  92), cv::Scalar( 49,  24,  89), cv::Scalar( 53,  26,  85),
  cv::Scalar( 58,  29,  82), cv::Scalar( 62,  31,  79), cv::Scalar( 67,  34,  75), cv::Scalar( 72,  36,  72), cv::Scalar( 76,  39,  69), cv::Scalar( 81,  41,  65), cv::Scalar( 85,  44,  62), cv::Scalar( 90,  46,  59),
  cv::Scalar( 95,  49,  56), cv::Scalar( 99,  51,  52), cv::Scalar(104,  54,  49), cv::Scalar(108,  56,  46), cv::Scalar(113,  59,  42), cv::Scalar(118,  61,  39), cv::Scalar(122,  64,  36), cv::Scalar(127,  66,  32),
  cv::Scalar(131,  69,  29), cv::Scalar(136,  71,  26), cv::Scalar(141,  74,  23), cv::Scalar(145,  76,  19), cv::Scalar(150,  79,  16), cv::Scalar(154,  81,  13), cv::Scalar(159,  84,   9), cv::Scalar(164,  86,   6),
  cv::Scalar(168,  89,   3), cv::Scalar(172,  90,   1), cv::Scalar(173,  89,   7), cv::Scalar(175,  87,  13), cv::Scalar(176,  86,  19), cv::Scalar(177,  84,  24), cv::Scalar(178,  83,  30), cv::Scalar(180,  81,  36),
  cv::Scalar(181,  80,  42), cv::Scalar(182,  78,  48), cv::Scalar(183,  77,  54), cv::Scalar(185,  75,  60), cv::Scalar(186,  74,  66), cv::Scalar(187,  72,  72), cv::Scalar(188,  70,  77), cv::Scalar(190,  69,  83),
  cv::Scalar(191,  67,  89), cv::Scalar(192,  66,  95), cv::Scalar(193,  64, 101), cv::Scalar(195,  63, 107), cv::Scalar(196,  61, 113), cv::Scalar(197,  60, 119), cv::Scalar(198,  58, 125), cv::Scalar(200,  57, 131),
  cv::Scalar(201,  55, 136), cv::Scalar(202,  54, 142), cv::Scalar(203,  52, 148), cv::Scalar(205,  50, 154), cv::Scalar(206,  49, 160), cv::Scalar(207,  47, 166), cv::Scalar(208,  46, 172), cv::Scalar(210,  44, 178),
  cv::Scalar(211,  43, 184), cv::Scalar(212,  41, 189), cv::Scalar(213,  40, 195), cv::Scalar(215,  38, 201), cv::Scalar(216,  37, 207), cv::Scalar(217,  35, 213), cv::Scalar(215,  37, 213), cv::Scalar(211,  41, 210),
  cv::Scalar(207,  45, 208), cv::Scalar(203,  49, 205), cv::Scalar(199,  52, 202), cv::Scalar(196,  56, 200), cv::Scalar(192,  60, 197), cv::Scalar(188,  64, 195), cv::Scalar(184,  67, 192), cv::Scalar(180,  71, 189),
  cv::Scalar(176,  75, 187), cv::Scalar(173,  79, 184), cv::Scalar(169,  83, 182), cv::Scalar(165,  86, 179), cv::Scalar(161,  90, 176), cv::Scalar(157,  94, 174), cv::Scalar(153,  98, 171), cv::Scalar(150, 102, 168),
  cv::Scalar(146, 105, 166), cv::Scalar(142, 109, 163), cv::Scalar(138, 113, 161), cv::Scalar(134, 117, 158), cv::Scalar(130, 120, 155), cv::Scalar(127, 124, 153), cv::Scalar(123, 128, 150), cv::Scalar(119, 132, 148),
  cv::Scalar(115, 136, 145), cv::Scalar(111, 139, 142), cv::Scalar(107, 143, 140), cv::Scalar(104, 147, 137), cv::Scalar(100, 151, 134), cv::Scalar( 96, 154, 132), cv::Scalar( 92, 158, 129), cv::Scalar( 88, 162, 127),
  cv::Scalar( 85, 166, 124), cv::Scalar( 81, 170, 121), cv::Scalar( 78, 172, 121), cv::Scalar( 76, 172, 124), cv::Scalar( 75, 172, 128), cv::Scalar( 73, 173, 132), cv::Scalar( 72, 173, 135), cv::Scalar( 70, 173, 139),
  cv::Scalar( 69, 173, 143), cv::Scalar( 67, 173, 147), cv::Scalar( 66, 173, 150), cv::Scalar( 64, 173, 154), cv::Scalar( 63, 173, 158), cv::Scalar( 61, 173, 162), cv::Scalar( 60, 173, 165), cv::Scalar( 58, 174, 169),
  cv::Scalar( 57, 174, 173), cv::Scalar( 55, 174, 176), cv::Scalar( 54, 174, 180), cv::Scalar( 52, 174, 184), cv::Scalar( 51, 174, 188), cv::Scalar( 49, 174, 191), cv::Scalar( 48, 174, 195), cv::Scalar( 46, 174, 199),
  cv::Scalar( 45, 174, 202), cv::Scalar( 43, 174, 206), cv::Scalar( 42, 175, 210), cv::Scalar( 40, 175, 214), cv::Scalar( 39, 175, 217), cv::Scalar( 37, 175, 221), cv::Scalar( 36, 175, 225), cv::Scalar( 34, 175, 228),
  cv::Scalar( 33, 175, 232), cv::Scalar( 31, 175, 236), cv::Scalar( 30, 175, 240), cv::Scalar( 28, 175, 243), cv::Scalar( 27, 176, 247), cv::Scalar( 26, 176, 251), cv::Scalar( 24, 176, 254), cv::Scalar( 23, 178, 254),
  cv::Scalar( 23, 180, 254), cv::Scalar( 22, 182, 253), cv::Scalar( 21, 184, 252), cv::Scalar( 21, 186, 252), cv::Scalar( 20, 189, 251), cv::Scalar( 19, 191, 250), cv::Scalar( 19, 193, 250), cv::Scalar( 18, 195, 249),
  cv::Scalar( 17, 197, 248), cv::Scalar( 17, 199, 248), cv::Scalar( 16, 202, 247), cv::Scalar( 15, 204, 246), cv::Scalar( 15, 206, 246), cv::Scalar( 14, 208, 245), cv::Scalar( 13, 210, 244), cv::Scalar( 13, 212, 244),
  cv::Scalar( 12, 215, 243), cv::Scalar( 11, 217, 242), cv::Scalar( 11, 219, 242), cv::Scalar( 10, 221, 241), cv::Scalar( 10, 223, 240), cv::Scalar(  9, 225, 240), cv::Scalar(  8, 228, 239), cv::Scalar(  8, 230, 238),
  cv::Scalar(  7, 232, 238), cv::Scalar(  6, 234, 237), cv::Scalar(  6, 236, 236), cv::Scalar(  5, 239, 236), cv::Scalar(  4, 241, 235), cv::Scalar(  4, 243, 234), cv::Scalar(  3, 245, 233), cv::Scalar(  2, 247, 233),
  cv::Scalar(  2, 249, 232), cv::Scalar(  1, 252, 231), cv::Scalar(  0, 254, 231), cv::Scalar(  3, 255, 231), cv::Scalar( 10, 255, 231), cv::Scalar( 17, 255, 232), cv::Scalar( 24, 255, 233), cv::Scalar( 31, 255, 233),
  cv::Scalar( 38, 255, 234), cv::Scalar( 45, 255, 235), cv::Scalar( 52, 255, 235), cv::Scalar( 59, 255, 236), cv::Scalar( 66, 255, 237), cv::Scalar( 73, 255, 237), cv::Scalar( 80, 255, 238), cv::Scalar( 87, 255, 239),
  cv::Scalar( 94, 255, 239), cv::Scalar(101, 255, 240), cv::Scalar(108, 255, 241), cv::Scalar(115, 255, 241), cv::Scalar(122, 255, 242), cv::Scalar(129, 255, 243), cv::Scalar(136, 255, 244), cv::Scalar(143, 255, 244),
  cv::Scalar(150, 255, 245), cv::Scalar(157, 255, 246), cv::Scalar(164, 255, 246), cv::Scalar(171, 255, 247), cv::Scalar(178, 255, 248), cv::Scalar(185, 255, 248), cv::Scalar(192, 255, 249), cv::Scalar(199, 255, 250),
  cv::Scalar(206, 255, 250), cv::Scalar(213, 255, 251), cv::Scalar(220, 255, 252), cv::Scalar(227, 255, 252), cv::Scalar(234, 255, 253), cv::Scalar(241, 255, 254), cv::Scalar(248, 255, 254), cv::Scalar(255, 255, 255)
};

const cv::Scalar kColormapPlasma[] =
{
  cv::Scalar(135,   8,  13), cv::Scalar(136,   7,  16), cv::Scalar(137,   7,  19), cv::Scalar(138,   7,  22), cv::Scalar(140,   6,  25), cv::Scalar(141,   6,  27), cv::Scalar(142,   6,  29), cv::Scalar(143,   6,  32),
  cv::Scalar(144,   6,  34), cv::Scalar(145,   6,  36), cv::Scalar(145,   5,  38), cv::Scalar(146,   5,  40), cv::Scalar(147,   5,  42), cv::Scalar(148,   5,  44), cv::Scalar(149,   5,  46), cv::Scalar(150,   5,  47),
  cv::Scalar(151,   5,  49), cv::Scalar(151,   5,  51), cv::Scalar(152,   4,  53), cv::Scalar(153,   4,  55), cv::Scalar(154,   4,  56), cv::Scalar(154,   4,  58), cv::Scalar(155,   4,  60), cv::Scalar(156,   4,  62),
  cv::Scalar(156,   4,  63), cv::Scalar(157,   4,  65), cv::Scalar(158,   3,  67), cv::Scalar(158,   3,  68), cv::Scalar(159,   3,  70), cv::Scalar(159,   3,  72), cv::Scalar(160,   3,  73), cv::Scalar(161,   3,  75),
  cv::Scalar(161,   2,  76), cv::Scalar(162,   2,  78), cv::Scalar(162,   2,  80), cv::Scalar(163,   2,  81), cv::Scalar(163,   2,  83), cv::Scalar(164,   2,  85), cv::Scalar(164,   1,  86), cv::Scalar(164,   1,  88),
  cv::Scalar(165,   1,  89), cv::Scalar(165,   1,  91), cv::Scalar(166,   1,  92), cv::Scalar(166,   1,  94), cv::Scalar(166,   1,  96), cv::Scalar(167,   0,  97), cv::Scalar(167,   0,  99), cv::Scalar(167,   0, 100),
  cv::Scalar(167,   0, 102), cv::Scalar(168,   0, 103), cv::Scalar(168,   0, 105), cv::Scalar(168,   0, 106), cv::Scalar(168,   0, 108), cv::Scalar(168,   0, 110), cv::Scalar(168,   0, 111), cv::Scalar(168,   0, 113),
  cv::Scalar(168,   1, 114), cv::Scalar(168,   1, 116), cv::Scalar(168,   1, 117), cv::Scalar(168,   1, 119), cv::Scalar(168,   1, 120), cv::Scalar(168,   2, 122), cv::Scalar(168,   2, 123), cv::Scalar(168,   3, 125),
  cv::Scalar(168,   3, 126), cv::Scalar(168,   4, 128), cv::Scalar(167,   4, 129), cv::Scalar(167,   5, 131), cv::Scalar(167,   5, 132), cv::Scalar(166,   6, 134), cv::Scalar(166,   7, 135), cv::Scalar(166,   8, 136),
  cv::Scalar(165,   9, 138), cv::Scalar(165,  10, 139), cv::Scalar(165,  11, 141), cv::Scalar(164,  12, 142), cv::Scalar(164,  13, 143), cv::Scalar(163,  14, 145), cv::Scalar(163,  15, 146), cv::Scalar(162,  16, 148),
  cv::Scalar(161,  17, 149), cv::Scalar(161,  19, 150), cv::Scalar(160,  20, 152), cv::Scalar(159,  21, 153), cv::Scalar(159,  22, 154), cv::Scalar(158,  23, 156), cv::Scalar(157,  24, 157), cv::Scalar(157,  25, 158),
  cv::Scalar(156,  26, 160), cv::Scalar(155,  27, 161), cv::Scalar(154,  29, 162), cv::Scalar(154,  30, 163), cv::Scalar(153,  31, 165), cv::Scalar(152,  32, 166), cv::Scalar(151,  33, 167), cv::Scalar(150,  34, 168),
  cv::Scalar(149,  35, 170), cv::Scalar(148,  36, 171), cv::Scalar(148,  38, 172), cv::Scalar(147,  39, 173), cv::Scalar(146,  40, 174), cv::Scalar(145,  41, 176), cv::Scalar(144,  42, 177), cv::Scalar(143,  43, 178),
  cv::Scalar(142,  44, 179), cv::Scalar(141,  46, 180), cv::Scalar(140,  47, 181), cv::Scalar(139,  48, 182), cv::Scalar(138,  49, 183), cv::Scalar(137,  50, 184), cv::Scalar(136,  51, 186), cv::Scalar(136,  52, 187),
  cv::Scalar(135,  53, 188), cv::Scalar(134,  55, 189), cv::Scalar(133,  56, 190), cv::Scalar(132,  57, 191), cv::Scalar(131,  58, 192), cv::Scalar(130,  59, 193), cv::Scalar(129,  60, 194), cv::Scalar(128,  61, 195),
  cv::Scalar(127,  62, 196), cv::Scalar(126,  64, 197), cv::Scalar(125,  65, 198), cv::Scalar(124,  66, 199), cv::Scalar(123,  67, 200), cv::Scalar(122,  68, 201), cv::Scalar(122,  69, 202), cv::Scalar(121,  70, 203),
  cv::Scalar(120,  71, 204), cv::Scalar(119,  73, 204), cv::Scalar(118,  74, 205), cv::Scalar(117,  75, 206), cv::Scalar(116,  76, 207), cv::Scalar(115,  77, 208), cv::Scalar(114,  78, 209), cv::Scalar(113,  79, 210),
  cv::Scalar(113,  81, 211), cv::Scalar(112,  82, 212), cv::Scalar(111,  83, 213), cv::Scalar(110,  84, 213), cv::Scalar(109,  85, 214), cv::Scalar(108,  86, 215), cv::Scalar(107,  87, 216), cv::Scalar(106,  88, 217),
  cv::Scalar(106,  90, 218), cv::Scalar(105,  91, 218), cv::Scalar(104,  92, 219), cv::Scalar(103,  93, 220), cv::Scalar(102,  94, 221), cv::Scalar(101,  95, 222), cv::Scalar(100,  97, 222), cv::Scalar( 99,  98, 223),
  cv::Scalar( 99,  99, 224), cv::Scalar( 98, 100, 225), cv::Scalar( 97, 101, 226), cv::Scalar( 96, 102, 226), cv::Scalar( 95, 104, 227), cv::Scalar( 94, 105, 228), cv::Scalar( 93, 106, 229), cv::Scalar( 93, 107, 229),
  cv::Scalar( 92, 108, 230), cv::Scalar( 91, 110, 231), cv::Scalar( 90, 111, 231), cv::Scalar( 89, 112, 232), cv::Scalar( 88, 113, 233), cv::Scalar( 87, 114, 233), cv::Scalar( 87, 116, 234), cv::Scalar( 86, 117, 235),
  cv::Scalar( 85, 118, 235), cv::Scalar( 84, 119, 236), cv::Scalar( 83, 121, 237), cv::Scalar( 82, 122, 237), cv::Scalar( 81, 123, 238), cv::Scalar( 81, 124, 239), cv::Scalar( 80, 126, 239), cv::Scalar( 79, 127, 240),
  cv::Scalar( 78, 128, 240), cv::Scalar( 77, 129, 241), cv::Scalar( 76, 131, 241), cv::Scalar( 75, 132, 242), cv::Scalar( 75, 133, 243), cv::Scalar( 74, 135, 243), cv::Scalar( 73, 136, 244), cv::Scalar( 72, 137, 244),
  cv::Scalar( 71, 139, 245), cv::Scalar( 70, 140, 245), cv::Scalar( 69, 141, 246), cv::Scalar( 68, 143, 246), cv::Scalar( 68, 144, 247), cv::Scalar( 67, 145, 247), cv::Scalar( 66, 147, 247), cv::Scalar( 65, 148, 248),
  cv::Scalar( 64, 149, 248), cv::Scalar( 63, 151, 249), cv::Scalar( 62, 152, 249), cv::Scalar( 62, 154, 249), cv::Scalar( 61, 155, 250), cv::Scalar( 60, 156, 250), cv::Scalar( 59, 158, 250), cv::Scalar( 58, 159, 251),
  cv::Scalar( 57, 161, 251), cv::Scalar( 56, 162, 251), cv::Scalar( 56, 163, 252), cv::Scalar( 55, 165, 252), cv::Scalar( 54, 166, 252), cv::Scalar( 53, 168, 252), cv::Scalar( 52, 169, 252), cv::Scalar( 51, 171, 253),
  cv::Scalar( 51, 172, 253), cv::Scalar( 50, 174, 253), cv::Scalar( 49, 175, 253), cv::Scalar( 48, 177, 253), cv::Scalar( 47, 178, 253), cv::Scalar( 47, 180, 253), cv::Scalar( 46, 181, 253), cv::Scalar( 45, 183, 254),
  cv::Scalar( 44, 184, 254), cv::Scalar( 44, 186, 254), cv::Scalar( 43, 187, 254), cv::Scalar( 42, 189, 254), cv::Scalar( 42, 190, 254), cv::Scalar( 41, 192, 254), cv::Scalar( 41, 194, 253), cv::Scalar( 40, 195, 253),
  cv::Scalar( 39, 197, 253), cv::Scalar( 39, 198, 253), cv::Scalar( 39, 200, 253), cv::Scalar( 38, 202, 253), cv::Scalar( 38, 203, 253), cv::Scalar( 37, 205, 252), cv::Scalar( 37, 206, 252), cv::Scalar( 37, 208, 252),
  cv::Scalar( 37, 210, 252), cv::Scalar( 36, 211, 251), cv::Scalar( 36, 213, 251), cv::Scalar( 36, 215, 251), cv::Scalar( 36, 216, 250), cv::Scalar( 36, 218, 250), cv::Scalar( 36, 220, 249), cv::Scalar( 37, 221, 249),
  cv::Scalar( 37, 223, 248), cv::Scalar( 37, 225, 248), cv::Scalar( 37, 226, 247), cv::Scalar( 37, 228, 247), cv::Scalar( 38, 230, 246), cv::Scalar( 38, 232, 246), cv::Scalar( 38, 233, 245), cv::Scalar( 39, 235, 245),
  cv::Scalar( 39, 237, 244), cv::Scalar( 39, 238, 243), cv::Scalar( 39, 240, 243), cv::Scalar( 39, 242, 242), cv::Scalar( 38, 244, 241), cv::Scalar( 37, 245, 241), cv::Scalar( 36, 247, 240), cv::Scalar( 33, 249, 240)
};

const cv::Scalar kColormapSepia[] =
{
  cv::Scalar(  0,   0,   0), cv::Scalar(  1,   1,   2), cv::Scalar(  2,   3,   4), cv::Scalar(  3,   4,   6), cv::Scalar(  4,   5,   7), cv::Scalar(  6,   7,   9), cv::Scalar(  7,   8,  11), cv::Scalar(  8,   9,  13),
  cv::Scalar(  9,  11,  14), cv::Scalar( 10,  12,  16), cv::Scalar( 11,  13,  17), cv::Scalar( 12,  14,  18), cv::Scalar( 13,  15,  19), cv::Scalar( 14,  16,  21), cv::Scalar( 15,  17,  22), cv::Scalar( 16,  18,  23),
  cv::Scalar( 16,  19,  24), cv::Scalar( 17,  20,  25), cv::Scalar( 18,  21,  26), cv::Scalar( 19,  21,  27), cv::Scalar( 19,  22,  27), cv::Scalar( 20,  23,  28), cv::Scalar( 21,  24,  29), cv::Scalar( 21,  24,  30),
  cv::Scalar( 22,  25,  31), cv::Scalar( 23,  26,  32), cv::Scalar( 23,  27,  33), cv::Scalar( 24,  27,  34), cv::Scalar( 24,  28,  35), cv::Scalar( 25,  29,  36), cv::Scalar( 26,  29,  37), cv::Scalar( 26,  30,  38),
  cv::Scalar( 27,  31,  39), cv::Scalar( 27,  32,  40), cv::Scalar( 28,  32,  41), cv::Scalar( 28,  33,  42), cv::Scalar( 29,  34,  43), cv::Scalar( 30,  35,  44), cv::Scalar( 30,  35,  45), cv::Scalar( 31,  36,  46),
  cv::Scalar( 31,  37,  47), cv::Scalar( 32,  38,  48), cv::Scalar( 33,  39,  49), cv::Scalar( 33,  39,  50), cv::Scalar( 34,  40,  51), cv::Scalar( 34,  41,  52), cv::Scalar( 35,  42,  53), cv::Scalar( 36,  42,  55),
  cv::Scalar( 36,  43,  56), cv::Scalar( 37,  44,  57), cv::Scalar( 37,  45,  58), cv::Scalar( 38,  46,  59), cv::Scalar( 39,  46,  60), cv::Scalar( 39,  47,  61), cv::Scalar( 40,  48,  62), cv::Scalar( 40,  49,  63),
  cv::Scalar( 41,  50,  64), cv::Scalar( 42,  51,  65), cv::Scalar( 42,  51,  66), cv::Scalar( 43,  52,  67), cv::Scalar( 43,  53,  68), cv::Scalar( 44,  54,  69), cv::Scalar( 45,  55,  70), cv::Scalar( 45,  55,  71),
  cv::Scalar( 46,  56,  72), cv::Scalar( 46,  57,  73), cv::Scalar( 47,  58,  75), cv::Scalar( 48,  59,  76), cv::Scalar( 48,  60,  77), cv::Scalar( 49,  61,  78), cv::Scalar( 50,  61,  79), cv::Scalar( 50,  62,  80),
  cv::Scalar( 51,  63,  81), cv::Scalar( 51,  64,  82), cv::Scalar( 52,  65,  83), cv::Scalar( 53,  66,  84), cv::Scalar( 53,  67,  85), cv::Scalar( 54,  67,  86), cv::Scalar( 54,  68,  87), cv::Scalar( 55,  69,  89),
  cv::Scalar( 56,  70,  90), cv::Scalar( 56,  71,  91), cv::Scalar( 57,  72,  92), cv::Scalar( 58,  73,  93), cv::Scalar( 58,  73,  94), cv::Scalar( 59,  74,  95), cv::Scalar( 59,  75,  96), cv::Scalar( 60,  76,  97),
  cv::Scalar( 61,  77,  98), cv::Scalar( 61,  78,  99), cv::Scalar( 62,  79, 101), cv::Scalar( 63,  80, 102), cv::Scalar( 63,  81, 103), cv::Scalar( 64,  81, 104), cv::Scalar( 64,  82, 105), cv::Scalar( 65,  83, 106),
  cv::Scalar( 66,  84, 107), cv::Scalar( 66,  85, 108), cv::Scalar( 67,  86, 109), cv::Scalar( 68,  87, 111), cv::Scalar( 68,  88, 112), cv::Scalar( 69,  89, 113), cv::Scalar( 69,  90, 114), cv::Scalar( 70,  90, 115),
  cv::Scalar( 71,  91, 116), cv::Scalar( 71,  92, 117), cv::Scalar( 72,  93, 118), cv::Scalar( 73,  94, 120), cv::Scalar( 73,  95, 121), cv::Scalar( 74,  96, 122), cv::Scalar( 75,  97, 123), cv::Scalar( 75,  98, 124),
  cv::Scalar( 76,  99, 125), cv::Scalar( 77, 100, 126), cv::Scalar( 77, 101, 127), cv::Scalar( 78, 102, 128), cv::Scalar( 78, 103, 129), cv::Scalar( 79, 104, 130), cv::Scalar( 80, 105, 131), cv::Scalar( 80, 106, 132),
  cv::Scalar( 81, 107, 133), cv::Scalar( 82, 108, 134), cv::Scalar( 82, 109, 135), cv::Scalar( 83, 110, 136), cv::Scalar( 84, 111, 137), cv::Scalar( 85, 112, 138), cv::Scalar( 86, 113, 139), cv::Scalar( 87, 114, 140),
  cv::Scalar( 88, 115, 140), cv::Scalar( 89, 116, 141), cv::Scalar( 90, 117, 142), cv::Scalar( 91, 118, 143), cv::Scalar( 92, 119, 143), cv::Scalar( 93, 120, 144), cv::Scalar( 94, 122, 145), cv::Scalar( 95, 123, 146),
  cv::Scalar( 96, 124, 147), cv::Scalar( 97, 125, 147), cv::Scalar( 98, 126, 148), cv::Scalar( 99, 127, 149), cv::Scalar(100, 128, 150), cv::Scalar(101, 129, 151), cv::Scalar(102, 130, 152), cv::Scalar(103, 131, 153),
  cv::Scalar(105, 132, 153), cv::Scalar(106, 133, 154), cv::Scalar(107, 134, 155), cv::Scalar(108, 135, 156), cv::Scalar(110, 136, 157), cv::Scalar(111, 137, 158), cv::Scalar(112, 138, 159), cv::Scalar(113, 140, 160),
  cv::Scalar(115, 141, 160), cv::Scalar(116, 142, 161), cv::Scalar(117, 143, 162), cv::Scalar(118, 144, 163), cv::Scalar(119, 145, 164), cv::Scalar(121, 146, 164), cv::Scalar(122, 147, 165), cv::Scalar(123, 148, 166),
  cv::Scalar(124, 149, 167), cv::Scalar(126, 150, 168), cv::Scalar(127, 152, 168), cv::Scalar(128, 153, 169), cv::Scalar(129, 154, 170), cv::Scalar(131, 155, 171), cv::Scalar(132, 156, 172), cv::Scalar(133, 157, 173),
  cv::Scalar(135, 158, 174), cv::Scalar(136, 159, 175), cv::Scalar(137, 160, 176), cv::Scalar(139, 161, 177), cv::Scalar(140, 162, 178), cv::Scalar(141, 163, 179), cv::Scalar(142, 164, 180), cv::Scalar(144, 165, 181),
  cv::Scalar(145, 166, 182), cv::Scalar(146, 167, 184), cv::Scalar(148, 168, 185), cv::Scalar(149, 169, 186), cv::Scalar(150, 170, 187), cv::Scalar(152, 171, 188), cv::Scalar(153, 172, 189), cv::Scalar(154, 173, 190),
  cv::Scalar(156, 174, 191), cv::Scalar(157, 175, 192), cv::Scalar(158, 176, 193), cv::Scalar(160, 177, 194), cv::Scalar(161, 179, 195), cv::Scalar(162, 180, 196), cv::Scalar(164, 181, 197), cv::Scalar(165, 182, 198),
  cv::Scalar(166, 183, 199), cv::Scalar(168, 184, 200), cv::Scalar(169, 185, 201), cv::Scalar(171, 186, 202), cv::Scalar(172, 187, 203), cv::Scalar(173, 188, 204), cv::Scalar(175, 189, 205), cv::Scalar(176, 190, 206),
  cv::Scalar(178, 192, 207), cv::Scalar(179, 193, 208), cv::Scalar(180, 194, 209), cv::Scalar(182, 195, 210), cv::Scalar(183, 196, 211), cv::Scalar(184, 197, 212), cv::Scalar(186, 198, 213), cv::Scalar(187, 199, 213),
  cv::Scalar(189, 200, 214), cv::Scalar(190, 202, 215), cv::Scalar(191, 203, 216), cv::Scalar(193, 204, 217), cv::Scalar(194, 205, 218), cv::Scalar(196, 206, 219), cv::Scalar(197, 207, 220), cv::Scalar(198, 208, 221),
  cv::Scalar(200, 209, 221), cv::Scalar(201, 211, 222), cv::Scalar(203, 212, 223), cv::Scalar(204, 213, 224), cv::Scalar(205, 214, 225), cv::Scalar(207, 215, 226), cv::Scalar(208, 216, 227), cv::Scalar(210, 217, 228),
  cv::Scalar(211, 219, 228), cv::Scalar(212, 220, 229), cv::Scalar(214, 221, 230), cv::Scalar(215, 222, 231), cv::Scalar(217, 223, 232), cv::Scalar(218, 224, 233), cv::Scalar(219, 226, 234), cv::Scalar(221, 227, 235),
  cv::Scalar(222, 228, 235), cv::Scalar(224, 229, 236), cv::Scalar(225, 230, 237), cv::Scalar(226, 231, 238), cv::Scalar(228, 233, 239), cv::Scalar(229, 234, 240), cv::Scalar(231, 235, 241), cv::Scalar(232, 236, 241),
  cv::Scalar(233, 237, 242), cv::Scalar(235, 238, 243), cv::Scalar(236, 240, 244), cv::Scalar(238, 241, 245), cv::Scalar(239, 242, 246), cv::Scalar(241, 243, 247), cv::Scalar(242, 244, 247), cv::Scalar(243, 245, 248),
  cv::Scalar(245, 247, 249), cv::Scalar(246, 248, 250), cv::Scalar(248, 249, 251), cv::Scalar(249, 250, 252), cv::Scalar(251, 251, 252), cv::Scalar(252, 253, 253), cv::Scalar(254, 254, 254), cv::Scalar(255, 255, 255)
};

const cv::Scalar kColormapTemperature[] =
{
  cv::Scalar(217,   0,  36), cv::Scalar(219,   2,  35), cv::Scalar(221,   4,  35), cv::Scalar(223,   6,  34), cv::Scalar(225,   8,  33), cv::Scalar(227,  10,  32), cv::Scalar(229,  11,  32), cv::Scalar(231,  13,  31),
  cv::Scalar(233,  15,  30), cv::Scalar(235,  17,  29), cv::Scalar(237,  19,  29), cv::Scalar(239,  21,  28), cv::Scalar(241,  23,  27), cv::Scalar(243,  25,  26), cv::Scalar(245,  27,  26), cv::Scalar(247,  29,  25),
  cv::Scalar(248,  32,  26), cv::Scalar(248,  36,  27), cv::Scalar(249,  40,  28), cv::Scalar(249,  44,  29), cv::Scalar(250,  48,  30), cv::Scalar(250,  52,  31), cv::Scalar(251,  56,  32), cv::Scalar(251,  60,  33),
  cv::Scalar(252,  64,  34), cv::Scalar(252,  68,  35), cv::Scalar(253,  72,  37), cv::Scalar(253,  75,  38), cv::Scalar(254,  79,  39), cv::Scalar(254,  83,  40), cv::Scalar(255,  87,  41), cv::Scalar(255,  90,  42),
  cv::Scalar(255,  94,  44), cv::Scalar(255,  97,  45), cv::Scalar(255, 100,  46), cv::Scalar(255, 103,  48), cv::Scalar(255, 106,  49), cv::Scalar(255, 110,  50), cv::Scalar(255, 113,  52), cv::Scalar(255, 116,  53),
  cv::Scalar(255, 119,  54), cv::Scalar(255, 123,  56), cv::Scalar(255, 126,  57), cv::Scalar(255, 129,  58), cv::Scalar(255, 132,  60), cv::Scalar(255, 135,  61), cv::Scalar(255, 138,  63), cv::Scalar(255, 141,  65),
  cv::Scalar(255, 144,  66), cv::Scalar(255, 146,  68), cv::Scalar(255, 149,  70), cv::Scalar(255, 152,  71), cv::Scalar(255, 155,  73), cv::Scalar(255, 157,  75), cv::Scalar(255, 160,  77), cv::Scalar(255, 163,  78),
  cv::Scalar(255, 166,  80), cv::Scalar(255, 168,  82), cv::Scalar(255, 171,  83), cv::Scalar(255, 174,  85), cv::Scalar(255, 176,  87), cv::Scalar(255, 179,  89), cv::Scalar(255, 181,  91), cv::Scalar(255, 183,  93),
  cv::Scalar(255, 186,  95), cv::Scalar(255, 188,  97), cv::Scalar(255, 190,  99), cv::Scalar(255, 193, 101), cv::Scalar(255, 195, 103), cv::Scalar(255, 197, 105), cv::Scalar(255, 200, 107), cv::Scalar(255, 202, 109),
  cv::Scalar(255, 204, 111), cv::Scalar(255, 207, 113), cv::Scalar(255, 209, 115), cv::Scalar(255, 211, 117), cv::Scalar(255, 213, 120), cv::Scalar(255, 214, 122), cv::Scalar(255, 216, 124), cv::Scalar(255, 218, 127),
  cv::Scalar(255, 219, 129), cv::Scalar(255, 221, 132), cv::Scalar(255, 222, 134), cv::Scalar(255, 224, 136), cv::Scalar(255, 225, 139), cv::Scalar(255, 227, 141), cv::Scalar(255, 228, 143), cv::Scalar(255, 230, 146),
  cv::Scalar(255, 232, 148), cv::Scalar(255, 233, 151), cv::Scalar(255, 235, 153), cv::Scalar(255, 236, 155), cv::Scalar(255, 237, 158), cv::Scalar(255, 238, 160), cv::Scalar(255, 239, 163), cv::Scalar(255, 240, 165),
  cv::Scalar(255, 241, 167), cv::Scalar(255, 242, 170), cv::Scalar(255, 242, 172), cv::Scalar(255, 243, 174), cv::Scalar(255, 244, 177), cv::Scalar(255, 245, 179), cv::Scalar(255, 246, 182), cv::Scalar(255, 247, 184),
  cv::Scalar(255, 248, 186), cv::Scalar(255, 249, 189), cv::Scalar(255, 250, 192), cv::Scalar(255, 250, 195), cv::Scalar(255, 251, 198), cv::Scalar(255, 251, 201), cv::Scalar(255, 251, 204), cv::Scalar(255, 252, 207),
  cv::Scalar(255, 252, 210), cv::Scalar(255, 252, 213), cv::Scalar(255, 253, 216), cv::Scalar(255, 253, 219), cv::Scalar(255, 254, 222), cv::Scalar(255, 254, 225), cv::Scalar(255, 254, 228), cv::Scalar(255, 255, 232),
  cv::Scalar(255, 255, 235), cv::Scalar(254, 255, 236), cv::Scalar(252, 255, 237), cv::Scalar(251, 255, 239), cv::Scalar(250, 255, 240), cv::Scalar(248, 255, 241), cv::Scalar(247, 255, 243), cv::Scalar(245, 255, 244),
  cv::Scalar(244, 255, 245), cv::Scalar(243, 255, 247), cv::Scalar(241, 255, 248), cv::Scalar(240, 255, 250), cv::Scalar(239, 255, 251), cv::Scalar(237, 255, 252), cv::Scalar(236, 255, 254), cv::Scalar(235, 255, 255),
  cv::Scalar(232, 254, 255), cv::Scalar(228, 253, 255), cv::Scalar(225, 252, 255), cv::Scalar(222, 251, 255), cv::Scalar(219, 251, 255), cv::Scalar(216, 250, 255), cv::Scalar(213, 249, 255), cv::Scalar(210, 248, 255),
  cv::Scalar(207, 247, 255), cv::Scalar(204, 246, 255), cv::Scalar(201, 245, 255), cv::Scalar(198, 244, 255), cv::Scalar(195, 244, 255), cv::Scalar(192, 243, 255), cv::Scalar(189, 242, 255), cv::Scalar(186, 240, 255),
  cv::Scalar(184, 238, 255), cv::Scalar(182, 236, 255), cv::Scalar(179, 234, 255), cv::Scalar(177, 233, 255), cv::Scalar(174, 231, 255), cv::Scalar(172, 229, 255), cv::Scalar(170, 227, 255), cv::Scalar(167, 225, 255),
  cv::Scalar(165, 223, 255), cv::Scalar(163, 222, 255), cv::Scalar(160, 220, 255), cv::Scalar(158, 218, 255), cv::Scalar(155, 216, 255), cv::Scalar(153, 214, 255), cv::Scalar(151, 211, 255), cv::Scalar(148, 209, 255),
  cv::Scalar(146, 206, 255), cv::Scalar(143, 203, 255), cv::Scalar(141, 200, 255), cv::Scalar(139, 197, 255), cv::Scalar(136, 195, 255), cv::Scalar(134, 192, 255), cv::Scalar(132, 189, 255), cv::Scalar(129, 186, 255),
  cv::Scalar(127, 184, 255), cv::Scalar(124, 181, 255), cv::Scalar(122, 178, 255), cv::Scalar(120, 175, 255), cv::Scalar(117, 172, 255), cv::Scalar(115, 169, 255), cv::Scalar(113, 165, 255), cv::Scalar(111, 162, 255),
  cv::Scalar(109, 159, 255), cv::Scalar(107, 155, 255), cv::Scalar(105, 152, 255), cv::Scalar(103, 148, 255), cv::Scalar(101, 145, 255), cv::Scalar( 99, 141, 255), cv::Scalar( 97, 138, 255), cv::Scalar( 95, 134, 255),
  cv::Scalar( 93, 131, 255), cv::Scalar( 91, 127, 255), cv::Scalar( 89, 124, 255), cv::Scalar( 87, 120, 255), cv::Scalar( 85, 116, 255), cv::Scalar( 83, 112, 255), cv::Scalar( 82, 109, 255), cv::Scalar( 80, 105, 255),
  cv::Scalar( 78, 101, 255), cv::Scalar( 77,  97, 255), cv::Scalar( 75,  93, 255), cv::Scalar( 73,  89, 255), cv::Scalar( 71,  85, 255), cv::Scalar( 70,  81, 255), cv::Scalar( 68,  77, 255), cv::Scalar( 66,  73, 255),
  cv::Scalar( 65,  69, 255), cv::Scalar( 63,  65, 255), cv::Scalar( 61,  61, 255), cv::Scalar( 61,  60, 254), cv::Scalar( 60,  58, 254), cv::Scalar( 60,  57, 253), cv::Scalar( 59,  55, 253), cv::Scalar( 59,  54, 252),
  cv::Scalar( 58,  53, 252), cv::Scalar( 58,  51, 251), cv::Scalar( 57,  50, 251), cv::Scalar( 57,  48, 250), cv::Scalar( 56,  47, 250), cv::Scalar( 56,  45, 249), cv::Scalar( 55,  44, 249), cv::Scalar( 55,  42, 248),
  cv::Scalar( 54,  41, 248), cv::Scalar( 54,  40, 247), cv::Scalar( 53,  38, 245), cv::Scalar( 53,  37, 243), cv::Scalar( 52,  36, 241), cv::Scalar( 52,  35, 239), cv::Scalar( 52,  34, 237), cv::Scalar( 51,  32, 235),
  cv::Scalar( 51,  31, 233), cv::Scalar( 50,  30, 231), cv::Scalar( 50,  29, 229), cv::Scalar( 50,  28, 227), cv::Scalar( 49,  26, 225), cv::Scalar( 49,  25, 223), cv::Scalar( 48,  24, 221), cv::Scalar( 48,  23, 219),
  cv::Scalar( 48,  22, 217), cv::Scalar( 47,  20, 213), cv::Scalar( 46,  19, 210), cv::Scalar( 45,  17, 207), cv::Scalar( 44,  16, 203), cv::Scalar( 43,  14, 200), cv::Scalar( 42,  13, 196), cv::Scalar( 41,  12, 193),
  cv::Scalar( 40,  10, 190), cv::Scalar( 39,   9, 186), cv::Scalar( 38,   7, 183), cv::Scalar( 37,   6, 179), cv::Scalar( 36,   4, 176), cv::Scalar( 35,   3, 173), cv::Scalar( 34,   1, 169), cv::Scalar( 33,   0, 166)
};

const cv::Scalar kColormapThermal[] =
{
  cv::Scalar(  0,   0,   0), cv::Scalar(  4,   1,   3), cv::Scalar(  8,   2,   5), cv::Scalar( 12,   2,   8), cv::Scalar( 15,   3,  10), cv::Scalar( 18,   4,  13), cv::Scalar( 21,   5,  15), cv::Scalar( 23,   6,  17),
  cv::Scalar( 25,   7,  18), cv::Scalar( 27,   7,  20), cv::Scalar( 29,   8,  21), cv::Scalar( 32,   9,  23), cv::Scalar( 34,  10,  24), cv::Scalar( 36,  11,  24), cv::Scalar( 38,  12,  25), cv::Scalar( 40,  13,  26),
  cv::Scalar( 43,  13,  27), cv::Scalar( 45,  14,  28), cv::Scalar( 47,  14,  29), cv::Scalar( 50,  15,  30), cv::Scalar( 52,  15,  31), cv::Scalar( 54,  15,  31), cv::Scalar( 57,  15,  32), cv::Scalar( 59,  16,  33),
  cv::Scalar( 62,  16,  34), cv::Scalar( 64,  16,  35), cv::Scalar( 66,  16,  36), cv::Scalar( 69,  16,  37), cv::Scalar( 71,  17,  38), cv::Scalar( 74,  17,  39), cv::Scalar( 76,  17,  39), cv::Scalar( 79,  17,  40),
  cv::Scalar( 82,  18,  41), cv::Scalar( 84,  18,  42), cv::Scalar( 87,  18,  42), cv::Scalar( 90,  18,  43), cv::Scalar( 92,  19,  43), cv::Scalar( 95,  19,  44), cv::Scalar( 98,  19,  45), cv::Scalar(100,  19,  45),
  cv::Scalar(103,  20,  46), cv::Scalar(106,  20,  46), cv::Scalar(109,  20,  47), cv::Scalar(111,  20,  47), cv::Scalar(114,  20,  48), cv::Scalar(117,  20,  49), cv::Scalar(119,  20,  50), cv::Scalar(122,  20,  51),
  cv::Scalar(125,  20,  52), cv::Scalar(127,  20,  53), cv::Scalar(129,  19,  55), cv::Scalar(131,  18,  58), cv::Scalar(133,  17,  61), cv::Scalar(135,  15,  65), cv::Scalar(136,  13,  68), cv::Scalar(137,  10,  72),
  cv::Scalar(138,   7,  75), cv::Scalar(139,   4,  79), cv::Scalar(139,   0,  83), cv::Scalar(139,   0,  87), cv::Scalar(138,   0,  91), cv::Scalar(138,   0,  95), cv::Scalar(137,   0,  99), cv::Scalar(136,   0, 102),
  cv::Scalar(135,   0, 105), cv::Scalar(134,   0, 109), cv::Scalar(133,   0, 112), cv::Scalar(132,   0, 115), cv::Scalar(131,   0, 117), cv::Scalar(130,   0, 120), cv::Scalar(129,   0, 122), cv::Scalar(128,   0, 125),
  cv::Scalar(127,   0, 127), cv::Scalar(126,   0, 130), cv::Scalar(124,   0, 132), cv::Scalar(123,   0, 134), cv::Scalar(122,   0, 137), cv::Scalar(121,   0, 139), cv::Scalar(120,   0, 141), cv::Scalar(119,   0, 143),
  cv::Scalar(118,   0, 145), cv::Scalar(116,   0, 147), cv::Scalar(115,   0, 149), cv::Scalar(114,   0, 151), cv::Scalar(113,   0, 153), cv::Scalar(112,   0, 155), cv::Scalar(111,   0, 157), cv::Scalar(109,   0, 159),
  cv::Scalar(108,   0, 161), cv::Scalar(107,   0, 163), cv::Scalar(106,   0, 165), cv::Scalar(105,   0, 167), cv::Scalar(103,   0, 168), cv::Scalar(102,   0, 170), cv::Scalar(101,   0, 172), cv::Scalar(100,   0, 174),
  cv::Scalar( 99,   0, 176), cv::Scalar( 97,   0, 178), cv::Scalar( 96,   0, 179), cv::Scalar( 95,   0, 181), cv::Scalar( 94,   0, 183), cv::Scalar( 93,   0, 185), cv::Scalar( 91,   1, 186), cv::Scalar( 90,   2, 188),
  cv::Scalar( 89,   4, 190), cv::Scalar( 88,   6, 192), cv::Scalar( 86,   8, 193), cv::Scalar( 85,  10, 195), cv::Scalar( 84,  12, 197), cv::Scalar( 83,  15, 198), cv::Scalar( 81,  16, 200), cv::Scalar( 80,  18, 202),
  cv::Scalar( 79,  20, 203), cv::Scalar( 77,  22, 205), cv::Scalar( 76,  23, 206), cv::Scalar( 75,  25, 208), cv::Scalar( 73,  26, 210), cv::Scalar( 72,  28, 211), cv::Scalar( 71,  29, 213), cv::Scalar( 69,  31, 214),
  cv::Scalar( 68,  32, 216), cv::Scalar( 67,  34, 218), cv::Scalar( 65,  35, 219), cv::Scalar( 64,  36, 221), cv::Scalar( 62,  38, 222), cv::Scalar( 61,  39, 224), cv::Scalar( 59,  41, 225), cv::Scalar( 58,  42, 227),
  cv::Scalar( 56,  44, 228), cv::Scalar( 55,  45, 229), cv::Scalar( 53,  47, 231), cv::Scalar( 52,  48, 232), cv::Scalar( 50,  50, 234), cv::Scalar( 48,  52, 235), cv::Scalar( 46,  53, 236), cv::Scalar( 45,  55, 237),
  cv::Scalar( 43,  57, 239), cv::Scalar( 41,  58, 240), cv::Scalar( 39,  60, 241), cv::Scalar( 37,  62, 242), cv::Scalar( 35,  64, 243), cv::Scalar( 32,  66, 244), cv::Scalar( 30,  68, 245), cv::Scalar( 27,  70, 246),
  cv::Scalar( 25,  72, 247), cv::Scalar( 22,  74, 247), cv::Scalar( 19,  76, 248), cv::Scalar( 15,  78, 249), cv::Scalar( 10,  80, 249), cv::Scalar(  7,  83, 249), cv::Scalar(  3,  86, 250), cv::Scalar(  0,  88, 250),
  cv::Scalar(  0,  91, 250), cv::Scalar(  0,  93, 249), cv::Scalar(  0,  96, 249), cv::Scalar(  0,  99, 249), cv::Scalar(  0, 101, 249), cv::Scalar(  0, 104, 249), cv::Scalar(  0, 106, 249), cv::Scalar(  0, 109, 248),
  cv::Scalar(  0, 111, 248), cv::Scalar(  0, 114, 248), cv::Scalar(  0, 116, 247), cv::Scalar(  0, 118, 247), cv::Scalar(  0, 121, 246), cv::Scalar(  0, 123, 246), cv::Scalar(  0, 125, 245), cv::Scalar(  0, 128, 245),
  cv::Scalar(  0, 130, 244), cv::Scalar(  0, 132, 244), cv::Scalar(  0, 134, 243), cv::Scalar(  0, 137, 243), cv::Scalar(  0, 139, 242), cv::Scalar(  0, 141, 241), cv::Scalar(  0, 143, 241), cv::Scalar(  0, 145, 240),
  cv::Scalar(  0, 148, 239), cv::Scalar(  0, 150, 239), cv::Scalar(  0, 152, 238), cv::Scalar(  0, 154, 237), cv::Scalar(  0, 156, 236), cv::Scalar(  0, 158, 236), cv::Scalar(  0, 160, 235), cv::Scalar(  0, 162, 234),
  cv::Scalar(  0, 164, 233), cv::Scalar(  0, 166, 233), cv::Scalar(  0, 168, 232), cv::Scalar(  0, 170, 231), cv::Scalar(  0, 172, 230), cv::Scalar(  4, 174, 229), cv::Scalar( 11, 176, 228), cv::Scalar( 18, 178, 228),
  cv::Scalar( 23, 179, 227), cv::Scalar( 27, 181, 226), cv::Scalar( 31, 183, 225), cv::Scalar( 35, 185, 224), cv::Scalar( 39, 187, 223), cv::Scalar( 43, 188, 223), cv::Scalar( 47, 190, 222), cv::Scalar( 50, 192, 221),
  cv::Scalar( 54, 194, 220), cv::Scalar( 57, 195, 219), cv::Scalar( 61, 197, 218), cv::Scalar( 64, 199, 218), cv::Scalar( 68, 200, 217), cv::Scalar( 71, 202, 216), cv::Scalar( 75, 204, 215), cv::Scalar( 79, 205, 215),
  cv::Scalar( 83, 207, 214), cv::Scalar( 86, 208, 214), cv::Scalar( 90, 210, 213), cv::Scalar( 94, 211, 213), cv::Scalar( 98, 213, 212), cv::Scalar(101, 214, 212), cv::Scalar(105, 216, 212), cv::Scalar(109, 217, 212),
  cv::Scalar(113, 218, 212), cv::Scalar(117, 220, 212), cv::Scalar(121, 221, 212), cv::Scalar(125, 222, 212), cv::Scalar(128, 223, 212), cv::Scalar(132, 224, 213), cv::Scalar(136, 226, 213), cv::Scalar(140, 227, 214),
  cv::Scalar(144, 228, 214), cv::Scalar(147, 229, 215), cv::Scalar(151, 230, 216), cv::Scalar(155, 231, 217), cv::Scalar(159, 232, 218), cv::Scalar(162, 233, 219), cv::Scalar(166, 234, 220), cv::Scalar(170, 235, 221),
  cv::Scalar(173, 236, 222), cv::Scalar(177, 237, 224), cv::Scalar(181, 238, 225), cv::Scalar(184, 238, 226), cv::Scalar(188, 239, 228), cv::Scalar(191, 240, 229), cv::Scalar(195, 241, 230), cv::Scalar(199, 242, 231),
  cv::Scalar(202, 243, 233), cv::Scalar(206, 244, 234), cv::Scalar(209, 245, 236), cv::Scalar(213, 245, 237), cv::Scalar(216, 246, 238), cv::Scalar(220, 247, 240), cv::Scalar(223, 248, 241), cv::Scalar(227, 249, 243),
  cv::Scalar(230, 249, 244), cv::Scalar(234, 250, 246), cv::Scalar(237, 251, 247), cv::Scalar(241, 252, 249), cv::Scalar(244, 253, 250), cv::Scalar(248, 253, 252), cv::Scalar(251, 254, 253), cv::Scalar(255, 255, 255)
};

const cv::Scalar kColormapTurbo[] =
{
  cv::Scalar( 59,  18,  48), cv::Scalar( 66,  21,  49), cv::Scalar( 74,  24,  50), cv::Scalar( 81,  27,  52), cv::Scalar( 88,  30,  53), cv::Scalar( 95,  33,  54), cv::Scalar(101,  35,  55), cv::Scalar(108,  38,  56),
    cv::Scalar(114,  41,  57), cv::Scalar(121,  44,  58), cv::Scalar(127,  47,  59), cv::Scalar(133,  50,  60), cv::Scalar(139,  53,  60), cv::Scalar(145,  55,  61), cv::Scalar(150,  58,  62), cv::Scalar(156,  61,  63),
    cv::Scalar(161,  64,  64), cv::Scalar(166,  67,  64), cv::Scalar(171,  69,  65), cv::Scalar(176,  72,  65), cv::Scalar(181,  75,  66), cv::Scalar(186,  78,  67), cv::Scalar(190,  80,  67), cv::Scalar(194,  83,  67),
    cv::Scalar(199,  86,  68), cv::Scalar(203,  88,  68), cv::Scalar(206,  91,  69), cv::Scalar(210,  94,  69), cv::Scalar(214,  96,  69), cv::Scalar(217,  99,  69), cv::Scalar(221, 102,  70), cv::Scalar(224, 104,  70),
    cv::Scalar(227, 107,  70), cv::Scalar(230, 109,  70), cv::Scalar(232, 112,  70), cv::Scalar(235, 115,  70), cv::Scalar(237, 117,  70), cv::Scalar(240, 120,  70), cv::Scalar(242, 122,  70), cv::Scalar(244, 125,  70),
    cv::Scalar(246, 127,  70), cv::Scalar(248, 130,  70), cv::Scalar(249, 132,  69), cv::Scalar(251, 135,  69), cv::Scalar(252, 137,  69), cv::Scalar(253, 140,  68), cv::Scalar(253, 142,  67), cv::Scalar(254, 145,  66),
    cv::Scalar(254, 147,  65), cv::Scalar(254, 150,  64), cv::Scalar(254, 152,  63), cv::Scalar(254, 155,  62), cv::Scalar(253, 157,  60), cv::Scalar(252, 160,  59), cv::Scalar(252, 162,  57), cv::Scalar(251, 165,  56),
    cv::Scalar(249, 168,  54), cv::Scalar(248, 170,  52), cv::Scalar(246, 172,  51), cv::Scalar(245, 175,  49), cv::Scalar(243, 177,  47), cv::Scalar(241, 180,  45), cv::Scalar(239, 182,  43), cv::Scalar(237, 185,  42),
    cv::Scalar(235, 187,  40), cv::Scalar(233, 189,  38), cv::Scalar(230, 192,  37), cv::Scalar(228, 194,  35), cv::Scalar(225, 196,  33), cv::Scalar(223, 198,  32), cv::Scalar(220, 201,  30), cv::Scalar(218, 203,  29),
    cv::Scalar(215, 205,  28), cv::Scalar(212, 207,  27), cv::Scalar(210, 209,  26), cv::Scalar(207, 211,  25), cv::Scalar(204, 213,  24), cv::Scalar(202, 215,  24), cv::Scalar(199, 217,  23), cv::Scalar(196, 218,  23),
    cv::Scalar(194, 220,  23), cv::Scalar(191, 222,  23), cv::Scalar(189, 224,  24), cv::Scalar(186, 225,  24), cv::Scalar(184, 227,  25), cv::Scalar(182, 228,  26), cv::Scalar(180, 229,  27), cv::Scalar(177, 231,  29),
    cv::Scalar(175, 232,  30), cv::Scalar(172, 233,  32), cv::Scalar(169, 235,  34), cv::Scalar(166, 236,  36), cv::Scalar(163, 237,  39), cv::Scalar(160, 238,  41), cv::Scalar(157, 239,  44), cv::Scalar(154, 240,  47),
    cv::Scalar(151, 241,  50), cv::Scalar(148, 243,  53), cv::Scalar(145, 244,  56), cv::Scalar(141, 244,  59), cv::Scalar(138, 245,  63), cv::Scalar(135, 246,  66), cv::Scalar(131, 247,  70), cv::Scalar(128, 248,  74),
    cv::Scalar(124, 249,  77), cv::Scalar(121, 249,  81), cv::Scalar(118, 250,  85), cv::Scalar(114, 251,  89), cv::Scalar(111, 251,  93), cv::Scalar(108, 252,  97), cv::Scalar(104, 252, 101), cv::Scalar(101, 253, 105),
    cv::Scalar( 98, 253, 109), cv::Scalar( 95, 253, 113), cv::Scalar( 92, 254, 116), cv::Scalar( 89, 254, 120), cv::Scalar( 86, 254, 124), cv::Scalar( 83, 254, 128), cv::Scalar( 80, 254, 132), cv::Scalar( 77, 254, 135),
    cv::Scalar( 75, 254, 139), cv::Scalar( 72, 254, 142), cv::Scalar( 70, 254, 146), cv::Scalar( 68, 254, 149), cv::Scalar( 66, 254, 152), cv::Scalar( 64, 253, 155), cv::Scalar( 62, 253, 158), cv::Scalar( 61, 252, 161),
    cv::Scalar( 59, 252, 164), cv::Scalar( 58, 251, 166), cv::Scalar( 57, 251, 169), cv::Scalar( 55, 250, 172), cv::Scalar( 55, 249, 174), cv::Scalar( 54, 248, 177), cv::Scalar( 53, 248, 179), cv::Scalar( 53, 247, 182),
    cv::Scalar( 52, 245, 185), cv::Scalar( 52, 244, 187), cv::Scalar( 52, 243, 190), cv::Scalar( 51, 242, 192), cv::Scalar( 51, 241, 195), cv::Scalar( 51, 239, 197), cv::Scalar( 51, 238, 200), cv::Scalar( 51, 237, 202),
    cv::Scalar( 52, 235, 205), cv::Scalar( 52, 234, 207), cv::Scalar( 52, 232, 209), cv::Scalar( 53, 231, 212), cv::Scalar( 53, 229, 214), cv::Scalar( 53, 227, 216), cv::Scalar( 54, 226, 218), cv::Scalar( 54, 224, 221),
    cv::Scalar( 54, 222, 223), cv::Scalar( 55, 220, 225), cv::Scalar( 55, 218, 227), cv::Scalar( 56, 216, 229), cv::Scalar( 56, 215, 231), cv::Scalar( 56, 213, 232), cv::Scalar( 57, 211, 234), cv::Scalar( 57, 209, 236),
    cv::Scalar( 57, 207, 237), cv::Scalar( 57, 205, 239), cv::Scalar( 58, 203, 240), cv::Scalar( 58, 200, 242), cv::Scalar( 58, 198, 243), cv::Scalar( 58, 196, 244), cv::Scalar( 58, 194, 246), cv::Scalar( 57, 192, 247),
    cv::Scalar( 57, 190, 248), cv::Scalar( 57, 188, 249), cv::Scalar( 56, 186, 249), cv::Scalar( 55, 183, 250), cv::Scalar( 55, 181, 251), cv::Scalar( 54, 179, 251), cv::Scalar( 53, 176, 252), cv::Scalar( 52, 174, 252),
    cv::Scalar( 51, 171, 253), cv::Scalar( 50, 169, 253), cv::Scalar( 49, 166, 253), cv::Scalar( 48, 163, 253), cv::Scalar( 47, 161, 254), cv::Scalar( 46, 158, 254), cv::Scalar( 45, 155, 254), cv::Scalar( 44, 152, 254),
    cv::Scalar( 43, 149, 253), cv::Scalar( 41, 146, 253), cv::Scalar( 40, 143, 253), cv::Scalar( 39, 140, 253), cv::Scalar( 38, 137, 252), cv::Scalar( 36, 134, 252), cv::Scalar( 35, 131, 251), cv::Scalar( 34, 128, 251),
    cv::Scalar( 32, 125, 250), cv::Scalar( 31, 122, 250), cv::Scalar( 30, 119, 249), cv::Scalar( 28, 116, 248), cv::Scalar( 27, 113, 247), cv::Scalar( 26, 110, 247), cv::Scalar( 24, 107, 246), cv::Scalar( 23, 104, 245),
    cv::Scalar( 22, 101, 244), cv::Scalar( 21,  99, 243), cv::Scalar( 20,  96, 242), cv::Scalar( 19,  93, 241), cv::Scalar( 17,  90, 239), cv::Scalar( 16,  88, 238), cv::Scalar( 15,  85, 237), cv::Scalar( 14,  82, 236),
    cv::Scalar( 13,  80, 234), cv::Scalar( 13,  77, 233), cv::Scalar( 12,  75, 232), cv::Scalar( 11,  73, 230), cv::Scalar( 10,  70, 229), cv::Scalar( 10,  68, 227), cv::Scalar(  9,  66, 226), cv::Scalar(  8,  64, 224),
    cv::Scalar(  8,  62, 222), cv::Scalar(  7,  60, 221), cv::Scalar(  7,  58, 219), cv::Scalar(  6,  56, 217), cv::Scalar(  6,  54, 215), cv::Scalar(  5,  52, 214), cv::Scalar(  5,  50, 212), cv::Scalar(  5,  48, 210),
    cv::Scalar(  4,  47, 208), cv::Scalar(  4,  45, 206), cv::Scalar(  3,  43, 203), cv::Scalar(  3,  41, 201), cv::Scalar(  3,  40, 199), cv::Scalar(  2,  38, 197), cv::Scalar(  2,  36, 195), cv::Scalar(  2,  35, 192),
    cv::Scalar(  2,  33, 190), cv::Scalar(  1,  31, 187), cv::Scalar(  1,  30, 185), cv::Scalar(  1,  28, 182), cv::Scalar(  1,  27, 180), cv::Scalar(  1,  25, 177), cv::Scalar(  1,  24, 174), cv::Scalar(  1,  22, 172),
    cv::Scalar(  1,  21, 169), cv::Scalar(  1,  20, 166), cv::Scalar(  1,  18, 163), cv::Scalar(  1,  17, 160), cv::Scalar(  1,  16, 157), cv::Scalar(  1,  14, 154), cv::Scalar(  1,  13, 151), cv::Scalar(  1,  12, 148),
    cv::Scalar(  1,  11, 145), cv::Scalar(  1,  10, 142), cv::Scalar(  1,   9, 139), cv::Scalar(  1,   8, 135), cv::Scalar(  1,   7, 132), cv::Scalar(  2,   6, 129), cv::Scalar(  2,   5, 125), cv::Scalar(  2,   4, 122)
};

const cv::Scalar kColormapViridis[] =
{
  cv::Scalar( 84,   1,  68), cv::Scalar( 86,   2,  68), cv::Scalar( 87,   4,  69), cv::Scalar( 89,   5,  69), cv::Scalar( 90,   7,  70), cv::Scalar( 92,   8,  70), cv::Scalar( 93,  10,  70), cv::Scalar( 94,  11,  70),
  cv::Scalar( 96,  13,  71), cv::Scalar( 97,  14,  71), cv::Scalar( 99,  16,  71), cv::Scalar(100,  17,  71), cv::Scalar(101,  19,  71), cv::Scalar(103,  20,  72), cv::Scalar(104,  22,  72), cv::Scalar(105,  23,  72),
  cv::Scalar(106,  24,  72), cv::Scalar(108,  26,  72), cv::Scalar(109,  27,  72), cv::Scalar(110,  28,  72), cv::Scalar(111,  29,  72), cv::Scalar(112,  31,  72), cv::Scalar(113,  32,  72), cv::Scalar(115,  33,  72),
  cv::Scalar(116,  35,  72), cv::Scalar(117,  36,  72), cv::Scalar(118,  37,  72), cv::Scalar(119,  38,  72), cv::Scalar(120,  40,  72), cv::Scalar(121,  41,  72), cv::Scalar(122,  42,  71), cv::Scalar(122,  44,  71),
  cv::Scalar(123,  45,  71), cv::Scalar(124,  46,  71), cv::Scalar(125,  47,  71), cv::Scalar(126,  48,  70), cv::Scalar(126,  50,  70), cv::Scalar(127,  51,  70), cv::Scalar(128,  52,  70), cv::Scalar(129,  53,  69),
  cv::Scalar(129,  55,  69), cv::Scalar(130,  56,  69), cv::Scalar(131,  57,  68), cv::Scalar(131,  58,  68), cv::Scalar(132,  59,  68), cv::Scalar(132,  61,  67), cv::Scalar(133,  62,  67), cv::Scalar(133,  63,  66),
  cv::Scalar(134,  64,  66), cv::Scalar(134,  65,  66), cv::Scalar(135,  66,  65), cv::Scalar(135,  68,  65), cv::Scalar(136,  69,  64), cv::Scalar(136,  70,  64), cv::Scalar(136,  71,  63), cv::Scalar(137,  72,  63),
  cv::Scalar(137,  73,  62), cv::Scalar(137,  74,  62), cv::Scalar(138,  76,  62), cv::Scalar(138,  77,  61), cv::Scalar(138,  78,  61), cv::Scalar(138,  79,  60), cv::Scalar(139,  80,  60), cv::Scalar(139,  81,  59),
  cv::Scalar(139,  82,  59), cv::Scalar(139,  83,  58), cv::Scalar(140,  84,  58), cv::Scalar(140,  85,  57), cv::Scalar(140,  86,  57), cv::Scalar(140,  88,  56), cv::Scalar(140,  89,  56), cv::Scalar(140,  90,  55),
  cv::Scalar(141,  91,  55), cv::Scalar(141,  92,  54), cv::Scalar(141,  93,  54), cv::Scalar(141,  94,  53), cv::Scalar(141,  95,  53), cv::Scalar(141,  96,  52), cv::Scalar(141,  97,  52), cv::Scalar(141,  98,  51),
  cv::Scalar(141,  99,  51), cv::Scalar(142, 100,  50), cv::Scalar(142, 101,  50), cv::Scalar(142, 102,  49), cv::Scalar(142, 103,  49), cv::Scalar(142, 104,  49), cv::Scalar(142, 105,  48), cv::Scalar(142, 106,  48),
  cv::Scalar(142, 107,  47), cv::Scalar(142, 108,  47), cv::Scalar(142, 109,  46), cv::Scalar(142, 110,  46), cv::Scalar(142, 111,  46), cv::Scalar(142, 112,  45), cv::Scalar(142, 113,  45), cv::Scalar(142, 113,  44),
  cv::Scalar(142, 114,  44), cv::Scalar(142, 115,  44), cv::Scalar(142, 116,  43), cv::Scalar(142, 117,  43), cv::Scalar(142, 118,  42), cv::Scalar(142, 119,  42), cv::Scalar(142, 120,  42), cv::Scalar(142, 121,  41),
  cv::Scalar(142, 122,  41), cv::Scalar(142, 123,  41), cv::Scalar(142, 124,  40), cv::Scalar(142, 125,  40), cv::Scalar(142, 126,  39), cv::Scalar(142, 127,  39), cv::Scalar(142, 128,  39), cv::Scalar(142, 129,  38),
  cv::Scalar(142, 130,  38), cv::Scalar(142, 130,  38), cv::Scalar(142, 131,  37), cv::Scalar(142, 132,  37), cv::Scalar(142, 133,  37), cv::Scalar(142, 134,  36), cv::Scalar(142, 135,  36), cv::Scalar(142, 136,  35),
  cv::Scalar(142, 137,  35), cv::Scalar(141, 138,  35), cv::Scalar(141, 139,  34), cv::Scalar(141, 140,  34), cv::Scalar(141, 141,  34), cv::Scalar(141, 142,  33), cv::Scalar(141, 143,  33), cv::Scalar(141, 144,  33),
  cv::Scalar(140, 145,  33), cv::Scalar(140, 146,  32), cv::Scalar(140, 146,  32), cv::Scalar(140, 147,  32), cv::Scalar(140, 148,  31), cv::Scalar(139, 149,  31), cv::Scalar(139, 150,  31), cv::Scalar(139, 151,  31),
  cv::Scalar(139, 152,  31), cv::Scalar(138, 153,  31), cv::Scalar(138, 154,  31), cv::Scalar(138, 155,  30), cv::Scalar(137, 156,  30), cv::Scalar(137, 157,  30), cv::Scalar(137, 158,  31), cv::Scalar(136, 159,  31),
  cv::Scalar(136, 160,  31), cv::Scalar(136, 161,  31), cv::Scalar(135, 161,  31), cv::Scalar(135, 162,  31), cv::Scalar(134, 163,  32), cv::Scalar(134, 164,  32), cv::Scalar(133, 165,  33), cv::Scalar(133, 166,  33),
  cv::Scalar(133, 167,  34), cv::Scalar(132, 168,  34), cv::Scalar(131, 169,  35), cv::Scalar(131, 170,  36), cv::Scalar(130, 171,  37), cv::Scalar(130, 172,  37), cv::Scalar(129, 173,  38), cv::Scalar(129, 173,  39),
  cv::Scalar(128, 174,  40), cv::Scalar(127, 175,  41), cv::Scalar(127, 176,  42), cv::Scalar(126, 177,  44), cv::Scalar(125, 178,  45), cv::Scalar(124, 179,  46), cv::Scalar(124, 180,  47), cv::Scalar(123, 181,  49),
  cv::Scalar(122, 182,  50), cv::Scalar(121, 182,  52), cv::Scalar(121, 183,  53), cv::Scalar(120, 184,  55), cv::Scalar(119, 185,  56), cv::Scalar(118, 186,  58), cv::Scalar(117, 187,  59), cv::Scalar(116, 188,  61),
  cv::Scalar(115, 188,  63), cv::Scalar(114, 189,  64), cv::Scalar(113, 190,  66), cv::Scalar(112, 191,  68), cv::Scalar(111, 192,  70), cv::Scalar(110, 193,  72), cv::Scalar(109, 193,  74), cv::Scalar(108, 194,  76),
  cv::Scalar(107, 195,  78), cv::Scalar(106, 196,  80), cv::Scalar(105, 197,  82), cv::Scalar(104, 197,  84), cv::Scalar(103, 198,  86), cv::Scalar(101, 199,  88), cv::Scalar(100, 200,  90), cv::Scalar( 99, 200,  92),
  cv::Scalar( 98, 201,  94), cv::Scalar( 96, 202,  96), cv::Scalar( 95, 203,  99), cv::Scalar( 94, 203, 101), cv::Scalar( 92, 204, 103), cv::Scalar( 91, 205, 105), cv::Scalar( 90, 205, 108), cv::Scalar( 88, 206, 110),
  cv::Scalar( 87, 207, 112), cv::Scalar( 86, 208, 115), cv::Scalar( 84, 208, 117), cv::Scalar( 83, 209, 119), cv::Scalar( 81, 209, 122), cv::Scalar( 80, 210, 124), cv::Scalar( 78, 211, 127), cv::Scalar( 77, 211, 129),
  cv::Scalar( 75, 212, 132), cv::Scalar( 73, 213, 134), cv::Scalar( 72, 213, 137), cv::Scalar( 70, 214, 139), cv::Scalar( 69, 214, 142), cv::Scalar( 67, 215, 144), cv::Scalar( 65, 215, 147), cv::Scalar( 64, 216, 149),
  cv::Scalar( 62, 216, 152), cv::Scalar( 60, 217, 155), cv::Scalar( 59, 217, 157), cv::Scalar( 57, 218, 160), cv::Scalar( 55, 218, 162), cv::Scalar( 54, 219, 165), cv::Scalar( 52, 219, 168), cv::Scalar( 50, 220, 170),
  cv::Scalar( 48, 220, 173), cv::Scalar( 47, 221, 176), cv::Scalar( 45, 221, 178), cv::Scalar( 43, 222, 181), cv::Scalar( 41, 222, 184), cv::Scalar( 40, 222, 186), cv::Scalar( 38, 223, 189), cv::Scalar( 37, 223, 192),
  cv::Scalar( 35, 223, 194), cv::Scalar( 33, 224, 197), cv::Scalar( 32, 224, 200), cv::Scalar( 31, 225, 202), cv::Scalar( 29, 225, 205), cv::Scalar( 28, 225, 208), cv::Scalar( 27, 226, 210), cv::Scalar( 26, 226, 213),
  cv::Scalar( 25, 226, 216), cv::Scalar( 25, 227, 218), cv::Scalar( 24, 227, 221), cv::Scalar( 24, 227, 223), cv::Scalar( 24, 228, 226), cv::Scalar( 25, 228, 229), cv::Scalar( 25, 228, 231), cv::Scalar( 26, 229, 234),
  cv::Scalar( 27, 229, 236), cv::Scalar( 28, 229, 239), cv::Scalar( 29, 229, 241), cv::Scalar( 30, 230, 244), cv::Scalar( 32, 230, 246), cv::Scalar( 33, 230, 248), cv::Scalar( 35, 231, 251), cv::Scalar( 37, 231, 253)
};

template <typename T>
void ColorLookup(const cv::Mat &values, const cv::Scalar *map, cv::Mat &colored, double limit_from, double limit_to)
{
  VCP_CHECK_NOTNULL(map);

  double interval = (limit_to - limit_from)/255.0;
  // Prevent division by zero.
  const double divider = std::fabs(interval) > 0.0 ? interval : 1.0;

  for (int row = 0; row < values.rows; ++row)
  {
    const T *val_ptr = values.ptr<T>(row);
    unsigned char *col_ptr = colored.ptr<unsigned char>(row);
    for (int col = 0; col < values.cols; ++col)
    {
      T tval = *val_ptr++;
      double val = std::max(limit_from, std::min(limit_to, (double)tval));
      int lookup = static_cast<int>(std::floor((val-limit_from)/divider));
      cv::Scalar c = map[lookup];

      *col_ptr++ = (uchar)c.val[0];
      *col_ptr++ = (uchar)c.val[1];
      *col_ptr++ = (uchar)c.val[2];
    }
  }
}


inline const cv::Scalar *GetColorMap(const ColorMap &colormap)
{
  switch(colormap)
  {
  case ColorMap::Autumn:
    return kColormapAutumn;
  case ColorMap::Parula:
    return kColormapParula;
  case ColorMap::Bone:
    return kColormapBone;
  case ColorMap::Cold:
    return kColormapCold;
  case ColorMap::Disparity:
    return kColormapDisparity;
  case ColorMap::Earth:
    return kColormapEarth;
  case ColorMap::Gray:
    return kColormapGray;
  case ColorMap::Hot:
    return kColormapHot;
  case ColorMap::HSV:
    return kColormapHSV;
  case ColorMap::Inferno:
    return kColormapInferno;
  case ColorMap::Jet:
    return kColormapJet;
  case ColorMap::Magma:
    return kColormapMagma;
  case ColorMap::Pastel:
    return kColormapPastel;
  case ColorMap::Plasma:
    return kColormapPlasma;
  case ColorMap::Sepia:
    return kColormapSepia;
  case ColorMap::Temperature:
    return kColormapTemperature;
  case ColorMap::Thermal:
    return kColormapThermal;
  case ColorMap::Turbo:
    return kColormapTurbo;
  case ColorMap::Viridis:
    return kColormapViridis;
  default:
    VCP_ERROR("GetColorMap(): Given enum '" << colormap << "' is not (yet) supported!");
    break;
  }
  return nullptr;
}

const std::string ColorMapToString(const ColorMap &cm)
{
  switch(cm)
  {
  case ColorMap::Autumn:
    return "Autumn";
  case ColorMap::Bone:
    return "Bone";
  case ColorMap::Cold:
    return "Cold";
  case ColorMap::Disparity:
    return "Disparity";
  case ColorMap::Earth:
    return "Earth";
  case ColorMap::Gray:
    return "Gray";
  case ColorMap::Hot:
    return "Hot";
  case ColorMap::HSV:
    return "HSV";
  case ColorMap::Inferno:
    return "Inferno";
  case ColorMap::Jet:
    return "Jet";
  case ColorMap::Magma:
    return "Magma";
  case ColorMap::Parula:
    return "Parula";
  case ColorMap::Pastel:
    return "Pastel";
  case ColorMap::Plasma:
    return "Plasma";
  case ColorMap::Sepia:
    return "Sepia";
  case ColorMap::Temperature:
    return "Temperature";
  case ColorMap::Thermal:
    return "Thermal";
  case ColorMap::Turbo:
    return "Turbo";
  case ColorMap::Viridis:
    return "Viridis";
  default:
    {
      std::stringstream s;
      s << static_cast<int>(cm);
      return s.str();
    }
  }
}
std::ostream& operator<<(std::ostream & os, const ColorMap &cm)
{
  os << ColorMapToString(cm);
  return os;
}

void Colorize(const cv::Mat &values, const ColorMap &colormap, cv::Mat &colored, double limit_from, double limit_to)
{
  VCP_LOG_DEBUG("Colorize()");
  assert(values.channels() == 1);

  const cv::Scalar *map = GetColorMap(colormap);

  // If no limits are provided, use the full image data scale.
  if (!(std::fabs(limit_from) > 0.0 || std::fabs(limit_to) > 0.0))
  {
    cv::minMaxLoc(values, &limit_from, &limit_to);
  }

  // Allocate output memory.
  colored.create(values.size(), CV_8UC3);
  
  switch(values.type())
  {
  case CV_8UC1:  ColorLookup<unsigned char>(values, map, colored, limit_from, limit_to); break;
  case CV_32FC1: ColorLookup<float>(values, map, colored, limit_from, limit_to); break;
  case CV_32SC1: ColorLookup<int>(values, map, colored, limit_from, limit_to); break;
  case CV_64FC1: ColorLookup<double>(values, map, colored, limit_from, limit_to); break;
  default: VCP_ERROR("Colorize(): Image type not supported!"); break;
  }
}



cv::Scalar GetPseudocolor(double value, const ColorMap &colormap, double limit_from, double limit_to)
{
  VCP_LOG_DEBUG("GetPseudocolor()");
  const cv::Scalar *map = GetColorMap(colormap);

  // Lookup color
  double interval = (limit_to - limit_from)/255.0;
  const double divider = std::fabs(interval) > 0.0 ? interval : 1.0; // // Prevent division by zero.
  const double val = std::max(limit_from, std::min(limit_to, (double)value));
  int lookup = static_cast<int>(std::floor((val-limit_from)/divider));
  return map[lookup];
}


cv::Mat Highlight(const cv::Mat &image, const cv::Mat &mask, const cv::Scalar &highlight, double color_opacity)
{
  VCP_LOG_DEBUG("Highlight()");
  cv::Mat vis;
  vcp::imutils::ConvertTo8U(image, vis);

  const bool use_gray = highlight[0] < 0.0 || highlight[1] < 0.0 || highlight[2] < 0.0;
  if (use_gray)
  {
    if (image.channels() == 1)
    {
      VCP_LOG_WARNING("Highlight(): Cannot gray out the unmasked region if the input is already gray. Switching to a default highlight color.");
      return Highlight(vis, mask, cv::Scalar(255,0,0), color_opacity);
    }

    cv::Mat overlay = vis.clone();
    vcp::imutils::Ensure3Channels(overlay);
    vis = vcp::imutils::Grayscale(image, false, false);
    // Dampen the outside region even more
    cv::addWeighted(vis, 0.6, cv::Mat::zeros(vis.size(), vis.type()), 0.4, 0.0, vis);
    overlay.copyTo(vis, mask);
  }
  else
  {
    vcp::imutils::Ensure3Channels(vis);
    cv::Mat masked = vis.clone();
    masked.setTo(highlight, mask);
    if (color_opacity < 1.0)
      cv::addWeighted(masked, color_opacity, vis, 1.0-color_opacity, 0.0, vis);
    else
      vis = masked;
  }
  return vis;
}
} // namespace pseudocolor
} // namespace imvis
} // namespace vcp
