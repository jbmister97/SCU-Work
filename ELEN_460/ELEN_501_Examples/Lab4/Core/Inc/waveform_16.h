// waveform_16.h


#ifndef _WAVEFORM_16_
#define _WAVEFORM_16_

const uint16_t sine_table_16_256[] = 
{
  32768, 33572, 34375, 35178, 35979, 36779, 37575, 38369, 39160, 39947, 40729, 41507, 42279, 43046, 43806, 44560, 
  45307, 46046, 46777, 47500, 48214, 48918, 49613, 50298, 50972, 51635, 52287, 52927, 53555, 54170, 54772, 55362, 
  55937, 56499, 57046, 57579, 58097, 58599, 59086, 59557, 60012, 60451, 60873, 61278, 61665, 62036, 62389, 62723, 
  63040, 63339, 63619, 63881, 64124, 64348, 64553, 64738, 64905, 65052, 65180, 65288, 65377, 65446, 65495, 65525, 
  65535, 65525, 65495, 65446, 65377, 65288, 65180, 65052, 64905, 64738, 64553, 64348, 64124, 63881, 63619, 63339, 
  63040, 62723, 62389, 62036, 61665, 61278, 60873, 60451, 60012, 59557, 59086, 58599, 58097, 57579, 57046, 56499, 
  55937, 55362, 54772, 54170, 53555, 52927, 52287, 51635, 50972, 50298, 49613, 48918, 48214, 47500, 46777, 46046,
  45307, 44560, 43806, 43046, 42279, 41507, 40729, 39947, 39160, 38369, 37575, 36779, 35979, 35178, 34375, 33572, 
  32768, 31963, 31160, 30357, 29556, 28756, 27960, 27166, 26375, 25588, 24806, 24028, 23256, 22489, 21729, 20975, 
  20228, 19489, 18758, 18035, 17321, 16617, 15922, 15237, 14563, 13900, 13248, 12608, 11980, 11365, 10763, 10173, 
  9598, 9036, 8489, 7956, 7438, 6936, 6449, 5978, 5523, 5084, 4662, 4257, 3870, 3499, 3146, 2812, 
  2495, 2196, 1916, 1654, 1411, 1187, 982, 797, 630, 483, 355, 247, 158, 89, 40, 10, 
  1, 10, 40, 89, 158, 247, 355, 483, 630, 797, 982, 1187, 1411, 1654, 1916, 2196, 
  2495, 2812, 3146, 3499, 3870, 4257, 4662, 5084, 5523, 5978, 6449, 6936, 7438, 7956, 8489, 9036, 
  9598, 10173, 10763, 11365, 11980, 12608, 13248, 13900, 14563, 15237, 15922, 16617, 17321, 18035, 18758, 19489, 
  20228, 20975, 21729, 22489, 23256, 24028, 24806, 25588, 26375, 27166, 27960, 28756, 29556, 30357, 31160, 31963
};


const uint16_t half_sine_table_16_256[] = 
{
  0,       804,  1608,  2412,  3215,  4018,  4821,  5622,  6423,  7223,  8022,  8819,  9615, 10410, 11203, 11995, 
  12785, 13573, 14358, 15142, 15923, 16702, 17479, 18252, 19023, 19791, 20557, 21319, 22078, 22833, 23585, 24334,
  25079, 25820, 26557, 27290, 28019, 28744, 29465, 30181, 30892, 31599, 32302, 32999, 33691, 34378, 35061, 35737,
  36409, 37075, 37735, 38390, 39039, 39682, 40319, 40950, 41574, 42193, 42805, 43411, 44010, 44603, 45189, 45768, 
  46340, 46905, 47463, 48014, 48558, 49094, 49623, 50145, 50659, 51165, 51664, 52155, 52638, 53113, 53580, 54039, 
  54490, 54933, 55367, 55793, 56211, 56620, 57021, 57413, 57796, 58171, 58537, 58894, 59242, 59582, 59912, 60234, 
  60546, 60849, 61143, 61428, 61704, 61970, 62227, 62474, 62713, 62941, 63161, 63370, 63570, 63761, 63942, 64114, 
  64275, 64427, 64570, 64702, 64825, 64938, 65042, 65135, 65219, 65293, 65357, 65411, 65456, 65490, 65515, 65530,
  65535, 65530, 65515, 65490, 65456, 65411, 65357, 65293, 65219, 65135, 65042, 64938, 64825, 64702, 64570, 64427, 
  64275, 64114, 63942, 63761, 63570, 63370, 63161, 62941, 62713, 62474, 62227, 61970, 61704, 61428, 61143, 60849,
  60546, 60234, 59912, 59582, 59242, 58894, 58537, 58171, 57796, 57413, 57021, 56620, 56211, 55793, 55367, 54933,
  54490, 54039, 53580, 53113, 52638, 52155, 51664, 51165, 50659, 50145, 49623, 49094, 48558, 48014, 47463, 46905, 
  46340, 45768, 45189, 44603, 44010, 43411, 42805, 42193, 41574, 40950, 40319, 39682, 39039, 38390, 37735, 37075,
  36409, 35737, 35061, 34378, 33691, 32999, 32302, 31599, 30892, 30181, 29465, 28744, 28019, 27290, 26557, 25820,
  25079, 24334, 23585, 22833, 22078, 21319, 20557, 19791, 19023, 18252, 17479, 16702, 15923, 15142, 14358, 13573,
  12785, 11995, 11203, 10410,  9615,  8819,  8022,  7223,  6423,  5622,  4821,  4018,  3215,  2412,  1608,   804,
};


const uint16_t linear_ramp_table_16_256[] = 
{
  0,257,514,771,1028,1285,1542,1799,2056,2313,2570,2827,3084,3341,3598,3855,
  4112,4369,4626,4883,5140,5397,5654,5911,6168,6425,6682,6939,7196,7453,7710,7967,
  8224,8481,8738,8995,9252,9509,9766,10023,10280,10537,10794,11051,11308,11565,11822,12079,
  12336,12593,12850,13107,13364,13621,13878,14135,14392,14649,14906,15163,15420,15677,15934,16191,
  16448,16705,16962,17219,17476,17733,17990,18247,18504,18761,19018,19275,19532,19789,20046,20303,
  20560,20817,21074,21331,21588,21845,22102,22359,22616,22873,23130,23387,23644,23901,24158,24415,
  24672,24929,25186,25443,25700,25957,26214,26471,26728,26985,27242,27499,27756,28013,28270,28527,
  28784,29041,29298,29555,29812,30069,30326,30583,30840,31097,31354,31611,31868,32125,32382,32639,
  32896,33153,33410,33667,33924,34181,34438,34695,34952,35209,35466,35723,35980,36237,36494,36751,
  37008,37265,37522,37779,38036,38293,38550,38807,39064,39321,39578,39835,40092,40349,40606,40863,
  41120,41377,41634,41891,42148,42405,42662,42919,43176,43433,43690,43947,44204,44461,44718,44975,
  45232,45489,45746,46003,46260,46517,46774,47031,47288,47545,47802,48059,48316,48573,48830,49087,
  49344,49601,49858,50115,50372,50629,50886,51143,51400,51657,51914,52171,52428,52685,52942,53199,
  53456,53713,53970,54227,54484,54741,54998,55255,55512,55769,56026,56283,56540,56797,57054,57311,
  57568,57825,58082,58339,58596,58853,59110,59367,59624,59881,60138,60395,60652,60909,61166,61423,
  61680,61937,62194,62451,62708,62965,63222,63479,63736,63993,64250,64507,64764,65021,65278,65535,
};




const uint16_t triangle_wave_table_16_256[] = 
{
  0,514,1028,1542,2056,2570,3084,3598,4112,4626,5140,5654,6168,6682,7196,7710,
  8224,8738,9252,9766,10280,10794,11308,11822,12336,12850,13364,13878,14392,14906,15420,15934,
  16448,16962,17476,17990,18504,19018,19532,20046,20560,21074,21588,22102,22616,23130,23644,24158,
  24672,25186,25700,26214,26728,27242,27756,28270,28784,29298,29812,30326,30840,31354,31868,32382,
  32896,33410,33924,34438,34952,35466,35980,36494,37008,37522,38036,38550,39064,39578,40092,40606,
  41120,41634,42148,42662,43176,43690,44204,44718,45232,45746,46260,46774,47288,47802,48316,48830,
  49344,49858,50372,50886,51400,51914,52428,52942,53456,53970,54484,54998,55512,56026,56540,57054,
  57568,58082,58596,59110,59624,60138,60652,61166,61680,62194,62708,63222,63736,64250,64764,65278,
  65535,65278,64764,64250,63736,63222,62708,62194,61680,61166,60652,60138,59624,59110,58596,58082,
  57568,57054,56540,56026,55512,54998,54484,53970,53456,52942,52428,51914,51400,50886,50372,49858,
  49344,48830,48316,47802,47288,46774,46260,45746,45232,44718,44204,43690,43176,42662,42148,41634,
  41120,40606,40092,39578,39064,38550,38036,37522,37008,36494,35980,35466,34952,34438,33924,33410,
  32896,32382,31868,31354,30840,30326,29812,29298,28784,28270,27756,27242,26728,26214,25700,25186,
  24672,24158,23644,23130,22616,22102,21588,21074,20560,20046,19532,19018,18504,17990,17476,16962,
  16448,15934,15420,14906,14392,13878,13364,12850,12336,11822,11308,10794,10280,9766,9252,8738,
  8224,7710,7196,6682,6168,5654,5140,4626,4112,3598,3084,2570,2056,1542,1028,514,
};

#endif
