#ifndef Video_seam_hpp
#define Video_seam_hpp

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>
#include <iostream>
#include <cmath>
#include <algorithm>
#include <map>
#include <unordered_map>
#include <set>
#include <stack>
#include <vector>
#include <time.h>
#include <queue>

using namespace std;
using namespace cv;

const int INFMAXI = 99999999;
const int MAXI_SIZE = 10000000;
const int FRAME_NUM = 3;

void video_seam_carving ();


#endif