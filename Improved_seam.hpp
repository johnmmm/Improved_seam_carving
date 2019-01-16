#ifndef Improved_seam_hpp
#define Improved_seam_hpp

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

using namespace std;
using namespace cv;

const int INFMAX = 99999999;

void improved_seam_carving (Mat& inputImage, Mat& outputImage);


#endif