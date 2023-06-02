#pragma once
#define TX2_used

#include <opencv2/opencv.hpp>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <time.h>
#include <math.h>
#include <fstream>
#include <X11/Xlib.h>

#include <mutex>

#ifdef _SL_JETSON_
//Import Canbus API
#include <unistd.h>
#include <string.h>
#include <signal.h>
#include <libgen.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <sys/uio.h>
#include <linux/can.h>
#include <net/if.h>
#include <linux/can/raw.h>

#pragma comment(lib,"sl_zed64.lib") 
#pragma comment(lib,"freeglut.lib") 
#pragma comment(lib,"glew32.lib")
#pragma comment(lib,"nppi.lib")
#pragma comment(lib,"cuda.lib") 
#pragma comment(lib,"cudart.lib")

#ifdef _DEBUG
#pragma comment(lib,"../../../CV31_VC14/lib/opencv_imgproc310D.lib")
#pragma comment(lib,"../../../CV31_VC14/lib/opencv_core310D.lib")
#pragma comment(lib,"../../../CV31_VC14/lib/opencv_highgui310D.lib")
#pragma comment(lib,"../../../CV31_VC14/lib/opencv_imgcodecs310D.lib")
#pragma comment(lib,"../../../CV31_VC14/lib/opencv_objdetect310D.lib")
#else
#pragma comment(lib,"../../../CV31_VC14/lib/opencv_imgproc310.lib")
#pragma comment(lib,"../../../CV31_VC14/lib/opencv_core310.lib")
#pragma comment(lib,"../../../CV31_VC14/lib/opencv_highgui310.lib")
#pragma comment(lib,"../../../CV31_VC14/lib/opencv_imgcodecs310.lib")
#pragma comment(lib,"../../../CV31_VC14/lib/opencv_objdetect310.lib")
#endif
#endif
void FileWrite(char *, std::string label[], int *, int );
int FileRead(char *, std::string label[], int *, int);
cv::Scalar distancecolor(int distance, int max);
void radar(cv::Mat &outputMat, float *cammodz, float *cammodx, cv::Point target[], int boxcnt, cv::Scalar color);
void resizeWindowOnce(std::string winname, int screenx, int screeny, bool &rflag, bool &radarflag);

class Grounddot
{
private:
	FILE * dOBSlog;
	FILE *dPRKlog;
	int buswidth = 2000;
	int carwidth = 1500;
	int motorwidth = 1000;
	int *grounddotx;
	int *grounddoty;
	int arrw;
	int arrh;

	int *idata3x;
	int *idata3y;
	int *idata4x;
	int *idata4y;
	void sortindex();

	float nearestDepth_ = 100000;
	std::mutex nearestDepthLock_;

public:
	Grounddot();
	~Grounddot();

	int gndspl = 100;
	int gndspw = 100;

	int linethres = 100;

	int spacelength = 4000;
	int spacewidth = 2400;

	int buslength = 8000;
	int carlength = 3000;
	int motorlength = 1000;

	cv::Point target[40];

	void parkinglotdetect(const cv::Mat &inputMat, cv::Mat &outputMat, float *cammodz, float *cammodx, bool &calibflag, bool &wflag, bool &rflag);
	void vehicledetect(const cv::Mat &inputMat, cv::Mat &outputMat, float *cammodz, float *cammodx, bool &calibflag, bool &wflag, bool &rflag);
	void gnddotline(const cv::Mat &inputMat, cv::Mat &outputMat, float *cammodz, float *cammodx, bool &obsprk, bool &calibflag, bool &wflag, bool &rflag);
	void grounddotSearching(cv::Mat &outputMat, float *cammodz, float *cammodx, bool &wflag);

	float getNearestDepth();
};


class Groundline
{
private:
	float *jdata;
	int *idata;
	int *backup;
	int MA_func(const cv::Mat &inputMat, int &MA_col, int i);
public:
	int dist1 = 1500;
	int spaceW = 5000;
	int spaceL = 2400;
	int camthres = 550;
	Groundline(int w, int h);
	~Groundline();
	void parkinglot(const cv::Mat &inputMat, cv::Mat &outputMat, float *cammodx, bool &calibflag, bool &wflag, bool &rflag);
	void obstacledetect(const cv::Mat &inputMat, cv::Mat &outputMat, float *cammodx, bool &calibflag, bool &wflag, bool &rflag);
	void ground_detect(const cv::Mat &inputMat, cv::Mat &outputMat, float *cammodz, float *cammodx, bool &obsprk, bool &calibflag, bool &wflag, bool &rflag);
};


class Cameramodule
{
private:
	const float todeg = 0.017453;
	const float c_f = 3.6;
	const float c_z = 1.9;//float z = 2.075434;
	const float c_x = 3.3;
	const float yh = 0;
	float halfrow;
	float halfcol;
	void calibdeg(cv::Mat &outputMat);
public:
	float *cammodz;
	float *cammodx;
	int c_theta = 8;
	int c_h = 740;
	Cameramodule(int w, int h);
	~Cameramodule();
	void cammodset(cv::Mat &outputMat, bool &wflag);
};
