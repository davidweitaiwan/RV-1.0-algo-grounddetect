#include "re_tx1.h"

using namespace std;
Groundline::Groundline(int w, int h)
{
	this->idata = new int[w];
	this->jdata = new float[h];
	this->backup = new int[w];
}
Groundline::~Groundline()
{
	delete this->idata;
	delete this->jdata;
	delete this->backup;
}
void Groundline::parkinglot(const cv::Mat &inputMat, cv::Mat &outputMat, float *cammodx, bool &calibflag, bool &wflag, bool &rflag)
{
	int low1dist, low2dist, high1dist, high2dist;

	float dist2;//record distance between car and parking space
	int low1 = -1, low2 = -1, high1 = -1, high2 = -1;
	int X1_high = 0, X2_high = 0, Z1_high = 0, Z2_high = 0;
	int X1_low = 0, X2_low = 0, Z1_low = 0, Z2_low = 0;
	int m_low, m_high;
	int lowpoint = 0, highpoint = 0;
	float dist;
	bool spacetype = true;//false: road space;		true: garage space;
	bool highflag = false;
	for (int i = 0; i < outputMat.cols; i += 10)
	{
		dist = inputMat.at<float>(idata[i], i);
		if (dist >= dist1 && dist <= dist1 + 1000)				//distance between car and parking space
		{
			if (high1 == -1)									//if first high point not found
			{
				low1 = i;										//set first low point as low1
				dist2 = dist;									//record the distance between car and parking space
			}
			else if (low1 != -1 && high2 != -1)					//if first low point found & second high point found
			{
				if (spacetype)									//if garage space bool:1
				{
					if (abs(cammodx[idata[i] * inputMat.cols + i] - cammodx[idata[low1] * inputMat.cols + low1]) >= spaceL)				//parking space length 2.4m
					{
						low2 = i;								//set second low point as low2
						break;									//break loop
					}
				}
			}
		}
		else if (low1 != -1 && dist >= spaceW && dist <= spaceW + 2000)//garage space width 5.5m
		{
			if (spacetype)												//if garage space bool:1
			{
				if (low1 != -1 && high1 == -1)
					high1 = i;
				else if (low1 != -1 && high1 != -1)
					if (abs(cammodx[idata[i] * inputMat.cols + i] - cammodx[idata[high1] * inputMat.cols + high1]) >= spaceL)//parking space length 2.4m
						high2 = i;
			}
		}
	}
	if (low1 != -1)
		cv::circle(outputMat, cv::Point(low1, idata[low1]), 4, cv::Scalar(255, 255, 255), 2, 8);
	if (low2 != -1)
		cv::circle(outputMat, cv::Point(low2, idata[low2]), 4, cv::Scalar(0, 255, 255), 2, 8);
	if (high1 != -1)
		cv::circle(outputMat, cv::Point(high1, idata[high1]), 4, cv::Scalar(255, 0, 255), 2, 8);
	if (high2 != -1)
		cv::circle(outputMat, cv::Point(high2, idata[high2]), 4, cv::Scalar(128, 0, 255), 2, 8);

	if (low1 != -1 && high1 != -1 && high2 != -1)
	{
		cv::line(outputMat, cv::Point(low1, idata[low1]), cv::Point(high1, idata[high1]), cv::Scalar(255, 0, 0), 2, 8);
		cv::line(outputMat, cv::Point(high1, idata[high1]), cv::Point(high2, idata[high2]), cv::Scalar(255, 0, 0), 2, 8);
		cv::line(outputMat, cv::Point(high2, idata[high2]), cv::Point(high2, idata[low1]), cv::Scalar(255, 0, 0), 2, 8);
		cv::line(outputMat, cv::Point(high2, idata[low1]), cv::Point(low1, idata[low1]), cv::Scalar(255, 0, 0), 2, 8);
	}
	if (low1 != -1 && low2 != -1)//print lines
	{
		lowpoint = (idata[low1] + idata[low2]) / 2;
		highpoint = (idata[high1] + idata[high2]) / 2;
		cv::line(outputMat, cv::Point(low1, lowpoint), cv::Point(low2, lowpoint), cv::Scalar(0, 255, 0), 2, 8);
		cv::line(outputMat, cv::Point(low2, lowpoint), cv::Point(high2, highpoint), cv::Scalar(0, 255, 0), 2, 8);
		cv::line(outputMat, cv::Point(high2, highpoint), cv::Point(high1, highpoint), cv::Scalar(0, 255, 0), 2, 8);
		cv::line(outputMat, cv::Point(high1, highpoint), cv::Point(low1, lowpoint), cv::Scalar(0, 255, 0), 2, 8);

		low1dist = abs(cammodx[idata[low1] * inputMat.cols + low1]) / 10;	//cm
		low2dist = abs(cammodx[idata[low2] * inputMat.cols + low2]) / 10;	//cm
		X1_low = low1dist & 0xff;
		X1_high = (low1dist >> 8) & 0xff;
		m_low = (int)(inputMat.at<float>(idata[low1], low1) / 10) & 0xff;
		m_high = ((int)(inputMat.at<float>(idata[low1], low1) / 10) >> 8) & 0xff;
		Z1_low = (int)(inputMat.at<float>(idata[high1], high1) / 10) & 0xff;
		Z1_high = ((int)(inputMat.at<float>(idata[high1], high1) / 10) >> 8) & 0xff;
		Z2_low = (int)(inputMat.at<float>(idata[high2], high2) / 10) & 0xff;
		Z2_high = ((int)(inputMat.at<float>(idata[high2], high2) / 10) >> 8) & 0xff;
		X2_low = low2dist & 0xff;
		X2_high = (low2dist >> 8) & 0xff;

	}
	else
	{
		X1_high = 255; X1_low = 255; Z1_high = 255; Z1_low = 255; m_high = 255;
		X2_high = 255; X2_low = 255; Z2_high = 255; Z2_low = 255; m_low = 255;
	}
	if (calibflag)
	{
		cv::namedWindow("setPrkcalib", cv::WINDOW_NORMAL);
		cv::createTrackbar("spaceW", "setPrkcalib", &spaceW, 8000);
		cv::createTrackbar("spaceL", "setPrkcalib", &spaceL, 4000);
		cv::createTrackbar("dist1", "setPrkcalib", &dist1, 2000);
		calibflag = 0;
	}
	if (wflag)
	{
		static string label[] = { "spaceW", "spaceL", "dist1" };
		static int para[] = { spaceW, spaceL,  dist1 };
		FileWrite(const_cast<char *>("Prkparameter.txt"), label, para, sizeof(para) / sizeof(para[0]));
		wflag = false;
	}
}
void Groundline::obstacledetect(const cv::Mat &inputMat, cv::Mat &outputMat, float *cammodx, bool &calibflag, bool &wflag, bool &rflag)
{
	int x11 = 0, x12 = 0, high1 = inputMat.rows - 1, low1 = 0;//x11:方塊第一點 x12:方塊第二點 high1:向上找最高點 low1:最低點
	int boxcnt = 0;		//計算方塊數量
	bool x1flag = false;//是否找到方塊第一點 旗標
	bool x2flag = false;
	bool drawbox = false;
	int target[60];
	float distance1 = 0;
	int thres = 100;
	int step = 10;

	for (int i = 0; i < inputMat.cols; i += step)
	{
		if (i > 0)
		{
			if (abs(inputMat.at<float>((idata[i] - 10) < 0 ? idata[i] : idata[i] - 10, i) - inputMat.at<float>((idata[i - step] - 10) < 0 ? idata[i - step] : idata[i - step] - 10, i - step)) <= thres)
			{
				if (!x1flag)
				{
					x11 = i - step;
					x1flag = 1;
					x2flag = 0;
					high1 = inputMat.rows - 1;
					low1 = 0;
				}
				if (i >= inputMat.cols - step - 1)
					goto ENDCOLS;
			}
			else
			{
				if (!x2flag)
				{
				ENDCOLS:
					x12 = i - step;
					if (abs(cammodx[idata[x11] * outputMat.cols + x11] - cammodx[idata[x12] * outputMat.cols + x12]) >= 800)
						drawbox = 1;
					else
						high1 = inputMat.rows - 1;
					x2flag = 1;
					x1flag = 0;
				}
			}
		}
		distance1 = inputMat.at<float>((idata[i] - 10) < 0 ? idata[i] : idata[i] - 10, i);

		for (int j = idata[i] - 10; j >= 0; j -= step)
		{
			if (abs(inputMat.at<float>(j, i) - distance1) <= thres)
			{
				cv::circle(outputMat, cv::Point(i, j), 3, distancecolor(inputMat.at<float>(j, i), 10000), 1, 8);
				if (j <= high1)
					high1 = j;
				if (j >= low1)
					low1 = j;
			}
			else
				break;
		}
		cv::circle(outputMat, cv::Point(i, high1), 3, cv::Scalar(0, 0, 255), 1, 8);
		cv::circle(outputMat, cv::Point(i, low1), 3, cv::Scalar(255, 0, 0), 1, 8);
		if (drawbox)
		{
			cv::circle(outputMat, cv::Point(x11, 10), 3, cv::Scalar(0, 0, 0), 1, 8);
			cv::circle(outputMat, cv::Point(x12, 10), 3, cv::Scalar(255, 255, 255), 1, 8);
			boxcnt++;
			cv::rectangle(outputMat, cv::Rect(cv::Point(x11, low1), cv::Point(x12, high1)), distancecolor(boxcnt * 1200, 10000), 2, 8);
			drawbox = 0;
			high1 = inputMat.rows - 1;
			low1 = 0;
			target[boxcnt * 2 - 1] = x11;
			target[boxcnt * 2] = x12;
		}
	}
	if (calibflag)
	{
		cv::namedWindow("setObscalib", cv::WINDOW_NORMAL);
		cv::createTrackbar("thres", "setObscalib", &thres, 500);
		cv::createTrackbar("step", "setObscalib", &step, 100);
		calibflag = 0;
	}
	cv::putText(outputMat, cv::String("Targets:" + std::to_string(boxcnt)), cv::Point(10, inputMat.rows - 10), 0, 0.6, cv::Scalar(100, 100, 255), 1.8, 8);
}
int Groundline::MA_func(const cv::Mat &inputMat, int &MA_col, int i)
{
	float avg = 0, diff = 0;
	//MA_col開始
	avg = 0;
	diff = 0;
	int cnt = MA_col;
	for (int a = i - (MA_col - 1); a <= i; a++)
	{
		avg += idata[a];
		backup[a] = idata[a];
	}

	avg /= MA_col;
	for (int b = i - (MA_col - 1); b <= i; b++)
		diff += abs(idata[b] - avg);
	diff /= MA_col;
	for (int c = i - (MA_col - 1); c <= i; c++)
		if ((idata[c] > (avg + diff)) || (idata[c] < (avg - diff)))
		{
			backup[c] = 0;
			cnt--;
		}
	avg = 0;
	for (int d = i - (MA_col - 1); d <= i; d++)
		avg += backup[d];
	if (cnt > 0)
		avg /= cnt;
	else
	{
		if (i >= MA_col)
			avg = idata[i - MA_col];
		else
			avg = inputMat.rows - 1;
	}
	//MA_col結束
	return avg;
}
void Groundline::ground_detect(const cv::Mat &inputMat, cv::Mat &outputMat, float *cammodz, float *cammodx, bool &obsprk, bool &calibflag, bool &wflag, bool &rflag)
{
	int MA_col = 20;
	bool checkpoint = false;

	for (int i = 0; i < inputMat.cols; i++)
	{
		checkpoint = false;
		for (int j = inputMat.rows - 1; j >= 0; j--)
		{
			jdata[j] = inputMat.at<float>(j, i);
			if (jdata[j] == INFINITY)
				jdata[j] = 20000;
			if ((cammodz[j] - jdata[j]) > camthres)
			{
				if (!checkpoint)
				{
					idata[i] = j;
				}
				checkpoint = true;
			}
		}
		if (!checkpoint)
		{
			idata[i] = idata[i - 1];
		}
		if (i >= (MA_col - 1))
		{
			idata[i - (MA_col - 1)] = MA_func(inputMat, MA_col, i);//MA
			if (i > (MA_col - 1))
			{
				cv::line(outputMat, cv::Point(i - MA_col, idata[i - MA_col]),
					cv::Point(i - (MA_col - 1), idata[i - (MA_col - 1)]), cv::Scalar(0, 0, 255), 3);
			}
		}
	}
	if (obsprk)
		parkinglot(inputMat, outputMat, cammodx, calibflag, wflag, rflag);
	else
		obstacledetect(inputMat, outputMat, cammodx, calibflag, wflag, rflag);
}
