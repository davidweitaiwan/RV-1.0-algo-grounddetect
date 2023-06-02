#include "re_tx1.h"
using namespace std;
Grounddot::Grounddot()
{
	dOBSlog = fopen("dOBSlog.txt", "a");
	dPRKlog = fopen("dPRKlog.txt", "a");
	time_t filetime = time(NULL);
	fprintf(dOBSlog, "%s", ctime(&filetime));
	fprintf(dPRKlog, "%s", ctime(&filetime));
}
Grounddot::~Grounddot()
{
	fclose(dOBSlog);
	fclose(dPRKlog);
}
void Grounddot::parkinglotdetect(const cv::Mat &inputMat, cv::Mat &outputMat, float *cammodz, float *cammodx, bool &calibflag, bool &wflag, bool &rflag)
{
	int boxcnt = 0;		//count the 
	bool x1flag = false;//find first dot flag
	bool x2flag = false;
	int firstpoint, lastpoint;
	static int cntframe = 0;
	static cv::Point points[1][4];
	const cv::Point* ppt[1] = { points[0] };
	const int npt[] = { 4 };
	cv::Scalar vhctypecolor;
	memset(target, -1, 40 * sizeof(cv::Point));

	int X1_high = 0, X2_high = 0, Y1_high = 0, Y2_high = 0;
	int X1_low = 0, X2_low = 0, Y1_low = 0, Y2_low = 0;

	if (cntframe >= 10000)
		cntframe = 0;
	for (int i = 0; i < arrw; i++)
	{
		if (i > 0 && idata3x[i] != -1 && idata3y[i] != -1)
		{
			cv::line(outputMat, cv::Point(idata3x[i - 1], idata3y[i - 1]), cv::Point(idata3x[i], idata3y[i]), cv::Scalar(100, 255, 100), 2, 8);
			if (i > 1 && cammodz[idata3y[i]] <= 18000 && idata4y[i - 1] - idata4y[i - 2] == 0 && cammodz[idata3y[i]] - cammodz[idata3y[i - 2]] >= spacelength)
			{
				if (!x1flag && !x2flag)
				{
					x1flag = 1;
					firstpoint = i - 1;
				}
			}
			else if (i > 1 && cammodz[idata3y[i]] <= 18000 && x1flag && !x2flag && (cammodz[idata3y[i - 1]] - cammodz[idata3y[i]] >= spacelength || cammodz[idata3y[i - 2]] - cammodz[idata3y[i]] >= spacelength))
			{
				x2flag = 1;
				lastpoint = i;
			}
			else if (x1flag && !x2flag && abs(idata4y[i] - idata4y[firstpoint + 1]) <= 3)
			{

			}
			else
			{
				x1flag = 0;
				x2flag = 0;
			}
			if (x1flag)
				cv::circle(outputMat, cv::Point(idata3x[firstpoint], idata3y[firstpoint]), 4, cv::Scalar(255, 255, 255), 2, 8);
			if (x1flag)
				cv::circle(outputMat, cv::Point(idata3x[firstpoint + 1], idata3y[firstpoint + 1]), 4, cv::Scalar(255, 128, 255), 2, 8);
			if (x2flag)
				cv::circle(outputMat, cv::Point(idata3x[lastpoint], idata3y[lastpoint]), 4, cv::Scalar(0, 255, 255), 2, 8);
			if (x2flag)
				cv::circle(outputMat, cv::Point(idata3x[lastpoint - 1], idata3y[lastpoint - 1]), 4, cv::Scalar(255, 0, 128), 2, 8);
			if (x1flag && x2flag)
			{
				x1flag = 0;
				x2flag = 0;
				int templ = (lastpoint - firstpoint) * gndspl;
				int lengthcnt;
				if (templ >= spacewidth)
				{
					lengthcnt = spacelength / gndspw;
					points[0][0] = cv::Point(idata3x[firstpoint], idata3y[firstpoint]);
					points[0][1] = cv::Point(idata3x[lastpoint], idata3y[lastpoint]);
					//points[0][2] = cv::Point(grounddotx[idata4x[lastpoint] + idata4y[lastpoint] * arrw], grounddoty[idata4x[lastpoint] + idata4y[lastpoint] * arrw]);
					//points[0][3] = cv::Point(grounddotx[idata4x[firstpoint] + idata4y[firstpoint] * arrw], grounddoty[idata4x[firstpoint] + idata4y[firstpoint] * arrw]);
					points[0][2] = cv::Point((grounddotx[idata4x[lastpoint] + ((idata4y[lastpoint] + lengthcnt >= arrh) ? idata4y[lastpoint] : idata4y[lastpoint] + lengthcnt) * arrw]), (grounddoty[idata4x[lastpoint] + ((idata4y[lastpoint] + lengthcnt >= arrh) ? idata4y[lastpoint] : idata4y[lastpoint] + lengthcnt) * arrw]));
					points[0][3] = cv::Point((grounddotx[idata4x[firstpoint] + ((idata4y[firstpoint] + lengthcnt >= arrh) ? idata4y[firstpoint] : idata4y[firstpoint] + lengthcnt) * arrw]), (grounddoty[idata4x[firstpoint] + ((idata4y[firstpoint] + lengthcnt >= arrh) ? idata4y[firstpoint] : idata4y[firstpoint] + lengthcnt) * arrw]));
					//points[0][2] = cv::Point((grounddotx[idata4x[lastpoint] + (idata4y[lastpoint] + widthcnt) * arrw] == -1) ? idata3x[lastpoint] : grounddotx[idata4x[lastpoint] + (idata4y[lastpoint] + widthcnt) * arrw], (grounddoty[idata4x[lastpoint] + (idata4y[lastpoint] + widthcnt) * arrw] == -1) ? idata3y[lastpoint] : grounddoty[idata4x[lastpoint] + (idata4y[lastpoint] + widthcnt) * arrw]);
					//points[0][3] = cv::Point((grounddotx[idata4x[firstpoint] + (idata4y[firstpoint] + widthcnt) * arrw] == -1) ? idata3x[firstpoint] : grounddotx[idata4x[firstpoint] + (idata4y[firstpoint] + widthcnt) * arrw], (grounddoty[idata4x[firstpoint] + (idata4y[firstpoint] + widthcnt) * arrw] == -1) ? idata3y[firstpoint] : grounddoty[idata4x[firstpoint] + (idata4y[firstpoint] + widthcnt) * arrw]);
					vhctypecolor = cv::Scalar(0, 0, 255);
					cv::polylines(outputMat, ppt, npt, 1, 1, vhctypecolor, 5);
					fprintf(dPRKlog, "(%d, %d)(%d, %d)\tframe:%d\n", points[0][3].x, points[0][3].y, points[0][2].x, points[0][2].y, cntframe);
					cv::putText(outputMat, "x: " + std::to_string((int)cammodx[points[0][0].x + points[0][0].y * outputMat.cols]), cv::Point(points[0][0].x, points[0][0].y + 10), 0, 0.8, cv::Scalar(50, 255, 255), 2, 8);
					cv::putText(outputMat, "x: " + std::to_string((int)cammodx[points[0][1].x + points[0][1].y * outputMat.cols]), cv::Point(points[0][1].x, points[0][1].y + 10), 0, 0.8, cv::Scalar(50, 255, 255), 2, 8);
					cv::putText(outputMat, "y: " + std::to_string((int)cammodz[points[0][0].y]), cv::Point(points[0][0].x, points[0][0].y + 30), 0, 0.8, cv::Scalar(0, 120, 200), 2, 8);
					cv::putText(outputMat, "y: " + std::to_string((int)cammodz[points[0][1].y]), cv::Point(points[0][1].x, points[0][1].y + 30), 0, 0.8, cv::Scalar(0, 120, 200), 2, 8);
					target[boxcnt * 2] = points[0][0];
					target[boxcnt * 2 + 1] = points[0][2];
					boxcnt++;

					X1_low = (int)(cammodx[points[0][0].x + points[0][0].y * outputMat.cols] / 10) & 0xff;
					X1_high = ((int)(cammodx[points[0][0].x + points[0][0].y * outputMat.cols] / 10) >> 8) & 0xff;
					Y1_low = (int)(cammodz[points[0][0].y] / 10) & 0xff;
					Y1_high = ((int)(cammodz[points[0][0].y] / 10) >> 8) & 0xff;
					Y2_low = (int)(cammodz[points[0][1].y] / 10) & 0xff;
					Y2_high = ((int)(cammodz[points[0][1].y] / 10) >> 8) & 0xff;
					X2_low = (int)(cammodx[points[0][1].x + points[0][1].y * outputMat.cols] / 10) & 0xff;
					X2_high = ((int)(cammodx[points[0][1].x + points[0][1].y * outputMat.cols] / 10) >> 8) & 0xff;

				}
				else
				{
					X1_high = 255; X1_low = 255; Y1_high = 255; Y1_low = 255;
					X2_high = 255; X2_low = 255; Y2_high = 255; Y2_low = 255;
					continue;
				}
				cntframe++;
			}
			else
			{
				X1_high = 255; X1_low = 255; Y1_high = 255; Y1_low = 255;
				X2_high = 255; X2_low = 255; Y2_high = 255; Y2_low = 255;
			}
		}
	}
	if (calibflag)
	{
		cv::namedWindow("setPrkcalib", cv::WINDOW_NORMAL);
		cv::createTrackbar("spacelength", "setPrkcalib", &spacelength, 10000);
		cv::createTrackbar("spacewidth", "setPrkcalib", &spacewidth, 5000);
		cv::createTrackbar("linethres", "setPrkcalib", &linethres, 1000);
		calibflag = 0;
	}
	if (wflag)
	{
		static string label[] = { "spacelength", "spacewidth", "linethres" };
		static int para[] = { spacelength, spacewidth, linethres };
		FileWrite(const_cast<char *>("dPrkparameter.txt"), label, para, sizeof(para) / sizeof(para[0]));
		wflag = false;
	}
	if (rflag)
		radar(outputMat, cammodz, cammodx, target, boxcnt, vhctypecolor);
	cv::line(outputMat, cv::Point(outputMat.cols / 2, outputMat.rows * 0.75), cv::Point(outputMat.cols / 2, outputMat.rows - 1), cv::Scalar(255, 255, 255), 2, 8);
	cv::putText(outputMat, cv::String("Targets:" + std::to_string(boxcnt)), cv::Point(10, inputMat.rows - 10), 0, 0.6, cv::Scalar(0, 255, 255), 1.6, 8);
	cv::putText(outputMat, cv::String("cntframe:" + std::to_string(cntframe)), cv::Point(10, inputMat.rows - 30), 0, 0.6, cv::Scalar(0, 255, 255), 1.6, 8);
	cv::putText(outputMat, "Parking", cv::Point(10, 30), 0, 1, cv::Scalar(0, 255, 255), 2, 8);
	cv::putText(outputMat, "First line depth: " + std::to_string((int)cammodz[grounddoty[arrw / 2]]), cv::Point(10, 60), 0, 0.8, cv::Scalar(0, 255, 255), 2, 8);
}
void Grounddot::vehicledetect(const cv::Mat &inputMat, cv::Mat &outputMat, float *cammodz, float *cammodx, bool &calibflag, bool &wflag, bool &rflag)
{
	int boxcnt = 0;		//\ADp\BA\E2\A4\E8\B6\F4\BCƶq
	bool x1flag = false;//\ACO\A7_\A7\E4\A8\EC\A4\E8\B6\F4\B2Ĥ@\C2I \BAX\BC\D0
	bool x2flag = false;
	int firstpoint, lastpoint;
	int firstp;
	static int cntframe = 0;
	static cv::Point points[1][4];
	const cv::Point* ppt[1] = { points[0] };
	const int npt[] = { 4 };
	cv::Scalar vhctypecolor;
	memset(target, -1, 40 * sizeof(cv::Point));

	int X1_high = 0, X2_high = 0, Y1_high = 0, Y2_high = 0;
	int X1_low = 0, X2_low = 0, Y1_low = 0, Y2_low = 0;

	if (cntframe >= 10000)
		cntframe = 0;
	for (int i = 0; i < arrw; i++)
	{
		if (i > 0 && idata3x[i] != -1 && idata3y[i] != -1)
		{
			cv::line(outputMat, cv::Point(idata3x[i - 1], idata3y[i - 1]), cv::Point(idata3x[i], idata3y[i]), cv::Scalar(100, 255, 100), 2, 8);
			if (i > 2 && cammodz[idata3y[i]] <= 10000 && idata4y[i] - idata4y[i - 1] == 0 && cammodz[idata3y[i - 3]] - cammodz[idata3y[i - 1]] >= 1000)
			{
				if (!x1flag && !x2flag)
				{
					x1flag = 1;
					firstp = idata4y[i - 1];
					firstpoint = i - 1;
				}
			}
			else if (i > 2 && cammodz[idata3y[i]] <= 10000 && x1flag && !x2flag && idata4y[i] - idata4y[firstpoint] > 1)
			{
				x2flag = 1;
				lastpoint = i - 1;
			}
			else if (x1flag && !x2flag && abs(idata4y[i] - idata4y[firstpoint]) <= 1)
			{

			}
			else
			{
				x1flag = 0;
				x2flag = 0;
			}
			if (x1flag)
				cv::circle(outputMat, cv::Point(idata3x[firstpoint], idata3y[firstpoint]), 4, cv::Scalar(255, 255, 255), 2, 8);
			if (x1flag)
				cv::circle(outputMat, cv::Point(idata3x[firstpoint + 1], idata3y[firstpoint + 1]), 4, cv::Scalar(255, 128, 255), 2, 8);
			if (x2flag)
				cv::circle(outputMat, cv::Point(idata3x[lastpoint], idata3y[lastpoint]), 4, cv::Scalar(0, 255, 255), 2, 8);
			if (x2flag)
				cv::circle(outputMat, cv::Point(idata3x[lastpoint - 1], idata3y[lastpoint - 1]), 4, cv::Scalar(255, 0, 128), 2, 8);
			if (x1flag && x2flag)
			{
				x1flag = 0;
				x2flag = 0;
				int templ = (lastpoint - firstpoint) * gndspl;
				int widthcnt;

				if (templ >= buslength)
				{
					widthcnt = buswidth / gndspw;
					vhctypecolor = cv::Scalar(255, 255, 255);
					fprintf(dOBSlog, "bus\t(%d, %d)(%d, %d)\tframe:%d\n", points[0][0].x, points[0][0].y, points[0][1].x, points[0][1].y, cntframe);
				}
				else if (templ >= carlength)
				{
					widthcnt = carwidth / gndspw;
					vhctypecolor = cv::Scalar(50, 255, 255);
					fprintf(dOBSlog, "car\t(%d, %d)(%d, %d)\tframe:%d\n", points[0][0].x, points[0][0].y, points[0][1].x, points[0][1].y, cntframe);
				}
				else if (templ >= motorlength)
				{
					widthcnt = motorwidth / gndspw;
					vhctypecolor = cv::Scalar(50, 50, 255);
					fprintf(dOBSlog, "motor\t(%d, %d)(%d, %d)\tframe:%d\n", points[0][0].x, points[0][0].y, points[0][1].x, points[0][1].y, cntframe);
				}
				else
				{
					X1_high = 255; X1_low = 255; Y1_high = 255; Y1_low = 255;
					X2_high = 255; X2_low = 255; Y2_high = 255; Y2_low = 255;
					continue;
				}
				points[0][0] = cv::Point(idata3x[firstpoint], idata3y[firstpoint]);
				points[0][1] = cv::Point(idata3x[lastpoint], idata3y[lastpoint]);
				points[0][2] = cv::Point((grounddotx[idata4x[lastpoint] + ((idata4y[lastpoint] + widthcnt >= arrh) ? idata4y[lastpoint] : idata4y[lastpoint] + widthcnt) * arrw]), (grounddoty[idata4x[lastpoint] + ((idata4y[lastpoint] + widthcnt >= arrh) ? idata4y[lastpoint] : idata4y[lastpoint] + widthcnt) * arrw]));
				points[0][3] = cv::Point((grounddotx[idata4x[firstpoint] + ((idata4y[firstpoint] + widthcnt >= arrh) ? idata4y[firstpoint] : idata4y[firstpoint] + widthcnt) * arrw]), (grounddoty[idata4x[firstpoint] + ((idata4y[firstpoint] + widthcnt >= arrh) ? idata4y[firstpoint] : idata4y[firstpoint] + widthcnt) * arrw]));
				//points[0][2] = cv::Point((grounddotx[idata4x[lastpoint] + (idata4y[lastpoint] + widthcnt) * arrw] == -1) ? idata3x[lastpoint] : grounddotx[idata4x[lastpoint] + (idata4y[lastpoint] + widthcnt) * arrw], (grounddoty[idata4x[lastpoint] + (idata4y[lastpoint] + widthcnt) * arrw] == -1) ? idata3y[lastpoint] : grounddoty[idata4x[lastpoint] + (idata4y[lastpoint] + widthcnt) * arrw]);
				//points[0][3] = cv::Point((grounddotx[idata4x[firstpoint] + (idata4y[firstpoint] + widthcnt) * arrw] == -1) ? idata3x[firstpoint] : grounddotx[idata4x[firstpoint] + (idata4y[firstpoint] + widthcnt) * arrw], (grounddoty[idata4x[firstpoint] + (idata4y[firstpoint] + widthcnt) * arrw] == -1) ? idata3y[firstpoint] : grounddoty[idata4x[firstpoint] + (idata4y[firstpoint] + widthcnt) * arrw]);
				cv::polylines(outputMat, ppt, npt, 1, 1, vhctypecolor, 5);
				cv::putText(outputMat, "x: " + std::to_string((int)cammodx[points[0][0].x + points[0][0].y * outputMat.cols]), cv::Point(points[0][0].x, points[0][0].y + 10), 0, 0.8, cv::Scalar(50, 255, 255), 2, 8);
				cv::putText(outputMat, "x: " + std::to_string((int)cammodx[points[0][1].x + points[0][1].y * outputMat.cols]), cv::Point(points[0][1].x, points[0][1].y + 10), 0, 0.8, cv::Scalar(50, 255, 255), 2, 8);
				cv::putText(outputMat, "y: " + std::to_string((int)cammodz[points[0][0].y]), cv::Point(points[0][0].x, points[0][0].y + 30), 0, 0.8, cv::Scalar(0, 120, 200), 2, 8);
				cv::putText(outputMat, "y: " + std::to_string((int)cammodz[points[0][1].y]), cv::Point(points[0][1].x, points[0][1].y + 30), 0, 0.8, cv::Scalar(0, 120, 200), 2, 8);
				target[boxcnt * 2] = points[0][0];
				target[boxcnt * 2 + 1] = points[0][2];
				boxcnt++;
				cntframe++;

				X1_low = (int)(cammodx[points[0][0].x + points[0][0].y * outputMat.cols] / 10) & 0xff;
				X1_high = ((int)(cammodx[points[0][0].x + points[0][0].y * outputMat.cols] / 10) >> 8) & 0xff;
				Y1_low = (int)(cammodz[points[0][0].y] / 10) & 0xff;
				Y1_high = ((int)(cammodz[points[0][0].y] / 10) >> 8) & 0xff;
				Y2_low = (int)(cammodz[points[0][1].y] / 10) & 0xff;
				Y2_high = ((int)(cammodz[points[0][1].y] / 10) >> 8) & 0xff;
				X2_low = (int)(cammodx[points[0][1].x + points[0][1].y * outputMat.cols] / 10) & 0xff;
				X2_high = ((int)(cammodx[points[0][1].x + points[0][1].y * outputMat.cols] / 10) >> 8) & 0xff;

			}
			else
			{
				X1_high = 255; X1_low = 255; Y1_high = 255; Y1_low = 255;
				X2_high = 255; X2_low = 255; Y2_high = 255; Y2_low = 255;
			}
		}
	}
	if (calibflag)
	{
		cv::namedWindow("setVhicalib", cv::WINDOW_NORMAL);
		cv::createTrackbar("buslength", "setVhicalib", &buslength, 10000);
		cv::createTrackbar("carlength", "setVhicalib", &carlength, 5000);
		cv::createTrackbar("motorlength", "setVhicalib", &motorlength, 3000);
		calibflag = 0;
	}
	if (wflag)
	{
		static string label[] = { "buslength", "carlength", "motorlength" };
		static int para[] = { buslength, carlength, motorlength };
		FileWrite(const_cast<char *>("dVhcparameter.txt"), label, para, sizeof(para) / sizeof(para[0]));
		wflag = false;
	}
	if (rflag)
		radar(outputMat, cammodz, cammodx, target, boxcnt, vhctypecolor);
	cv::line(outputMat, cv::Point(outputMat.cols / 2, outputMat.rows * 0.75), cv::Point(outputMat.cols / 2, outputMat.rows - 1), cv::Scalar(255, 255, 255), 2, 8);
	cv::putText(outputMat, cv::String("Targets:" + std::to_string(boxcnt)), cv::Point(10, inputMat.rows - 10), 0, 0.6, cv::Scalar(0, 255, 255), 1.6, 8);
	cv::putText(outputMat, cv::String("cntframe:" + std::to_string(cntframe)), cv::Point(10, inputMat.rows - 30), 0, 0.6, cv::Scalar(0, 255, 255), 1.6, 8);
	cv::putText(outputMat, "Vehicle", cv::Point(10, 30), 0, 1, cv::Scalar(0, 255, 255), 2, 8);
	cv::putText(outputMat, "First line depth: " + std::to_string((int)cammodz[grounddoty[arrw / 2]]), cv::Point(10, 60), 0, 0.8, cv::Scalar(0, 255, 255), 2, 8);
}
void Grounddot::gnddotline(const cv::Mat &inputMat, cv::Mat &outputMat, float *cammodz, float *cammodx, bool &obsprk, bool &calibflag, bool &wflag, bool &rflag)
{
	int filterW = 1000;
	int cnt = 0;
	static bool data4flag = 1;//0 : green | 1 : yellow
	//cv::Point *idata3 = new cv::Point(arrw);
	//cv::Point *idata4 = new cv::Point(arrw);
	int idatatempx = -1;
	int idatatempy = -1;
	for (int i = 0; i < arrw; i++)
	{
		idata3x[i] = -1;
		idata3y[i] = -1;
		idata4x[i] = -1;
		idata4y[i] = -1;
	}

	float localNearestDepth = 100000;

	for (int i = 0; i < arrw; i++)
	{
		bool Yflag = false;
		for (int j = 0; j < arrh; j++)
		{
			if (grounddotx[i + j * arrw] == -1 && grounddoty[i + j * arrw] == -1)
				continue;
			if (cammodz[grounddoty[i + j * arrw]] - inputMat.at<float>(grounddoty[i + j * arrw], grounddotx[i + j * arrw]) > linethres)// || inputMat.at<float>(obsgroundcammodx[i][j].y, obsgroundcammodx[i][j].x) > (cammod[obsgroundcammodx[i][j].y] * 1.6)
			{
				if ((cammodz[(grounddoty[i + j * arrw] - 30 < 0) ? grounddoty[i + j * arrw] : (grounddoty[i + j * arrw] - 30)] - inputMat.at<float>((grounddoty[i + j * arrw] - 30 < 0) ? grounddoty[i + j * arrw] : (grounddoty[i + j * arrw] - 30), grounddotx[i + j * arrw])) > linethres)// || inputMat.at<float>(obsgroundcammodx[i][j].y - 30, obsgroundcammodx[i][j].x) > (cammod[obsgroundcammodx[i][j].y - 30] * 1.6)
				{
					if (!Yflag)
					{
						if (!data4flag)
						{
							//idata3[cnt] = cv::Point(grounddotx[i + j * arrw], grounddoty[i + j * arrw]);
							idata3x[cnt] = grounddotx[i + j * arrw];
							idata3y[cnt] = grounddoty[i + j * arrw];
							idata4x[cnt] = i;
							idata4y[cnt] = j;
						}
						else
						{
							/*idata3[cnt] = (grounddotx[i + ((j * gndspw) / filterW) * (filterW / gndspw) * arrw] == -1 && grounddoty[i + ((j * gndspw) / filterW) * (filterW / gndspw) * arrw] == -1)
							? cv::Point(grounddotx[i + j * arrw], grounddoty[i + j * arrw])
							: cv::Point(grounddotx[i + ((j * gndspw) / filterW) * (filterW / gndspw) * arrw], grounddoty[i + ((j * gndspw) / filterW) * (filterW / gndspw) * arrw]);
							idata4[cnt] = (grounddotx[i + ((j * gndspw) / filterW) * (filterW / gndspw) * arrw] == -1 && grounddoty[i + ((j * gndspw) / filterW) * (filterW / gndspw) * arrw] == -1)
							? cv::Point(i, j)
							: cv::Point(i, ((j * gndspw) / filterW) * (filterW / gndspw));*/
							idata3x[cnt] = (grounddotx[i + ((j * gndspw) / filterW) * (filterW / gndspw) * arrw] == -1 && grounddoty[i + ((j * gndspw) / filterW) * (filterW / gndspw) * arrw] == -1)
								? grounddotx[i + j * arrw] : grounddotx[i + ((j * gndspw) / filterW) * (filterW / gndspw) * arrw];
							idata3y[cnt] = (grounddotx[i + ((j * gndspw) / filterW) * (filterW / gndspw) * arrw] == -1 && grounddoty[i + ((j * gndspw) / filterW) * (filterW / gndspw) * arrw] == -1)
								? grounddoty[i + j * arrw] : grounddoty[i + ((j * gndspw) / filterW) * (filterW / gndspw) * arrw];
							idata4x[cnt] = i;
							idata4y[cnt] = (grounddotx[i + ((j * gndspw) / filterW) * (filterW / gndspw) * arrw] == -1 && grounddoty[i + ((j * gndspw) / filterW) * (filterW / gndspw) * arrw] == -1)
								? j : ((j * gndspw) / filterW) * (filterW / gndspw);
						}
						Yflag = 1;
						cnt++;
						//=======

						localNearestDepth = localNearestDepth > cammodz[grounddoty[i + j * arrw]] ? cammodz[grounddoty[i + j * arrw]] : localNearestDepth;

						if (idatatempx != 1 && idatatempy != -1)
							cv::line(outputMat, cv::Point(idatatempx, idatatempy), cv::Point(grounddotx[i + j * arrw], grounddoty[i + j * arrw]), cv::Scalar(255, 255, 0), 2, 8);
						idatatempx = grounddotx[i + j * arrw];
						idatatempy = grounddoty[i + j * arrw];
						cv::circle(outputMat, cv::Point(grounddotx[i + j * arrw], grounddoty[i + j * arrw]), 1, cv::Scalar(100, 190, 190), -1, 8);
						cv::circle(outputMat, cv::Point(grounddotx[i + ((j * gndspw) / filterW) * (filterW / gndspw) * arrw], grounddoty[i + ((j * gndspw) / filterW) * (filterW / gndspw) * arrw]), 1, cv::Scalar(50, 190, 190), -1, 8);
					}
				}
				else
					if (!Yflag)
						cv::circle(outputMat, cv::Point(grounddotx[i + j * arrw], grounddoty[i + j * arrw]), 1, cv::Scalar(0, 0, 255), -1, 8);
			}
			else
				if (j != arrh - 1 && !Yflag)
					cv::circle(outputMat, cv::Point(grounddotx[i + j * arrw], grounddoty[i + j * arrw]), 1, cv::Scalar(0, 190, 0), -1, 8);
				else if (j == arrh - 1 && !Yflag)
				{
					cv::circle(outputMat, cv::Point(grounddotx[i + j * arrw], grounddoty[i + j * arrw]), 1, cv::Scalar(50, 190, 190), -1, 8);
					if (!Yflag)
					{
						if (!data4flag)
						{
							idata3x[cnt] = grounddotx[i + j * arrw];
							idata3y[cnt] = grounddoty[i + j * arrw];
							idata4x[cnt] = i;
							idata4y[cnt] = j;
						}
						else
						{
							idata3x[cnt] = (grounddotx[i + ((j * gndspw) / filterW) * (filterW / gndspw) * arrw] == -1 && grounddoty[i + ((j * gndspw) / filterW) * (filterW / gndspw) * arrw] == -1)
								? grounddotx[i + j * arrw] : grounddotx[i + ((j * gndspw) / filterW) * (filterW / gndspw) * arrw];
							idata3y[cnt] = (grounddotx[i + ((j * gndspw) / filterW) * (filterW / gndspw) * arrw] == -1 && grounddoty[i + ((j * gndspw) / filterW) * (filterW / gndspw) * arrw] == -1)
								? grounddoty[i + j * arrw] : grounddoty[i + ((j * gndspw) / filterW) * (filterW / gndspw) * arrw];
							idata4x[cnt] = i;
							idata4y[cnt] = (grounddotx[i + ((j * gndspw) / filterW) * (filterW / gndspw) * arrw] == -1 && grounddoty[i + ((j * gndspw) / filterW) * (filterW / gndspw) * arrw] == -1)
								? j : ((j * gndspw) / filterW) * (filterW / gndspw);
						}

						localNearestDepth = localNearestDepth > cammodz[grounddoty[i + j * arrw]] ? cammodz[grounddoty[i + j * arrw]] : localNearestDepth;

						if (idatatempx != 1 && idatatempy != -1)
							cv::line(outputMat, cv::Point(idatatempx, idatatempy), cv::Point(grounddotx[i + j * arrw], grounddoty[i + j * arrw]), cv::Scalar(255, 255, 0), 2, 8);
						idatatempx = grounddotx[i + j * arrw];
						idatatempy = grounddoty[i + j * arrw];
						Yflag = 1;
						cnt++;
					}
				}
		}
		//cv::circle(outputMat, obsgroundcammodx[i][11], 3, cv::Scalar(255, 255, 0), 1, 8);
	}

	std::unique_lock<std::mutex> nearestDepthLocker(this->nearestDepthLock_, std::defer_lock);
	nearestDepthLocker.lock();
	this->nearestDepth_ = localNearestDepth;
	nearestDepthLocker.unlock();

	if (obsprk)
		parkinglotdetect(inputMat, outputMat, cammodz, cammodx, calibflag, wflag, rflag);
	else
		vehicledetect(inputMat, outputMat, cammodz, cammodx, calibflag, wflag, rflag);
}

void Grounddot::sortindex()
{
	int *tempmodx = new int[arrw * arrh];
	int *tempmody = new int[arrw * arrh];
	int cnt = 0;
	for (int i = arrw - 1; i >= 0; i -= 2)
	{
		for (int j = 0; j < arrh; j++)
		{
			tempmodx[j * arrw + cnt] = this->grounddotx[j * arrw + i];
			tempmody[j * arrw + cnt] = this->grounddoty[j * arrw + i];
		}
		cnt++;
	}
	for (int i = 1; i < arrw; i += 2)
	{
		for (int j = 0; j < arrh; j++)
		{
			tempmodx[j * arrw + cnt] = this->grounddotx[j * arrw + i];
			tempmody[j * arrw + cnt] = this->grounddoty[j * arrw + i];
		}
		cnt++;
	}
	for (int i = 0; i < arrw; i++)
		for (int j = 0; j < arrh; j++)
		{
			this->grounddotx[j * arrw + i] = tempmodx[j * arrw + i];
			this->grounddoty[j * arrw + i] = tempmody[j * arrw + i];
		}
	delete[]tempmodx;
	delete[]tempmody;
}
void Grounddot::grounddotSearching(cv::Mat &outputMat, float *cammodz, float *cammodx, bool &wflag)
{
	float templ;
	float tempw;
	int iterl = 0, iterw = 0;
	int diff1 = 10;

	tempw = cammodz[outputMat.rows - 1];
	iterw += gndspw;
	int cntx = 0;
	int cnty = 0;
	int arrw = 0;
	int arrh = 0;
	for (int j = outputMat.rows - 1; j >= 0; j--)
	{
		if (tempw + iterw >= 10000)
			diff1 = 200;
		else if (tempw + iterw >= 8000)
			diff1 = 120;
		else if (tempw + iterw >= 7000)
			diff1 = 90;
		else if (tempw + iterw >= 5000)
			diff1 = 60;
		else if (tempw + iterw >= 4000)
			diff1 = 42;
		else if (tempw + iterw >= 3000)
			diff1 = 34;
		else if (tempw + iterw >= 2500)
			diff1 = 18;
		else
			diff1 = 12;
		if (abs(cammodz[j] - (tempw + iterw)) <= diff1)
		{
			iterw += gndspw;
			templ = cammodx[j * outputMat.cols + outputMat.cols / 2];
			cntx = 0;
			iterl = 0;
			for (int i = outputMat.cols / 2; i < outputMat.cols; i++)
			{
				if (abs(cammodx[j * outputMat.cols + i] - (templ + iterl)) <= diff1)
				{
					iterl += gndspl;
					cntx += 2;
					arrw = (arrw >= cntx) ? arrw : cntx;
				}
			}
			cnty++;
		}
	}
	arrw++;
	arrh = cnty;
	this->arrh = arrh;
	this->arrw = arrw;

	this->grounddotx = new int[arrw * arrh];
	this->grounddoty = new int[arrw * arrh];
	this->idata3x = new int[arrw];
	this->idata3y = new int[arrw];
	this->idata4x = new int[arrw];
	this->idata4y = new int[arrw];
	for (int i = 0; i < arrw * arrh; i++)
	{
		this->grounddotx[i] = -1;
		this->grounddoty[i] = -1;
	}
	cntx = 0;
	cnty = 0;
	iterl = 0;
	iterw = 0;
	tempw = cammodz[outputMat.rows - 1];
	iterw += gndspw;
	for (int j = outputMat.rows - 1; j >= 0; j--)
	{
		if (tempw + iterw >= 10000)
			diff1 = 200;
		else if (tempw + iterw >= 8000)
			diff1 = 120;
		else if (tempw + iterw >= 7000)
			diff1 = 90;
		else if (tempw + iterw >= 5000)
			diff1 = 60;
		else if (tempw + iterw >= 4000)
			diff1 = 42;
		else if (tempw + iterw >= 3000)
			diff1 = 34;
		else if (tempw + iterw >= 2500)
			diff1 = 18;
		else
			diff1 = 12;
		if (abs(cammodz[j] - (tempw + iterw)) <= diff1)
		{
			this->grounddotx[cnty * arrw + 0] = outputMat.cols / 2;
			this->grounddoty[cnty * arrw + 0] = j;
			iterw += gndspw;
			templ = cammodx[j * outputMat.cols + outputMat.cols / 2];
			cntx = 0;
			iterl = 0;
			for (int i = outputMat.cols / 2; i < outputMat.cols; i++)
			{
				if (abs(cammodx[j * outputMat.cols + i] - (templ + iterl)) <= diff1)
				{
					cntx++;
					this->grounddotx[cnty * arrw + cntx] = i;
					this->grounddoty[cnty * arrw + cntx] = j;
					cntx++;
					this->grounddotx[cnty * arrw + cntx] = outputMat.cols - i;
					this->grounddoty[cnty * arrw + cntx] = j;
					iterl += gndspl;
				}
			}
			cnty++;
		}
	}
	Grounddot::sortindex();
	cv::namedWindow("setGnddot", cv::WINDOW_NORMAL);
	cv::createTrackbar("gndspl", "setGnddot", &gndspl, 2000);
	cv::createTrackbar("gndspw", "setGnddot", &gndspw, 2000);
	if (wflag)
	{
		string label[] = { "gndspl", "gndspw" };
		int para[] = { gndspl, gndspw };
		FileWrite(const_cast<char *>("Gnddotparameter.txt"), label, para, sizeof(para) / sizeof(para[0]));
		wflag = false;
	}
	for (int i = 0; i < arrw; i++)
		for (int j = 0; j < arrh; j++)
			if (cv::Point(this->grounddotx[j * arrw + i], this->grounddoty[j * arrw + i]) != cv::Point(-1, -1))
				cv::circle(outputMat, cv::Point(this->grounddotx[j * arrw + i], this->grounddoty[j * arrw + i]), 3, distancecolor(i, arrw + 1), 1, 8);
}

float Grounddot::getNearestDepth()
{
	const std::lock_guard<std::mutex> lock(this->nearestDepthLock_);
	return this->nearestDepth_;
}