#include "re_tx1.h"

using namespace std;
Cameramodule::Cameramodule(int w, int h)
{
	this->cammodz = new float[h];
	this->cammodx = new float[w * h];
	this->halfcol = w / 2;
	this->halfrow = h / 2;
}
Cameramodule::~Cameramodule()
{
	delete cammodz;
	delete cammodx;
}
void Cameramodule::calibdeg(cv::Mat &outputMat)
{
	const float todeg = 0.017453;
	int dw = 6;
	float temp;
	for (int deg = 0; deg <= 60; deg += 10)
	{
		for (int k = 1000; k <= 5000; k += 10)
		{
			for (int j = 0; j < outputMat.rows; j++)
			{
				if (abs(cammodz[j] - k) <= dw)
				{
					temp = cammodz[j] * tan(deg * todeg);
					for (int i = 0; i < outputMat.cols; i++)
					{
						if (abs(cammodx[j * outputMat.cols + i] - temp) <= dw)
							cv::circle(outputMat, cv::Point(i, j), 0.3, distancecolor(deg, 60), -1, 8);
					}
				}
			}
		}
	}
}
void Cameramodule::cammodset(cv::Mat &outputMat, bool &wflag)
{
	int d1 = 1200;
	int d2 = 2000;
	int d3 = 2500;
	int d4 = 3000;
	int d5 = 4000;
	int d6 = 5000;
	int dw = 20;//�u�e
	for (int u = outputMat.cols / 4; u < outputMat.cols / 2; u++)
		for (int v = 0; v < outputMat.rows; v++)
		{
			cammodz[v] = ((c_h * (c_f * cos(c_theta * todeg) + (c_z * (halfrow - v) / halfrow) * sin(c_theta * todeg))) / (c_f * sin(c_theta * todeg)
				- (c_z * (halfrow - v) / halfrow) * cos(c_theta * todeg))) - c_f * cos(c_theta * todeg);
			if (cammodz[v] >= d1 && cammodz[v] < d1 + dw)
			{
				cv::circle(outputMat, cv::Point(u, v), 2, cv::Scalar(0, 0, 255), -1);
				cv::putText(outputMat, cv::String(std::to_string(d1)), cv::Point(50, v + 10), 0, 0.8, cv::Scalar(0, 0, 255), 1);
			}
			else if (cammodz[v] >= d2 && cammodz[v] < d2 + dw)
			{
				cv::circle(outputMat, cv::Point(u, v), 2, cv::Scalar(0, 128, 255), -1);
				cv::putText(outputMat, cv::String(std::to_string(d2)), cv::Point(50, v + 10), 0, 0.8, cv::Scalar(0, 128, 255), 1);
			}
			else if (cammodz[v] >= d3 && cammodz[v] < d3 + dw)
			{
				cv::circle(outputMat, cv::Point(u, v), 2, cv::Scalar(0, 255, 255), -1);
				cv::putText(outputMat, cv::String(std::to_string(d3)), cv::Point(50, v + 10), 0, 0.8, cv::Scalar(0, 255, 255), 1);
			}
			else if (cammodz[v] >= d4 && cammodz[v] < d4 + dw)
			{
				cv::circle(outputMat, cv::Point(u, v), 2, cv::Scalar(0, 255, 0), -1);
				cv::putText(outputMat, cv::String(std::to_string(d4)), cv::Point(50, v + 10), 0, 0.8, cv::Scalar(0, 255, 0), 1);
			}
			else if (cammodz[v] >= d5 && cammodz[v] < d5 + dw)
			{
				cv::circle(outputMat, cv::Point(u, v), 2, cv::Scalar(255, 255, 0), -1);
				cv::putText(outputMat, cv::String(std::to_string(d5)), cv::Point(50, v + 10), 0, 0.8, cv::Scalar(255, 255, 0), 1);
			}
			else if (cammodz[v] >= d6 && cammodz[v] < d6 + dw)
			{
				cv::circle(outputMat, cv::Point(u, v), 2, cv::Scalar(255, 0, 0), -1);
				cv::putText(outputMat, cv::String(std::to_string(d6)), cv::Point(50, v + 10), 0, 0.8, cv::Scalar(255, 0, 0), 1);
			}
		}
	int dx1 = 0;
	int dx2 = 400;
	int dx3 = 800;
	int dx4 = 1200;
	int dx5 = 1600;
	int dx6 = 2000;
	int dxw = 5;//�u�e
	for (int i = 0; i < outputMat.cols; i++)
	{
		for (int j = 0; j < outputMat.rows; j++)
		{
			cammodx[j * outputMat.cols + i] = (c_h / (c_f * sin(c_theta * todeg) - (c_z * (halfrow - j) / halfrow) * cos(c_theta * todeg))) * (c_x * (i - halfcol) / halfcol);
			if (abs(cammodx[j * outputMat.cols + i]) >= dx1 && abs(cammodx[j * outputMat.cols + i]) < dx1 + dxw)
				cv::circle(outputMat, cv::Point(i, j), 1, cv::Scalar(0, 0, 255), -1);
			else if (abs(cammodx[j * outputMat.cols + i]) >= dx2 && abs(cammodx[j * outputMat.cols + i]) < dx2 + dxw)
				cv::circle(outputMat, cv::Point(i, j), 1, cv::Scalar(0, 128, 255), -1);
			else if (abs(cammodx[j * outputMat.cols + i]) >= dx3 && abs(cammodx[j * outputMat.cols + i]) < dx3 + dxw)
				cv::circle(outputMat, cv::Point(i, j), 1, cv::Scalar(0, 255, 255), -1);
			else if (abs(cammodx[j * outputMat.cols + i]) >= dx4 && abs(cammodx[j * outputMat.cols + i]) < dx4 + dxw)
				cv::circle(outputMat, cv::Point(i, j), 1, cv::Scalar(128, 255, 255), -1);
			else if (abs(cammodx[j * outputMat.cols + i]) >= dx5 && abs(cammodx[j * outputMat.cols + i]) < dx5 + dxw)
				cv::circle(outputMat, cv::Point(i, j), 1, cv::Scalar(255, 255, 255), -1);
			else if (abs(cammodx[j * outputMat.cols + i]) >= dx6 && abs(cammodx[j * outputMat.cols + i]) < dx6 + dxw)
				cv::circle(outputMat, cv::Point(i, j), 1, cv::Scalar(255, 128, 255), -1);
		}
	}
	calibdeg(outputMat);
	if (wflag)
	{
		string label[] = { "c_theta", "c_h" };
		int para[] = { c_theta, c_h };
		FileWrite(const_cast<char *>("camfile.txt"), label, para, sizeof(para) / sizeof(para[0]));
		wflag = false;
	}
	cv::namedWindow("setCamcalib", cv::WINDOW_NORMAL);
	cv::createTrackbar("height", "setCamcalib", &c_h, 2000);
	cv::createTrackbar("theta", "setCamcalib", &c_theta, 90);
}