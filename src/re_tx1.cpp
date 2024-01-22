#include "imageProc.h"
#include "re_tx1.h"


void FileWrite(char *fn, std::string label[], int *data, int dl)
{
	FILE *fw;
	fw = fopen(fn, "w");
	std::cout << "Writting file..." << fn << std::endl;
	for (int i = 0; i < dl; i++)
	{
		fprintf(fw, "%d\n", data[i]);
		std::cout << label[i] <<  ": " << data[i] << std::endl;
	}
	std::cout << "Complete.\n" << std::endl;
	fclose(fw);
}
int FileRead(char *fn, std::string label[], int *data, int dl)
{
	FILE *fr;
	fr = fopen(fn, "r");
	if (fr == NULL)
		return 0;
	std::cout << "Reading file..." << fn << std::endl;
	for (int i = 0; i < dl; i++)
	{
		fscanf(fr, "%d", &data[i]);
		std::cout << label[i] << ": " << data[i] << std::endl;
	}
	std::cout << "Complete.\n" << std::endl;
	fclose(fr);
	return 1;
}

cv::Scalar distancecolor(int distance, int max)
{
	int x1, x2, x3, x4;
	int r = 0, g = 0, b = 0;
	x1 = max * 0.2;
	x2 = max * 0.4;
	x3 = max * 0.6;
	x4 = max * 0.8;
	if (distance >= 0 && distance < x1)
	{
		r = 255;
		g = (distance * 255) / x1;
		b = 0;
	}
	else if (distance >= x1 && distance < x2)
	{
		g = 255;
		r = -(distance * 255) / (x2 - x1) + 255;
		b = 0;
	}
	else if (distance >= x2 && distance < x3)
	{
		g = 255;
		b = (distance * 255) / (x3 - x2);
		r = 0;
	}
	else if (distance >= x3 && distance < x4)
	{
		b = 255;
		g = -(distance * 255) / (x4 - x3) + 255;
		r = 0;
	}
	else if (distance >= x4)
	{
		b = 255;
		r = (distance * 255) / (x4 - x3);
		g = 0;
	}
	return cv::Scalar(b, g, r);
}
void radar(cv::Mat &outputMat, float *cammodz, float *cammodx, cv::Point target[], int boxcnt, cv::Scalar color)
{
	cv::Mat radarMat(outputMat.rows, outputMat.cols * 0.4, CV_8UC3, cv::Scalar(128, 128, 64));
	static cv::Mat car = cv::imread("car.png");
	cv::Mat carROI = radarMat(cv::Rect(0, radarMat.rows / 2 - car.rows / 2, car.cols, car.rows));
	cv::addWeighted(carROI, 0.5, car, 0.5, 0, carROI);
	static int Xo = radarMat.rows / 2;
	static int Yo = car.cols;
	int x1 = 0;
	int x2 = 0;
	int y1 = 0;
	int y2 = 0;
	if (boxcnt > 0)
		for (int i = 0; i < boxcnt; i++)
		{
			x1 = cammodx[target[i * 2].x + target[i * 2].y * outputMat.cols] / 40;
			x2 = cammodx[target[i * 2 + 1].x + target[i * 2].y * outputMat.cols] / 40;
			y1 = cammodz[target[i * 2].y] / 40;
			y2 = cammodz[target[i * 2 + 1].y] / 40;
			cv::rectangle(radarMat, cv::Rect(cv::Point(Yo + y1, Xo + x1), cv::Point(Yo + y2, Xo + x2)), color, -1);
			cv::putText(radarMat, cv::String(std::to_string(i + 1)), cv::Point(Yo + y1, Xo + x1), 0, 0.6, cv::Scalar(0, 255, 255), 1.6, 8);
		}
	cv::imshow("radar", radarMat);
}

/**
 * Entry Point
 */
int main(int argc, char **argv)
{
	cv::Mat depthmat;
	cv::Mat rgbmat_left;
	int width = 640;// 1280
	int height = 360;// 720
	
	int selectfunc = 1;//1:ground | 2:calib
	bool obsprk = 1;//0:obs | 1:prk
	bool camgnd = 0;//0:cam | 1:gnd
	bool dotlineflag = 0;//0:gnddot | 1:gndline
	bool resizeflag = true;
	bool radarflag = false;
	bool Wflag = false;
	bool calibflag = false;
	bool camgndfirstrun = true;

	Cameramodule cammod(width, height);
	Groundline gndline(width, height);
	Grounddot gnddot;

	int datalength = 2;
	std::string *label = new std::string[datalength];
	int *para = new int[datalength];
	label[0] = "c_theta";
	label[1] = "c_h";
	para[0] = cammod.c_theta;
	para[1] = cammod.c_h;
	if (!FileRead(const_cast<char *>("camfile.txt"), label, para, datalength))
		std::cout << "ERROR open " << "camfile.txt" << " Init..." << std::endl;
	else
	{
		cammod.c_theta = para[0];
		cammod.c_h = para[1];
	}

	label[0] = "gndspl";
	label[1] = "gndspw";
	para[0] = gnddot.gndspl;
	para[1] = gnddot.gndspw;
	if (!FileRead(const_cast<char *>("Gnddotparameter.txt"), label, para, datalength))
		std::cout << "ERROR open " << "Gnddotparameter.txt" << " Init..." << std::endl;
	else
	{
		gnddot.gndspl = para[0];
		gnddot.gndspw = para[1];
	}
	delete[] label;
	delete para;

	datalength = 3;
	label = new std::string[datalength];
	para = new int[datalength];
	label[0] = "spacelength";
	label[1] = "spacewidth";
	label[2] = "linethres";
	para[0] = gnddot.spacelength;
	para[1] = gnddot.spacewidth;
	para[2] = gnddot.linethres;
	if (!FileRead(const_cast<char *>("dPrkparameter.txt"), label, para, datalength))
		std::cout << "ERROR open " << "dPrkparameter.txt" << " Init..." << std::endl;
	else
	{
		gnddot.spacelength = para[0];
		gnddot.spacewidth = para[1];
		gnddot.linethres = para[2];
		gndline.camthres = para[2];
	}

	label[0] = "buslength";
	label[1] = "carlength";
	label[2] = "motorlength";
	para[0] = gnddot.buslength;
	para[1] = gnddot.carlength;
	para[2] = gnddot.motorlength;
	if (!FileRead(const_cast<char *>("dVhcparameter.txt"), label, para, datalength))
		std::cout << "ERROR open " << "dVhcparameter.txt" << " Init..." << std::endl;
	else
	{
		gnddot.buslength = para[0];
		gnddot.carlength = para[1];
		gnddot.motorlength = para[2];
	}
	delete[] label;
	delete para;


	rclcpp::init(argc, argv);
	auto params = std::make_shared<Params>("grounddetect_params_node");
	ZEDNode zedSub(params);
	
	bool newRGBMatF, newDepthMatF;
	newRGBMatF = false;
	newDepthMatF = false;

	MultiWorkingRate workingRate(1000, 2);// [0]: working rate, [1]: fps
	workingRate.start();
	std::this_thread::sleep_for(1s);

	// ImagePublisher (TODO: refine Image.msg)
	auto imgPub = std::make_shared<GroundDetectPublisher>(params);
	cv::Size pubImgSize(params->mainCameraWidth, params->mainCameraHeight);

    std::vector<int> encodeParam;
    encodeParam.push_back(cv::IMWRITE_JPEG_QUALITY);
    encodeParam.push_back(70);
    std::vector<uchar> pubImgVec;

	/**
	 * Main Loop
	 */
	while (1)
	{
		cv::Mat grabRGBMat, grabDepthMat;
		zedSub.getRGBImage(grabRGBMat, newRGBMatF);
		zedSub.getDepthImage(grabDepthMat, newDepthMatF);
		rgbmat_left = grabRGBMat.clone();
		depthmat = grabDepthMat.clone();

		if (newDepthMatF)
			workingRate.addOneCnt(1);
		workingRate.addOneCnt(0);

		if (camgndfirstrun)
		{
			cammod.cammodset(rgbmat_left, Wflag);
			cv::destroyAllWindows();
			gnddot.grounddotSearching(rgbmat_left, cammod.cammodz, cammod.cammodx, Wflag);
			cv::destroyAllWindows();
			camgndfirstrun = 0;
		}
		
		int k = cv::waitKey(10) % 256;
		if (k == 27)
			goto END;
		else if (k == 'c' || k == 'C')
		{
			cv::destroyAllWindows();
			selectfunc = 2;
			camgnd = !camgnd;
			resizeflag = 1;
		}
		else if (k == 'o' || k == 'O')
		{
			cv::destroyAllWindows();
			selectfunc = 1;
			obsprk = 0;
			dotlineflag = !dotlineflag;
			resizeflag = 1;
		}
		else if (k == 'p' || k == 'P')
		{
			cv::destroyAllWindows();
			selectfunc = 1;
			obsprk = 1;
			dotlineflag = !dotlineflag;
			resizeflag = 1;
		}
		else if (k == 'q' || k == 'Q')
		{
			calibflag = true;
		}
		else if (k == 'r' || k == 'R')
		{
			radarflag = !radarflag;
			if (!radarflag)
				cv::destroyWindow("radar");
			resizeflag = 1;
		}
		else if (k == 's' || k == 'S')
		{
			Wflag = true;
		}

		if (selectfunc == 1)
		{
			if (dotlineflag)
				gndline.ground_detect(depthmat, rgbmat_left, cammod.cammodz, cammod.cammodx, obsprk, calibflag, Wflag, radarflag);
			else
			{
				gnddot.gnddotline(depthmat, rgbmat_left, cammod.cammodz, cammod.cammodx, obsprk, calibflag, Wflag, radarflag);
				float nDist = gnddot.getNearestDepth();
				float em = 0.0;
				if (nDist > params->safetyDistance)
					em = 0.0;
				else
					em = 1.0;
				zedSub.setEmergency(params->nodeName, em, (vehicle_interfaces::EmergencyScoreDirection)params->safetyDirection);
			}
		}
		else if (selectfunc == 2)
		{
			if (camgnd == 0)
				cammod.cammodset(rgbmat_left, Wflag);
			else if (camgnd == 1)
				gnddot.grounddotSearching(rgbmat_left, cammod.cammodz, cammod.cammodx, Wflag);
		}
		std::vector<float> rateVec = workingRate.getRate();
		cv::putText(rgbmat_left, std::to_string((int)rateVec[1]) + "fps", cv::Point(10, height - 70), 0, 1, cv::Scalar(0, 255, 255), 2, 8);
		cv::putText(rgbmat_left, std::to_string((int)rateVec[0]) + "Hz", cv::Point(10, height - 50), 0, 0.6, cv::Scalar(0, 255, 255), 1.6, 8);
		cv::putText(rgbmat_left, "MIRDC", cv::Point(10, 90), 0, 1, cv::Scalar(128, 255, 255), 2, 8);
		cv::imshow("Left image", rgbmat_left);

		// Image method (TODO: GroundDetect method)
		if (newRGBMatF || newDepthMatF)
		{
			cv::imencode(".jpg", rgbmat_left, pubImgVec, encodeParam);
			imgPub->pubImage(pubImgVec, pubImgSize);
		}
	}
END:
    zedSub.close();
	return EXIT_SUCCESS;
}
