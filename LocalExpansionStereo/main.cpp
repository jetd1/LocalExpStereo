
#define _CRT_SECURE_NO_WARNINGS

#include <omp.h>
#include <time.h>
#include <opencv2/opencv.hpp>
#include "FastGCStereo.h"
#include "Evaluator.h"
#include "ArgsParser.h"
#include "CostVolumeEnergy.h"
#include "Utilities.hpp"
#include <opencv2/stereo.hpp>
#include <cstring>
#include <cstdio>
#include <algorithm>
#include <iostream>
#include <chrono> 
using namespace std;
using namespace cv;
struct Options
{
	std::string mode = ""; // "MiddV2" or "Census"
	std::string outputDir = "";
	std::string targetDir = "";
	int iterations = 5;
	int pmIterations = 2;
	bool doDual = false;  // two view cross check

	int ndisp = 0;
	float smooth_weight = 1.0;
	float mc_threshold = 0.5;
	int filterRadious = 20;

	int threadNum = -1;
	void loadOptionValues(ArgsParser& argParser)
	{
		argParser.TryGetArgment("outputDir", outputDir);
		argParser.TryGetArgment("targetDir", targetDir);
		argParser.TryGetArgment("mode", mode);
		printf("%s () %s () %s ==================\n", outputDir.c_str(), targetDir.c_str(), mode.c_str());
		if (mode == "MiddV2")
			smooth_weight = 1.0;
		else if (mode == "Census")
			smooth_weight = 0.5;

		argParser.TryGetArgment("threadNum", threadNum);
		argParser.TryGetArgment("doDual", doDual);
		argParser.TryGetArgment("iterations", iterations);
		argParser.TryGetArgment("pmIterations", pmIterations);

		argParser.TryGetArgment("ndisp", ndisp);
		argParser.TryGetArgment("filterRadious", filterRadious);
		argParser.TryGetArgment("smooth_weight", smooth_weight);
		argParser.TryGetArgment("mc_threshold", mc_threshold);
	}

	void printOptionValues(FILE * out = stdout)
	{
		fprintf(out, "----------- parameter settings -----------\n");
		fprintf(out, "mode           : %s\n", mode.c_str());
		fprintf(out, "outputDir      : %s\n", outputDir.c_str());
		fprintf(out, "targetDir      : %s\n", targetDir.c_str());

		fprintf(out, "threadNum      : %d\n", threadNum);
		fprintf(out, "doDual         : %d\n", (int)doDual);
		fprintf(out, "pmIterations   : %d\n", pmIterations);
		fprintf(out, "iterations     : %d\n", iterations);

		fprintf(out, "ndisp          : %d\n", ndisp);
		fprintf(out, "filterRadious  : %d\n", filterRadious);
		fprintf(out, "smooth_weight  : %f\n", smooth_weight);
		fprintf(out, "mc_threshold   : %f\n", mc_threshold);
	}
};

const Parameters paramsBF = Parameters(20, 20, "BF", 10);
const Parameters paramsGF = Parameters(1.0f, 20, "GF", 0.0001f);
const Parameters paramsGFfloat = Parameters(1.0f, 20, "GFfloat", 0.0001f); // Slightly faster
uint8_t num1[65536];
struct Calib
{
	float cam0[3][3];
	float cam1[3][3];
	float doffs;
	float baseline;
	int width;
	int height;
	int ndisp;
	int isint;
	int vmin;
	int vmax;
	float dyavg;
	float dymax;
	float gt_prec; // for V2 only

	// ----------- format of calib.txt ----------
	//cam0 = [2852.758 0 1424.085; 0 2852.758 953.053; 0 0 1]
	//cam1 = [2852.758 0 1549.445; 0 2852.758 953.053; 0 0 1]
	//doffs = 125.36
	//baseline = 178.089
	//width = 2828
	//height = 1924
	//ndisp = 260
	//isint = 0
	//vmin = 36
	//vmax = 218
	//dyavg = 0.408
	//dymax = 1.923

	Calib()
		: doffs(0)
		, baseline(0)
		, width(0)
		, height(0)
		, ndisp(0)
		, isint(0)
		, vmin(0)
		, vmax(0)
		, dyavg(0)
		, dymax(0)
		, gt_prec(-1)
	{
	}

	Calib(std::string filename)
		: Calib()
	{
		FILE* fp = fopen(filename.c_str(), "r");
		char buff[512];

		if (fp != nullptr)
		{
			if (fgets(buff, sizeof(buff), fp) != nullptr) sscanf(buff, "cam0 = [%f %f %f; %f %f %f; %f %f %f]\n", &cam0[0][0], &cam0[0][1], &cam0[0][2], &cam0[1][0], &cam0[1][1], &cam0[1][2], &cam0[2][0], &cam0[2][1], &cam0[2][2]);
			if (fgets(buff, sizeof(buff), fp) != nullptr) sscanf(buff, "cam1 = [%f %f %f; %f %f %f; %f %f %f]\n", &cam1[0][0], &cam1[0][1], &cam1[0][2], &cam1[1][0], &cam1[1][1], &cam1[1][2], &cam1[2][0], &cam1[2][1], &cam1[2][2]);
			if (fgets(buff, sizeof(buff), fp) != nullptr) sscanf(buff, "doffs = %f\n", &doffs);
			if (fgets(buff, sizeof(buff), fp) != nullptr) sscanf(buff, "baseline = %f\n", &baseline);
			if (fgets(buff, sizeof(buff), fp) != nullptr) sscanf(buff, "width = %d\n", &width);
			if (fgets(buff, sizeof(buff), fp) != nullptr) sscanf(buff, "height = %d\n", &height);
			if (fgets(buff, sizeof(buff), fp) != nullptr) sscanf(buff, "ndisp = %d\n", &ndisp);
			if (fgets(buff, sizeof(buff), fp) != nullptr) sscanf(buff, "isint = %d\n", &isint);
			if (fgets(buff, sizeof(buff), fp) != nullptr) sscanf(buff, "vmin = %d\n", &vmin);
			if (fgets(buff, sizeof(buff), fp) != nullptr) sscanf(buff, "vmax = %d\n", &vmax);
			if (fgets(buff, sizeof(buff), fp) != nullptr) sscanf(buff, "dyavg = %f\n", &dyavg);
			if (fgets(buff, sizeof(buff), fp) != nullptr) sscanf(buff, "dymax = %f\n", &dymax);
			fclose(fp);
		}
	}
};

void fillOutOfView(cv::Mat& volume, int mode, int margin = 0)
{
	int D = volume.size.p[0];
	int H = volume.size.p[1];
	int W = volume.size.p[2];

	if (mode == 0)
		for (int d = 0; d < D; d++)
			for (int y = 0; y < H; y++)
			{
				auto p = volume.ptr<float>(d, y);
				auto q = p + d + margin;
				float v = *q;
				while (p != q) {
					*p = v;
					p++;
				}
			}
	else
		for (int d = 0; d < D; d++)
			for (int y = 0; y < H; y++)
			{
				auto q = volume.ptr<float>(d, y) + W;
				auto p = q - d - margin;
				float v = p[-1];
				while (p != q) {
					*p = v;
					p++;
				}
			}
}

cv::Mat convertVolumeL2R(cv::Mat& volSrc, int margin = 0)
{
	int D = volSrc.size[0];
	int H = volSrc.size[1];
	int W = volSrc.size[2];
	cv::Mat volDst = volSrc.clone();

	for (int d = 0; d < D; d++)
	{
		cv::Mat_<float> s0(H, W, volSrc.ptr<float>() + H * W*d);
		cv::Mat_<float> s1(H, W, volDst.ptr<float>() + H * W*d);
		s0(cv::Rect(d, 0, W - d, H)).copyTo(s1(cv::Rect(0, 0, W - d, H)));

		cv::Mat edge1 = s0(cv::Rect(W - 1 - margin, 0, 1, H)).clone();
		cv::Mat edge0 = s0(cv::Rect(d + margin, 0, 1, H)).clone();
		for (int x = W - 1 - d - margin; x < W; x++)
			edge1.copyTo(s1.col(x));
		for (int x = 0; x < margin; x++)
			edge0.copyTo(s1.col(x));
	}
	return volDst;
}


void MidV2(const std::string inputDir, const std::string outputDir, const Options& options)
{
	cv::Mat imL, imR, dispGT, nonocc;
	imL = imread(inputDir + "/img_0.png", IMREAD_COLOR);
	imR = imread(inputDir + "/img_1.png", IMREAD_COLOR);

	float errorThresh = 0.5f;
	float vdisp = 0; // Purtubation of vertical displacement in the range of [-vdisp, vdisp]
	float maxdisp = (float)options.ndisp - 1;

	Parameters param = paramsGF;
	param.windR = options.filterRadious;
	param.lambda = options.smooth_weight;

	{
		_mkdir((outputDir + "midv2_debug").c_str());
		auto tic = std::chrono::high_resolution_clock::now();
		FastGCStereo stereo(imL, imR, param, maxdisp, 0, vdisp);
		stereo.saveDir = outputDir + "midv2_debug/";

		IProposer* prop1 = new ExpansionProposer(1);
		IProposer* prop2 = new RandomProposer(7, maxdisp);
		IProposer* prop3 = new ExpansionProposer(2);
		IProposer* prop4 = new RansacProposer(1);
		stereo.addLayer(5, { prop1, prop4, prop2 });
		stereo.addLayer(15, { prop3, prop4 });
		stereo.addLayer(25, { prop3, prop4 });

		cv::Mat labeling, rawdisp;
		if (options.doDual)
			stereo.run(options.iterations, { 0, 1 }, options.pmIterations, labeling, rawdisp);
		else
			stereo.run(options.iterations, { 0 }, options.pmIterations, labeling, rawdisp);

		delete prop1;
		delete prop2;
		delete prop3;
		delete prop4;

		auto toc = std::chrono::high_resolution_clock::now();
		auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(toc - tic);

		cvutils::io::save_pfm_file(outputDir + "disp_midv2.pfm", stereo.getEnergyInstance().computeDisparities(labeling));
		if (options.doDual)
			cvutils::io::save_pfm_file(outputDir + "disp_midv2_dual.pfm", stereo.getEnergyInstance().computeDisparities(rawdisp));

		{
			FILE *fp = fopen((outputDir + "midv2_time.txt").c_str(), "w");
			if (fp != nullptr)
			{ 
				fprintf(fp, "%ld ms\n", duration.count());
				fclose(fp);
			}
		}
	}
}

void censusTransform(const cv::Mat& imgL, const cv::Mat& imgR, int kernel_size, cv::Mat& censusL, cv::Mat& censusR, int R, int C)
{
	int hk = kernel_size / 2;
	for (int r = 0; r < R; r++) {
		for (int c = 0; c < C; c++) {
			auto a = imgL.at<uchar>(r, c);
			auto b = imgR.at<uchar>(r, c);
			uint tr, tc, x = 0, y = 0, cnt = 0, bit = 0;
			for (int rr = -hk; rr <= hk; rr++) {
				for (int cc = -hk; cc <= hk; cc++) {
					tr = std::max(0, std::min(r + rr, R - 1));
					tc = std::max(0, std::min(c + cc, C - 1));
					x = x << 1 | (a <= imgL.at<uchar>(tr, tc));
					y = y << 1 | (b <= imgR.at<uchar>(tr, tc));
					cnt++;
					if (cnt % 32 == 31) {
						censusL.at<cv::Vec4i>(r, c)[bit] = x;
						censusR.at<cv::Vec4i>(r, c)[bit] = y;
						cnt = 0;
						x = 0;
						y = 0;
						bit++;
					}

				}
			}
		}
	}
}

float getDis(uint x) {
	return num1[x >> 16] + num1[x & 65535];
}

float getDis(cv::Vec4i a, cv::Vec4i b) {
	float t = 0;
	for (int i = 0; i < 4; i++) {
		t += getDis(a[i] ^ b[i]);
	}
	return t / 121;
}
void build_cost_volume(const cv::Mat& dispL, const cv::Mat& dispR, cv::Mat& volL, cv::Mat& volR, int D, int R, int C) {

	for (int d = 0; d < D; d++) {
		for (int r = 0; r < R; r++) {
			for (int c = 0; c < C; c++) {
				int l2r = std::max(0, c - d), r2l = std::min(C - 1, c + d);
				volL.at<float>(d, r, c) = getDis(dispL.at<cv::Vec4i>(r, c), dispR.at<cv::Vec4i>(r, l2r));
				volR.at<float>(d, r, c) = getDis(dispR.at<cv::Vec4i>(r, c), dispL.at<cv::Vec4i>(r, r2l));
			}
		}
	}
}


void Census(const std::string inputDir, const std::string outputDir, const Options& options)
{

	Mat imL = imread(inputDir + "/img_0.png", IMREAD_COLOR);
	Mat imR = imread(inputDir + "/img_1.png", IMREAD_COLOR);
	Mat imgL = imread(inputDir + "/img_0.png", IMREAD_GRAYSCALE);
	Mat imgR = imread(inputDir + "/img_1.png", IMREAD_GRAYSCALE);
	Mat dispGT, nonocc;
	

	float maxdisp = (float)options.ndisp - 1;
	float errorThresh = 1.0f;

	Parameters param = paramsGF;
	param.windR = options.filterRadious;
	param.lambda = options.smooth_weight;
	param.th_col = options.mc_threshold; // tau_CNN in the paper

	int sizes[] = { options.ndisp, imL.rows, imL.cols };
	int picsize[] = { imL.rows, imL.cols };
	cv::Mat volL = cv::Mat_<float>(3, sizes);
	cv::Mat volR = cv::Mat_<float>(3, sizes);

	auto tic = std::chrono::high_resolution_clock::now();
	cv::Mat censusImageLeft, censusImageRight;
	censusImageLeft.create(imL.rows, imL.cols, CV_32SC4);
	censusImageRight.create(imL.rows, imL.cols, CV_32SC4);
	censusTransform(imgL, imgR, 9, censusImageLeft, censusImageRight, imL.rows, imL.cols);

	build_cost_volume(censusImageLeft, censusImageRight, volL, volR, options.ndisp, imL.rows, imL.cols);
	{
		_mkdir((outputDir + "census_debug").c_str());
		FastGCStereo stereo(imL, imR, param, maxdisp);
        stereo.setStereoEnergyCPU(std::make_unique<CostVolumeEnergy>(imL, imR, volL, volR, param, maxdisp));
		stereo.saveDir = outputDir + "census_debug/";
		int w = imL.cols;
		IProposer* prop1 = new ExpansionProposer(1);
		IProposer* prop2 = new RandomProposer(7, maxdisp);
		IProposer* prop3 = new ExpansionProposer(2);
		IProposer* prop4 = new RansacProposer(1);
		stereo.addLayer(int(w * 0.01), { prop1, prop4, prop2 });
		stereo.addLayer(int(w * 0.03), { prop3, prop4 });
		stereo.addLayer(int(w * 0.09), { prop3, prop4 });

		cv::Mat labeling, rawdisp;
		if (options.doDual)
			stereo.run(options.iterations, { 0, 1 }, options.pmIterations, labeling, rawdisp);
		else
			stereo.run(options.iterations, { 0 }, options.pmIterations, labeling, rawdisp);

		delete prop1;
		delete prop2;
		delete prop3;
		delete prop4;

		auto toc = std::chrono::high_resolution_clock::now();
		auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(toc - tic);
		cvutils::io::save_pfm_file(outputDir + "disp_census.pfm", stereo.getEnergyInstance().computeDisparities(labeling));
		if (options.doDual)
			cvutils::io::save_pfm_file(outputDir + "disp_census_dual.pfm", stereo.getEnergyInstance().computeDisparities(rawdisp));

		{
			FILE* fp = fopen((outputDir + "census_time.txt").c_str(), "w");
			if (fp != nullptr) {
				fprintf(fp, "%ld ms\n", duration.count());
				fclose(fp);
			}
		}

		//delete eval;
	}
}



int main(int argc, const char** args)
{
	ArgsParser parser(argc, args);
	Options options;
	options.loadOptionValues(parser);
	unsigned int seed = (unsigned int)time(NULL);
	options.printOptionValues();
	FILE* foptions = fopen((options.outputDir + "/options.txt").c_str(), "w");
	options.printOptionValues(foptions);
	fclose(foptions);

	int nThread = omp_get_max_threads();
#pragma omp parallel for
	for (int j = 0; j < nThread; j++)
	{
		srand(seed + j);
		cv::theRNG() = seed + j;
	}

	if (options.threadNum > 0)
		omp_set_num_threads(options.threadNum);

	if (options.outputDir.length())
		_mkdir((options.outputDir).c_str());

	printf("\n\n");
	for (int i = 1; i < 65536; i++) {
		num1[i] = num1[i >> 1] + (i & 1);
	}
	if (options.mode == "MiddV2")
	{
		printf("Running by Middlebury V2 mode.\n");
		MidV2(options.targetDir + "/", options.outputDir + "/", options);
	}
	else if (options.mode == "Census")
	{
		printf("Running by Census mode.\n");
		string inputDir = options.targetDir;
		string outputDir = options.outputDir;
		Census(inputDir + "/", outputDir + "/", options);
	}

	else
	{
		printf("Specify the following arguments:\n");
		printf("  -mode [MiddV2, Census]\n");
		printf("  -targetDir [PATH_TO_IMAGE_DIR]\n");
		printf("  -outputDir [PATH_TO_OUTPUT_DIR]\n");
	}

	return 0;
}

/* Make a bat file....

echo off

set bin=%~dp0x64\Release\LocalExpansionStereo.exe
set datasetroot=%~dp0data
set resultsroot=%~dp0results

mkdir "%resultsroot%"
"%bin%" -targetDir "%datasetroot%\MiddV2\cones" -outputDir "%resultsroot%\cones" -mode MiddV2 -smooth_weight 1 -doDual 1
"%bin%" -targetDir "%datasetroot%\MiddV2\teddy" -outputDir "%resultsroot%\teddy" -mode MiddV2 -smooth_weight 1
"%bin%" -targetDir "%da tasetroot%\MiddV3\Adirondack" -outputDir "%resultsroot%\Adirondack" -mode MiddV3 -smooth_weight 0.5


*/
