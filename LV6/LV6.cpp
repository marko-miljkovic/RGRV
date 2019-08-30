#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <iostream>
#include <cmath>
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <stdlib.h>
#include <time.h>

using namespace cv;
using namespace cv::xfeatures2d;
using namespace std;

struct RV3DPOINT
{
	int u, v, d;
};

void ReadKinectPic(string pathRGB, Mat* depthImage, vector<RV3DPOINT>* point3DArray, int& n3DPoints)
{

	int* DepthMap = new int[(depthImage->cols) * (depthImage->rows)];
	memset(DepthMap, 0, (depthImage->cols) * (depthImage->rows) * sizeof(int));


	int u, v, d;
	int dmin = 2047;
	int dmax = 0;

	n3DPoints = 0;

	//Get DepthImage file path
	string pathDepth = pathRGB.substr(0, pathRGB.length() - 4);
	pathDepth.append("-D.txt");

	FILE* fp;

	fopen_s(&fp, pathDepth.c_str(), "r");

	if (fp)
	{
		bool bOK = true;

		//Determine max and min depth values and get Depth map
		for (v = 0; v < depthImage->rows; v++)
		{
			for (u = 0; u < depthImage->cols; u++)
			{
				if (!(bOK = (fscanf_s(fp, "%d ", &d) == 1)))
					break;


				if (d == 2047)
				{
					d = -1;
				}
				else
				{


					//determine min and max d
					if (d < dmin)
						dmin = d;

					if (d > dmax)
						dmax = d;
				}

				DepthMap[v * depthImage->cols + u] = d;


				if (d != -1)
				{
					RV3DPOINT pt3;
					pt3.u = u;
					pt3.v = v;
					pt3.d = d;

					point3DArray->push_back(pt3);

				}


			}
		}

		fclose(fp);
	}


	//get  number of valid 3D points
	n3DPoints = point3DArray->size();

	//Form grayscale pic -> Scale from 1 to 255 (reserve 0 for undefined regions)
	for (v = 0; v < depthImage->rows; v++)
	{
		for (u = 0; u < depthImage->cols; u++)
		{
			d = DepthMap[v * depthImage->cols + u];

			if (d != -1)
				d = ((d - dmin) * 254 / (dmax - dmin)) + 1;
			else
				d = 0;

			((uchar*)(depthImage->data + v * depthImage->step))[u] = d;

		}
	}

	delete[] DepthMap;
}

void DetectingDominantPlane(vector<RV3DPOINT>& point3DArray, vector<RV3DPOINT>& dominantPlane) {

	int r;
	float epsilon = 3.0;
	vector<RV3DPOINT> points;
	vector<RV3DPOINT> T;

	srand((unsigned)time(NULL));
	
	for (int i = 0; i < 100; i++) {

		Mat A(3,3, CV_32FC1), R(3,1, CV_32FC1), C(3,1, CV_32FC1), invA(3,3, CV_32FC1);

		//Getting 3 random points to determine a plane

		for (int j = 0; j < 3; j++) {

				r = rand() % point3DArray.size();
				points.push_back(point3DArray[r]);

		}

		//Calculating plane parameters

		for (int j = 0; j < 3; j++) {

			A.at<float>(j, 0) = points[j].u;
			A.at<float>(j, 1) = points[j].v;
			A.at<float>(j, 2) = 1;

			C.at<float>(j, 0) = points[j].d;

		}

		invert(A, invA);
		R = invA * C;

		//Points laying inside the plane

		for (int j = 0; j < point3DArray.size(); j++) {

			if (abs(point3DArray[j].d - (R.at<float>(0, 0) * point3DArray[j].u + R.at<float>(1, 0) * point3DArray[j].v + R.at<float>(2, 0))) <= epsilon) {

				T.push_back(point3DArray[j]);

			}

		}

		//Checking if detected plane is dominant

		if (T.size() > dominantPlane.size()) {

			dominantPlane = T;

		}

		points.clear();
		T.clear();

	}

}

void ShowDominantPlane(vector<RV3DPOINT>& dominantPlane, Mat& cDepthImage){

	int* DepthMap = new int[(cDepthImage.cols) * (cDepthImage.rows)];
	memset(DepthMap, 0, (cDepthImage.cols) * (cDepthImage.rows) * sizeof(int));

	for (int i = 0; i < dominantPlane.size(); i++) {

		DepthMap[dominantPlane[i].v * cDepthImage.cols + dominantPlane[i].u] = dominantPlane[i].d;

	}

	for (int i = 0; i < cDepthImage.rows; i++) {

		for (int j = 0; j < cDepthImage.cols; j++) {

			if (DepthMap[i * cDepthImage.cols + j] != 0) {

				cDepthImage.at<Vec3b>(i, j) = Vec3b(0, 0, 255);

			}

		}

	}

	namedWindow("Dominant plane", WINDOW_AUTOSIZE);
	imshow("Dominant plane", cDepthImage);
	waitKey(0);
	destroyAllWindows();

}

string PicSequence(int i) {

	string s;

	switch (i) {

	case 0: s = "001"; break;
	case 1: s = "002"; break;
	case 2: s = "003"; break;
	case 3: s = "133"; break;
	case 4: s = "242"; break;
	case 5: s = "270"; break;
	case 6: s = "300"; break;
	case 7: s = "392"; break;
	default: s = "411"; break;

	}

	return s;

}

//Function for finding Mat type

/*string type2str(int type) {
	string r;

	uchar depth = type & CV_MAT_DEPTH_MASK;
	uchar chans = 1 + (type >> CV_CN_SHIFT);

	switch (depth) {
	case CV_8U:  r = "8U"; break;
	case CV_8S:  r = "8S"; break;
	case CV_16U: r = "16U"; break;
	case CV_16S: r = "16S"; break;
	case CV_32S: r = "32S"; break;
	case CV_32F: r = "32F"; break;
	case CV_64F: r = "64F"; break;
	default:     r = "User"; break;
	}

	r += "C";
	r += (chans + '0');

	return r;
}*/

int main(int argc, char** argv) {

	//Checking the size of depth image

	/*int cnt = 1;

	ifstream inDat("../../../KinectPics/sl-00001-D.txt");

	if (!inDat) 
	{
		cout << "Error" << endl;
		cout << "Cannot open file!" << endl;
		return 1; 
	}

	string line;
	string line2;

	getline(inDat, line);

	while (getline(inDat, line2)) {

		cnt++;

	}

	inDat.close();

	cout << "Number of rows in depth image: " << cnt << endl;

	cnt = 0;

	stringstream ss;

	ss << line;

	string temp;
	int found;
	while (!ss.eof()) {

		ss >> temp;

		if (stringstream(temp) >> found)
			cnt++;

		temp = "";
	}

	cout << "Number of columns in depth image: " << cnt << endl;*/

	//For all 9 images

	for (int i = 0; i < 9; i++) {

		//Reading depth image and RGB image

		Mat depthImage = imread("../../../KinectPics/sl-00" + PicSequence(i) + ".bmp", CV_LOAD_IMAGE_GRAYSCALE);
		Mat imgRGB = depthImage.clone();

		vector<RV3DPOINT> point3DArray;
		int n3DPoints;

		ReadKinectPic("../../../KinectPics/sl-00" + PicSequence(i) + ".bmp", &depthImage, &point3DArray, n3DPoints);

		cout << "Number of 3D points: " << n3DPoints << endl;

		namedWindow("Depth Image", WINDOW_AUTOSIZE);
		namedWindow("RGB Image", WINDOW_AUTOSIZE);
		imshow("Depth Image", depthImage);
		imshow("RGB Image", imgRGB);
		waitKey(0);
		destroyAllWindows();

		//Detecting dominant plane

		vector<RV3DPOINT> dominantPlane;

		DetectingDominantPlane(point3DArray, dominantPlane);

		cout << "Size of dominant plane " << dominantPlane.size() << endl;

		//Showing dominant plane

		Mat cDepthImage;

		cvtColor(depthImage, cDepthImage, COLOR_GRAY2BGR);

		// -- Finding Mat type

		/*string ty = type2str(cDepthImage.type());
		printf("Matrix: %s %dx%d \n", ty.c_str(), cDepthImage.cols, cDepthImage.rows);*/

		namedWindow("RGB Image", WINDOW_AUTOSIZE);
		imshow("RGB Image", imgRGB);

		ShowDominantPlane(dominantPlane, cDepthImage);

	}

	return 0;

}
