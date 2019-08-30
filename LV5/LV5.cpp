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

using namespace cv;
using namespace cv::xfeatures2d;
using namespace std;

void Convert2DPointsTo3DPoints(vector<KeyPoint>& points2D_L, vector<KeyPoint>& points2D_R, Mat& E, Mat& P, Mat& points3D)
{

	/*
	Details in:
	D. Kurtagic, Trodimenzionalna rekonstrukcija scene iz dvije slike (završni rad - preddiplomski
	studij). Elektrotehnicki fakultet, Osijek, September 9, 2010.
	*/

	//Find the inverse of P (projection matrix)
	Mat Pinv;

	invert(P, Pinv, DECOMP_SVD);

	//Determine the singular value decomposition (svd) of E (essential matrix)
	Mat U, V, Vt, D;
	SVD::compute(E, D, U, Vt);

	//Define W
	Mat W = Mat::zeros(3, 3, CV_32F);
	W.at<float>(0, 1) = -1;
	W.at<float>(1, 0) = 1;
	W.at<float>(2, 2) = 1;

	Mat A = U * W * Vt;

	Mat b = Mat::zeros(3, 1, CV_32F);
	b.at<float>(0, 0) = U.at<float>(0, 2);
	b.at<float>(1, 0) = U.at<float>(1, 2);
	b.at<float>(2, 0) = U.at<float>(2, 2);

	Mat Ainv, Ainv_b;
	invert(A, Ainv, DECOMP_SVD);
	Ainv_b = Ainv * b;

	/**** Helper matrices ****/
	Mat Lpi = Mat(3, 1, CV_32F);
	Mat Rpi = Mat(3, 1, CV_32F);
	Mat ARpi = Mat(3, 1, CV_32F);

	Mat S = Mat(2, 1, CV_32F);

	Mat X = Mat(2, 2, CV_32F);
	Mat x1 = Mat(1, 1, CV_32F);
	Mat x2 = Mat(1, 1, CV_32F);
	Mat x4 = Mat(1, 1, CV_32F);

	Mat Y = Mat(2, 1, CV_32F);
	Mat y1 = Mat(1, 1, CV_32F);
	Mat y2 = Mat(1, 1, CV_32F);

	//2D points in left (model) and right (scene) images
	Mat Lm = Mat(3, 1, CV_32F);
	Mat Rm = Mat(3, 1, CV_32F);

	//Iteratively convert 2D point pairs to 3D points
	for (size_t i = 0; i < points2D_L.size(); i++)
	{
		Lm.at<float>(0, 0) = points2D_L[i].pt.x;
		Lm.at<float>(1, 0) = points2D_L[i].pt.y;
		Lm.at<float>(2, 0) = 1;

		Rm.at<float>(0, 0) = points2D_R[i].pt.x;
		Rm.at<float>(1, 0) = points2D_R[i].pt.y;
		Rm.at<float>(2, 0) = 1;

		Lpi = Pinv * Lm;
		Rpi = Pinv * Rm;

		//Init X table
		ARpi = Ainv * Rpi;
		x1 = Lpi.t() * Lpi;
		x2 = Lpi.t() * ARpi;
		x4 = ARpi.t() * ARpi;

		X.at<float>(0, 0) = -x1.at<float>(0, 0);
		X.at<float>(0, 1) = x2.at<float>(0, 0);
		X.at<float>(1, 0) = x2.at<float>(0, 0);
		X.at<float>(1, 1) = -x4.at<float>(0, 0);

		//Init Y table
		y1 = Lpi.t() * Ainv_b;
		y2 = ARpi.t() * Ainv_b;

		Y.at<float>(0, 0) = -y1.at<float>(0, 0);
		Y.at<float>(1, 0) = y2.at<float>(0, 0);

		solve(X, Y, S);

		float s = S.at<float>(0, 0);
		float t = S.at<float>(1, 0);

		Lpi.at<float>(0, 0) = s * Lpi.at<float>(0, 0);
		Lpi.at<float>(1, 0) = s * Lpi.at<float>(1, 0);
		Lpi.at<float>(2, 0) = s * Lpi.at<float>(2, 0);

		ARpi.at<float>(0, 0) = t * ARpi.at<float>(0, 0);
		ARpi.at<float>(1, 0) = t * ARpi.at<float>(1, 0);
		ARpi.at<float>(2, 0) = t * ARpi.at<float>(2, 0);

		points3D.at<float>(0, i) = (Lpi.at<float>(0, 0) + ARpi.at<float>(0, 0) - Ainv_b.at<float>(0, 0)) / 2;
		points3D.at<float>(1, i) = (Lpi.at<float>(1, 0) + ARpi.at<float>(1, 0) - Ainv_b.at<float>(1, 0)) / 2;
		points3D.at<float>(2, i) = (Lpi.at<float>(2, 0) + ARpi.at<float>(2, 0) - Ainv_b.at<float>(2, 0)) / 2;

	}

}

int main(int argc, char** argv) {

	//Loading and showing the images

	Mat img_left = imread("../../../pics/image3.bmp", IMREAD_GRAYSCALE), img_right = imread("../../../pics/image4.bmp", IMREAD_GRAYSCALE);

	if (img_left.empty() || img_right.empty())
	{
		cout << "Could not open or find the image!\n" << endl;
		return -1;
	}

	namedWindow("Left image", WINDOW_AUTOSIZE);
	namedWindow("Right image", WINDOW_AUTOSIZE);
	imshow("Left image", img_left);
	imshow("Right image", img_right);
	waitKey(0);
	destroyAllWindows();

	//Detecting keypoints and computing descriptors using SIFT detector

	Ptr<SIFT> detector = SIFT::create();
	vector<KeyPoint> keypoints_left, keypoints_right;
	Mat descriptors_left, descriptors_right;

	detector->detectAndCompute(img_left, noArray(), keypoints_left, descriptors_left);
	detector->detectAndCompute(img_right, noArray(), keypoints_right, descriptors_right);

	Mat colored_left_img, colored_right_img;

	cvtColor(img_left, colored_left_img, COLOR_GRAY2BGR);
	cvtColor(img_right, colored_right_img, COLOR_GRAY2BGR);

	drawKeypoints(img_left, keypoints_left, colored_left_img);
	drawKeypoints(img_right, keypoints_right, colored_right_img);

	namedWindow("Keypoints on the left image", WINDOW_AUTOSIZE);
	namedWindow("Keypoints on the right image", WINDOW_AUTOSIZE);
	imshow("Keypoints on the left image", colored_left_img);
	imshow("Keypoints on the right image", colored_right_img);
	waitKey(0);
	destroyAllWindows();

	//Matching descriptor vectors with a FLANN based matcher

	Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create(DescriptorMatcher::FLANNBASED);
	vector< vector<DMatch> > knn_matches;

	matcher->knnMatch(descriptors_left, descriptors_right, knn_matches, 2);

	// -- Filter matches using the Lowe's ratio test

	const float ratio_thresh = 0.75f;
	vector<DMatch> good_matches;
	for (size_t i = 0; i < knn_matches.size(); i++)
	{
		if (knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance)
		{
			good_matches.push_back(knn_matches[i][0]);
		}
	}

	// -- Drawing matches

	Mat img_matches;
	drawMatches(img_left, keypoints_left, img_right, keypoints_right, good_matches, img_matches, Scalar::all(-1),
		Scalar::all(-1), vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

	namedWindow("Keypoint matches", WINDOW_AUTOSIZE);
	imshow("Keypoint matches", img_matches);
	waitKey(0);
	destroyAllWindows();

	//Calculating fundamental matrix

	vector<Point2f> good_matches_left;
	vector<Point2f> good_matches_right;

	for (int i = 0; i < good_matches.size(); i++)
	{
		// -- Get the keypoints from the good matches
		good_matches_left.push_back(keypoints_left[good_matches[i].queryIdx].pt);	
		good_matches_right.push_back(keypoints_right[good_matches[i].trainIdx].pt);

	}

	vector<uchar> mask;

	Mat F = findFundamentalMat(good_matches_left, good_matches_right, FM_RANSAC, 3, 0.99, mask);

	//Filtering kepoints using fundamental matrix

	Mat img_matches_filtered;
	vector<char> inliers(mask.begin(),mask.end());

	drawMatches(img_left, keypoints_left, img_right, keypoints_right, good_matches, img_matches_filtered, Scalar::all(-1),
		Scalar::all(-1), inliers, DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

	namedWindow("Keypoint matches filtered", WINDOW_AUTOSIZE);
	imshow("Keypoint matches filtered", img_matches_filtered);
	waitKey(0);
	destroyAllWindows();

	//Reading camera matrix parameters

	Mat P;

	FileStorage fsL("../../../intrinsics.xml", FileStorage::READ);		
	
	if (fsL.isOpened() == false) {

		cout << "Could not open .xml file!" << endl;
		return -1;

	}

	fsL["Intrinsics"] >> P;
	fsL.release();

	//Calculating esential matrix

	Mat E;

	gemm(P,F,1.0,Mat(),0.0,E,GEMM_1_T);
	gemm(E,P,1.0,Mat(),0.0,E);
	
	//Calculating 3D point coordinates

	Mat points3D = Mat(3,keypoints_left.size(),CV_32F);

	Convert2DPointsTo3DPoints(keypoints_left,keypoints_right,E,P,points3D);

	//Writing elements of points3D to .txt file

	ofstream outFile("points3d.txt");

	if (!outFile.is_open()) {

		cout << "Error: Cannot open file!" << endl;
		return -1;

	}

	for (int i = 0; i < keypoints_left.size(); i++) {

		for (int j = 0; j < 3; j++) {

			if (j < 2) {

				outFile << points3D.at<float>(j, i) << ",";

			}
			else {

				outFile << points3D.at<float>(j, i) << "\n";

			}

		}

	}

	outFile.close();

	return 0;

}
