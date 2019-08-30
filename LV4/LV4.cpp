#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <iostream>

using namespace cv;
using namespace cv::xfeatures2d;
using namespace std;


int cnt = 0;

struct Data1 {

	Mat img, img_copy;
	vector<Point> points;

};

static void onMouse(int event, int x, int y, int flags, void* userData) {

	Data1* p = (Data1*)userData;

	if (event == EVENT_LBUTTONDOWN) {

		if (cnt < 3) {

			p->points.push_back(Point(x, y));

			circle(p->img_copy, Point(x, y), 2, Scalar(255, 255, 255), CV_FILLED, CV_AA, 0);

			imshow("Cropping the image", p->img_copy);

			cnt++;

		}
		else {

			Rect rect;
			Point top_left;
			int width, height;

			//Last point

			p->points.push_back(Point(x, y));

			circle(p->img_copy, Point(x, y), 2, Scalar(255, 255, 255), CV_FILLED, CV_AA, 0);

			imshow("Cropping the image", p->img_copy);

			cnt = 0;

			//Selecting top left point

			top_left.x = p->points[0].x;
			top_left.y = p->points[0].y;

			for (int i = 1; i < 4; i++) {

				if ((p->points[i].x + p->points[i].y) < (top_left.x + top_left.y)) {

					top_left = p->points[i];

				}

			}

			cout << "Top left point: " << top_left << endl << endl;

			//Calculating width

			width = abs(p->points[0].x - top_left.x);

			for (int i = 1; i < 4; i++) {

				if (abs(p->points[i].x - top_left.x) > width) {

					width = abs(p->points[i].x - top_left.x);

				}

			}

			cout << "Width: " << width << endl << endl;

			//Calculating height

			height = abs(p->points[0].y - top_left.y);

			for (int i = 1; i < 4; i++) {

				if (abs(p->points[i].y - top_left.y) > height) {

					height = abs(p->points[i].y - top_left.y);

				}

			}

			cout << "Height: " << height << endl << endl;

			//Cropping the image

			rect = Rect(top_left.x, top_left.y, width, height);

			p->img = p->img(rect);

			destroyAllWindows();
			namedWindow("Cropped Image", WINDOW_AUTOSIZE);
			imshow("Cropped Image", p->img);
			waitKey(0);

		}

	}

}

int main(int argc, char** argv) {

	//Loading and showing the images

	Mat img_referent = imread("../../../test_images/ImageT0.jpg", IMREAD_GRAYSCALE), img_working = imread("../../../test_images/ImageT2.jpg", IMREAD_GRAYSCALE);

	if (img_referent.empty() || img_working.empty())
	{
		cout << "Could not open or find the image!\n" << endl;
		return -1;
	}

	namedWindow("Referent image", WINDOW_AUTOSIZE);
	namedWindow("Working image", WINDOW_AUTOSIZE);
	imshow("Referent image", img_referent);
	imshow("Working image", img_working);
	waitKey(0);
	destroyAllWindows();

	//Cutting object image from the referent image

	Data1 d1;
	d1.img = img_referent;
	d1.img_copy = img_referent.clone();

	namedWindow("Cropping the image", WINDOW_AUTOSIZE);
	setMouseCallback("Cropping the image", onMouse, &d1);
	imshow("Cropping the image", img_referent);
	waitKey(0);
	destroyAllWindows();

	Mat img_object = d1.img;

	//Detecting keypoints and computing descriptors using SIFT detector
	
	Ptr<SIFT> detector = SIFT::create();
	vector<KeyPoint> keypoints_object, keypoints_work;
	Mat descriptors_object, descriptors_work;

	detector->detectAndCompute(img_object, noArray(), keypoints_object, descriptors_object);
	detector->detectAndCompute(img_working, noArray(), keypoints_work, descriptors_work);

	Mat colored_object_img, colored_work_img;

	cvtColor(img_object, colored_object_img, COLOR_GRAY2BGR);
	cvtColor(img_working, colored_work_img, COLOR_GRAY2BGR);

	drawKeypoints(img_object, keypoints_object, colored_object_img);
	drawKeypoints(img_working, keypoints_work, colored_work_img);

	namedWindow("Keypoints on the object image", WINDOW_AUTOSIZE);
	namedWindow("Keypoints on the working image", WINDOW_AUTOSIZE);
	imshow("Keypoints on the object image", colored_object_img);
	imshow("Keypoints on the working image", colored_work_img);
	waitKey(0);
	destroyAllWindows();

	//Matching descriptor vectors with a FLANN based matcher

	Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create(DescriptorMatcher::FLANNBASED);
	vector< vector<DMatch> > knn_matches;

	matcher->knnMatch(descriptors_object, descriptors_work, knn_matches, 2);

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
	drawMatches(img_object, keypoints_object, img_working, keypoints_work, good_matches, img_matches, Scalar::all(-1),
		Scalar::all(-1), vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

	namedWindow("Keypoint matches", WINDOW_AUTOSIZE);
	imshow("Keypoint matches", img_matches);
	waitKey(0);
	destroyAllWindows();

	//Localize the object

	vector<Point2f> object;
	vector<Point2f> work;

	for (int i = 0; i < good_matches.size(); i++)
	{
		// -- Get the keypoints from the good matches
		object.push_back(keypoints_object[good_matches[i].queryIdx].pt);
		work.push_back(keypoints_work[good_matches[i].trainIdx].pt);
	}

	// -- Find homography matrix between two pictures

	Mat H = findHomography(object, work, RANSAC);

	// -- Get the corners from the object image

	vector<Point2f> obj_corners(4);

	obj_corners[0] = Point2f(0, 0);
	obj_corners[1] = Point2f((float)img_object.cols, 0);
	obj_corners[2] = Point2f((float)img_object.cols, (float)img_object.rows);
	obj_corners[3] = Point2f(0, (float)img_object.rows);

	// -- Get the corners from the working image using homography matrix

	vector<Point2f> scene_corners(4);

	perspectiveTransform(obj_corners, scene_corners, H);

	// -- Draw lines between the corners (the mapped object in the scene - working image)

	line(img_matches, scene_corners[0] + Point2f((float)img_object.cols, 0),
		scene_corners[1] + Point2f((float)img_object.cols, 0), Scalar(0, 255, 0), 4);
	line(img_matches, scene_corners[1] + Point2f((float)img_object.cols, 0),
		scene_corners[2] + Point2f((float)img_object.cols, 0), Scalar(0, 255, 0), 4);
	line(img_matches, scene_corners[2] + Point2f((float)img_object.cols, 0),
		scene_corners[3] + Point2f((float)img_object.cols, 0), Scalar(0, 255, 0), 4);
	line(img_matches, scene_corners[3] + Point2f((float)img_object.cols, 0),
		scene_corners[0] + Point2f((float)img_object.cols, 0), Scalar(0, 255, 0), 4);

	namedWindow("Detected object", WINDOW_AUTOSIZE);
	imshow("Detected object", img_matches);
	waitKey(0);
	destroyAllWindows();

	return 0;

}
