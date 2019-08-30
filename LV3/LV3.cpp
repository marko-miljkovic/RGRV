#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/core.hpp"
#include <iostream>
#include <cmath>

using namespace cv;
using namespace std;

int cnt(0);

struct Data1 {

	Mat img, img_copy;
	vector<Point> points;

};

struct Data2 {

	Mat img, cameraMatrix, distCoeffs, rmat, tvec;
	vector<Point3f> object_points;
	vector<Point2f> img_points;

};

static void onMouse(int event, int x, int y, int flags, void* userData) {

	Data1* p = (Data1*)userData;

	if (event == EVENT_LBUTTONDOWN) {
			
		if (cnt < 3) {

			p->points.push_back(Point(x, y));

			circle(p->img_copy, Point(x,y), 2, Scalar(255,255,255), CV_FILLED, CV_AA, 0);

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

			width = abs(p->points[0].x-top_left.x);

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

static void onMouse2(int event, int x, int y, int flags, void* userData) {

	Data2* p = (Data2*)userData;

	if (event == EVENT_LBUTTONDOWN) {

		if (cnt < 3) {

			p->img_points.push_back(Point2f(x, y));

			circle(p->img, Point(x, y), 3, Scalar(255, 255, 255), CV_FILLED, CV_AA, 0);

			imshow("Selecting corresponding points", p->img);

			cnt++;

		}
		else {

			//Last point

			p->img_points.push_back(Point2f(x, y));

			circle(p->img, Point(x, y), 3, Scalar(255, 255, 255), CV_FILLED, CV_AA, 0);

			imshow("Selecting corresponding points", p->img);

			cnt = 0;

			Mat rvec;

			solvePnP(p->object_points, p->img_points, p->cameraMatrix, p->distCoeffs, rvec, p->tvec);

			Rodrigues(rvec, p->rmat);

			cout << "Rotation matrix: " << p->rmat << endl << endl;
			cout << "Translation vector: " << p->tvec << endl << endl;

			destroyAllWindows();

		}

	}

}

int main(int argc, char** argv) {

	//First CameraCalib folder is located two folders above current location

	Mat edges, coloredEdges, scene = imread("../../CameraCalib/CameraCalib/Scene_Img.jpg", IMREAD_GRAYSCALE);
	Data1 d1;

	d1.img = scene;
	d1.img_copy = scene.clone();

	if (scene.empty()) {
		cout << " Error opening image " << endl;
		cout << " Program Arguments: [image_name -- default Scene_Img.jpg]" << endl;
		return -1;
	}

	//Cropping the image

	namedWindow("Cropping the image", WINDOW_AUTOSIZE);
	setMouseCallback("Cropping the image", onMouse, &d1);
	imshow("Cropping the image", scene);
	waitKey(0);
	destroyAllWindows();

	//Cropped image

	scene = d1.img;
	
	//Edge detection

	Canny(scene, edges, 50, 200, 3);

	namedWindow("Edges", WINDOW_AUTOSIZE);
	imshow("Edges", edges);
	waitKey(0);
	destroyAllWindows();

	// Copy edges to the images that will display the results in BGR

	cvtColor(edges, coloredEdges, COLOR_GRAY2BGR);
	Mat cE_copy = coloredEdges.clone();

	// Standard Hough Line Transform

	vector<Vec2f> lines; // will hold the results of the detection

	HoughLines(edges, lines, 1, CV_PI / 180, 100, 0, 0); // runs the actual detection

	// Draw the lines

	for (size_t i = 0; i < lines.size(); i++)
	{
		float rho = lines[i][0], theta = lines[i][1];
		Point pt1, pt2;
		double a = cos(theta), b = sin(theta);
		double x0 = a * rho, y0 = b * rho;
		pt1.x = cvRound(x0 + 1000 * (-b));
		pt1.y = cvRound(y0 + 1000 * (a));
		pt2.x = cvRound(x0 - 1000 * (-b));
		pt2.y = cvRound(y0 - 1000 * (a));
		line(coloredEdges, pt1, pt2, Scalar(0, 0, 255), 3, LINE_AA);
	}

	namedWindow("Detected Lines (in red) - Standard Hough Line Transform", WINDOW_AUTOSIZE);
	imshow("Detected Lines (in red) - Standard Hough Line Transform", coloredEdges);

	waitKey(0);

	destroyAllWindows();

	//Drawing the dominant line

	float rho = lines[0][0], theta = lines[0][1];
	Point pt1, pt2;
	double a = cos(theta), k = sin(theta);
	double x0 = a * rho, y0 = k * rho;
	pt1.x = cvRound(x0 + 1000 * (-k));
	pt1.y = cvRound(y0 + 1000 * (a));
	pt2.x = cvRound(x0 - 1000 * (-k));
	pt2.y = cvRound(y0 - 1000 * (a));
	line(cE_copy, pt1, pt2, Scalar(0, 0, 255), 3, LINE_AA);

	namedWindow("Dominant line", WINDOW_AUTOSIZE);
	imshow("Dominant line", cE_copy);

	waitKey(0);

	destroyAllWindows();

	//Entering real parameters

	float theta_real, ro_real;

	cout << "Enter real line parameters: " << endl << endl;
	cout << "Enter the distance: " << endl << endl;
	cin >> ro_real;
	cout << endl << endl << "Enter the angle: " << endl << endl;
	cin >> theta_real;

	//Calculating Rotation matrix and Translation vector

	Data2 d2;

	d2.img = scene;

	FileStorage fsL("../../CameraCalib/CameraCalib/cameraparams.xml", FileStorage::READ);
	fsL["camera_matrix"] >> d2.cameraMatrix;
	fsL["distortion_coefficients"] >> d2.distCoeffs;
	fsL.release();

	vector<Point3f> objectPoints;

	objectPoints.push_back(Point3f(0, 17, 0));
	objectPoints.push_back(Point3f(26, 17, 0));
	objectPoints.push_back(Point3f(26, 0, 0));
	objectPoints.push_back(Point3f(0, 0, 0));

	d2.object_points = objectPoints;

	namedWindow("Selecting corresponding points", WINDOW_AUTOSIZE);
	setMouseCallback("Selecting corresponding points", onMouse2, &d2);
	imshow("Selecting corresponding points", scene);
	waitKey(0);
	destroyAllWindows();

	Mat R = d2.rmat, t = d2.tvec, cameraMatrix = d2.cameraMatrix;


	//Calculating line parameters in paper coordinate system

	Mat A, b;

	gemm(cameraMatrix,R,1.0,Mat(),0.0,A);

	cout << endl << endl << "Matrix A: " << A << endl << endl;

	gemm(cameraMatrix, t, 1.0, Mat(), 0.0, b);

	cout << "Vector b: " << A << endl << endl;

	float lambdaX, lambdaY, lambdaRo;

	lambdaX = A.at<double>(0,0) * cos(lines[0][1]) + A.at<double>(1,0) * sin(lines[0][1]) - lines[0][0]* A.at<double>(2, 0);
	lambdaY = A.at<double>(0, 1) * cos(lines[0][1]) + A.at<double>(1, 1) * sin(lines[0][1]) - lines[0][0] * A.at<double>(2, 1);
	lambdaRo = b.at<double>(2) * lines[0][0] - b.at<double>(0) * cos(lines[0][1]) - b.at<double>(1) * sin(lines[0][1]);

	float theta2, ro2;

	theta2 = atan2(lambdaY,lambdaX);
	ro2 = lambdaRo / (sqrt(pow(lambdaX,2)+ pow(lambdaY, 2)));

	//Showing results

	cout << "Equation of dominant line: y = " << -(cos(theta2)) / (sin(theta2)) << "x + " << ro2 / sin(theta2) << endl << endl;
	cout << "Equation of dominant real line: y = " << -(cos(theta_real)) / (sin(theta_real)) << "x + " << ro_real / sin(theta_real) << endl << endl;
	cout << "Theta: " << theta2 << endl << endl;
	cout << "Rho: " << ro2 << endl << endl;
	cout << "Theta real: " << theta_real << endl << endl;
	cout << "Rho real: " << ro_real << endl << endl;
	cout << "Difference in distance: " << abs(ro2 - ro_real) << endl << endl;
	cout << "Difference in angle: " << abs(theta2 - theta_real) << endl << endl;

	return 0;

}
