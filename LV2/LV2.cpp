#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>

using namespace std;
using namespace cv;

struct Podaci{

	Subdiv2D subdiv;
	Scalar inTriangleColor, outTriangleColor, delaunayColor;
	vector<Point> convex_hull;

};

static void draw_delaunay(Mat& img, Subdiv2D& subdiv, Scalar delaunay_color) {

	Point2f pt1, pt2;
	vector<int> edge;
	int edge1 = 0, edge2;
	bool flag = false;
	Rect rect(20, 20, 400, 400);

	//Checking of getEdge function

	/*pt1 = subdiv.getVertex(4, &edge1);

	cout << "Edge ID: " << edge1 << endl;

	waitKey(2000);

	for (int i = 0; i < 10; i++){

		edge1=subdiv.getEdge(edge1,0x00);

		cout << "Edge ID: " << edge1 << endl;

		waitKey(2000);
	}*/

	for (int i = 4; i < 24; i++) {

		pt1 = subdiv.getVertex(i, &edge1);

		if (edge1 != 0) {

			subdiv.edgeDst(edge1, &pt2);

			for (int j = 0; j < edge.size(); j++) {


				if (edge[j] == edge1 || edge[j] == subdiv.rotateEdge(edge1, 2))
					flag = true;

			}

			if (rect.contains(pt2) && flag == false) {

				line(img, pt1, pt2, delaunay_color, 1, CV_AA, 0);
				edge.push_back(edge1);
				edge.push_back(subdiv.rotateEdge(edge1, 2));

			}

			edge2 = subdiv.getEdge(edge1, 0x00);

			flag = false;

			while (edge2 != edge1) {

				subdiv.edgeDst(edge2, &pt2);

				for (int j = 0; j < edge.size(); j++) {


					if (edge[j] == edge2 || edge[j] == subdiv.rotateEdge(edge2, 2))
						flag = true;

				}

				if (rect.contains(pt2) && flag == false) {

					line(img, pt1, pt2, delaunay_color, 1, CV_AA, 0);
					edge.push_back(edge2);
					edge.push_back(subdiv.rotateEdge(edge2, 2));

				}

				edge2 = subdiv.getEdge(edge2, 0x00);

				flag = false;

			}

			edge1 = 0;
		}
	}

}

static void onMouse(int event, int x, int y, int flags, void *korisnickiPodaci){

	//Example for checking function parameters

	/*if (event == EVENT_LBUTTONDOWN) {

			Point* p = (Point*)korisnickiPodaci;
			p->x = x;
			p->y = y;
			cout << *p;

	}*/

	Point2f pt(x, y), pt1;
	int edge, vert, edgeRot;
	Podaci *p = (Podaci*)korisnickiPodaci;
	vector<Point> vertex, vertex_out;

	if (event == EVENT_LBUTTONDOWN){

		Mat img = imread("blank_page.jpg");
		Mat img_copy;
		Size velicina(440, 440);
		resize(img, img_copy, velicina);

		draw_delaunay(img_copy, p->subdiv, p->delaunayColor);

		if (pointPolygonTest(p->convex_hull, pt, false)==1){

			p->subdiv.locate(pt, edge, vert);

			cout << "Edge 1 ID: " << edge << endl;

			for (int i = 0; i < 3; i++){

				if (i == 0){

					p->subdiv.edgeDst(edge, &pt1);
					vertex.push_back(pt1);

					cout << "Point 1: " << vertex[0] << endl << endl;

					//Neighbour triangle 1

					edgeRot = p->subdiv.rotateEdge(edge, 2);
					p->subdiv.edgeDst(edgeRot, &pt1);
					vertex_out.push_back(pt1);

					edgeRot = p->subdiv.getEdge(edgeRot, 0x13);
					p->subdiv.edgeDst(edgeRot, &pt1);

					if (pointPolygonTest(p->convex_hull, pt1, false) != -1) {

						vertex_out.push_back(pt1);

						edgeRot = p->subdiv.getEdge(edgeRot, 0x13);
						p->subdiv.edgeDst(edgeRot, &pt1);
						vertex_out.push_back(pt1);

						fillConvexPoly(img_copy, vertex_out, p->outTriangleColor);

					}

					vertex_out.clear();

				}
				else{

					edge = p->subdiv.getEdge(edge, 0x13);
					p->subdiv.edgeDst(edge, &pt1);
					vertex.push_back(pt1);

					cout << "Edge " << i+1 << " ID: " << edge << endl;
					cout << "Point " << i+1 << " : " << vertex[i] << endl << endl;

					//Neighbour triangles 2 and 3

					edgeRot = p->subdiv.rotateEdge(edge, 2);
					p->subdiv.edgeDst(edgeRot, &pt1);
					vertex_out.push_back(pt1);

					edgeRot = p->subdiv.getEdge(edgeRot, 0x13);
					p->subdiv.edgeDst(edgeRot, &pt1);

					if (pointPolygonTest(p->convex_hull, pt1, false) != -1) {

						vertex_out.push_back(pt1);

						edgeRot = p->subdiv.getEdge(edgeRot, 0x13);
						p->subdiv.edgeDst(edgeRot, &pt1);
						vertex_out.push_back(pt1);

						fillConvexPoly(img_copy, vertex_out, p->outTriangleColor);

					}

					vertex_out.clear();

				}

			}

			fillConvexPoly(img_copy, vertex, p->inTriangleColor);

			vertex.clear();

		}

		imshow("Delaunay triangulation", img_copy);

	}

}

int main(int argc, char *argv[]){

	Rect rect(20,20,400,400);
	Scalar point_color(255, 0, 0), delaunay_color(0,0,0);
	vector<Point> tocke;
	vector<Point> convex_hull;
	Subdiv2D subdiv(rect);
	int x, y;

	srand((unsigned)time(NULL));

	for (int i = 0; i < 20; i++){
		x = rand() % 401 + 20;
		y = rand() % 401 + 20;
		tocke.push_back(Point(x,y));
	}

	Mat img = imread("blank_page.jpg");
	Mat img_copy;
	Size velicina(440, 440);
	resize(img, img_copy, velicina);

	for (int i=0; i < 20; i++){
		subdiv.insert(tocke[i]);

		//Drawing points one by one

		/*circle(img_copy,tocke[i],2,point_color,CV_FILLED,CV_AA,0);
		imshow("Points", img_copy);
		waitKey(500);*/
	}

	//Checking of getVertex() function

	/*Point2f toc;

	for (int i = 4; i < 12; i++){

		toc = subdiv.getVertex(i);
		circle(img_copy, toc, 2, point_color, CV_FILLED, CV_AA, 0);
		imshow("Points", img_copy);
		waitKey(500);

	}*/

	convexHull(tocke, convex_hull);

	//Drawing the convex hull

	//polylines(img_copy, convex_hull, true, Scalar(0,0,0));

	draw_delaunay(img_copy, subdiv, delaunay_color);

	Podaci p;

	p.subdiv = subdiv;
	p.inTriangleColor = Scalar(0, 0, 255);
	p.outTriangleColor = Scalar(255, 0, 0);
	p.delaunayColor = delaunay_color;
	p.convex_hull = convex_hull;

	namedWindow("Delaunay triangulation", WINDOW_AUTOSIZE);

	setMouseCallback("Delaunay triangulation", onMouse, &p);

	imshow("Delaunay triangulation", img_copy);

	waitKey(0);

	return 0;

}
