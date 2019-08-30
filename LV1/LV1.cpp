#include <conio.h>
#include <vector>
#include <vtkAutoInit.h>
#include <math.h>
VTK_MODULE_INIT(vtkRenderingOpenGL2);
VTK_MODULE_INIT(vtkInteractionStyle);
VTK_MODULE_INIT(vtkRenderingFreeType);
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkActor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkInteractorStyle.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkAxesActor.h>
#include <vtkTextActor.h>
#include <vtkTransform.h>
#include <iostream>
#include "RVL3DTools.h"
#include "Display.h"
#include "Body.h"
#include "stdio.h"
using namespace std;

#define PI	3.14159265359


int main(int argc, char *argv[])
{
	vtkSmartPointer<vtkMatrix4x4> T = vtkSmartPointer<vtkMatrix4x4>::New();
	double alpha1, alpha2, alpha3;

	printf("Upisite kuteve okreta, redom prvog, drugog i treceg clanka: ");
	scanf("%lf %lf %lf", &alpha1, &alpha2, &alpha3);
	alpha1 *= (PI / 180);
	alpha2 *= (PI / 180);
	alpha3 *= (PI / 180);

	//**** Create display. ****//

	Display window;

	window.Create(800, 600, "Racunalna geometrija LV1", 1.0, 1.0, 1.0);


	//**** Create scene. ****//

	Body scene;


	//1.  Base
	Body base;

	base.CreateBox(0.4, 0.4, 0.15, 0.5, 0.5, 0.5);

	scene.AddPart(&base);

	//**** Axis 1 ****//
	//Fixed position 1 relative to global coordinate system
	//position 0.1 above the centre of mass of base(1 / 2 base = 0.075 + 1 / 2 cylinder = 0.025)
	double T10[16] = {
		cos(alpha1), -sin(alpha1), 0, 0.0,
		sin(alpha1), cos(alpha1), 0, 0.0,
		0, 0, 1, 0.1,
		0, 0, 0, 1
	};


	//**** Axis 2 ****//
	////Fixed position 2 relative to position 1
	double T21[16] = {
		cos(alpha2), -sin(alpha2), 0, 0.275,
		sin(alpha2), cos(alpha2), 0, 0.0,
		0, 0, 1, 0.15,
		0, 0, 0, 1
	};

	//Determine T20 
	double T20[16];

	T->Multiply4x4(T10, T21, T20);

	//Fixed position 3 relative to position 2
	double T32[16] = {
		cos(alpha3), -sin(alpha3), 0, 0.275,
		sin(alpha3), cos(alpha3), 0, 0.0,
		0, 0, 1, 0.15,
		0, 0, 0, 1
	};

	//Determine T30
	double T30[16];

	T->Multiply4x4(T20, T32, T30);

	//2. Joint 1
	Body joint1;

	scene.AddPart(&joint1);

	joint1.CreateCylinder(0.025, 0.05, 20, 0.5, 0.5, 0.5);

	//Joint 1 relative to Axis 1
	//Rotate joint about x axis by 90° 
	double TJ11[16] = {
		1, 0, 0, 0.0,
		0, 0, -1, 0.0,
		0, 1, 0, 0.0,
		0, 0, 0, 1
	};

	double TJ10[16];

	T->Multiply4x4(T10, TJ11, TJ10);

	joint1.Transform(TJ10);


	//3. Link 1
	Body link1;

	scene.AddPart(&link1);

	link1.CreateBox(0.1, 0.4, 0.1, 0.5, 0.5, 0.5);

	//Link 1 relative to Axis 1
	//Position link 1 above axis 1 (link 1 needs to be rotated about the x axis)
	double TL11[16] = {
		0, -1, 0, 0.1375,
		1, 0, 0, 0.0,
		0, 0, 1, 0.075,
		0, 0, 0, 1
	};


	double TL10[16];

	T->Multiply4x4(T10, TL11, TL10);

	link1.Transform(TL10);

	//4  Joint 2
	Body joint2;

	scene.AddPart(&joint2);

	joint2.CreateCylinder(0.025, 0.05, 20, 0.5, 0.5, 0.5);

	//Joint 2 relative to Axis 2 (Rotate joint about x axis by 90° )
	double TJ22[16] = {
		1, 0, 0, 0.0,
		0, 0, -1, 0.0,
		0, 1, 0, 0.0,
		0, 0, 0, 1
	};

	double TJ20[16];

	T->Multiply4x4(T20, TJ22, TJ20);

	joint2.Transform(TJ20);


	//5. Link 2
	Body link2;

	scene.AddPart(&link2);

	link2.CreateBox(0.1, 0.4, 0.1, 0.5, 0.5, 0.5);

	//Link 2 relative to Axis 2
	//Position link 2 above axis 2 (link 2 needs to be rotated about the x axis)
	double TL22[16] = {
		0, -1, 0, 0.1375,
		1, 0, 0, 0.0,
		0, 0, 1, 0.075,
		0, 0, 0, 1
	};


	double TL20[16];

	T->Multiply4x4(T20, TL22, TL20);

	link2.Transform(TL20);

	//6. Joint 3
	Body joint3;

	scene.AddPart(&joint3);

	joint3.CreateCylinder(0.025, 0.05, 20, 0.5, 0.5, 0.5);

	//Joint 3 relative to Axis 3
	double TJ33[16] = {
		1, 0, 0, 0.0,
		0, 0, -1, 0.0,
		0, 1, 0, 0.0,
		0, 0, 0, 1
	};

	double TJ30[16];

	T->Multiply4x4(T30, TJ33, TJ30);

	joint3.Transform(TJ30);

	//7. Link 3
	Body link3;

	scene.AddPart(&link3);

	link3.CreateBox(0.1, 0.4, 0.1, 0.5, 0.5, 0.5);

	//Link 3 relative to Axis 3
	//Position link 3 above axis 3 (link 3 needs to be rotated about the x axis)
	double TL33[16] = {
		0, -1, 0, 0.1375,
		1, 0, 0, 0.0,
		0, 0, 1, 0.075,
		0, 0, 0, 1
	};


	double TL30[16];

	T->Multiply4x4(T30, TL33, TL30);

	link3.Transform(TL30);

	//**** Add reference axes ****//
	vtkSmartPointer<vtkAxesActor> axes = vtkSmartPointer<vtkAxesActor>::New();

	// The axes are positioned with a user transform
	vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
	transform->Translate(-0.5, 0.0, 0.0);

	axes->SetUserTransform(transform);
	axes->SetTotalLength(0.2, 0.2, 0.2);
	axes->GetXAxisCaptionActor2D()->GetTextActor()->SetTextScaleModeToNone();
	axes->GetYAxisCaptionActor2D()->GetTextActor()->SetTextScaleModeToNone();
	axes->GetZAxisCaptionActor2D()->GetTextActor()->SetTextScaleModeToNone();
	window.renderer->AddActor(axes);

	cout << "Upisite kut okreta: " << endl;

	//**** Render scene. ****//
	scene.AddToDisplay(&window);

	window.Run();

	//

	return 0;
}
