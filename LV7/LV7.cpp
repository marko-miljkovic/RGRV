#include <vtkAutoInit.h> 
VTK_MODULE_INIT(vtkRenderingOpenGL2);
VTK_MODULE_INIT(vtkInteractionStyle);
VTK_MODULE_INIT(vtkRenderingFreeType);
#include <iostream>
#include <string>
#include <ctime>
#include <Windows.h>
#include <vtkIterativeClosestPointTransform.h>
#include <vtkObjectBase.h>
#include <vtkSmartPointer.h>
#include <vtkPLYReader.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkProperty.h>
#include <vtkPoints.h>
#include <vtkCellArray.h>
#include <vtkCamera.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkIterativeClosestPointTransform.h>
#include <vtkTransformPolyDataFilter.h>
#include <vtkLandmarkTransform.h>

using namespace std;

int main(int argc, char* argv[]) {

	//Displaying a point cloud

	/*vtkSmartPointer<vtkPLYReader> plyReader = vtkSmartPointer<vtkPLYReader> :: New();
	plyReader->SetFileName("../../../3Dmodeli/bunny.ply");
	plyReader->Update();

	vtkSmartPointer<vtkPolyData> inputPD = plyReader->GetOutput();

	vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper> :: New();
	mapper->SetInputData(inputPD);
	
	vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor> ::New();
	actor->SetMapper(mapper);
	actor->GetProperty()->SetColor(1.0, 0.0, 0.0);
	actor->GetProperty()->SetPointSize(5);

	vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer> ::New();
	renderer->AddActor(actor);
	renderer->SetBackground(1.0, 1.0, 1.0);
	renderer->ResetCamera();
	renderer->GetActiveCamera();

	vtkSmartPointer<vtkRenderWindow> window = vtkSmartPointer<vtkRenderWindow> ::New();
	window->AddRenderer(renderer);
	window->SetSize(800,600);
	window->SetWindowName("Scena");

	vtkSmartPointer<vtkRenderWindowInteractor> interactor = vtkSmartPointer<vtkRenderWindowInteractor> ::New();
	interactor->SetRenderWindow(window);

	vtkSmartPointer<vtkInteractorStyleTrackballCamera> interactorStyle = vtkSmartPointer<vtkInteractorStyleTrackballCamera> ::New();
	interactor->SetInteractorStyle(interactorStyle);

	interactor->Start();*/

	//Start time measuring

	LARGE_INTEGER ctr1, ctr2, freq;
	QueryPerformanceCounter((LARGE_INTEGER*)& ctr1);

	//Reading original point cloud

	vtkSmartPointer<vtkPLYReader> plyReader = vtkSmartPointer<vtkPLYReader> ::New();
	plyReader->SetFileName("../../../3Dmodeli/bunny.ply");
	plyReader->Update();

	vtkSmartPointer<vtkPolyData> originalPD = plyReader->GetOutput();

	//Reading a point cloud that the original one is going to be transformed to

	vtkSmartPointer<vtkPLYReader> plyReader1 = vtkSmartPointer<vtkPLYReader> ::New();
	plyReader1->SetFileName("../../../3Dmodeli/bunny_t1.ply");
	plyReader1->Update();

	vtkSmartPointer<vtkPolyData> destinationPD = plyReader1->GetOutput();

	//Implementing iterative closest point algorithm

	vtkSmartPointer<vtkIterativeClosestPointTransform> icp = vtkSmartPointer<vtkIterativeClosestPointTransform> ::New();
	icp->SetSource(originalPD);
	icp->SetTarget(destinationPD);
	icp->GetLandmarkTransform()->SetModeToRigidBody();
	icp->SetMaximumNumberOfIterations(100);
	icp->SetMaximumNumberOfLandmarks(50);
	icp->Update();

	vtkSmartPointer<vtkTransformPolyDataFilter> icpTransformFilter = vtkSmartPointer<vtkTransformPolyDataFilter> ::New();
	icpTransformFilter->SetInputData(originalPD);
	icpTransformFilter->SetTransform(icp);
	icpTransformFilter->Update();

	vtkSmartPointer<vtkPolyData> resultPD = icpTransformFilter->GetOutput();

	//Showing results

	vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper> ::New();
	mapper->SetInputData(resultPD);
	vtkSmartPointer<vtkPolyDataMapper> mapper1 = vtkSmartPointer<vtkPolyDataMapper> ::New();
	mapper1->SetInputData(destinationPD);

	vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor> ::New();
	actor->SetMapper(mapper);
	actor->GetProperty()->SetColor(1.0, 0.0, 0.0);
	actor->GetProperty()->SetPointSize(5);
	vtkSmartPointer<vtkActor> actor1 = vtkSmartPointer<vtkActor> ::New();
	actor1->SetMapper(mapper1);
	actor1->GetProperty()->SetColor(0.0, 1.0, 0.0);
	actor1->GetProperty()->SetPointSize(5);

	vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer> ::New();
	renderer->AddActor(actor);
	renderer->AddActor(actor1);
	renderer->SetBackground(1.0, 1.0, 1.0);
	renderer->ResetCamera();
	renderer->GetActiveCamera();

	vtkSmartPointer<vtkRenderWindow> window = vtkSmartPointer<vtkRenderWindow> ::New();
	window->AddRenderer(renderer);
	window->SetSize(800, 600);
	window->SetWindowName("Scena");

	vtkSmartPointer<vtkRenderWindowInteractor> interactor = vtkSmartPointer<vtkRenderWindowInteractor> ::New();
	interactor->SetRenderWindow(window);

	vtkSmartPointer<vtkInteractorStyleTrackballCamera> interactorStyle = vtkSmartPointer<vtkInteractorStyleTrackballCamera> ::New();
	interactor->SetInteractorStyle(interactorStyle);

	interactor->Start(); 

	//Showing parameters

	// -- Displaying measured time

	QueryPerformanceCounter((LARGE_INTEGER*)& ctr2);
	QueryPerformanceFrequency((LARGE_INTEGER*)& freq);
	float timeValue = (ctr2.QuadPart - ctr1.QuadPart) * 1000.0 / freq.QuadPart;
	cout << "Measured time: " << timeValue << " ms" << endl;
	cout << "Maximum number of iterations: " << icp->GetMaximumNumberOfIterations() << endl;
	cout << "Maximum number of landmarks: " << icp->GetMaximumNumberOfLandmarks() << endl;

	return 0;

}
