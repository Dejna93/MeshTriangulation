//
// Created by dejna on 15.02.17.
//
/*
#include <vtkCleanPolyData.h>
#include <vtkDataSetMapper.h>
#include <vtkDelaunay3D.h>
#include <vtkXMLPolyDataReader.h>
#include <vtkVertexGlyphFilter.h>
#include <vtkAppendPolyData.h>
#include <vtkPointSource.h>
#include <vtkStructuredGridReader.h>
#include "include/vtk/SmoothPolyDataFilter.h"


void SmoothPolyDataFilter::main(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    // Create parabola over a grid of points
    vtkSmartPointer<vtkPoints> points =
            vtkSmartPointer<vtkPoints>::New();

    int GridSize = 20;
    double z;
    for (int x = -GridSize; x < GridSize; x++)
    {
        for (int y = -GridSize; y < GridSize; y++)
        {
            z = vtkMath::Random(-1, 1)+ 0.05*x*x + 0.05*y*y;
            points->InsertNextPoint(x, y, z);
        }
    }

    // Add the grid points to a polydata object
    vtkSmartPointer<vtkPolyData> inputPolyData =
            vtkSmartPointer<vtkPolyData>::New();
    inputPolyData->SetPoints(pclTovtk(cloud));

    // Triangulate the grid points
    vtkSmartPointer<vtkDelaunay2D> delaunay =
            vtkSmartPointer<vtkDelaunay2D>::New();
#if VTK_MAJOR_VERSION <= 5
    delaunay->SetInput(inputPolyData);
#else
    delaunay->SetInputData(inputPolyData);
#endif
    delaunay->Update();

    vtkSmartPointer<vtkSmoothPolyDataFilter> smoothFilter =
            vtkSmartPointer<vtkSmoothPolyDataFilter>::New();
    smoothFilter->SetInputConnection(delaunay->GetOutputPort());
    smoothFilter->SetNumberOfIterations(15);
    smoothFilter->SetRelaxationFactor(0.1);
    smoothFilter->FeatureEdgeSmoothingOff();
    smoothFilter->BoundarySmoothingOn();
    smoothFilter->Update();

    // Update normals on newly smoothed polydata
    vtkSmartPointer<vtkPolyDataNormals> normalGenerator = vtkSmartPointer<vtkPolyDataNormals>::New();
    normalGenerator->SetInputConnection(smoothFilter->GetOutputPort());
    normalGenerator->ComputePointNormalsOn();
    normalGenerator->ComputeCellNormalsOn();
    normalGenerator->Update();

    vtkSmartPointer<vtkPolyDataMapper> inputMapper =
            vtkSmartPointer<vtkPolyDataMapper>::New();
    inputMapper->SetInputConnection(delaunay->GetOutputPort());
    vtkSmartPointer<vtkActor> inputActor =
            vtkSmartPointer<vtkActor>::New();
    inputActor->SetMapper(inputMapper);

    vtkSmartPointer<vtkPolyDataMapper> smoothedMapper =
            vtkSmartPointer<vtkPolyDataMapper>::New();
    smoothedMapper->SetInputConnection(normalGenerator->GetOutputPort());
    vtkSmartPointer<vtkActor> smoothedActor =
            vtkSmartPointer<vtkActor>::New();
    smoothedActor->SetMapper(smoothedMapper);

    // There will be one render window
    vtkSmartPointer<vtkRenderWindow> renderWindow =
            vtkSmartPointer<vtkRenderWindow>::New();
    renderWindow->SetSize(600, 300);

    // And one interactor
    vtkSmartPointer<vtkRenderWindowInteractor> interactor =
            vtkSmartPointer<vtkRenderWindowInteractor>::New();
    interactor->SetRenderWindow(renderWindow);

    // Define viewport ranges
    // (xmin, ymin, xmax, ymax)
    double leftViewport[4] = { 0.0, 0.0, 0.5, 1.0 };
    double rightViewport[4] = { 0.5, 0.0, 1.0, 1.0 };

    // Setup both renderers
    vtkSmartPointer<vtkRenderer> leftRenderer =
            vtkSmartPointer<vtkRenderer>::New();
    renderWindow->AddRenderer(leftRenderer);
    leftRenderer->SetViewport(leftViewport);
    leftRenderer->SetBackground(.6, .5, .4);

    vtkSmartPointer<vtkRenderer> rightRenderer =
            vtkSmartPointer<vtkRenderer>::New();
    renderWindow->AddRenderer(rightRenderer);
    rightRenderer->SetViewport(rightViewport);
    rightRenderer->SetBackground(.4, .5, .6);

    // Add the input parabola to the left and the smoothed parabola to the right
    leftRenderer->AddActor(inputActor);
    rightRenderer->AddActor(smoothedActor);

    leftRenderer->ResetCamera();
    leftRenderer->GetActiveCamera()->Azimuth(130);
    leftRenderer->GetActiveCamera()->Elevation(-80);

    rightRenderer->ResetCamera();
    rightRenderer->GetActiveCamera()->Azimuth(130);
    rightRenderer->GetActiveCamera()->Elevation(-80);

    renderWindow->Render();
    interactor->Start();
}

vtkSmartPointer<vtkPoints> SmoothPolyDataFilter::pclTovtk(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {

    vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
    std::cout << "Befor size " << cloud->points.size() <<"\n";
    for(pcl::PointCloud<pcl::PointXYZ>::iterator it = cloud->points.begin() ; it < cloud->points.end() ; ++it )
    {
        points->InsertNextPoint(it->x,it->y,it->z);
    }
    std::cout << "After size " << points->GetNumberOfPoints() <<"\n";

    return points;
}

void SmoothPolyDataFilter::del3d(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {

    // Read the file
    vtkSmartPointer<vtkStructuredGridReader> reader =
            vtkSmartPointer<vtkStructuredGridReader>::New();
    reader->SetFileName("/home/dejna/abaqus_plugin/CloudMesh/workspace/project/points/microstructure0.vtk");
    reader->Update();

    vtkSmartPointer<vtkStructuredGridGeometryFilter> geometryFilter =
            vtkSmartPointer<vtkStructuredGridGeometryFilter>::New();
    geometryFilter->SetInputConnection(reader->GetOutputPort());
    geometryFilter->Update();

    // Visualize
    vtkSmartPointer<vtkPolyDataMapper> mapper =
            vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputConnection(geometryFilter->GetOutputPort());

    vtkSmartPointer<vtkActor> actor =
            vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);

    vtkSmartPointer<vtkRenderer> renderer =
            vtkSmartPointer<vtkRenderer>::New();
    vtkSmartPointer<vtkRenderWindow> renderWindow =
            vtkSmartPointer<vtkRenderWindow>::New();
    renderWindow->AddRenderer(renderer);
    vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor =
            vtkSmartPointer<vtkRenderWindowInteractor>::New();
    renderWindowInteractor->SetRenderWindow(renderWindow);

    renderer->AddActor(actor);
    renderer->SetBackground(.3, .6, .3); // Background color green

    renderWindow->Render();
    renderWindowInteractor->Start();
    //Read the file


    vtkSmartPointer<vtkVertexGlyphFilter> glyphFilter =
            vtkSmartPointer<vtkVertexGlyphFilter>::New();
#if VTK_MAJOR_VERSION <= 5
    glyphFilter->SetInputConnection(polyData->GetProducerPort());
#else
    glyphFilter->SetInputData(polyData);
#endif
    glyphFilter->Update();

    // Visualize

    vtkSmartPointer<vtkPolyDataMapper> mapper =
            vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputConnection(glyphFilter->GetOutputPort());

    vtkSmartPointer<vtkActor> actor =
            vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);

    vtkSmartPointer<vtkRenderer> renderer =
            vtkSmartPointer<vtkRenderer>::New();
    renderer->AddActor(actor);
    renderer->SetBackground(.0, .0, .0); // Background color green

    vtkSmartPointer<vtkRenderWindow> renderWindow =
            vtkSmartPointer<vtkRenderWindow>::New();
    renderWindow->AddRenderer(renderer);

    vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor =
            vtkSmartPointer<vtkRenderWindowInteractor>::New();
    renderWindowInteractor->SetRenderWindow(renderWindow);

    renderWindowInteractor->Start();

}
*/