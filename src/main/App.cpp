//
// Created by dejna on 14.02.17.
//

#include "include/App.h"

App::App() {
    this->io = new Io();
    this->meshing = Meshing();
}

App::App(int argc, char **argv) : io(NULL) {

    this->io = new Io();

    if (io->input(argc, argv) == 0) {
        io = NULL;
    }
}

bool App::isValid() const {
    return io != NULL;
}

int App::run() {
    meshing = Meshing(io->loadPCD());
    meshing.setDao(io->getDao());

    if (io->getDao().getBoolAttribute("show_loaded")) {
        Visualization visualization = Visualization();
        visualization.view(meshing.getCloud());
    }

    // std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> test;
    // for (int i=0 ; i <3 ; i++){
    //    test.push_back(meshing.getCloud());
    //  }
    //Visualization visualization = Visualization();
    // visualization.view(meshing.getCloud());
    // visualization.view_visaulization(visualization.simpleVis(meshing.getCloud()));
    //  std::cout << "View\n";

    return 1;

}