//
// Created by dejna on 14.02.17.
//

#ifndef MESH_TRIANGULATION_APP_H
#define MESH_TRIANGULATION_APP_H

#include "Io.h"
#include "Meshing.h"
#include "include/Visualization.h"


class App {

public:
    App();

    App(int argc, char **argv);
    //App(Io io);

    bool isValid() const;

    int run();

private:
    Io *io;
    Meshing meshing;

};

#endif //MESH_TRIANGULATION_APP_H
