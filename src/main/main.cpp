// --------------
// -----Main-----
// --------------
#include "include/App.h"


int
main(int argc, char **argv) {
    /*
     * Io ioManager = Io();
    if (ioManager.input(argc, argv)) {
        //ioManager.getDao().print();
        Meshing meshCreator = Meshing(ioManager);
        meshCreator.setInputCloud(ioManager.loadPCD());
        meshCreator.run_calculation();
    }
    return 0;
    */



    App app = App(argc, argv);
    if (app.isValid()) {
        app.run();
    }

}
