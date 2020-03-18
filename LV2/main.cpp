/**
* Created by sbecirovic
* Laboratorijska vjezba 2
* Improved viewer
*
*/


#include "improved_viewer.h"
#include <easy3d/viewer/model.h>
#include <easy3d/viewer/drawable_points.h>
#include <easy3d/viewer/setting.h>
#include <easy3d/util/logging.h>


using namespace easy3d;

// This example shows how to
//		- estimation normal information of a point cloud.
//      - re-orientation the normals.


int main(int argc, char** argv) {
    // Initialize logging.
    logging::initialize(argv[0]);
    try {
        // Kreiranje viewera
        ImprovedViewer viewer("LV2ImprovedViewer");

      //  viewer.ucitajLinijeIzDatoteke("testlinije.txt", "indices.txt");
       // viewer.add_drawable(viewer.linije);

        viewer.fit_screen();
        // Run the viewer
        viewer.run();
        return EXIT_SUCCESS;

    } catch (const std::runtime_error &e) {
        LOG(ERROR) << "Caught a fatal error: " + std::string(e.what());
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;

}

