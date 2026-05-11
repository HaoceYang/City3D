//
// Created by Jin on 19/07/2022.
//
/*
Copyright (C) 2017  Liangliang Nan
https://3d.bk.tudelft.nl/liangliang/ - liangliang.nan@gmail.com

This program is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public License
as published by the Free Software Foundation; either version 2
of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
*/

#include "../model/point_set.h"
#include "../model/map.h"
#include "../model/map_io.h"
#include "../model/point_set_io.h"
#include "../method/method_global.h"
#include "../method/reconstruction.h"
#include "../basic/file_utils.h"


namespace {

bool reconstruct_file(
        const std::string& input_cloud_file,
        const std::string& result_file,
        const std::string& footprint_file
) {
    std::cout << "\tread input point cloud from file: " << input_cloud_file << std::endl;

    std::cout << "\tloading input point cloud data..." << std::endl;
    PointSet *pset = PointSetIO::read(input_cloud_file);
    if (!pset) {
        std::cerr << "\tfailed loading point cloud data from file: " << input_cloud_file << std::endl;
        return false;
    }

    Reconstruction recon;

    Map *footprint = recon.generate_footprint(pset);
    MapIO::save(footprint_file, footprint);

    std::cout << "\tsegmenting individual buildings..." << std::endl;
    recon.segmentation(pset, footprint);

    std::cout << "\textracting roof planes..." << std::endl;
    if (!recon.extract_roofs(pset, footprint)) {
        std::cerr << "\tno roofs could be extracted from the point cloud (" << input_cloud_file << ")" << std::endl;
        delete pset;
        delete footprint;
        return false;
    }

    Map *result = new Map;
#ifdef HAS_GUROBI
    std::cout << "\treconstructing the buildings (using the Gurobi solver)..." << std::endl;
    bool status = recon.reconstruct(pset, footprint, result, LinearProgramSolver::GUROBI);
#else
    std::cout << "\treconstructing the buildings (using the SCIP solver)..." << std::endl;
    bool status = recon.reconstruct(pset, footprint, result, LinearProgramSolver::SCIP);
#endif

    bool saved = false;
    if (status && result->size_of_facets() > 0) {
        if (MapIO::save(result_file, result)) {
            std::cout << "\treconstruction result saved to file: " << result_file << std::endl;
            saved = true;
        }
        else {
            std::cerr << "\tfailed to save reconstruction result to file: " << result_file << std::endl;
        }
    }
    else {
        std::cerr << "\treconstruction failed. Input point cloud from file: " << input_cloud_file << std::endl;
    }

    delete pset;
    delete footprint;
    delete result;
    return saved;
}

void print_usage(const char* executable) {
    std::cerr
        << "Usage:\n"
        << "  " << executable << "\n"
        << "      Process all .ply files in City3D/data/building_instances.\n"
        << "  " << executable << " <input.ply> <output.obj>\n"
        << "      Process one PLY point cloud and write one OBJ reconstruction.\n";
}

}

int main(int argc, char **argv) {
    ///ToDo: user may need to tune these parameters for their datasets
    Method::min_points = 40;
    Method::pixel_size = 0.15;

    if (argc == 3) {
        const std::string input_cloud_file = argv[1];
        const std::string result_file = argv[2];
        const std::string footprint_file = FileUtils::name_less_extension(result_file) + "_GeneratedFootprint.obj";

        if (FileUtils::extension_in_lower_case(input_cloud_file) != "ply") {
            std::cerr << "input file must be a .ply point cloud: " << input_cloud_file << std::endl;
            return EXIT_FAILURE;
        }
        if (FileUtils::extension_in_lower_case(result_file) != "obj") {
            std::cerr << "output file must be an .obj mesh: " << result_file << std::endl;
            return EXIT_FAILURE;
        }

        return reconstruct_file(input_cloud_file, result_file, footprint_file)
            ? EXIT_SUCCESS
            : EXIT_FAILURE;
    }

    if (argc != 1) {
        print_usage(argv[0]);
        return EXIT_FAILURE;
    }

    const std::string directory = std::string(CITY3D_ROOT_DIR) + "/../data/building_instances";
    //get the file names of the input point clouds
    std::vector<std::string> all_file_names;
    FileUtils::get_files(directory, all_file_names, false);
    for (std::size_t i = 0; i < all_file_names.size(); ++i) {
        std::cout << "- processing " << i + 1 << "/" << all_file_names.size() << " file..." << std::endl;
        const std::string &file_name = all_file_names[i];

        if (file_name.find("ply") != std::string::npos) {

            const std::string input_cloud_file = file_name;
            std::string result_file = file_name.substr(0, file_name.find(".ply")) + "_ReconstructedModel.obj";
            std::string footprint_file = file_name.substr(0, file_name.find(".ply")) + "_GeneratedFootprint.obj";

            reconstruct_file(input_cloud_file, result_file, footprint_file);
        }
    }

    return EXIT_SUCCESS;
}
