
#include "include/Io.h"


Io::Io() {
    this->dao = Dao();
}


int Io::input(int argc, char **argv) {

    if (!parseInital(argc, argv)) {
        return 0;
    }
    if (!parseFilepath(argc, argv)) {
        return 0;
    }
    return parseParams(argc, argv);

}

int Io::parseInital(int argc, char **argv) {

    if (pcl::console::find_argument(argc, argv, "-h") >= 0 || argc == 1) {
        printUsage(argv[0]);
        return 0;
    }
    return 1;
}

int Io::parseFilepath(int argc, char **argv) {
    std::string filename("");
    if (pcl::console::parse_argument(argc, argv, "-f", filename) != -1) {
        std::cout << filename;
        return this->setFilepath(filename);

    }
    return 0;
}

int Io::parseParams(int argc, char **argv) {
    std::string filename("");
    if (pcl::console::parse_argument(argc, argv, "-p", filename) != -1) {
        this->setParams(filename);
        return 1;
    }
    return 0;
}


int Io::setFilepath(std::string filepath) {
    boost::filesystem::path tmp{filepath};
    if (boost::filesystem::exists(tmp)) {
        this->filepath = tmp;
        this->current_folder = this->filepath.parent_path();
        this->setPointFolder(this->current_folder);
        this->setSTLFolder(this->current_folder);
        return 1;
    }
    std::cout << "Filepath does not exists \n";
    return 0;
}

void Io::setParams(std::string filepath) {

    namespace pt = boost::property_tree;
    pt::ptree propTree;

    boost::property_tree::ini_parser::read_ini(filepath, propTree);

    std::cout << "LOADER \n";
    for (auto &section : propTree) {
        for (auto &key : section.second) {

            std::string value = key.second.get_value<string>();
            //if (section.first == "poisson") {
            // std::cout << key.first << " " << value << std::endl;
            dao.loadParams(key.first, value);
            //  }
            //  if (section.first == "options"){
            //      optionDao.loadParams(key.first, value);
            //  }
            //  if (section.first == "filtering") {
            //       filteringDao.loadParams(key.first, value);
            //   }
        }
    }


    if (dao.getStringAttribute("points") != "") {
        setPointFolder(dao.getStringAttribute("points"));
    }
    if (dao.getStringAttribute("stl") != "") {
        setSTLFolder(dao.getStringAttribute("stl"));
    }

}


boost::filesystem::path Io::getFilepath() {
    return filepath;
}


boost::filesystem::path Io::createFolder(std::string filepath) {
    boost::filesystem::path path(filepath);
    return this->createFolder(path);
}

boost::filesystem::path Io::createFolder(boost::filesystem::path filepath) {
    boost::filesystem::path folder_path = filepath.parent_path();

    if (boost::filesystem::is_directory(folder_path)) {
        folder_path /= boost::lexical_cast<std::string>(id_from_file(filepath));
        cout << "folder_path " << folder_path.string() << "\n";
        create_folder(folder_path);
        return folder_path;
    }
    return folder_path;
}


int Io::id_from_file(boost::filesystem::path filepath) {

    std::string file = filepath.string();
    std::regex integer("(\\+|-)?[[:digit:]]+");
    std::smatch output;

    std::regex_search(file, output, integer);

    if (output.str() != "") {
        return boost::lexical_cast<int>(output.str());
    }

    // default folder
    return 0;

}


void Io::printUsage(const char *name) {
    std::cout << "\n\nUsage: " << name << " [options]\n\n"
              << "Options:\n"
              << "-------------------------------------------\n"
              << "-f           path to PCD file\n"
              << "-v           show visualisation of point cloud\n"
              << "-s           method of smoothing 0-poisson 1-vtk \n"
              << "-p           save clusters cloud to pcd\n"
              << "-r           filter cloud \n"
              << "\n";
}


int Io::create_folder(boost::filesystem::path path) {
    if (!boost::filesystem::exists(path)) {
        int result = boost::filesystem::create_directory(path);
        if (result) {
            this->current_folder = path;
        }
        return result;
    }
    return 0;
}

void Io::setPointFolder(boost::filesystem::path filepath) {
    if (boost::filesystem::exists(filepath)) {
        if (boost::filesystem::is_directory(filepath) && filepath.filename() == "points") {
            this->point_folder = filepath;
        } else {
            boost::filesystem::path stl_tmp = filepath.parent_path() /= "points";
            if (boost::filesystem::exists(stl_tmp)) {
                this->stl_folder = stl_tmp;
            } else {
                this->stl_folder = createFolder(stl_tmp);
            }
        }
    }

}

void Io::setSTLFolder(boost::filesystem::path filepath) {
    if (boost::filesystem::exists(filepath)) {
        if (boost::filesystem::is_directory(filepath) && filepath.filename() == "stl") {
            this->stl_folder = filepath;
        } else {
            boost::filesystem::path stl_tmp = filepath.parent_path() /= "stl";
            if (boost::filesystem::exists(stl_tmp)) {
                this->stl_folder = stl_tmp;
            } else {
                this->stl_folder = createFolder(stl_tmp);
            }
        }
    }
}

int Io::isFolderByName(std::string name, boost::filesystem::path folder) {
    return 0;
}

void Io::printStack() {
    cout << "Filepath \t" << filepath.string()
         << "\n Current folder\t" << current_folder.string()
         << "\n Points folder\t" << point_folder.string()
         << "\n Stl folder\t" << stl_folder.string()
         << "\n\n";
    setPointFolder(current_folder);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr Io::loadPCD() {
    return loadPCD(this->getFilepath());
}

pcl::PointCloud<pcl::PointXYZ>::Ptr Io::loadPCD(boost::filesystem::path filepath) {
    return loadPCD(filepath.string());
}

pcl::PointCloud<pcl::PointXYZ>::Ptr Io::loadPCD(std::string filepath) {

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PCLPointCloud2 cloud_blob;
    pcl::io::loadPCDFile(filepath, cloud_blob);
    pcl::fromPCLPointCloud2(cloud_blob, *cloud);
    return cloud;
}

int Io::saveSTL(pcl::PolygonMesh output_mesh) {

    return saveSTL(0, output_mesh);
}

int Io::saveSTL(int index, pcl::PolygonMesh output_mesh) {
    filesys::path new_stl = this->makePathToNewSTL(this->filepath.string());

    if (new_stl.string() != "") {
        new_stl /= join(this->filepath.filename(), index) + ".stl";
        std::cout << "Saving to " << new_stl.string() << "\n";
        return pcl::io::savePolygonFileSTL(new_stl.string(), output_mesh);
    }
    std::cout << "Eror saving\n";
    return 0;
}

boost::filesystem::path Io::getStlFolder() {
    return stl_folder;
}


Dao Io::getDao() {
    return dao;
}

void Io::setPointFolder(std::string folder) {
    filesys::path path(folder);
    if (filesys::exists(folder) && filesys::is_directory(folder)) {
        this->point_folder = folder;
    }
}

void Io::setSTLFolder(std::string folder) {
    filesys::path path(folder);
    if (filesys::exists(folder) && filesys::is_directory(folder)) {
        this->stl_folder = folder;
    }
}

filesys::path Io::makePathToNewPoint(std::string filepath) {
    filesys::path point_path = this->point_folder;

    point_path /= boost::lexical_cast<std::string>(id_from_file(filepath));
    create_folder(point_path);

    return point_path;
}

filesys::path Io::makePathToNewSTL(std::string filepath) {
    filesys::path stl_path = this->stl_folder;

    stl_path /= boost::lexical_cast<std::string>(id_from_file(filepath));
    create_folder(stl_path);

    return stl_path;
}

std::string Io::join(std::string name, int id) {
    return name + boost::lexical_cast<std::string>(id);
}

std::string Io::join(filesys::path name, int id) {
    return name.stem().string() + "_" + boost::lexical_cast<std::string>(id);
}

int Io::savePCD(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    return savePCD(cloud, 0);
}

int Io::savePCD(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int index) {
    filesys::path new_point = this->makePathToNewPoint(this->filepath.string());
    std::cout << new_point.string();
    if (new_point.string() != "") {
        new_point /= join(this->filepath.filename(), index) + ".pcd";

        // return pcl::io::savePolygonFileSTL(new_stl.string(), output_mesh);
        return pcl::io::savePCDFileASCII(new_point.string(), *cloud);
    }
    return 0;

}


int writeToPCDBinary(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, string filename, int number) {
    pcl::PCDWriter writer;
    return writer.writeBinary<pcl::PointXYZ>(filename, *cloud); //*
}
