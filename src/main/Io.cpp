#include "include/Io.h"



Io::Io()
{
    this->poissonDao = PoissonDao();
}

Io::~Io()
{

}

int Io::input(int argc, char **argv) {

    if (!parseInital(argc, argv)) {
        return 0;
    }
    if (!parseFilepath(argc, argv)) {
        return 0;
    }

    if (!parseMethodSmoothing(argc, argv)) {
        return 0;
    }
    parseParams(argc, argv);
    parseSavingPCD(argc, argv);
    parseVisualisation(argc, argv);
    parseFitlering(argc, argv);

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

int Io::parseMethodSmoothing(int argc, char **argv) {
    int method = 0;
    if (pcl::console::parse_argument(argc, argv, "-s", method) != -1) {
        this->setSmoothinMethod(method);
        return 1;
    }
    return 0;
}

void Io::parseSavingPCD(int argc, char **argv) {
    if (pcl::console::find_argument(argc, argv, "-t") >= 0) {
        this->setSavingPcd(1);
    }
    this->setSavingPcd();
}

void Io::parseVisualisation(int argc, char **argv) {
    if (pcl::console::find_argument(argc, argv, "-v") >= 0) {
        this->setShowVisualisation(1);
    }
    this->setShowVisualisation();
}

void Io::parseFitlering(int argc, char **argv) {
    if (pcl::console::find_argument(argc, argv, "-r") >= 0) {
        this->setFiltering(1);
    }
    this->setFiltering();
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

    for (auto &section : propTree) {
            for (auto &key : section.second) {
                std::string value = key.second.get_value<string>();
                if (section.first == "poisson") {
                    poissonDao.loadParams(key.first, value);
                }
                if (section.first == "options"){
                    optionDao.loadParams(key.first, value);
                }
                if (section.first == "filtering") {
                    filteringDao.loadParams(key.first, value);
                }

            }
    }
    optionDao.print();
    poissonDao.print();
}

void Io::setShowVisualisation(int visualisation) {
    this->show_visualisation = (visualisation != 0);
}

void Io::setSmoothinMethod(int method) {
    if (method >= 0 && method <= 2) {
        this->smoothing_method = method;
    } else {
        // default poisson
        this->smoothing_method = 0;
    }
}

void Io::setSavingPcd(int saving) {
    this->save_pcd = (saving != 0);
}

void Io::setFiltering(int filtering) {
    this->filtering = (filtering != 0);
}

boost::filesystem::path Io::getFilepath() {
    return filepath;
}

int Io::getSmoothingMethod() {
    return this->smoothing_method;
}

bool Io::isSavePCD() {
    return this->save_pcd;
}

bool Io::isFiltering() {
    return this->filtering;
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
    stringstream strStream;

    for (auto it = file.rbegin(); it != file.rend(); ++it) {
        if (isdigit(*it)) {
            strStream << *it;
        }
    }
    return boost::lexical_cast<int>(strStream.str());
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

int Io::writeToPCD(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, boost::filesystem::path path) {
    pcl::PCDWriter writer;
    boost::filesystem::path created_folder = createFolder(path);
    writer.write<pcl::PointXYZ>((created_folder /= path.filename()).string(), *cloud, false);
    return 1;
}

int Io::writeToPCD(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {

    return writeToPCD(cloud, this->filepath);
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
         << "\n Smooting \t" << smoothing_method
         << "\n Filtering \t" << filtering
         << "\n Visualisation \t" << show_visualisation
         << "\n Save PCD \t" << save_pcd
         << "\n\n";
    setPointFolder(current_folder);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr Io::loadPCD() {

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PCLPointCloud2 cloud_blob;
    pcl::io::loadPCDFile(getFilepath().string(), cloud_blob);
    pcl::fromPCLPointCloud2(cloud_blob, *cloud);
    return cloud;
}

FilteringDao Io::getFilterDao() {
    return this->filteringDao;
}

OptionDao Io::getOptionDao() {
    return this->optionDao;
}

PoissonDao Io::getPoissonDao() {
    return this->poissonDao;
}


int writeToPCDBinary(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, string filename, int number)
{
	pcl::PCDWriter writer;
	return writer.writeBinary<pcl::PointXYZ>(filename, *cloud); //*
}
