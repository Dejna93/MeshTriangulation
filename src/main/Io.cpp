#include "include/Io.h"



Io::Io()
{
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
    parseSavingPCD(argc, argv);
    parseVisualisation(argc, argv);
    parseFitlering(argc, argc);

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

        return this->setFilepath(filename);
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
    if (pcl::console::find_argument(argc, argv, "-p") >= 0) {
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
        return 1;
    }
    std::cout << "Filepath does not exists \n";
    return 0;
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


int Io::nr_from_file(std::string filepath) {
//	std::regex wzorzec("([0 - 9])\d+");
	//std::string result = "";
	//if (std::regex_search(filepath, result, wzorzec))
	//{
	//	return std::stoi(result);
	//}
	return 0;
}

int Io::writeToPCD(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, string filename, int number)
{
	pcl::PCDWriter writer;
	return writer.write<pcl::PointXYZ>(filename, *cloud, false); //*
}


int writeToPCDBinary(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, string filename, int number)
{
	pcl::PCDWriter writer;
	return writer.writeBinary<pcl::PointXYZ>(filename, *cloud); //*
}
