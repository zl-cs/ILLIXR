#include <map>
#include <fstream>
#include <string>
#include <optional>

#include <opencv2/core/mat.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <eigen3/Eigen/Dense>

#include "csv_iterator.hpp"

typedef unsigned long long ullong;
unsigned count=0;
typedef struct {
	Eigen::Vector3d angular_v;
	Eigen::Vector3d linear_a;
} raw_imu_type;

class lazy_load_image {
public:
	lazy_load_image(const std::string& path)
		: _m_path(path)
	{ }
	std::unique_ptr<cv::Mat> load() const {
		auto img = std::unique_ptr<cv::Mat>{new cv::Mat{cv::imread(_m_path, cv::IMREAD_UNCHANGED)}};
		/* TODO: make this load in grayscale */
		assert(!img->empty());
		return img;
	}
    const std::string get_path()
    {
        return _m_path;
    }
private:
	std::string _m_path;
};

typedef struct {
	std::optional<raw_imu_type> imu0;
	std::optional<lazy_load_image> cam0;
	std::optional<lazy_load_image> cam1;
} sensor_types;

static
std::map<ullong, sensor_types>
load_data() {
	const char* illixr_data_c_str = std::getenv("ILLIXR_DATA");
	if (!illixr_data_c_str) {
		std::cerr << "Please define ILLIXR_DATA" << std::endl;
		abort();
	}
	std::string illixr_data = std::string{illixr_data_c_str};

	std::map<ullong, sensor_types> data;

	const std::string imu0_subpath = "/groundtruth.txt";
   	std::ifstream imu0_file {illixr_data + imu0_subpath};
	if (!imu0_file.good()) {
		std::cerr << "${ILLIXR_DATA}" << imu0_subpath << " (" << illixr_data << imu0_subpath << ") is not a good path" << std::endl;
		abort();
	}
    //for now cam0 is depth
	const std::string cam0_subpath = "/associate_depth.csv";
	std::ifstream cam0_file {illixr_data + cam0_subpath};
	if (!cam0_file.good()) {
		std::cerr << "${ILLIXR_DATA}" << cam0_subpath << " (" << illixr_data << cam0_subpath << ") is not a good path" << std::endl;
		abort();
	}
	std::string unit_conv;
	//not skipping the first line 
	for(CSVIterator row{cam0_file, 0}; row != CSVIterator{}; ++row) {
	    unit_conv = row[0].c_str();
        auto decimal_start = unit_conv.find(".");
        if(decimal_start != std::string::npos) {unit_conv.erase(decimal_start,1);}
        ullong t = std::stoull(unit_conv);
//        std::cout<<"depth t: "<<t<<std::endl;
		data[t].cam0 = {illixr_data +  "/" + row[9]};
	}
	//for now cam1 is color
//	const std::string cam1_subpath = "/depth_rgb.csv";
	const std::string cam1_subpath = "/associate_color.csv";
	std::ifstream cam1_file {illixr_data + cam1_subpath};
	if (!cam1_file.good()) {
		std::cerr << "${ILLIXR_DATA}" << cam1_subpath << " (" << illixr_data << cam1_subpath << ") is not a good path" << std::endl;
		abort();
	}
	//not skipping the first line and we will skip rgb if no match depth found
	for(CSVIterator row{cam1_file, 0}; row != CSVIterator{}; ++row) {
	    unit_conv = row[0].c_str();
        auto decimal_start = unit_conv.find(".");
        if(decimal_start != std::string::npos) {unit_conv.erase(decimal_start,1);}
        ullong t = std::stoull(unit_conv);
//        std::cout<<"color t: "<<t<<std::endl;
        if(data.count(t))
        {
    		data[t].cam1 = {illixr_data + "/" + row[3]};
        }
	}
	std::cout<<"finished data loading"<<std::endl;
	return data;
}
