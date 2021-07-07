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
	std::unique_ptr<cv::Mat> unmodified_load() const {
		auto img = std::unique_ptr<cv::Mat>{new cv::Mat{cv::imread(_m_path, cv::IMREAD_UNCHANGED)}};
		assert(!img->empty());
		return img;
	}
	std::unique_ptr<cv::Mat> modified_load() const {
		//auto img = std::unique_ptr<cv::Mat>{new cv::Mat{cv::imread(_m_path, cv::IMREAD_UNCHANGED)}};
		cv::Mat original_mat = cv::imread(_m_path, cv::IMREAD_UNCHANGED);
		cv::Mat *converted_mat = new cv::Mat();
		cv::cvtColor(original_mat,*converted_mat,cv::COLOR_RGB2RGBA,0);
		auto img = std::unique_ptr<cv::Mat>{converted_mat};
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

	const std::string dataset_subpath = "/final.csv";
	std::ifstream dataset_file {illixr_data + dataset_subpath};
	if (!dataset_file.good()) {
		std::cerr << "${ILLIXR_DATA}" << dataset_subpath << " (" << illixr_data << dataset_subpath << ") is not a good path" << std::endl;
		abort();
	}
	std::string unit_conv;
	//not skipping the first line 
	for(CSVIterator row{dataset_file, 0}; row != CSVIterator{}; ++row) {
	    unit_conv = row[0].c_str();
        auto decimal_start = unit_conv.find(".");
        if(decimal_start != std::string::npos) {unit_conv.erase(decimal_start,1);}
        ullong t = std::stoull(unit_conv);
//        std::cout<<"depth t: "<<t<<std::endl;
		data[t].cam0 = {illixr_data +  "/" + row[9]};
		data[t].cam1 = {illixr_data +  "/" + row[11]};
	}
	std::cout<<"finished data loading"<<std::endl;
	return data;
}
