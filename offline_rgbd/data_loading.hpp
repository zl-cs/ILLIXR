#include "common/error_util.hpp"
#include "csv_iterator.hpp"

#include <cassert>
#include <eigen3/Eigen/Dense>
#include <fstream>
#include <map>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <optional>
#include <string>

typedef unsigned long long ullong;

typedef struct {
    Eigen::Vector3d angular_v;
    Eigen::Vector3d linear_a;
} raw_imu_type;

class lazy_load_image {
public:
    lazy_load_image() { }
    lazy_load_image(const std::string& path)
        : _m_path(path) { }

    cv::Mat load() const {
        auto img = cv::imread(_m_path, cv::IMREAD_UNCHANGED);
        // assert(!img->empty());
		// cv::resize(*img, *img, cv::Size(720, 405));
        return img;
    }

    cv::Mat load_depth() const {
        auto img = cv::imread(_m_path, cv::IMREAD_ANYDEPTH);
        // assert(!img->empty());
        // cv::resize(*img, *img, cv::Size(720, 405));
        return img;
    }


private:
    std::string _m_path;
};

typedef struct {
    lazy_load_image rgb;
    lazy_load_image depth;
} sensor_types;

static std::map<ullong, sensor_types> load_data() {
    // const char* illixr_data_c_str = std::getenv("ILLIXR_DATA");
    // if (!illixr_data_c_str) {
    //  std::cerr << "Please define ILLIXR_DATA" << std::endl;
    //     ILLIXR::abort();
    // }
    // std::string illixr_data = std::string{illixr_data_c_str};

    std::string illixr_data = "/home/henrydc/mnt/tinker/dataset/table3";
    // std::string illixr_data = "path-to-aligned-sequence";

    std::map<ullong, sensor_types> data;

    const std::string cam1_subpath = "/associate/rgb.txt";
    std::ifstream cam1_file {illixr_data + cam1_subpath};
    if (!cam1_file.good()) {
        std::cerr << "${ILLIXR_DATA}" << cam1_subpath << " (" << illixr_data << cam1_subpath << ") is not a good path" << std::endl;
        ILLIXR::abort();
    }
    for (CSVIterator row{cam1_file, 1}; row != CSVIterator{}; ++row) {
        ullong t = std::stoull(row[0]);
        std::string fname = row[1];
        data[t].rgb = {illixr_data + "/" + row[1]};
        std::cout << "[DEBUG] Loaded right cam: "
                 << t << " "
                 << illixr_data + "/" + row[1] << "\n";
    }

    const std::string depth_subpath = "/associate/depth.txt";
    std::ifstream depth_file {illixr_data + depth_subpath};
    if (!depth_file.good()) {
        std::cerr << "${ILLIXR_DATA}" << depth_subpath << " (" << illixr_data << depth_subpath << ") is not a good path" << std::endl;
        ILLIXR::abort();
    }
    for (CSVIterator row{depth_file, 1}; row != CSVIterator{}; ++row) {
        ullong t = std::stoull(row[0]);
        std::string fname = row[1];
        data[t].depth = {illixr_data + "/" + row[1]};
        // std::cout << "[DEBUG] Loaded depth cam: "
        //          << t << " "
        //          << illixr_data + "/" + row[1] << "\n";
    }
    return data;
}