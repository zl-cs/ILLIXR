#include <random>
#include "common/data_format.hpp"

using namespace ILLIXR;

class Deviation {
public:
    static void add_gaussian_to_pos(pose_type &pose, double min, double max) {
        pose.position.x() += get_gaussian(min, max);
        pose.position.y() += get_gaussian(min, max);
        pose.position.z() += get_gaussian(min, max);
    }
    
    static void add_gaussian_to_pos(fast_pose_type &fast_pose, double min, double max) {
        add_gaussian_to_pos(fast_pose.pose, min, max);
    }

    static void add_uniform_to_pos(pose_type &pose, double min, double max) {
        pose.position.x() += get_uniform(min, max);
        pose.position.y() += get_uniform(min, max);
        pose.position.z() += get_uniform(min, max);
    }
    
    static void add_uniform_to_pos(fast_pose_type &fast_pose, double min, double max) {
        add_uniform_to_pos(fast_pose.pose, min, max);
    }

    static void add_gaussian_to_ori(pose_type &pose, double mean, double sigma) {
        pose.orientation.x() += get_gaussian(mean, sigma);
        pose.orientation.y() += get_gaussian(mean, sigma);
        pose.orientation.z() += get_gaussian(mean, sigma);
    }

    static void add_gaussian_to_ori(fast_pose_type &fast_pose, double mean, double sigma) {
        add_gaussian_to_ori(fast_pose.pose, mean, sigma);
    }

    static void add_uniform_to_ori(pose_type &pose, double min, double max) {
        pose.orientation.x() += get_uniform(min, max);
        pose.orientation.y() += get_uniform(min, max);
        pose.orientation.z() += get_uniform(min, max);
    }

    static void add_uniform_to_ori(fast_pose_type &fast_pose, double min, double max) {
        add_uniform_to_ori(fast_pose.pose, min, max);
    }
private:
    static double get_gaussian(double mean, double sigma) {
        std::random_device rd;
        std::mt19937 gen(rd()); 
        std::normal_distribution<double> dis(mean, sigma);
        return dis(gen);
    }

    static double get_uniform(double min, double max) {
        std::random_device rd;
        std::mt19937 gen(rd()); 
        std::uniform_real_distribution<double> dis(min, max);
        return dis(gen);
    }
};