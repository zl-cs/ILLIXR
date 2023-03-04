#define PING 0x01
#define PONG 0x02

enum ImageType {
    COMPRESSED,
    UNCOMPRESSED
};

struct pose_transfer {
//    ILLIXR::pose_type pose;

    ILLIXR::time_point         sensor_time; // Recorded time of sensor data ingestion
    Eigen::Vector3f    position;
    Eigen::Quaternionf orientation;

    long predict_computed_time;
    long predict_target_time;

    static pose_transfer serialize(ILLIXR::pose_type pose, long predict_computed_time, long predict_target_time) {
        pose_transfer pt;
        pt.sensor_time = pose.sensor_time;
        pt.position = pose.position;
        pt.orientation = pose.orientation;

        pt.predict_computed_time = predict_computed_time;
        pt.predict_target_time = predict_target_time;
        return pt;
    }

     ILLIXR::pose_type deserialize() {
        ILLIXR::pose_type pose;
        pose.sensor_time = sensor_time;
        pose.position = position;
        pose.orientation = orientation;
        return pose;
    }
};

struct rendered_frame_header {
    ImageType type;
    int size_left;
    int size_right;

    pose_transfer pose;
    long sample_time;
    long render_time;
};