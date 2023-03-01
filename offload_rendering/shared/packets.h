#define PING 0x01
#define PONG 0x02

enum ImageType {
    COMPRESSED,
    UNCOMPRESSED
};

struct pose_transfer {
    ILLIXR::pose_type pose;
    long predict_computed_time;
    long predict_target_time;
};

struct rendered_frame_header {
    ImageType type;
    int size_left;
    int size_right;
    int rows;
    int cols;

    pose_transfer pose;
    long sample_time;
    long render_time;
};