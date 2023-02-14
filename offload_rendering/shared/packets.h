#define PING 0x01
#define PONG 0x02

enum ImageType {
    COMPRESSED,
    UNCOMPRESSED
};

struct rendered_frame_header {
    ImageType type;
    int size_left;
    int size_right;
    int rows;
    int cols;
};