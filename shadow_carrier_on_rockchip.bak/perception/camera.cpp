/**
 * camera.cpp — USB摄像头驱动 (V4L2)
 * ====================================
 * 职责: 打开/dev/video设备，持续取MJPG帧，存入共享内存环
 *
 * 依赖: libjpeg-turbo (已装), librga (需sudo)
 *
 * 接口:
 *   Camera(int device_id)          — 打开指定摄像头
 *   bool read_frame(Mat& frame)    — 取一帧（阻塞）
 *   Size get_resolution()          — 获取当前分辨率
 *   void release()                 — 关闭摄像头
 *
 * TODO: 实际实现
 * - V4L2 ioctl 直接取MJPG帧（跳过OpenCV，减少拷贝）
 * - 使用 libjpeg-turbo 硬件解码 JPEG → YUV/RGB
 * - 三缓冲换页，生产-消费者分离（NPU推理线程直接拿最新帧）
 * - 支持动态切换分辨率
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <linux/videodev2.h>

// V4L2 常量（板子上的摄像头规格）
#define CAMERA_DEVICE   "/dev/video10"
#define CAMERA_WIDTH    1280
#define CAMERA_HEIGHT   720
#define CAMERA_FORMAT   V4L2_PIX_FMT_MJPEG
#define BUFFER_COUNT    3             // 三缓冲

class Camera {
public:
    Camera(int device_id = 10);
    ~Camera();
    bool open();
    bool read_frame(unsigned char** data, size_t* len);  // 返回MJPG裸数据
    void release();

private:
    int fd;                          // V4L2文件描述符
    int device_id;
    struct buffer {
        void*  start;
        size_t length;
    } *buffers;
    unsigned int n_buffers;

    bool init_mmap();
    bool start_streaming();
    bool stop_streaming();
};

// 占位实现（后续填坑）
Camera::Camera(int id) : fd(-1), device_id(id), buffers(nullptr), n_buffers(0) {}
Camera::~Camera() { release(); }

bool Camera::open() {
    // TODO: fd = ::open(device_path, O_RDWR | O_NONBLOCK)
    // TODO: ioctl VIDIOC_QUERYCAP
    // TODO: ioctl VIDIOC_S_FMT (set MJPG, 1280x720)
    // TODO: init_mmap()
    // TODO: start_streaming()
    fprintf(stderr, "[camera] TODO: V4L2 init for /dev/video%d\n", device_id);
    return false;
}

bool Camera::read_frame(unsigned char** data, size_t* len) {
    // TODO: ioctl VIDIOC_DQBUF (dequeue filled buffer)
    // TODO: *data = buffers[buf.index].start
    // TODO: *len = buf.bytesused
    // TODO: ioctl VIDIOC_QBUF (requeue buffer after NPU done)
    return false;
}

void Camera::release() {
    if (fd >= 0) {
        stop_streaming();
        // TODO: munmap buffers
        close(fd);
        fd = -1;
    }
}
