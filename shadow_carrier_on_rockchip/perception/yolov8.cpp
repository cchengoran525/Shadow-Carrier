/**
 * yolov8.cpp — NPU YOLOv8推理封装
 * ================================
 * 职责: 加载RKNN模型，接收图像帧，返回检测结果
 *
 * 底层引擎: 复用 shopping_car_vision 中已验证的 rknn_yolov8_demo
 *           核心代码在 ../shopping_car_vision/rknn_model_zoo/examples/yolov8/cpp/
 *
 * 接口:
 *   bool load_model(const char* path)         — 加载.rknn模型到NPU
 *   vector<Detection> infer(const Mat& frame) — 推理一帧，返回检测列表
 *   void release()                            — 卸载模型
 *
 * Detection 结构:
 *   { class_name, x1, y1, x2, y2, confidence }
 *
 * 注意:
 *   - 模型只需加载一次，之后infer()可反复调用（不要每次subprocess！）
 *   - 输入必须是640x640 RGB
 *   - NPU推理是异步的，可以pipeline摄像头取帧和推理
 *
 * TODO: 实际实现
 * - 把现有rknn_yolov8_demo的main.cc改成库调用（去掉main函数，暴露API）
 * - 或者直接链接现有的.o文件，只写个薄封装
 * - 输入预处理（resize+letterbox）放在调用侧或用RGA硬件加速
 */

#include <vector>
#include <string>
#include "rknn_api.h"

// === 复用现有代码的头文件（相对路径指向原项目） ===
// #include "../../shopping_car_vision/rknn_model_zoo/examples/yolov8/cpp/yolov8.h"
// #include "../../shopping_car_vision/rknn_model_zoo/examples/yolov8/cpp/postprocess.h"

// 检测结果结构体（和postprocess.h中的一致）
typedef struct {
    int x1, y1, x2, y2;          // 边界框（原始图像坐标系）
    float confidence;             // 置信度 0~1
    int class_id;                 // 类别ID (0=person)
    std::string class_name;       // 类别名
} Detection;

class YoloDetector {
public:
    YoloDetector();
    ~YoloDetector();

    // 加载模型（只需调用一次）
    bool load_model(const char* model_path, const char* labels_path);

    // 推理一帧（可反复调用）
    // frame: RGB格式，会被内部resize到640x640
    // 返回: 检测到的目标列表
    std::vector<Detection> infer(const unsigned char* frame_data,
                                  int width, int height, int channels);

    // 释放资源
    void release();

    // 设置阈值
    void set_conf_threshold(float conf)   { conf_threshold = conf; }
    void set_nms_threshold(float nms)     { nms_threshold = nms; }

private:
    rknn_context ctx;                     // NPU上下文
    bool model_loaded;
    float conf_threshold;
    float nms_threshold;

    // 图像预处理
    bool preprocess(const unsigned char* src, int w, int h, int c,
                    unsigned char* dst, int dst_w, int dst_h);

    // 后处理（调用现有postprocess.cc的逻辑）
    std::vector<Detection> postprocess(void* outputs[], int output_count,
                                        int original_w, int original_h);
};

// 占位实现
YoloDetector::YoloDetector()
    : ctx(0), model_loaded(false),
      conf_threshold(0.25f), nms_threshold(0.45f) {}

YoloDetector::~YoloDetector() { release(); }

bool YoloDetector::load_model(const char* model_path, const char* labels_path) {
    // TODO: 调用 init_yolov8_model() 加载模型到NPU
    // TODO: 调用 init_post_process() 加载标签
    // 关键: 模型加载后 ctx 常驻内存，不要每次推理都加载！
    fprintf(stderr, "[yolov8] TODO: load model from %s\n", model_path);
    return false;
}

std::vector<Detection> YoloDetector::infer(
    const unsigned char* frame_data, int w, int h, int c) {
    std::vector<Detection> results;
    if (!model_loaded) {
        fprintf(stderr, "[yolov8] ERROR: model not loaded!\n");
        return results;
    }

    // TODO: 1. preprocess (resize+letterbox 640x640)
    // TODO: 2. rknn_inputs_set (把预处理后的数据送入NPU)
    // TODO: 3. rknn_run (触发NPU推理，~15-20ms)
    // TODO: 4. rknn_outputs_get (拿推理结果)
    // TODO: 5. postprocess (解析输出tensor → Detection列表)
    // TODO: 6. rknn_outputs_release (释放输出)

    return results;
}

void YoloDetector::release() {
    if (model_loaded) {
        // TODO: release_yolov8_model()
        // TODO: deinit_post_process()
        model_loaded = false;
    }
}
