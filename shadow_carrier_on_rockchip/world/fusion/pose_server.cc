// pose_server.cc —— YOLOv8-Pose 常驻旁路服务
// 用法: pose_server <pose.rknn> [in_jpg] [out_json]
// 默认: /dev/shm/pose_in.jpg -> /dev/shm/pose_out.json
// 输出(按 [HRI] 定稿的派生量): {"ts","infer_ms","persons":[{bbox,conf,facing,offering,arms_raised}]}
// facing: front/back/left/right (相对相机); offering: 手臂伸向相机前方/递出
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/stat.h>
#include <time.h>
#include <math.h>
#include "yolov8-pose.h"
#include "file_utils.h"
#include "image_utils.h"

static double now_ms() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1000.0 + ts.tv_nsec / 1e6;
}

// COCO: 0 nose,1 Leye,2 Reye,3 Lear,4 Rear,5 Lsho,6 Rsho,7 Lelb,8 Relb,9 Lwr,10 Rwr,11 Lhip,12 Rhip
static const char* facing_of(object_detect_result* r) {
    float face = (r->keypoints[0][2] + r->keypoints[1][2] + r->keypoints[2][2]
                + r->keypoints[3][2] + r->keypoints[4][2]) / 5.0f;
    float bw = r->box.right - r->box.left;
    float sh_dx = fabsf(r->keypoints[5][0] - r->keypoints[6][0]);
    if (face > 0.40f && (bw <= 1 || sh_dx > 0.30f * bw)) return "front";
    if (face < 0.15f) return "back";
    float mid_sh = (r->keypoints[5][0] + r->keypoints[6][0]) / 2.0f;
    return (r->keypoints[0][0] < mid_sh) ? "left" : "right";
}

static int offering_of(object_detect_result* r) {
    float bh = r->box.bottom - r->box.top;
    if (bh < 60) return 0;
    // 手腕高于肩+15%身高 => 手抬到胸前以上 (递物/伸臂候选)
    float thr_l = r->keypoints[5][1] + 0.15f * bh;
    float thr_r = r->keypoints[6][1] + 0.15f * bh;
    int l_up = r->keypoints[9][2] > 0.30f && r->keypoints[9][1] < thr_l;
    int r_up = r->keypoints[10][2] > 0.30f && r->keypoints[10][1] < thr_r;
    return (l_up || r_up) ? 1 : 0;
}

static int arms_raised_of(object_detect_result* r) {
    float bh = r->box.bottom - r->box.top;
    if (bh < 60) return 0;
    float thr = r->box.top + 0.35f * bh;   // 手腕进入人体上部 35%
    int l = r->keypoints[9][2] > 0.30f && r->keypoints[9][1] < thr;
    int rr = r->keypoints[10][2] > 0.30f && r->keypoints[10][1] < thr;
    return (l || rr) ? 1 : 0;
}

int main(int argc, char** argv) {
    if (argc < 2) {
        printf("%s <pose.rknn> [in_jpg] [out_json]\n", argv[0]);
        return -1;
    }
    const char* model = argv[1];
    const char* in_jpg = argc > 2 ? argv[2] : "/dev/shm/pose_in.jpg";
    const char* out_json = argc > 3 ? argv[3] : "/dev/shm/pose_out.json";

    rknn_app_context_t ctx;
    memset(&ctx, 0, sizeof(ctx));
    if (init_post_process() != 0) { printf("post_process init fail\n"); }
    if (init_yolov8_pose_model(model, &ctx) != 0) { printf("init pose fail\n"); return -1; }

    printf("pose_server ready, watching %s -> %s\n", in_jpg, out_json);
    fflush(stdout);

    time_t last_s = 0;
    long last_ns = 0;
    while (1) {
        struct stat st;
        if (stat(in_jpg, &st) == 0 &&
            (st.st_mtim.tv_sec != last_s || st.st_mtim.tv_nsec != last_ns)) {
            last_s = st.st_mtim.tv_sec;
            last_ns = st.st_mtim.tv_nsec;
            image_buffer_t img; memset(&img, 0, sizeof(img));
            if (read_image(in_jpg, &img) == 0) {
                object_detect_result_list od; memset(&od, 0, sizeof(od));
                double t0 = now_ms();
                int ret = inference_yolov8_pose_model(&ctx, &img, &od);
                double dt = now_ms() - t0;
                if (ret == 0) {
                    FILE* fo = fopen(out_json, "w");
                    if (fo) {
                        fprintf(fo, "{\"ts\": %.3f, \"infer_ms\": %.1f, \"persons\": [",
                                (double)time(NULL), dt);
                        for (int i = 0; i < od.count; i++) {
                            object_detect_result* r = &od.results[i];
                            fprintf(fo, "%s{\"bbox\": [%d, %d, %d, %d], \"conf\": %.3f, "
                                        "\"facing\": \"%s\", \"offering\": %d, \"arms_raised\": %d}",
                                    i ? ", " : "",
                                    r->box.left, r->box.top, r->box.right, r->box.bottom,
                                    r->prop, facing_of(r), offering_of(r), arms_raised_of(r));
                        }
                        fprintf(fo, "]}\n");
                        fclose(fo);
                    }
                    printf("infer %.1f ms, %d persons\n", dt, od.count);
                    fflush(stdout);
                } else {
                    printf("infer fail ret=%d\n", ret);
                }
                if (img.virt_addr) free(img.virt_addr);
            }
        }
        usleep(50000);
    }
    return 0;
}
