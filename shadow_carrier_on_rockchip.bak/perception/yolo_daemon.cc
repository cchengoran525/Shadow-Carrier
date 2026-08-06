/**
 * yolo_daemon.cc - YOLO NPU 常驻推理进程
 * 模型加载一次，循环从stdin读图像路径，JSON结果输出到stderr
 * v3: 修复 stdout 污染问题
 *   - rknn库(rknn_model_zoo utils)的调试printf全部写到stdout, 会污染JSON管道
 *   - 修复: stdout重定向到/dev/null丢弃库输出, JSON改走stderr隔离
 * v4: out.jpg 写 tmp + rename 原子替换
 *   - 避免 consumer 读到写一半的 out.jpg 导致花屏
 * 编译: 见下方 compile 命令
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/time.h>
#include "yolov8.h"
#include "image_utils.h"
#include "file_utils.h"
#include "image_drawing.h"

static double now_ms() {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return tv.tv_sec * 1000.0 + tv.tv_usec / 1000.0;
}

int main(int argc, char **argv) {
    if (argc < 2) {
        fprintf(stderr, "Usage: %s <model_path>\n", argv[0]);
        return 1;
    }
    const char *model_path = argv[1];

    // v3: 把stdout重定向到/dev/null, 丢弃rknn库的调试printf
    //     JSON协议走stderr, 与库输出彻底隔离
    int nullfd = open("/dev/null", O_WRONLY);
    if (nullfd >= 0) {
        dup2(nullfd, STDOUT_FILENO);
        if (nullfd != STDOUT_FILENO) close(nullfd);
    }

    // 加载模型（仅一次）
    double t0 = now_ms();
    rknn_app_context_t app_ctx;
    memset(&app_ctx, 0, sizeof(app_ctx));
    init_post_process();

    int ret = init_yolov8_model(model_path, &app_ctx);
    if (ret != 0) {
        fprintf(stderr, "FATAL: init model fail! ret=%d\n", ret);
        return 1;
    }
    fprintf(stderr, "[yolo_daemon] model loaded in %.0fms, ready\n", now_ms() - t0);

    // 主循环
    char *line = NULL;
    size_t cap = 0;
    ssize_t len;
    int fid = 0;

    while ((len = getline(&line, &cap, stdin)) > 0) {
        if (len > 0 && line[len-1] == '\n') line[--len] = '\0';
        if (len > 0 && line[len-1] == '\r') line[--len] = '\0';
        if (len == 0) continue;

        image_buffer_t src;
        memset(&src, 0, sizeof(src));

        double t1 = now_ms();
        ret = read_image(line, &src);
        double t_read = now_ms() - t1;
        if (ret != 0) {
            fprintf(stderr, "{\"error\":\"read_fail\"}\n");
            fflush(stderr);
            continue;
        }

        object_detect_result_list od;
        double t2 = now_ms();
        ret = inference_yolov8_model(&app_ctx, &src, &od);
        double t_infer = now_ms() - t2;

        // 画框
        char txt[256];
        for (int i = 0; i < od.count; i++) {
            object_detect_result *d = &(od.results[i]);
            int x1 = d->box.left, y1 = d->box.top;
            int x2 = d->box.right, y2 = d->box.bottom;
            sprintf(txt, "%s %.1f%%", coco_cls_to_name(d->cls_id), d->prop * 100);
            draw_rectangle(&src, x1, y1, x2-x1, y2-y1, 0xFF0000FF, 2);
            draw_text(&src, txt, x1, y1-4, 0xFF0000FF, 1.0);
        }

        // v4: JPG 写 tmp 文件 + rename 原子替换
        // rename 保证 consumer 读到的是完整帧 (旧inode或新inode, 绝不会半帧)
        double t3 = now_ms();
        ret = write_image("/dev/shm/yolo_out.tmp.jpg", &src);
        double t_write = now_ms() - t3;
        if (ret == 0) {
            rename("/dev/shm/yolo_out.tmp.jpg", "/dev/shm/yolo_out.jpg");
        } else {
            fprintf(stderr, "[yolo_daemon] write_image fail\n");
        }

        fprintf(stderr, "[stage] read=%.1fms infer=%.1fms write=%.1fms\n",
                t_read, t_infer, t_write);

        // JSON输出到 stderr (隔离的协议通道)
        fprintf(stderr, "{\"frame\":%d,\"ms\":%.1f,\"count\":%d,\"det\":[", fid++, t_infer, od.count);
        for (int i = 0; i < od.count; i++) {
            object_detect_result *d = &(od.results[i]);
            if (i > 0) fprintf(stderr, ",");
            fprintf(stderr, "{\"c\":\"%s\",\"x1\":%d,\"y1\":%d,\"x2\":%d,\"y2\":%d,\"p\":%.2f}",
                    coco_cls_to_name(d->cls_id), d->box.left, d->box.top,
                    d->box.right, d->box.bottom, d->prop);
        }
        fprintf(stderr, "]}\n");
        fflush(stderr);
        if (src.virt_addr) free(src.virt_addr);
    }

    free(line);
    release_yolov8_model(&app_ctx);
    deinit_post_process();
    fprintf(stderr, "[yolo_daemon] shutdown\n");
    return 0;
}
