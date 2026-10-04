// pose_server.cc v2 —— YOLOv8-Pose 旁路服务（干净帧源 + 可标定诊断特征）
// 用法: pose_server <pose.rknn> [帧源] [out_json]
//   帧源: 单文件, 或 "auto"(默认) = 监听 /dev/shm/yolo_frame_a.jpg 与 _b.jpg 取最新mtime
//         ⚠️ 必须喂干净原帧; 喂画框的 yolo_out.jpg 会污染关键点(判据恒真, HRI 10-04 实测)
//   输出: {ts,t_mono,infer_ms,persons:[{bbox,conf,facing,offering,arms_raised, arms:[{side,elbow_deg,ext_ratio,wrist_up,n}]}]}
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

#define KP_NOSE 0
#define KP_LSHO 5
#define KP_RSHO 6
#define KP_LELB 7
#define KP_RELB 8
#define KP_LWRI 9
#define KP_RWRI 10
#define KP_LHIP 11
#define KP_RHIP 12
#define CONF_MIN 0.30f

static const char* FRAME_A = "/dev/shm/yolo_frame_a.jpg";
static const char* FRAME_B = "/dev/shm/yolo_frame_b.jpg";

static double now_ms(void) {
    struct timespec ts; clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1000.0 + ts.tv_nsec / 1e6;
}

// 肘角(度): 180=完全伸直
static float elbow_angle(float* sho, float* elb, float* wri) {
    float v1x = sho[0]-elb[0], v1y = sho[1]-elb[1];
    float v2x = wri[0]-elb[0], v2y = wri[1]-elb[1];
    float n1 = sqrtf(v1x*v1x+v1y*v1y), n2 = sqrtf(v2x*v2x+v2y*v2y);
    if (n1 < 1e-3f || n2 < 1e-3f) return -1.f;
    float c = (v1x*v2x+v1y*v2y)/(n1*n2);
    if (c > 1.f) c = 1.f; if (c < -1.f) c = -1.f;
    return acosf(c) * 57.29578f;
}

static const char* facing_of(object_detect_result* r) {
    float face = (r->keypoints[0][2]+r->keypoints[1][2]+r->keypoints[2][2]
                + r->keypoints[3][2]+r->keypoints[4][2])/5.0f;
    float bw = r->box.right - r->box.left;
    float sh_dx = fabsf(r->keypoints[KP_LSHO][0]-r->keypoints[KP_RSHO][0]);
    if (face > 0.40f && (bw <= 1 || sh_dx > 0.30f*bw)) return "front";
    if (face < 0.15f) return "back";
    float mid = (r->keypoints[KP_LSHO][0]+r->keypoints[KP_RSHO][0])/2.0f;
    return (r->keypoints[KP_NOSE][0] < mid) ? "left" : "right";
}

int main(int argc, char** argv) {
    if (argc < 2) { printf("%s <pose.rknn> [frame|auto] [out_json]\n", argv[0]); return -1; }
    const char* model = argv[1];
    const char* framespec = argc > 2 ? argv[2] : "auto";
    const char* out_json = argc > 3 ? argv[3] : "/dev/shm/pose_out.json";

    rknn_app_context_t ctx; memset(&ctx, 0, sizeof(ctx));
    init_post_process();
    if (init_yolov8_pose_model(model, &ctx) != 0) { printf("init pose fail\n"); return -1; }
    printf("pose_server v2 ready | frames=%s -> %s\n", framespec, out_json);
    fflush(stdout);

    time_t last_s = 0; long last_ns = 0;
    while (1) {
        // 选源: auto 取 a/b 中 mtime 最新者
        const char* src = framespec;
        struct stat sa, sb;
        int ha = (stat(FRAME_A,&sa)==0), hb = (stat(FRAME_B,&sb)==0);
        if (strcmp(framespec,"auto")==0) {
            if (ha && hb) src = (sa.st_mtim.tv_sec > sb.st_mtim.tv_sec ||
                (sa.st_mtim.tv_sec==sb.st_mtim.tv_sec && sa.st_mtim.tv_nsec >= sb.st_mtim.tv_nsec)) ? FRAME_A : FRAME_B;
            else if (ha) src = FRAME_A; else if (hb) src = FRAME_B; else { usleep(100000); continue; }
        }
        struct stat st;
        if (stat(src,&st)!=0) { usleep(100000); continue; }
        if (st.st_mtim.tv_sec==last_s && st.st_mtim.tv_nsec==last_ns) { usleep(50000); continue; }
        last_s = st.st_mtim.tv_sec; last_ns = st.st_mtim.tv_nsec;

        image_buffer_t img; memset(&img,0,sizeof(img));
        if (read_image(src,&img)!=0) { if(img.virt_addr) free(img.virt_addr); usleep(100000); continue; }
        object_detect_result_list od; memset(&od,0,sizeof(od));
        double t0 = now_ms();
        int ret = inference_yolov8_pose_model(&ctx,&img,&od);
        double dt = now_ms()-t0;
        if (ret==0) {
            FILE* fo=fopen(out_json,"w");
            if (fo) {
                fprintf(fo,"{\"ts\": %.3f, \"t_mono\": %.0f, \"infer_ms\": %.1f, \"src\": \"%s\", \"persons\": [",
                        (double)time(NULL), now_ms(), dt, src);
                for (int i=0;i<od.count;i++) {
                    object_detect_result* r=&od.results[i];
                    float bh = r->box.bottom - r->box.top;
                    if (bh < 1) bh = 1;
                    float sho[2][2] = {{r->keypoints[KP_LSHO][0],r->keypoints[KP_LSHO][1]},
                                       {r->keypoints[KP_RSHO][0],r->keypoints[KP_RSHO][1]}};
                    int elb_id[2]={KP_LELB,KP_RELB}, wri_id[2]={KP_LWRI,KP_RWRI};
                    const char* side[2]={"L","R"};
                    float sho_c[2]={r->keypoints[KP_LSHO][2],r->keypoints[KP_RSHO][2]};
                    float elb_c[2]={r->keypoints[KP_LELB][2],r->keypoints[KP_RELB][2]};
                    float wri_c[2]={r->keypoints[KP_LWRI][2],r->keypoints[KP_RWRI][2]};
                    int offering = 0;
                    fprintf(fo,"%s{\"bbox\": [%d, %d, %d, %d], \"conf\": %.3f, \"facing\": \"%s\", \"arms\": [",
                            i?", ":"", r->box.left,r->box.top,r->box.right,r->box.bottom, r->prop, facing_of(r));
                    for (int a=0;a<2;a++) {
                        float* elb = r->keypoints[elb_id[a]];
                        float* wri = r->keypoints[wri_id[a]];
                        int valid = (sho_c[a]>CONF_MIN && elb_c[a]>CONF_MIN && wri_c[a]>CONF_MIN);
                        float ang=-1, ext=-1, wup=-9;
                        if (valid) {
                            ang = elbow_angle(sho[a], elb, wri);
                            float dx=wri[0]-sho[a][0], dy=wri[1]-sho[a][1];
                            ext = sqrtf(dx*dx+dy*dy)/bh;          // 腕-肩距 / 身高
                            wup = (sho[a][1]-wri[1])/bh;          // >0 = 腕高于肩
                        }
                        fprintf(fo,"%s{\"side\":\"%s\",\"elbow_deg\":%.1f,\"ext_ratio\":%.3f,\"wrist_up\":%.3f,\"valid\":%d}",
                                a?",":"", side[a], ang, ext, wup, valid);
                        // offering 判据 (10-04 P/N 标定: TPR 78% / FPR 0%, n=111 行实测)
                        //   任一有效臂满足 2/3: 肘角<125° | 前伸量<0.21 | 腕高>-0.25 (均按身高归一)
                        if (valid) {
                            int votes = (ang < 125.0f) + (ext < 0.21f) + (wup > -0.25f);
                            if (votes >= 2) offering = 1;
                        }
                    }
                    fprintf(fo,"], \"offering\": %d, \"arms_raised\": %d}",
                            offering, (r->keypoints[KP_LWRI][2]>CONF_MIN && (r->keypoints[KP_LSHO][1]-r->keypoints[KP_LWRI][1])/bh > 0.10f) ? 1 : 0);
                }
                fprintf(fo,"]}\n"); fclose(fo);
            }
            printf("infer %.1f ms, %d persons (%s)\n", dt, od.count, src); fflush(stdout);
        } else printf("infer fail ret=%d\n", ret);
        if (img.virt_addr) free(img.virt_addr);
    }
    return 0;
}
