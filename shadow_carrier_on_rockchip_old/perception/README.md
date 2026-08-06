# Perception 模块 — 摄像头+YOLO 实况

## 硬件

| 项目 | 详情 |
|------|------|
| 摄像头 | Microdia USB Camera (0c45:6366)，UVC 驱动 |
| 设备节点 | `/dev/video10`（采集）/ `/dev/video11`（元数据） |
| 支持格式 | MJPG (30fps @ 1920x1080)，YUYV (5fps @ 1920x1080) |
| NPU | Rockchip RK3566，0.8 TOPS @ 780MHz |

## 推流方案对比

| 版本 | 架构 | FPS | 说明 |
|------|------|:---:|------|
| v1 `video_stream.py` | Python subprocess 冷启动 C++ | **0.8** | 模型加载 5.5s/次 |
| v2 `video_stream_fast.py` | daemon 常驻 + 磁盘I/O | **3.7** | 4次磁盘I/O/帧 + PNG编码 |
| v3 `video_stream_v3.py` | /dev/shm 内存盘 + JPG | 14.6(假) | ⚠️ stdout被污染, FPS是假象 |
| v4 `video_stream_v4.py` | JSON走stderr隔离 | 5.1 | 真帧率但串行 |
| v5 `video_stream_v5.py` | 双线程流水线 + 双缓冲 | 11.4 | ⚠️ producer无背压→延迟累积+帧竞争 |
| v6 `video_stream_v6.py` | 背压(inflight) + 原子替换 | 3.8-6.4 | read_fail=0但sleep轮询慢 |
| **v7 `video_stream_v7.py`** | **背压 + 原子替换 + watchdog** | **11.3** ✅ | read_fail=0, 延迟有界, daemon崩溃自动重启 |

> **v7.2 是当前正确版本。** 真实检测帧率 11.3 FPS，`read_fail=0` 零坏帧，信号量背压延迟有界，**独立 watchdog 线程自动恢复 daemon 崩溃**（NPU 偶发崩溃不再冻结管线）。

## 优化成果总结

**从 0.8 FPS → 11.4 FPS，净提升 14 倍**，全程纯软件，无硬件改动。四步杠杆依次叠加：

| 杠杆 | 手法 | FPS 提升 |
|------|------|:--------:|
| ① 消灭模型冷启动 | C++ daemon 常驻，模型只加载一次 | 0.8 → 3.7 |
| ② 消灭磁盘 I/O + PNG | `/dev/shm`(tmpfs) 内存盘 + JPG 替代 PNG | 3.7 → 5.1(真实) |
| ③ 流水线并行 | 生产/消费双线程，daemon 永不空闲 | 5.1 → 11.4 |
| ④ 背压保序 | 信号量限制在飞帧数，消除延迟累积+坏帧 | 稳定在 11.4 |

### 最反直觉的三个发现

1. **FPS 虚高不可信**：v3 曾"14.6 FPS"，实际是 daemon stdout 被 rknn 库 printf 污染，Python 没真等推理完成。**帧率必须验证帧完整性 + 真实耗时**，而非只看循环计数。
2. **`/tmp` 不是内存盘**：KickPi 上 `/tmp` 挂在 eMMC，真磁盘 I/O。`/dev/shm` 才是 tmpfs。嵌入式上"临时文件"也要查挂载点。
3. **流水线必须背压**：生产者快于消费者时，无限积压 → 延迟累积 + 帧竞争坏帧。**任何生产者-消费者架构都要有界队列**，否则必出幺蛾子。

### 延迟的本质

端到端 ~200ms 延迟中：
- NPU 推理 55-60ms（RK3566 硬件极限，不可优化）
- JPEG 编解码 + 传输 ~140ms
- **物理下限 ≈ 200ms**，再优化只能降低分辨率/帧率换延迟，不划算

### 当前 CPU 分布（4 核 RK3566）

| 进程 | CPU | 用途 |
|------|-----|------|
| python v7 | ~90% 单核 | 抓帧编码 + JPEG 解码推流 |
| yolo_daemon | ~57% 单核 | NPU 推理 + 画框编码 |
| 剩余 | ~50% | 其他核空闲，可跑决策/串口/蓝牙 |

> 结论：检测管线已达性价比最优。后续若需更高帧率，走 MJPG 零解码抓帧（v8）或降分辨率，但收益递减。


## 推流服务

```bash
# v7 信号量背压流水线（当前推荐）
cd /home/kickpi/shadow_carrier_on_rockchip/scripts
(cd scripts && nohup python3 -u video_stream_v7.py --port 8080 &)

# Windows 观看
浏览器打开 http://192.168.137.190:8080/viewer
```

| URL | 内容 |
|-----|------|
| `/viewer` | 网页查看器（视频+检测列表+FPS） |
| `/stream` | 纯 MJPEG 视频流 |
| `/api/detections` | JSON 检测结果 |

## yolo_daemon — C++ 常驻推理进程

```
编译: g++ yolo_daemon.cc + postprocess.o + yolov8.o + utils.a -lrknnrt → yolo_daemon
启动: ./yolo_daemon <model.rknn>
协议: stdin 接收图片路径(一行一个) → stderr 返回JSON结果(一行一个)
      stdout 重定向到 /dev/null (丢弃rknn库调试printf)
v3输出: /dev/shm/yolo_out.jpg (内存盘, JPG格式, 不写eMMC)
性能: 模型加载 51-140ms(一次性)
      read=35ms(JPEG解码) infer=77ms(NPU推理) write=70ms(JPG编码) @1920x1080
      1280x720 输入 + 流水线并行 → 端到端 11.4 FPS
阶段计时: 每次推理输出到 stderr: [stage] read=..ms infer=..ms write=..ms
```

## 踩坑记录

### 1. 摄像头被占用 (Device or resource busy)
- 现象: `VIDIOC_REQBUFS returned -1`
- 原因: XFCE桌面 + rkaiq服务 或 残留Python僵尸进程持有fd
- 解法: `systemctl disable slim` 关桌面；`fuser /dev/video10` 查凶手；`pkill -9` 杀僵尸

### 2. USB 重置后设备号漂移
- 现象: `/dev/video10` 消失变 `/dev/video11`
- 原因: USB unbind/rebind 后内核重新分配
- 解法: 重启恢复；或用 `v4l2-ctl --list-devices` 重新扫

### 3. OpenCV OBSensor 后端干扰
- 现象: `obsensor_uvc_stream_channel` 错误，高编号video设备打不开
- 原因: OpenCV 的 Intel RealSense 后端抢先探测所有UVC设备，失败不释放
- 解法: 杀掉残留进程释放设备锁

### 4. NPU 驱动掉线
- 现象: `failed to open rknpu module`
- 原因: USB 重置导致连续内存碎片化
- 解法: **必须重启**

### 5. Python subprocess 每帧冷启动
- 现象: FPS 0.8，每帧5-6秒
- 原因: 每次 `subprocess.run()` 重新加载模型到NPU
- 解法: `yolo_daemon.cc` 模型常驻，stdin/stdout协议通信

### 6. 磁盘 I/O + PNG 编码双重瓶颈
- 现象: daemon 推理只需 ~75ms，但端到端 FPS 仅 3.7
- 原因 A: `/tmp` 挂在 eMMC (`/dev/root`)，**不是内存盘**
- 原因 B: daemon 写 `out.png`，**PNG 编码 ~80ms**（JPG 仅 ~15-20ms）
- 解法: 帧文件移入 `/dev/shm`(tmpfs内存盘) + daemon 输出改 JPG

### 7. 后台进程被 SSH 会话清理
- 现象: `nohup python3 ... &` 启动后找不到进程
- 原因: SSH MCP 每次 exec 独立会话，`&` 后台进程被清理；daemon 子进程持有 fd 致 exec 挂起
- 解法: `(cd <dir> && nohup python3 -u script.py > /tmp/log 2>&1 &)` 子shell + nohup

### 8. pkill -f 自杀陷阱
- 现象: 组合命令 `pkill -f X; nohup ... &` 后半段不执行
- 原因: `pkill -f 'time.sleep'` 匹配到执行命令的父 shell 自身，把自己 SIGTERM 了
- 解法: 清理用 `pkill -x <精确进程名>`

### 9. daemon stdout 被 rknn 库 printf 污染（v3 花屏/卡顿/死锁根因）⭐
- 现象: 网页画面一半灰、卡顿、跑久了冻结；FPS 虚高 (14.6) 但画面不更新
- 原因: rknn_model_zoo 库的调试 printf 全写到 stdout，与 JSON 协议混在一起
  - Python readline 读到 printf 行 → json解析失败 → `infer()=0ms`（没等推理完成）
  - Python 立刻读 out.jpg → **daemon 正在写 → 读到半帧 → 一半灰**
  - latest_jpeg 更新频率远低于显示 FPS → 推流卡顿
  - 管道积压数百帧灌满 64KB → daemon 卡 `pipe_write` → 全冻结
- 诊断: `ps` 看 daemon 卡在 `pipe_write`，Python 卡在 `do_sys_poll`；抓流发现 5 秒只发 1 帧
- 解法: daemon `dup2(/dev/null, STDOUT)` 丢弃库输出，JSON 改走 **stderr** 隔离
  - Python: `stdout=DEVNULL, stderr=PIPE`，过滤 `{` 开头的行
- 验证: `echo capture.jpg | yolo_daemon model.rknn 2>&1` 输出应只剩 JSON

### 10. 单线程串行 = 低帧率 (v4)
- 现象: 修复污染后 FPS 从"14.6"跌到真实 5.1
- 原因: Python 等 daemon 推理时没有并行准备下一帧 → 抓帧+推理串行
- 解法 (v5): 双线程流水线
  - 生产者: 抓帧→编码→双缓冲→喂daemon（不等结果）
  - 消费者: 收JSON→读out.jpg→推流
  - FPS = max(生产者速率, daemon速率) ≈ 11.4

### 11. producer 无背压 → 延迟累积 + 帧竞争花屏 (v5→v7)
- 现象: 网页延迟越来越大 + 偶发灰屏(下半灰从下褪掉) + 灰屏时YOLO无结果
- 原因: producer抓帧速率(75ms) > daemon处理速率(~130ms), 路径无限积压
  - 积压导致延迟累积 (画面越看越旧)
  - 积压时帧时序错乱, daemon读到producer正在写的frame → 坏帧
  - 坏帧 → read_fail(无检测结果) + 坏out.jpg(花屏)
- 解法:
  - 信号量背压 (MAX_INFLIGHT=2): producer在槽满时阻塞等consumer → 延迟有界(~200ms)
  - daemon out.jpg 改 tmp+rename 原子替换 → consumer永读完整帧
- 验证: API stats.read_fail 应保持 0

### 12. consumer 用 select 轮询 stderr → 拖慢管线 + 心跳误判 (v7.1→v7.2)
- 现象: 加 watchdog 时用 select 非阻塞读 stderr, fps 从 11.4 掉到 5, 且 daemon 反复重启(restarts涨)
- 原因 A (慢): select 每秒轮询 + text-mode 管道交互, consumer 吞吐下降
- 原因 B (误判): select 时序问题导致 last_json_t 没及时更新 → 心跳超时 → kill 正常 daemon → 恶性循环
- 解法 (v7.2): 
  - consumer 恢复阻塞 readline (高效, daemon死时EOF返回)
  - watchdog 独立线程, 每1s检查 poll(退出) + last_json_t(心跳超时5s)
  - 心跳变量跨线程共享, watchdog 只读、consumer 只写
- 验证: pkill yolo_daemon → 日志"[watchdog] daemon退出,重启" → restarts+1 → frames恢复增长

## 架构演进

```
v1   [拍帧]--文件-->[subprocess冷启C++]--文件-->[Python]    0.8fps
v2   [Python]--文件-->[daemon常驻]--文件-->[Python]         3.7fps (磁盘IO+PNG)
v3   [Python]--内存盘-->[daemon]--内存盘-->[Python]         14.6fps(假, stdout污染)
v4   [Python]--内存盘-->[daemon(stderr JSON)]--内存盘-->[Python]  5.1fps (串行)
v5   [生产者]--双缓冲-->[daemon]--内存盘-->[消费者]    11.4fps(延迟累积)
v6   [生产者+背压轮询]-->[daemon原子替换]-->[消费者]   3.8fps(轮询慢)
v7.2 [生产者+背压]-->[daemon原子替换]-->[消费者] 11.3fps ✅ + watchdog独立线程
```

## 后续优化方向（v6 备选）

- **MJPG 零解码抓帧**: V4L2 直接读摄像头 MJPG 原始字节，跳过 OpenCV 解码+编码，生产者 CPU 下降
- **降 JPEG 质量**: 75→60，编码/解码更快
- **降分辨率**: 960x540，编码/推理更快，检测距离变短
- **全 C++ 管线**: 填 camera.cpp + yolov8.cpp TODO，彻底消除 Python
