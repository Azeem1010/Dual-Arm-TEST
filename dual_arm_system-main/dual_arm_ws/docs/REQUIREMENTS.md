# ROS 2 雙機械手臂協作系統 — 完整開發 Master Prompt（含 OpenCV 校正）

## 一、開發目標

你是一位具備 ROS 2、MoveIt 2、Gazebo Harmonic、SolidWorks、OpenCV、TF2、ros2_control、CM-530、DYNAMIXEL 與 ESP32 micro-ROS 專業能力的機器人系統工程師。

請直接開發完整的雙臂視覺辨識、自動抓取、交接與分類系統。

系統包含：

- PhantomX Pincher 四自由度機械手臂 × 2
- AX-12A 馬達 × 8
- 共用 CM-530 控制器 × 1
- MoveIt 2 雙臂運動規劃
- Gazebo Harmonic 高擬真 CAD 工作站
- OpenCV 三相機視覺辨識與校正
- TF2 座標轉換
- ESP32 micro-ROS 雙吸盤控制
- A→B→C 雙臂協作
- C 區八格依序放置

開發環境：

**第一階段：** Windows 11、WSL2 Ubuntu 24.04、ROS 2 Jazzy、Gazebo Harmonic、MoveIt 2。

**第二階段：** NVIDIA Jetson AGX Orin、實體 CM-530、AX-12A、ESP32 與三顆 USB 相機。

先完成 Windows 模擬，再處理實體部署。

## 二、必須參考的四個 GitHub 專案

### 1. PhantomX Pincher ROS 2

https://github.com/snt-spacer/phantomx_pincher

使用單臂 URDF、Xacro、Mesh、運動學及 MoveIt 範例，改造成 Jazzy 相容的八軸雙臂模型。

### 2. 雙臂 SolidWorks CAD

https://github.com/xian1022/dual_arm_3d

使用實際 SolidWorks／STEP 資料，建立高度還原 CAD 的雙臂 URDF 與 Gazebo 工作站。

保留 AX-12A、支架、底座、連桿、吸盤及重要零件的外觀和實際比例。

### 3. CM-530 ROS Bridge

https://github.com/xian1022/CM530_ROS_BRIDGE

流程圖：

https://github.com/xian1022/CM530_ROS_BRIDGE/blob/main/docs/images/dual-arm-flowchart.png

兩支手臂共用一片 CM-530，馬達 ID 預期為：

- Arm 1：17、3、2、15
- Arm 2：12、1、8、16

Serial：57600 baud、8N1。

必須依實際第 17 版韌體確認指令、ID 與回傳格式，不能虛構控制協定。

### 4. OpenCV 三相機 ROS 2

https://github.com/Azeem1010/three-camera-vision-ros2.git

**重要：這個 OpenCV 專案目前尚未完成相機校正。**

請直接整合原有 `vision_pkg`、`vision_interfaces`、`env_vision_node`、`arm1_vision_node`、`arm2_vision_node`、`vision_core` 等實際存在的元件。

不得假設原專案已有正確的 Camera Matrix、Homography、手眼校正或實體座標轉換參數。

先核對實際原始碼與介面，保留可用功能，再補齊校正與驗證流程。

## 三、完整系統架構

```text
                    ROS 2 Jazzy
                         |
              Three Camera System
                         |
            OpenCV / Vision Interfaces
                         |
               Camera Calibration
                         |
                  TF2 Transform
                         |
                Task Coordinator
                         |
                  Zone Manager
                         |
                     MoveIt 2
                         |
            Planning Scene / OMPL
                         |
             +-----------+-----------+
             |                       |
      arm1_controller         arm2_controller
             |                       |
             +-----------+-----------+
                         |
                    ros2_control
                         |
             +-----------+-----------+
             |                       |
        Simulation                Hardware
             |                       |
      gz_ros2_control       CM530SystemInterface
             |                       |
       Gazebo Joints         Single Serial Manager
                                     |
                                   CM-530
                                     |
                                  8 AX-12A

                Gripper Manager
                         |
              +----------+----------+
              |                     |
         Gazebo Gripper       micro-ROS ESP32
```

視覺、任務決策、MoveIt、軌跡控制、硬體及末端工具須分成獨立模組。

## 四、雙臂 URDF 與 CAD 建模

建立 `dual_arm_description`，包含：

- 唯一 `world` 根節點
- `arm1`、`arm2` 兩條四軸運動鏈
- 八個唯一命名的 Revolute Joint
- 兩組吸盤 TCP
- CAD 對應的 Link、Joint Origin 與 Axis
- Visual Mesh
- 簡化 Collision Mesh
- Inertial
- Joint Limits
- TF Frame

從 PhantomX 原始模型產生可重複使用的 Xacro Macro，再依 `dual_arm_3d` 校正尺寸及裝配姿態。

不可把整個靜態 CAD 當成單一不可動模型。

## 五、Gazebo 高擬真工作站

建立：

`dual_arm_gazebo/worlds/dual_arm_workspace.sdf`

必須顯示：

- 高度還原 CAD 的兩支機械手臂
- 完整底座、支架、馬達及吸盤
- 雙臂工作平台
- A、B、C 工作區
- C 區八個分類孔位
- 藍色圓形瓶蓋
- 三台模擬相機
- 重力、碰撞與物理接觸

工作區以 OpenCV 專案規格為依據：

- 粉紅泡棉板 600 × 300 mm
- C/B/A 三區各 200 × 300 mm
- C 區四排、每排兩孔
- 孔位由使用者側往遠處編號 1～8
- 藍色瓶蓋直徑 20 mm

Gazebo 必須提供完整工作站的等角、頂視、正視及側視畫面，與 SolidWorks 原始外觀比較。

## 六、OpenCV 未校正狀態與安全限制

**目前三顆相機均視為尚未完成實體校正，除非實際檢查到有效且已驗證的校正檔。**

在完成校正之前：

1. 可以測試 Camera Image Topic。
2. 可以測試 HSV 物件辨識。
3. 可以測試粉紅區域與藍色瓶蓋偵測。
4. 可以進行模擬環境的校正與座標驗證。
5. 不可將尚未校正的像素座標直接送給實體 MoveIt 抓取流程。
6. 不可使用預設 Homography 假裝實機已校正。
7. 不可將合成影像的成功結果標記為實體相機校正成功。
8. 校正失敗、校正檔過期或相機安裝姿態變動時，禁止自動抓取。

建立 Calibration Status / Validity 機制。

任務執行前必須確認相關相機校正有效，且目標資料沒有過期。

模擬相機與實體相機分別保存校正資料，不能直接共用未經驗證的校正矩陣。

## 七、OpenCV 三相機完整校正流程

### 7.1 Environment Camera

對應 `env_vision_node`。

功能：

- 辨識粉紅色工作區
- 確認 C/B/A 三區
- 偵測藍色瓶蓋
- 判斷 A、B 區物件狀態
- 監控 B 區交接情況

必須完成：

1. Camera Intrinsic Calibration。
2. Lens Distortion Calibration。
3. 工作平面與相機相對姿態校正。
4. 工作區角點或標記點定位。
5. 平面 Homography 或適用的座標轉換。
6. A/B/C 區域邊界驗證。
7. 重投影與實際定位誤差測試。

輸出可重複載入的相機校正 YAML。

### 7.2 Arm 1 Camera

對應 `arm1_vision_node`。

功能：在 Home_A 姿態取得 A 區瓶蓋位置，供 Arm 1 抓取。

必須完成：

1. 相機內參與畸變校正。
2. Home_A 拍攝姿態定義。
3. 相機相對於 Arm 1 基座／末端的外參或手眼關係校正。
4. A 區工作平面 Homography。
5. Pixel → Plane → Arm 1 Base Frame 轉換。
6. 抓取位置誤差驗證。
7. 相機或手臂姿態改變時的校正有效性判斷。

如果是固定於手臂的相機，需要使用正確的手眼座標關係與 TF2；不可把只在 Home_A 有效的平面映射任意套用到其他姿態。

### 7.3 Arm 2 Camera

對應 `arm2_vision_node`。

功能：在 Home_C 姿態辨識 C 區八個格位。

必須完成：

1. 相機內參與畸變校正。
2. Home_C 拍攝姿態定義。
3. 相機與 Arm 2 座標關係校正。
4. C 區平面 Homography。
5. 八個格位中心座標校正。
6. 孔位編號方向確認。
7. 格位占用與放置結果驗證。
8. Pixel → Arm 2 Base Frame 轉換誤差測試。

必須確認實際編號 1～8 與影像辨識輸出一致，避免左右反轉、上下顛倒或視角造成位置錯配。

## 八、校正工具與檔案

優先沿用 OpenCV 原專案已有的校正工具。

缺少功能時再補充：

- ChArUco / Chessboard Calibration
- Camera Intrinsic Calibration
- Distortion Correction
- Homography Calibration
- Hand-Eye / Extrinsic Calibration（依實際相機安裝方式）
- TF2 Validation
- Pixel-to-World Validation
- Calibration Report Generator

建立或整理：

```text
dual_arm_vision_adapter/
├── config/
│   ├── sim/
│   │   ├── env_camera.yaml
│   │   ├── arm1_camera.yaml
│   │   └── arm2_camera.yaml
│   └── hardware/
│       ├── env_camera.yaml
│       ├── arm1_camera.yaml
│       └── arm2_camera.yaml
├── calibration/
├── launch/
└── test/
```

每份有效校正檔應保存：

- Camera ID / Frame ID
- Image Width / Height
- Camera Matrix
- Distortion Coefficients
- Extrinsic / Homography
- Reference Frame
- Calibration Timestamp
- Calibration Image Evidence
- Reprojection Error
- Workspace Position Error
- Validation Result

以上欄位以符合相機模型和實際校正方法為準，不適用的欄位需明確標示。

校正誤差的允收門檻必須依瓶蓋尺寸、孔位尺寸、吸盤大小與機械手臂精度建立，不可任意選定未驗證的毫米數字。

校正無效或超出容許誤差，必須回報並阻止相關自動運動。

## 九、Gazebo 模擬相機校正

模擬模式也必須測試整套 Vision Calibration Pipeline。

使用 Gazebo Camera Sensor 與 `ros_gz_bridge` 發布影像。

驗證：

1. 相機影像與 CameraInfo 正確。
2. Frame ID 與 TF2 正確。
3. 影像解析度與模型一致。
4. OpenCV 可辨識模擬瓶蓋及孔位。
5. 物件座標能轉換到正確 Base Frame。
6. 與 Gazebo Ground Truth 比較位置誤差。
7. 將模擬校正狀態標記為有效或失敗。

Gazebo 內已知的相機參數可以作為模擬基準，但仍須通過座標和辨識驗證。

**模擬校正成功不代表 Jetson 實體相機校正成功。**

## 十、MoveIt 2 與雙臂控制

建立：

- arm1 Planning Group
- arm2 Planning Group
- both_arms Planning Group
- arm1_controller
- arm2_controller
- Planning Scene
- OMPL
- Joint State Feedback
- Collision Objects

使用 `gz_ros2_control` 完成 Gazebo 八軸控制。

兩支手臂可以獨立 Plan & Execute。

兩個控制器不得重複控制同一個關節。

四自由度機械手臂的 IK 必須符合實際機構自由度與末端姿態限制。

## 十一、A→B→C 協作流程

建立 Task Coordinator、Zone Manager 與 Slot Manager。

Arm 1：

1. 確認 Arm 1 Camera 已校正。
2. 移至 Home_A。
3. 呼叫原有 `get_pick_target`。
4. 驗證回傳座標與時間戳。
5. 轉換至 MoveIt 使用的 Frame。
6. 規劃並抓取瓶蓋。
7. 進入 B 區。
8. 放置、確認釋放、退出 B 區。

Arm 2：

1. 確認 B 區物件存在。
2. 取得 B 區進入權。
3. 抓取瓶蓋。
4. 移至 Home_C。
5. 確認 Arm 2 Camera 校正有效。
6. 呼叫原有 `get_grid_state`。
7. 取得下一個合法孔位座標。
8. 轉換至 MoveIt 使用的 Frame。
9. 規劃並放置瓶蓋。
10. 呼叫 `verify_place`。
11. 更新格位狀態。

需要保留原始 `vision_interfaces` 中實際存在的 Service / Topic 名稱與訊息型別，不得猜測欄位。

未完成校正的實體模式禁止進入自動抓取狀態。

## 十二、模擬吸盤與 ESP32 micro-ROS

模擬模式使用 Gazebo Gripper Backend。

實體模式使用 ESP32 micro-ROS Backend。

兩者需支援：

- 吸盤啟動
- 吸盤關閉
- 吸附確認
- 物件釋放
- 故障回報

Gazebo 必須具有物理附著及釋放機制，並同步更新 MoveIt Planning Scene。

ESP32 和 CM-530 使用獨立通訊通道。

## 十三、CM-530 硬體控制

建立單一 `CM530SystemInterface`，管理八軸命令與回授。

不得讓兩個 Controller 同時開啟同一序列埠。

必須完成：

- Joint Name → Motor ID
- rad → AX-12A Position
- Zero Offset
- Direction Sign
- Joint Limits
- Serial Scheduler
- ACK / READ
- JointState Feedback
- Timeout / Fault Handling

Serial ACK 不代表馬達到位。

必須以實際位置回授確認執行結果。

實體控制預設禁止啟用；硬體急停必須與軟體控制分離。

## 十四、必須建立的 ROS 2 Packages

```text
dual_arm_ws/
├── src/
│   ├── dual_arm_description/
│   ├── dual_arm_gazebo/
│   ├── dual_arm_moveit_config/
│   ├── dual_arm_controllers/
│   ├── vision_pkg/
│   ├── vision_interfaces/
│   ├── dual_arm_vision_adapter/
│   ├── dual_arm_coordinator/
│   ├── dual_arm_gripper/
│   ├── dual_arm_interfaces/
│   ├── cm530_hardware/
│   ├── esp32_micro_ros/
│   └── dual_arm_bringup/
├── third_party/
├── docs/
│   ├── ARCHITECTURE.md
│   ├── SOURCE_AUDIT.md
│   ├── CAD_VALIDATION.md
│   ├── VISION_CALIBRATION.md
│   ├── VISION_INTEGRATION.md
│   ├── TEST_PLAN.md
│   ├── TEST_REPORT.md
│   ├── PROGRESS.md
│   └── ORIN_DEPLOYMENT.md
├── AGENTS.md
├── CLAUDE.md
└── README.md
```

保留第三方原始碼版本和來源，避免直接破壞現有四個 GitHub 專案。

## 十五、啟動模式

實際建立並測試：

1. `mock.launch.py` — RViz 虛擬雙臂
2. `sim.launch.py` — Gazebo CAD 完整工作站
3. `vision_sim.launch.py` — Gazebo 三相機 + OpenCV
4. `calibration.launch.py` — 視覺校正工具
5. `demo_pick_place.launch.py` — A→B→C 模擬搬運
6. `hardware.launch.py` — Orin 實體系統

所有 Launch 必須有正確的 ROS 2 Package 與 Launch 設定，不能只宣告檔名而不實作。

## 十六、開發順序

依以下順序直接執行：

**Phase 1 — 原始碼與環境查核**

Clone 四個 GitHub 專案，確認版本、分支、依賴與實際功能。

**Phase 2 — 雙臂 URDF**

完成八軸 Xacro、TF、Joint、Mesh 與 CAD 幾何整合。

**Phase 3 — Gazebo CAD 工作站**

完成雙臂、平台、粉紅工作區、藍色瓶蓋、八格孔位與三相機。

**Phase 4 — MoveIt 2**

完成八軸 Gazebo 控制、兩個 Controller、單臂與雙臂規劃。

**Phase 5 — OpenCV 原始專案整合**

編譯既有 `vision_pkg` 與 `vision_interfaces`，接收 Gazebo 模擬相機影像。

**Phase 6 — OpenCV 校正**

完成三相機內參、畸變、外參／手眼關係、Homography、TF2 與定位誤差驗證。模擬與實體校正獨立處理；實體尚未接入時標示為 NOT_RUN 或 BLOCKED。

**Phase 7 — 視覺與運動整合**

通過有效校正狀態檢查後，使用原始 Vision Services 取得抓取及孔位目標，執行 MoveIt 模擬運動。

**Phase 8 — 完整搬運**

完成模擬吸盤、A→B→C 協作、B 區互鎖與 C 區八格驗證。

**Phase 9 — CM-530 與 ESP32**

完成 Hardware Interface、Mock Serial、micro-ROS 與安全控制介面。

**Phase 10 — Orin 部署**

建立部署文件、相機實體校正流程、關節校正流程和實機驗收程序。

## 十七、驗收與工程規則

每個階段必須記錄：

- 實際修改的檔案
- 執行的編譯與測試命令
- 終端機輸出
- Gazebo 或 RViz 截圖
- 測試結果
- 尚未完成的功能

狀態只使用 PASS、FAIL、BLOCKED、NOT_RUN。

尤其不得把以下情況視為通過：

- OpenCV 能偵測瓶蓋 ≠ 相機已完成校正。
- Homography 程式存在 ≠ Homography 已完成實際校正。
- Gazebo 座標正確 ≠ 真實相機座標已正確。
- 相機內參已校正 ≠ 相機與機械手臂的外參已校正。
- MoveIt 規劃成功 ≠ 實體機械手臂可以安全執行。
- CM-530 ACK 成功 ≠ AX-12A 已到達目標位置。

請維護 `AGENTS.md`、`CLAUDE.md`、`docs/PROGRESS.md` 以支援 Codex 與 Claude Code 交接。

## 十八、立即開始執行

請先查核四個 GitHub 專案，建立 ROS 2 Jazzy 工作空間，再完成 CAD 雙臂 URDF、Gazebo 高擬真場景、MoveIt 2 控制以及三相機 OpenCV 整合。

**特別要求：OpenCV 原始專案目前還未完成校正，必須把校正視為正式開發任務，不可忽略，也不可自行填入假校正參數。**

優先完成 Windows WSL2 的三相機模擬辨識、座標校正、MoveIt 2 和完整 A→B→C 搬運。

之後再於 Jetson AGX Orin 完成實體相機重新校正、CM-530 與 ESP32 整合。

直接開始實作、編譯、測試與修正，不要只停留在架構規劃。

最終交付必須是一套可運行的雙臂 ROS 2 系統，具有高度還原 CAD 的 Gazebo 工作站、可校正的三相機 OpenCV、MoveIt 2 雙臂規劃、A→B→C 自動搬運，以及 Orin 實體部署介面。