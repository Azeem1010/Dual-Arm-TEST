# arm1 手動 XY 操作與校正

手動輸入只有 X、Y（公尺，`arm1_base` 座標系）。程式計算
`travel_z = pick_z + approach_height`：`pick_z` 是物品上表面接觸高度，
`approach_height` 是經量測確認的抬升距離。一般 XY 移動全程使用移動高度，吸盤朝下。
此入口不控制吸附，也不執行下降取物；後續取放流程再使用接觸高度。

目前使用者尚未量測實機。軟體與模擬結果不能當成實機校正，硬體模板會拒絕啟動。
`CM530_ROS_BRIDGE-main` 未修改、未燒錄；本次沒有向實機發送運動指令。

## 1. 建置與模擬

```bash
cd /home/orin/雙手臂
bash scripts/build.sh
source /tmp/dual-arm-ros2/install/setup.bash
export ROS_DOMAIN_ID=164 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
ros2 launch dual_arm_control arm1.launch.py mode:=demo control:=manual
```

另一個終端機載入相同環境。此模式只啟動 arm1 bridge、模型、MoveIt、手動服務，
不啟動自動 A→B coordinator 或視覺節點。`mode:=demo` 使用模擬 CM530，不開實體串口。

```bash
source /tmp/dual-arm-ros2/install/setup.bash
export ROS_DOMAIN_ID=164 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
ros2 run dual_arm_control manual_xy --current

# 模擬起點；預設僅規劃，不執行。
ros2 run dual_arm_control manual_xy --x 0.15 --y 0.04 --prepare
# 檢查預覽後，明確執行初始定位。
ros2 run dual_arm_control manual_xy --x 0.15 --y 0.04 --prepare --execute

# 沿 X 增加 5 mm，先預覽再執行。
ros2 run dual_arm_control manual_xy --x 0.155 --y 0.04
ros2 run dual_arm_control manual_xy --x 0.155 --y 0.04 --execute
ros2 run dual_arm_control manual_xy --x 0.15 --y 0.04 --execute
# 沿 Y 增加 5 mm，返回起點。
ros2 run dual_arm_control manual_xy --x 0.15 --y 0.045 --execute
ros2 run dual_arm_control manual_xy --x 0.15 --y 0.04 --execute
```

上述絕對座標僅適用於所附**模擬模型**，不是實機建議座標。
`--prepare` 是建立起點的獨立操作：使用碰撞檢查的關節空間規劃，途中可能改變高度與吸盤方向，
僅終點要求朝下且位於移動高度。一般命令不加此旗標，必須從已到位的姿態開始走 Cartesian 平面路徑。
每次 `--execute` 都從新讀值重新規劃，不會重播先前的預覽；輸出當次目標與軌跡資訊供核對。

```bash
ros2 run dual_arm_control manual_xy --cancel
ros2 run dual_arm_control manual_xy --current --output /tmp/arm1_current.json
```

取消請求會取消活躍 MoveIt action，並直接呼叫 bridge 的停止服務，鎖定後續軌跡、由執行迴圈中止並 HOLD。
執行階段發生例外也會要求 bridge 停止，避免只取消上層 action 而底層繼續移動。
停止服務的回覆可能是「已鎖定，等待執行迴圈 HOLD」，必須待 READ 不再回覆 busy 並核對停止。
逾時或中斷後須讀取狀態確認結果，
不要因沒有收到回覆就認定手臂未動。取消即使在閒置時也會鎖定 bridge；檢查後需重新啟動及明確 enable，沒有自動復歸。

## 2. 實機量測工作表

複製 `src/dual_arm_control/config/manual_xy_hardware.yaml` 為自己的設定。
保持 `calibrated` 與 `hardware_calibrated` 為 false，先填入以下資料：

1. **吸盤**：相對 `arm1_end_effector` 的接觸點 xyz、rpy，以及包覆實際吸盤的碰撞圓柱半徑／長度。
   TCP 的 +Z 朝向物品；圓柱由接觸點沿 -Z 延伸至安裝端。偏心或不規則工具需以足夠大的包覆尺寸建模。
   啟動時在記憶體中更新 TCP 與碰撞形狀，原始 upstream 檔案不改動。
2. **關節**：依 ID **17、3、2、15** 順序，各記錄至少三個已知 URDF 角度與同時取得的 READ。
   角度須不同且跨度至少 0.2 rad。這些是量測姿態，不是要程式下發的角度。
   手動擺位前應支撐手臂並確認適合的斷扭力狀態；不要以推撞機械極限的方式量測。
3. **範圍**：各關節可用的 raw_min/max 與 lower/upper_rad，不能超出模型限制。
   HOME raw 不能直接當作 URDF 零角度，也不能一律使用 512。
4. **平面／場景**：物品上表面高度、抬升距離、XY 可用範圍、桌面中心／尺寸，以及靜止 arm2 的包覆障礙盒。
   若物品厚度不同，不能共用同一個接觸高度。
5. **獨立驗證**：另外三個不同、吸盤朝下的姿態，每個記錄四軸 raw、實測 TCP xyz、實測 TCP +Z 單位方向。
   不能直接把程式 FK 輸出抄回實測欄位。這三個姿態不應使用前述校正參考樣本。

`joint_references` 中每個關節的格式：

```yaml
# 以下為欄位格式，不是可直接使用的校正數據。
# - {radians: 實測弧度, raw: 當下整數讀值}
```

離線工具只讀檔，不開啟串口、不修改設定：

```bash
# 把 /絕對路徑/實測設定.yaml 換成自己完成的工作表。
ros2 run dual_arm_control check_xy_calibration \
  --config /絕對路徑/實測設定.yaml \
  --urdf /home/orin/雙手臂/src/dual_arm_description/urdf/arm1.urdf --fit
```

`--fit` 提供四軸 zero_raw 與 direction 候選值，並要求參考樣本與 AX 300°/1023 比例的誤差不超過 2 ticks。
將結果填回 bridge 欄位，補齊機構限制及獨立驗證資料後，移除 `--fit` 執行完整驗證：
各姿態位置誤差 ≤5 mm、方向模型誤差 ≤3°、實測朝下誤差 ≤3°。
只有完成相應量測後才能將 `tcp_measured`、`plane_measured`、`limits_measured`、`scene_measured` 設為 true。
啟動硬體時也會重新驗證資料，不只檢查旗標。

## 3. 校正通過後的實機交接

此步驟目前**尚未執行**。完成量測、驗證場景後，填入實際串口並設定兩個 calibrated 旗標為 true。
正常結束占用串口的 `cm530_monitor.py`，確認原先的自動流程及其他 bridge 均已停止。
同一串口只由一個 bridge 使用。兩個操作終端機需使用相同 ROS domain。

```bash
source /tmp/dual-arm-ros2/install/setup.bash
export ROS_DOMAIN_ID=0 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
ros2 launch dual_arm_control arm1.launch.py mode:=hardware control:=manual \
  config:=/絕對路徑/實測設定.yaml
```

啟動只建立連線與讀值；馬達仍需明確啟用：

```bash
ros2 run dual_arm_control manual_xy --current
ros2 service call /cm530/arm1/enable std_srvs/srv/Trigger '{}'
```

bridge 執行 HOLD→TORQUE 1，不送 HOME。先以實測、淨空的起點 `(x0,y0)` 預覽 `--prepare`，
檢查起始定位路徑後才明確執行。之後測試 `(x0+0.005,y0)`、回起點、`(x0,y0+0.005)`、回起點。
每步確認軌跡與現場淨空，再單次執行。

使用 `--output` 保存每步的 JSON。內容包括 target_raw、measured_raw、目標 xyz 與 READ 換算的 FK xyz。
**FK 位置是由讀值推算，並非外部量測**；還需記錄尺規／量具觀察到的實際位移。
目前實機座標驗證表保持空白，待完成現場測試後填寫：

| 步驟 | 實機 X | 實機 Y | 設定移動高度 | 實測位移 | 結果 |
|---|---|---|---|---|---|
| P0 | 待量測 | 待量測 | 待量測 | 待量測 | 未執行 |
| X +5 mm | 待量測 | 待量測 | 同上 | 待量測 | 未執行 |
| 返回 P0 | 待量測 | 待量測 | 同上 | 待量測 | 未執行 |
| Y +5 mm | 待量測 | 待量測 | 同上 | 待量測 | 未執行 |
| 返回 P0 | 待量測 | 待量測 | 同上 | 待量測 | 未執行 |

## 4. 介面與測試

- `/arm1/manual_xy`：`dual_arm_interfaces/srv/ManualXY`，輸入 x、y、execute、prepare，回傳 success/message。
- `/arm1/manual_xy/current`、`/arm1/manual_xy/cancel`：`std_srvs/srv/Trigger`。
- `/cm530/arm1/stop`：`std_srvs/srv/Trigger`，停止鎖定及 HOLD；此鎖定作用於共用 bridge，因此手動模式只配置 arm1。
- 平面路徑要求 Cartesian fraction 完整成功；空軌跡、關節／raw 越界、起點變動、姿態或高度不符均拒絕。
- 手動服務序列化請求；預覽會更新碰撞場景，但不發送執行 action 或馬達啟用命令。
- 測試：`PYTHONPATH=src/dual_arm_control python3 -m pytest -q src/dual_arm_control/test`。
- 真 MoveIt＋模擬 CM530 整合：載入 workspace 並設定 domain 164 後執行 `python3 scripts/manual_xy_integration.py`。
