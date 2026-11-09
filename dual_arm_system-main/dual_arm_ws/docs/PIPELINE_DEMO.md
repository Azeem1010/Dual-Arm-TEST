# Gazebo A → B → C 接續展示

執行程式：`scripts/demo_transfer_pipeline.py`。使用專用 ROS domain 43 的 Gazebo 場景。新版鏡頭支架已轉向前方，C 區為齊平白格；關節限制保持不變。此為模擬真值導引展示，尚不是完成校正的視覺取放或完整工作站碰撞驗收。

本次順序為 `cap_5`、`cap_6`、`cap_4`，分別送到原 C 區第 3、4、5 孔。選擇這三個 A 物體，是因為目前模型對其他部分 A 位置或 C 孔位仍有可達性／碰撞限制，不能宣稱八物體流程完成。

流程：

1. Arm 1 將第一個物體由 A 放到 B，退出共享區。
2. Arm 1 回 A 接近下一個物體，同時 Arm 2 接近 B；使用單一 `both_arms` MoveIt 目標，避免兩個 `/move_action` 目標互相取消。
3. 分別確認吸附回覆，再同步抬升；以 Gazebo PosePublisher 的物體位置確認兩物體實際跟隨吸盤。
4. Arm 2 搬至 C；只有物體實際離開 B 區後，才清除 B 占用狀態，允許 Arm 1 把下一個物體送往 B。
5. Arm 2 在白格上方釋放物體，退開後檢查物體是否停在白格表面。Arm 1 完成下一個 B 放置，再重複步驟 2。

搬運速度縮放本版為 0.50；接觸段與加速度縮放皆為 0.25，集中於 `config/sim/runtime.json`。這是規劃速度縮放，實際畫面速度也受 Gazebo 的即時倍率影響。開始及退出 B 區使用經使用者同意的模擬 `home_A`／`home_C` 換算，詳見 HOME_POSES.md；不再使用 CAD 零位作預備姿勢。

歷史單物體測試在 C 孔底姿態因 TCP 誤差 >1 mm 停止；後續舊版三物體凹洞展示已 PASS。本版白格頂面 z=14 mm，TCP 在 z=25.3 mm 釋放後，驗證物體中心 z≈19 mm，不能沿用舊版孔底的 6 mm 驗證值。完整碰撞模型仍需另行驗證。

每次實際放置的驗證容差：XY 距離 ≤2 mm、Z 誤差 ≤1.5 mm；吸附插件保留既有 1 mm 接觸範圍。收到吸附 ACK 本身不算取物成功，還必須通過抬升跟隨檢查。任一錯誤停止序列，不自動釋放仍被持有的物體，也不移動／重設物體位置來製造成功。

最新結構化證據在 `docs/evidence/transfer_pipeline.json`，本次文字紀錄為 `revised_pipeline.log`；只有最終 `pipeline_completed` 且 `status=PASS` 才代表本次指定物體數展示完成。`--count 1|2|3` 可選數量，預設三個。執行中的 FAIL 預設狀態不是完成結果。歷史三物體 PASS 在 before_revision_20261109，不能替代新版驗證。

重跑需要新的模擬場景（不是把執行中的物體瞬移回 A）：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=43
export GZ_PARTITION=dual_arm_pipeline_demo
ros2 launch dual_arm_bringup sim.launch.py gz_args:=-r
# 同一環境的另一終端：
python3 scripts/demo_transfer_pipeline.py
```

完整自動校正、全環境／附著物 PlanningScene 碰撞檢查、故障後人工恢復介面及八孔順序生產循環尚未完成。此程式不得接到硬體 domain 使用。
