# arm1 舊版對應實機定位結果

2026-11-05，使用者確認舊版四軸零點 512、方向 +1 已在目前手臂實測有效，吸盤／底座／桌面位置未變，並確認現場淨空、arm2 在活動範圍外及可立即停止。

本次使用 `manual_xy_legacy_confirmed.yaml`，以舊版 end_effector 為目標，將新 TCP 別名設為零偏移，保留舊版 10 mm 工具碰撞盒及其 +Z 5 mm 中心。目標是舊版 end_effector 座標，不宣稱已量得實際吸盤接觸點。

使用者確認的既有校正可以作為另一種資料來源，無須捏造三組量測樣本。此設定記錄來源及確認項目；現場淨空確認僅限當次時段，逾時拒絕新的執行請求。原始量測模板與其檢查仍保留。

## 執行結果

- 第一次 VERSION 回覆 ERR,BAD_CMD；使用者確認第 17 版後，重新建立唯讀連線成功，VERSION/PING/GET_HOME/READ 全數通過。單次錯誤原因未確定。
- 硬體模式、ROS domain 165；arm1 開始 raw `[229,490,514,500]`。
- MoveIt 成功規劃目標 `(0.15,0.04,0.05)` m，速度／加速度比例 0.05，預覽約 66 秒。
- HOLD→TORQUE 啟用成功，實際執行 AX 軌跡，最後指令 `[563,628,708,813]`。執行期間讀值持續改變；MoveIt／bridge 的關節到位檢查回報成功。
- 手動模式的末端平面驗證未通過 1 mm 條件，已送 `HOLD,arm1` 並收到 `OK,HOLD,arm1`，後續運動鎖定，沒有自動重送。
- 停止後 READ `[561,633,710,812]`，模型 FK 約 `(0.150000,0.038428,0.045392)` m；相對目標 Z 約低 4.61 mm，三維位置差約 4.87 mm。這是讀值換算，不是外部量測。
- arm2 未收到運動命令。arm1 保持 HOLD 扭力，未自動斷扭力。

這次確認了「MoveIt 求解 → ROS 軌跡 → CM530 → 實體關節運動」鏈路可用，但沒有通過末端高度精度驗證。應檢查機構負載、馬達跟隨誤差與座標對應，再決定下一次動作；不要只放寬容差或自動重送。

## 後續照片診斷

使用者提供的現場照片顯示吸盤朝左側，A 區在右側；這與目前把 `(0.15,0.04)` 當作 A 方向的假設不一致。停止後底座 raw 為 561，而板上 arm1 的第 17 版 HOME 預設底座 raw 為 90；舊版單臂韌體的 HOME 則是四軸 512。手動模式在本次測試中只讀取 HOME，沒有執行 HOME，因此手臂確實不在第 17 版 HOME。HOME 是 raw 預設值，並不保證朝向 A，也不是碰撞檢查過的路徑。

照片加上讀值說明模型 `arm1_base` 到實體 A 區的方向仍需核對，不能只用是否回 HOME 解釋。已將舊版硬體設定中的 `physical_base_alignment_confirmed` 設為 false，並讓硬體模式拒絕再次啟動，直到實體方向與模型座標對齊。本次沒有再發送 HOME 或其他運動命令。

## 檔案

- `validation/manual_xy_hardware_start.json`
- `validation/manual_xy_hardware_plan.json`
- `validation/manual_xy_hardware_after_hold.json`
- `validation/manual_xy_hardware_result.json`

本次修改後 89 項測試通過。CM530 原始資料夾未修改，未燒錄韌體。
