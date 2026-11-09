# 續接工作紀錄 — 2026-11-09

## 使用者最新要求（取代先前保留錯誤 CAD 的限制）

1. 末端小方塊是鏡頭支架，依實機照片修正前後顛倒；同步修正相關 CAD、STEP、STL、URDF／Gazebo、碰撞及說明。
2. C 區八格以白色方塊填滿，表面齊平，取消凹洞；同步更新放置高度、視覺格位說明及驗證。
3. README 清楚列出 Windows／WSL 與 Jetson Orin 下載專案後的安裝、建置、啟動及現有限制。
4. 搬運速度再適度提高，接觸段保持較低速度。
5. 預備姿勢使用韌體 home_A、home_C 的關節數字計算，不再把 CAD q=0 當作 home。
6. 持續更新本檔，額度或對話中斷後可直接續接。

## 最新狀態（優先於以下全部內容）

- 使用者提供實機零姿態：四顆馬達 raw 512、手臂伸直。已套用為模擬暫定值，換算與數字見 HOME_POSES.md。
- 兩物體搬運回歸 PASS（Gazebo 真值、無畫面），證據為 transfer_pipeline.json 與 zero512_pipeline.log。先前手動中斷的那一輪備份在 evidence/before_zero512/，不是搬運失敗。
- 待使用者提供：Arm 1 馬達 17 的三筆讀值（步驟見 HOME_POSES.md），用來取代 j1 暫定值 192.3；確認「伸直」是否指垂直朝上。
- `cm530_hardware` 與 `hardware.launch.py` 已完成並通過 Mock Serial 測試，尚未接實機。`config/calibration.yaml` 仍全為 null，故意讓驅動拒絕啟動，只能填實機量測值。
- 模擬已全部關閉，沒有進行中的工作階段。下方的 session 編號與「進行中」描述都已過時。

## 目前執行狀態

### 最新續接覆核（優先於下方歷史描述）

- 本輪新增 `cad/dual_arm_corrected_geometry.SLDPRT`：SolidWorks 已儲存並重新開啟，342 個 solid bodies、warnings=0。是多實體幾何，不是原特徵樹組合。
- PackAndGo 雖回覆 PASS files=267，實際只有組合檔與清單，沒有完整可攜引用；不把 native_portable 宣稱為完成。
- HOME named states 局部建置 PASS（home_named_states_build.log），README／HOME_POSES 已更新為模擬假設，不再要求使用者先提供實測值才能模擬。
- 新版 GUI 與兩物體測試已啟動，但 Windows 磁碟讀取導致載入很慢；本輪 exec GUI session=92015、pipeline session=39417。先看 revised_gui_launch.log / revised_pipeline.log，勿重複啟動。前一輪舊 domain43 遺留 Gazebo 2521/2522 已定點停止。

- 舊版三物體 `transfer_pipeline.json` 已 PASS，三物體落孔與雙臂同步取物有實際回饋。舊模擬已停止。
- 已完成 10 個末端零件翻向前方、STEP/STL/URDF 生成，八個白格填平至 z=14 mm；`model_revision_validation.json` PASS，TCP／軸保持不變。
- 8 個 coordinator 測試 PASS。新版全套 10 套件建置已完成；加入 HOME named states 後的局部重建正在本輪執行。
- 使用者已回覆：沒有零位方向量測，先建立模擬換算。設定已寫入 runtime.json；home_A=(-30,-10,40,60)°、home_C=(30,-10,40,60)°；是反算零位假設，hardware_use_allowed=false。
- 搬運／接觸速度改為 0.50／0.25，加速度 0.25。新白格中心高度 0.019 m。新版取放驗收尚未完成。
- README Windows／Orin 指南、啟動腳本已完成。需依本輪結果更新狀態。
- 本機 SolidWorks 2019 打不開較新版原始 SLDASM（8192）。已成功從修正版 STEP 匯入新 `cad/native_corrected/dual_arm_corrected_geometry.SLDASM`，但引用仍指向 SolidWorks 暫存目錄；**不可單獨交付此檔**。`scripts/package_native_cad.ps1` 正在處理 PackAndGo 引用封裝問題。
- 上次修正版 smoke_runtime 已記錄 FAIL：三相機、控制器、網格 PASS，部分 views 未收到，動作等待太短且 shutdown 未通過。已將 action 等待 15 秒改為 90 秒，尚待重驗，不可將更長等待當成通過。

- 舊版三物體管線在 ROS_DOMAIN_ID=43、GZ_PARTITION=dual_arm_pipeline_demo 執行中。啟動 exec session=48173，搬運 session=93915。
- 2026-11-09 接到新要求時，cap_5 已放 C 第3孔、cap_6 已完成 C 放置，cap_4 正在 B 交接流程。**先讀 docs/evidence/transfer_pipeline.json / .log 確認最終狀態，勿重跑或假設已完成。**
- 原單物體流程在 C 孔底姿態 TCP 誤差 >1 mm 停止；三物體版本改於孔口上方釋放，已驗證至少第一個實際落孔。新齊平白格需重新驗證，不可沿用凹洞高度。
- 舊版流程為模擬真值定位，非已校正的視覺自動取放；完整工作站與負載 PlanningScene 驗收仍未完成。

## 待辦與進度

- [ ] 鏡頭支架 CAD 零件辨識、方向修正及原生 SolidWorks / STEP 輸出。
- [ ] 重新生成並檢查 STL、URDF 視覺／碰撞與 Gazebo。
- [ ] C 八格白色齊平實體填塊與所有高度使用處更新。
- [ ] HOME 原始值換算設定、姿態計算與 named states，待零位方向確認。
- [ ] 速度集中設定與腳本整合。
- [ ] Windows、Orin 使用指南及根目錄 README。
- [ ] 建置、模型一致性、模擬啟動、實際取放重驗與結果紀錄。

## 已知來源與工具

- 照片：`C:/Users/39165/AppData/Local/Temp/codex-clipboard-1fc3dd90-212f-44af-890f-72f8ab0c955d.png`。應複製到專案 docs/reference，避免暫存檔消失。
- 韌體來源：`third_party/CM530_ROS_BRIDGE/17 ROS to CM530 ver. dual arm/APP/inc/arm_config.h`。
- home_A=[90,567,712,825]、home_C=[590,567,712,825]；原始位置值不是 URDF rad。CAD 零位 raw offset / 方向尚缺，已透過非同步問題詢問使用者。
- SolidWorks COM `SldWorks.Application` 註冊存在，尚未驗證自動化能否啟動。沒有授權修改來源 checkout；工作副本應置於 `cad/`，匯出與生成器改指向工作副本。
- CAD 主組合：`third_party/dual_arm_3d/雙臂_交付/完成_雙臂總組合.sldasm`；STEP：同目錄 `雙臂總組合.STEP`。
- 末端主零件 arm1 component_010、011、019；arm2 component_231、232、240。尚不能僅憑名稱假定哪個是鏡頭支架，需檢視幾何。
- WSL Ubuntu 中 ROS Jazzy / Gazebo Harmonic 已安裝；OpenCascade / VTK 在 `.cad-venv`。
- 建置：`bash scripts/build.sh`。不可直接重跑早期 `integrate_description.py`，會覆寫後續設定。

完成每項後更新待辦、證據路徑、假設與未完成部分。不得將假設的 HOME 轉換標為實機校正通過。
