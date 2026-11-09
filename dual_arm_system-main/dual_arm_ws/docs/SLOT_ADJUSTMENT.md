# C 區孔位最小調整量測方案

## 本次結論（2026-11-08）

保留原始 CAD、模擬孔位與手臂底座。本文件只提出方案；尚未套用任何位移。

優先研究八孔整體沿 **世界座標 −Y 移動 10 mm** 的獨立平台嵌件。依目前數值模型，**7 mm 是 1 mm 搜尋網格上找到的最小運動學可行位移**，但最小關節限位餘裕僅 1.27°；10 mm 方案為 5.06°。這些是目前 ±1.5 rad 模擬關節限制下的幾何結果，不代表實體允許角度或製造公差。**後續 MoveIt 包圍盒自碰撞檢查未通過，所以目前沒有可直接採用的完整可行方案。**

| 方案 | 位移 | 八孔各 31 個垂直取樣點 | 最小關節限位餘裕 | 最緊位置 |
|---|---:|---|---:|---|
| −Y 網格最小方案 | 7 mm | PASS（248 點） | 1.27° | 第 7 孔，上方 30 mm，腕關節 |
| −Y 優先驗證方案 | 10 mm | PASS（248 點） | 5.06° | 第 2 孔，上方 13 mm，肘關節 |
| +X 替代方案 | 19 mm | PASS（248 點） | 0.18° | 第 7 孔，上方 30 mm，腕關節 |
| 只提高嵌件 | 0–60 mm | FAIL（搜尋未找到完整可行方案） | — | 詳見原始量測 |

孔徑保持 25 mm；兩列 X 中心保持 −167.5 / −132.5 mm。由負 Y 至正 Y 的四排中心如下（世界座標，mm）：

| 排 | 原位置 Y | −7 mm 方案 Y | −10 mm 方案 Y |
|---|---:|---:|---:|
| 1 | −15 | −22 | −25 |
| 2 | 20 | 13 | 10 |
| 3 | 55 | 48 | 45 |
| 4 | 90 | 83 | 80 |

孔號按每排先 X=−167.5，再 X=−132.5 編號，與量測腳本一致。

重新使用目前 CAD 運動鏈做正向運動學驗證：三個方案共 744 點皆滿足既定位置與吸盤向下條件；最大數值位置殘差小於 0.000006 mm。這是求解器自洽性，**不是定位精度**。相鄰 1 mm 取樣點最大單關節變化小於 0.015 rad；連續軌跡、速度、工作站碰撞及實機校正仍為 NOT_RUN。正向與逆向檢查使用同一運動鏈，不能證明 CAD 軸向符合實機。

### 自碰撞追加量測

以隔離的 MoveIt mock 啟動查詢 `/check_state_validity`，沒有執行軌跡。使用目前 URDF 的保守零件包圍盒及 SRDF 排除規則，Arm 1 固定在模型零位：

| 方案 | 被判無效的取樣點 | 發生孔位 | 狀態 |
|---|---:|---|---|
| −Y 7 mm | 49 / 248 | 第 1、2 孔 | FAIL |
| −Y 10 mm | 62 / 248 | 第 1、2 孔 | FAIL |
| +X 19 mm | 62 / 248 | 第 1、2 孔 | FAIL |

接觸對皆為 `arm2_cad_link_2` 與 `arm2_cad_link_4`。另取每個方案、每個失敗孔位的一個代表姿態，共六點，以原始主要零件 STL 做 VTK 表面交叉檢查，這六點未發現表面交叉。這表示包圍盒可能過於保守，但不足以排除實體包含、其他零件或其他姿態的干涉。**沒有新增碰撞排除、沒有縮小包圍盒來強行通過，也沒有將此結果改判為 PASS。**

下一步應建立經驗證的網格或凸分解碰撞模型，納入平台、泡棉、杯體與雙臂動作，再重新搜尋最小位移。現在的 7 mm 是運動學搜尋結果，不能稱為最小無碰撞平台調整量。

證據：[`slot_self_collision.json`](evidence/slot_self_collision.json)、[`slot_mesh_witnesses.json`](evidence/slot_mesh_witnesses.json)。兩份資料的範圍不同，不可把六點表面檢查當成 744 點完整碰撞驗證。

建議採用可調式替換嵌件，先保留 −7 / −10 mm 兩個定位方案。移動 25 mm 孔僅 7–10 mm 會與原孔重疊，不能直接當作在原板追加鑽孔的加工圖；需重新檢查支撐底板、泡棉、白底與杯體的完整間隙。只移白底不能改變實際孔位。上述方案尚未通過加工或實機驗收。

量測依據：[`slot_adjustment.json`](evidence/slot_adjustment.json)。本次重驗：[`slot_proposal_verification.json`](evidence/slot_proposal_verification.json)，包含輸入檔與模型 SHA-256、極限點、殘差及 248 點完整性檢查。

## Study assumptions and replay

User decision: preserve source CAD and measure/propose a minimal platform/slot adjustment. No candidate adjustment has been applied to CAD or simulation slots.

Scope: numerical four-DOF CAD IK with TCP +Z pointing down, model limits ±1.5 rad, current slot floor top z=1 mm and simulated cap height 10 mm. Placement TCP z=11 mm. Each candidate checks all eight slots at 31 approach heights from contact to 30 mm above contact. This checks geometry and limits only; it does not certify collision avoidance or physical reach.

| Candidate | First feasible on 1 mm search grid | Dense approach |
|---|---:|---|
| Move all holes toward user (negative world Y) | 7 mm | PASS |
| Move all holes toward arm base (positive world X) | 19 mm | PASS |
| Raise the hole insert only | No solution found from 0–60 mm | FAIL |

Preferred direction for further validation: negative Y. At 7 mm shift, new row centers would be -22, 13, 48, 83 mm, retaining x=-167.5/-132.5 mm and 25 mm holes. This is the measured grid minimum, not a globally proven minimum or a fabrication recommendation. The joint-limit margin is small; a 10 mm candidate is also evaluated in evidence/slot_adjustment.json.

To preserve original SolidWorks files, prototype a separately versioned adjustable insert/foam template. Moving cutouts requires checking support beneath the new openings; translating white bottoms alone does not move physical CAD holes. A physical implementation may require a replacement/offset platform insert rather than drilling overlapping holes in the current plate. No machining is authorized or performed here.

Before selecting a final dimension: verify CAD TCP sign and horn axes against physical assembly, calibrate joint zeros/limits, measure real cap height and cup compliance, add collision-aware approach/withdrawal tests, then recompute margins. Absolute hardware error and collision margins are NOT_RUN. Replay: `PYTHONPATH=src/dual_arm_coordinator python3 scripts/measure_slot_adjustment.py`.
