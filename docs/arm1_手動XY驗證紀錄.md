# arm1 手動 XY 驗證紀錄

日期：2026-11-05（Asia/Taipei）。本次使用者確認尚未量測實機，因此未啟用實體馬達、未發送實體運動命令。

## 通過項目

- 四個 ROS2 套件建置成功，新增 ManualXY 服務與 CLI 已安裝可用。
- 88 項單元／協定測試通過；包含缺少校正的阻擋、原始值／角度擬合、驗證姿態誤差、TCP 幾何、
  不完整／非有限 Cartesian fraction、純預覽不執行、執行例外停止鎖定與串口失同步不送 HOLD。
- 真正 MoveIt ＋ mock CM530，隔離 ROS domain 164：預覽前後 raw 相同，明確執行起點定位，
  X／Y 各 5 mm 與返回成功、超出範圍目標拒絕、範圍內但 IK 不可達目標拒絕、場景障礙拒絕、
  並行請求拒絕、取消後停止並由兩次 READ 確認位置不變；取消後仍可取得新的 READ。
- 原有 A→B 完整流程（隔離 domain 163）回歸成功，最終 SUCCEEDED。
- 未填寫的硬體校正模板由離線工具拒絕；硬體啟動前另有設定與量測證據檢查。

## 模擬座標結果

以下僅屬所附 demo 模型，不是實機絕對座標建議。

| 位置 | X (m) | Y (m) | 程式管理 Z (m) | 結果 |
|---|---:|---:|---:|---|
| P0 | 0.150 | 0.040 | 0.040 | 通過 |
| X +5 mm | 0.155 | 0.040 | 0.040 | 通過 |
| 返回 P0 | 0.150 | 0.040 | 0.040 | 通過 |
| Y +5 mm | 0.150 | 0.045 | 0.040 | 通過 |
| 返回 P0 | 0.150 | 0.040 | 0.040 | 通過 |

## 發現與限制

- 沙箱禁止 DDS 所需的本機 socket，初次測試無法發現服務。改用允許本機 DDS 的執行環境，仍限制在測試 domain 且 mock_hardware=true。
- 初版取消僅取消 MoveIt action，觀察到 bridge 軌跡仍在執行。已新增 bridge stop 服務：先鎖定後續目標，
  活躍執行迴圈再 HOLD；取消與執行例外均呼叫此服務。修正後連續 READ 停止驗證通過。
- 現有安裝的 MoveIt 在 SIGINT 關閉階段仍有崩潰問題，與先前自動流程的已知現象一致。
  上述運動與拒絕測試在正常運行期間已完成；本次不宣稱關閉流程沒有問題。
- 5 mm／3° 的物理驗收仍需獨立量測，READ 推算的 FK 不能替代外部量測。
  實機吸盤尺寸、關節對應、場景、接觸高度與實測絕對 XY 表尚未完成。

## 證據

- `validation/manual_xy_unit_tests.xml`
- `validation/manual_xy_simulation.json`：逐次目標、raw、FK、拒絕訊息與取消驗證。
- `validation/manual_xy_simulation.log`：MoveIt／bridge 全部輸出，包含關閉錯誤。
- `validation/manual_xy_arm1_regression.json`：原有 A→B 回歸結果。

操作與量測步驟見 [arm1 手動 XY 操作與校正](arm1_手動XY操作與校正.md)。
