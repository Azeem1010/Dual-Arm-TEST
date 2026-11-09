# CAD 修正版

`dual_arm_corrected.STEP` 是目前可用的修正版交換幾何，包含雙臂及八個齊平白色填塊；Gazebo 使用同一生成流程輸出的 STL。

`dual_arm_corrected_geometry.SLDPRT` 是由修正版幾何另存的 SolidWorks 2019 多實體零件檔。已儲存並重新開啟，讀到 342 個實體、warnings=0（證據：solidworks_standalone.log）。它適合查看完整修正幾何，但不是保留原組合特徵／配合的可動裝配模型。

`native_corrected/` 的中間 SLDASM 使用 3D Interconnect 暫存引用；`native_portable/` 的 PackAndGo 回覆雖無錯誤，實際未收齊全部引用，**這兩個目錄不可當作可攜完整組合交付**。請優先開啟上面的 STEP 或獨立 SLDPRT。

`corrections.json` 記錄可重現的修正：兩臂末端支架、吸盤固定件與其相關固定件，繞 CAD 座標的吸盤軸旋轉 180°。修正後鏡頭支架向前伸出；吸盤接觸點與馬達運動軸保持原位。對照圖為 `../docs/evidence/camera_mount_corrected_side.png`，實機照片在 `../docs/reference/physical_camera_mount.png`。

生成器讀取 pinned 原始 STEP，再套用此修正清單，保持零件索引可追溯。不要把已修正的 STEP 再當作原始輸入套一次旋轉。

`working/` 是原生 SolidWorks 檔案的工作副本，**其中 SLDASM / SLDPRT 尚未完成方向修改**。本機 SolidWorks 2019（27.5）開啟組合件回傳 8192 / swFutureVersion，無法編輯較新版檔案；它們不能被當成已修正版交付。原生特徵樹與組合約束的更新需使用相容版本 SolidWorks，或由可用 STEP 另行匯入建模。原始來源仍保留在 third_party 以追溯。

生成 STEP / STL 需要獨立 CAD Python 環境；一般 ROS 使用者不必安裝 CAD 依賴即可啟動已匯出的模型。

```bash
python3 -m venv .cad-venv
.cad-venv/bin/pip install cadquery-ocp==8.0.1.1.0 vtk
.cad-venv/bin/python scripts/export_cad.py
python3 scripts/generate_cad_robot.py
python3 scripts/generate_world.py
bash scripts/build.sh
```

上述指令在 `dual_arm_ws` 執行。原生檔限制對應的官方錯誤碼：[SolidWorks swFileLoadError](https://help.solidworks.com/2019/English/api/swconst/SolidWorks.Interop.swconst~SolidWorks.Interop.swconst.swFileLoadError_e.html)。
