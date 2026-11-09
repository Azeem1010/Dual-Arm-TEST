# 視覺介面鏡像

此套件的 `msg/`、`srv/`、`CMakeLists.txt` 與 `package.xml` 取自工作區內
`three-camera-vision-ros2-main/vision_interfaces`。主控與視覺節點共用同一個
`vision_interfaces` 建置結果；建置時只加入 `src/vision_interfaces`，不要同時加入
上游的同名套件。

若視覺端更新介面，須同步以上檔案，並在建置前檢查兩邊的 `msg/` 和 `srv/`
完全相同。主控目前使用 `CaptureRanking`、`GetZoneState`、`GetPickTarget`、
`GetGridState`、`VerifyPlace`；其餘訊息供視覺節點及監看工具使用。
