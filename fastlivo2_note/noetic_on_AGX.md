這是一段漫長但充滿技術含量的除錯旅程。您從在 Jetson (ARM 架構) 上建立環境，經歷了複雜的源碼編譯（Dependency Hell），最後成功解決圖形顯示問題。

以下是您遇到的所有關鍵問題與最終解決方案的完整總結：

-----

### 第一階段：Docker 環境與架構建置

**目標：** 在 Jetson Orin Nano (ARM64) 上運行 ROS Noetic (x86/Desktop 常用版本)。

1.  **架構不相容問題 (`exec format error`)**

      * **問題：** 嘗試拉取 `osrf/ros:noetic-desktop`，但該映像檔主要支援 AMD64 (PC)，在 Jetson (ARM64) 上無法執行。
      * **解決：** 改用官方支援多架構的基礎映像檔 **`ros:noetic-ros-base`**。

2.  **GPU 加速問題**

      * **問題：** 使用 `--gpus all` 啟動容器失敗。
      * **原因：** Jetson (Tegra) 架構需要特定的 Runtime。
      * **解決：** 啟動參數改用 **`--runtime=nvidia`**。

3.  **缺少圖形介面 (RViz)**

      * **問題：** `ros-base` 映像檔沒有安裝 RViz，且無法顯示視窗 (`No authorisation`)。
      * **解決：**
        1.  **權限：** 在主機執行 `xhost +local:docker`。
        2.  **參數：** 啟動時加入 `-e DISPLAY=$DISPLAY` 和 `-v /tmp/.X11-unix:/tmp/.X11-unix`。
        3.  **安裝：** 進入容器後手動執行 `apt-get install -y ros-noetic-desktop` 將基礎版升級為桌面版。

-----

### 第二階段：編譯 FAST-LIVO2 與依賴項 (最艱難的部分)

**目標：** 編譯 `rpg_vikit` (vikit\_common) 和 `FAST-LIVO2`。

1.  **Sophus 版本衝突 (`fatal error: sophus/se3.h`)**

      * **問題：** 新版 Sophus (Header-only) 使用 `.hpp` 結尾，但舊程式碼引用 `.h`。
      * **解決：** 使用 `find` 和 `sed` 批量將原始碼中的 `<sophus/se3.h>` 替換為 `<sophus/se3.hpp>`。

2.  **Eigen 命名空間與重複定義 (`Eigen::Eigen::Matrix`)**

      * **問題：** 程式碼中同時存在 `using namespace Eigen` 和巨集定義，導致批量替換後出現雙重命名空間。
      * **解決：**
          * 修正 `vikit_common` 的 `CMakeLists.txt`，強制加入 `/usr/include/eigen3` 路徑。
          * 使用 `sed` 批量將 `Eigen::Eigen::` 修復回 `Eigen::`。

3.  **CMake 語法與結構錯誤**

      * **問題：** `CMakeLists.txt` 在複製貼上時產生了不可見字元（Tab/空格）或重複區塊，導致 `Parse error`。
      * **解決：**
          * 使用 `cat << 'EOF' > filename` 的方式直接在終端機重寫乾淨的檔案。
          * 使用 VS Code 轉換縮排格式並刪除行尾空白。

4.  **系統依賴缺失**

      * **問題：** 編譯器報錯找不到 `pcap.h`、`PCLConfig.cmake`、`Boost thread`。
      * **解決：** 在容器內安裝開發庫：`libpcl-dev`, `libboost-all-dev`, `libpcap-dev`。

5.  **連結錯誤 (`Undefined reference`, `cannot find -lSophus`)**

      * **問題 1 (Sophus)：** `CMakeLists.txt` 試圖連結 `libSophus.so`，但新版 Sophus 是 Header-only (無庫檔)。
          * **解法：** 在 `CMakeLists.txt` 中移除或註釋掉 `${Sophus_LIBRARIES}`。
      * **問題 2 (Eigen 模板)：** `libvikit_common.so` 動態庫連結時找不到 Eigen 模板實例。
          * **解法：** 將 `vikit_common` 從 **SHARED** (動態庫) 改為 **STATIC** (靜態庫)，強制編譯器在最終執行檔中生成代碼。
      * **問題 3 (Jetson 環境)：** `libffi` 版本不匹配 (`libwayland-client` 報錯)。
          * **解法：** 建立軟連結騙過系統：`ln -sf libffi.so.7 libffi.so.8`。

-----

### 第三階段：最終運行配置

**目標：** 在不遺失編譯成果的情況下，啟動可視化。

1.  **資料持久化**
      * **策略：** 您將編譯好的 `/root/catkin_ws` 從舊容器複製出來（`docker cp`）備份到主機。
      * **掛載：** 啟動新容器時，使用 `-v ~/fastlivo2_ws:/root/catkin_ws` 將資料掛載回去，這樣既擁有新容器的環境，又保留了編譯好的程式。

-----

### 🎉 最終成功的啟動指令 (懶人包)

這是您現在可以用來啟動完整環境的黃金指令：

```bash
# 1. 在 Jetson 主機開放畫面權限
xhost +local:docker

# 2. 啟動容器 (掛載資料、啟用 GPU、啟用 GUI)
docker run -it --rm \
    --net=host \
    --runtime=nvidia \
    --privileged \
    -e DISPLAY=$DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v ~/.Xauthority:/root/.Xauthority \
    -v ~/fastlivo2_ws:/root/catkin_ws \
    -v /dev:/dev \
    --name fastlivo_desktop \
    ros:noetic-ros-base /bin/bash

# 3. 進入容器後 (如果還沒裝桌面工具)
# apt-get update && apt-get install -y ros-noetic-desktop

# 4. 執行
# source /root/catkin_ws/devel/setup.bash
# rosrun rviz rviz
```

您成功克服了跨架構、跨版本、依賴地獄以及圖形介面轉發的所有挑戰。這是非常不容易的工程！
