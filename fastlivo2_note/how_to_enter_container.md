# 🚀 Fast-LIVO2 操作小抄 (Jetson AGX Orin)

**ssh遠端連線前置作業：**
請確認你從電腦連線時有加 `-X` 
參數：
`ssh -X robot@<Jetson_IP>`

-----

### 視窗 1：啟動主程式 (RViz)

*(請依序執行以下指令)*

**1. 允許 Docker 使用螢幕 & 更新畫面授權 (關鍵步驟)**

```bash
xhost +local:docker
cat ~/.Xauthority | docker exec -i fastlivo_run sh -c 'cat > /root/.Xauthority'
```

**2. 喚醒並進入容器**

```bash
docker start fastlivo_run
docker exec -it fastlivo_run /bin/bash
```

**3. 修正驅動並啟動程式 (在容器內)**
*(如果之前有寫入 .bashrc，這兩行其實會自動執行，但手動打最保險)*

```bash
export LD_PRELOAD=/lib/aarch64-linux-gnu/libusb-1.0.so.0
source /root/catkin_ws/devel/setup.bash

# 啟動 Fast-LIVO (這時 RViz 視窗應該要跳出來)
roslaunch fast_livo mapping_avia.launch
```

-----

### 視窗 2：播放數據 (需開新的 SSH 視窗)

*(當視窗 1 已經跑起來後，開第二個視窗)*

**1. 連線並進入同一個容器**

```bash
# 記得換成你的 IP
ssh -X robot@<Jetson_IP>

# 直接進入
docker exec -it fastlivo_run /bin/bash
```

**2. 前往數據資料夾並播放**

```bash
cd /root/catkin_ws/bag
rosbag play hku_main_building.bag
```

-----

### 🚪 如何安全退出？

1.  在 **視窗 2** (播放數據) 按 `Ctrl + C` 停止播放。
2.  在 **視窗 1** (RViz) 按 `Ctrl + C` 關閉程式。
3.  在兩個視窗都輸入 `exit` 離開容器。

-----

### ⚠️ 絕對禁止事項 (Don'ts)

  * ❌ **不要再執行 `docker run ...`**：這會建立一個全新的空環境，導致之前裝好的依賴全部消失。請務必使用 `docker start` 和 `docker exec`。
  * ❌ **不要執行 `docker rm fastlivo_run`**：這會把整個環境刪除。

  ---

  如果不使用 SSH，直接坐在 Jetson 前面（接上螢幕、鍵盤、滑鼠）操作，其實**更簡單**，因為不需要處理網路傳輸畫面，速度會快非常多！

但在本機操作有一個關鍵點：**你需要告訴容器，螢幕的位置改變了**（從網路端變成了本機端）。

以下是本機操作的操作步驟：

### 🖥️ 本機操作小抄 (直接在 Jetson 上)

請打開終端機 (`Ctrl` + `Alt` + `T`)，然後執行以下步驟：

#### 1\. 開放本機顯示權限

這步跟 SSH 一樣，要允許 Docker 畫圖在你的螢幕上。

```bash
xhost +local:docker
```

#### 2\. 喚醒容器 (如果它沒在跑)

```bash
docker start fastlivo_run
```

#### 3\. 進入容器 (⚠️ 關鍵差異點)

這裡跟 SSH 不一樣！因為你之前建立容器時，`DISPLAY` 變數可能記住了 SSH 的位置 (例如 `localhost:10.0`)。
現在我們在本機，必須用 `-e` 參數**強制覆蓋**成目前的螢幕設定 (通常是 `:0` 或 `:1`)。

```bash
# 注意多了 -e DISPLAY=$DISPLAY 這段
docker exec -it -e DISPLAY=$DISPLAY fastlivo_run /bin/bash
```

#### 4\. 執行程式 (跟之前一樣)

進去之後，剩下的動作完全一模一樣：

```bash
# 1. 掛載 USB 驅動防崩潰 (如果在 .bashrc 加過了可省略)
export LD_PRELOAD=/lib/aarch64-linux-gnu/libusb-1.0.so.0

# 2. 載入環境
source /root/catkin_ws/devel/setup.bash

# 3. 啟動 (RViz 畫面會瞬間跳出來，不會延遲)
roslaunch fast_livo mapping_avia.launch
```

-----

### 💡 為什麼要加 `-e DISPLAY=$DISPLAY`？

  * **情境：** 當你用 SSH 連線時，你的 `$DISPLAY` 名字叫做 `localhost:10.0`。
  * **情境：** 當你在本機操作時，你的 `$DISPLAY` 名字叫做 `:0`。
  * **問題：** 容器啟動後，它會傻傻地記住第一次設定的值。如果你昨天用 SSH 建的容器，今天在本機打開 RViz，它會試圖把畫面送到 `localhost:10.0` (網路端)，結果就是本機螢幕一片黑，什麼都沒發生。
  * **解法：** 在 `docker exec` 時加上 `-e DISPLAY=$DISPLAY`，就是告訴容器：「忘記之前的設定，現在請把畫面送到我眼前這個螢幕！」