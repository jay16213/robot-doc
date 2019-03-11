# 爬梯機器人使用手冊

本文件主要說明爬梯機器人的硬體配置及基本操作。

> 建議先閱讀 [ROS Tutorials](http://wiki.ros.org/ROS/Tutorials) 第一章, 才能理解指令及開發流程

## Hardware

- 主控板: [Udoo](https://www.udoo.org/)
  - OS: Ubuntu 16.04 for udoo
  - ROS Version: Kinetic
  - 共有兩張, 上面那張閒置中, 下面的才是

- 超音波 sensor (HC-SR04)

- 雲台馬達 (Dynamixel)
    - 共兩顆, 分別控制上下、左右

- 履帶馬達
    - 共有四顆, 2顆控制手臂舉放, 2顆控制行走
    - 本文件將以手臂馬達、行走馬達分別稱之

- Dlink AP

- 爬梯功能需要用到以下設備
    - Nvidia TX2
    - [ZED](https://www.stereolabs.com/)

### 供電
#### 4顆 12V 電池
- 共供給 2 張 Udoo, 1台AP, 1個雲台馬達
    - 目前 1 張 Udoo 閒置, AP 吃行動電源的電, 有 2 顆閒置電池
    - 雲台馬達需用轉接頭才能接上電池

#### 2顆 24V 電池
- 供給行走、手臂馬達


#### 1顆可供 110V 的行動電源
- 供給 TX2, AP

> **NOTE**
> - 12V, 24V有專用充電器, 平均充2小時就會滿 (充電器有指示燈), 兩個充電器皆無充滿自動斷電功能，所以盡量不要充過夜
> - 12V專用充電器需用雲台馬達的轉接頭才能接上電池
> - 雲台馬達最耗電, 使用完務必要充飽以免下次實驗無法使用

## Environment Setup
For Udoo, See [udoo_setup](udoo_setup.md)

## 基本操作指令
> Udoo ip: 192.168.0.197

### ssh 進 Udoo
```bash
ssh udooer@<ip_address> # password: udooer
```

### 開啟所有馬達 (行走 + 手臂 + 雲台)
```bash
roslaunch tracked_robot all_in_one.launch # press Ctrl+C to exit
```
- 成功開啟會看到雲台馬達的型號敘述及 `Initialization is completed ...` 等輸出, 如有紅字輸出代表有錯誤, 請檢查
    1. 電源是否有電且開啟
    2. 線路有無脫落
    3. 雲台馬達的 device name 是否跑掉造成程式找不到 (正確為 `/dev/ttyUSB1`)
        - [udoo_setup](udoo_setup.md) 中有如何修改 launch 檔的教學

- 單獨開啟行走及手臂馬達
    ```bash
    rosrun tracked_robot Motor_node
    ```
- 單獨開啟雲台馬達
    ```bash
    roslaunch my_dynamixel_workbench_tutorial position_control.launch
    ```

### 開啟手動控制介面
```bash
rosrun tracked_robot Manual_node # press e to exit
```

#### 手動控制鍵盤對應
##### 雲台馬達
| 按鍵      | 指令         |
| --------- | ------------ |
| 9         | 回復初始位置 |
| o         | 上           |
| l (小寫L) | 下           |
| i         | 左           |
| p         | 右           |

##### 履帶馬達
| 按鍵  | 指令                                    |
| ----- | --------------------------------------- |
| Up    | 前                                      |
| Down  | 後                                      |
| Left  | 左                                      |
| Right | 右                                      |
| a     | 前腳抬起                                |
| z     | 前腳放下                                |
| s     | 後腳抬起                                |
| x     | 後腳放下                                |
| space | 停止所有動作 (不包含雲台馬達)           |
| r     | 設定行走速度 = 350                      |
| f     | 設定行走速度 = 1000                     |
| u     | 行走速度增加 (一次+500, 上限 30000)     |
| y     | 行走速度減少 (一次-500, 下限 1000)      |
| j     | 前後腳抬起 90 度 (用於一開始之手臂抬起) |

> **NOTE** 遇到無法控制情形時請檢查
> 1. 是否按到 caps lock (程式僅接受小寫字母)
> 2. all_in_one.launch 有無噴錯誤訊息導致馬達沒有正確開啟
> 3. ssh 是否斷線, wifi 是否正常
>
> 指令不要連按, 否則可能會造成 command queue 塞滿導致指令無法及時反應

## Developmant
- See [Development](development.md)
