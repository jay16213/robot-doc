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
For Udoo, See [Udoo_setup](Udoo_setup.md)

## 操作指令
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
        - [Udoo_setup](udoo_setup.md) 中有如何修改 launch 檔的教學

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
- 手動控制鍵盤對應請參考機器人指令文件

## Developmant
- See [Development](development.md)
