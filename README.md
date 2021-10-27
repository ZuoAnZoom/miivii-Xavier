# miivii-Xavier

小车相关代码备份，以及数据采集等相关说明

## 数据采集说明

1. 小车需要先从千斤顶上卸下。
2. 请先打开小车侧面的电源，为红色把手。横为打开，竖为关闭。
3. 再打开小车后面的黑色旋钮开关，等待小车自检通过。小车自检会滴滴叫，不叫了就通过了。
4. 检查米文大脑摄像头是否都连接上，路由器是否正常连接，车载屏幕是否连接。
4. 给米文大脑供电，电源电压注意需要先调整到 16V。
5. 米文大脑开机，红色按钮按一下，线上有标明 POWER ONKEY。
6. 开机后，桌面上有 5 个脚本，分别为：

|序号|脚本名|功能|注意|
|:---:|:---:|:---:|:---:|
|1|open_car_chassis.sh|启动车辆底盘|请确保小车电源已打开，并且车底盘开关已打开且自检通过|
|2|open_camera.sh|打开 8 个摄像头|/|
|3|start_key_control.sh|开启键盘控制功能|按 q 退出控制，会提示是否同时关闭车底盘|
|4|record_camera.sh|开始记录摄像头数据|注意查看A组和B组摄像头的录制 FPS，需要按提示选择录制哪一组摄像头的数据|
|5|kill_chassis.sh|手动关闭车辆底盘|/|

7. 在新终端内，启动脚本 1, 确保车底盘已经打开，并能显示“转向”、“油门”、“刹车” 信息。
8. 在新终端内，启动脚本 3, 就可以用键盘控制小车了。
9. 若需要记录摄像头数据，在新终端内先启动脚本 2, 再在新终端内启动脚本 4, 按照脚本提示操作，即可完成数据采集。


## 注意事项

1. 数据采集前，请先确保小车电量是否充足，米文大脑移动电源电量是否充足，若需要用 WIFI，请确保路由自身电量是否充足。
2. 数据采集前，务必检查硬盘的剩余空间，并且确认 A 组和 B 组相机的 FPS值。
3. 数据采集前，务必确认需要采集哪些摄像头的数据，以及是否需要对摄像头进行标定。
4. 不要随意 `sudo apt upgrade`，以避免不必要的包升级导致的代码兼容性问题。
5. 米文大脑的两个 USB 接口容易出现问题，务必在开机前先用 USB-hub 接入其中一个 USB 接口，并在 USB-hub 上连接键盘和鼠标使用。






