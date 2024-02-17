| Supported Targets | ESP32-S3 |
| ----------------- | -------- |

# _ESP32投影矫正_

ESP32实时获取陀螺仪数据，计算后输出矫正画面，使得投影在某固定平面上的画面始终不变形

## 硬件设计
**esp32s3n8r8**主控，**bno085**陀螺仪，**lcos**投影模块(**RGB**接口)

## 软件架构
考虑到陀螺仪传感器的可移植性，本项目采用`espidf`和`arduino`共同使用开发



```
├── CMakeLists.txt
├── mains
│   ├── CMakeLists.txt
│   ├── main.c
│   └── ...
├── ...
└── README.md                  This is the file you are currently reading
```
