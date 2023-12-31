# led-matrix

[TOC]

## 介绍

基于 MM32G0001 设计的 LED 点阵屏模块，可级联，通过串口控制。



## 使用方法

### 单个模块

连接 VCC（3.3 ~ 5V），GND，DIN 引脚连接串口的 TXD。

串口配置为 1000000 波特率，8bit 数据，1停止位，无奇偶校验位。

串口发送 8 个 0xFF，则可以点亮所有 led，发送 8 个 0x00，则会熄灭所有 led。

帧内发送数据的间隔需要小于 1.5ms，每两帧数据之间的间隔需要大于 2ms。



### 多个模块

下一个模块的 VCC 连接 上一个模块的 VCC，GND 连接 GND，DIN 连接 DOUT。第一个模块的连接同单个模块。

串口配置为 1000000 波特率，8bit 数据，1停止位，无奇偶校验位。

串口发送 8\*n 个 0xFF，则可以点亮所有 led，发送 8\*n 个 0x00，则会熄灭所有 led；n 为模块数量。

帧内发送数据的间隔需要小于 1.5ms，每两帧数据之间的间隔需要大于 2ms。



### 更新固件

上电前，保证模块的 DIN 引脚为低电平，然后上电，可进入等待下载固件的状态。

模块背面的 D 触点为 SWDIO，C 为 SWCLK，G 为 GND，使用 SWD 方式下载固件。
