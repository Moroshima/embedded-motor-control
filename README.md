# 嵌入式控制系统课程设计

> 本课程设计由任课老师所提供的参考资料可从本仓库的 Releases 中下载

Course Design of Embedded Control System

本课程设计直流电机调速控制系统，并进行仿真和实验验证。具体内容包括：

1. 建立直流电机的数学建模，设计转速反馈单闭环直流调速系统并进行仿真验证；
2. 设计系统硬件电路，完成硬件电路连接和测试；
3. 设计PID控制程序，实现电动机转速控制，给出不同参数下的系统输出变量变化曲线图，分析实验结果得出相关结论；
4. 实现下述功能：① 设置转速并实时显示当前转速；② 当转速达到设定转速时，绿色LED灯常亮；③当前转速与设定转速波动大于5%时，异常报警。
5. 分析系统在突然起动、突加负载、突减负载等条件下的电机动态特性。

课程设计详细要求参见 `嵌入式控制系统课程设计.docx`

## 环境配置

> [GNU Arm Embedded Toolchain](https://developer.arm.com/downloads/-/gnu-rm) 目前已被 [Arm GNU Toolchain](https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads) 取代，且可无痛升级，故在此以为替代。

- [STM32CubeMX](https://www.st.com/en/development-tools/stm32cubemx.html)
- [Visual Studio Code](https://code.visualstudio.com/)
- [Arm GNU Toolchain](https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads)
- [MinGW](https://osdn.net/projects/mingw/releases/)
- [OpenOCD](https://gnutoolchains.com/arm-eabi/openocd/)
- [J-Link / J-Trace](https://www.segger.com/downloads/jlink/)

### 使用 Scoop 避免手动配置环境变量

> 使用 Scoop 对于网络环境有一定的要求，请务必注意

考虑到 MinGW 与 OpenOCD 在安装完成后需要进行一定的配置，无法做到 OOTB，因此个人推荐使用 [Scoop](https://scoop.sh/) 对此二者进行安装（非常幸运的是，Scoop 确实提供了对应的包）。

#### 安装命令

如果你使用的是 Powershell 7，可以直接执行如下命令来安装 Scoop 与我们所需要的依赖：

##### 安装 Scoop

```powershell
irm get.scoop.sh | iex
```

##### 安装 MinGW

```powershell
scoop install mingw
```

##### 安装 OpenOCD

```powershell
scoop install openocd
```

#### 检查环境变量

重启终端即可使用如下命令查看两者是否被正确安装并添加入环境变量：

```powershell
gcc --version
```

```powershell
openocd --version
```

### ST-LINK 与 J-Link 驱动安装

个人在项目开发时使用了 ST-LINK/V2 进行烧录，同时使用 J-Link 仅用于串口调试。

#### ST-LINK

我们仅需要安装驱动，因此可以直接下载 [OpenOCD](https://gnutoolchains.com/arm-eabi/openocd/) 的压缩包并解压，然后在 `drivers/ST-Link` 下找到与操作系统对应的 `dpinst_x.exe` 安装包进行安装（在这里我们下载 OpenOCD 的压缩包仅是为了利用其中的 ST-LINK 驱动程序安装包，并不涉及使用其进行环境配置）。

#### J-Link

直接从 [J-Link / J-Trace](https://www.segger.com/downloads/jlink/) 下载安装包安装即可，需要注意安装时请务必勾选安装驱动，个人建议默认配置项安装即可。

### VSCode 插件

实际开发中，我们所需用到的核心插件只有如下两个：

> 注意安装 C/C++ 并不需要安装 C/C++ Extension Pack ~~（虽然VSCode右下角一直在弹框提示安装就是了）~~

- [C/C++](https://marketplace.visualstudio.com/items?itemName=ms-vscode.cpptools)
- [Makefile Tools](https://marketplace.visualstudio.com/items?itemName=ms-vscode.makefile-tools)

## 打开项目

双击 `embedded.ioc` 即可以在 STM32CubeMX 中打开项目配置，需要注意本项目并未使用 [Arm Keil MDK](https://www.keil.com/download/) 进行开发，因此无法直接使用 Keil µVision 打开该项目。

## 编译与烧录

### CLI

#### 编译

因为本项目在 STM32CubeMX 中 Project Manager > Project > IDE/ToolChain 配置为了 Makefile，因此直接执行：

```powershell
make
```

即可

#### 烧录

在确保 ST-LINK 驱动被正确安装且被正确插入的情况下执行如下命令：

```powershell
openocd -f interface/stlink.cfg -f target/stm32f4x.cfg -c "program build/embedded.hex verify reset exit"
```

### Tasks

> 如何运行 tasks 参见 [Tasks in Visual Studio Code](https://code.visualstudio.com/docs/editor/tasks)

为了简化开发流程，个人对 tasks 进行了一定的配置，因此直接执行即可。

## 串口调试

使用 [VOFA+](https://www.vofa.plus/) 并将数据引擎设置为 RawData 后点击界面左上角连接按钮即可连接单片机串口开始调试。

## 附录

### 接线参考

#### 核心板

| STM32F401CDUx 核心板 | ST-LINK V2 |
| -------------------- | ---------- |
| GND                  | GND        |
| SCK                  | SWCLK      |
| DIO                  | SWDIO      |
| 3V3                  | 3.3V       |

#### 底板

| 底板 | J-OB V2 |
| ---- | ------- |
| 3.3  | -       |
| TX   | RXD     |
| RX   | TXD     |
| GND  | GND     |

#### 电机

电机除要正确方向连接排线外，还需要从 9-12V 断口处引线至电机驱动供电接口。

## 参考

- [配置VS Code 开发STM32【宇宙&最强编辑器】 - 知乎 (zhihu.com)](https://zhuanlan.zhihu.com/p/468568448)
- [为了爽快地调试硬件，我跟串口调试助手杠上了 - 知乎 (zhihu.com)](https://zhuanlan.zhihu.com/p/54671276)
- [【STM32】HAL库 STM32CubeMX教程六----定时器中断 - 古月居 (guyuehome.com)](https://www.guyuehome.com/36007)
- [STM32 Timers Explained Tutorial - Timer Modes Examples Interrupts pwm (deepbluembedded.com)](https://deepbluembedded.com/stm32-timers-tutorial-hardware-timers-explained/)
- [【STM32CobeMx】利用定时器输出PWM_counter period_米杰的声音的博客-CSDN博客](https://blog.csdn.net/Roger_717/article/details/119699868)
- [STM32_7——TIMER定时器实验 - 知乎 (zhihu.com)](https://zhuanlan.zhihu.com/p/525439399)
- [PID控制器 - 维基百科，自由的百科全书 (wikipedia.org)](https://zh.wikipedia.org/wiki/PID控制器)

除却以上参考资料，OpenAI ChatGPT 对本项目开发也起到了莫大的帮助。

## License

[GPLv3](https://github.com/Moroshima/embedded-motor-control/blob/master/LICENSE)
