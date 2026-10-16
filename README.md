# example_soem

这是一个基于 C 和 SOEM 的最小 EtherCAT 电机控制示例，用于通过 EtherCAT 网关发送 Encos
电机 CAN 报文。

## 构建

```bash
git submodule update --init external/SOEM
cmake -S . -B build
cmake --build build
ctest --test-dir build --output-on-failure
```

## Windows 10 x64 / Visual Studio 2017

Windows 构建支持 Visual Studio 2017 x64。EtherCAT 使用 SOEM 的 Win32 后端和
Npcap；Qt 界面在 Windows 上提供 UDP-CAN 和 TCP-TCP。SocketCAN 和
`socketcan_8motor_simulator` 仅在 Linux 上构建。

准备环境：

1. 安装带有“使用 C++ 的桌面开发”工作负载的 Visual Studio 2017。
2. 安装 CMake 3.14 或更高版本，并加入 `PATH`。
3. 安装 Qt 5.12 的 `msvc2017_64` 套件。
4. 安装 Npcap，并在安装器中选中 **Install Npcap in WinPcap API-compatible
   Mode**。EtherCAT 运行时需要管理员权限。

在 **x64 Native Tools Command Prompt for VS 2017** 中执行（按实际 Qt 路径修改）：

```bat
cd C:\path\to\ENCOS
cmake -S . -B build-vs2017 -G "Visual Studio 15 2017" -A x64 ^
  -DCMAKE_PREFIX_PATH=C:\Qt\5.12.12\msvc2017_64
cmake --build build-vs2017 --config Release
ctest --test-dir build-vs2017 -C Release --output-on-failure
```

运行 Qt 界面：

```bat
C:\Qt\5.12.12\msvc2017_64\bin\windeployqt.exe ^
  build-vs2017\Release\qt_motor_control_ui.exe
build-vs2017\Release\qt_motor_control_ui.exe
```

运行 EtherCAT demo 时，Windows 网卡参数不是 Linux 的 `eth0`，而是 Npcap
设备名称。先列出可用接口，再把第一列完整复制为 `<ifname>`：

```bat
build-vs2017\Release\example_soem_demo.exe --list-adapters
```

## 运行权限

可以使用 root 运行：

```bash
sudo ./build/example_soem_demo eth0 spd 0 2 0 11 2 1.0 2.0 1
```

也可以给 demo 二进制授予 raw socket 权限：

```bash
sudo setcap cap_net_raw,cap_net_admin+ep ./build/example_soem_demo
./build/example_soem_demo eth0 spd 0 2 0 11 2 1.0 2.0 1
```

## 编号规则

所有对外输入编号都从 0 开始：

- `slaveId`：SOEM 从站顺序减一
- `busId`：网关内部总线编号
- `slot`：该总线内的 PDO 电机槽位编号

日志里会额外打印 SOEM 原始从站号，即 `slaveId + 1`。

## 支持的 PDO 网关格式

EtherCAT 层根据从站输出 PDO 字节数识别网关类型：

- `86` 字节：classic CAN，2 路 bus，每路 3 个槽位
- `336` 字节：CAN FD，3 路 bus，每路 8 个槽位
- `896` 字节：CAN FD，8 路 bus，每路 8 个槽位

## Demo 用法

```text
example_soem_demo <ifname> <op> <slaveId> <busId> <slot> <motorId> <model> [--eff] [--canfd] [--flag <value>] [args...]
```

默认发送 classic CAN 标准帧，即 `eff=0 canfd=0`，对应 `flag=0x00`。如果电机信息里
`eff=1`，在 `model` 后添加 `--eff`；如果 `canfd=1`，添加 `--canfd`。也可以用
`--flag 0x06` 直接指定底层 `MotorConfig.flag`。

示例：

```bash
sudo ./build/example_soem_demo eth0 spd 0 2 0 11 2 1.0 2.0 1
sudo ./build/example_soem_demo eth0 pos 0 2 0 11 2 0.5 2.0 3.0 1
sudo ./build/example_soem_demo eth0 get-param 0 2 0 11 2 5
sudo ./build/example_soem_demo eth0 spd 0 2 0 11 2 --canfd 1.0 2.0 1
```

demo 在发送选定报文后，会以 1 ms 周期运行 200 次收发循环。正式应用应替换为自己的实时循环。

`model` 是 `include/example_soem/motor_layer.h` 中 `MotorModel` 枚举的数值。

## 检查

代码变更后运行：

```bash
bash scripts/run_clang_format.sh --check
bash scripts/run_clang_tidy.sh build
```

## SocketCAN 八电机反馈模拟器

`socketcan_8motor_simulator` 用于测试 `qt_motor_control_ui` 的 8 电机反馈表。
它监听 Motor ID `0x001` 到 `0x008`，接收奇数 ID 的速度指令和偶数 ID 的位置指令，
并立即发送类型 1 反馈。终端每秒打印一次收发帧、位置、速度、电流和温度。

```bash
cmake --build build --target socketcan_8motor_simulator qt_motor_control_ui
./build/socketcan_8motor_simulator vcan0
```

然后在另一个终端启动 UI，选择 `SocketCAN` 和 `vcan0`：

```bash
./build/qt_motor_control_ui
```

模拟器默认使用 `±10 A` 电流反馈范围，与 UI 默认的 `EC_A2806_P2` 型号一致。其他型号可用：

```bash
./build/socketcan_8motor_simulator vcan0 --current-range 30 --print-ms 1000
```

Qt 界面方向键中央的 **Calibration** 按钮会先暂停周期控制报文，然后发送参数查询：

- CAN `1/3/5/7`：`E1 02`，读取当前速度（RPM）
- CAN `2/4/6/8`：`E1 01`，读取当前位置（度）

界面解析类型 5 应答 `A0 <查询代码> <float32 big-endian>`，将结果写入对应的四个速度和
四个位置输入框，再重新生成八个控制报文。`socketcan_8motor_simulator` 同样支持这些查询，
可用于验证校准流程。
