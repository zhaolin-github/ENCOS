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
