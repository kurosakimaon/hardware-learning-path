# 硬件工程师修炼路线图

从点灯到交钥匙的七关硬件课程。每一关都按“先算数字、再写模型、最后上板”的顺序组织：C 程序把公式跑成可复现的输出，HTML 实验台把波形和时序画出来，`tutorial.md` 负责讲清楚“为什么”和“哪里会翻车”。

全套资料离线可用，不依赖第三方库、不联网、不需要开发板。

## 学习顺序

| 关卡 | 模块 | 主题 | 核心产出 |
|---|---|---|---|
| 第 1 关 | [`circuit_basics/`](circuit_basics/) | 筑基期 · 电路基础 | 节点电压法求解器、RC 暂态、分压器负载效应 |
| 第 2 关 | [`analog_electronics/`](analog_electronics/) | 炼气期 · 模拟电路 | 整流/滤波、运放非理想性、LDO 与 Buck 账本 |
| 第 3 关 | [`digital_electronics/`](digital_electronics/) | 结丹期 · 数字电路 | 逻辑门、时序/亚稳态、建立保持时间、FSM |
| 第 4 关 | [`mcu_embedded/`](mcu_embedded/) | 兵器期 · MCU 与嵌入式 | GPIO 寄存器、UART 波特率、I²C 总线、中断与看门狗 |
| 第 5 关 | [`pcb_design/`](pcb_design/) | 阵法期 · PCB 与 EMC | 线宽/过孔、阻抗与串扰、去耦与回流路径 |
| 第 6 关 | [`instruments_debug/`](instruments_debug/) | 望闻问切 · 仪器与排障 | 示波器链路、测量不确定度、故障树二分法 |
| 第 7 关 | [`miniboard/`](miniboard/) | 手搓期 · MiniBoard | 电源预算、BOM/DRC、固件骨架、I²C 协议、上电剧本、整机验收 |

总入口：[`硬件学习路线图.html`](硬件学习路线图.html)（双击打开）。

## 环境要求

- Windows 10/11
- [MSYS2](https://www.msys2.org/) 的 **UCRT64** 环境，GCC 16.2（C17）
- 任意现代浏览器（Chrome / Edge / Firefox）

首次打开控制台先执行：

```powershell
chcp 65001
```

否则中文输出会乱码。`analog_electronics/` 下的三个程序会自行调用 `SetConsoleOutputCP(65001)` 兜底。

## 快速开始

### 编译并运行单个示例

```bash
cd circuit_basics
gcc -std=c17 -O2 -Wall -Wextra 01_node_solver.c -o 01_node_solver.exe
./01_node_solver.exe
```

### 一键编译全部 24 个 C 示例

MSYS2 / Git Bash：

```bash
./build_all.sh
```

Windows CMD：

```bat
build_all.bat
```

### 一键验证（编译 + 运行 + 检查退出码）

```bash
./verify_all.sh
```

脚本会：

1. 用 `-std=c17 -O2 -Wall -Wextra -Werror` 编译全部 C 文件；
2. 逐个运行，日志写入 `build/logs/`；
3. 任一程序崩溃或返回非零退出码即报错。

`build/`、`.review_out/`、`.review_build/`、`_ref_tmp/` 都是本地产物，不进入版本库。

## 目录说明

每个关卡目录都包含：

- `tutorial.md` —— 主课本：概念、公式、常见坑、家庭作业、通关标准；
- `*.c` —— 可在 PC 上直接运行的计算/仿真程序；
- `*_animator.html` / `*_lab.html` / `*_playground.html` —— 离线交互实验台；
- `index.html` —— 本关导航页。

全项目共 **24 个 C 程序、24 个 HTML 页面、8 份课程 Markdown**（另有根目录的 `README.md` 与 `ERRATA.md`）。

## 关键验收数字

跑完第 7 关，应该能亲手复现这些数字（不是背下来）：

| 项目 | 参考值 | 出处 |
|---|---|---|
| 省电模式静态电流 | 3V3 侧 1.68 mA / USB 5V 侧 1.72 mA | `phase1_power_budget` |
| 报警模式最坏 LDO 温升 | 17.5 °C（SOT-89，θJA≈250 °C/W） | `phase1_power_budget` |
| BOM 总价 | ¥23.52（修完 3 个 FAIL 后 ¥23.84） | `phase2_bom_drc` |
| 上电 → OLED 首帧 | 80 ms（规格 ≤200 ms） | `phase3_firmware_core` |
| 4.7 kΩ + 150 pF 的 I²C 上升时间 | 597 ns（100 kHz 限 1000 ns） | `phase4_protocol_i2c` |
| 24 小时压缩验收 | 10000 tick、6/6 PASS、串口整帧通过率 99.60% | `phase6_bench` |

## 校订说明

本仓库在整理过程中对若干公式、数值、跨系列寄存器描述和实验现象做了勘误，详见 [`ERRATA.md`](ERRATA.md)。如果你在旧笔记里抄到过“UART 3% 漂移读成 0xD5”“F103 有 MODER”“100 W 白炽灯串 5 V 板”之类的说法，以本仓库当前版本为准。

## 使用许可

本项目用于教学与学习交流，未附开源许可证。如需转载、改造或用于商业培训，请先联系作者。
