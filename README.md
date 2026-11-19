# STM32G474 无感 FOC —— 线性观测器（龙伯格）方案

**核心特点：用线性观测器（Luenberger）替代滑模观测器（SMO），低速段性能显著更好。**

> **TL;DR** — Hand-written **sensorless FOC** on **STM32G474 + DRV8340S**.
> Uses a **linear (Luenberger) observer** instead of a sliding-mode observer.
> Cost: a few dozen lines of float math — no lookup tables, no CORDIC, no vendor library.
>
> - Holds closed-loop down to **~1 rps (60 rpm)** under *ideal conditions*
>   (no load, accurate R/L); carries a hand-applied load up to **~40 rps (2400 rpm)**
> - Angle error vs. an AS5600 magnetic encoder: **±3° @ 1 rps**
>   (the same motor, same gains, with an SMO: **±105°** — unusable below ~2 rps)
> - 21.25 kHz current loop on a 170 MHz Cortex-M4, floats only
> - All PI gains derived analytically (pole-zero cancellation + phase margin), not trial-and-error
> - ⚠️ **Research / hobby project, not production-ready.** No parameter identification
>   (R and L are measured by hand), speed loop still tuned manually, validated on a
>   single motor. See [已知限制与不足](#已知限制与不足).
>
> 中文一句话：用线性观测器把无感 FOC 的闭环下限从约 5 rps 拉到 **1 rps**，
> 高速段带载到 **40 rps**；参数不是试出来的，是按零极点对消 + 相位裕度算出来的。

## 实测效果
<img width="1248" height="575" alt="image" src="https://github.com/user-attachments/assets/00bafacf-1d2d-41e2-98ee-263120f8bb99" />

| 项目 | 结果 |
|---|---|
| 最低无感闭环转速 | **约 1 rps（60 rpm）**，空载、理想条件下 |
| 最高带载转速 | **约 40 rps（2400 rpm）**，手动加阻测试，非持续负载 |
| 电流环频率 | 21.25 kHz |
| 速度环频率 | 664 Hz（32 分频） |
| ISR 耗时 | 18.4 µs / 47 µs 周期（含 21 kHz 全速遥测）|
| 电流环跟踪误差 | ≤ 2 mA（给定 vs 反馈）|
| 三相重构和 | \|Ia+Ib+Ic\| < 0.1 mA |

> **测试条件**：单台电机（Rs=2.55 Ω / Ls=0.9 mH / 7 对极）、母线 12 V、室温、空载或手动摩擦加载。
> 这些数字说明"能跑成什么样"，不代表换一台电机、换一套硬件也能复现同样的下限。

## 为什么用线性观测器

同一套硬件、同一个电机、同样的 PI 参数下，对比两种观测器的**角度误差**
（用手持磁编码器 AS5600 作真值）：

| 转速 | 龙伯格角度误差 | SMO 角度误差 |
|---|---|---|
| 1 rps | **±3°** | **±105°**（已失效，闭环会堵转）|
| 2 rps | ±2° | ±18° |
| 5 rps | ±2° | ±4° |
| 10 rps | ±3° | ±1.5° |

> 这是**同一台电机、同一套参数**下的对比。SMO 的失效点由 `Smo_K`、低通截止频率和采样率共同决定，
> 换电机、换采样率后这个下限会变——不能据此得出"SMO 不行"的普遍结论，
> 只能说它在这台电机 + 这个采样率下不适合低速。

原因：SMO 的滑模输出是 ±Smo_K 的方波，靠低通滤波提取等效控制量。
滑模纹波经低通后的残留约为 `Smo_K × fc / fs`，
在 1 rps 时反电势幅值（约 0.22 V）已经和残留纹波同量级，
角度估计自然失效。

线性观测器没有 sign 函数，不存在这个噪声源——代价是对 R、L 参数更敏感。

## 硬件

| 部分 | 型号 / 参数 |
|---|---|
| MCU | STM32G474RBT3（170 MHz, CORDIC/FMAC, HRTIM）|
| 栅极驱动 | DRV8340S（SPI 配置，内部死区）|
| 电流采样 | 3× 低边 0.1 Ω + 差分放大 ×10 |
| 母线电压采样 | 4:1 分压 |
| 编码器（调试用）| AS5600，I2C DMA，12 bit |
| 电机 | 相电阻 2.55 Ω，相电感 0.9 mH，7 对极 |
| 母线 | 12 V |

## 代码结构

```
Core/Src/mc_ctrl.c   FOC 主流程、初始化、PID、SVPWM 输出、USB 命令
Core/Src/mc_math.c   Clark/Park、SVPWM、龙伯格观测器、SMO、PLL
Core/Src/pid.c       PID 控制器
Core/Src/jf.c        JustFloat 格式遥测（VOFA+ 直接可看）
Core/Src/drv8340s.c  DRV8340 SPI 配置与故障处理
```

## 编译与烧录

需要 CMake + Ninja + arm-none-eabi 工具链（或 ST 的 clang 工具链）。

```bash
cmake -S . -B build/Release -G Ninja \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake
cmake --build build/Release

STM32_Programmer_CLI -c port=SWD mode=UR -w build/Release/project.elf -v -rst
```

## 使用

USB CDC 虚拟串口，**直接发一个数字就是目标转速（rps）**：

```
20.0     -> 目标 20 rps
0        -> 停机
```

遥测用 JustFloat 协议（6 通道 float + 尾帧 `00 00 80 7F`），
可直接用 [VOFA+](https://www.vofa.plus/) 的 JustFloat 控件显示。

| 通道 | 内容 |
|---|---|
| 1~3 | 三相电流 Ia / Ib / Ic (A) |
| 4 | 转速 (rps) |
| 5 | 观测器角度 (rad) |
| 6 | 备用（编码器角度 / 给定值，视版本）|

## 调参方法

本项目的 PI 参数不是试出来的，按下面三步算出来：

1. **零极点对消定比值**：`Ki/Kp = R/L`（实际用了 1.5×R/L，多一点相位超前）
2. **穿越频率定增益**：`Kp = ωc·L`，`Ki = ωc·R`
3. **ωc 由相位裕度反推**：`ωc = π/(6·Td)` 对应 PM=60°，其中 `Td` 是
   "采样到更新占空比"的延时（约 1.5 个控制周期，含零保持）

详见 [docs/调参方法.md](docs/调参方法.md)。

## 已知限制与不足

坦率地说，这个项目的定位是**学习 / 验证**，不是能直接拿去做产品的代码。目前至少还缺：

**参数辨识**
- `PHASE_R` / `PHASE_L` 是万用表或电桥手测后填进去的常量，**没有在线辨识**
- 换电机要重新测量、重新算一遍 PI 参数
- 电阻随温度漂移（铜约 +0.4 %/℃）没有补偿，大电流或长时间运行后观测器精度会下降
- 没有对电感饱和建模

**速度环仍是手调的**
- 电流环和 PLL 的参数有明确解析依据（见 [docs/调参方法.md](docs/调参方法.md)）
- 但速度环只有"比 PLL 慢 5 倍"这个经验起点，最终还是要看阶跃响应手动调
- 没有自整定，也没有摩擦 / 齿槽转矩补偿

**验证范围有限**
- 只在**一台电机、一套硬件**上验证过
- 没有做温升、长时间连续运行的可靠性测试
- 带载能力是"用手捏"测出来的，没有标准负载台架

**其他**
- 低速下限依赖负载和惯量：空载约 1 rps，带载时明显升高；1 rps 以下反电势过小会丢锁
- 无感不能从零速启动，必须先用开环把电机拖起来再切闭环
- 死区补偿在本电机（2.55 Ω）上收益小于风险，代码里是**关闭**的
- 电压输出必须留余量（`Valid_Ratio`），否则下管采样窗口消失、电流采样失效
- ISR 耗时 18.4 µs（`-Oz` 实测；`-O2` 后未复测），`sin/cos`、`fmaxf`、`extern const` 还有优化空间
- 编码器只用于调试和验证，不是产品功能
- 没有实现弱磁；SPMSM 下 Id=0 已是最优，但换成 IPMSM 就缺 MTPA 这块

## 致谢

- STM32 HAL / CubeMX 生成的框架代码（BSD-3-Clause，见各文件头）
- TI DRV8340 数据手册

## License

MIT，见 [LICENSE](LICENSE)。第三方组件（`Drivers/`、`Middlewares/`、`USB_Device/`）
保留 ST 的 BSD-3-Clause 许可。
