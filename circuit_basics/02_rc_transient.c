/* =====================================================================
 * 02_rc_transient.c —— RC 暂态：一个指数函数怎么就让你的板子不工作了
 *
 * 【这个文件讲什么】
 *   直流稳态里电容是开路（01_node_solver.c 已经用数字证明过了），
 *   所以真正要算的只有一件事：电容上的电压【不能跳变】，它以
 *       v(t) = V_inf + (V_0 - V_inf) * e^(-t/τ)，τ = R * C
 *   的速度向目标值爬。这个文件做四件事：
 *     1) 用前向欧拉法【数值积分】一遍，和解析解逐点对比，让你亲眼看见
 *        数值方法的误差跟 dt 成正比，以及 dt 取大了为什么会发散；
 *     2) 用 ASCII 字符画出充电曲线，把 τ / 3τ / 5τ 的位置钉在图上；
 *     3) 扫描真实会用的 R、C 组合，打印过 1/3·VCC、过 2/3·VCC 的时刻、
 *        10%~90% 上升时间 tr 和 -3dB 截止频率 fc；
 *     4) 算上电冲击电流峰值，以及【长走线 + 100nF】为什么不是去耦而是谐振。
 *   全部是纯 C（C17），数值积分、画图、单位换算都是手写的。
 *
 * 【怎么编译运行】（MSYS2 / MinGW-w64，gcc 16.2 实测零警告）
 *   gcc -std=c17 -O2 -Wall -Wextra 02_rc_transient.c -o 02_rc_transient.exe
 *   chcp 65001            （先把控制台切到 UTF-8，否则中文和 τ 会乱码）
 *   ./02_rc_transient.exe
 *
 * 【预期看到什么】
 *   - 欧拉法：dt = τ/10 时最大误差约 1.9%·VCC；dt 缩小 10 倍，误差也缩小
 *     10 倍（一阶方法的签名）；dt = 2.5τ 时放大因子 |1 - dt/τ| = 1.5 > 1，
 *     解振荡着发散（隐式欧拉同一 dt 照样收敛，只是变钝）。
 *   - 上升时间公式全部对得上：t(1/3) = 0.405τ、t(2/3) = 1.099τ、
 *     tr(10%~90%) = 2.197τ、5τ = 99.3%。
 *   - 10k + 100nF：τ = 1.000 ms，5τ = 5.00 ms，盖住典型的毫秒级抖动；
 *     更长的抖动（5~10ms 的烂开关）要靠软件确认。
 *   - 上电冲击：100µF 电解并 0.5Ω 回路电阻，I0 = 3.333A，τ = 150µs；
 *     回路电阻趋近 0 时 I0 趋近无穷——这就是缓启动电路存在的全部理由。
 *   - 30cm 走线（300nH）+ 100nF：谐振 918.9kHz，Q ≈ 8.7，上电振铃冲过 2 倍电压。
 *
 * 【老师傅点拨】
 *   第一，τ = RC 不是【电容充满电要多久】，是【还差 36.8%】。硬件工程师嘴里
 *   的【稳定了】通常是 5τ，也就是 99.3%——差的那 0.7% 在 ADC 前级就是几毫伏，
 *   在数字线上什么都不是。什么时候按 3τ 算、什么时候按 5τ 算，取决于后面接谁。
 *   第二，永远先问一句：这个回路的总电阻 R 到底是多少？答案是【走线电阻 +
 *   驱动级内阻 + 电容 ESR】，不是你原理图上标的那一颗。上电那 100ns 里，
 *   电容看过去就是一片短路，你的 3.3V LDO 是被【短路】打的。
 * =====================================================================
 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define BUFSZ 512
#define PLOT_ROWS 20
#define PLOT_COLS 128

/* =====================================================================
 * 一、打印与单位换算
 *     工程师不用裸的 0.000001，用 1u。单位换对错，数量级错误当场现形。
 * ===================================================================== */

/* 终端显示宽度：ASCII 占 1 列，UTF-8 多字节字符（中日韩）占 2 列 */
static size_t disp_width(const char *s) {
    size_t w = 0;
    for (const char *p = s; *p; ++p) {
        const unsigned char c = (unsigned char)*p;
        if (c < 0x80) {
            w += 1;
        } else if ((c & 0xC0) != 0x80) {
            w += 2;  /* 多字节字符的首字节，续字节属于同一个字符，跳过 */
        }
    }
    return w;
}

/* 打印 s 并用空格补到显示宽度 width */
static void pad(const char *s, size_t width) {
    fputs(s, stdout);
    const size_t w = disp_width(s);
    if (w < width) printf("%*s", (int)(width - w), "");
}

static void rule(const char *title) {
    printf("\n============================================================\n  %s\n"
           "============================================================\n", title);
}

static void fmt_v(char *out, double v) {
    snprintf(out, BUFSZ, "%.3f V", v);
}

static void fmt_t(char *out, double sec) {
    const double a = fabs(sec);
    if (a >= 1.0) {
        snprintf(out, BUFSZ, "%.3f s", sec);
    } else if (a >= 1e-3) {
        snprintf(out, BUFSZ, "%.3f ms", sec * 1e3);
    } else if (a >= 1e-6) {
        snprintf(out, BUFSZ, "%.3f us", sec * 1e6);
    } else if (a >= 1e-9) {
        snprintf(out, BUFSZ, "%.3f ns", sec * 1e9);
    } else {
        snprintf(out, BUFSZ, "%.3f ps", sec * 1e12);
    }
}

static void fmt_r(char *out, double ohm) {
    if (ohm >= 1e6) {
        snprintf(out, BUFSZ, "%.3f MOhm", ohm / 1e6);
    } else if (ohm >= 1e3) {
        snprintf(out, BUFSZ, "%.3f kOhm", ohm / 1e3);
    } else {
        snprintf(out, BUFSZ, "%.3f Ohm", ohm);
    }
}

static void fmt_c(char *out, double farad) {
    if (farad >= 1e-3) {
        snprintf(out, BUFSZ, "%.2f mF", farad * 1e3);
    } else if (farad >= 1e-6) {
        snprintf(out, BUFSZ, "%.2f uF", farad * 1e6);
    } else if (farad >= 1e-9) {
        snprintf(out, BUFSZ, "%.2f nF", farad * 1e9);
    } else {
        snprintf(out, BUFSZ, "%.2f pF", farad * 1e12);
    }
}

static void fmt_hz(char *out, double hz) {
    if (hz >= 1e6) {
        snprintf(out, BUFSZ, "%.1f MHz", hz / 1e6);
    } else if (hz >= 1e3) {
        snprintf(out, BUFSZ, "%.1f kHz", hz / 1e3);
    } else {
        snprintf(out, BUFSZ, "%.1f Hz", hz);
    }
}

static void fmt_i(char *out, double amp) {
    const double a = fabs(amp);
    if (a >= 1.0) {
        snprintf(out, BUFSZ, "%.3f A", amp);
    } else if (a >= 1e-3) {
        snprintf(out, BUFSZ, "%.3f mA", amp * 1e3);
    } else {
        snprintf(out, BUFSZ, "%.3f uA", amp * 1e6);
    }
}

/* =====================================================================
 * 二、解析解
 *     一阶 RC 的解只有这一个式子，闭着眼睛也要能默写：
 *         v(t) = V_inf + (V_0 - V_inf) * e^(-t/τ)
 *     充电：V_0 = 0，V_inf = VCC  →  v = VCC(1 - e^(-t/τ))
 *     放电：V_0 = VCC，V_inf = 0  →  v = VCC·e^(-t/τ)
 *     电流：i(t) = C·dv/dt = (VCC - V_0)/R · e^(-t/τ)  ← 指数衰减，t=0 最大
 * ===================================================================== */
/* 不用 M_PI：-std=c17 严格模式下它未必可见，自己定义最保险 */
#define PI 3.14159265358979323846

static double analytic(double t, double v0, double v_inf, double tau) {
    return v_inf + (v0 - v_inf) * exp(-t / tau);
}

/* 从 v0 单调走向 v_inf，到达绝对值阈值 v_th 的时刻。
 * 解 v0 + (v_inf - v0)(1 - e^(-t/τ)) = v_th  →  t = -τ·ln(1 - (v_th - v0)/(v_inf - v0)) */
static double time_to(double v0, double v_inf, double tau, double v_th) {
    const double frac = (v_th - v0) / (v_inf - v0);
    if (frac <= 0.0) return 0.0;
    if (frac >= 1.0) return INFINITY;
    return -tau * log(1.0 - frac);
}

/* =====================================================================
 * 三、ASCII 波形绘制
 *     没有 matplotlib 也能看清形状。画图的目的是把 τ 这个抽象时间常数
 *     【变成眼睛能看到的东西】——63.2% 那条线在哪，5τ 到了多高。
 * ===================================================================== */
static void ascii_plot(const double *ys, size_t ny, double t_max, double y_max,
                       double grid_step, const char *grid_label, int rows, int cols) {
    static char grid[PLOT_ROWS][PLOT_COLS + 1];

    for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < cols; ++c) grid[r][c] = ' ';
        grid[r][cols] = '\0';
    }

    /* 先画网格竖线（默认每 1τ 一条） */
    const int n_grid = (int)(t_max / grid_step);
    for (int k = 1; k <= n_grid; ++k) {
        const int col = (int)llround(grid_step * k / t_max * (cols - 1));
        if (col < 0 || col >= cols) continue;
        for (int r = 0; r < rows; ++r) {
            if (grid[r][col] == ' ') {
                grid[r][col] = ':';
            }
        }
    }

    /* 再描曲线（最近邻采样，允许 ys 比 cols 密得多） */
    for (int c = 0; c < cols; ++c) {
        const size_t idx = (size_t)((double)c / (cols - 1) * (double)(ny - 1));
        double y = ys[idx];
        if (y < 0.0) y = 0.0;
        if (y > y_max) y = y_max;
        int r = rows - 1 - (int)llround(y / y_max * (rows - 1));
        if (r < 0) r = 0;
        if (r >= rows) r = rows - 1;
        grid[r][c] = '#';
    }

    char buf[BUFSZ];
    for (int r = 0; r < rows; ++r) {
        const double yv = y_max * (rows - 1 - r) / (rows - 1);
        printf("  ");
        fmt_v(buf, yv);
        pad(buf, 10);
        printf(" |%s\n", grid[r]);
    }

    printf("  ");
    for (int c = 0; c < 12 + cols; ++c) putchar('-');
    printf("\n");
    char ticks[PLOT_COLS + 1];
    for (int c = 0; c < cols; ++c) ticks[c] = ' ';
    ticks[cols] = '\0';
    for (int k = 0; k <= n_grid; ++k) {
        const int col = (int)llround(grid_step * k / t_max * (cols - 1));
        if (col < 0 || col >= cols) continue;
        ticks[col] = '|';
    }
    printf("              %s\n", ticks);
    fmt_t(buf, t_max);
    printf("              t = 0"
           "                                    横轴 0 ~ %s，每条竖线 : = %s\n",
           buf, grid_label);
}

/* =====================================================================
 * 四、演示 ①：前向欧拉 vs 解析解 —— 数值仿真的第一课
 *     方程：C·dv/dt = (VCC - v)/R   →   dv/dt = (VCC - v)/τ
 *     前向欧拉：v[k+1] = v[k] + dt·(VCC - v[k])/τ
 *     把它写成 v[k+1] = a·v[k] + (1-a)·VCC，a = 1 - dt/τ。
 *     稳定条件 |a| < 1  →  dt < 2τ。dt 取大了不是【不准】，是【炸掉】。
 * ===================================================================== */
static void demo_euler(void) {
    rule("演示 1 欧拉法 vs 解析解：数值仿真什么时候可信");

    const double vcc = 3.3;
    const double tau = 1e-3;  /* 10k + 100nF，等会儿的消抖电路 */
    const double t_end = 5.0 * tau;

    char buf[BUFSZ];
    fmt_v(buf, vcc);
    printf("  电路：VCC = %s 经 R 给 C 充电，τ = RC = ", buf);
    fmt_t(buf, tau);
    printf("%s，仿真到 5τ = ", buf);
    fmt_t(buf, t_end);
    printf("%s\n\n", buf);
    printf("  ");
    pad("dt", 13);
    pad("步数", 8);
    pad("最大误差", 14);
    pad("占 VCC", 11);
    pad("5τ 处误差", 13);
    printf("判定\n");
    printf("  --------------------------------------------------------------------------------------\n");

    const double exact_end = analytic(t_end, 0.0, vcc, tau);

    const double ratios[] = {1.0 / 10.0, 1.0 / 100.0, 1.0 / 1000.0, 1.0 / 10000.0, 2.5};
    const size_t n_ratio = sizeof ratios / sizeof ratios[0];
    for (size_t j = 0; j < n_ratio; ++j) {
        const double ratio = ratios[j];
        const double dt = ratio * tau;
        const int steps = (int)(t_end / dt);
        double v = 0.0;
        double max_err = 0.0;
        int blew = 0;
        for (int k = 0; k < steps; ++k) {
            v += dt * (vcc - v) / tau;  /* 就这一行，就是整个数值积分 */
            if (!isfinite(v)) {
                blew = 1;
                break;
            }
            const double err = fabs(v - analytic((k + 1) * dt, 0.0, vcc, tau));
            if (err > max_err) max_err = err;
        }

        const char *verdict;
        if (blew) {
            verdict = "已溢出：解在振荡中发散，仿真器当场报错";
        } else if (ratio > 2.0) {
            verdict = "发散中：放大因子 |1 - dt/τ| = 1.50 > 1，只是 2 步还没溢出";
        } else if (ratio >= 0.5) {
            verdict = "勉强收敛，但误差已经没法看";
        } else if (ratio >= 0.05) {
            verdict = "可用，但要清楚它偏高（欧拉法总是超前）";
        } else {
            verdict = "足够精确，误差已在设计余量之下";
        }

        char err_str[BUFSZ];
        char pct_str[BUFSZ];
        char end_str[BUFSZ];
        if (blew) {
            snprintf(err_str, BUFSZ, "inf");
            snprintf(pct_str, BUFSZ, "—");
            snprintf(end_str, BUFSZ, "inf");
        } else {
            fmt_v(err_str, max_err);
            snprintf(pct_str, BUFSZ, "%.3f %%", max_err / vcc * 100.0);
            fmt_v(end_str, fabs(v - exact_end));
        }

        printf("  ");
        fmt_t(buf, dt);
        pad(buf, 13);
        snprintf(buf, BUFSZ, "%d", steps);
        pad(buf, 8);
        pad(err_str, 14);
        pad(pct_str, 11);
        pad(end_str, 13);
        printf("%s\n", verdict);
    }

    printf("\n  师傅批注：\n");
    printf("    看【dt / 最大误差】这一列：dt 每缩小 10 倍，误差也缩小 10 倍。\n");
    printf("    这就是【一阶方法】的指纹——误差正比于 dt。你以后看 SPICE 的\n");
    printf("    .tran  step 参数、看 trtol 容差，本质上就是在管这张表。\n");
    printf("    dt = 2.5τ 那行：a = 1 - 2.5 = -1.5，每步乘 -1.5，\n");
    printf("    解变成振荡着发散。仿真器报【时间步长过大、不收敛】就是这个。\n");
    printf("    工程经验：想算清一个 τ，至少给 50~100 步；\n");
    printf("    想算清整个 5τ 过程，全程别少于 500 步。\n");

    /* 反过来问：如果不用解析解，我只知道【导数】呢？—— 隐式欧拉永远稳定 */
    printf("\n  [附] 同一道题用【后向（隐式）欧拉】：v[k+1] = (v[k] + dt/τ·VCC)/(1 + dt/τ)\n");
    printf("  ");
    pad("dt", 13);
    pad("最大误差", 14);
    printf("说明\n");
    for (size_t j = 0; j < n_ratio; ++j) {
        const double ratio = ratios[j];
        const double dt = ratio * tau;
        const int steps = (int)(t_end / dt);
        double v = 0.0, max_err = 0.0;
        for (int k = 0; k < steps; ++k) {
            v = (v + dt * vcc / tau) / (1.0 + dt / tau);
            const double err = fabs(v - analytic((k + 1) * dt, 0.0, vcc, tau));
            if (err > max_err) max_err = err;
        }
        printf("  ");
        fmt_t(buf, dt);
        pad(buf, 13);
        fmt_v(buf, max_err);
        pad(buf, 14);
        printf("%s\n", ratio > 1.0 ? "照样收敛：隐式方法无条件稳定，代价是抹平了细节"
                                   : "与显式同阶（误差正比 dt），但方向偏慢");
    }
}

/* =====================================================================
 * 五、演示 ②：把 τ 画出来 —— 63.2% / 95% / 99.3% 到底长什么样
 * ===================================================================== */
static void demo_waveform(void) {
    rule("演示 2 ASCII 波形：τ、3τ、5τ 在图上的位置");

    const double vcc = 3.3;
    const double tau = 1e-3;
    const double t_end = 5.0 * tau;
    const int n = 2001;

    static double charge[2001];
    static double discharge[2001];
    for (int k = 0; k < n; ++k) {
        const double t = t_end * k / (n - 1);
        charge[k] = analytic(t, 0.0, vcc, tau);
        discharge[k] = analytic(t, vcc, 0.0, tau);
    }

    char buf[BUFSZ];
    fmt_v(buf, vcc);
    printf("  充电（v0 = 0，走向 %s），τ = ", buf);
    fmt_t(buf, tau);
    printf("%s：\n", buf);
    ascii_plot(charge, (size_t)n, t_end, vcc, tau, "τ", 11, 61);

    fmt_v(buf, vcc);
    printf("\n  放电（v0 = %s，走向 0）——同一指数，翻个面：\n", buf);
    ascii_plot(discharge, (size_t)n, t_end, vcc, tau, "τ", 11, 61);

    printf("\n  关键刻度（这一小段背下来，以后靠它估）：\n");
    printf("    ");
    pad("时刻", 10);
    pad("到达", 12);
    printf("含义\n");
    const double marks[] = {1.0, 2.0, 3.0, 5.0};
    const char *notes[] = {
        "走完 63.2%。此时电流只剩 36.8%，还剩 1/3 的路没走",
        "走完 86.5%。数字电路通常认为【快到了】，模拟电路认为【还早】",
        "走完 95.0%。12 位 ADC 采样保持：95% 还不够，要 0.5LSB",
        "走完 99.3%。工程上的【稳定了】，也是消抖/延时最常用的窗口",
    };
    for (int k = 0; k < 4; ++k) {
        const double frac = 1.0 - exp(-marks[k]);
        char pct[BUFSZ];
        snprintf(pct, BUFSZ, "%.1f %%", frac * 100.0);
        printf("    ");
        snprintf(buf, BUFSZ, "%dτ", (int)marks[k]);
        pad(buf, 10);
        pad(pct, 12);
        printf("%s\n", notes[k]);
    }
    printf("\n  注意充电曲线的起点是【陡】的：t=0 那一刻 dv/dt 最大，\n");
    fmt_i(buf, vcc / 10000.0);
    printf("  i(0) = VCC/R = %s（10k 限流）。电容在 t=0 就是一片短路，\n", buf);
    printf("  你的电源在那一刻看到的负载是【最小】的——上电打嗝全在这。\n");
}

/* =====================================================================
 * 六、演示 ③：真实 R、C 组合扫描
 *     几个必须刻进肌肉的系数（都从 -τ·ln(1-x) 来）：
 *       过 1/3·VCC : 0.405τ     （施密特/555 下阈值）
 *       过 2/3·VCC : 1.099τ     （施密特/555 上阈值）
 *       10%%~90%% 上升时间 tr = ln(9)·τ = 2.197τ
 *       -3dB 截止频率 fc = 1/(2πτ)
 * ===================================================================== */
static void demo_sweep(void) {
    rule("演示 3 R、C 组合扫描：τ / 阈值时刻 / 上升时间 / 截止频率");

    printf("  通用系数：t(1/3·VCC) = 0.405τ，t(2/3·VCC) = 1.099τ，"
           "tr(10~90) = 2.197τ，fc = 1/(2πτ)\n\n");
    printf("  ");
    pad("R", 14);
    pad("C", 10);
    pad("τ", 12);
    pad("过1/3VCC", 12);
    pad("过2/3VCC", 12);
    pad("tr(10~90)", 12);
    pad("fc", 11);
    printf("这是干什么用的\n");
    printf("  --------------------------------------------------------------------------------------------------------\n");

    struct Case {
        double r;
        double c;
        const char *use;
    };
    const struct Case cases[] = {
        {10e3, 100e-9, "按键消抖（τ=1ms，吃短抖动，长抖动靠软件）"},
        {10e3, 1e-9, "复位脚小幅延时 / 高频滤波（τ=10us）"},
        {1e3, 100e-9, "PWM 转 DAC 的快档（τ=100us，纹波大）"},
        {100e3, 100e-9, "慢速上电指示延时（τ=10ms）"},
        {4.7e3, 10e-6, "几十毫秒级的电源就绪延时（τ=47ms）"},
        {10e3, 22e-12, "走线寄生：10k 驱动 + 22pF（τ=220ns，别忽略）"},
        {1e6, 100e-12, "高阻节点被 100pF 拖慢（τ=100us，运放输入常见）"},
        {47e3, 680e-9, "8 位精度的 PWM-DAC（τ=31.96ms）"},
    };

    char buf[BUFSZ];
    for (size_t k = 0; k < sizeof cases / sizeof cases[0]; ++k) {
        const double tau = cases[k].r * cases[k].c;
        printf("  ");
        fmt_r(buf, cases[k].r);
        pad(buf, 14);
        fmt_c(buf, cases[k].c);
        pad(buf, 10);
        fmt_t(buf, tau);
        pad(buf, 12);
        fmt_t(buf, time_to(0.0, 1.0, tau, 1.0 / 3.0));
        pad(buf, 12);
        fmt_t(buf, time_to(0.0, 1.0, tau, 2.0 / 3.0));
        pad(buf, 12);
        fmt_t(buf, log(9.0) * tau);
        pad(buf, 12);
        fmt_hz(buf, 1.0 / (2.0 * PI * tau));
        pad(buf, 11);
        printf("%s\n", cases[k].use);
    }

    printf("\n  师傅批注：\n");
    printf("    最后一行 47k + 680nF：τ = 31.96 ms，5τ = 160 ms。\n");
    printf("    也就是说你的【DAC】要 160ms 才走到位——拿它做音频、\n");
    printf("    做闭环控制，等于自杀。PWM 转 DAC 只配做缓慢的参考电压。\n");
    printf("    想要快又想要平，这两个要求在 RC 上是天然矛盾的：\n");
    printf("    带宽 = 1/(2πτ)，精度 ∝ τ。要同时拿，只能上运放或者上真 DAC。\n");
}

/* =====================================================================
 * 七、演示 ④：三个真实场景的落点计算
 * ===================================================================== */
static void demo_applications(void) {
    rule("演示 4 场景落点：消抖为什么是 10k+100nF，PWM 转 DAC 为什么勉强");

    char buf[BUFSZ];
    /* ---- 场景 A：机械按键消抖 ---- */
    printf("  【A】机械按键消抖：上拉 10k，引脚对地 100nF，VCC = 3.3V\n");
    const double rA = 10e3, cA = 100e-9, vccA = 3.3;
    const double tauA = rA * cA;
    fmt_t(buf, tauA);
    printf("    τ = RC = %s，5τ = ", buf);
    fmt_t(buf, 5.0 * tauA);
    printf("%s\n", buf);
    printf("    机械触点抖动典型持续 1 ~ 5 ms（弹跳 3~10 次，每次 0.1~1ms）。\n");
    fmt_t(buf, 5.0 * tauA);
    printf("    5τ = %s 能吃掉典型的毫秒级短抖动；但抖到 5~10ms 的烂开关，\n", buf);
    printf("    RC 只能减弱、不能保证，最后仍要软件采样确认。\n");
    printf("    按下时的电流（松开时输入脚高阻，几乎不耗电）：\n");
    fmt_i(buf, vccA / rA);
    printf("      I = 3.3 / 10k = %s；换成 1k 就是 ", buf);
    fmt_i(buf, vccA / 1e3);
    printf("%s，\n", buf);
    fmt_i(buf, vccA / 100e3);
    printf("      换成 100k 只有 %s，但源阻抗高了 10 倍，脚上感应噪声明显变大。\n", buf);
    printf("    为什么是 10k 而不是 1k 或 1M —— 三方折中：\n");
    fmt_i(buf, vccA / rA);
    printf("      1) 功耗：10k 时按下电流 %s，电池设备扛得住；\n", buf);
    printf("      2) 抗噪：源阻抗越高，电场耦合越容易把节点抬过阈值；100nF 提供低阻。\n");
    printf("      3) 触点寿命：按下瞬间 100nF 通过开关触点放电，峰值由 ESR/走线决定\n");
    printf("         （几安、几 ns，能量仅 0.5·C·V²≈0.55µJ），不会烧 IO；但廉价开关\n");
    printf("         反复电蚀会接触不良，要长寿命就在电容支路串 100Ω~1kΩ。\n");
    printf("      上拉电流走的是 10k + 开关，不灌进高阻输入脚；换成 100Ω 只是白白耗\n");
    printf("      33mA，并不会「烧引脚」。\n");
    printf("    结论：10k + 100nF（τ=1ms）是功耗、抗噪、触点寿命的交点。\n");

    /* ---- 场景 B：PWM 变 DAC ---- */
    printf("\n  【B】PWM 变 DAC：3.3V / 20kHz PWM，RC 低通还原模拟量\n");
    printf("    纹波近似（τ 远大于 PWM 周期时，电容在小窗口里近似线性充放）：\n");
    printf("      ΔV ≈ VCC · T / (2τ)，T = 1/f_pwm\n\n");
    printf("    ");
    pad("R", 14);
    pad("C", 10);
    pad("τ", 12);
    pad("纹波", 12);
    pad("有效位数", 11);
    pad("建立时间 5τ", 14);
    printf("能不能用\n");
    printf("    ------------------------------------------------------------------------------------------------\n");
    struct PwmCase {
        double r, c;
    };
    const struct PwmCase pwm_cases[] = {{1e3, 1e-6}, {10e3, 1e-6}, {47e3, 680e-9}, {100e3, 4.7e-6}};
    const double f_pwm = 20e3;
    for (size_t k = 0; k < sizeof pwm_cases / sizeof pwm_cases[0]; ++k) {
        const double tau = pwm_cases[k].r * pwm_cases[k].c;
        const double ripple = vccA / (f_pwm * 2.0 * tau);
        const double bits = log2(vccA / ripple);
        char bs[BUFSZ];
        snprintf(bs, BUFSZ, "%.1f bit", bits);
        const char *verdict;
        if (bits >= 12.0) {
            verdict = "纹波够了，但 5τ = 秒级，只能当静态参考";
        } else if (bits >= 9.5) {
            verdict = "8 位有余量，代价是建立时间要认";
        } else if (bits >= 7.5) {
            verdict = "勉强当 8 位参考用，别指望线性度";
        } else if (bits >= 5.5) {
            verdict = "只够读电位器级别的应用";
        } else {
            verdict = "纹波太大，只能做粗略的慢信号";
        }
        printf("    ");
        fmt_r(buf, pwm_cases[k].r);
        pad(buf, 14);
        fmt_c(buf, pwm_cases[k].c);
        pad(buf, 10);
        fmt_t(buf, tau);
        pad(buf, 12);
        fmt_v(buf, ripple);
        pad(buf, 12);
        pad(bs, 11);
        fmt_t(buf, 5.0 * tau);
        pad(buf, 14);
        printf("%s\n", verdict);
    }
    fmt_v(buf, vccA / 256.0);
    printf("    想要 8 位（纹波 ≤ 3.3/256 = %s）：τ ≥ 128·T = ", buf);
    fmt_t(buf, 128.0 / f_pwm);
    printf("%s，\n", buf);
    fmt_t(buf, 640.0 / f_pwm);
    printf("    代价是建立时间 5τ = %s —— 慢到你必须接受。这就是【RC 换不来精度】的意思。\n", buf);

    /* ---- 场景 C：555 / 施密特阈值 ---- */
    printf("\n  【C】555 多谐振荡器 / 施密特触发器：为什么公式里是 0.693 和 1.1\n");
    const double r1 = 10e3, r2 = 10e3, cC = 100e-9;
    const double t_hi = 0.693 * (r1 + r2) * cC;  /* 充电走 R1+R2，从 1/3 充到 2/3 */
    const double t_lo = 0.693 * r2 * cC;         /* 放电只走 R2，从 2/3 放到 1/3 */
    printf("    555 的上阈值 = 2/3·VCC，下阈值 = 1/3·VCC，这两个比例是芯片内部\n");
    printf("    三颗 5k 电阻分出来的——是的，555 的名字就来自这三颗 5k。\n");
    printf("    从 1/3 充到 2/3：t = -τ·ln(1/2) = 0.693τ（充电走 R1+R2）\n");
    printf("    从 2/3 放到 1/3：t = 0.693τ（放电只走 R2）\n");
    printf("    取 R1 = R2 = 10k，C = 100nF：\n");
    fmt_t(buf, t_hi);
    printf("      t_high = 0.693·(R1+R2)·C = %s\n", buf);
    fmt_t(buf, t_lo);
    printf("      t_low  = 0.693·R2·C      = %s\n", buf);
    fmt_t(buf, t_hi + t_lo);
    printf("      周期 = %s，频率 = ", buf);
    fmt_hz(buf, 1.0 / (t_hi + t_lo));
    printf("%s，占空比 = %.1f%%\n", buf, (r1 + r2) / (r1 + 2.0 * r2) * 100.0);
    printf("    看到没：占空比天生大于 50%%（因为充电必经 R1），\n");
    printf("    要 50%% 占空比就得给 R2 并一颗二极管把充电旁路掉——\n");
    printf("    这不是 hack，这就是 555 数据手册上那张图的来历。\n");
}

/* =====================================================================
 * 八、演示 ⑤：上电冲击电流 —— 电容在 t=0 是一片短路
 *     回路：V --[Rlim = 源内阻+走线]-- [C + ESR] -- GND
 *     I(0) = V / (Rlim + ESR)      ← 峰值，就在合闸那一瞬间
 *     τ = (Rlim + ESR)·C
 *     每次上电，电阻上固定烧掉 ½CV² 的焦耳，与 R 大小无关！
 *     （R 大只是烧得慢，总能量一分不少。这个结论反直觉，但它是对的。）
 * ===================================================================== */
static void demo_inrush(void) {
    rule("演示 5 上电冲击电流：电容充电那一刻，你的电源被短路了");

    const double vcc = 5.0;
    char buf[BUFSZ];
    fmt_v(buf, vcc);
    printf("  电路：理想 %s --[Rlim：源内阻 + 走线 + 开关接触电阻]-- [C + ESR] -- GND\n", buf);
    printf("  I(0) = VCC / (Rlim + ESR)；τ = (Rlim + ESR)·C；每次上电电阻上烧掉 0.5·C·VCC²\n\n");
    printf("  ");
    pad("场景", 22);
    pad("C", 10);
    pad("Rlim+ESR", 12);
    pad("I(0)", 12);
    pad("τ", 11);
    pad("峰值功率", 13);
    pad("单次能耗", 12);
    printf("后果\n");
    printf("  ----------------------------------------------------------------------------------------------------------------\n");

    struct Case {
        const char *name;
        double c;
        double rlim;  /* 已经包含 ESR 的回路总电阻 */
    };
    const struct Case cases[] = {
        {"芯片旁 100nF 去耦", 100e-9, 0.5},
        {"MCU 电源脚 10uF MLCC", 10e-6, 0.55},
        {"板级 100uF 电解输入", 100e-6, 1.5},
        {"470uF 大电容 + 细线", 470e-6, 1.0},
        {"热插拔 1000uF 底板", 1000e-6, 0.35},
        {"理想导线直冲 100uF", 100e-6, 0.005},
    };
    for (size_t k = 0; k < sizeof cases / sizeof cases[0]; ++k) {
        const double i0 = vcc / cases[k].rlim;
        const double tau = cases[k].rlim * cases[k].c;
        const double p0 = vcc * vcc / cases[k].rlim;
        const double e = 0.5 * cases[k].c * vcc * vcc;
        const char *verdict;
        if (i0 > 500.0) {
            verdict = "不可能发生：导线先熔，或电源打嗝保护，或 USB 口直接断电";
        } else if (tau < 1e-6 && i0 > 1.0) {
            /* 回路电感让 di/dt 来不及建立，真实波形是窄尖峰 */
            verdict = "窄尖峰（几 ns）：不烧元件，但制造 EMI 和电源平面振铃";
        } else if (i0 >= 10.0) {
            verdict = "电源 UVLO 掉压 / USB 限流 500mA 直接跳闸，必须缓启动";
        } else if (i0 > 3.0) {
            verdict = "轻则输出打嗝，重则复位；开关触点在这一下开始烧蚀";
        } else {
            verdict = "量级可控，这是设计想让你看到的那一档";
        }
        char p_str[BUFSZ];
        char e_str[BUFSZ];
        snprintf(p_str, BUFSZ, "%.2f W", p0);
        if (e >= 1.0) {
            snprintf(e_str, BUFSZ, "%.2f J", e);
        } else if (e >= 1e-3) {
            snprintf(e_str, BUFSZ, "%.2f mJ", e * 1e3);
        } else {
            snprintf(e_str, BUFSZ, "%.2f uJ", e * 1e6);
        }

        printf("  ");
        pad(cases[k].name, 22);
        fmt_c(buf, cases[k].c);
        pad(buf, 10);
        fmt_r(buf, cases[k].rlim);
        pad(buf, 12);
        fmt_i(buf, i0);
        pad(buf, 12);
        fmt_t(buf, tau);
        pad(buf, 11);
        pad(p_str, 13);
        pad(e_str, 12);
        printf("%s\n", verdict);
    }

    printf("\n  师傅批注：\n");
    printf("    先给小电容那一行打个补丁：10A 是【理想电压源 + 纯电阻回路】算出来的，\n");
    printf("    真实回路还有几 nH 电感，di/dt 被卡住，电流压根来不及冲到 10A——\n");
    printf("    100nF 的真实冲击是【窄尖峰】，峰值几安、持续几 ns。\n");
    printf("    但这恰恰是最阴的一种：幅度大、时间窄、频次高，\n");
    printf("    它不烧元件，它制造 EMI，并且把电源平面打出振铃。\n");
    printf("    看最后一行：回路电阻只剩 5mΩ 时，I(0) = 1000A。当然它不会真的到 1000A，\n");
    printf("    因为电源有内阻、走线有电感、电容有 ESR、电池会压降——\n");
    printf("    但这些【寄生】不是设计出来的，是撞运气。所以工程做法是：\n");
    printf("    把限流这件事【主动设计出来】——热插拔控制器 + MOS 缓启动、\n");
    printf("    NTC 热敏电阻（冷态几欧，热态几十毫欧）、或者先串一颗电阻再短接。\n");
    printf("    再注意【单次能耗】那一列：它只跟 C 和 VCC² 有关，跟 R 无关。\n");
    printf("    把 R 加大不会少烧能量，只会把同一份能量摊到更长的时间上。\n");
    printf("    所以真正省电的办法是【减少上电时要充的总电容】，\n");
    printf("    或者干脆别反复上下电——这也是为什么大电容板子要做缓启动而不是硬撞。\n");

    /* 一个具体画面：去耦电容到底能顶多久 */
    printf("\n  [附] 去耦电容能顶多久？—— 用 ΔV = I·Δt / C 口算：\n");
    printf("  ");
    pad("电容", 12);
    pad("允许跌落", 12);
    pad("负载电流", 12);
    printf("能顶\n");
    struct Top {
        double c;
        double dv;
        double i;
    };
    const struct Top tops[] = {
        {100e-9, 0.05, 0.05},
        {1e-6, 0.05, 0.05},
        {10e-6, 0.1, 0.2},
        {100e-6, 0.1, 0.2},
        {470e-6, 0.1, 0.5},
    };
    for (size_t k = 0; k < sizeof tops / sizeof tops[0]; ++k) {
        const double dt = tops[k].dv * tops[k].c / tops[k].i;
        printf("  ");
        fmt_c(buf, tops[k].c);
        pad(buf, 12);
        fmt_v(buf, tops[k].dv);
        pad(buf, 12);
        fmt_i(buf, tops[k].i);
        pad(buf, 12);
        fmt_t(buf, dt);
        printf("%s（只够喂 %zu 个 5ns 的时钟周期）\n", buf, (size_t)(dt / 5e-9));
    }
    fmt_t(buf, 0.05 * 100e-9 / 0.05);
    printf("    → 100nF 能顶 10us 吗？不能，它只顶 %s。160MHz 的 MCU 一个时钟 6.25ns，\n", buf);
    printf("      也就是说 100nF 只够喂十几个周期。这就是为什么去耦要【每个电源脚一颗、紧挨着放】，\n");
    printf("      而不是板上凑三颗大的——大电容管毫秒，小电容管纳秒，它们不是一个班。\n");
}

/* =====================================================================
 * 九、演示 ⑥：长走线 + 100nF —— 这不是去耦，这是一台谐振器
 *     走线有电感（PCB 约 1nH/mm，排线更长），电容有 ESL。
 *     串联谐振 f = 1/(2π√(LC))，品质因数 Q = √(L/C)/R。
 *     R < 2√(L/C) → 欠阻尼 → 上电振铃，过冲接近 2 倍电压。
 *     这里顺手解一遍二阶微分方程，让你看看【数值仿真】处理二阶也这么短。
 * ===================================================================== */
static void demo_long_trace(void) {
    rule("演示 6 长走线 + 100nF：为什么去耦电容要【紧贴】芯片");

    const double vcc = 3.3;
    const double c = 100e-9;

    char buf[BUFSZ];
    printf("  同一颗 100nF，接法不同，行为完全不同：\n");
    printf("  ");
    pad("接法", 30);
    pad("回路 L", 11);
    pad("回路 R", 11);
    pad("谐振频率", 12);
    pad("Q", 8);
    printf("上电会怎样\n");
    printf("  ------------------------------------------------------------------------\n");

    struct Case {
        const char *name;
        double l;
        double r;
    };
    const struct Case cases[] = {
        {"0402 紧贴引脚（ESL 0.6nH）", 0.6e-9, 0.05},
        {"0603 走线 3mm（约 3nH）", 3.0e-9, 0.08},
        {"杜邦线 10cm（约 100nH）", 100e-9, 0.25},
        {"排线 30cm（约 300nH）", 300e-9, 0.20},
        {"长排线 30cm + 细线阻 3.6Ω", 300e-9, 3.60},
    };
    for (size_t k = 0; k < sizeof cases / sizeof cases[0]; ++k) {
        const double f0 = 1.0 / (2.0 * PI * sqrt(cases[k].l * c));
        const double z0 = sqrt(cases[k].l / c);
        const double q = z0 / cases[k].r;
        char qs[BUFSZ];
        snprintf(qs, BUFSZ, "%.2f", q);
        /* 二阶阶跃过冲 = exp(-π/√(4Q²-1))，Q 越大冲得越狠，Q→∞ 时逼近 2 倍 */
        const double ovs = (q < 0.5) ? 0.0 : exp(-PI / sqrt(4.0 * q * q - 1.0));
        char oss[BUFSZ];
        snprintf(oss, BUFSZ, "%.0f%%", ovs * 100.0);
        char verdict[BUFSZ * 3];
        if (q < 0.5) {
            snprintf(verdict, sizeof verdict, "过阻尼：平滑爬升，不振铃（这是想要的）");
        } else if (q < 1.5) {
            snprintf(verdict, sizeof verdict, "过冲约 %s，荡一两下就停", oss);
        } else if (q < 5.0) {
            snprintf(verdict, sizeof verdict, "过冲约 %s，反复冲击几十 ns", oss);
        } else {
            snprintf(verdict, sizeof verdict, "过冲约 %s：接近 2 倍电压，耐压余量先遭殃", oss);
        }
        printf("  ");
        pad(cases[k].name, 30);
        snprintf(buf, BUFSZ, "%.1f nH", cases[k].l * 1e9);
        pad(buf, 11);
        fmt_r(buf, cases[k].r);
        pad(buf, 11);
        fmt_hz(buf, f0);
        pad(buf, 12);
        pad(qs, 8);
        printf("%s\n", verdict);
    }

    /* ---- 真解一遍二阶：L·di/dt = V - vC - i·R，C·dvC/dt = i ---- */
    printf("\n  [解一遍给你看] 30cm 排线（300nH）+ 0.2Ω + 100nF，上 3.3V：\n");
    const double L = 300e-9, r = 0.20;
    const double t_end = 3.0e-6;
    const int steps = 300000;
    const double dt = t_end / steps;
    static double vc[300000 / 3000 + 1];
    double i = 0.0, vC = 0.0, peak = 0.0, i_peak = 0.0;
    size_t store = 0;
    for (int k = 0; k < steps; ++k) {
        const double di = (vcc - vC - i * r) / L;
        const double dv = i / c;
        i += di * dt;
        vC += dv * dt;
        if (vC > peak) peak = vC;
        if (i > i_peak) i_peak = i;
        if (k % 3000 == 0) {
            vc[store++] = vC;
        }
    }
    fmt_t(buf, dt);
    printf("  数值积分（%d 步，dt = %s）结果：\n", steps, buf);
    ascii_plot(vc, store, t_end, 2.2 * vcc, 0.5e-6, "0.5us", 13, 61);
    fmt_v(buf, peak);
    printf("    电容电压峰值 = %s（是 VCC 的 %.2f 倍）\n", buf, peak / vcc);
    fmt_i(buf, i_peak);
    printf("    回路电流峰值 = %s\n", buf);
    printf("    解析对照：欠阻尼二阶，特征阻抗 √(L/C) = %.3f Ω，", sqrt(L / c));
    fmt_hz(buf, 1.0 / (2.0 * PI * sqrt(L * c)));
    printf("谐振 %s，", buf);
    fmt_v(buf, 2.0 * vcc);
    printf("理论峰值 ≈ 2·VCC = %s\n", buf);

    printf("\n  师傅批注：\n");
    printf("    谐振频率 918.9kHz —— 正好撞在 MCU 时钟及其谐波、DC-DC 开关频率的频段上。\n");
    printf("    于是【加了电容反而更吵】：芯片在 1MHz 附近抽电流，\n");
    printf("    走线电感 + 电容正在谐振，电源纹波被放大好几倍。\n");
    printf("    现场表现就是：加了一颗 100nF，辐射测试反而超标；\n");
    printf("    把电容挪近一点，或者换 0402 封装（ESL 小一半），立刻好。\n");
    printf("    第二层意思在【为什么是 100nF 而不是 10uF】：\n");
    printf("    电容自己也有 ESL（0402 约 0.4~0.6nH，0805 约 0.7nH，大容值更大），\n");
    printf("    超过自谐振频率它就变成电感，去耦彻底失效：\n");
    printf("      100nF / ESL 0.6nH → 自谐振 ");
    fmt_hz(buf, 1.0 / (2.0 * PI * sqrt(0.6e-9 * 100e-9)));
    printf("%s\n", buf);
    printf("      1uF   / ESL 0.7nH → 自谐振 ");
    fmt_hz(buf, 1.0 / (2.0 * PI * sqrt(0.7e-9 * 1e-6)));
    printf("%s\n", buf);
    printf("      10uF  / ESL 1.0nH → 自谐振 ");
    fmt_hz(buf, 1.0 / (2.0 * PI * sqrt(1.0e-9 * 10e-6)));
    printf("%s\n", buf);
    printf("    所以去耦网络是【并联不同容值、各管一段频段】，\n");
    printf("    再靠【紧贴 + 短回路】把 L 压下去。L 是敌人，不是 C。\n");
}

int main(void) {
    printf("================================================================\n");
    printf("  第 1 关 · 电路基础 · RC 暂态数值仿真（一阶/二阶，零第三方库）\n");
    printf("  核心只有一句：电容电压不能跳变，它以 τ = RC 的速率指数逼近\n");
    printf("================================================================\n");

    demo_euler();
    demo_waveform();
    demo_sweep();
    demo_applications();
    demo_inrush();
    demo_long_trace();

    printf("\n================================================================\n");
    printf("  收工。把这三句带走：\n");
    printf("  1) τ = RC 不是【充满的时间】，是【还差 36.8%%】。稳定 = 5τ = 99.3%%。\n");
    printf("  2) 上电那一瞬间，所有电容都是短路，你的电源看到的是【最小负载】。\n");
    printf("     峰值电流 = VCC / 回路总电阻，而这个总电阻常常只有几十毫欧。\n");
    printf("  3) 去耦的敌人是电感不是电容。同一颗 100nF，紧贴是滤波，\n");
    printf("     拉长 30cm 就是一座 918.9kHz 的谐振塔。\n");
    printf("  下一步：打开 rc_animator.html 亲手把 τ 拖来拖去，\n");
    printf("  再把 voltage_divider_lab.html 的负载滑杆拉到和 R2 同级——\n");
    printf("  看着那个误差数字变红，你就毕业了。\n");
    printf("================================================================\n");
    return 0;
}
