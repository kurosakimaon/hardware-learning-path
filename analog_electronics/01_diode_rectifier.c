/* ================================================================
 *  01_diode_rectifier.c —— 二极管：从方程的解到"开机烧整流桥"
 * ================================================================
 *
 *  【这个程序讲什么】
 *  徒弟，教科书告诉你"二极管导通压降 0.7V"。
 *  师傅先给你把这句话拆开：那个 0.7V 是从哪来的？为什么 1mA 时是
 *  0.55V、1A 时是 1.1V？为什么整流桥数据表第一行写的是
 *  "IFSM = 30A @ 8.3ms"——一个 1A 的整流桥，凭什么允许 30A？
 *
 *  这个程序就是回答这几问的算盘（纯 C 实现，无任何第三方库）：
 *    1) 不假设 0.7V，直接用肖克利方程 Is·(exp(Vd/nVt)-1)，
 *       用牛顿迭代解出真实工作点（Is / n / Vt / 体电阻 Rs 都能改）；
 *    2) 数值仿真半波 / 全波(中心抽头) / 桥式整流 + 滤波电容 + 负载的
 *       完整时域波形，量出纹波峰峰值、二极管导通角、峰值电流、
 *       有效值电流、反向峰值电压 PIV、电容 ESR 发热；
 *    3) 把电容、负载、源阻抗、ESR 各扫一遍，让你亲眼看到：
 *       电容越大 → 纹波越小，但二极管峰值电流越大、导通角越小、
 *       发热越凶、开机浪涌越猛。这是同一件事的两面，不是两个指标。
 *    4) 最后算稳压管限流电阻与功耗——模拟工程师的第一道应用题。
 *
 *  ★ 为什么整流后开机瞬间的电流能烧整流桥？★
 *  因为开机那一刻，滤波电容是空的，它两端电压不能突变，
 *  等于你把一个"短路"直接怼在变压器次级上。此时限流的只有
 *  绕组电阻 + 线路电阻 + 二极管体电阻，加起来常常不到 0.5Ω。
 *  如果电源刚好在峰值附近合闸：I_pk ≈ Vpeak / Rs ≈ 17V / 0.2Ω = 85A。
 *  稳态时二极管只在"电容被负载放掉一点、且输入瞬时值超过它"的
 *  那一小段导通，峰值不过 1~2A；开机那一脚：过零合闸约稳态的十几倍，
 *  峰值合闸可达几十倍（取决于合闸相位与回路电阻）。
 *  更狠的是：浪涌会先打断最弱那只二极管的键合线——桥子里一只开路，
 *  剩下三只变成半波工作、集体过载，第二天客户打电话说"板子冒烟了"。
 *  所以大电容输入端要么串 NTC 热敏电阻（开机后自己被加热降阻），
 *  要么继电器短接 NTC，要么用带缓启动的整流/DCDC 前端。
 *  下面"源阻抗扫描"那一栏就是这一课的账本。
 *
 *  【怎么编译运行】
 *    gcc -std=c17 -O2 -Wall -Wextra 01_diode_rectifier.c -o 01_diode.exe
 *    运行前先在控制台执行： chcp 65001     ← 不做这一步中文全是乱码
 *    再运行： .\01_diode.exe
 *    （程序内部也会调用 SetConsoleOutputCP(65001) 兜底，
 *      但 chcp 65001 管的是输入侧与某些终端的字体，手动执行最稳。）
 *
 *  【预期看到什么（四段）】
 *    ① 工作点表：同一个二极管，1µA→1A，压降 0.33V→1.03V，动态电阻
 *       从 26kΩ 掉到 0.08Ω——"0.7V"这个谎言当场破产；
 *    ② 三种整流拓扑对比：半波纹波最大、PIV 吃 2 倍，桥式 PIV 只吃 1 倍；
 *    ③ 电容 / 负载 / 源阻抗 / ESR 四张扫描表 + 开机浪涌倍数；
 *    ④ 稳压管限流电阻的设计窗口，两行"无解"教你什么叫余量。
 *
 *  【老师傅点拨】
 *    看到"纹波"别只盯着容值。电容上有两块压降：容值那块（I·Δt/C）
 *    和 ESR 那块（I_peak × ESR）。峰值电流一冲，ESR 上就是一个尖，
 *    你示波器量到的纹波里那个窄尖就是它，而万用表根本看不见它。
 *    另一个：二极管发热要用有效值电流算（≈I_rms²·V_F），不是平均电流。
 *    导通角一窄，峰值/有效值比（crest）能到 3~5，
 *    同一个"平均 150mA"的负载，二极管可能吃到 1A 有效值。
 * ================================================================ */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef _WIN32
#include <windows.h>
#endif

/* -std=c17 严格模式下 M_PI 不一定可见，自己定义，别依赖扩展 */
#define PI 3.14159265358979323846

/* ----------------------------------------------------------------
 * 中文对齐：printf 的 %-Ns 按"字节"补空格，而一个中文占 3 字节、显示 2 列，
 * 所以中文表格必然错位。这个函数按"显示宽度"补空格，表才对齐。
 * ---------------------------------------------------------------- */
static char *pad(const char *s, int width)
{
    static char buf[512];
    size_t len = strlen(s);
    int w = 0;
    int i;

    if (len >= sizeof(buf)) len = sizeof(buf) - 1;
    for (i = 0; i < (int)len; ++i) {
        unsigned char c = (unsigned char)s[i];
        if (c < 0x80) w += 1;                    /* ASCII */
        else if ((c & 0xC0) == 0x80) w += 0;     /* UTF-8 延续字节 */
        else w += (c >= 0xE0) ? 2 : 1;           /* 三字节（中文）算 2 列 */
    }
    memcpy(buf, s, len);
    buf[len] = '\0';
    if (w < width && len + (size_t)(width - w) < sizeof(buf)) {
        int k;
        for (k = 0; k < width - w; ++k) buf[len + (size_t)k] = ' ';
        buf[len + (size_t)(width - w)] = '\0';
    }
    return buf;
}

static void setup_console_utf8(void)
{
#ifdef _WIN32
    SetConsoleOutputCP(65001);
#endif
}

/* ----------------------------------------------------------------
 * 肖克利方程与牛顿迭代
 *   I = Is * (exp(Vd / (n*Vt)) - 1)
 *   直接反解 Vd 时指数会溢出，工程上解"给定外围求 I"的形式：
 *   KVL:  Vdrive = I*Rser + n*Vt*ln(1 + I/Is)
 *   这是关于 I 的超越方程，用带二分保护的牛顿法，一定收敛。
 * ---------------------------------------------------------------- */
typedef struct {
    double Is;         /* 反向饱和电流 (A) */
    double n;          /* 发射系数 1~2 */
    double Vt;         /* 热电压 (V)，室温 25.85mV */
    double Rs;         /* 体电阻/键合线电阻 (Ω)，大电流时全靠它撑住曲线 */
    const char *name;
} Diode;

static double diode_drop(const Diode *d, double I)
{                                /* 给定电流求压降（解析式） */
    double arg;
    if (I <= 0.0) return 0.0;
    arg = I / d->Is;
    if (arg > 1e15) arg = 1e15;  /* 防 log 溢出后的数值噪声 */
    return d->n * d->Vt * log(1.0 + arg) + I * d->Rs;
}

static double diode_rd(const Diode *d, double I)
{                                /* 小信号动态电阻 */
    if (I <= 0.0) return d->n * d->Vt / d->Is;   /* 截止区：近似开路 */
    return d->n * d->Vt / I + d->Rs;
}

/* 解 f(I) = Vdrive - I*Rser - drop(I) = 0，牛顿 + 二分保护 */
static double solve_current(const Diode *d, double Vdrive, double Rser)
{
    double lo, hi, I;
    int it;

    if (Vdrive <= 0.0) {
        return -d->Is * (1.0 - exp(Vdrive / (d->n * d->Vt)));   /* 反偏漏电 */
    }
    lo = 0.0;
    hi = Vdrive / Rser;                       /* 二极管当短路时的最大电流 */
    if (Vdrive - lo * Rser - diode_drop(d, lo) <= 0.0) return 0.0;  /* 连结压降都推不动 */
    if (Vdrive - hi * Rser - diode_drop(d, hi) > 0.0) return hi;    /* 二极管压降可忽略 */

    I = 0.5 * (lo + hi);
    for (it = 0; it < 80; ++it) {
        double fi = Vdrive - I * Rser - diode_drop(d, I);
        double dfdI, Inew;
        if (fi > 0.0) lo = I; else hi = I;    /* 收窄区间（保命用） */
        dfdI = -Rser - d->n * d->Vt / (I + d->Is);
        Inew = I - fi / dfdI;
        if (!(Inew > lo && Inew < hi)) Inew = 0.5 * (lo + hi);   /* 牛顿飞了就二分 */
        if (fabs(Inew - I) < 1e-13 + 1e-10 * fabs(I)) { I = Inew; break; }
        I = Inew;
    }
    return I;
}

/* ----------------------------------------------------------------
 * 整流 + 电容滤波 + 阻性负载 的时域仿真
 *   mode 0 = 半波（1 只二极管，负半周不充电）
 *   mode 1 = 全波中心抽头（2 只二极管，两个半绕组轮流充）
 *   mode 2 = 桥式（4 只，每次 2 只串联导通）
 * ---------------------------------------------------------------- */
typedef struct {
    double vout_max, vout_min;
    double vc_max, vc_min;       /* 不含 ESR 的电容电压 */
    double ripple_pp;            /* 输出总纹波（含 ESR 尖） */
    double ripple_cap;           /* 其中容值分量 */
    double spike_esr;            /* 其中 ESR 尖 */
    double vout_dc;
    double diode_peak, diode_rms, diode_avg;
    double crest;
    double cond_deg;             /* 导通角（按纹波周期 360° 计） */
    double piv;
    double cap_rms;              /* 流进电容的纹波电流有效值 */
    double esr_loss;             /* ESR 发热 */
    double surge_peak;           /* 开机第一周期间的峰值 */
    double iout_dc;
} RectResult;

static RectResult simulate_rectifier(double Vrms, double freq, const Diode *d,
                                     double Rsrc, double C, double ESR,
                                     double Rload, int mode, double phase0,
                                     int cycles)
{
    RectResult r;
    double T = 1.0 / freq;
    const int spc = 6000;                     /* 每工频周期采样点数 */
    const double dt = T / (double)spc;
    const int total = spc * cycles;
    const int nJunc = (mode == 2) ? 2 : 1;    /* 导通路径上串联几个结 */
    const double T_rect = (mode == 0) ? T : T / 2.0;   /* 纹波周期 */
    /* 导通角满量程：半波的纹波周期 = 整个工频周期(360°)；
       全波/桥式的纹波周期 = 工频半周期，所以满量程是 180°，这才是手册口径 */
    const double deg_full = (mode == 0) ? 360.0 : 180.0;
    Diode chain;                              /* 串联 n 个结：nVt 翻倍 */
    double vc = 0.0;                          /* 状态量：电容电压 */
    double sum_sq = 0.0, sum_avg = 0.0, sum_cap_sq = 0.0;
    double t_steady = 0.0;
    double cond_time = 0.0;                   /* 当前纹波周期内的导通时长 */
    double t_since_period = 0.0;
    double cond_sum = 0.0;                    /* 已完成纹波周期的导通角累加 */
    int cond_cnt = 0;
    int k;

    r.vout_max = 0.0; r.vout_min = 1e30;
    r.vc_max = 0.0;   r.vc_min = 1e30;
    r.ripple_pp = 0.0; r.ripple_cap = 0.0; r.spike_esr = 0.0;
    r.vout_dc = 0.0;
    r.diode_peak = 0.0; r.diode_rms = 0.0; r.diode_avg = 0.0;
    r.crest = 0.0; r.cond_deg = 0.0; r.piv = 0.0;
    r.cap_rms = 0.0; r.esr_loss = 0.0; r.surge_peak = 0.0; r.iout_dc = 0.0;

    chain = *d;
    chain.n = d->n * (double)nJunc;
    chain.Rs = d->Rs * (double)nJunc;

    for (k = 0; k < total; ++k) {
        double t = (double)k * dt;
        double vs = sqrt(2.0) * Vrms * sin(2.0 * PI * freq * t + phase0);
        double I = 0.0;
        double iLoad, iCap, vout;
        int settling;

        if (mode == 1) {                      /* 中心抽头：±vs 各一只二极管 */
            double I2;
            I = solve_current(&chain, vs - vc, Rsrc);
            I2 = solve_current(&chain, -vs - vc, Rsrc);
            if (I < 0.0) I = 0.0;
            if (I2 < 0.0) I2 = 0.0;
            I += I2;
        } else {
            double vsrc = (mode == 2) ? fabs(vs) : vs;
            I = solve_current(&chain, vsrc - vc, Rsrc);
            if (I < 0.0) I = 0.0;
        }

        iLoad = vc / Rload;
        iCap = I - iLoad;
        vc += iCap * dt / C;
        if (vc < 0.0) vc = 0.0;
        vout = vc + ESR * iCap;               /* ESR 上的瞬时压降 → 纹波里的尖 */

        settling = (k < spc * 3);             /* 前 3 个周期当开机暂态 */
        if (settling) {
            if (I > r.surge_peak) r.surge_peak = I;
            continue;
        }
        t_steady += dt;
        if (vout > r.vout_max) r.vout_max = vout;
        if (vout < r.vout_min) r.vout_min = vout;
        if (vc > r.vc_max) r.vc_max = vc;
        if (vc < r.vc_min) r.vc_min = vc;
        if (I > r.diode_peak) r.diode_peak = I;
        sum_sq += I * I * dt;
        sum_avg += I * dt;
        sum_cap_sq += iCap * iCap * dt;
        if (I > 1e-6) cond_time += dt;

        /* 反向峰值电压：半波 = vc - vs；中心抽头 = vc + |vs|；
         * 桥式：截止管承受 Vc + Vd(on)，在导通峰值处 ≈ |vs| - Vd。
         * 这里按 |vs| 计，比真实值多算约一个二极管压降（偏保守，不误导选型）。 */
        {
            double rev = 0.0;
            if (mode == 0) rev = vc - vs;
            else if (mode == 1) rev = vc + fabs(vs);
            else rev = fabs(vs);
            if (rev > r.piv) r.piv = rev;
        }

        t_since_period += dt;   /* 每满一个纹波周期，结算并重开窗口 */
        if (t_since_period >= T_rect) {
            cond_sum += deg_full * cond_time / T_rect;
            ++cond_cnt;
            t_since_period = 0.0;
            cond_time = 0.0;
        }
    }

    r.ripple_pp = r.vout_max - r.vout_min;
    r.ripple_cap = r.vc_max - r.vc_min;
    r.spike_esr = ESR * r.diode_peak;
    r.vout_dc = 0.5 * (r.vout_max + r.vout_min);
    r.iout_dc = r.vout_dc / Rload;
    r.diode_rms = (t_steady > 0) ? sqrt(sum_sq / t_steady) : 0.0;
    r.diode_avg = (t_steady > 0) ? sum_avg / t_steady : 0.0;
    r.cap_rms = (t_steady > 0) ? sqrt(sum_cap_sq / t_steady) : 0.0;
    r.crest = (r.diode_rms > 0) ? r.diode_peak / r.diode_rms : 0.0;
    r.cond_deg = (cond_cnt > 0) ? cond_sum / (double)cond_cnt : 0.0;
    r.esr_loss = ESR * r.cap_rms * r.cap_rms; /* 纹波电流在 ESR 上的发热 */
    return r;
}

static const char *mode_name(int m)
{
    return (m == 0) ? "半波(1只)" : (m == 1) ? "全波中心抽头(2只)" : "桥式(4只)";
}

static void hr(const char *title)
{
    printf("\n");
    printf("=== %s ===============================================\n", title);
}

/* ----------------------------------------------------------------
 * ① 二极管工作点：牛顿迭代解方程
 * ---------------------------------------------------------------- */
static void part1_operating_point(void)
{
    static const Diode sig  = {1e-12, 1.00, 25.85e-3, 0.10,  "1N4148 小信号"};
    static const Diode pwr  = {1e-9,  1.85, 25.85e-3, 0.035, "1N4007 整流"};
    static const Diode sch  = {1e-6,  1.05, 25.85e-3, 0.060, "1N5819 肖特基"};
    static const Diode leds = {1e-15, 1.90, 25.85e-3, 0.60,  "红色 LED"};
    const Diode *all[4];
    const double targets[5] = {1e-6, 0.1e-3, 1e-3, 10e-3, 100e-3};
    const double targets2[3] = {0.2, 1.0, 3.0};
    typedef struct { const Diode *d; double V; double R; const char *what; } C2;
    C2 cs[5];
    int i, j;

    all[0] = &sig; all[1] = &pwr; all[2] = &sch; all[3] = &leds;

    hr("① 不假设 0.7V：牛顿迭代解肖克利方程");

    printf("  回路方程：V源 = I·(R源 + R体) + n·Vt·ln(1 + I/Is)   ← 超越方程，牛顿法\n\n");
    printf("  %-18s %10s %11s %12s %11s\n",
           "device", "I(mA)", "Vd(V)", "rd(ohm)", "Vsrc(V)");
    printf("  ---------------------------------------------------------------\n");
    for (i = 0; i < 4; ++i) {
        for (j = 0; j < 5; ++j) {
            double I = targets[j];
            printf("  %s%10.3f %11.4f %12.3f %11.4f\n",
                   pad(all[i]->name, 18), I * 1e3, diode_drop(all[i], I),
                   diode_rd(all[i], I),
                   diode_drop(all[i], I) + I * (all[i]->Rs + 1.0));
        }
        for (j = 0; j < 3; ++j) {
            double I = targets2[j];
            printf("  %s%10.1f %11.4f %12.3f %11.4f\n",
                   pad(all[i]->name, 18), I * 1e3, diode_drop(all[i], I),
                   diode_rd(all[i], I),
                   diode_drop(all[i], I) + I * (all[i]->Rs + 1.0));
        }
        printf("\n");
    }

    printf("  解方程 sanity check（不是查表，是让牛顿法去解）：\n");
    printf("  %s%14s %14s\n", pad("回路", 26), "解出I(mA)", "落在Vd(V)");
    printf("  ---------------------------------------------------------\n");
    cs[0].d = &sig;  cs[0].V = 2.0;  cs[0].R = 1000.0; cs[0].what = "2V 串 1k 推 1N4148";
    cs[1].d = &sig;  cs[1].V = 2.0;  cs[1].R = 100.0;  cs[1].what = "2V 串 100 推 1N4148";
    cs[2].d = &pwr;  cs[2].V = 12.0; cs[2].R = 100.0;  cs[2].what = "12V 串 100 推 1N4007";
    cs[3].d = &leds; cs[3].V = 3.3;  cs[3].R = 220.0;  cs[3].what = "3.3V 串 220 点 LED（你天天算的）";
    cs[4].d = &sch;  cs[4].V = 3.3;  cs[4].R = 220.0;  cs[4].what = "3.3V 串 220 肖特基钳位";
    for (i = 0; i < 5; ++i) {
        double I = solve_current(cs[i].d, cs[i].V, cs[i].R);
        printf("  %s%14.3f %14.4f\n", pad(cs[i].what, 26),
               I * 1e3, diode_drop(cs[i].d, I));
    }
    printf("\n  读表：\n");
    printf("    · 1N4007 从 1µA 到 1A，压降 0.33V → 1.03V。哪里蹦出来的 0.7V？\n");
    printf("    · 动态电阻 rd = nVt/I + Rs：10µA 时 2.6kΩ，1A 时 0.06Ω。\n");
    printf("      小信号衰减、开关损耗、钳位漏电流，全看这个 rd，不看 0.7V。\n");
    printf("    · 肖特基压降低 3 倍（省钱在效率上），但反偏漏电大 1000 倍，\n");
    printf("      而且每升 10℃ 再翻一倍——高温机壳里靠肖特基做反接保护的要小心。\n");
    printf("    · 1N4148 额定只有 200mA，上面 1A/3A 两行是算给你看曲线的，\n");
    printf("      真那么用它在半秒钟内就会开路——记住器件是有额定值的。\n");
}

/* ----------------------------------------------------------------
 * ② 三种拓扑对比
 * ---------------------------------------------------------------- */
static void part2_topologies(void)
{
    const Diode d = {1e-9, 1.85, 25.85e-3, 0.035, "1N4007"};
    const double Vrms = 12.0, freq = 50.0;
    const double C = 1000e-6, ESR = 0.08, Rload = 100.0, Rsrc = 0.30;
    int m;

    hr("② 半波 / 全波中心抽头 / 桥式：同一台变压器，三种命运");

    printf("  条件：12Vrms/50Hz，R源=%.2fΩ，C=%.0fµF(ESR=%.0fmΩ)，RL=%.0fΩ\n\n",
           Rsrc, C * 1e6, ESR * 1e3, Rload);
    printf("  %s%9s %9s %9s %9s %9s %9s %9s %9s\n",
           pad("拓扑", 20), "Vout(V)", "rip_mV", "cond_deg",
           "Ipk(A)", "Irms(A)", "pk/rms", "PIV(V)", "Icap(A)");
    printf("  -------------------------------------------------------------------------\n");
    {
        RectResult rr[3];
        for (m = 0; m < 3; ++m) {
            rr[m] = simulate_rectifier(Vrms, freq, &d, Rsrc, C, ESR, Rload, m, 0.0, 14);
            printf("  %s%9.2f %9.1f %9.1f %9.3f %9.3f %9.2f %9.2f %9.3f\n",
                   pad(mode_name(m), 20), rr[m].vout_dc, rr[m].ripple_pp * 1e3,
                   rr[m].cond_deg, rr[m].diode_peak, rr[m].diode_rms, rr[m].crest,
                   rr[m].piv, rr[m].cap_rms);
        }
        printf("\n  师傅划重点：\n");
        printf("    · 纹波频率：半波 = 50Hz，全波/桥式 = 100Hz → 同样电容纹波近似减半，\n");
        printf("      而且半波让变压器直流磁偏，长期发热，电源工程里基本被禁用。\n");
        printf("    · PIV：半波 %.1fV、中心抽头 %.1fV（都≈2·Vpeak，实际因输出略低于峰值\n",
               rr[0].piv, rr[1].piv);
        printf("      而小一两伏），桥式只要 %.1fV(≈Vpeak)。所以低压大电流一律桥式\n", rr[2].piv);
        printf("      （管子多但耐压低、铜利用率高）；中心抽头只在高压小电流或需要\n");
        printf("      隔离双绕组时用。\n");
        printf("    · 导通角只有 %.0f~%.0f°（全波/桥式满量程 180°）：每个半周约七成时间\n",
               rr[1].cond_deg, rr[2].cond_deg);
        printf("      二极管是截止的，全部能量挤在那一小段灌进电容——pk/rms 高达 3~4，\n");
        printf("      这也是电容输入式滤波功率因数只有 0.5~0.6 的根本原因。\n");
    }
}

/* ----------------------------------------------------------------
 * ③ 电容 / 负载 / 源阻抗 / ESR 扫描 + 开机浪涌
 * ---------------------------------------------------------------- */
static void part3_cap_and_surge(void)
{
    const Diode d = {1e-9, 1.85, 25.85e-3, 0.035, "1N4007"};
    const double Vrms = 12.0, freq = 50.0, Rsrc = 0.30, Rload = 220.0;
    const double caps[6] = {100e-6, 220e-6, 470e-6, 1000e-6, 2200e-6, 4700e-6};
    const double rls[5] = {2200.0, 680.0, 220.0, 100.0, 47.0};
    const double rss[5] = {0.15, 0.30, 1.0, 2.2, 10.0};
    typedef struct { double esr; const char *name; const char *hint; } EC;
    EC ecs[4];
    int i;

    hr("③ 电容越大越好吗？——纹波、峰值电流、浪涌的三方交易");

    printf("  (a) 扫电容：固定 12Vrms、RL=220Ω、R源=0.30Ω、ESR=80mΩ、桥式\n\n");
    printf("  %-9s %9s %9s %9s %9s %9s %9s %10s %11s\n",
           "C(uF)", "Vout(V)", "rip_mV", "cond_deg", "Ipk(A)", "Irms(A)",
           "pk/rms", "ESR_mW", "dVcap_mV");
    printf("  ----------------------------------------------------------------------------------\n");
    for (i = 0; i < 6; ++i) {
        RectResult r = simulate_rectifier(Vrms, freq, &d, Rsrc, caps[i], 0.08, Rload, 2, 0.0, 14);
        printf("  %-9.0f %8.2f %9.1f %8.1f %8.3f %8.3f %9.2f %10.1f %10.1f\n",
               caps[i] * 1e6, r.vout_dc, r.ripple_pp * 1e3, r.cond_deg,
               r.diode_peak, r.diode_rms, r.crest, r.esr_loss * 1e3,
               r.ripple_cap * 1e3);
    }
    printf("\n  看到没：C 从 100µF 加到 4700µF，纹波降了二十多倍，\n");
    printf("          但二极管峰值电流翻了一倍多，导通角一路缩，\n");
    printf("          pk/rms 从 2.9 涨到 4 以上——变压器铜损(I_rms²)和二极管发热一起涨。\n");
    printf("          你以为加了个「便宜的大电容」，其实是给整流桥和变压器\n");
    printf("          换了个更凶的负载。经验值：每 100mA 负载配 1000~2200µF 就够了，\n");
    printf("          再大就是花钱买浪涌。\n");

    printf("\n  (b) 扫负载（C=1000µF 固定）：\n");
    printf("  %-9s %10s %9s %9s %9s %9s %10s\n",
           "RL(ohm)", "Iout_mA", "rip_mV", "Ipk(A)", "Irms(A)", "cond_deg", "Icap(A)");
    printf("  ----------------------------------------------------------------------\n");
    for (i = 0; i < 5; ++i) {
        RectResult r = simulate_rectifier(Vrms, freq, &d, Rsrc, 1000e-6, 0.08, rls[i], 2, 0.0, 14);
        printf("  %-9.0f %9.1f %9.1f %8.3f %8.3f %8.1f %9.3f\n",
               rls[i], r.iout_dc * 1e3, r.ripple_pp * 1e3,
               r.diode_peak, r.diode_rms, r.cond_deg, r.cap_rms);
    }
    printf("  → 最后一列是选电解电容的硬指标：手册写的「纹波电流额定值」\n");
    printf("    必须大于这一列，且高温还要降额。纹波电流超额的电容，\n");
    printf("    表现不是立刻坏，而是电解液干涸、容值掉一半、ESR 涨十倍，\n");
    printf("    半年后客户现场批量返修——这是最讨厌的那种故障。\n");

    printf("\n  (c) 扫源阻抗（C=2200µF、RL=220Ω 固定）——这一栏就是「烧桥」的账本\n");
    printf("  %-16s %9s %9s %11s %11s %9s\n",
           "Rsrc(ohm)", "Vout(V)", "rip_mV", "surge_Ipk", "steady_Ipk", "ratio");
    printf("  ------------------------------------------------------------------\n");
    for (i = 0; i < 5; ++i) {
        /* 峰值合闸：电容空着时电源正好在 90°，这才是 IFSM 要扛的最坏情况 */
        RectResult r = simulate_rectifier(Vrms, freq, &d, rss[i], 2200e-6, 0.05, Rload, 2, PI / 2.0, 8);
        printf("  %-16.2f %8.2f %9.1f %10.2f %10.2f %10.1f\n",
               rss[i], r.vout_dc, r.ripple_pp * 1e3,
               r.surge_peak, r.diode_peak,
               (r.diode_peak > 0 ? r.surge_peak / r.diode_peak : 0.0));
    }
    printf("  两个结论，都是拿钱换来的：\n");
    printf("    · 表里的 surge_Ipk 按最坏情况算：电容空着、电源正好在峰值合闸。\n");
    printf("      过零合闸只有十几倍，峰值合闸能到几十倍——所以 0.15Ω 一脚几十安，\n");
    printf("      而 1A 级整流桥的 IFSM 通常只给 30A/8.3ms。你开十次没事，\n");
    printf("      第十一次恰好在峰值合闸、电容正好空着，键合线就断了。\n");
    printf("      对策（按成本排）：串 5~10Ω 功率电阻或 NTC → 继电器短接 NTC →\n");
    printf("      有源浪涌限制 → 带缓启动的 DCDC 前端。别裸奔。\n");
    printf("    · 但源阻抗大了，稳态 Vout 掉、纹波反而变大（RC 分压把 100Hz 漏过来）。\n");
    printf("      「浪涌小」和「带载好」是矛盾的——所以要 NTC + 继电器，两全。\n");

    printf("\n  (d) 扫 ESR（C=1000µF、RL=220Ω）：同样的容值，不同的「心」\n");
    printf("  %s%10s %11s %11s   %s\n",
           pad("电容类型(ESR)", 26), "rip_mV", "esr_mV", "esr_mW", "寿命启示");
    printf("  ------------------------------------------------------------------\n");
    ecs[0].esr = 0.50;  ecs[0].name = "普通电解 105℃(500mΩ)"; ecs[0].hint = "尖峰最大，发热最凶";
    ecs[1].esr = 0.08;  ecs[1].name = "低纹波电解(80mΩ)";      ecs[1].hint = "常规选择";
    ecs[2].esr = 0.012; ecs[2].name = "聚合物钽(12mΩ)";        ecs[2].hint = "尖峰小，但要防浪涌击穿";
    ecs[3].esr = 0.004; ecs[3].name = "MLCC 陶瓷(4mΩ)";        ecs[3].hint = "尖峰最小，但可能引发谐振振铃";
    for (i = 0; i < 4; ++i) {
        RectResult r = simulate_rectifier(Vrms, freq, &d, Rsrc, 1000e-6, ecs[i].esr, Rload, 2, 0.0, 14);
        printf("  %s%10.1f %11.1f %11.2f   %s\n",
               pad(ecs[i].name, 26), r.ripple_pp * 1e3,
               r.spike_esr * 1e3, r.esr_loss * 1e3, ecs[i].hint);
    }
    printf("  · 电解寿命经验：温度每降 10℃ 寿命翻倍。105℃/2000h 的料，\n");
    printf("    自己把它周围的 60℃ 算进去，别指望它跑 2000 小时以上；\n");
    printf("    低温环境下普通电解 ESR 会涨好几倍——「冬天冷启动烧电源」就是它。\n");
    printf("  · 只换 MLCC 也不白给：低 ESR + 绕组电感会谐振，\n");
    printf("    示波器上你会看到一串衰减振铃，EMI 也跟着爆——要串小电阻或加 RC。\n");
}

/* ----------------------------------------------------------------
 * ④ 稳压管（齐纳）设计：限流电阻的窗口与最坏功耗
 * ---------------------------------------------------------------- */
static void part4_zener(void)
{
    /* 5.1V/500mW 稳压管的线性化模型：Vz(Iz) = Vz0 + Zz·Iz
       标称 5.1V @ 5mA，工作点附近动态电阻约 10Ω。
       注意 Vz 会随电流上升——这正是"稳压管不能当基准"的根。 */
    const double Vz_nom = 5.1, Iz_test = 5.0e-3, Zz = 10.0;
    const double Vz0 = Vz_nom - Zz * Iz_test;   /* 从标称点外推到零电流：5.05V */
    const double Iz_min = 1.0e-3;               /* 低于它就掉进膝区 */
    const double Pz_max = 0.5;                  /* SOD-123 500mW，先按额定算 */
    double Iz_max, Vz_max, Vz_min, R, Vin, IL, Iz, I_R, P_R, P_Z, line_reg;
    typedef struct { double vin_min, vin_max, rl_min; } Case;
    Case cases[4];
    int i;

    hr("④ 稳压管（齐纳）设计：限流电阻的窗口与最坏功耗");

    /* 解 (Vz0 + Zz·Iz)·Iz = Pz_max，取正根 */
    Iz_max = (-Vz0 + sqrt(Vz0 * Vz0 + 4.0 * Zz * Pz_max)) / (2.0 * Zz);
    Vz_max = Vz0 + Zz * Iz_max;
    Vz_min = Vz0 + Zz * Iz_min;

    printf("  条件：Vz=%.1fV@%.0fmA，Zz≈%.0fΩ（Iz=%.0fmA 时升到 %.2fV），\n",
           Vz_nom, Iz_test * 1e3, Zz, Iz_max * 1e3, Vz_max);
    printf("        Iz(min)=%.1fmA，Pz(max)=%.0fmW\n\n", Iz_min * 1e3, Pz_max * 1e3);

    printf("  %-14s %14s %10s %10s %10s %8s\n",
           "Vin(V)", "RLmin(ohm)", "Rmax_ohm", "Rmin_ohm", "Rsel_ohm", "status");
    printf("  ------------------------------------------------------------------\n");
    cases[0].vin_min = 8.0; cases[0].vin_max = 12.0; cases[0].rl_min = 1000.0;
    cases[1].vin_min = 8.0; cases[1].vin_max = 12.0; cases[1].rl_min = 150.0;
    cases[2].vin_min = 6.0; cases[2].vin_max = 12.0; cases[2].rl_min = 150.0;
    cases[3].vin_min = 5.6; cases[3].vin_max = 12.0; cases[3].rl_min = 680.0;
    for (i = 0; i < 4; ++i) {
        double il_max = Vz_min / cases[i].rl_min;
        double Rmax = (cases[i].vin_min - Vz_min) / (Iz_min + il_max); /* 最低压最重负载仍够 Iz */
        double Rmin = (cases[i].vin_max - Vz_max) / Iz_max;            /* 最高压空载不超功率 */
        int ok = Rmax > Rmin * 1.05;
        double Rsel = ok ? floor(0.5 * (Rmax + Rmin) + 0.5) : 0.0; /* 四舍五入到整数 Ω */
        char label[32];
        snprintf(label, sizeof(label), "%.1f~%.1f", cases[i].vin_min, cases[i].vin_max);
        printf("  %-14s %14.0f %10.0f %10.0f %10.0f %8s\n",
               label, cases[i].rl_min, Rmax, Rmin, Rsel, ok ? "OK" : "无解!");
    }
    printf("\n  第 3、4 行无解，原因就两条，背下来：\n");
    printf("    · Vin_min 只比 Vz 高 0.9V / 0.5V → 限流电阻没有空间，\n");
    printf("      输入一 sag，稳压管掉进膝区，输出跟着输入跑。\n");
    printf("      经验：Vin_min 至少 Vz + 2V，否则别用稳压管，直接上 LDO。\n");
    printf("    · 空载（IL=0）时全部电流灌进稳压管 → 电阻选小了就超功率。\n");
    printf("      这就是「稳压管电路必须一直带负载」的由来；不满足就补一个\n");
    printf("      假负载（比如 10kΩ 吃掉 0.5mA），或者干脆换 LDO。\n");

    R = 470.0; Vin = 12.0; IL = 5e-3;
    /* KVL: Vin - (Vz0 + Zz·Iz) = R·(Iz + IL) → Iz */
    Iz = (Vin - Vz0 - R * IL) / (R + Zz);
    I_R = Iz + IL;
    P_R = I_R * I_R * R;
    P_Z = (Vz0 + Zz * Iz) * Iz;
    line_reg = Zz / (R + Zz);                  /* dVout/dVin */
    printf("\n  取 R=470Ω、Vin=12V、IL=5mA（此时 Vz=%.2fV，不是标称的 %.1fV）：\n",
           Vz0 + Zz * Iz, Vz_nom);
    printf("    I_R = %.2fmA，Iz = %.2fmA，P_R = %.0fmW（0805 的 125mW 已用到 78%%，\n",
           I_R * 1e3, Iz * 1e3, P_R * 1e3);
    printf("    选 1206/0.25W 起），P_Z = %.0fmW\n", P_Z * 1e3);
    printf("    线性稳定度 ≈ %.1fmV/V（Zz=%.0fΩ 与 R 分压而来）\n",
           line_reg * 1e3, Zz);
    printf("    → 上一级整流留下的 1.2V 纹波，会漏进输出 %.0fmV。\n",
           line_reg * 1200.0);
    printf("      这就是「稳压管只配当粗略钳位/保护，不配当基准」的算术证明：\n");
    printf("      想要干净基准，要么前面 RC 两级分压，要么直接上基准源/低噪 LDO。\n");
}

int main(void)
{
    setup_console_utf8();
    printf("\n");
    printf("##############################################################\n");
    printf("#  第 2 关 · 模拟电路 · 01 二极管与整流\n");
    printf("#  从「方程的解」到「开机为什么烧整流桥」\n");
    printf("##############################################################\n");

    part1_operating_point();
    part2_topologies();
    part3_cap_and_surge();
    part4_zener();

    printf("\n--------------------------------------------------------------\n");
    printf("本程序小结（师傅要你记住的四句话）：\n");
    printf("  1. 二极管没有固定压降，只有「在这个电流下的压降」；rd = Vt/I 才是真身。\n");
    printf("  2. 滤波电容不是越大越好：纹波↓，但峰值电流、导通角窄化、铜损、浪涌全↑。\n");
    printf("  3. 开机浪涌 ≈ Vpeak/R源，只跟「电容空不空 + 回路电阻」有关，与负载无关，\n");
    printf("     所以要 NTC / 继电器 / 缓启动，别赌运气。\n");
    printf("  4. 稳压管是「要最小电流」的器件：没负载它就烧，输入没余量它就抖。\n");
    return 0;
}
