/* ================================================================
 *  02_opamp_lab.c —— 运放实战计算台：把「理想运放」这四个字打回原形
 * ================================================================
 *
 *  【这个程序讲什么】
 *  徒弟，你在第 1 关学会了两句咒语：「虚短」和「虚断」。
 *  靠着这两句，你能在纸上把增益写得飞起。然后你把板子焊出来，
 *  示波器告诉你：波形顶部是平的，或者正弦变成了三角，
 *  或者输出干脆贴在电源轨上一动不动。
 *
 *  这个程序就是那台告诉你真相的算盘（纯 C 实现，无任何第三方库）。
 *  它内置一个「不理想的运放」：
 *    · 有限直流增益 Aol      → 虚短只是"近似"，V- 和 V+ 总有 µV 级差
 *    · 单位增益带宽 GBW      → 闭环带宽 = GBW / 噪声增益，不是无穷
 *    · 压摆率 SR             → 大信号时运放是电流源，正弦被拉成三角
 *    · 输出摆幅 vs 电源轨    → 单电源运放的"轨到轨"是有条件的
 *    · 输入偏置电流 Ib       → 同相端少一个电阻，输出就被顶到轨
 *    · 输入失调电压 Vos      → 增益越高，它被放大得越狠
 *
 *  针对反相 / 同相 / 电压跟随 / 差分四种拓扑，它会算：理想增益、
 *  有限 Aol 下的真实增益与误差、闭环 -3dB 带宽、最大不失真频率
 *  min(f_cl, SR/2πVpeak)、输出是否撞轨、同相端偏置电阻取多少、
 *  Vos 与 Ib 折算到输出的误差。仪表放大器（第 5 种拓扑）单独一节：
 *  把三运放的节点方程列成 7×7 线性方程组用高斯消元当场解出来
 *  （含有限 Aol 与电阻失配），和 01 里解二极管是同一个路子——
 *  套公式你永远不知道 CMRR 是怎么被 0.1% 的电阻吃掉的。
 *
 *  【怎么编译运行】
 *    gcc -std=c17 -O2 -Wall -Wextra 02_opamp_lab.c -o 02_opamp.exe
 *    运行前先在控制台执行： chcp 65001     ← 不做这一步中文全是乱码
 *    再运行： .\02_opamp.exe
 *    （程序内部也会调用 SetConsoleOutputCP(65001) 兜底。）
 *
 *  【预期看到什么（七段）】
 *    ① 四只运放的"体检表"：从 µA741 到精密零漂移，参数摆一排；
 *    ② 4 种拓扑 × 多组电阻（换两只运放各跑一遍）：增益误差、噪声增益、
 *       闭环带宽、满功率带宽，谁当限制因素直接写出来；
 *    ③ 同一批案例的直流账：Vos·NG、Ib 通路、偏置电阻建议、撞轨判定；
 *    ④ 虚短虚断体检：算出真实 Vd，以及反相放大器输入阻抗为什么是 R1；
 *    ⑤ 翻车案例 A：增益 100 + GBW 1MHz，20kHz 正弦到底出来多少伏；
 *    ⑥ 翻车案例 B：电压跟随器直接带 1nF，二阶模型算出 Q 和过冲百分比；
 *    ⑦ 仪表放大器：解方程组算真实增益与 CMRR，看电阻失配的杀伤力。
 *
 *  【老师傅点拨】
 *    判断一个运放电路能不能活，只算三个数就够：
 *      噪声增益 NG = 1 + R2/R1（反相也一样，别用信号增益！）
 *      带宽 ≈ GBW / NG
 *      满功率带宽 = SR / (2π·Vpeak)
 *    再问一句"输出峰值 + 直流偏置，离电源轨还剩几伏"。
 *    三个数加一个"余量"，全对才叫设计；对不上就是你在意淫运放。
 * ================================================================ */

#include <math.h>
#include <stdio.h>
#include <string.h>

#ifdef _WIN32
#include <windows.h>
#endif

/* -std=c17 严格模式下 M_PI 不一定可见，自己定义，别依赖扩展 */
#define PI 3.14159265358979323846
#define TWO_PI (2.0 * PI)

/* ----------------------------------------------------------------
 * 最小复数工具（代替 C++ 的 std::complex）：A(f) = Aol / (1 + j·f/fb)
 * ---------------------------------------------------------------- */
typedef struct { double re, im; } Cplx;

static Cplx cplx_div(Cplx a, Cplx b)
{
    Cplx r;
    double den = b.re * b.re + b.im * b.im;
    r.re = (a.re * b.re + a.im * b.im) / den;
    r.im = (a.im * b.re - a.re * b.im) / den;
    return r;
}

static double cplx_abs(Cplx a) { return sqrt(a.re * a.re + a.im * a.im); }
static double cplx_arg(Cplx a) { return atan2(a.im, a.re); }

/* ----------------------------------------------------------------
 * 中文对齐：printf 的 %-Ns 按「字节」补空格，中文占 3 字节显示 2 列，表必歪。
 * 这个函数按「显示宽度」补，表才齐。
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
        if (c < 0x80) w += 1;
        else if ((c & 0xC0) == 0x80) w += 0;
        else w += (c >= 0xE0) ? 2 : 1;
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

static void hr(const char *title)
{
    printf("\n=== %s ===============================================\n", title);
}

/* ----------------------------------------------------------------
 * 非理想运放模型
 *   单极点开环：A(f) = Aol / (1 + j·f/fb)，fb = GBW/Aol
 * ---------------------------------------------------------------- */
typedef struct {
    const char *name;
    double Aol_dB, GBW, SR_Vus;
    double Vpos, Vneg;             /* 电源轨 */
    double vo_high, vo_low;        /* 输出可达的绝对电压上下限 */
    double vicm_high, vicm_low;    /* 输入共模范围 */
    double Vos_mV, Ib_nA, Ro_ohm;
} OpAmp;

static double op_aol(const OpAmp *o) { return pow(10.0, o->Aol_dB / 20.0); }
static double op_fb(const OpAmp *o)  { return o->GBW / op_aol(o); }
static double op_sr(const OpAmp *o)  { return o->SR_Vus * 1e6; }   /* V/s */

static Cplx openLoop(const OpAmp *o, double f)
{
    Cplx num, den;
    num.re = op_aol(o); num.im = 0.0;
    den.re = 1.0;       den.im = f / op_fb(o);
    return cplx_div(num, den);
}

/* ----------------------------------------------------------------
 * 一个放大案例
 *   g_ideal : 理想信号增益（反相为负）
 *   ng      : 噪声增益 = 1 + R2/R1，决定误差、带宽、噪声
 *   rp      : 同相端推荐的直流对地电阻
 *   r_ib    : 反相端看出去的等效电阻（Ib 直接乘它到输出）
 * ---------------------------------------------------------------- */
typedef struct {
    const char *topo;
    const char *vals;
    double g_ideal, ng, rp, r_ib, vin_peak, zin;
} Case;

static double parallel(double a, double b) { return a * b / (a + b); }

static double gainFinite(const OpAmp *o, const Case *c)
{
    return c->g_ideal / (1.0 + c->ng / op_aol(o));
}

static double bandwidth(const OpAmp *o, const Case *c) { return o->GBW / c->ng; }

static double fullPowerBW(const OpAmp *o, double vout_peak)
{
    if (vout_peak <= 0.0) return 1e12;
    return op_sr(o) / (TWO_PI * vout_peak);
}

static Cplx gainAt(const OpAmp *o, const Case *c, double f)
{
    Cplx A = openLoop(o, f);
    Cplx T;                        /* 环路增益 = A·β，β = 1/NG */
    Cplx num, den, G;
    T.re = A.re / c->ng; T.im = A.im / c->ng;
    num.re = c->g_ideal * T.re; num.im = c->g_ideal * T.im;
    den.re = 1.0 + T.re;  den.im = T.im;
    G = cplx_div(num, den);
    return G;
}

/* ----------------------------------------------------------------
 * ① 四只运放体检表
 * ---------------------------------------------------------------- */
static void part1_models(void)
{
    OpAmp ops[4] = {
        {"µA741 老前辈", 100.0, 1.0e6, 0.5, +15.0, -15.0, 13.0,   -14.0, 12.0,  -15.0, 1.5,    500.0,  75.0},
        {"LM358 万金油", 100.0, 1.1e6, 0.5,   5.0,   0.0,  3.5,     0.02,  3.5,    0.0, 2.0,     45.0,  50.0},
        {"轨到轨CMOS",   104.0, 1.0e6, 0.6,   5.0,   0.0,  4.95,    0.05,  5.4,   -0.3, 1.0,     0.001, 40.0},
        {"精密零漂移",   130.0, 8.0e6, 2.0,   5.0,   0.0,  4.985,   0.015, 4.985,  0.015, 0.0075, 0.0005, 10.0},
    };
    int i;

    hr("① 四只运放的体检表：你手里的从来不是同一个东西");

    printf("  %s%8s %10s %8s %15s %14s %11s %10s %9s\n",
           pad("opamp", 18), "Aol_dB", "GBW_kHz", "SR_Vus",
           "out_range_V", "vicm_V", "Vos_mV", "Ib_nA", "fb_Hz");
    printf("  -------------------------------------------------------------------"
           "---------------------\n");
    for (i = 0; i < 4; ++i) {
        char outr[24], vicr[24];
        snprintf(outr, sizeof(outr), "%.2f~%.2f", ops[i].vo_low, ops[i].vo_high);
        snprintf(vicr, sizeof(vicr), "%.2f~%.2f", ops[i].vicm_low, ops[i].vicm_high);
        printf("  %s%8.0f %10.0f %8.1f %15s %14s %11.4f %10.4f %9.2f\n",
               pad(ops[i].name, 18), ops[i].Aol_dB, ops[i].GBW / 1e3,
               ops[i].SR_Vus, outr, vicr, ops[i].Vos_mV, ops[i].Ib_nA, op_fb(&ops[i]));
    }
    printf("\n  师傅划重点：\n");
    printf("    · fb = GBW/Aol 只有 2~10Hz。开环增益在 10Hz 以上就以 -20dB/dec 往下掉，\n");
    printf("      你在 1MHz 下还拿 1e5 去算增益误差，那是拿八年前的地图找今天的路。\n");
    printf("    · LM358 供电 5V 时输出只能到 3.5V！它宣传的是「单电源」，\n");
    printf("      不是「轨到轨」。你要喂 0~3.3V 的 ADC，上面缺的 1.5V 就是你的量程误差。\n");
    printf("    · 轨到轨输出的代价写在脚注里：RL 越小摆幅越差。\n");
    printf("      「rail-to-rail, RL = 10k to V+/2」那行小字才是真相，2k 负载就没门了。\n");
    printf("    · Ib 从 500nA（双极输入）到 0.001nA（CMOS 输入）差 6 个数量级。\n");
    printf("      1MΩ 反馈电阻配 741：500nA × 1MΩ = 0.5V 直流偏移，量程当场作废。\n");
}

/* ----------------------------------------------------------------
 * 案例集：4 种拓扑 × 多组电阻
 * ---------------------------------------------------------------- */
static Case make_inv(const char *v, double r1, double r2, double vp)
{
    Case c;
    c.topo = "反相"; c.vals = v;
    c.g_ideal = -r2 / r1; c.ng = 1.0 + r2 / r1;
    c.rp = parallel(r1, r2); c.r_ib = r2; c.vin_peak = vp; c.zin = r1;
    return c;
}

static Case make_noninv(const char *v, double r1, double r2, double vp)
{
    Case c;
    c.topo = "同相"; c.vals = v;
    c.g_ideal = 1.0 + r2 / r1; c.ng = 1.0 + r2 / r1;
    c.rp = parallel(r1, r2); c.r_ib = r2; c.vin_peak = vp; c.zin = 1e12;
    return c;
}

static Case makeCases(int idx)
{
    Case c;
    switch (idx) {
        case 0:  return make_inv("1k/10k",       1e3,   10e3,  0.100);
        case 1:  return make_inv("1k/100k",      1e3,   100e3, 0.010);
        case 2:  return make_inv("100k/1M",      100e3, 1e6,   0.100);
        case 3:  return make_noninv("1k/9k",     1e3,   9e3,   0.100);
        case 4:  return make_noninv("1k/99k",    1e3,   99e3,  0.010);
        case 5:  return make_noninv("100k/900k", 100e3, 900e3, 0.100);
        case 6:
            c.topo = "电压跟随"; c.vals = "unity";
            c.g_ideal = 1.0; c.ng = 1.0; c.rp = 0.0; c.r_ib = 0.0;
            c.vin_peak = 2.0; c.zin = 1e12;
            return c;
        case 7:
            c.topo = "电压跟随"; c.vals = "unity Rs=100k";
            c.g_ideal = 1.0; c.ng = 1.0; c.rp = 100e3; c.r_ib = 0.0;
            c.vin_peak = 2.0; c.zin = 1e12;
            return c;
        case 8:
            c.topo = "差分"; c.vals = "10k/100k matched";
            c.g_ideal = 10.0; c.ng = 11.0;
            c.rp = parallel(10e3, 100e3); c.r_ib = 100e3;
            c.vin_peak = 0.050; c.zin = 20e3;
            return c;
        default:
            c.topo = "差分"; c.vals = "10k/20k matched";
            c.g_ideal = 2.0; c.ng = 3.0;
            c.rp = parallel(10e3, 20e3); c.r_ib = 20e3;
            c.vin_peak = 0.100; c.zin = 20e3;
            return c;
    }
}

static int caseCount(void) { return 10; }

/* ----------------------------------------------------------------
 * ② 交流账
 * ---------------------------------------------------------------- */
static void part2_ac(const OpAmp *o)
{
    int i;

    hr("② 五种拓扑 × 多组电阻：增益精度、噪声增益、带宽、满功率带宽");

    printf("  运放：%s（Aol=%.0fdB=%.0f，GBW=%.2fMHz，SR=%.1fV/us）\n",
           o->name, o->Aol_dB, op_aol(o), o->GBW / 1e6, o->SR_Vus);
    printf("  注意 NG：反相放大器的信号增益是 R2/R1，但决定带宽与误差的是 NG = 1+R2/R1。\n");
    printf("  少算这个 1，高增益时误差差 1%%，带宽差 10%%。\n\n");

    printf("  %s%17s %8s %9s %8s %7s %10s %10s %10s   %s\n",
           pad("拓扑", 11), "R", "G_id", "G_real", "err_%", "NG",
           "f3dB_kHz", "fFP_kHz", "fMAX_kHz", "limiter");
    printf("  -------------------------------------------------------------------"
           "----------------------\n");
    for (i = 0; i < caseCount(); ++i) {
        Case c = makeCases(i);
        double g = gainFinite(o, &c);
        double err = 100.0 * (g - c.g_ideal) / c.g_ideal;
        double f3 = bandwidth(o, &c);
        double vpk = fabs(g) * c.vin_peak;
        double ffp = fullPowerBW(o, vpk);
        double fmax = (f3 < ffp) ? f3 : ffp;
        printf("  %s%17s %8.2f %9.4f %8.3f %7.1f %10.1f %10.1f %10.1f   %s\n",
               pad(c.topo, 11), c.vals, c.g_ideal, g, err, c.ng,
               f3 / 1e3, ffp / 1e3, fmax / 1e3,
               (ffp < f3) ? "压摆率" : "带宽");
    }

    printf("\n  看懂三件事：\n");
    printf("    · 同样是增益 10，1k/9k 和 100k/900k 的误差与带宽一模一样，\n");
    printf("      但后者的 Johnson 噪声、Ib 误差、PCB 表面漏电流、寄生耦合全变大。\n");
    printf("      电阻不是随便挑的：先定带宽和噪声，再定阻抗量级（音频 1k~10k 是甜点）。\n");
    printf("    · 增益 100 那两行 f3dB 只剩 10kHz 上下。你要放大 20kHz 的正弦？\n");
    printf("      教科书说「闭环增益 = 100」，GBW 说「你先问过我」。\n");
    printf("    · 电压跟随器带 2V 峰值时，0.5V/us 的料 fFP 只有 ~40kHz：\n");
    printf("      过了这个频率，输出正弦被拉成三角——不是削波，是压摆率限制。\n");
}

/* ----------------------------------------------------------------
 * ③ 直流账
 * ---------------------------------------------------------------- */
static void part3_dc(const OpAmp *o)
{
    int i;

    hr("③ 直流账：Vos 被放大多少、同相端那个电阻为什么必须有");

    printf("  规则：Vos 与同相端的压降都按噪声增益 NG 放大，不是按信号增益。\n");
    printf("  同相端直流电阻取 Rp = R1∥R2，让两端直流电阻平衡，偏置电流 effects 互相抵消。\n\n");

    printf("  工作点假设：把直流偏置放在输出窗口正中（LM358 用 5V 时即 1.76V），\n");
    printf("  head 列 = 离最近一轨还剩多少毫伏，负数就是这一周波形已经贴轨。\n\n");

    printf("  %s%17s %7s %12s %11s %10s %10s %9s   %s\n",
           pad("拓扑", 11), "R", "NG", "Vos*NG_mV", "IbR2_mV",
           "Rp_rec_k", "Vpk_out", "head_mV", "verdict");
    printf("  -------------------------------------------------------------------"
           "----------------------\n");
    for (i = 0; i < caseCount(); ++i) {
        Case c = makeCases(i);
        double g = gainFinite(o, &c);
        double vpk = fabs(g) * c.vin_peak;
        double vos_out = o->Vos_mV * c.ng;                 /* mV */
        double ib_out = o->Ib_nA * c.r_ib * 1e-6;          /* nA × Ω = nV → ×1e-6 = mV */
        /* 最坏情形：同相端没焊 Rp，V 单位 */
        double errp = (vos_out + ib_out) * 1e-3;
        /* 偏置放在输出窗口正中，最坏失调把中心整体推到一边，
           因此两轨的余量分别被 vpk 和 errp 吃掉，取更紧的那一侧。 */
        double head = 1e3 * ((o->vo_high - o->vo_low) * 0.5 - vpk - errp);
        const char *verdict = (head < 0.0) ? "撞轨!" : ((head < 100.0) ? "极限" : "OK");
        printf("  %s%17s %7.1f %12.2f %11.2f %10.2f %10.3f %9.1f   %s\n",
               pad(c.topo, 11), c.vals, c.ng, vos_out, ib_out,
               c.rp / 1e3, vpk, head, verdict);
    }

    printf("\n  三个必须刻进脑子的数字：\n");
    printf("    · 741 配 R2=100k：Ib·R2 = 500nA × 100k = %.0fmV。你的信号只有 10mV，\n",
           500.0 * 100e3 * 1e-6);
    printf("      偏置电流造成的偏移是信号的 5 倍——这就是「精密电路别乱用大电阻」的算术根据。\n");
    printf("    · 增益 100 的同相放大器，Vos=2mV 的 LM358 输出自带 %.0fmV 偏移。\n",
           2.0 * 100.0);
    printf("      想通这条就明白：精密前端要么选 µV 级 Vos 的料，要么把增益拆成 10×10\n");
    printf("      （NG 各 11，Vos 只被放大 11 倍，两级的失调还不完全相关）。\n");
    printf("    · 表里唯一撞轨的是那两行电压跟随器：输出 2V 峰值要占 4V 摆幅，\n");
    printf("      而 LM358 在 5V 供电下的输出窗口只有 0.02~3.5V，共 3.48V。\n");
    printf("      负半周先贴 0V 轨——这不是「单电源不够方便」，是物理上装不下。\n");
    printf("      单电源设计必须每一步都问「离最近那一轨还有几伏」，而不是只问离 V+。\n");
}

/* ----------------------------------------------------------------
 * ④ 虚短虚断体检
 * ---------------------------------------------------------------- */
static void part4_ideal_check(void)
{
    /* 这一段用 ±15V 供电的 741：Aol=1e5，Ib=500nA，vo_high=13V */
    OpAmp o = {"µA741 老前辈", 100.0, 1.0e6, 0.5, +15.0, -15.0, 13.0, -14.0,
               12.0, -15.0, 1.5, 500.0, 75.0};
    typedef struct { double vout, rbig; const char *what; } W;
    W ws[5];
    int i;

    hr("④ 虚短虚断：什么时候成立，什么时候当场崩塌");

    printf("  对象：±15V 供电的 741（Aol=1e5，Ib=500nA，输出上限 13V）。\n");
    printf("  虚短的本质：Vd = Vout / Aol。它是「小」，不是「零」。\n");
    printf("  虚断的本质：流入输入端的电流 = Ib。它是「小」，不是「零」。\n\n");

    printf("  %s%9s %12s %14s   %s\n",
           pad("工况", 26), "Vout(V)", "Vd_uV", "IbR2_mV", "verdict");
    printf("  -------------------------------------------------------------------\n");
    ws[0].vout = 0.10; ws[0].rbig = 1e3; ws[0].what = "增益10 输出0.1V R2=1k";
    ws[1].vout = 1.00; ws[1].rbig = 1e3; ws[1].what = "增益10 输出1V   R2=1k";
    ws[2].vout = 9.00; ws[2].rbig = 1e5; ws[2].what = "增益100 输出9V  R2=100k";
    ws[3].vout = 12.0; ws[3].rbig = 1e5; ws[3].what = "输出顶到 12V（贴轨）";
    ws[4].vout = 13.6; ws[4].rbig = 1e5; ws[4].what = "要求 13.6V（超 vo_high）";
    for (i = 0; i < 5; ++i) {
        double vd = ws[i].vout / op_aol(&o) * 1e6;
        double ibmv = o.Ib_nA * ws[i].rbig * 1e-6;
        const char *verdict = (ws[i].vout > o.vo_high)
                                  ? "虚短崩塌：反馈失效"
                                  : ((vd > 50.0) ? "Vd 已可比拟 Vos" : "虚短成立");
        printf("  %s%9.2f %12.2f %14.2f   %s\n",
               pad(ws[i].what, 26), ws[i].vout, vd, ibmv, verdict);
    }
    printf("\n  崩塌的四种情形（板子上最常见的四个）：\n");
    printf("    1) 输出撞轨：运放饱和，Vd 不再是 Vout/Aol 而是几伏，反馈网络失去意义。\n");
    printf("       现象：正弦顶部削平，而且你调输入电位器输出纹丝不动。\n");
    printf("    2) 共模超出输入范围：轨到轨输入的料也有禁区。有些是 N 沟道输入对，\n");
    printf("       共模必须高于负轨 1.5V 以上；现象是波形到某个电平「反转」或跳变\n");
    printf("       （crossover distortion），示波器上看像丢了一小段。\n");
    printf("    3) 输出短路或负载过重：内部限流，摆幅塌掉，芯片发热甚至热失控。\n");
    printf("    4) 带宽/压摆率不够：虚短在高频不成立——运放纠正误差需要时间，\n");
    printf("       信号变化快过它的纠正能力，V- 跟不上 V+，误差随频率线性放大。\n");
    printf("\n  还有一条阴人的：反相放大器的「虚地」不是地。\n");
    printf("    从信号源看进去，输入阻抗 = R1（因为 V- 是虚地）。\n");
    printf("    R1=1k 就意味着信号源要驱动 1k 负载；想用 R1=1M 抬高阻抗？\n");
    printf("    Ib·R1 和 Johnson 噪声立刻把你吃掉。高阻抗源（压电、电极、光电二极管）\n");
    printf("    先上一级同相跟随器或跨阻放大器，别硬扛。\n");
}

/* ----------------------------------------------------------------
 * ⑤ 翻车案例 A：增益 100 + GBW 1MHz + 20kHz
 * ---------------------------------------------------------------- */
static void part5_fail_gbw(void)
{
    OpAmp o = {"GBW=1MHz 通用料", 100.0, 1.0e6, 0.5, +15.0, -15.0, 13.0, -14.0,
               12.0, -15.0, 1.0, 200.0, 75.0};
    double r1 = 1e3, r2 = 99e3;
    Case c;
    double vin = 0.010;
    const double fs[6] = {100.0, 1e3, 5e3, 10e3, 20e3, 50e3};
    Cplx G20;
    int i, k;

    hr("⑤ 翻车案例 A：标称增益 100，20kHz 正弦实际出来多少伏？");

    c.topo = "同相"; c.vals = "1k/99k";
    c.g_ideal = 100.0; c.ng = 1.0 + r2 / r1;
    c.rp = parallel(r1, r2); c.r_ib = r2; c.vin_peak = 0.010; c.zin = 1e12;

    printf("  设计意图：同相放大 100 倍，输入 10mV 峰值 → 期望输出 1.00V 峰值 @20kHz\n");
    printf("  NG = 1 + R2/R1 = %.1f → f3dB = GBW/NG = %.2f kHz（已经低于 20kHz 了）\n\n",
           c.ng, bandwidth(&o, &c) / 1e3);

    printf("  %10s %10s %12s %12s %10s\n",
           "f_Hz", "|G|", "phase_deg", "Vout_pk_V", "pct_of_100");
    printf("  ----------------------------------------------------------\n");
    for (i = 0; i < 6; ++i) {
        Cplx G = gainAt(&o, &c, fs[i]);
        printf("  %10.0f %10.2f %12.1f %12.3f %10.1f\n",
               fs[i], cplx_abs(G), cplx_arg(G) * 180.0 / PI,
               cplx_abs(G) * vin, cplx_abs(G));
    }

    G20 = gainAt(&o, &c, 20e3);
    printf("\n  20kHz：|G| = %.2f（不是 100），相位滞后 %.1f°，输出峰值 %.3fV（不是 1.00V）\n",
           cplx_abs(G20), cplx_arg(G20) * 180.0 / PI, cplx_abs(G20) * vin);

    printf("\n  时域抽样（20kHz/10mV 峰值正弦，一个周期 12 点）：\n");
    printf("  err 按满量程（1000mV）折算，不用瞬时值当分母，否则过零点附近是天文数字：\n");
    printf("  %10s %10s %12s %12s %10s\n",
           "phase_deg", "Vin_mV", "ideal_mV", "actual_mV", "err_%FS");
    printf("  ----------------------------------------------------------\n");
    for (k = 0; k < 12; ++k) {
        double ph = (double)k * 30.0 * PI / 180.0;
        double vi = vin * 1e3 * sin(ph);
        double ideal = vi * 100.0;
        double actual = cplx_abs(G20) * vin * 1e3 * sin(ph + cplx_arg(G20));
        double e = 100.0 * (actual - ideal) / (100.0 * vin * 1e3);
        printf("  %10.0f %10.2f %12.2f %12.2f %10.1f\n",
               ph * 180.0 / PI, vi, ideal, actual, e);
    }
    printf("\n  师傅判词：\n");
    printf("    · 这不是失真，是线性缩水 + 相位滞后。THD 表上看不出来，\n");
    printf("      只有拿信号发生器一比幅度、或者看波特图才发现。\n");
    printf("    · 三条出路，按成本排：\n");
    printf("      (a) 拆两级 10 × 10：NG 各 11，f3dB ≈ 91kHz，级联后仍然 100 倍；\n");
    printf("      (b) 换高 GBW 的料：20kHz × 100 需要 GBW ≥ 2MHz，留 5 倍余量买 10MHz；\n");
    printf("      (c) 反相改同相没用（NG 一样），别指望换拓扑能救带宽。\n");
    printf("    · 铁律：GBW ≥ 5~10 × f_max × 闭环增益。5 倍是余量，不是贪婪。\n");
}

/* ----------------------------------------------------------------
 * ⑥ 翻车案例 B：跟随器带容性负载
 *   L(s) = Aol / [(1+s/ωb)(1+s/ωo)]，ωo = 1/((Ro+Rser)·CL)
 *   特征方程 s² + s(ωb+ωo) + ωb·ωo·(1+Aol) = 0
 *   ωn = √(ωb·ωo·(1+Aol))，ζ = (ωb+ωo)/(2ωn)，Q = 1/(2ζ)
 *   过冲 MP = exp(-πζ/√(1-ζ²))
 * ---------------------------------------------------------------- */
static void part6_fail_cap(void)
{
    OpAmp o = {"GBW=1MHz 通用料", 100.0, 1.0e6, 0.5, +15.0, -15.0, 13.0, -14.0,
               12.0, -15.0, 1.0, 200.0, 500.0};
    typedef struct { double cl, rser; const char *label; } Run;
    Run runs[7];
    int i;

    hr("⑥ 翻车案例 B：电压跟随器直接怼 1nF，Q 值和过冲从哪来");

    printf("  运放：%s，Aol=%.0f，GBW=%.1fMHz，开环输出电阻 Ro=%.0fΩ\n",
           o.name, op_aol(&o), o.GBW / 1e6, o.Ro_ohm);
    printf("  跟随器 β=1，环路增益直流值 = Aol。输出节点 Ro 与负载电容 CL 又造出一个极点。\n");
    printf("  这个极点一旦跑进环路带宽以内，就再抢走 90° 相位裕度 → 振铃、过冲、自激。\n\n");

    printf("  %s%10s %12s %10s %9s %10s   %s\n",
           pad("CL / 隔离电阻", 24), "fp2_kHz", "wn_kHz", "zeta", "Q",
           "MP_%", "verdict");
    printf("  -------------------------------------------------------------------\n");

    runs[0].cl = 1e-12;   runs[0].rser = 0.0;   runs[0].label = "CL=1pF 无隔离";
    runs[1].cl = 100e-12; runs[1].rser = 0.0;   runs[1].label = "CL=100pF 无隔离";
    runs[2].cl = 1e-9;    runs[2].rser = 0.0;   runs[2].label = "CL=1nF 无隔离";
    runs[3].cl = 1e-9;    runs[3].rser = 50.0;  runs[3].label = "CL=1nF + 串50R*";
    runs[4].cl = 1e-9;    runs[4].rser = 200.0; runs[4].label = "CL=1nF + 串200R*";
    runs[5].cl = 10e-9;   runs[5].rser = 200.0; runs[5].label = "CL=10nF + 串200R*";
    runs[6].cl = 100e-9;  runs[6].rser = 0.0;   runs[6].label = "CL=100nF 无隔离";

    for (i = 0; i < 7; ++i) {
        double wb = TWO_PI * op_fb(&o);
        double wo = 1.0 / ((o.Ro_ohm + runs[i].rser) * runs[i].cl);
        double wn = sqrt(wb * wo * (1.0 + op_aol(&o)));
        /* 反馈取在运放脚上时，隔离电阻 Rser 在环路里插入零点 ωz=1/(Rser·CL)，
           特征方程 s² + s[(ωb+ωo) + Aol·ωb·ωo/ωz] + ωb·ωo·(1+Aol) = 0 */
        double znum = wb + wo;
        double zeta, q, mp;
        const char *verdict;
        if (runs[i].rser > 0.0)
            znum += op_aol(&o) * wb * wo * runs[i].rser * runs[i].cl;
        zeta = znum / (2.0 * wn);
        q = 1.0 / (2.0 * zeta);
        mp = (zeta < 1.0)
                 ? exp(-PI * zeta / sqrt(1.0 - zeta * zeta)) * 100.0
                 : 0.0;
        verdict = (zeta < 0.354) ? "Q>1.41 严重过冲/自激"
                : (zeta < 0.707) ? "振铃明显" : "可接受";
        printf("  %s%10.0f %12.1f %10.3f %9.2f %10.1f   %s\n",
               pad(runs[i].label, 24), wo / TWO_PI / 1e3, wn / TWO_PI / 1e3,
               zeta, q, mp, verdict);
    }
    printf("\n  * 带串阻的行按「反馈取在运放脚上、电容挂在电阻外面」计算：\n");
    printf("    此时 Rser 与 CL 在环路里造出零点 ωz=1/(Rser·CL)，ζ 显著变大。\n");
    printf("\n  师傅判词：\n");
    printf("    · 同一个运放，CL 从 100pF 加到 1nF，Q 从「温顺」跳到「自激边缘」。\n");
    printf("      你只是换了一根更长的测试线、在输出多焊了一个 1µF 电容，电路就振了。\n");
    printf("    · 现象对照：方波响应带一圈衰减振铃；接传感器正常，接 2 米线缆就振荡——\n");
    printf("      99%% 就是这个 Ro·CL 极点。\n");
    printf("    · 四种对策（按有效性/成本）：\n");
    printf("      (a) 输出串隔离电阻（反馈取在运放脚上，电容挂电阻外面）：Rser 在环路里\n");
    printf("          插入零点 ωz=1/(Rser·CL)，把容性极点顶掉——表里两行对比看得见；\n");
    printf("      (b) 反馈电阻上并一个小电容 Cf，造一个零点对掉这个极点；\n");
    printf("      (c) 输出到地串 RC snubber（几十Ω + 几 nF）；\n");
    printf("      (d) 直接选手册写了「Cload 额定 1µF 单位增益稳定」的料。\n");
    printf("    · 别看见「轨到轨 + 满摆幅」就以为能带电容。真正的答案在\n");
    printf("      「Capacitive load drive: 1000pF」那一行。\n");
}

/* ----------------------------------------------------------------
 * ⑦ 仪表放大器：列 7×7 节点方程，高斯消元解真实增益与 CMRR
 *
 *   未知量顺序：[v1-, v2-, v3-, v3+, vo1, vo2, vo3]
 *     vo1 = A(Va - v1-)      vo2 = A(Vb - v2-)      vo3 = A(v3+ - v3-)
 *     节点 v1-: (vo1-v1-)/Rf + (v2--v1-)/Rg = 0
 *     节点 v2-: (vo2-v2-)/Rf + (v1--v2-)/Rg = 0
 *     节点 v3-: (vo1-v3-)/R3 + (vo3-v3-)/R4 = 0
 *     节点 v3+: (vo2-v3+)/R3p + (0-v3+)/R4p = 0
 * ---------------------------------------------------------------- */
static int solveLinear(int n, double m[8][9])
{
    int col, r, cc, piv;
    for (col = 0; col < n; ++col) {
        piv = col;
        for (r = col + 1; r < n; ++r)
            if (fabs(m[r][col]) > fabs(m[piv][col])) piv = r;
        if (fabs(m[piv][col]) < 1e-300) return 0;
        if (piv != col) {
            for (cc = 0; cc <= n; ++cc) {
                double t = m[col][cc];
                m[col][cc] = m[piv][cc];
                m[piv][cc] = t;
            }
        }
        for (r = 0; r < n; ++r) {
            double f;
            if (r == col) continue;
            f = m[r][col] / m[col][col];
            for (cc = col; cc <= n; ++cc) m[r][cc] -= f * m[col][cc];
        }
    }
    for (r = 0; r < n; ++r) {
        m[r][n] /= m[r][r];
        m[r][r] = 1.0;
    }
    return 1;
}

/* 返回 vo3 / (Va - Vb)；共模输入 com 单独给定 */
static double inampGain(double A, double Rf, double Rg, double R3, double R4,
                        double R3p, double R4p, double va, double vb, double *vo3_out)
{
    double m[8][9];
    memset(m, 0, sizeof(m));
    /* 行 0：A·v1- + vo1 = A·Va */
    m[0][0] = A;  m[0][4] = 1.0;                     m[0][7] = A * va;
    /* 行 1：A·v2- + vo2 = A·Vb */
    m[1][1] = A;  m[1][5] = 1.0;                     m[1][7] = A * vb;
    /* 行 2：A·v3- - A·v3+ + vo3 = 0 */
    m[2][2] = A;  m[2][3] = -A;  m[2][6] = 1.0;      m[2][7] = 0.0;
    /* 行 3：节点 v1- */
    m[3][0] = -(1.0 / Rf + 1.0 / Rg);  m[3][1] = 1.0 / Rg;  m[3][4] = 1.0 / Rf;
    /* 行 4：节点 v2- */
    m[4][1] = -(1.0 / Rf + 1.0 / Rg);  m[4][0] = 1.0 / Rg;  m[4][5] = 1.0 / Rf;
    /* 行 5：节点 v3- */
    m[5][2] = -(1.0 / R3 + 1.0 / R4);  m[5][4] = 1.0 / R3;  m[5][6] = 1.0 / R4;
    /* 行 6：节点 v3+ */
    m[6][3] = -(1.0 / R3p + 1.0 / R4p); m[6][5] = 1.0 / R3p;
    if (!solveLinear(7, m)) return 0.0;
    if (vo3_out) *vo3_out = m[6][7];
    return (va - vb != 0.0) ? m[6][7] / (va - vb) : 0.0;
}

static void part7_inamp(void)
{
    const double Rf = 25e3, Rg = 25e3, R3 = 10e3, R4 = 10e3;
    const double A = 1e5;                       /* Aol = 100dB */
    const double vd_in = 0.001;                 /* 1mV 差模 */
    double vo3 = 0.0;
    double g_ideal = 1.0 + 2.0 * Rf / Rg;
    const double As[3] = {1e4, 1e5, 1e6};
    const double tols[6] = {0.0, 0.001, 0.005, 0.01, 0.02, 0.05};
    int i;

    hr("⑦ 仪表放大器：解方程组算真实增益，看电阻失配怎么吃掉 CMRR");

    printf("  电路：U1/U2 输入级（Rf=%0.0fk 各一只，Rg=%0.0fk 跨在两反相端之间），\n",
           Rf / 1e3, Rg / 1e3);
    printf("        U3 差分级（R3=R4=%0.0fk，理想时单位增益差动输出）。\n", R3 / 1e3);
    printf("  理想公式：G = (1 + 2Rf/Rg) · (R4/R3) = %.1f\n", 1.0 + 2.0 * Rf / Rg);
    printf("  现在不套公式，把 7 个未知量的节点方程列出来高斯消元。\n\n");

    printf("  %s%14s %14s %12s %12s\n",
           pad("工况", 30), "G_diff", "误差_ppm", "Vo3_uV", "备注");
    printf("  -------------------------------------------------------------------\n");

    /* 理想电阻 + 有限 Aol，纯差模；Aol 扫 80/100/120dB 看误差怎么缩
       注意：方程组按 vo=A(V+−V−) 列，输出与 (Va−Vb) 反相，所以比误差用 |G| */
    for (i = 0; i < 3; ++i) {
        double gi = fabs(inampGain(As[i], Rf, Rg, R3, R4, R3, R4,
                                   vd_in / 2.0, -vd_in / 2.0, &vo3));
        char lab[40], note[40];
        snprintf(lab, sizeof(lab), "匹配电阻，Aol=%.0fdB", 20.0 * log10(As[i]));
        snprintf(note, sizeof(note), "第一级环路增益≈%.0f", 0.5 * As[i] / g_ideal);
        printf("  %s%14.6f %14.1f %12.2f   %s\n", pad(lab, 30), gi,
               1e6 * (gi - g_ideal) / g_ideal, vo3 * 1e6, note);
    }

    printf("\n  现在只动差分级的电阻——这才是仪表放大器的真正命门：\n");
    printf("  （R3、R4 配对失配 tol，R3'=R3(1+tol)，R4'=R4(1-tol)，最坏情况）\n\n");
    printf("  %s%14s %14s %14s %14s\n",
           pad("失配 tol", 30), "G_diff", "G_cm", "CMRR_dB", "1V共模漏mV");
    printf("  -------------------------------------------------------------------\n");
    for (i = 0; i < 6; ++i) {
        double tol = tols[i];
        double r3p = R3 * (1.0 + tol), r4p = R4 * (1.0 - tol);
        double v1 = 0.0, v2 = 0.0;
        /* 差模响应 */
        double gd = inampGain(A, Rf, Rg, R3, R4, r3p, r4p, vd_in / 2.0, -vd_in / 2.0, &v1);
        /* 共模响应：两端同幅同相输入，理想时输出为 0 */
        double vcm = 1.0;
        double gcm, cmrr;
        char lab[40];
        inampGain(A, Rf, Rg, R3, R4, r3p, r4p, vcm, vcm, &v2);
        gcm = v2 / vcm;
        cmrr = (fabs(gcm) > 1e-15) ? 20.0 * log10(fabs(gd / gcm)) : 200.0;
        snprintf(lab, sizeof(lab), "tol=%.2f%% (%s)", tol * 100.0,
                 (tol == 0.0) ? "理想" : (tol <= 0.001 ? "0.1% 精密" :
                  (tol <= 0.005 ? "0.5%" : (tol <= 0.01 ? "1% 普通" : "2~5% 垃圾"))));
        printf("  %s%14.6f %14.6f %14.1f %14.3f\n",
               pad(lab, 30), gd, gcm, cmrr, fabs(gcm) * 1e3);
    }

    printf("\n  师傅判词：\n");
    printf("    · 有限 Aol 只贡献几十 ppm，而且 Aol 每 +20dB 就降一个数量级——\n");
    printf("      这不是主要矛盾。真正的主要矛盾在上一张表的下半段。\n");
    printf("    · 共模增益近似就是失配率：gcm ≈ tol，CMRR ≈ 20·log10(G/tol)。\n");
    printf("      1%% 的电阻失配 → CMRR 只剩约 50dB → 1V 共模漏进输出 10mV。\n");
    printf("      你的热电偶信号是 1mV，共模是 220V 整流环境的共模干扰。\n");
    printf("      增益算得再准，CMRR 不够就是白干。\n");
    printf("    · 所以仪表放大器要么买单片（AD620/INA333，激光修调的电阻网络在片内，\n");
    printf("      匹配到 0.02%%，CMRR 100dB 起），要么用 0.1%% 配对电阻 + 同批次同温度。\n");
    printf("      分立搭 INA 是「我会搭电路」的练习，不是产品方案。\n");
    printf("    · 还有一条：第一级的噪声增益 = 1 + 2Rf/Rg，它决定带宽和噪声放大。\n");
    printf("      Rg 越小 → 增益越高 → 环路增益越低 → 带宽和高频 CMRR 一起塌。\n");
    printf("      手册上那条「CMRR vs Gain」的下降曲线，就是这里来的。\n");
}

int main(void)
{
    OpAmp cheap = {"LM358 万金油", 100.0, 1.1e6, 0.5, 5.0, 0.0, 3.5, 0.02,
                   3.5, 0.0, 2.0, 45.0, 50.0};
    OpAmp prec  = {"精密零漂移", 130.0, 8.0e6, 2.0, 5.0, 0.0, 4.985, 0.015,
                   4.985, 0.015, 0.0075, 0.0005, 10.0};

    setup_console_utf8();
    printf("\n");
    printf("##############################################################\n");
    printf("#  第 2 关 · 模拟电路 · 02 运放实战计算台\n");
    printf("#  虚短虚断是近似，GBW 和 SR 才是老板\n");
    printf("##############################################################\n");

    part1_models();

    part2_ac(&cheap);
    printf("\n  ---- 同一批案例，换成精密零漂移（Aol=130dB，GBW=8MHz，SR=2V/us）----\n");
    part2_ac(&prec);

    part3_dc(&cheap);
    part4_ideal_check();
    part5_fail_gbw();
    part6_fail_cap();
    part7_inamp();

    printf("\n--------------------------------------------------------------\n");
    printf("本程序小结（师傅要你会背的四步检查）：\n");
    printf("  1. 先写 NG = 1 + R2/R1。所有误差、带宽、噪声都乘它，信号增益不是它。\n");
    printf("  2. 带宽 = GBW/NG，满功率带宽 = SR/(2π·Vpeak)，取小的那个当 f_max，再留 5 倍余量。\n");
    printf("  3. 直流：Vout 偏移 ≈ Vos·NG + Ib·Rp·NG + Ib·R2，Rp 必须 = R1∥R2；\n");
    printf("     然后拿 Vout_peak + 偏移 去和电源轨比，离轨不足 0.1V 就是撞轨预备役。\n");
    printf("  4. 输出接了什么？>1nF 容性或 <1kΩ 感性，立刻查 Q 值和相位裕度，\n");
    printf("     别等示波器上出现振铃才想起来问师傅。\n");
    printf("--------------------------------------------------------------\n");
    return 0;
}
