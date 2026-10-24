/*
 * ============================================================================
 * 01_scope_model.c —— 示波器整条采集链路建模（第 6 关 · 望闻问切）
 * ============================================================================
 *
 * 【这个文件讲什么】
 *   把一台数字示波器拆成一条链路来建模（纯 C17，无第三方库）：
 *     信号(基波+谐波+噪声) → 探头负载效应(复数分压) → 带宽(一阶低通)
 *     → 采样率(最近邻采样) → 存储深度(被迫降采样) → 垂直位数(量化+ENOB)
 *   然后用三个"事故现场"演示仪器是怎么骗你的：
 *     (a) 采样率不足 → 混叠，看到"稳定得可疑"的假波形，
 *         假频率 f_alias = |f_signal - round(f_signal/fs)*fs|
 *     (b) 带宽不足 → 上升时间变缓，tr_total = sqrt(tr_scope^2 + tr_signal^2)
 *     (c) 垂直档位太大 → 幅度只占几个 LSB，有效位数哗哗往下掉
 *   最后附赠一个"选型建议输出器"：给频率/上升时间/幅度，
 *   输出建议带宽(>=0.35/tr 或 5 倍基波)、采样率(>=5x 带宽，实时采样)、
 *   存储深度和垂直档位。
 *
 * 【怎么编译运行】(MSYS2 gcc 16.2，Windows)
 *   先在控制台执行：chcp 65001   （否则中文输出会乱码）
 *   gcc -std=c17 -O2 -Wall -Wextra 01_scope_model.c -o 01_scope_model.exe
 *   .\01_scope_model.exe                  # 跑全部演示（固定随机种子，可复现）
 *   .\01_scope_model.exe 20e6 2e-9 0.8    # 选型查询：频率(Hz) 上升时间(s) 幅度(V)
 *
 * 【预期看到什么】
 *   - 一张 ASCII 叠加图：'·' 是真实波形，'O' 是采样点落在的位置；
 *   - 1MHz 信号用 1.1MSPS 采样：屏幕上"重现"的是 100kHz 假正弦（标红警告）；
 *   - 带宽演示：350MHz 带宽量 10ns 上升沿几乎无损，20MHz 带宽量出来 19.5ns；
 *   - 垂直档位演示：80V 满量程量 100mV 信号，理想有效位数只剩 4 位出头，
 *     几千个采样点只落在十几个电平上，波形是"台阶"不是线；
 *   - 选型表：四行常见被测信号对应的建议配置。
 *
 * 【老师傅点拨】
 *   1. 带宽和采样率是两回事：带宽管"幅值衰减多少"，采样率管"你看见的是
 *      不是这个信号"。带宽够、采样率不够，波形又稳又假，最坑。
 *   2. 探头不是导线，是电路的一部分。×1 档 100pF 挂上高阻节点，
 *      先被吃掉一半的是信号本身。
 *   3. 看到"稳定得不像话"的波形，先抬采样率再下结论；
 *      看到"过冲振铃一大片"，先换弹簧地再看。
 * ============================================================================
 */

#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* MinGW 严格模式下 M_PI 不一定可见，自己给一个 */
#define PI 3.14159265358979323846

/* ---------------------------------------------------------------------------
 * 可复现随机源：xorshift64* + Marsaglia 极坐标高斯（固定种子，逐次可复现）
 * --------------------------------------------------------------------------- */
typedef struct {
    unsigned long long s;
} Rng;

static void rngSeed(Rng *r, unsigned long long seed)
{
    r->s = seed ? seed : 88172645463325252ULL; /* 0 会让 xorshift 死掉 */
}

/* [0,1) 均匀分布 */
static double rngUnit(Rng *r)
{
    r->s ^= r->s >> 12;
    r->s ^= r->s << 25;
    r->s ^= r->s >> 27;
    unsigned long long v = (r->s * 2685821657736338717ULL) >> 11;
    return (double)v * (1.0 / 9007199254740992.0);
}

/* 均值 mu、标准差 sigma 的高斯噪声（极坐标法 Box-Muller） */
static double rngGauss(Rng *r, double mu, double sigma)
{
    double u1, u2, s;
    do {
        u1 = 2.0 * rngUnit(r) - 1.0;
        u2 = 2.0 * rngUnit(r) - 1.0;
        s = u1 * u1 + u2 * u2;
    } while (s >= 1.0 || s == 0.0);
    return mu + sigma * u1 * sqrt(-2.0 * log(s) / s);
}

/* 分配失败就直接退出：演示程序没有"没内存还硬撑"的必要 */
static void *xmalloc(size_t bytes)
{
    void *p = malloc(bytes);
    if (!p) {
        fputs("内存分配失败\n", stderr);
        exit(1);
    }
    return p;
}

/* ---------------------------------------------------------------------------
 * 被测信号源模型
 * --------------------------------------------------------------------------- */
typedef struct {
    double vth;        /* Thevenin 输出阻抗（Ω），探头一挂就分压的那个"内阻" */
    const char *desc;
} Source;

static double edge1st(double t, double tr)
{
    /* 一阶系统阶跃：0→1，tr(10%-90%) = 2.2 * tau */
    const double tau = tr / 2.2;
    if (t < 0.0) return 0.0;
    return 1.0 - exp(-t / tau);
}

/* ---------------------------------------------------------------------------
 * 示波器模型
 * --------------------------------------------------------------------------- */
typedef struct {
    double bwHz;        /* -3dB 带宽 */
    double fs;          /* 采样率 (Sa/s) */
    long memPoints;     /* 存储深度（点数） */
    int bits;           /* ADC 垂直位数 */
    double vdiv;        /* 垂直档位 V/div（满量程 = vdiv * 8div） */
    double atten;       /* 探头衰减比：1 或 10 */
    double probeC;      /* 探头输入电容（F）：x1≈100pF，x10≈10pF */
    double scopeRin;    /* 示波器输入电阻（Ω）：1e6 / 5e4 / 50 */
    double noiseV;      /* 输入 referred 噪声 RMS（V） */
    bool acCouple;      /* AC 耦合（演示用，会把直流抹掉） */
} Scope;

/* 探头 + 示波器输入在频率 f 上的输入阻抗模值（Ω）
 * x1: Rin ∥ (1/jωC)；x10: (Rprobe+Rin) ∥ C，通常 C 占主导 */
static double probeZmag(const Scope *sc, double f)
{
    const double w = 2.0 * PI * f;
    const double Zc = 1.0 / (w * sc->probeC);
    const double Rp = (sc->atten > 5.0) ? 9.0e6 : 0.0; /* x10 探头内置 9MΩ */
    const double R = Rp + sc->scopeRin;
    return (R * Zc) / sqrt(R * R + Zc * Zc);
}

/* 一阶低通（带宽）：对均匀网格信号做指数滑动滤波
 * alpha = dt / (tau + dt)，tau = 1/(2π·BW) */
static void lowpass(const double *in, size_t n, double dt, double bwHz,
                    double *out)
{
    const double tau = 1.0 / (2.0 * PI * bwHz);
    const double a = dt / (tau + dt);
    double y = (n > 0) ? in[0] : 0.0;
    for (size_t i = 0; i < n; ++i) {
        y += a * (in[i] - y);
        out[i] = y;
    }
}

typedef struct {
    double *y;      /* 采集到的点（已量化），malloc 分配，调用方负责 free */
    double fsEff;   /* 考虑存储深度后的有效采样率 */
    long n;
} Acq;

/* 完整采集链路：源(网格信号) → 探头负载 → 带宽 → 存储深度 → 采样 → 量化+噪声
 * fRef：评估探头负载效应的代表性频率（一般取信号 5 次谐波或 0.35/tr 量级） */
static Acq acquire(const double *grid, size_t nGrid, double dtGrid,
                   const Source *src, const Scope *sc, Rng *rng, double fRef)
{
    Acq a;
    /* 1) 探头负载：复数分压。×10 探头衰减+示波器 ×10 还原互相抵消，
     *    真正吃掉信号的是"输入阻抗 vs 源阻抗"的分压，所以只需乘 kLoad。 */
    const double Zp = probeZmag(sc, fRef);
    const double kLoad = Zp / (Zp + src->vth);
    double *loaded = (double *)xmalloc(nGrid * sizeof(double));
    double *bw = (double *)xmalloc(nGrid * sizeof(double));
    for (size_t i = 0; i < nGrid; ++i) loaded[i] = grid[i] * kLoad;
    /* 3) AC 耦合（抹直流） */
    if (sc->acCouple) {
        double mean = 0.0;
        for (size_t i = 0; i < nGrid; ++i) mean += loaded[i];
        mean /= (double)nGrid;
        for (size_t i = 0; i < nGrid; ++i) loaded[i] -= mean;
    }
    /* 4) 带宽低通 */
    lowpass(loaded, nGrid, dtGrid, sc->bwHz, bw);
    /* 5) 存储深度限制：窗口 = n_grid*dtGrid，可存点数 = memPoints */
    const double window = dtGrid * (double)nGrid;
    double fsEff = sc->fs;
    if ((double)sc->memPoints < sc->fs * window)
        fsEff = (double)sc->memPoints / window; /* 仪器被迫降采样 */
    const double dtAcq = 1.0 / fsEff;
    const long n = (long)(window / dtAcq);
    /* 6) 最近邻采样 + 7) 垂直量化 + 噪声 */
    const double fsFull = sc->vdiv * 8.0;                 /* 满量程电压 */
    const double lsb = fsFull / pow(2.0, (double)sc->bits);
    a.fsEff = fsEff;
    a.n = n;
    a.y = (double *)xmalloc((size_t)((n > 0) ? n : 1) * sizeof(double));
    for (long i = 0; i < n; ++i) {
        const double t = (double)i * dtAcq;
        long idx = (long)llround(t / dtGrid);
        if (idx < 0) idx = 0;
        if (idx >= (long)nGrid) idx = (long)nGrid - 1;
        const double v = bw[(size_t)idx] + rngGauss(rng, 0.0, sc->noiseV);
        a.y[(size_t)i] = round(v / lsb) * lsb; /* 量化 */
    }
    free(loaded);
    free(bw);
    return a;
}

/* ---------------------------------------------------------------------------
 * 统计工具
 * --------------------------------------------------------------------------- */
static double peakToPeak(const double *v, size_t n)
{
    double lo = v[0], hi = v[0];
    for (size_t i = 0; i < n; ++i) {
        if (v[i] < lo) lo = v[i];
        if (v[i] > hi) hi = v[i];
    }
    return hi - lo;
}

/* 上升时间：在波形自身幅度范围内取 10%/90% 阈值，找最陡一次上升跳变 */
static double riseTime10_90(const double *v, size_t n, double dt)
{
    if (n == 0) return -1.0;
    double lo = v[0], hi = v[0];
    for (size_t i = 0; i < n; ++i) {
        if (v[i] < lo) lo = v[i];
        if (v[i] > hi) hi = v[i];
    }
    const double vLo = lo + 0.1 * (hi - lo);
    const double vHi = lo + 0.9 * (hi - lo);
    double best = -1.0;
    for (size_t i = 1; i + 1 < n; ++i) {
        /* 找穿越 vLo 的上升点 */
        if (!(v[i - 1] < vLo && v[i] >= vLo)) continue;
        const double f0 = (vLo - v[i - 1]) / (v[i] - v[i - 1]);
        const double tLo = ((double)i - 1.0 + f0) * dt;
        bool found = false;
        double tHi = 0.0;
        for (size_t j = i + 1; j < n; ++j) {
            if (v[j - 1] < vLo) break; /* 又掉回去了，不算一次干净跳变 */
            if (v[j - 1] < vHi && v[j] >= vHi) {
                const double f1 = (vHi - v[j - 1]) / (v[j] - v[j - 1]);
                tHi = (double)(j - 1) * dt + f1 * dt;
                found = true;
                break;
            }
        }
        if (!found) continue;
        const double tr = tHi - tLo;
        if (best < 0.0 || tr < best) best = tr;
    }
    return best;
}

/* ---------------------------------------------------------------------------
 * ASCII 叠加图：'.' 真实波形，'=' 采集点连线（示波器屏幕上你看到的线），
 *               'O' 采样点落点
 * --------------------------------------------------------------------------- */
#define ASCII_W 76
#define ASCII_H 15

static void asciiOverlay(const double *truth, size_t nTruth, double dtGrid,
                         const Acq *a, const char *title)
{
    char g[ASCII_H][ASCII_W + 1];
    for (int r = 0; r < ASCII_H; ++r) {
        memset(g[r], ' ', (size_t)ASCII_W);
        g[r][ASCII_W] = '\0';
    }
    double lo = 1e30, hi = -1e30;
    for (size_t i = 0; i < nTruth; ++i) {
        if (truth[i] < lo) lo = truth[i];
        if (truth[i] > hi) hi = truth[i];
    }
    for (long i = 0; i < a->n; ++i) {
        if (a->y[(size_t)i] < lo) lo = a->y[(size_t)i];
        if (a->y[(size_t)i] > hi) hi = a->y[(size_t)i];
    }
    const double span = (hi - lo) < 1e-12 ? 1.0 : (hi - lo);
    /* 1) 真实波形打底：按列铺（网格远密于列数） */
    const size_t ncol = (size_t)ASCII_W;
    const size_t per = nTruth / ncol + 1;
    for (size_t c = 0; c < ncol; ++c) {
        int rmin = ASCII_H, rmax = -1;
        for (size_t k = 0; k < per; ++k) {
            size_t i = c * per + k;
            if (i >= nTruth) break;
            int r = (int)((hi - truth[i]) / span * (double)(ASCII_H - 1));
            if (r < 0) r = 0;
            if (r >= ASCII_H) r = ASCII_H - 1;
            if (r < rmin) rmin = r;
            if (r > rmax) rmax = r;
        }
        for (int r = rmin; r <= rmax; ++r) g[r][c] = '.';
    }
    const double tEnd = dtGrid * (double)nTruth;
    /* 2) 采集点连线：示波器就是把这些点用直线连起来画给你的 */
    for (long i = 1; i < a->n; ++i) {
        size_t c0 = (size_t)(((double)(i - 1) / a->fsEff) / tEnd * ASCII_W);
        size_t c1 = (size_t)(((double)i / a->fsEff) / tEnd * ASCII_W);
        if (c0 >= ncol) c0 = ncol - 1;
        if (c1 >= ncol) c1 = ncol - 1;
        int r0 = (int)((hi - a->y[(size_t)(i - 1)]) / span * (double)(ASCII_H - 1));
        int r1 = (int)((hi - a->y[(size_t)i]) / span * (double)(ASCII_H - 1));
        if (r0 < 0) r0 = 0;
        if (r0 >= ASCII_H) r0 = ASCII_H - 1;
        if (r1 < 0) r1 = 0;
        if (r1 >= ASCII_H) r1 = ASCII_H - 1;
        const long cl = (long)c0, ch = (long)c1;
        for (long c = cl; c <= ch; ++c) {
            const double f = (c1 > c0)
                ? (double)(c - (long)c0) / (double)((long)c1 - (long)c0) : 0.0;
            const long r = (long)((double)r0 + llround((double)(r1 - r0) * f));
            const long aLo = ((long)r0 < r) ? (long)r0 : r;
            const long aHi = ((long)r0 > r) ? (long)r0 : r;
            for (long rr = aLo; rr <= aHi; ++rr)
                g[(size_t)rr][(size_t)c] = '=';
        }
    }
    /* 3) 采样点盖顶 */
    for (long i = 0; i < a->n; ++i) {
        size_t c = (size_t)(((double)i / a->fsEff) / tEnd * ASCII_W);
        if (c >= ncol) c = ncol - 1;
        int r = (int)((hi - a->y[(size_t)i]) / span * (double)(ASCII_H - 1));
        if (r < 0) r = 0;
        if (r >= ASCII_H) r = ASCII_H - 1;
        g[(size_t)r][c] = 'O';
    }
    printf("\n  %s   （'.'真实波形  '='仪器画给你的线  'O'采样点  纵轴 %.2f ~ %.2f V）\n",
           title, hi, lo);
    for (int r = 0; r < ASCII_H; ++r) printf("  |%s\n", g[r]);
}

/* 采集点与真实波形的偏差 RMS（示波器"每个采样点"错多少，含量化+噪声+带宽差） */
static double rmsDeviation(const Acq *a, const double *truth, size_t nTruth,
                           double dtGrid)
{
    double s = 0.0;
    long k = 0;
    for (long i = 0; i < a->n; ++i) {
        long idx = (long)llround(((double)i / a->fsEff) / dtGrid);
        if (idx < 0) idx = 0;
        if (idx >= (long)nTruth) idx = (long)nTruth - 1;
        const double d = a->y[(size_t)i] - truth[(size_t)idx];
        s += d * d;
        ++k;
    }
    return (k > 0) ? sqrt(s / (double)k) : 0.0;
}

static int cmpDouble(const void *x, const void *y)
{
    const double a = *(const double *)x, b = *(const double *)y;
    return (a > b) - (a < b);
}

/* 波形里实际出现的量化台阶（不同电平）个数——垂直档位浪费最直观的证据 */
static long distinctLevels(const double *v, size_t n)
{
    double *s = (double *)xmalloc(n * sizeof(double));
    memcpy(s, v, n * sizeof(double));
    qsort(s, n, sizeof(double), cmpDouble);
    long cnt = 1;
    for (size_t i = 1; i < n; ++i)
        if (s[i] != s[i - 1]) ++cnt;
    free(s);
    return cnt;
}

/* ---------------------------------------------------------------------------
 * 混叠计算：f_alias = |f - round(f/fs)*fs|
 * --------------------------------------------------------------------------- */
static double aliasFreq(double f, double fs)
{
    return fabs(f - nearbyint(f / fs) * fs);
}

/* ---------------------------------------------------------------------------
 * 演示主体
 * --------------------------------------------------------------------------- */
static void demoAliasing(void)
{
    typedef struct { double fs; double window; const char *tag; } Case;
    const Case cases[3] = {
        {10.0e6, 4.0e-6,  "正常：10MSPS（fs/f = 10）"},
        {1.1e6,  20.0e-6, "欠采样：1.1MSPS（fs/f = 1.1）"},
        {2.0e6,  4.0e-6,  "刚好 2 倍：2MSPS（相位撞车）"},
    };
    printf("\n=========== 演示 (a)：采样率不足 → 混叠（假波形又稳又假）===========\n");
    const double f = 1.0e6; /* 1MHz 正弦，幅度 0.8V */
    const Source src = {50.0, "50Ω 信号源"};
    Rng rng;
    rngSeed(&rng, 6006);
    const double ph = PI / 4.0; /* 加 45° 初相：2 倍采样时幅值只剩 71%（初相一变就在 0~100% 间跳） */
    for (int ci = 0; ci < 3; ++ci) {
        const double dt = 1.0 / 50.0e9; /* 真实波形的"上帝视角"网格 */
        const size_t n = (size_t)(cases[ci].window / dt);
        double *truth = (double *)xmalloc(n * sizeof(double));
        for (size_t i = 0; i < n; ++i)
            truth[i] = 0.8 * sin(2.0 * PI * f * (double)i * dt + ph);
        const Scope sc = {1.0e9, cases[ci].fs, 100000000L, 12, 0.2, 10.0,
                          10e-12, 1e6, 2e-3, false};
        Acq a = acquire(truth, n, dt, &src, &sc, &rng, 5.0 * f);
        const double fa = aliasFreq(f, cases[ci].fs);
        printf("\n  [%s]  f_alias = |f - round(f/fs)*fs| = %.0f Hz\n",
               cases[ci].tag, fa);
        if (cases[ci].fs < 2.0 * f)
            printf("  !!! 混叠警告：fs < 2f。屏幕上重现的是 %.0f Hz 的\"稳定假波形\"，"
                   "周期 %.1f us，一点破绽都没有——这就是最坑的地方。\n",
                   fa, 1e6 / fa);
        else if (fabs(cases[ci].fs - 2.0 * f) < 1.0)
            printf("  ~ 提示：2 倍采样不混叠，但采样点每周期撞在同一相位（本例 45°），"
                   "幅值只剩 %.0f%%；初相一变，读数就在 0~100%% 之间跳。\n",
                   100.0 * sin(PI / 4.0));
        else
            printf("  ok 正常：fs/f = %.1f，重建波形与真实波形重合。\n",
                   cases[ci].fs / f);
        asciiOverlay(truth, n, dt, &a, cases[ci].tag);
        const double ppTruth = peakToPeak(truth, n);
        printf("  统计：真实峰峰 %.3f V，采集峰峰 %.3f V（%0.f%%），"
               "每个采样点相对真实值的偏差 RMS = %.1f mV\n",
               ppTruth, peakToPeak(a.y, (size_t)a.n),
               100.0 * peakToPeak(a.y, (size_t)a.n) / ppTruth,
               rmsDeviation(&a, truth, n, dt) * 1e3);
        free(a.y);
        free(truth);
    }
    printf("\n  老师傅小结：混叠产物频率是确定值，所以它\"稳定得可疑\"。"
           "2 倍采样那个案例最阴险：每个采样点都没错，错的是把点连成线的那双眼。"
           "看到新出现的正弦，先抬采样率（或用示波器的混叠警告），再谈电路。\n");
}

static void demoProbeLoad(void)
{
    typedef struct { const char *node; double rth; double f; } Row;
    const Row rows[3] = {
        {"DCDC 开关节点（低阻）", 20.0, 500e3},
        {"10MHz 时钟（中等驱动）", 150.0, 10e6},
        {"1MΩ 分压点（高阻）", 5.0e5, 1e3},
    };
    printf("\n=========== 演示 (a2)：探头是电路的一部分（负载效应）===========\n");
    for (int ri = 0; ri < 3; ++ri) {
        printf("\n  节点：%s，Thevenin %.3g Ω，代表频率 %.3g Hz\n",
               rows[ri].node, rows[ri].rth, rows[ri].f);
        const double attens[2] = {1.0, 10.0};
        for (int ai = 0; ai < 2; ++ai) {
            const double atten = attens[ai];
            const Scope sc = {500e6, 1.0e9, 10000000L, 12, 1.0, atten,
                              (atten > 5.0 ? 10e-12 : 100e-12), 1e6, 0.0, false};
            const double Zp = probeZmag(&sc, rows[ri].f);
            const double err = 100.0 * rows[ri].rth / (rows[ri].rth + Zp);
            printf("    ×%-2.0f 探头：输入阻抗 %8.3g Ω → 分压误差 %.2f%%%s\n",
                   atten, Zp, err, (err > 3.0 ? "   ← 超标！" : ""));
        }
    }
    printf("\n  老师傅小结：源阻抗越高、频率越高，×1 探头（100pF）吃得越狠。"
           "高阻节点默认 ×10，低频也要算这笔分压账。\n");
}

static void demoBandwidth(void)
{
    printf("\n=========== 演示 (b)：带宽不足 → 上升时间变缓 ===========\n");
    const double trSig = 10e-9;      /* 信号本身 10ns 上升沿（3.3V 摆幅） */
    const double dt = 1.0 / 100.0e9;
    const double window = 80e-9;
    const size_t n = (size_t)(window / dt);
    double *truth = (double *)xmalloc(n * sizeof(double));
    for (size_t i = 0; i < n; ++i)
        truth[i] = 3.3 * edge1st((double)i * dt - 10e-9, trSig);
    const Source src = {30.0, "30Ω 驱动"};
    Rng rng;
    rngSeed(&rng, 6007);
    const double bws[3] = {350e6, 100e6, 20e6};
    for (int bi = 0; bi < 3; ++bi) {
        const double bw = bws[bi];
        const Scope sc = {bw, 5.0e9, 10000000L, 12, 0.5, 10.0,
                          10e-12, 1e6, 1e-3, false};
        Acq a = acquire(truth, n, dt, &src, &sc, &rng, 0.35 / trSig);
        const double trScope = 0.35 / bw;
        /* tr_total = sqrt(tr_scope^2 + tr_signal^2)：级联系统的工程估计 */
        const double trPre = sqrt(trScope * trScope + trSig * trSig);
        const double trMeas = riseTime10_90(a.y, (size_t)a.n, 1.0 / a.fsEff);
        printf("\n  带宽 %.0f MHz：tr_scope = 0.35/BW = %.2f ns；"
               "sqrt 规则预测 tr_total = %.2f ns；实测 10-90%% = %.2f ns"
               "（sqrt 规则是工程估计，级联一阶系统实测略胖属正常）\n",
               bw / 1e6, trScope * 1e9, trPre * 1e9, trMeas * 1e9);
        printf("        峰峰 %.2f V，采样点对真实波形偏差 RMS = %.2f mV",
               peakToPeak(a.y, (size_t)a.n),
               rmsDeviation(&a, truth, n, dt) * 1e3);
        if (bw <= 100e6)
            printf("   ← 用 %.0fMHz 表子量 10ns 边沿，被量\"胖\" %.0f%%，过冲/裕量全别信",
                   bw / 1e6, 100.0 * (trPre / trSig - 1.0));
        printf("\n");
        free(a.y);
    }
    free(truth);
    printf("\n  老师傅小结：带宽决定\"边沿量得准不准\"。"
           "想反推真实 tr：tr_sig = sqrt(tr_meas^2 - tr_scope^2)，"
           "或者直接换带宽更高的表子。\n");
}

static void demoVertical(void)
{
    printf("\n=========== 演示 (c)：垂直档位太大 → 分辨率浪费 ===========\n");
    const double amp = 0.1; /* 100mV 小信号，1MHz */
    const double dt = 1.0 / 100.0e9;
    const size_t n = 500000; /* 5us 窗口 = 5 个完整 1MHz 周期 */
    double *truth = (double *)xmalloc(n * sizeof(double));
    for (size_t i = 0; i < n; ++i)
        truth[i] = amp * sin(2.0 * PI * 1.0e6 * (double)i * dt);
    const Source src = {50.0, "50Ω"};
    Rng rng;
    rngSeed(&rng, 6008);
    const double vdivs[3] = {10.0, 1.0, 0.05};
    for (int vi = 0; vi < 3; ++vi) {
        const double vdiv = vdivs[vi];
        const Scope sc = {200e6, 2.0e9, 10000000L, 12, vdiv, 10.0,
                          10e-12, 1e6, 0.0, false};
        Acq a = acquire(truth, n, dt, &src, &sc, &rng, 5.0e6);
        const double fsFull = vdiv * 8.0;
        const double lsb = fsFull / pow(2.0, (double)sc.bits);
        const double codes = 2.0 * amp / lsb; /* 峰峰占几个码 */
        const double effBits = log2(codes < 1.0 ? 1.0 : codes) + 1.0;
        const double qNoise = lsb / sqrt(12.0);
        printf("\n  档位 %.2f V/div（满量程 %.1f V）：LSB = %.2f mV，"
               "100mV 信号峰峰只占 %.1f 个码，理想有效位数 ≈ %.1f bit\n",
               vdiv, fsFull, lsb * 1e3, codes, effBits);
        const long lv = distinctLevels(a.y, (size_t)a.n);
        printf("        量化噪声理论 %.2f mV，采样点对真实值偏差 RMS = %.2f mV，"
               "%ld 个采样点里只出现 %ld 个不同电平（峰峰 %.0f mV）",
               qNoise * 1e3, rmsDeviation(&a, truth, n, dt) * 1e3, a.n, lv,
               peakToPeak(a.y, (size_t)a.n) * 1e3);
        if (vdiv >= 1.0)
            printf("   ← 波形是\"台阶\"不是线：量化误差比信号细节还大");
        printf("\n");
        free(a.y);
    }
    free(truth);
    printf("\n  老师傅小结：ENOB = (SNR - 1.76)/6.02。档位、噪声、耦合方式"
           "任何一个不对，标称 12 位也就剩个位数。先让信号占 6~8 格，再谈精度。\n");
}

static void demoMemory(void)
{
    printf("\n=========== 加餐：存储深度不够，仪器偷偷替你降采样 ===========\n");
    const double window = 100e-3;      /* 想看 100ms 上电过程 */
    const long mem = 1000000L;         /* 1M 点 */
    const double fsWant = 1.0e9;
    const double fsEff = (double)mem / window;
    printf("  窗口 %.0f ms × 想要 %.0f MSa/s → 需要 %.0f M 点，机器只有 %ld 点。\n",
           window * 1e3, fsWant / 1e6, fsWant * window / 1e6, mem);
    printf("  仪器自动把采样率压到 %.0f MSa/s：>%.0f kHz 的毛刺全部混叠/抹平。\n",
           fsEff / 1e6, fsEff / 2.0 / 1e3);
    printf("  抓单次毛刺三件套：短窗口 + 最大采样率 + 最大存储深度，缺一不可。\n");
}

/* ---------------------------------------------------------------------------
 * 选型建议输出器
 * --------------------------------------------------------------------------- */
static void advise(double fHz, double trSec, double ampV)
{
    printf("\n---- 选型建议：f=%.3g Hz, tr=%.3g s, 幅度=%.3g V ----\n",
           fHz, trSec, ampV);
    const double bwEdge = (trSec > 0.0) ? 0.35 / trSec : 0.0;
    const double bw5h = 5.0 * fHz;
    const double bwNeed = bwEdge > bw5h ? bwEdge : bw5h;
    const double ladderBw[8] = {50e6, 100e6, 200e6, 350e6, 500e6, 1e9, 2e9, 4e9};
    double bwPick = ladderBw[7];
    for (int i = 0; i < 8; ++i) {
        if (ladderBw[i] >= bwNeed) { bwPick = ladderBw[i]; break; }
    }
    printf("  带宽：max(0.35/tr, 5×f) = max(%.3g, %.3g) = %.3g Hz → 选 %.3g Hz 档\n",
           bwEdge, bw5h, bwNeed, bwPick);
    const double fsNeed = 5.0 * bwPick;
    const double ladderFs[11] = {10e6, 25e6, 50e6, 100e6, 250e6, 500e6, 1e9,
                                 2e9, 5e9, 10e9, 20e9};
    double fsPick = ladderFs[10];
    for (int i = 0; i < 11; ++i) {
        if (ladderFs[i] >= fsNeed) { fsPick = ladderFs[i]; break; }
    }
    printf("  采样率（实时）：≥5×带宽 = %.3g Sa/s → 选 %.3g Sa/s"
           "（10× 更稳；重复信号可用等效时间采样，单次事件不行）\n",
           fsNeed, fsPick);
    const double window = 5.0 / (fHz > 0.0 ? fHz : 1.0);
    const double memNeed = fsPick * window;
    const long ladderMem[7] = {1000000L, 2000000L, 4000000L, 8000000L,
                               16000000L, 32000000L, 64000000L};
    long memPick = ladderMem[6];
    for (int i = 0; i < 7; ++i) {
        if ((double)ladderMem[i] >= memNeed) { memPick = ladderMem[i]; break; }
    }
    printf("  存储深度：想看 %.3g s（约 5 个周期）× %.3g Sa/s = %.3g 点 → 选 %ld 点\n",
           window, fsPick, memNeed, memPick);
    const double ladderDiv[13] = {0.001, 0.002, 0.005, 0.01, 0.02, 0.05, 0.1,
                                  0.2, 0.5, 1.0, 2.0, 5.0, 10.0};
    double divPick = ladderDiv[12];
    for (int i = 0; i < 13; ++i) {
        if (2.0 * ampV <= 6.5 * ladderDiv[i]) { divPick = ladderDiv[i]; break; }
    }
    const double lsb = divPick * 8.0 / 4096.0;
    printf("  垂直档位：让 %.3g V 峰峰占 6~8 格 → 选 %.3g V/div，"
           "LSB≈%.2f mV（12 位 ADC）\n", 2.0 * ampV, divPick, lsb * 1e3);
    printf("  探头：高阻/高速节点默认 ×10（电容 ~10pF）；量开关节点用弹簧地；"
           "只有低频低阻大信号才考虑 ×1。\n");
}

static void demoAdvisor(void)
{
    printf("\n=========== 选型建议输出器（四个常见场景）===========\n");
    advise(8.0e6, 5e-9, 1.65);    /* 8MHz 时钟 */
    advise(100.0e6, 1e-9, 0.4);   /* 100MHz 差分时钟（单端看） */
    advise(50.0e3, 200e-9, 3.3);  /* 50kHz PWM */
    advise(1.0e3, 0.0, 0.1);      /* 1kHz 纹波小信号 */
    printf("\n  （想查自己信号：01_scope_model.exe <频率Hz> <上升时间s> <幅度V>）\n");
}

/* --------------------------------------------------------------------------- */
int main(int argc, char **argv)
{
    printf("================================================================\n");
    printf(" 示波器链路建模实验室 —— 第 6 关 · 望闻问切（软件模型，无硬件）\n");
    printf("================================================================\n");
    if (argc >= 4) {
        char *e1, *e2, *e3;
        const double f = strtod(argv[1], &e1);
        const double tr = strtod(argv[2], &e2);
        const double amp = strtod(argv[3], &e3);
        if (e1 == argv[1] || e2 == argv[2] || e3 == argv[3]) {
            fputs("参数需要是数字：频率(Hz) 上升时间(s) 幅度(V)\n", stderr);
            return 1;
        }
        advise(f, tr, amp);
        return 0;
    }
    if (argc == 2) { /* 只给频率 */
        char *e1;
        const double f = strtod(argv[1], &e1);
        if (e1 == argv[1]) {
            fputs("参数需要是数字：频率(Hz)\n", stderr);
            return 1;
        }
        advise(f, 0.0, 1.0);
        return 0;
    }
    demoAliasing();
    demoProbeLoad();
    demoBandwidth();
    demoVertical();
    demoMemory();
    demoAdvisor();
    printf("\n【老师傅收尾】带宽管幅值/边沿准不准，采样率管你看见的是不是它，\n"
           "存储深度管仪器有没有偷偷替你降采样，档位管位数用没用上，\n"
           "探头是电路的一部分。这五件事想清楚了，示波器才开始说真话。\n");
    return 0;
}
