/* =====================================================================
 * 02_impedance_crosstalk.c —— 特征阻抗、时延与串扰计算器
 *
 * 【这个文件讲什么】
 *   一根走线在低频下是"导线"，在高频下是"传输线"：信号沿它跑的时候，
 *   每一小段都有自己的电压和电流，二者之比就是特征阻抗 Z0。
 *   Z0 算不对 → 反射 → 过冲/回沟 → ADC 采样错、复位误触发；
 *   两根线挨太近 → 能量耦合 → 串扰 → 复位线被时钟"捎话"，板子随机重启。
 *   本程序把这些变成可算的数字：
 *     1) 微带线特征阻抗（Hammerstad 近似，含铜厚修正）
 *     2) 带状线特征阻抗（两条参考地平面之间）
 *     3) 单位长度时延（ps/inch ≈ 85·√εr_eff）与"何时必须当传输线处理"
 *        （走线总延时 > 上升时间的 1/6）
 *     4) 串扰：耦合系数 k = 8·e^(-πS/2H)（弱耦合近似），
 *        近端 V_ne/V = (k/2)(1-e^(-2TD/Tr))，远端 V_fe/V = k·(TD/Tr)·e^(-TD/Tr)
 *        ——用它可以量化 3W 规则到底把串扰压低了多少 dB。
 *
 * 【怎么编译运行】（MSYS2 / MinGW gcc 16.2，Windows，纯 C17）
 *   gcc -std=c17 -O2 -Wall -Wextra 02_impedance_crosstalk.c -o 02_impedance_crosstalk.exe
 *   .\02_impedance_crosstalk.exe
 *
 * 【运行前先看这里】
 *   控制台是中文，运行前务必先执行：chcp 65001
 *   否则 Windows 默认 GBK 代码页会把 UTF-8 的中文输出全打成乱码。
 *
 * 【预期看到什么】
 *   - 微带线 Z0 表：1.6mm FR4 做 50Ω 需要约 3.0 mm 线宽（板材 εr 不同会漂），
 *     0.8mm 板材约 1.5 mm，4 层板 0.21mm 介质约 0.34 mm —— 这就是
 *     "同样的 50Ω，板材越薄线越细"的来历
 *   - 带状线 Z0 表与"信号夹在两个平面之间 Z0 天生偏低"的直观感受
 *   - 50Ω 单端 / USB 90Ω 差分（线宽 + 间距）反解结果
 *   - 临界长度表：Tr=1ns 时约 3cm 就要当传输线；Tr=100ps 时 3mm 就算
 *   - 两个 100MHz 信号（Tr≈1ns）在 4 层板上平行 10mm、间距 0.2mm：
 *     串扰约 -27dB（约 150mV@3.3V 摆幅）；拉开到 3W 间距再降 ~20dB
 *
 * 【老师傅点拨】
 *   1. 先问"参考平面在哪一层"，再谈阻抗。参考层不完整/参考成电源层，
 *      公式算得再准都是自欺欺人——实际 Z0 会乱跳。
 *   2. εr 不是一个常数：FR4 在 100MHz~1GHz 大约 4.2~4.5，还要看玻纤布号
 *      （1080/2116/7628 的树脂含量不同）。跟叠层厂家要数据，别百度。
 *   3. 3W 规则（中心间距 ≥ 3 倍线宽）不是玄学：串扰按 e^(-πS/2H) 指数衰减，
 *      间距每多 1W，串扰大约再降 13~20 dB，看板材厚度。
 *   4. 相邻层平行走线是隐形炸弹：两层都参考同一地平面时，垂直间距 0.2mm
 *      的耦合比同层 3W 还强。正交走线就是为了切断这条耦合路径。
 *   5. 本程序的公式是"教学 + 估"级别。量产阻抗要交给叠层厂商用场 Solver
 *      出报告，你在 EDA 里设的阻抗规则只是给他们的意图说明。
 * ===================================================================== */

#include <math.h>
#include <stdio.h>

#define PI 3.14159265358979323846
#define C_LIGHT_MM_PER_PS 0.299792458  /* 光在真空中 0.3 mm/ps */
#define PS_PER_INCH_VAC 84.672         /* 25.4mm / 0.2998 mm/ps */

#define NMICRO_W 10
#define NSTRIP_W 7
#define NB 4
#define NCASE 3
#define NJOB 3
#define NTR 5
#define NPAR 5

static void section(const char *text) {
    printf("\n");
    printf("========================================================================\n");
    printf("%s\n", text);
    printf("========================================================================\n");
}

static void sub(const char *text) {
    printf("\n--- %s ---\n", text);
}

/* ---------------------------------------------------------------------
 * 微带线：Hammerstad & Jørum 近似（注意分母用的是有效介电常数 √εeff，不是 √εr）
 *   窄线 (W/H <= 1)：Z0 = (60/√εeff) · ln(8H/W + W/4H)
 *   宽线 (W/H >  1)：Z0 = (120π/√εeff) / [W/H + 1.393 + 0.667·ln(W/H + 1.444)]
 *   铜厚修正：有效线宽 We = W + (t/π)·(1 + ln(4π·max(1, W/H)))
 * 有效介电常数：
 *   εeff = (εr+1)/2 + (εr-1)/2 · (1 + 12H/W)^(-1/2)
 * 用 εr 而不是 εeff 是常见实现错误，会把 50Ω 线宽算小 20~25%。
 * ------------------------------------------------------------------- */
static double microEpsEff(double wMm, double hMm, double er) {
    const double u = wMm / hMm;
    return (er + 1.0) / 2.0 + (er - 1.0) / 2.0 / sqrt(1.0 + 12.0 / u);
}

static double microZ0(double wMm, double hMm, double er, double tMm) {
    double u, root;
    const double we = wMm + (tMm / PI) * (1.0 + log(4.0 * PI * fmax(1.0, wMm / hMm)));
    u = we / hMm;
    root = sqrt(microEpsEff(we, hMm, er));
    if (u <= 1.0) {
        return (60.0 / root) * log(8.0 / u + u / 4.0);
    }
    return (120.0 * PI / root) / (u + 1.393 + 0.667 * log(u + 1.444));
}

/* ---------------------------------------------------------------------
 * 带状线（信号在两个参考平面正中间，双线对称结构）：
 *   Z0 ≈ (59.95/√εr) · ln[ 4b / (π·(W + 0.393·t)) ]
 *   b = 两个参考平面之间的介质总厚度；εeff = εr（完全埋在介质里）
 * 适用 b/(W+t) > 0.5 的常规几何；更精确要场求解器。
 * ------------------------------------------------------------------- */
static double stripZ0(double wMm, double bMm, double er, double tMm) {
    const double we = wMm + 0.393 * tMm;
    return (59.95 / sqrt(er)) * log(4.0 * bMm / (PI * we));
}

/* 单位长度时延 */
static double psPerInch(double epsEff) { return PS_PER_INCH_VAC * sqrt(epsEff); }
static double psPerMm(double epsEff) { return psPerInch(epsEff) / 25.4; }

/* ---------------------------------------------------------------------
 * 一维二分求解（在 [lo,hi] 上解 f(x)=target，要求单调）
 *   C 里没有 std::function，用"函数指针 + 上下文指针"实现同样的可调用对象。
 * ------------------------------------------------------------------- */
typedef struct {
    double h;   /* 介质厚 mm */
    double er;  /* 相对介电常数 */
    double t;   /* 铜厚 mm */
} MicroJob;

typedef double (*SolverFn)(double x, const void *ctx);

static double microZ0Fn(double x, const void *ctx) {
    const MicroJob *j = (const MicroJob *)ctx;
    return microZ0(x, j->h, j->er, j->t);
}

static double bisect(SolverFn f, const void *ctx, double target, double lo, double hi) {
    double flo = f(lo, ctx) - target;
    int iter;
    for (iter = 0; iter < 200; ++iter) {
        const double mid = 0.5 * (lo + hi);
        const double fm = f(mid, ctx) - target;
        if (flo * fm <= 0.0) {
            hi = mid;
        } else {
            lo = mid;
            flo = fm;
        }
    }
    return 0.5 * (lo + hi);
}

/* ---------------------------------------------------------------------
 * 串扰模型（Bogatin《Signal and Power Integrity - Simplified》教学模型）
 *   耦合系数（微带弱耦合近似，S/H >= 0.7 时较准）：
 *     k = 8 · exp(-π·S / (2H))，S = 线间净距，H = 到参考平面介质厚度
 *   平行长度 l 的耦合延迟 TD = l · psPerMm(epsEff)
 *   近端（NEXT，方向朝源）：V_ne/V = (k/2)·(1 - e^(-2·TD/Tr))
 *   远端（FEXT，方向朝负载）：V_fe/V = k·(TD/Tr)·e^(-TD/Tr)
 * ------------------------------------------------------------------- */
static double couplingK(double sMm, double hMm, int *outValid) {
    const double raw = 8.0 * exp(-PI * sMm / (2.0 * hMm));
    if (outValid) *outValid = (sMm / hMm >= 0.7) && (raw <= 0.85);
    return fmin(0.85, raw);
}

static double dB(double ratio) { return 20.0 * log10(ratio); }

/* ---------------------------------------------------------------------
 * 1. 微带线 Z0 扫描表
 * ------------------------------------------------------------------- */
static void microstripTables(void) {
    static const double wList[NMICRO_W] = {0.15, 0.20, 0.25, 0.30, 0.40, 0.60, 1.00, 1.50, 2.00, 3.00};
    typedef struct { const char *name; double h; double er; } Case;
    static const Case cases[NCASE] = {
        {"2 层板 1.6mm（顶层参考底层大地）", 1.60, 4.4},
        {"2 层板 0.8mm", 0.80, 4.4},
        {"4 层板外层介质 0.2134mm（7628 PP）", 0.2134, 4.4},
    };
    const double t = 0.035;  /* 1 oz 铜 */
    int i, j;

    sub("微带线 Z0（Ω）：线宽 W 扫描，铜厚 35um（1 oz）");
    for (i = 0; i < NCASE; ++i) {
        const Case *c = &cases[i];
        printf("\n  %s，εr=%.1f，介质厚 H=%.2fmm\n", c->name, c->er, c->h);
        printf("  W(mm) ");
        for (j = 0; j < NMICRO_W; ++j) printf(" %6.2f", wList[j]);
        printf("\n  Z0    ");
        for (j = 0; j < NMICRO_W; ++j) printf(" %6.1f", microZ0(wList[j], c->h, c->er, t));
        printf("\n  εeff  ");
        for (j = 0; j < NMICRO_W; ++j) printf(" %6.2f", microEpsEff(wList[j], c->h, c->er));
        printf("\n");
    }
    printf("\n  读表：同样 50Ω，H 从 1.6mm 缩到 0.21mm，线宽从 ~3mm 缩到 ~0.34mm。\n");
    printf("  所以 4 层板能做到细线控阻抗，2 层板画 0.3mm 线其实是 ~127Ω 的高速天线。\n");
}

/* ---------------------------------------------------------------------
 * 2. 带状线 Z0 扫描表
 * ------------------------------------------------------------------- */
static void striplineTables(void) {
    static const double wList[NSTRIP_W] = {0.10, 0.15, 0.20, 0.30, 0.40, 0.60, 0.80};
    static const double bList[NB] = {0.40, 0.60, 1.00, 1.60};
    const double er = 4.4;
    const double t = 0.035;
    int i, j;

    sub("带状线 Z0（Ω）：两参考平面间距 b × 线宽 W，εr=4.4，铜厚 35um");
    printf("  b\\W    ");
    for (j = 0; j < NSTRIP_W; ++j) printf(" %6.2f", wList[j]);
    printf("\n");
    for (i = 0; i < NB; ++i) {
        const double b = bList[i];
        printf("  %5.2f   ", b);
        for (j = 0; j < NSTRIP_W; ++j) {
            const double z = stripZ0(wList[j], b, er, t);
            if (z <= 0.0) printf("  ---- ");  /* 几何已超出近似公式范围 */
            else printf(" %6.1f", z);
        }
        printf("\n");
    }
    printf("\n  注意 ---- 表示介质太薄/线太宽：带状线 Z0 天生比微带低，\n");
    printf("  介质 <0.4mm 时常规线宽根本做不出 50Ω——要么加厚度要么接受低阻。\n");
    printf("  另外：把信号参考到电源层，公式不变但电源层的噪声直接调制成共模——\n");
    printf("  这就是\"信号层别夹在两个电源层中间\"的第一条理由。\n");
}

/* ---------------------------------------------------------------------
 * 3. 反解：50Ω 单端、90Ω 差分
 * ------------------------------------------------------------------- */
static void solveImpedance(void) {
    const double t = 0.035;
    typedef struct { const char *name; double h; double er; } Job;
    static const Job jobs[NJOB] = {
        {"1.6mm FR4（εr=4.4）", 1.60, 4.4},
        {"0.8mm FR4（εr=4.4）", 0.80, 4.4},
        {"1J01A 外层：介质 0.2134mm（7628 PP，εr≈4.6），1 oz 铜", 0.2134, 4.6},
    };
    double solved[NJOB];
    int i;

    sub("反解 50Ω 微带线宽");
    for (i = 0; i < NJOB; ++i) {
        MicroJob ctx;
        double w50;
        ctx.h = jobs[i].h;
        ctx.er = jobs[i].er;
        ctx.t = t;
        w50 = bisect(microZ0Fn, &ctx, 50.0, 0.02, 20.0);
        solved[i] = w50;
        printf("  %-40s ：W = %.3f mm（%.1f mil），εeff=%.2f，时延 %.0f ps/inch\n",
               jobs[i].name, w50, w50 * 40.0, microEpsEff(w50, ctx.h, ctx.er),
               psPerInch(microEpsEff(w50, ctx.h, ctx.er)));
    }
    printf("  量级自查：1.6mm 板材 %.1f mm（期望 ~3mm 级）、0.8mm 板材 %.1f mm（期望 ~1.5mm 级）、\n",
           solved[0], solved[1]);
    printf("  4 层板 %.2f mm —— 若与期望差一个数量级，说明公式实现错了，别急着信输出。\n",
           solved[2]);

    sub("反解 USB HS 90Ω 差分（外层微带，先让单端≈48Ω，再拉开间距凑 90Ω）");
    printf("  注意：Zdiff = 2·Z0·(1-k)。若先把单端凑到 45Ω，k 必须为 0 才凑出 90Ω，\n");
    printf("  间距无解——所以单端要故意留高一点（≈48Ω），再靠耦合把差分阻抗拉下来。\n");
    for (i = 0; i < NJOB; ++i) {
        MicroJob ctx;
        double w, z0, kNeed, s;
        if (jobs[i].h > 0.5) continue;  /* 厚介质上差分对没意义 */
        ctx.h = jobs[i].h;
        ctx.er = jobs[i].er;
        ctx.t = t;
        w = bisect(microZ0Fn, &ctx, 48.0, 0.02, 5.0);
        z0 = microZ0(w, ctx.h, ctx.er, ctx.t);
        /* Zdiff = 2·Z0·(1-k)，解 k = 1 - Zdiff/(2·Z0)，再由 k=8e^(-πS/2H) 反解 S */
        kNeed = 1.0 - 90.0 / (2.0 * z0);
        if (kNeed <= 0.0 || kNeed >= 1.0) {
            printf("  %-40s ：单端 %.1fΩ，凑不出 90Ω 差分（单端偏高，先调 W）\n", jobs[i].name, z0);
            continue;
        }
        s = (2.0 * ctx.h / PI) * log(8.0 / kNeed);
        printf("  %-40s ：W=%.3f mm（单端 %.1fΩ），净距 S=%.3f mm（中心距 %.3f mm ≈ %.1f 倍线宽）\n",
               jobs[i].name, w, z0, s, s + w, (s + w) / w);
    }
    printf("  这就是 USB 差分对\"等长、等距、底下地完整、别跨分割\"里\"别跨分割\"的分量：\n");
    printf("  跨一次分割，上面所有数字作废。\n");
}

/* ---------------------------------------------------------------------
 * 4. 时延与临界长度
 * ------------------------------------------------------------------- */
static void delayAndCritical(void) {
    static const double trList[NTR] = {5.0, 1.0, 0.5, 0.2, 0.1};
    const double er = 4.4;
    const double dMicro = psPerMm(microEpsEff(0.30, 0.21, er));  /* 4 层板外层 */
    const double dStrip = psPerMm(er);                           /* 带状线 εeff=εr */
    int i;

    sub("单位长度时延与临界长度（总延时 > Tr/6 就必须当传输线处理）");
    printf("  4 层板外层微带：约 %.0f ps/inch（%.3f ps/mm）\n",
           psPerInch(microEpsEff(0.30, 0.21, er)), dMicro);
    printf("  带状线：εeff=εr（场全在介质里），约 %.0f ps/inch（%.3f ps/mm），比微带慢\n",
           psPerInch(er), dStrip);
    printf("  记法：FR4 上信号大约 6 mil ≈ 1 ps，1 cm ≈ 60 ps（外层）/ 70 ps（内层），1 ns ≈ 14~17 cm。\n\n");
    printf("  上升时间   临界长度(外层)   判断\n");
    for (i = 0; i < NTR; ++i) {
        const double trNs = trList[i];
        const double lCm = trNs * 1000.0 / 6.0 / dMicro / 10.0;  /* cm */
        const char *verdict = (lCm > 10.0) ? "普通走线，注意拓扑即可"
                              : (lCm > 2.0) ? "板级长度就要考虑端接"
                                            : "超过几厘米就反射满屏，必须端接+控阻抗";
        printf("  %5.1f ns      %6.1f cm       %s\n", trNs, lCm, verdict);
    }
    printf("  注意分界不是\"信号频率\"而是\"上升时间\"：一颗 12MHz 的复位芯片\n");
    printf("  如果 Tr=2ns，临界长度也只有 5cm——慢信号不等于慢边沿。\n");
}

/* ---------------------------------------------------------------------
 * 5. 串扰
 * ------------------------------------------------------------------- */
/* 原来 C++ 版里的 report lambda，这里改成显式传参的普通函数 */
static void report(double sMm, double lMm, double trS, double h, double dmm) {
    int valid = 0;
    const double k = couplingK(sMm, h, &valid);
    const double td = lMm * dmm * 1e-12;
    const double x = td / trS;
    const double vne = 0.5 * k * (1.0 - exp(-2.0 * x));
    const double vfe = k * x * exp(-x);
    const double swing = 3.3;
    printf("  S=%4.2fmm l=%5.1fmm Tr=%4.0fps | k=%5.3f%s  NEXT %6.1f dB（%5.1f%%）  FEXT %6.1f dB（%5.1f%%）",
           sMm, lMm, trS * 1e12, k, valid ? "    " : " 超范围",
           dB(vne), vne * 100.0, dB(vfe), vfe * 100.0);
    printf("   NEXT≈%.0f mV@3.3V\n", vne * swing * 1e3);
}

static void crosstalk(void) {
    const double h = 0.2134;  /* 4 层板外层介质（7628 PP） */
    const double er = 4.4;
    const double epsEff = microEpsEff(0.30, h, er);
    const double dmm = psPerMm(epsEff);
    const double tr = 1.0e-9;  /* 100MHz 类信号典型 Tr≈1ns */
    const double len = 10.0;   /* 平行长度 mm */
    static const double lenList[NPAR] = {2.0, 5.0, 10.0, 20.0, 40.0};
    int v1 = 0, v2 = 0, v3 = 0, i;
    double k1, k3, k4, kv;

    sub("场景题：两个 100MHz 信号（Tr=1ns），4 层板外层（H=0.21mm），平行 10mm");
    printf("  线宽 W=0.30mm。间距 0.2mm 就是中心距 2W，也就是\"没守 3W\"的样子：\n");
    report(0.20, len, tr, h, dmm);
    report(0.30, len, tr, h, dmm);
    report(0.50, len, tr, h, dmm);
    report(0.60, len, tr, h, dmm);   /* 中心距 3W */
    report(0.90, len, tr, h, dmm);   /* 中心距 4W */

    sub("3W 规则值多少 dB？（同长度同 Tr，只看间距的指数项）");
    k1 = couplingK(0.30, h, &v1);   /* 中心距 2W（净距 1W） */
    k3 = couplingK(0.60, h, &v3);   /* 中心距 3W（净距 2W） */
    k4 = couplingK(0.90, h, &v3);   /* 中心距 4W */
    printf("  中心距 2W → 3W：串扰降低 %.1f dB\n", dB(k3 / k1));
    printf("  中心距 3W → 4W：再降 %.1f dB\n", dB(k4 / k3));
    printf("  理论通式：ΔdB = -8.686·π·ΔS/(2H)，与线宽无关，只跟\"间距增量/介质厚\"有关。\n");
    printf("  介质越薄（4 层板），单位间距换来的 dB 越多——薄介质板\"拉开一点就有效\"。\n");

    sub("平行长度的影响（S=0.6mm，即 3W 间距）");
    printf("  NEXT 饱和长度 = vp·Tr ≈ %.0f mm；短于它，NEXT 近似∝长度\n",
           tr * 1e12 / dmm);
    for (i = 0; i < NPAR; ++i) report(0.60, lenList[i], tr, h, dmm);
    printf("  FEXT 在 TD=Tr（%.0f mm）附近最大，之后被自身延迟衰减拖住；\n",
           tr * 1e12 / dmm);
    printf("  结论：长线即使按 3W 拉开，平行段越长 FXT 仍然越吃——能短则短。\n");

    sub("相邻层平行走线（垂直净距 0.42mm 两片 PP 之间，等效 S≈0.42mm、H 同参考面）");
    kv = couplingK(0.42, h, &v2);
    printf("  等效耦合系数 k≈%.3f，与同层 S=0.42mm 同量级——\n", kv);
    printf("  两层平行 10mm ≈ 同层间距 0.42mm 的串扰。相邻信号层务必正交走线。\n");
}

int main(void) {
    section("02 特征阻抗、时延与串扰计算器（Hammerstad / Bogatin 教学模型）");
    printf("微带线=表面走线+下方一个参考平面；带状线=埋在两个参考平面之间。\n");
    printf("所有公式都需要一个前提：参考平面是完整的。参考不完整，先修叠层再谈阻抗。\n");

    microstripTables();
    striplineTables();
    solveImpedance();
    delayAndCritical();
    crosstalk();

    section("老师傅收口");
    printf("1) 阻抗三要素：线宽、介质厚度、εr。三者都是公差源，量产留 ±10%% 设计余量。\n");
    printf("2) 50Ω 不是目的，\"连续\"才是。突变点比绝对值更伤人：过孔 stub、\n");
    printf("   参考层切换、线宽骤变，都是反射点。\n");
    printf("3) 串扰预算从\"间距、平行长度、参考层\"三头挤：3W 保底、平行段切短、\n");
    printf("   差分对内等距（别在差分对中间掏空铜）。\n");
    printf("4) 复位、中断、I2C 这些\"慢线\"最容易被高速线欺负——布线时给它们让路，\n");
    printf("   别和时钟平行，必要时串 100Ω 或套地保护。\n");
    printf("5) 晶振下面禁止走线、晶振包地打孔，本质也是给振铃的谐波一条回家的近路。\n");
    printf("\n");
    return 0;
}
