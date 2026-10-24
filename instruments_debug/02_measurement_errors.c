/*
 * ============================================================================
 * 02_measurement_errors.c —— 万用表也会骗你：负载、位数、不确定度
 * ============================================================================
 *
 * 【这个文件讲什么】
 *   把一台数字万用表(DMM)建模出来（纯 C17，无第三方库）：输入阻抗、档位分辨率、
 *   精度规格 ±(%读数 + N 个字)、电流档负担电压。然后算四笔"误差账"：
 *     (a) 表笔/输入阻抗负载：量高阻分压点，表本身吃掉电压；
 *     (b) 3½ 位 vs 6½ 位表量同一个 2.5V 基准：分辨率和规格差 3 个数量级；
 *     (c) 多次测量统计：均值、样本标准差、仪器不确定度、合成不确定度，
 *         最后告诉你"这个数到底能报几位"；
 *     (d) 200mA 档 vs 10A 档：分辨率差 100 倍 + 负担电压，
 *         小电流量错档位 = "看不见"。
 *
 * 【怎么编译运行】(MSYS2 gcc 16.2，Windows)
 *   先在控制台执行：chcp 65001   （否则中文输出会乱码）
 *   gcc -std=c17 -O2 -Wall -Wextra 02_measurement_errors.c -o 02_measurement_errors.exe
 *   .\02_measurement_errors.exe     # 固定随机种子，结果可复现
 *
 * 【预期看到什么】
 *   - 用 10MΩ 输入阻抗的手持表量 1MΩ 电阻分压点：误差百分之几，标"超标"；
 *   - 同一个 2.5V 基准：3½ 位表在 20V 档只显示 2.50V（±130mV 规格），
 *     6½ 位台表能看见几十微伏的漂移；
 *   - 50 次重复测量的合成不确定度报告：均值后面能写的有效位数越来越少；
 *   - 6mA 电流用 10A 档(10mA 分辨率)量出来是 0.00——不是没电流，是档位吃掉了。
 *
 * 【老师傅点拨】
 *   1. 表笔一搭，表就是电路的一部分。10MΩ 输入阻抗量高阻节点，
 *      先算分压账再读数。
 *   2. 精度规格 ±(%读数+N字) 里那个"字"在低读数时才是主角：量 0.05V
 *      时 2 个字可能比 0.5% 还大。
 *   3. 测了 100 次取平均不会让系统误差变小——负载误差、校准误差，
 *      平均一万次还是错。平均值只对随机误差(噪声、重复性)有效。
 * ============================================================================
 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

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
 * 万用表模型
 * --------------------------------------------------------------------------- */
#define MAX_V_RANGES 4
#define MAX_A_RANGES 2

typedef struct {
    double fullScale;   /* 档位满量程 (V) */
    double resolution;  /* 显示分辨率 (V)，即 1 个字 */
    double pctOfRead;   /* 精度：± % 读数（1 年规格，直流） */
    int digitsOfErr;    /* 精度：± N 个字 */
} DCVRange;

typedef struct {
    double fullScale;    /* 满量程 (A) */
    double resolution;   /* 分辨率 (A) */
    double burdenOhm;    /* 该档等效串联电阻（分流器+表笔） */
    double pctOfRead;
    int digitsOfErr;
} ACurRange;

typedef struct {
    const char *name;
    double inputR;                    /* 直流电压档输入阻抗 (Ω) */
    DCVRange vRanges[MAX_V_RANGES];
    int nV;
    ACurRange aRanges[MAX_A_RANGES];
    int nA;
    double repSigma;                  /* 读数的重复性噪声 RMS（短期，相对读数的小比例） */
} DMM;

/* 常见手持 3½ 位表（如 DT830 一类）：最大显示 1999 */
static DMM make35(void)
{
    DMM d;
    d.name = "3½ 位手持表（1999 counts，10MΩ 输入）";
    d.inputR = 10.0e6;
    d.vRanges[0] = (DCVRange){2.0,    0.001,  0.5,  2};  /* 2V 档：1mV 字 */
    d.vRanges[1] = (DCVRange){20.0,   0.010,  0.8,  3};  /* 20V 档：10mV 字 */
    d.vRanges[2] = (DCVRange){200.0,  0.100,  1.0,  3};
    d.vRanges[3] = (DCVRange){1000.0, 1.0,    1.2,  5};
    d.nV = 4;
    d.aRanges[0] = (ACurRange){0.2,  0.0001, 1.1,   1.0, 3}; /* 200mA 档：0.1mA 字，负担 ~1.1Ω */
    d.aRanges[1] = (ACurRange){10.0, 0.010,  0.015, 1.0, 5}; /* 10A 档：10mA 字，负担 ~15mΩ */
    d.nA = 2;
    d.repSigma = 0.0004; /* 手写表短期重复性 ~4 个字量级 */
    return d;
}

/* 常见 6½ 位台式表（34401A 一类）：1200000 counts */
static DMM make65(void)
{
    DMM d;
    d.name = "6½ 位台式表（1200000 counts，10GΩ/10MΩ 可选）";
    d.inputR = 10.0e9;
    d.vRanges[0] = (DCVRange){1.0,    1.0e-6,   0.0015, 5}; /* 1V 档：1µV 字 */
    d.vRanges[1] = (DCVRange){10.0,   10.0e-6,  0.0015, 5}; /* 10V 档：10µV 字 */
    d.vRanges[2] = (DCVRange){100.0,  100.0e-6, 0.004,  5};
    d.nV = 3;
    d.aRanges[0] = (ACurRange){0.1, 1.0e-6, 10.0, 0.05, 3}; /* 100mA 档：1µA 字，负担 10Ω */
    d.aRanges[1] = (ACurRange){1.0, 10.0e-6, 1.0, 0.05, 3}; /* 1A 档：10µA 字，负担 1Ω */
    d.nA = 2;
    d.repSigma = 1.0e-7; /* 台表重复性低得多 */
    return d;
}

/* 选一个能装下读数且最小的直流电压档，返回下标 */
static int pickVRange(const DMM *d, double v)
{
    for (int i = 0; i < d->nV; ++i)
        if (fabs(v) <= d->vRanges[i].fullScale) return i;
    return d->nV - 1;
}

typedef struct {
    double displayed;   /* 表上显示的值（已按分辨率截断/舍入） */
    double specUnc;     /* 规格给出的最大误差绝对值：pct*|读数| + N*分辨率 */
    int range;          /* 所用直流电压档下标 */
} Reading;

/* 一次直流电压测量：真实值 → 输入阻抗负载 → 加噪声 → 按分辨率显示 */
static Reading measureV(const DMM *d, double vTrue, double rThovenin, Rng *rng)
{
    Reading out;
    const double loaded = vTrue * d->inputR / (d->inputR + rThovenin);
    const int ri = pickVRange(d, loaded);
    const DCVRange *r = &d->vRanges[ri];
    const double sigma = d->repSigma * (loaded > 1e-3 ? loaded : 1e-3);
    const double noisy = loaded + rngGauss(rng, 0.0, sigma);
    out.displayed = round(noisy / r->resolution) * r->resolution;
    out.specUnc = r->pctOfRead / 100.0 * fabs(out.displayed)
                  + (double)r->digitsOfErr * r->resolution;
    out.range = ri;
    return out;
}

/* ---------------------------------------------------------------------------
 * 演示 (a)：输入阻抗负载 —— 表笔一搭，电压就变
 * --------------------------------------------------------------------------- */
static void demoLoading(void)
{
    printf("\n=========== 演示 (a)：输入阻抗负载（高阻分压点）===========\n");
    printf("  场景：5.000V 电源上两个等值电阻分压，理论中点 2.500V。\n"
           "        中点 Thevenin 阻抗 = R1∥R2 = R/2。表一搭上去就分压。\n");
    const DMM d35 = make35(), d65 = make65();
    Rng rng;
    rngSeed(&rng, 6101);
    const double res[5] = {1.0e3, 10.0e3, 100.0e3, 1.0e6, 10.0e6};
    for (int i = 0; i < 5; ++i) {
        const double vTrue = 2.5;
        const double rth = res[i] / 2.0;
        Reading a = measureV(&d35, vTrue, rth, &rng);
        Reading b = measureV(&d65, vTrue, rth, &rng);
        const double e35 = 100.0 * (a.displayed - vTrue) / vTrue;
        const double e65 = 100.0 * (b.displayed - vTrue) / vTrue;
        printf("  R1=R2=%6.3g Ω (中点阻抗 %8.3g Ω)：3½ 表显 %.4f V（%+6.2f%%）%s"
               "  6½ 表显 %.6f V（%+.3f%%）%s\n",
               res[i], rth, a.displayed, e35,
               (fabs(e35) > 1.0 ? "[超标!]" : "        "),
               b.displayed, e65,
               (fabs(e65) > 1.0 ? "[超标!]" : ""));
    }
    printf("\n  算式就一条：误差 = Rth/(Rth+Rin)。1MΩ 分压点 + 10MΩ 手持表 ≈ 丢 5%%。\n"
           "  老师傅小结：先估计被测点阻抗再选表；高阻节点要么上 10GΩ 输入，"
           "要么改用跟随器/差分探头，别跟 10MΩ 较劲。\n");
}

/* ---------------------------------------------------------------------------
 * 演示 (b)：3½ vs 6½ 量 2.5V 基准
 * --------------------------------------------------------------------------- */
static void demoResolution(void)
{
    printf("\n=========== 演示 (b)：3½ 位 vs 6½ 位量同一个 2.5V 基准 ===========\n");
    printf("  被测：2.500000V 基准源，真实值带 ±30µV 的短期漂移（用噪声模拟）。\n");
    const DMM d35 = make35(), d65 = make65();
    Rng rng;
    rngSeed(&rng, 6102);
    printf("\n  3½ 表：2.5V 超 2V 档 → 只能上 20V 档，分辨率 10mV/字，"
           "规格 ±(0.8%%+3 字)=±%.0f mV\n",
           d35.vRanges[1].pctOfRead / 100.0 * 2.5 * 1e3
               + 3 * d35.vRanges[1].resolution * 1e3);
    for (int i = 0; i < 5; ++i) {
        Reading a = measureV(&d35, 2.5 + rngGauss(&rng, 0.0, 30e-6), 0.0, &rng);
        printf("    第 %d 次显示：%.2f V", i + 1, a.displayed);
        if (i == 0) printf("   ← 30µV 漂移？分辨率就没到那个量级，一个格都不带抖的");
        printf("\n");
    }
    printf("\n  6½ 表：上 10V 档，分辨率 10µV/字，规格 ±(0.0015%%+5 字)=±%.0f µV\n",
           d65.vRanges[1].pctOfRead / 100.0 * 2.5 * 1e6
               + 5 * d65.vRanges[1].resolution * 1e6);
    for (int i = 0; i < 5; ++i) {
        Reading b = measureV(&d65, 2.5 + rngGauss(&rng, 0.0, 30e-6), 0.0, &rng);
        printf("    第 %d 次显示：%.6f V\n", i + 1, b.displayed);
    }
    printf("\n  老师傅小结：3½ 表说\"2.50V，差不多\"；6½ 表能告诉你\"2.499981V，"
           "漂移 30µV 量级\"。判定\"行不行\"用 3½ 够了，做基准、调环路、"
           "看温漂，就得 6½——这不是讲究，是规格书要求的分辨率你得真看得见。\n");
}

/* ---------------------------------------------------------------------------
 * 演示 (c)：多次测量统计与合成不确定度
 * 均值/样本标准差手工算；仪器项按均匀分布 u = spec/√3；
 * 合成 u_c = sqrt(u_inst^2 + u_mean^2)，扩展 U(k=2) ≈ 95% 置信
 * --------------------------------------------------------------------------- */
static void demoStatistics(void)
{
    printf("\n=========== 演示 (c)：50 次重复测量 → 合成不确定度 ===========\n");
    Rng rng;
    rngSeed(&rng, 6103);
    const double rTrue = 4712.0; /* "标称 4.7k" 的电阻真实值 */
    const double sigma = 2.0;    /* 表笔接触/噪声导致的重复性 σ = 2Ω */
    const size_t N = 50;
    double *xs = (double *)xmalloc(N * sizeof(double));
    for (size_t i = 0; i < N; ++i) {
        const double v = rngGauss(&rng, rTrue, sigma);
        const double res = 1.0; /* 2kΩ 档分辨率 1Ω：显示按字舍入 */
        xs[i] = round(v / res) * res;
    }
    double mean = 0.0;
    for (size_t i = 0; i < N; ++i) mean += xs[i];
    mean /= (double)N;
    double var = 0.0;
    for (size_t i = 0; i < N; ++i) var += (xs[i] - mean) * (xs[i] - mean);
    var /= (double)(N - 1); /* 样本方差：自由度 N-1 */
    const double s = sqrt(var);
    /* 仪器不确定度：规格最大误差按均匀分布 → u = spec/√3 */
    const double spec = 0.8 / 100.0 * mean + 3 * 1.0;
    const double uInst = spec / sqrt(3.0);
    const double uMean = s / sqrt((double)N); /* A 类 */
    const double uC = sqrt(uInst * uInst + uMean * uMean);
    const double U = 2.0 * uC; /* 约 95% 置信 */
    printf("  N=%zu 次：均值 %.2f Ω，样本标准差 s = %.2f Ω\n", N, mean, s);
    printf("  仪器规格 ±(0.8%%+3 字) = ±%.2f Ω → 按均匀分布 u_inst = ±/√3 = %.2f Ω\n",
           spec, uInst);
    printf("  A 类（均值的标准不确定度）u_mean = s/√N = %.3f Ω\n", uMean);
    printf("  合成 u_c = sqrt(u_inst^2 + u_mean^2) = %.2f Ω；扩展 U(k=2) = ±%.2f Ω\n",
           uC, U);
    printf("  报告写法：(%.2f ± %.2f) kΩ (k=2) —— U=%.0f Ω 已经吃掉十位，\n"
           "            均值再精确的小数位都是自嗨。注意 s 比 u_inst 小一个量级：\n"
           "            这把椅子是\"规格\"晃，不是\"手\"晃——多测 500 次也救不了。\n",
           mean / 1e3, U / 1e3, U);
    printf("\n  反例加餐：如果每次测量都带同一个 +2%% 负载误差（系统误差），\n");
    const double sysErr = 0.02;
    double meanSys = 0.0;
    for (size_t i = 0; i < N; ++i) meanSys += xs[i] * (1.0 - sysErr);
    meanSys /= (double)N;
    printf("            均值变 %.2f Ω（偏了 %.2f Ω），但 s 一点没变、U 一点没变。\n"
           "            平均只对随机误差有效；系统误差只能靠换表/换方法/校准干掉。\n",
           meanSys, mean - meanSys);
    free(xs);
}

/* ---------------------------------------------------------------------------
 * 演示 (d)：电流档 —— 200mA 档 vs 10A 档
 * --------------------------------------------------------------------------- */
static void demoCurrent(void)
{
    printf("\n=========== 演示 (d)：量 6mA 该用哪个电流档 ===========\n");
    const DMM d = make35();
    const double vSupply = 5.0, vLed = 3.0, rLim = 330.0;
    const double iTrue = (vSupply - vLed) / rLim;
    printf("  场景：5.0V 电源点一个蓝 LED（压降约 3.0V）+ 330Ω 限流电阻，"
           "电流约 %.2f mA。\n", iTrue * 1e3);
    for (int i = 0; i < d.nA; ++i) {
        const ACurRange *r = &d.aRanges[i];
        /* 串进表：负担电压吃掉一部分电阻上的压降 */
        const double iMeas = (vSupply - vLed) / (rLim + r->burdenOhm);
        const double shown = round(iMeas / r->resolution) * r->resolution;
        const double spec = r->pctOfRead / 100.0 * iMeas
                            + (double)r->digitsOfErr * r->resolution;
        const double relErr = 100.0 * fabs(shown - iTrue) / iTrue;
        printf("  %5.0f mA 档（分辨率 %.1f mA，负担 %.3g Ω）：显示 %.2f mA",
               r->fullScale * 1e3, r->resolution * 1e3, r->burdenOhm, shown * 1e3);
        if (shown < r->resolution) {
            printf(" —— 电流是有的，是你选的档位看不见它。\n");
        } else if (relErr > 20.0) {
            printf(" —— 真值 %.2f mA 在这个档只有 %.1f 个字，"
                   "光舍入就错了 %.0f%%，规格 ±%.2f mA 还不够塞牙缝。← 档位吃掉了读数\n",
                   iMeas * 1e3, iMeas / r->resolution, relErr, spec * 1e3);
        } else {
            printf("（真值 %.2f mA，负担电压把电流吃掉了 %.2f%%，规格 ±%.2f mA）\n",
                   iMeas * 1e3, 100.0 * (iTrue - iMeas) / iTrue, spec * 1e3);
        }
    }
    printf("\n  两条铁律：\n");
    printf("  1) 电流档必须串联，且预估电流 > 档位满量程时先用最大档粗定位。\n");
    printf("  2) 严禁用电流档（内阻约 0.01Ω）去并联量电压 —— 等于把电源短路，\n"
           "     轻则烧表内保险丝，重则烧表笔/接口。量前先想：表笔插在哪个孔？\n");
}

/* --------------------------------------------------------------------------- */
int main(void)
{
    printf("================================================================\n");
    printf(" 测量误差实验室 —— 第 6 关 · 望闻问切（软件模型，无硬件）\n");
    printf("================================================================\n");
    demoLoading();
    demoResolution();
    demoStatistics();
    demoCurrent();
    printf("\n【老师傅收尾】每一次读数都是\"被测电路 × 仪器\"的乘积，\n"
           "不是电路单独的属性。负载、分辨率、精度规格、档位，四样都对上，\n"
           "数字才配叫测量结果；对不上，那只是仪器给你讲的一个故事。\n");
    return 0;
}
