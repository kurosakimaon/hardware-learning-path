/* =====================================================================
 * 03_decoupling_emc.c —— 去耦电容与 EMC 滤波器计算器
 *
 * 【这个文件讲什么】
 *   去耦电容不是"放个电容就完事"的仪式。一颗真实电容 = C + ESR + ESL，
 *   到了自谐振频率 f0 = 1/(2π√(LC)) 它最"硬"（阻抗最低 = ESR），
 *   再往上它就是颗电感——你精心放的 100nF，在 200MHz 时正在帮倒忙。
 *   更狠的是安装电感：电容到引脚每多 1mm 走线，约多 1nH，
 *   在 100MHz 下 1nH 就是 0.63Ω——比你整颗电容的谐振阻抗大十几倍。
 *   本程序把这些算成可看的数字：
 *     1) 单颗电容 Z-f 模型：Z = √( ESR² + (2πfL − 1/(2πfC))² )
 *     2) 0402/0603/0805 的自谐振点与最低阻抗（ESL≈0.5~0.7nH 量级）
 *     3) 单颗 100nF 在 100MHz 的残余阻抗（答案会让你重新审视"100nF 万能论"）
 *     4) 安装电感：走线 1/5/10mm 时 100MHz 阻抗恶化多少倍
 *     5) 多容值并联（导纳求和）：10µF+100nF+1nF 的频谱覆盖与反谐振峰
 *     6) "三个 100nF" vs "100nF+10nF+1µF"：一个降 ESR，一个扩频谱
 *     7) 目标阻抗反推：100MHz 下环路总 ESL 预算 → 允许走线长度
 *     8) 共模扼流圈 + Y 电容、X 电容的滤波拐点和粗略衰减量
 *     9) 去耦半径经验值 + 每类 IC 电源引脚的推荐去耦组合清单
 *
 * 【怎么编译运行】（MSYS2 / MinGW gcc 16.2，Windows，纯 C17）
 *   gcc -std=c17 -O2 -Wall -Wextra 03_decoupling_emc.c -o 03_decoupling_emc.exe
 *   .\03_decoupling_emc.exe
 *
 * 【运行前先看这里】
 *   控制台是中文，运行前务必先执行：chcp 65001
 *   否则 Windows 默认 GBK 代码页会把 UTF-8 的中文输出全打成乱码。
 *
 * 【预期看到什么】
 *   - 0603 100nF（ESL≈0.7nH）自谐振 ≈19MHz，谐振点阻抗 = ESR ≈ 40mΩ；
 *     同一颗到 100MHz 时残余阻抗 ≈ 0.43Ω——差了 10 倍
 *   - 100nF 离引脚 10mm：100MHz 阻抗恶化到 ≈ 6.7Ω，比贴着放差约 9 倍
 *   - 10µF+100nF+1nF 并联：频谱覆盖宽 3 个数量级，但容值交界处冒出反谐振峰
 *   - 三个 100nF 并联：谐振点附近阻抗降为 1/3，频谱覆盖一点没变宽
 *   - 目标阻抗 0.1Ω @100MHz → 允许总 ESL 仅 0.16nH → 0603 电容裸 ESL 就超预算，
 *     所以高速 IC 才要求电容贴背面/过孔就近打孔
 *   - 共模扼流圈 10mH + Y 电容 2.2nF：拐点 ≈34kHz，之后每十倍频程 ≈40dB，
 *     实际被寄生参数限制在 30~40dB——这就是 EMC 整改"加磁环有效但别指望 60dB"的道理
 *
 * 【老师傅点拨】
 *   1. 去耦的第一性原理是"缩小高频环路面积"，不是"提供电荷仓库"。
 *      电荷仓库那是大电容（10µF+）干的活，管的是 kHz~MHz 的慢电流。
 *   2. 电容离引脚的距离，从引脚→过孔→电容焊盘再回来，整圈都算。
 *      "离引脚 10mm"不是夸张，很多板子就是这么画的，白去耦。
 *   3. 反谐振峰不是玄学：大电容呈感性、小电容呈容性，两者在中间某频点
 *      并联谐振，阻抗顶起来。容值档数越多、ESR 搭配越合理，峰越矮。
 *   4. 同一个容值买两种 ESR 的（比如 100nF 混装 X7R 和 X5R）也能压反谐振峰，
 *      这是便宜又有效的土办法。
 *   5. 共模噪声靠"扼流圈 + Y 电容"堵回路，差模靠"X 电容 + 大电感/磁珠"，
 *      先分清你测到的是共模还是差模，再决定往哪砸钱。
 * ===================================================================== */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define PI 3.14159265358979323846

#define NCAP 3        /* 每个并联方案最多几颗电容 */
#define NMM 3
#define NCASE 3
#define NSTAGE 3
#define NCX 3
#define NFREQ 4

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
 * 真实电容模型：C 串联 ESR 再串联 ESL
 *   Z(f) = sqrt( ESR^2 + (2*pi*f*L - 1/(2*pi*f*C))^2 )
 * ESL 量级（表面贴装，含电容本体内部电感）：
 *   0402 ≈ 0.5 nH，0603 ≈ 0.7 nH，0805 ≈ 0.9 nH，1206 ≈ 1.1 nH
 * 每增加 1mm 安装走线（含过孔折算），约 +1 nH。
 * ------------------------------------------------------------------- */
typedef struct {
    const char *name;
    double cF;    /* 电容值 F */
    double esr;   /* 等效串联电阻 Ω */
    double eslH;  /* 等效串联电感 H（电容本体） */
} Cap;

static double capZ(const Cap *c, double fHz) {
    const double w = 2.0 * PI * fHz;
    const double x = w * c->eslH - 1.0 / (w * c->cF);
    return sqrt(c->esr * c->esr + x * x);
}

static double selfResonance(const Cap *c) {
    return 1.0 / (2.0 * PI * sqrt(c->eslH * c->cF));
}

/* 多颗电容并联：导纳求和 Y = Σ 1/(ESR + jX)，|Z| = 1/|Y| */
static double bankZ(const Cap *caps, int n, double fHz) {
    double g = 0.0, b = 0.0;
    int i;
    for (i = 0; i < n; ++i) {
        const Cap *c = &caps[i];
        const double w = 2.0 * PI * fHz;
        const double x = w * c->eslH - 1.0 / (w * c->cF);
        const double denom = c->esr * c->esr + x * x;
        g += c->esr / denom;
        b += -x / denom;
    }
    return 1.0 / sqrt(g * g + b * b);
}

/* 把额外安装电感加到电容上（C 里直接返回值，用普通结构体拷贝） */
static Cap withWiring(const Cap *c, double mm) {
    Cap out = *c;
    out.eslH += mm * 1.0e-9;  /* 每 mm ≈ 1 nH */
    return out;
}

/* 阻抗的自适应格式化：原来返回 std::string，这里写进调用者给的缓冲区 */
static void fmtOhm(char *buf, size_t n, double z) {
    if (z >= 100.0) snprintf(buf, n, "%8.1f ", z);
    else if (z >= 1.0) snprintf(buf, n, "%8.3f ", z);
    else if (z >= 0.001) snprintf(buf, n, "%8.1fm ", z * 1e3);
    else snprintf(buf, n, "%8.2fu ", z * 1e6);
}

static void fmtF(char *buf, size_t n, double hz) {
    if (hz >= 1e9) snprintf(buf, n, "%.2fGHz", hz / 1e9);
    else if (hz >= 1e6) snprintf(buf, n, "%.1fMHz", hz / 1e6);
    else snprintf(buf, n, "%.0fkHz", hz / 1e3);
}

/* 扫描一段对数频率：最低阻抗点 + 反谐振峰（局部极大值中的最大者）。
 * 同值并联没有局部极大 → peak 记 0，表示"没有反谐振峰"。 */
typedef struct {
    double zMin, fAtMin;
    double zPeak, fAtPeak;  /* 反谐振峰；无则 zPeak=0 */
} ScanResult;

static ScanResult scanBank(const Cap *caps, int nCap, double fLo, double fHi, int pointsPerDecade) {
    ScanResult r;
    double decades;
    int n, i;
    double *zs;

    r.zMin = 1e30;
    r.fAtMin = 0.0;
    r.zPeak = 0.0;
    r.fAtPeak = 0.0;

    decades = log10(fHi / fLo);
    n = (int)(decades * pointsPerDecade);
    zs = (double *)malloc(((size_t)n + 1u) * sizeof(double));
    if (zs == NULL) {
        fprintf(stderr, "scanBank：内存分配失败（%d 个采样点）\n", n + 1);
        exit(EXIT_FAILURE);
    }

    for (i = 0; i <= n; ++i) {
        const double f = fLo * pow(10.0, decades * i / n);
        zs[i] = bankZ(caps, nCap, f);
        if (zs[i] < r.zMin) { r.zMin = zs[i]; r.fAtMin = f; }
    }
    for (i = 1; i < n; ++i) {
        const double z = zs[i];
        if (z > zs[i - 1] && z >= zs[i + 1]) {
            if (z > r.zPeak) {
                r.zPeak = z;
                r.fAtPeak = fLo * pow(10.0, decades * i / n);
            }
        }
    }
    free(zs);
    return r;
}

/* 看 100kHz~1GHz 区间内"阻抗 < zLimit"的覆盖范围（原来返回 pair，这里用出参） */
static void coverage(const Cap *caps, int nCap, double zLimit, double *outLo, double *outHi) {
    double fLo = 0.0, fHi = 0.0;
    const int n = 2000;
    int i;
    for (i = 0; i <= n; ++i) {
        const double f = 1e5 * pow(10.0, 4.0 * i / n);  /* 100k~1G */
        if (bankZ(caps, nCap, f) < zLimit) {
            if (fLo == 0.0) fLo = f;
            fHi = f;
        }
    }
    *outLo = fLo;
    *outHi = fHi;
}

/* ---------------------------------------------------------------------
 * 1. 单颗电容：自谐振点、最低阻抗、100MHz 残余阻抗
 * ------------------------------------------------------------------- */
static void singleCaps(void) {
    static const Cap list[NCAP] = {
        {"0402 100nF（ESL≈0.5nH）", 100e-9, 0.030, 0.5e-9},
        {"0603 100nF（ESL≈0.7nH）", 100e-9, 0.040, 0.7e-9},
        {"0805 100nF（ESL≈0.9nH）", 100e-9, 0.045, 0.9e-9},
    };
    char fb[24];
    int i;

    sub("单颗 100nF（X7R）：封装决定 ESL，ESL 决定自谐振点");
    printf("  电容          自谐振 f0    谐振点阻抗(=ESR)   Z@100MHz    Z@500MHz\n");
    for (i = 0; i < NCAP; ++i) {
        const Cap *c = &list[i];
        const double f0 = selfResonance(c);
        fmtF(fb, sizeof(fb), f0);
        printf("  %-24s %-10s %8.0f mΩ      %8.3f Ω    %8.2f Ω\n",
               c->name, fb, c->esr * 1e3, capZ(c, 100e6), capZ(c, 500e6));
    }
    printf("  看到没有：100nF 在 100MHz 时早就「变成电感」了，残余阻抗 ~0.4Ω。\n");
    printf("  芯片内部时钟如果主要在 100~200MHz 抽电流，单靠 100nF 是压不住的，\n");
    printf("  必须再并联一颗自谐振点更高的（1nF，甚至 0402 封装的 470pF）。\n");

    sub("为什么 0402 比 0603 高频好？——不只是 ESL 小");
    printf("  同容值同介质下 0402 的电极面积小、ESL 小 → f0 高 ≈18%%；\n");
    printf("  但 0402 容易有 DC 偏压容量塌陷（标 100nF，加 3.3V 偏压剩 60nF），\n");
    printf("  介质 X5R/X7R 与电压系数要查厂商曲线。教学模型不含 DC 偏压修正。\n");
}

/* ---------------------------------------------------------------------
 * 2. 安装电感：电容离引脚 1/5/10mm 的代价
 * ------------------------------------------------------------------- */
static void mountingInductance(void) {
    static const double mmList[NMM] = {1.0, 5.0, 10.0};
    const Cap base = {"0603 100nF", 100e-9, 0.040, 0.7e-9};
    const Cap near = withWiring(&base, 0.5);  /* 0.5mm = 贴着放的基准 */
    const double z0 = capZ(&near, 100e6);
    char buf[48];
    int i;

    sub("安装电感：每 mm ≈ 1nH，看 100MHz 下阻抗恶化多少倍");
    fmtOhm(buf, sizeof(buf), z0);
    printf("  基准（0.5mm，几乎贴着）：Z@100MHz = %sΩ\n", buf);
    printf("  走线长度   总 ESL      Z@100MHz     恶化倍数\n");
    for (i = 0; i < NMM; ++i) {
        const Cap c = withWiring(&base, mmList[i]);
        const double z = capZ(&c, 100e6);
        fmtOhm(buf, sizeof(buf), z);
        printf("  %4.0f mm     %4.1f nH    %sΩ   %6.1f 倍\n",
               mmList[i], c.eslH * 1e9, buf, z / z0);
    }
    printf("  老师傅常说「电容离引脚 10mm 等于没放」——按这个模型 100MHz 处恶化约 9 倍，\n");
    printf("  不是吓你。而且这 10mm 是「引脚→过孔→走线→焊盘」整圈回路的长度，\n");
    printf("  不是直瞄距离。环路面积决定辐射，走线绕远一圈，辐射天线就大一号。\n");
}

/* ---------------------------------------------------------------------
 * 3. 多容值并联 vs 三个同值并联
 * ------------------------------------------------------------------- */
static void bankCompare(void) {
    static const Cap staggeredSrc[NCAP] = {
        {"10uF", 10e-6, 0.008, 0.9e-9},
        {"100nF", 100e-9, 0.040, 0.7e-9},
        {"1nF", 1e-9, 0.010, 0.5e-9},
    };
    static const Cap tripleSrc[NCAP] = {
        {"100nF#1", 100e-9, 0.040, 0.7e-9},
        {"100nF#2", 100e-9, 0.040, 0.7e-9},
        {"100nF#3", 100e-9, 0.040, 0.7e-9},
    };
    Cap staggered[NCAP], triple[NCAP];
    ScanResult a, b;
    double covALo = 0.0, covAHi = 0.0, covBLo = 0.0, covBHi = 0.0;
    char f1[24], f2[24], f3[24], f4[24];
    char za[48], zb[48], zap[48], zbp[48];
    int i;

    sub("方案 A：10µF(0805) + 100nF(0603) + 1nF(0402)，全部贴着放（+0.5mm 走线）");
    for (i = 0; i < NCAP; ++i) staggered[i] = withWiring(&staggeredSrc[i], 0.5);
    a = scanBank(staggered, NCAP, 1e5, 1e9, 80);

    sub("方案 B：三颗 100nF(0603) 并联，同样贴着放");
    for (i = 0; i < NCAP; ++i) triple[i] = withWiring(&tripleSrc[i], 0.5);
    b = scanBank(triple, NCAP, 1e5, 1e9, 80);

    fmtF(f1, sizeof(f1), a.fAtMin); fmtF(f2, sizeof(f2), a.fAtPeak);
    fmtF(f3, sizeof(f3), b.fAtMin); fmtF(f4, sizeof(f4), b.fAtPeak);
    fmtOhm(za, sizeof(za), a.zMin); fmtOhm(zap, sizeof(zap), a.zPeak);
    fmtOhm(zb, sizeof(zb), b.zMin); fmtOhm(zbp, sizeof(zbp), b.zPeak);
    printf("  方案          最低阻抗点            最低阻抗    最大反谐振峰           峰值阻抗\n");
    printf("  A 三容值      %-10s %sΩ     %-10s        %sΩ\n",
           f1, za, f2, zap);
    printf("  B 三颗同值    %-10s %sΩ     %-10s        %sΩ\n",
           f3, zb,
           b.zPeak > 0.0 ? f4 : "无（同值并联）",
           b.zPeak > 0.0 ? zbp : "    ----");

    /* 各自看 1MHz~300MHz 区间内"阻抗 < 0.5Ω"的覆盖带宽 */
    coverage(staggered, NCAP, 0.5, &covALo, &covAHi);
    coverage(triple, NCAP, 0.5, &covBLo, &covBHi);
    fmtF(f1, sizeof(f1), covALo); fmtF(f2, sizeof(f2), covAHi);
    fmtF(f3, sizeof(f3), covBLo); fmtF(f4, sizeof(f4), covBHi);
    printf("\n  Z<0.5Ω 的覆盖范围：A = %s ~ %s；B = %s ~ %s\n", f1, f2, f3, f4);
    printf("  结论：三颗同值只把谐振点附近阻抗砍到 1/3，覆盖宽度纹丝不动；\n");
    printf("  错开容值才能把低频（µF）到高频（nF）连成一片——代价是交界处有反谐振峰。\n");
    printf("  反谐振峰的压法：选不同 ESR 的电容搭配（比如混装 X7R/X5R），或串小电阻。\n");
}

/* ---------------------------------------------------------------------
 * 4. 目标阻抗反推 ESL 预算与允许走线长度
 * ------------------------------------------------------------------- */
static void targetImpedance(void) {
    typedef struct { double z; double f; } Case;
    static const Case cases[NCASE] = {
        {0.50, 200e6}, {0.10, 100e6}, {0.05, 200e6},
    };
    int i;

    sub("目标阻抗法：Z_target = 允许纹波 / 动态电流 → 反推 ESL 预算");
    printf("  场景：3.3V 内核允许纹波 100mV、动态电流 200mA → Z_target = 0.5Ω\n");
    printf("  最高关注频率 f_max = 200MHz（时钟 100MHz 的二次谐波附近）\n");
    printf("  感性段近似 Z ≈ 2πf·ESL_total → ESL_budget = Z_target/(2πf_max)\n\n");
    printf("  Z_target   f_max      ESL 预算     0603(0.7nH)加线预算   结论\n");
    for (i = 0; i < NCASE; ++i) {
        const double budget = cases[i].z / (2.0 * PI * cases[i].f);
        const double wire = (budget - 0.7e-9) * 1e9;  /* mm，每 mm 1nH */
        const char *verdict = (wire >= 3.0) ? "普通贴装可行"
                            : (wire >= 0.0) ? "必须打孔就近、走线以 0.1mm 级抠"
                                            : "电容本体 ESL 已超预算→换 0402/贴背面";
        printf("  %4.2fΩ    %4.0fMHz    %6.2f nH      %+6.2f mm          %s\n",
               cases[i].z, cases[i].f / 1e6, budget * 1e9, wire, verdict);
    }
    printf("  这就是手册里「去耦电容紧贴引脚、过孔打在焊盘上」的数学依据：\n");
    printf("  不是风格问题，是 0.nH 级的预算问题。\n");

    sub("去耦半径经验值（从引脚算起的回路总长，含过孔）");
    printf("  10µF 体电容：    ≤ 5~10 mm —— 管 kHz~MHz 慢电流，本来就不指望高频\n");
    printf("  100nF（0402/0603）：≤ 2~3 mm —— 管 10~100MHz，最常用的一档\n");
    printf("  1nF / 470pF：      ≤ 1~1.5 mm —— 管 100MHz~数百 MHz，配高速/时钟 IC\n");
    printf("  经验公式：关注频率每升 10 倍，允许回路长度缩到 1/10（Z=2πfL）。\n");
}

/* ---------------------------------------------------------------------
 * 5. 共模扼流圈 + Y/X 电容的滤波拐点与衰减
 * ------------------------------------------------------------------- */
static void emcFilters(void) {
    typedef struct { const char *name; double lH; double cF; } Stage;
    static const Stage stages[NSTAGE] = {
        {"10mH 共模扼流圈 + 2×2.2nF Y", 10e-3, 2.2e-9},
        {"1mH  共模扼流圈 + 2×4.7nF Y", 1e-3, 4.7e-9},
        {"100µH 共模扼流圈 + 2×10nF Y", 100e-6, 10e-9},
    };
    static const double freqList[NFREQ] = {150e3, 1e6, 10e6, 30e6};
    static const double cxList[NCX] = {100e-9, 470e-9, 1e-6};
    const double lDm = 5e-6;   /* 5µH：进线电感 + 串入磁珠的典型量级 */
    char fb[24];
    int i, j;

    sub("共模通道：扼流圈 L_cm + 对机壳 Y 电容（安全电流限制 Y 容量）");
    /* 共模：两只 Y 电容对机壳分压，等效单端 C/2 简化教学模型 */
    printf("  滤波器                          拐点 f0     150kHz   1MHz    10MHz   30MHz\n");
    printf("  （衰减按 40dB/dec 上升、封顶 40dB——寄生参数决定的实际上限）\n");
    for (i = 0; i < NSTAGE; ++i) {
        const double f0 = 1.0 / (2.0 * PI * sqrt(stages[i].lH * (stages[i].cF / 2.0)));
        fmtF(fb, sizeof(fb), f0);
        printf("  %-32s %-9s", stages[i].name, fb);
        for (j = 0; j < NFREQ; ++j) {
            const double f = freqList[j];
            const double il = (f <= f0) ? 0.0
                              : fmin(40.0, 40.0 * log10(f / f0));
            printf(" %5.0fdB", il);
        }
        printf("\n");
    }
    printf("  150kHz 正是 CISPR/GB 传导测试的起点：拐点必须压在 150kHz 以下，\n");
    printf("  否则测试频段第一条线就没有衰减——磁环「没起作用」多半是这个原因。\n");

    sub("差模通道：X 电容 + 线路残余电感（含保险丝/磁珠）");
    for (i = 0; i < NCX; ++i) {
        const double f0 = 1.0 / (2.0 * PI * sqrt(lDm * cxList[i]));
        fmtF(fb, sizeof(fb), f0);
        printf("  X 电容 %.3f uF + %.0f uH → 差模拐点 %s\n", cxList[i] * 1e6, lDm * 1e6, fb);
    }
    printf("  差模噪声大：先查电源纹波、去耦、回流，别急着堆 X 电容——\n");
    printf("  X 电容治的是「已经进了电源线往外跑」的那一段，治不了产生源。\n");
}

/* ---------------------------------------------------------------------
 * 6. 每类 IC 电源引脚的推荐去耦组合
 * ------------------------------------------------------------------- */
static void checklist(void) {
    sub("推荐去耦组合清单（老师傅默认值，最终仍以芯片手册为准）");
    printf("  MCU 数字 VDD：每个电源脚 1×100nF(0402/0603) 贴引脚 +\n");
    printf("                 每个电源域 1×4.7~10µF 体电容（入口处分流）\n");
    printf("  MCU 模拟 AVDD/VREF：1×1µF + 1×100nF，磁珠/0Ω 与数字源隔离，\n");
    printf("                 电容全部放在模拟区，回流不许跨数字分割\n");
    printf("  DC-DC 输入：2×10µF 低 ESR 陶瓷（紧贴 VIN/SW 环路最小化）+ 1×100nF\n");
    printf("  DC-DC 输出：22~100µF（聚合物或 MLCC）+ 1×100nF，环路面积是命根\n");
    printf("  时钟发生器/PLL：按手册！通常 1×10nF 或 1×470pF + 1×100nF，\n");
    printf("                 低噪声 LDO 单独供电，去耦一个都不许省\n");
    printf("  高速存储器：每对电源脚 100nF + 每组 1~2×10µF，SSO 噪声全靠这堆\n");
    printf("  LED/继电器驱动局部：入口 1×100µF 电解/聚合物——线宽救不了浪涌，\n");
    printf("                 能量储蓄还得靠大 C\n");
}

int main(void) {
    section("03 去耦电容与 EMC 滤波器计算器（RLC 串联模型 + 导纳并联叠加）");
    printf("去耦的本质：给芯片的高频动态电流修一条「离家最近」的环路。\n");
    printf("模型：理想 C + ESR + ESL 串联；并联网络用导纳求和；不含 DC 偏压/温度修正。\n");

    singleCaps();
    mountingInductance();
    bankCompare();
    targetImpedance();
    emcFilters();
    checklist();

    section("老师傅收口");
    printf("1) 电容的「有效频段」由自谐振点决定，超出 f0 它就是电感。错开容值才有宽带。\n");
    printf("2) 安装电感（走线+过孔）比电容本体 ESL 更常超标：1mm=1nH=100MHz 下 0.63Ω。\n");
    printf("3) 大电容管能量、小电容管高频、反谐振峰看 ESR——三者各管一段，缺一不可。\n");
    printf("4) EMC 滤波：先分共模/差模，再选扼流圈/Y/X；拐点压进测试频段才有衰减。\n");
    printf("5) 所有去耦计算的前提还是那句话：参考平面完整。地都碎了，电容就是装饰品。\n");
    printf("\n");
    return 0;
}
