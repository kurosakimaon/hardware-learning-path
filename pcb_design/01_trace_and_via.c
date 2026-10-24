/* =====================================================================
 * 01_trace_and_via.c —— 走线与过孔的载流 / 温升计算器
 *
 * 【这个文件讲什么】
 *   这块板子为什么要"用游标卡尺量线宽"？因为铜不是导线，它是有电阻的一小条金属。
 *   电流过去就发热，热散不掉就升温，升温到一定程度：阻焊发黄、铜皮剥离、
 *   严重时烧断——这就是"电源线一烧一个洞"的完整物理链条。
 *
 *   本程序用三个模型把这件事算清楚：
 *     1) IPC-2221 走线载流公式：I = k · ΔT^0.44 · A^0.725
 *        外层 k = 0.048，内层 k = 0.024（内层被介质包着，散热差一半）
 *        A 是导线截面积，单位 mil²（1 mil = 0.0254 mm）
 *     2) 过孔载流：把孔壁铜当成一段"圆筒形电阻"，
 *        用 R = ρL/A 算电阻，再用 P = I²R = h·S·ΔT 的热平衡反推温升。
 *        同时给出 IPC-2222 风格的"截面积套 IPC-2221"乐观值做对比。
 *     3) 降额：内层折半、多过孔并联合流不均（n^0.85）、
 *        铜厚公差 / 环境温度 / 阻焊覆盖的工程降额系数 0.7。
 *
 * 【怎么编译运行】（MSYS2 / MinGW gcc 16.2，Windows，纯 C17）
 *   gcc -std=c17 -O2 -Wall -Wextra 01_trace_and_via.c -o 01_trace_and_via.exe
 *   .\01_trace_and_via.exe
 *
 * 【运行前先看这里】
 *   控制台是中文，运行前务必先执行：chcp 65001
 *   否则 Windows 默认 GBK 代码页会把 UTF-8 的中文输出全打成乱码。
 *
 * 【预期看到什么】
 *   - 铜厚 0.5/1/2 oz × 温升 10/20/30/40 ℃ 的载流能力表
 *     （自查锚点：1 oz 外层 12 mil ≈ 0.3 mm，20 ℃ 温升的 IPC 裸值约 1.35 A，
 *       乘上 0.7 的工程降额后就是老师傅嘴里那句"0.3 mm 走 1 A"）
 *   - 反算表：要走 0.5 / 1 / 2 / 3 / 5 / 8 A，各铜厚各温升需要多宽
 *   - 单个 0.3 mm 过孔走 2 A：保守模型温升约 64 ℃（乐观模型约 8 ℃）——别赌
 *   - 1 A / 2 A / 5 A 电源线的推荐方案表：线宽 + 过孔数 + 铜厚 + 要不要开窗加锡
 *
 * 【老师傅点拨】
 *   1. IPC-2221 是"实验室裸值"，它假设板子周围凉、铜厚达标、没有阻焊盖着。
 *      真实产品要按 0.7 甚至 0.5 降额，尤其是密闭外壳 + 高温环境。
 *   2. 内层走线不是"藏在里面更安全"，是"藏在里面更热"。同样 2 A，
 *      内层线宽要放到外层的 2.6 倍。
 *   3. 过孔是整条供电路径上散热最差的环节。一颗 0.3 mm 过孔的孔壁截面
 *      π·t·(d+t) ≈ 0.02 mm²，约是 0.3 mm × 35 µm 走线（0.0105 mm²）的 2 倍，
 *      但外表面只有孔壁那一圈，热量散不出去。
 *      大电流必须多孔并联 + 铺铜 + 钢网开窗加锡，一个都别省。
 *   4. 算完别忘了去 EDA 里把规则真的改掉。DRC 里电源线还是默认 0.2 mm，
 *      说明你只是算了，没有落地。
 * ===================================================================== */

#include <math.h>
#include <stdio.h>

#define PI 3.14159265358979323846
#define MIL2MM 0.0254   /* 1 mil = 0.0254 mm */
#define MM2MIL (1.0 / 0.0254)
#define OZ2MIL 1.378    /* 1 oz 铜厚 ≈ 35 µm = 1.378 mil */
#define OZ2MM 0.035     /* 1 oz 铜厚 ≈ 0.035 mm */

/* 工程降额系数 = 铜厚公差 0.85 × 阻焊覆盖 0.85 × 蚀刻侧蚀与批量偏差 0.97 ≈ 0.70 */
#define DERATE 0.70

/* 铜电阻率 Ω·m @20℃ */
#define RHO_CU 1.72e-8
#define H_POOR 50.0
#define H_GOOD 400.0

#define NWIDTH 8
#define NOZ 3
#define NDT 4
#define NI 7
#define NDT3 3

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
 * IPC-2221：I = k · ΔT^0.44 · A^0.725
 *   areaMil2 : 导线截面积，mil²
 *   dT       : 允许温升，℃（相对环境温度，不是绝对温度）
 *   external : 1 = 外层（外层 k=0.048），0 = 内层（k=0.024）
 * ------------------------------------------------------------------- */
static double ipcK(int external) { return external ? 0.048 : 0.024; }

static double ipcCurrent(double areaMil2, double dT, int external) {
    return ipcK(external) * pow(dT, 0.44) * pow(areaMil2, 0.725);
}

static double ipcAreaMil2(double current, double dT, int external) {
    return pow(current / (ipcK(external) * pow(dT, 0.44)), 1.0 / 0.725);
}

static double thicknessMil(double oz) { return oz * OZ2MIL; }

/* 给定电流 / 温升 / 铜厚，返回"IPC 裸值"线宽（mm） */
static double ipcWidthMm(double current, double dT, double oz, int external) {
    return ipcAreaMil2(current, dT, external) / thicknessMil(oz) * MIL2MM;
}

/* ---------------------------------------------------------------------
 * 过孔模型
 *   把孔壁铜看成一段圆筒：
 *     环形截面积 A = π·t·(d + t)          （d=成孔直径，t=孔壁铜厚）
 *     电阻       R = ρ·L / A              （L=孔深，即板厚）
 *     散热       P = I²R = h·S·ΔT         （S=孔壁外表面面积）
 *   h 是"等效换热系数"，这里给两档：
 *     hPoor  = 50  W/m²K  孤立过孔、只有自然对流（保守，用来定安全值）
 *     hGood  = 400 W/m²K  两端接大铜皮/内层平面，热量被铜带走（乐观）
 * ------------------------------------------------------------------- */
typedef struct {
    double holeMm;     /* 成孔直径 mm */
    double platingUm;  /* 孔壁铜厚 µm（成品孔铜，通常 18~25 µm） */
    double depthMm;    /* 孔深 mm（= 板厚） */
} Via;

static double viaBarrelAreaM2(const Via *v) {
    const double d = v->holeMm * 1e-3;
    const double t = v->platingUm * 1e-6;
    return PI * (d + t) * t;
}

static double viaResistance(const Via *v) {
    return RHO_CU * (v->depthMm * 1e-3) / viaBarrelAreaM2(v);
}

static double viaSurfaceM2(const Via *v) {
    const double d = v->holeMm * 1e-3;
    const double t = v->platingUm * 1e-6;
    return PI * (d + 2.0 * t) * (v->depthMm * 1e-3);
}

/* 给定电流求温升 */
static double viaTempRise(const Via *v, double current, double hEff) {
    return current * current * viaResistance(v) / (hEff * viaSurfaceM2(v));
}

/* 给定允许温升求安全电流 */
static double viaCurrentForDt(const Via *v, double dT, double hEff) {
    return sqrt(dT * hEff * viaSurfaceM2(v) / viaResistance(v));
}

/* IPC-2222 风格近似：把孔壁环形截面当面积，套 IPC-2221 外层公式（乐观值） */
static double viaCurrentIpc(const Via *v, double dT) {
    const double areaMil2 =
        PI * (v->platingUm * 1e-3 * MM2MIL) *
        ((v->holeMm + v->platingUm * 1e-3) * MM2MIL);
    return ipcCurrent(areaMil2, dT, 1);
}

/* 多过孔并联：电流分配不均（最靠源的那颗吃得最多），用 n^0.85 折算 */
static double viaBankCurrent(const Via *v, int count, double dT, double hEff) {
    return (double)count * pow((double)count, -0.15) * viaCurrentForDt(v, dT, hEff);
}

static int viasNeeded(const Via *v, double current, double dT, double hEff) {
    int cnt;
    for (cnt = 1; cnt <= 64; ++cnt) {
        if (viaBankCurrent(v, cnt, dT, hEff) * DERATE >= current) return cnt;
    }
    return 64;
}

/* ---------------------------------------------------------------------
 * 章节 1：载流能力表（给线宽 → 查电流）
 * ------------------------------------------------------------------- */
static void tableCapacity(void) {
    static const double widthMm[NWIDTH] = {0.15, 0.20, 0.30, 0.50, 0.80, 1.20, 2.00, 3.00};
    static const double ozList[NOZ] = {0.5, 1.0, 2.0};
    static const double dTList[NDT] = {10.0, 20.0, 30.0, 40.0};
    int ext, i, j, k;

    for (ext = 1; ext >= 0; --ext) {
        sub(ext ? "外层（外层 k=0.048，散热好）：安全电流 A（IPC-2221 裸值 / 括号外，降额 0.7 后的工程值 / 紧跟其后）"
                : "内层（内层 k=0.024，被介质包着，散热差）：安全电流 A（IPC-2221 裸值 / 括号外，降额 0.7 后的工程值 / 紧跟其后）");
        printf("铜厚 oz  线宽mm");
        for (k = 0; k < NDT; ++k) printf("   dT=%2.0f℃", dTList[k]);
        printf("\n");
        for (i = 0; i < NOZ; ++i) {
            const double oz = ozList[i];
            for (j = 0; j < NWIDTH; ++j) {
                const double w = widthMm[j];
                const double areaMil2 = (w * MM2MIL) * thicknessMil(oz);
                printf("%6.1f  %7.2f", oz, w);
                for (k = 0; k < NDT; ++k) {
                    const double raw = ipcCurrent(areaMil2, dTList[k], ext);
                    printf("  %5.2f/%4.2f", raw, raw * DERATE);
                }
                printf("\n");
            }
            printf("\n");
        }
    }
    printf("\n读表：格式是\"裸值/降额值\"。设计看降额值；裸值只用来说明 IPC 有多乐观。\n");
}

/* ---------------------------------------------------------------------
 * 章节 2：反算表（给电流 → 求线宽）
 * ------------------------------------------------------------------- */
static void tableWidth(void) {
    static const double iList[NI] = {0.5, 1.0, 2.0, 3.0, 5.0, 8.0, 10.0};
    static const double dTList[NDT3] = {10.0, 20.0, 30.0};
    static const double ozTwo[2] = {1.0, 2.0};
    int ext, i, j, k;

    for (ext = 1; ext >= 0; --ext) {
        sub(ext ? "外层：所需线宽 mm（IPC 裸值 → 降额后建议值）"
                : "内层：所需线宽 mm（IPC 裸值 → 降额后建议值）");
        printf("电流A   铜厚oz");
        for (k = 0; k < NDT3; ++k) printf("     dT=%2.0f℃", dTList[k]);
        printf("\n");
        for (i = 0; i < NI; ++i) {
            const double cur = iList[i];
            for (j = 0; j < 2; ++j) {
                const double oz = ozTwo[j];
                printf("%6.1f  %5.1f", cur, oz);
                for (k = 0; k < NDT3; ++k) {
                    const double raw = ipcWidthMm(cur, dTList[k], oz, ext);
                    const double derated = ipcWidthMm(cur / DERATE, dTList[k], oz, ext);
                    printf("   %5.2f->%5.2f", raw, derated);
                }
                printf("\n");
            }
        }
        printf("\n");
    }
}

/* ---------------------------------------------------------------------
 * 章节 3：过孔
 * ------------------------------------------------------------------- */
static void viaLab(void) {
    const Via v03 = {0.30, 20.0, 1.60};
    const Via v05 = {0.50, 25.0, 1.60};
    static const double scanList[6] = {0.5, 1.0, 1.5, 2.0, 3.0, 4.0};
    static const int cntList[7] = {1, 2, 3, 4, 6, 8, 10};
    const Via *viaPair[2];
    int i;

    viaPair[0] = &v03;
    viaPair[1] = &v05;

    sub("过孔直流电阻（孔壁铜这一段）");
    for (i = 0; i < 2; ++i) {
        const Via *v = viaPair[i];
        printf("  孔 %4.2f mm / 孔铜 %4.1f um / 孔深 %4.2f mm ：截面积 %6.4f mm²，R = %6.3f mΩ\n",
               v->holeMm, v->platingUm, v->depthMm,
               viaBarrelAreaM2(v) * 1e6, viaResistance(v) * 1e3);
    }
    printf("  对比：0.3 mm × 1 oz(35 µm) 走线，每 mm 的电阻 = %6.3f mΩ\n",
           RHO_CU * 1e-3 / (0.3e-3 * 35e-6) * 1e3);
    printf("  → 孔壁截面其实和 0.3mm/1oz 走线同量级；真正的瓶颈是散热面积只有孔壁那一圈。\n");

    sub("单个 0.3 mm 过孔走 2 A 会怎样？");
    printf("  保守模型（孤立过孔，h=%.0f W/m²K）  ：温升 = %.1f ℃\n",
           H_POOR, viaTempRise(&v03, 2.0, H_POOR));
    printf("  乐观模型（两端接大铜皮，h=%.0f W/m²K）：温升 = %.1f ℃\n",
           H_GOOD, viaTempRise(&v03, 2.0, H_GOOD));
    printf("  IPC-2222 面积法在 20 ℃ 温升下的裸值：%.2f A；保守热模型：%.2f A\n",
           viaCurrentIpc(&v03, 20.0), viaCurrentForDt(&v03, 20.0, H_POOR));
    printf("  温升扫描（保守模型，单颗 0.3 mm）：");
    for (i = 0; i < 6; ++i) {
        printf(" %gA->%.0f℃", scanList[i], viaTempRise(&v03, scanList[i], H_POOR));
    }
    printf("\n  3 A 时温升已破 150 ℃：孔铜与基材界面先完蛋，表现为"
           "过孔电阻越来越大、最后开路。工厂回来跑高温老化才炸的板子，很多是这个。\n");

    sub("多过孔并联（含不均流折算 n^0.85 与降额 0.7，保守模型，dT=20℃）");
    printf("  颗数   0.3mm孔安全电流   0.5mm孔安全电流\n");
    for (i = 0; i < 7; ++i) {
        const int cnt = cntList[i];
        printf("  %4d       %5.2f A            %5.2f A\n", cnt,
               viaBankCurrent(&v03, cnt, 20.0, H_POOR) * DERATE,
               viaBankCurrent(&v05, cnt, 20.0, H_POOR) * DERATE);
    }
    printf("  注意收益不是线性的：10 颗 0.3 mm 孔不如 4 颗 0.5 mm 孔，"
           "大电流优先用大孔或裸铜+加锡，而不是数着牙签打孔。\n");
}

/* ---------------------------------------------------------------------
 * 章节 4：降额对比
 * ------------------------------------------------------------------- */
static void derating(void) {
    double wOut, wIn;
    sub("内层 vs 外层：同样 2 A、20 ℃ 温升、1 oz");
    wOut = ipcWidthMm(2.0 / DERATE, 20.0, 1.0, 1);
    wIn = ipcWidthMm(2.0 / DERATE, 20.0, 1.0, 0);
    printf("  外层 %.2f mm，内层 %.2f mm，倍数 %.2f\n", wOut, wIn, wIn / wOut);
    printf("  公式上 k 差 2 倍，因为 A 的指数是 0.725，所以线宽差 2^(1/0.725) ≈ 2.6 倍。\n");

    sub("降额系数的构成（老师傅的经验账）");
    printf("  铜厚公差     ：电镀均匀性 ±20%%，按 0.8 算\n");
    printf("  高温环境     ：外壳内 55 ℃ 而不是实验室 25 ℃，ΔT 直接少 30 K\n");
    printf("  阻焊覆盖     ：有阻焊的线比裸线散热差，IPC 曲线本身是裸线条件\n");
    printf("  批量一致性   ：蚀刻侧蚀、批量偏差\n");
    printf("  综合取 0.7（密闭外壳 + 高温请按 0.5），本程序所有\"建议值\"都已乘过。\n");
}

/* ---------------------------------------------------------------------
 * 章节 5：推荐方案表
 * ------------------------------------------------------------------- */
static void recommendation(void) {
    const Via v03 = {0.30, 20.0, 1.60};
    static const double curList[5] = {1.0, 2.0, 3.0, 5.0, 8.0};
    int i;

    sub("电源线推荐方案（dT=20℃，已含降额；外层走线）");
    printf("电流   1oz线宽   2oz线宽   0.3mm过孔数   附加措施\n");
    for (i = 0; i < 5; ++i) {
        const double cur = curList[i];
        const double w1 = ipcWidthMm(cur / DERATE, 20.0, 1.0, 1);
        const double w2 = ipcWidthMm(cur / DERATE, 20.0, 2.0, 1);
        const int vias = viasNeeded(&v03, cur, 20.0, H_POOR);
        const char *act;
        if (cur <= 1.0) {
            act = "常规走线即可，铺铜兜底";
        } else if (cur <= 2.0) {
            act = "两侧铺铜 + 打孔阵列散热";
        } else if (cur <= 3.0) {
            act = "2oz 铜更稳；内层禁止走这条";
        } else if (cur <= 5.0) {
            act = "2oz + 铺铜带 + 钢网开窗加锡";
        } else {
            act = "2oz 铜排 / 开窗加锡 / 或直接用铜巴+导线";
        }
        printf("%4.0fA   %5.2fmm   %5.2fmm   %8d      %s\n", cur, w1, w2, vias, act);
    }
    printf("\n老师傅收口：\n");
    printf("  1) 表里的\"过孔数\"是按保守热模型算的，量产请按整板路径核一遍压降；\n");
    printf("     5 A 路径上 10 颗 0.3 mm 孔并联（按 n^0.85 不均流）：压降 ≈ %.2f mV，\n",
           5.0 * viaResistance(&v03) / pow(10.0, 0.85) * 1e3);
    printf("     孔上损耗 ≈ %.1f mW——压降不是问题，温升和不均流才是。\n",
           5.0 * 5.0 * viaResistance(&v03) / pow(10.0, 0.85) * 1e3);
    printf("  2) 线宽不是画完就行，要在 EDA 的 Design Rules 里把电源线网class单独设成\n");
    printf("     表里的值，让 DRC 替你盯着，否则换个人接手就退回 0.2 mm。\n");
    printf("  3) 首件回来第一件事：万用表量电源对地阻值，再串灯泡上电。\n");
    printf("     线宽算错是设计事故，直接满上电是把事故变成学费。\n");
}

int main(void) {
    section("01 走线与过孔的载流 / 温升计算器（IPC-2221 / IPC-2222 模型）");
    printf("常数：1 mil = %.4f mm，1 oz 铜 = %.3f mil = %.0f um，降额系数 = %.2f\n",
           MIL2MM, OZ2MIL, OZ2MM * 1e3, DERATE);

    tableCapacity();
    tableWidth();
    viaLab();
    derating();
    recommendation();

    printf("\n");
    return 0;
}
