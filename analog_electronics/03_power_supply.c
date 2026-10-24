/* ================================================================
 *  03_power_supply.c —— 电源：LDO 与 Buck 的选型账本
 * ================================================================
 *
 *  【这个程序讲什么】
 *  徒弟，"3.3V 电源"这四个字，你在原理图上写一次，
 *  物理世界就向你收一次账：要么收在热上，要么收在铜和磁芯上，
 *  要么收在噪声上。三笔账你只能选一笔，不能赖掉。
 *
 *  这个算盘把三笔账都算出来（纯 C 实现，无任何第三方库）：
 *    ① LDO 的账：效率 = Vout/Vin（跟芯片厂一点关系都没有），
 *       压差 × 电流全部变成热，Tj = Ta + P·θja，
 *       同一颗 die 换个封装，结论从"能忍"变成"着火"；
 *    ② Buck 的账：D = Vout/Vin、ΔI_L = (Vin−Vout)·D/(f·L)、
 *       I_peak = Iout + ΔI/2、纹波 = ΔI·ESR + ΔI/(8·f·Cout)、
 *       效率 = 1 − (传导 + 开关 + 驱动 + 磁损)/Pout；
 *    ③ 电感的账：同样 33µH，铁氧体 0.9A 饱和就"跳崖"，
 *       铁硅铝 3.1A 才缓降——I_sat 那一行是磁件的命门。
 *
 *  ★ 为什么 LDO 的压差不是"越小越省钱"？★
 *  因为压差决定的是热，不是钱。12V→3.3V 500mA，压差 8.7V，
 *  P = 8.7 × 0.5 = 4.35W。SOT-223 的 θja 按裸板 JEDEC 约 105℃/W
 *  （铺铜很好也许能压到 80℃/W），结温要升 457℃（乐观也 348℃）——
 *  芯片 150℃ 就热关断，你连"效率低"这个机会都没有。
 *  更阴的是：热关断后温度降下来它又开，于是板子上出现一个
 *  几百赫兹的"打嗝"，你拿万用表量到的是一个忽高忽低的电压，
 *  你会去查运放、查参考、查 PCB，查一天发现是自己在烤自己。
 *  所以 LDO 只适合三种场景：压差小、电流小、或者需要它的低噪声
 *  （ADC 参考、PLL、射频）。它不是"便宜的 Buck 替代品"。
 *
 *  ★ 为什么 Buck 的电感感值不是越大越好？★
 *  ΔI_L = (Vin−Vout)·D/(f·L)：L 越大纹波越小，听起来只有好处。
 *  但同样感值、同样饱和电流的电感，感值靠更多匝数得到 →
 *  绕组铜损上升、体积上升、寄生电容上升（EMI 更差）、价格上升。
 *  行业默认 ΔI_L = 20%~40%·Iout：这是"效率—体积—成本"的折中，
 *  不是随便取的。而频率方向相反：f 越高 → L 越小 → 体积小、
 *  动态快，但每一周期都要付一次开关损耗（Qg·Vin·f + 桥臂交叠），
 *  500kHz 以上效率断崖，且栅极驱动电流直接从 Vin 抠走、
 *  在输入回路上拉出地弹。选频率就是在选你付哪一笔账。
 *
 *  【怎么编译运行】
 *    gcc -std=c17 -O2 -Wall -Wextra 03_power_supply.c -o 03_power.exe
 *    运行前先在控制台执行： chcp 65001     ← 不做这一步中文全是乱码
 *    再运行： .\03_power.exe
 *    （程序内部也会调用 SetConsoleOutputCP(65001) 兜底，
 *      但 chcp 65001 管的是输入侧与某些终端的字体，手动执行最稳。）
 *
 *  【预期看到什么（六段）】
 *    ① 八个工况的 LDO 热账：三个"能忍"、两个"热关断"、一个"必须换 Buck"；
 *    ② 同一个 12V→3.3V/1A 用六个开关频率扫：效率两头低中间高，
 *       感值和体积一路往下，开关损耗一路往上；
 *    ③ 四种电感材料对比：同样 33µH，饱和电流差 3 倍，成本差 5 倍；
 *    ④ Buck 的输出电容账：容值纹波 vs ESR 纹波，以及"电容 ESL
 *       谐振之后它已经是电感"这件事；
 *    ⑤ LDO PSRR 随频率塌方 + 两级 RC 后级滤波，ADC 前端该怎么喂；
 *    ⑥ 选型清单：带 20% 降额的结论表，以及"什么时候两者都不行"。
 *
 *  【老师傅点拨】
 *    看到"热关断"别以为安全。热关断救的是芯片，救不了你的产品——
 *    输出电压在打嗝，ADC 在采垃圾，客户退回来的板子上那颗 LDO
 *    外观完好无损，你会怀疑人生。热算要算最坏：最高环温 + 密闭壳 +
 *    最小铜皮（生产时省铜皮的正是同一批人）。
 *    另一个：Buck 布局先问"热回路在哪"。电流从输入电容 → 上管 →
 *    电感 → 负载 → 下管 → 回到电容，这个环路的面积就是天线面积。
 *    输入电容离开关节点超过 5mm，你测到的 EMI 就是它辐射的，
 *    跟电感选型没有半毛钱关系。
 * ================================================================ */

#include <math.h>
#include <stdio.h>
#include <string.h>

#ifdef _WIN32
#include <windows.h>
#endif

/* -std=c17 严格模式下 M_PI 不一定可见，自己定义，别依赖扩展 */
#define PI 3.14159265358979323846

static void setup_console_utf8(void)
{
#ifdef _WIN32
    SetConsoleOutputCP(65001);
#endif
}

/* ----------------------------------------------------------------
 * 中文对齐：printf 的 %-Ns 按"字节"补空格，中文占 3 字节却只显 2 列，
 * 所以中文表格必然错位。这个函数按"显示宽度"补空格。
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

static void hr(const char *title)
{
    printf("\n=== %s %s\n", pad(title, 90),
           "===============================================");
}

/* ----------------------------------------------------------------
 * 封装热阻库（θja：结到环境，含封装本体 + 焊盘 + 一小片铜皮）
 * 真实数值强烈取决于铜皮面积与风速，这里取"常规两面板、1in² 铜皮"
 * 的量级，够你判断"能不能用"，不够你签最终 BOM。
 * ---------------------------------------------------------------- */
typedef struct {
    const char *name;
    double theta_ja;   /* ℃/W */
    double note;       /* 说明用的铜皮条件（℃/W 的可信度全靠这个） */
} Pkg;

static const Pkg pkgs[4] = {
    {"SOT-23-5", 280.0, 1.0},
    {"SOT-223",  105.0, 1.0},
    {"MSOP-8",   190.0, 1.0},
    {"TO-263",    40.0, 2.0},
};

/* ② 算出来的三个电流数字要留给后面几段用（电感、电容都靠它们选型） */
double g_ipk = 0.0, g_irms = 0.0, g_dill = 0.0;

/* ----------------------------------------------------------------
 * ① LDO 的账
 *   P_diss = (Vin - Vout) * Iout + Iq * Vin
 *   Tj     = Ta + P_diss * θja
 *   热阻降额（JEDEC）：长期可靠性按 Tj ≤ 105℃ 设计，别按 150℃ 热关断。
 * ---------------------------------------------------------------- */
typedef struct {
    double vin, vout, iout, ta;
    int pkg;
    const char *what;
} LdoCase;

static void part1_ldo_thermal(void)
{
    static const LdoCase cs[8] = {
        {5.0,  3.3, 0.10, 55.0, 1, "5V→3.3V 100mA SOT-223"},
        {5.0,  3.3, 1.00, 55.0, 3, "5V→3.3V 1A TO-263"},
        {8.0,  3.3, 0.10, 55.0, 1, "8V→3.3V 100mA SOT-223"},
        {8.0,  3.3, 0.50, 55.0, 1, "8V→3.3V 500mA SOT-223"},
        {12.0, 3.3, 0.50, 55.0, 1, "12V→3.3V 500mA SOT-223"},
        {3.3,  1.8, 0.20, 55.0, 0, "3.3V→1.8V 200mA SOT-23-5"},
        {5.0,  3.3, 0.15, 55.0, 0, "5V→3.3V 150mA SOT-23-5"},
        {24.0, 5.0, 0.05, 55.0, 1, "24V→5V 50mA（耐压超）"},
    };
    const double kTjDesign = 105.0;   /* 长期降额线 */
    const double kTjTrip = 150.0;     /* 热关断线 */
    int i;

    hr("① LDO：压差 × 电流 = 热，效率跟芯片厂没有半点关系");

    printf("  规则一：LDO 是线性器件，效率 = Vout/Vin（忽略 µA 级静态电流）。\n");
    printf("  规则二：省下来的那部分电压，100%% 变成热量，一寸铜皮都不会少。\n");
    printf("  规则三：Tj = Ta + P·θja。105℃ 是长期降额线，150℃ 是热关断线，\n");
    printf("          拿 150℃ 当设计目标是新手最容易犯的致命错误。\n\n");

    printf("  %s%9s %8s %8s %9s %10s %10s %10s   %s\n",
           pad("工况", 20), "Vin", "Vout", "Iout", "η_%", "P_W",
           "θja", "Tj_℃", "verdict");
    printf("  -------------------------------------------------------------------"
           "-------------------\n");

    for (i = 0; i < 8; ++i) {
        const LdoCase *c = &cs[i];
        const Pkg *p = &pkgs[c->pkg];
        double eta = 100.0 * c->vout / c->vin;
        double pd = (c->vin - c->vout) * c->iout + 60e-6 * c->vin;   /* 60µA 静态电流 */
        double rise = pd * p->theta_ja;
        double tj = c->ta + rise;
        const char *verdict;
        if (c->vin > 20.0) verdict = "输入超耐压，先死的是它";
        else if (tj > kTjTrip) verdict = "热关断→打嗝，不可用";
        else if (tj > kTjDesign) verdict = "超降额，寿命/漂移买单";
        else if (tj > kTjDesign - 10.0) verdict = "贴着 105℃ 降额线，必须铺铜/降温";
        else if (tj > c->ta + 0.5 * (kTjDesign - c->ta)) verdict = "余量已用一半，留热余量";
        else verdict = "OK";
        printf("  %s%9.2f %8.2f %8.0f %9.1f %10.3f %10.0f %10.1f   %s\n",
               pad(c->what, 20), c->vin, c->vout, c->iout * 1e3, eta, pd,
               p->theta_ja, tj, verdict);
    }

    printf("\n  看懂四件事：\n");
    printf("    · 效率那一列（66%%、41%%、28%%）是物理定律，不是芯片水平差。\n");
    printf("      12V→3.3V 时 72%% 的输入功率在 LDO 里变成热：买再贵的 LDO 也一样。\n");
    printf("    · 同一颗 die：SOT-23-5 是 280℃/W，TO-263 是 40℃/W，差 7 倍。\n");
    printf("      所以「这颗 LDO 只能出 150mA」的说法是错的——\n");
    printf("      能出多少电流的是你的封装 + 铜皮 + 环温，芯片本身能出 1A。\n");
    printf("    · 24V 那行：常规 LDO 耐压 30V 看着够，但汽车抛负载、\n");
    printf("      电机反电动势一冲就是 60V+。耐压要按最坏瞬态算，不按标称。\n");
    printf("    · 顺带一句 dropout：压差（dropout）是「还能不能稳压」的问题，\n");
    printf("      和「热不热」是两个独立问题。3.3V 输入 3.0V 输出用普通 NPN 料\n");
    printf("      压差 1.1V 就直接失效了；低压差要选 P沟道/电荷泵栅驱的 LDO。\n");
}

/* ----------------------------------------------------------------
 * ② Buck 的账：占空比、电感纹波、峰值电流、损耗与效率、频率扫描
 *   连续导通模式（CCM）同步/非同步降压。
 *   D = Vout/Vin（理想，忽略管压降与死区）
 *   ΔI_L = (Vin - Vout) * D / (f * L)
 *   I_peak = Iout + ΔI/2；I_rms ≈ √(Iout² + ΔI²/12)
 *   ΔVout ≈ ΔI·ESR + ΔI/(8·f·Cout)
 * ---------------------------------------------------------------- */
typedef struct {
    double vin, vout, iout, fsw, L, esr, cout;
    double rds_on_hi, rds_on_lo;   /* 上下管导通电阻（非同步时下管是二极管，用 Vd 建模） */
    double vdiode;                 /* 非同步续流二极管压降 */
    double qg;                     /* 上管栅极总电荷 (C) */
    double sync;                   /* 1 = 同步整流（下管用 MOS），0 = 肖特基续流 */
    double dcr;                    /* 电感 DCR (Ω) */
} Buck;

typedef struct {
    double d, dill, ipeak, irms, vripple, t_on;
    double p_cond, p_sw, p_drv, p_mag, p_total, p_in, eff;
    double p_lo;                   /* 下管/续流器件的导通损耗，单独留着做对比 */
} BuckResult;

static BuckResult calcBuck(const Buck *b)
{
    BuckResult r;
    double pout, irms_sw, p_hi, p_lo, p_l, i_sw, e_sw, e_osscap, e_rr;

    r.d = b->vout / b->vin;
    r.t_on = r.d / b->fsw;
    r.dill = (b->vin - b->vout) * r.d / (b->fsw * b->L);
    r.ipeak = b->iout + r.dill / 2.0;
    r.irms = sqrt(b->iout * b->iout + r.dill * r.dill / 12.0);
    /* 输出纹波：ESR 阶跃 + 容值斜坡，二者峰值不同相，工程上直接相加偏保守 */
    r.vripple = r.dill * b->esr + r.dill / (8.0 * b->fsw * b->cout);

    pout = b->vout * b->iout;
    irms_sw = r.irms * sqrt(r.d);                    /* 流过上管电流的有效值 */
    p_hi = irms_sw * irms_sw * b->rds_on_hi;
    if (b->sync > 0.5) {
        double irms_lo = r.irms * sqrt(1.0 - r.d);
        p_lo = irms_lo * irms_lo * b->rds_on_lo;
    } else {
        p_lo = b->vdiode * b->iout * (1.0 - r.d);    /* 二极管导通损耗 ≈ Vd·Iout·(1-D) */
    }
    p_l = r.irms * r.irms * b->dcr;                  /* 电感铜损 */
    r.p_cond = p_hi + p_lo + p_l;
    r.p_lo = p_lo;

    /* 开关损耗：桥臂交叠 + 输出节点电容充放电。非同步还要算二极管反向恢复。
     * 交叠损耗按「每个开关周期的电压×电流交叠能量」算：E ≈ 0.5·Vin·I·(tr+tf)，
     * 小功率同步 Buck 的 tr+tf 典型 ~20ns（是开关沿时间，不是 t_on = D/fsw！
     * 拿 t_on 当交叠时间会让开关损耗与频率无关，频率扫描的账就全错了）。 */
    i_sw = b->iout;
    e_sw = 0.5 * b->vin * i_sw * 20e-9;              /* 每周期交叠能量（tr+tf≈20ns） */
    e_osscap = 0.5 * b->vin * 300e-12 * b->vin;      /* 开关节点寄生电容每周期充放 */
    e_rr = (b->sync > 0.5) ? 0.0 : 20e-9 * b->vin;  /* 肖特基 Qrr ≈20nC（0.3µC 是 15 倍夸大） */
    r.p_sw = (e_sw + e_osscap + e_rr) * b->fsw;

    r.p_drv = b->qg * b->vin * b->fsw;               /* 栅驱电荷直接从 Vin 抠走 */
    r.p_mag = 0.012 * pout;                          /* 磁芯损耗按 1.2% 输出功率估 */

    r.p_total = r.p_cond + r.p_sw + r.p_drv + r.p_mag + 0.0025;  /* +2.5mW 控制 IC 静态 */
    r.p_in = pout + r.p_total;
    r.eff = 100.0 * pout / r.p_in;
    return r;
}

static void part2_buck(void)
{
    static const double e24[12] = {10, 12, 15, 18, 22, 27, 33, 39, 47, 56, 68, 82};
    static const double fsw[6] = {100e3, 200e3, 500e3, 750e3, 1.5e6, 2.5e6};
    Buck b = {12.0, 3.3, 1.0, 500e3, 33e-6, 20e-3, 22e-6,
              0.045, 0.035, 0.45, 4.0e-9, 1.0, 0.060};
    BuckResult r;
    Buck b2;
    BuckResult r2;
    int i;

    hr("② Buck：D、ΔI、峰值电流、效率——每笔账都要落在具体元件上");

    printf("  工况：Vin=12V，Vout=3.3V，Iout=1.0A，fsw=%.0fkHz，L=%.0fµH，"
           "Cout=%.0fµF/ESR=%.0fmΩ，同步整流\n",
           b.fsw / 1e3, b.L / 1e-6, b.cout / 1e-6, b.esr * 1e3);
    r = calcBuck(&b);

    printf("\n  ---- 稳态波形量出来的数字 ----\n");
    printf("  占空比 D = Vout/Vin              = %.3f（%.1f%%）\n", r.d, r.d * 100.0);
    printf("  上管导通时间 t_on = D/fsw        = %.1f ns\n", r.t_on * 1e9);
    printf("  电感纹波 ΔI_L                    = %.3f A（%.0f%% 负载）\n",
           r.dill, 100.0 * r.dill / b.iout);
    printf("  电感峰值电流 I_peak              = %.3f A ← 电感的 I_sat 必须大于它\n",
           r.ipeak);
    printf("  电感有效值电流 I_rms             = %.3f A ← 铜损用这个算\n", r.irms);
    printf("  （本例 33µH 的 ΔI 只有 %.0f%%——低于常说的 20~40%%，代价是磁件更大；\n",
           100.0 * r.dill / b.iout);
    printf("    下面扫频时按 30%% 重新选感值，两笔账别混。）\n");
    printf("  开关管峰值电流                   = %.3f A ← 芯片内部限流要高于它\n",
           r.ipeak);
    printf("  输出电压纹波 ΔVout               = %.1f mV_pp（其中 ESR 项 %.1fmV）\n",
           r.vripple * 1e3, r.dill * b.esr * 1e3);
    printf("  二极管/下管承受的反压             = %.1f V（=Vin，含尖峰再加 20%%）\n",
           b.vin);

    printf("\n  ---- 损耗分解（这就是效率的账本）----\n");
    printf("  %s %12s %10s\n", pad("损耗项", 26), "P_mW", "占比_%");
    printf("  -------------------------------------------------\n");
    printf("  %s %12.1f %10.1f\n", pad("传导（上管+下管+电感DCR）", 26),
           r.p_cond * 1e3, 100.0 * r.p_cond / r.p_total);
    printf("  %s %12.1f %10.1f\n", pad("开关（交叠+节点电容）", 26),
           r.p_sw * 1e3, 100.0 * r.p_sw / r.p_total);
    printf("  %s %12.1f %10.1f\n", pad("栅极驱动 Qg·Vin·f", 26),
           r.p_drv * 1e3, 100.0 * r.p_drv / r.p_total);
    printf("  %s %12.1f %10.1f\n", pad("磁芯损耗", 26),
           r.p_mag * 1e3, 100.0 * r.p_mag / r.p_total);
    printf("  %s %12.1f %10.1f\n", pad("控制 IC 静态", 26),
           2.5, 100.0 * 0.0025 / r.p_total);
    printf("  效率 = %.1f%%（Pout=%.2fW，总损耗=%.0fmW）\n",
           r.eff, b.vout * b.iout, r.p_total * 1e3);

    printf("\n  ---- 非同步（肖特基续流）对比：同样工况，下管换成 0.45V 二极管 ----\n");
    b2 = b;
    b2.sync = 0.0;
    r2 = calcBuck(&b2);
    printf("  非同步效率 %.1f%% vs 同步 %.1f%%：低电压大电流时二极管那 %.0fmW 就是命门。\n",
           r2.eff, r.eff, (r2.p_lo - r.p_lo) * 1e3);
    printf("  但 Vout 高（≥12V）小电流时，同步整流的开销（静态+驱动）反而不划算。\n");

    printf("\n  ---- 开关频率扫描：你付哪一笔账？----\n");
    printf("  %s %10s %10s %10s %10s %9s %10s   %s\n",
           pad("fsw", 10), "L_µH", "ΔI_%", "I_pk_A", "dV_mV",
           "P_sw_mW", "η_%", "备注");
    printf("  -------------------------------------------------------------------\n");
    for (i = 0; i < 6; ++i) {
        Buck t = b;
        double L, ln, pick, mag;
        BuckResult rr;
        int k;
        t.fsw = fsw[i];
        /* 每个频率重新按「纹波电流 = 30% 负载」选电感 */
        L = (b.vin - b.vout) * (b.vout / b.vin) / (t.fsw * 0.3 * b.iout);
        /* 归到 E24 系列：先把 µH 值归一化到 [10,100) 这个十倍频程，
         * 再向上取最近的系列值——向上取是为了纹波只小不超 30%。
         * （不先归一化，3~80µH 的计算值跟 10~82 的表根本对不上号。） */
        ln = L / 1e-6; /* µH */
        mag = 1.0;
        while (ln >= 100.0) { ln /= 10.0; mag *= 10.0; }
        while (ln < 10.0)   { ln *= 10.0; mag /= 10.0; }
        pick = e24[11];
        for (k = 0; k < 12; ++k)
            if (e24[k] >= ln) { pick = e24[k]; break; }
        t.L = pick * mag * 1e-6;
        rr = calcBuck(&t);
        printf("  %9.0fk %10.1f %10.1f %10.3f %10.2f %9.1f %10.1f   L=%.4gµH %s\n",
               fsw[i] / 1e3, t.L / 1e-6, 100.0 * rr.dill / b.iout, rr.ipeak,
               rr.vripple * 1e3, rr.p_sw * 1e3, rr.eff, t.L / 1e-6,
               (fsw[i] > 1.2e6) ? "小但费开关损耗/EMI 更凶"
                                : ((fsw[i] < 200e3) ? "效率好但磁件巨大" : "甜点区"));
    }

    printf("\n  师傅判词：\n");
    printf("    · 效率曲线是两头低中间高：低频输给磁损与体积，高频输给开关损耗。\n");
    printf("      别在曲线上挑最高那个点，要挑「磁件买得到 + EMI 过得了 + 芯片支持」的点。\n");
    printf("    · ΔI 取 30%% 是行业折中，不是自然律：\n");
    printf("      更小 → 电感体积/成本上去了；更大 → 峰值电流、铜损、输出纹波都上去。\n");
    printf("    · 记住一句：Vin 越高、Vout 越低，D 越小，电感纹波电流越凶\n");
    printf("      （ΔI ∝ (Vin−Vout)·D = (Vin−Vout)·Vout/(Vin·f·L)），\n");
    printf("      24V→3.3V 的峰值电流会比 5V→3.3V 大得多，芯片限流也会先撞。\n");
}

/* ----------------------------------------------------------------
 * ③ 电感材料：同样 33µH，四条 I-U 曲线的脾气
 *   饱和不是"慢慢变软"就是好，而是"悬崖式跳崖"最危险。
 * ---------------------------------------------------------------- */
typedef struct {
    const char *mat;
    double l, isat, irms, dcr;
    const char *size;
    double cost;
    const char *temper;
} Ind;

static void part3_inductor(void)
{
    static const Ind inds[4] = {
        {"铁氧体鼓形",     33.0, 0.90, 0.70, 90.0,  "7.3×7.3×3.5", 0.35, "悬崖式：过 I_sat 感值瞬间掉 60%"},
        {"一体成型铁粉",   33.0, 1.70, 1.50, 55.0,  "6.0×6.0×3.0", 0.85, "较缓：分布气隙，感值渐进下降"},
        {"铁硅铝 Sendust", 33.0, 3.10, 2.60, 42.0,  "11×11×6.0",   1.90, "极软：软饱和，最抗 DC 偏磁"},
        {"空心/磁屏蔽线绕", 33.0, 99.0, 1.80, 210.0, "10×10×8.0",  2.60, "无饱和：但体积大、自谐振低"},
    };
    static const char *const verdict[4] = {
        "超 I_sat 19%，且 I_rms 0.70A < 1.00A：温升先超，别用",
        "余量 58%：可用（但要确认高温下 I_sat 还会再降 10~20%）",
        "余量 189%：绰绰有余，代价是体积和钱",
        "无饱和之虞：但 DCR 210mΩ 的铜损会自己发热",
    };
    int i;

    hr("③ 电感：同感值不同材料，饱和曲线的脾气差 3 倍");

    printf("  选型只看两行：I_sat（感值掉 30%% 的电流）和 I_rms（温升 40℃ 的电流）。\n");
    printf("  这两行不是「最大允许值」，是「开始出事」的门槛。\n\n");

    printf("  %s %9s %10s %10s %9s %10s %8s   %s\n",
           pad("磁芯材料", 16), "感值_µH", "I_sat_A", "I_rms_A",
           "DCR_mΩ", "尺寸_mm", "价格", "饱和曲线脾气");
    printf("  -------------------------------------------------------------------"
           "-----------------\n");
    for (i = 0; i < 4; ++i) {
        printf("  %s %9.1f %10.2f %10.2f %9.0f %10s %8.2f   %s\n",
               pad(inds[i].mat, 16), inds[i].l, inds[i].isat, inds[i].irms,
               inds[i].dcr, inds[i].size, inds[i].cost, inds[i].temper);
    }

    printf("\n  把 ② 算出的 I_peak = 1.073A 落到这四只料上：\n");
    for (i = 0; i < 4; ++i)
        printf("    · %s：%s\n", pad(inds[i].mat, 16), verdict[i]);

    printf("\n  师傅判词：\n");
    printf("    · 电感饱和是电源里最难查的故障，因为它「一开始是好的」：\n");
    printf("      常温 0.9A 余量看着够，夏天壳内 70℃ 时磁导率再降，\n");
    printf("      某个负载突变瞬间感值掉一半 → ΔI 翻倍 → 更热 → 彻底塌。\n");
    printf("      现象是「重载才重启」「跑十分钟才出问题」，你查代码查到想辞职。\n");
    printf("    · 手册上 I_sat 的定义每家不同（掉 10%%/30%%/到膝盖），\n");
    printf("      要问清楚，或者干脆自己拿 LCR 表测「电流 vs 感值」曲线。\n");
    printf("    · 磁屏蔽 ≠ 低 EMI 的保命符，它只是让你邻板少挨打；\n");
    printf("      真正的辐射源是热回路面积和开关节点的 dV/dt。\n");
}

/* ----------------------------------------------------------------
 * ④ 输出电容：容值纹波、ESR 纹波、以及"过了 ESL 谐振它就是电感"
 * ---------------------------------------------------------------- */
typedef struct {
    const char *kind;
    double c, esr, esl, ripples;
} Cap;

static void part4_output_cap(void)
{
    static const Cap caps[5] = {
        {"铝电解 22µF",  22e-6, 0.90,  12e-9, 0.0},
        {"低ESR铝电解",  22e-6, 0.090, 12e-9, 0.0},
        {"陶瓷 X5R 22µF", 22e-6, 0.012, 0.7e-9, 0.0},
        {"陶瓷 X5R 10µF", 10e-6, 0.018, 0.6e-9, 0.0},
        {"陶瓷 X7R 4.7µF", 4.7e-6, 0.025, 0.5e-9, 0.0},
    };
    double dill = 0.145;  /* 沿用 ② 的纹波电流（33µH @500kHz） */
    double fsw = 500e3;
    int i;

    hr("④ 输出电容：容值、ESR、ESL 三笔分开算，别只写一个 µF");

    printf("  纹波电流 ΔI_L = %.2f A，开关频率 %.0f kHz。三项分别是：\n", dill, fsw / 1e3);
    printf("    容值纹波 = ΔI/(8·f·C)   ESR 纹波 = ΔI·ESR   ESL 尖峰 = ΔI·ESL·fsw（估）\n\n");

    printf("  %s %12s %12s %12s %12s   %s\n",
           pad("电容", 20), "容值_mV", "ESR_mV", "ESL_mV", "合计_mV", "备注");
    printf("  -------------------------------------------------------------------\n");
    for (i = 0; i < 5; ++i) {
        double dv_c = dill / (8.0 * fsw * caps[i].c) * 1e3;
        double dv_e = dill * caps[i].esr * 1e3;
        double dv_l = dill * caps[i].esl * fsw * 1e3;
        printf("  %s %12.2f %12.2f %12.2f %12.2f   %s\n",
               pad(caps[i].kind, 20), dv_c, dv_e, dv_l, dv_c + dv_e + dv_l,
               (caps[i].esr > 0.3) ? "ESR 主导：典型电解的坑"
                                   : ((caps[i].c < 10e-6) ? "容值项偏大，需并联大容量"
                                                          : "三项都小，好选择"));
    }

    printf("\n  还有两件必须知道的事：\n");
    printf("    1) 陶瓷电容会「变没」。X5R 22µF 在偏压 12V、温度 85℃ 时，\n");
    printf("       有效容值常常只剩标称的 30~40%%（直流偏压效应 + 温度）。\n");
    printf("       手册上有「容值 vs 偏压」曲线——那才是你的电容。\n");
    printf("       所以工程上按 worst-case 有效容值算纹波，并按 2 只并联摊 ESL/ESR。\n");
    printf("    2) 每个电容都在某频率自谐振（f_sr = 1/(2π√(LC))），\n");
    printf("       过了那一点它的阻抗随频率上升——它已经是电感了。\n");
    printf("       所以「22µF + 100nF」并联不是玄学：100nF 的 f_sr 高，\n");
    printf("       负责 MHz 以上那段（正好是开关沿和谐振所在频段）。\n");
    printf("       Buck 输入端同理：必须有一只低 ESL 小电容离开关节点 < 3mm。\n");
}

/* ----------------------------------------------------------------
 * ⑤ LDO 的高频账：PSRR 塌方 + 两级 RC 后级滤波（ADC/PLL 怎么喂）
 *   PSRR(f)：直流很高，随频率按 ~ -20dB/dec 往下掉，
 *   到开关频率处常常只剩 20~30dB；Buck 的纹波就漏在这里。
 * ---------------------------------------------------------------- */
typedef struct { double f; double psrr; } Frow;

static void part5_ldo_noise(void)
{
    static const Frow rows[6] = {{100.0, 72.0}, {1e3, 66.0}, {10e3, 52.0},
                                 {100e3, 36.0}, {500e3, 22.0}, {1e6, 16.0}};
    /* 两级 RC：每级 R=10Ω, C=10µF → 单级 fc≈1.6kHz，在 500kHz 处约 -49dB/级 */
    double r_res = 10.0, c_cap = 10e-6;
    double fc = 1.0 / (PI * 2.0 * r_res * c_cap);
    int i;

    hr("⑤ LDO 的低噪声是「便宜」的：PSRR 随频率塌方，靠后级 RC 补");

    printf("  场景：Buck 12V→5V（输出纹波 30mV_pp，开关频率 500kHz）→\n");
    printf("        LDO 5V→3.3V 给 16 位 ADC 供电。ADC 电源噪声预算 100µV。\n\n");

    printf("  %s %12s %14s %14s %14s\n",
           pad("频率", 12), "PSRR_dB", "漏到输出_µV", "RC 后_µV", "余量");
    printf("  -------------------------------------------------------------------\n");
    for (i = 0; i < 6; ++i) {
        double ripple_in = 30e-3;                    /* Buck 输出纹波（V_pp） */
        double leak, atten, after;
        const char *m;
        if (rows[i].f < 1e3) ripple_in = 1e-3;       /* 低频段 Buck 本身噪声小 */
        leak = ripple_in * pow(10.0, -rows[i].psrr / 20.0);
        atten = pow(1.0 / sqrt(1.0 + pow(rows[i].f / fc, 2)), 2);
        after = leak * atten;
        m = (after < 100e-6) ? "满足 100µV 预算" : "超预算";
        printf("  %10.0f Hz %12.0f %14.1f %14.1f   %s\n",
               rows[i].f, rows[i].psrr, leak * 1e6, after * 1e6, m);
    }

    printf("\n  师傅判词：\n");
    printf("    · 关键一行是 500kHz：PSRR 只剩 22dB。LDO 的数据表第一行写\n");
    printf("      「PSRR = 70dB @100Hz」，那行小字是给你看广告的，\n");
    printf("      真正有用的是那张「PSRR vs 频率」的图，多数人不看。\n");
    printf("    · 所以低噪声前端的正确姿势不是「换个贵 LDO」，而是：\n");
    printf("      (a) 两级 RC（或 LC π 型）后级滤波——它在你关心的频率上干活；\n");
    printf("      (b) LDO 前后各留一只独立的地 Return，别共一段地；\n");
    printf("      (c) 或者干脆给模拟供电单独用一只变压器绕组 / 独立 DCDC 错峰。\n");
    printf("    · 代价要算清：两级 10Ω 电阻在 200mA 时压降 4V——你的 LDO 输出\n");
    printf("      得先给到 7V 再降，热账立刻回来了。低噪声从来不是免费的。\n");
    printf("    · 铁律：算噪声要在「噪声实际所在的频率」上算，\n");
    printf("      在 100Hz 上算 PSRR 来救 500kHz 的纹波，是自我安慰。\n");
}

/* ----------------------------------------------------------------
 * ⑥ 选型清单（带降额）
 * ---------------------------------------------------------------- */
typedef struct { const char *what; double want; double derate; const char *note; } ChkItem;

static void part6_checklist(void)
{
    static const ChkItem items[8] = {
        {"LDO 输入电压",   24.0, 0.80, "按最坏瞬态（抛负载/反电动势）取，不按标称"},
        {"LDO 输出电流",    0.5, 0.80, "芯片标 1A ≠ 你的封装能出 1A，先看热"},
        {"LDO 功耗",        0.85, 0.80, "Tj≤105℃ 反推出来的允许功耗"},
        {"Buck 输入电压",  24.0, 0.85, "芯片耐压留 15%，尖峰另算（加 TVS）"},
        {"电感 I_sat",     1.073, 0.80, "峰值电流留 20%，高温再打折"},
        {"电感 I_rms",     1.001, 0.80, "有效值电流，决定温升不是饱和"},
        {"输出电容耐压",    3.3, 0.50, "陶瓷 50% 降额是硬规矩，偏压还要打折"},
        {"续流管反压",     12.0, 0.75, "=Vin + 尖峰，别按 1.1 倍侥幸"},
    };
    int i;

    hr("⑥ 选型清单：带 20% 降额的结论，以及「两者都不行」的时候");

    printf("  %s %14s %12s %14s   %s\n",
           pad("要降额的量", 18), "最坏值", "降额系数", "选型下限", "备注");
    printf("  -------------------------------------------------------------------\n");
    for (i = 0; i < 8; ++i) {
        double limit = items[i].want / items[i].derate;
        printf("  %s %14.2f %12.2f %14.2f   %s\n",
               pad(items[i].what, 18), items[i].want, items[i].derate,
               limit, items[i].note);
    }

    printf("\n  决策树（背下来，比记芯片型号有用）：\n");
    printf("    1) Vin/Vout > 1.6 或 (Vin−Vout)·Iout > 0.5W  →  排除 LDO，看 Buck。\n");
    printf("    2) 压差小 + 电流小 + 要低噪声（ADC 参考、PLL、射频、比较器）→ LDO。\n");
    printf("    3) Buck 开关频率与 ADC 采样时钟有整数倍关系 → 会混叠进信号带；\n");
    printf("       要么同步两者频率，要么错开，要么给模拟单独一路。\n");
    printf("    4) 要负压 / 升压 / 输入输出交叠 → Buck 不行，看 Buck-Boost 或电荷泵。\n");
    printf("    5) 需要 µV 级噪声（如低相噪 VCO） → 两者都不行：\n");
    printf("       电池 / 线性电源 + RC/LC 多级滤波 + 有源滤波，钱和体积都要给。\n");
    printf("    6) 效率目标 > 90%% 且压差 > 3V → 只有 Buck；\n");
    printf("       同时接受它的三个账单：电感体积、开关节点 EMI、输出纹波。\n");

    printf("\n  最后一条：电源是「系统级」问题，不是「一颗芯片」问题。\n");
    printf("    同一片 LDO，热回路面积不同、输入电容位置不同，\n");
    printf("    结果可以从「过不了 EMC」到「安安稳稳跑十年」。选完芯片只是开始。\n");
}

int main(void)
{
    setup_console_utf8();
    printf("\n");
    printf("##############################################################\n");
    printf("#  第 2 关 · 模拟电路 · 03 电源选型账本\n");
    printf("#  热、铜与磁、噪声——三笔账你只能选一笔\n");
    printf("##############################################################\n");

    part1_ldo_thermal();
    part2_buck();
    part3_inductor();
    part4_output_cap();
    part5_ldo_noise();
    part6_checklist();

    printf("\n--------------------------------------------------------------\n");
    printf("本程序小结（师傅要你会背的四步检查）：\n");
    printf("  1. 先算 (Vin−Vout)·Iout。超过 0.5W 就开始谈封装，超过 1W 就谈 Buck。\n");
    printf("  2. Tj = Ta + P·θja 用 105℃ 设计线，不用 150℃ 热关断线；\n");
    printf("     环温取密闭壳内实测值，不是取 25℃。\n");
    printf("  3. Buck 三行必算：D=Vout/Vin、ΔI=(Vin−Vout)D/(fL)、I_peak=Iout+ΔI/2；\n");
    printf("     拿 I_peak 去和电感 I_sat、芯片限流比，比完再谈效率。\n");
    printf("  4. 噪声在「噪声实际所在的频率」上算：PSRR/阻抗都要查那一列，\n");
    printf("     别看数据表第一行的广告数字。\n");
    printf("--------------------------------------------------------------\n");
    return 0;
}
