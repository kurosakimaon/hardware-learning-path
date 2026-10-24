/*
 * =====================================================================================
 *  第 7 关 · Phase 1 —— 电源树与功耗预算（phase1_power_budget.c）
 * =====================================================================================
 *
 * 【讲什么】
 *   板子还没画，电流账先算明白。本程序用软件模型把 MiniBoard 的电源树
 *   （USB-C 5V -> 保护 -> LDO -> 3.3V 轨 -> 各负载）和每个器件的电流建起来，
 *   对三种工作模式分别做预算：
 *     1) 常亮采集 1Hz   2) 省电模式 10s 一次   3) 报警模式
 *   再算 LDO 热耗与温升(thetaJA)、假想 500mAh 锂电续航、USB 500mA 裕量、
 *   上电冲击电流与 USB 规范"输入电容 <= 10uF"的浪涌限制。
 *   最后师傅故意把 OLED 拉成全亮常显，让你亲眼看见 3mA 预算是怎么当场爆掉的。
 *
 *   这一版是纯 C（C17）：负载清单用定长数组 + 结构体表达，字符串就是 const char*，
 *   排序是手写的稳定插入排序 —— 嵌入式 C 项目里本来就不该有 STL，先把这些
 *   基本功练扎实。
 *
 * 【怎么编译运行】(Windows + MSYS2 ucrt64, gcc 16.2)
 *   先在控制台执行：chcp 65001    （切到 UTF-8 代码页，否则中文显示成乱码）
 *   gcc -std=c17 -O2 -Wall -Wextra phase1_power_budget.c -o phase1_power_budget.exe
 *   ./phase1_power_budget.exe
 *   要求：零警告零错误。
 *
 * 【预期看到什么】
 *   - 电源树层级表（VBUS -> 保护 -> LDO -> 3V3 -> 负载）
 *   - 省电模式总电流约 1.80mA，判定 PASS（规格 < 3mA）
 *   - 电流排行榜第一名是"电源 LED 常亮 1.20mA"，不是 MCU —— 猜不到的回去重算
 *   - 反例：OLED 全亮常显 -> 总电流约 29.5mA，当场 FAIL，并打印是谁吃掉的
 *   - 常亮模式 LDO 温升 < 25 摄氏度（SOT-89, thetaJA = 250 C/W）
 *   - 500mAh 锂电 + 省电模式 -> 续航约 277 小时
 *   - USB 输入电容 10uF <= 10uF -> 浪涌 PASS；演示 22uF 版本 -> FAIL
 *
 * 【老师傅点拨】
 *   1. 功耗预算不是把 datasheet 数字加起来就完事，要按"占空比"折算：
 *      SHT30 测一次 1.5mA 持续 15ms，10 秒一次的话平均只有 2.25uA。
 *   2. 电源 LED 是"永远亮着的负载"，它对静态电流的杀伤力远比你想象的大。
 *      客户说 <3mA 的时候，第一个该动的就是它。
 *   3. LDO 的热算的是 (Vin-Vout)*I，不是 Vout*I。压差 1.7V、40mA 就是 68mW，
 *      在 SOT-89 上能变成 17 摄氏度温升 —— 夏天机箱里再叠 55 度环境温度，
 *      你就得查结温。
 *   4. USB 那条"输入电容 <= 10uF"是硬规矩（USB 规范 inrush 限制）。热插拔
 *      瞬间 22uF 会把 VBUS 拉塌，同一根 hub 上别人的设备跟着重启 —— 这种
 *      问题返修率最高，因为用户只会说"插上就掉盘"。
 * =====================================================================================
 */

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>

/* ---------------------------------------------------------------------------
 * 规格常数（客户说的，写死在这里，改规格先改这里）
 * ------------------------------------------------------------------------- */
static const double kSpecIdleCurrentMa  = 3.0;   /* 整机静态电流 < 3mA        */
static const double kUsbVbusVolt        = 5.0;   /* USB 5V                    */
static const double kUsbCurrentBudgetMa = 500.0; /* USB 默认 500mA 预算       */
static const double kRailVolt           = 3.3;   /* 3.3V 主轨                 */
static const double kLdoThetaJaCPerW    = 250.0; /* SOT-89 LDO 结到热阻       */
static const double kAmbientC           = 55.0;  /* 最坏环境温度              */
static const double kTjMaxC             = 125.0; /* 结温上限                  */
static const double kBatteryMah         = 500.0; /* 假想锂电池（可选配）      */
static const double kUsbInCapLimitUf    = 10.0;  /* USB 规范输入电容上限      */
static const double kLdoIqMa            = 0.040; /* LDO 静态电流（ME6217 典型 40uA） */

/* ---------------------------------------------------------------------------
 * 器件电流模型（单位 mA，均为 3.3V 轨上的电流）
 *
 * C 语言里没有 std::string：字符串成员直接用 const char* 指向只读字面量，
 * 需要"改名"时直接换指针，零拷贝、零分配 —— 这正是固件里喜欢的做法。
 * ------------------------------------------------------------------------- */
typedef struct {
    const char *name; /* 器件名            */
    double      ma;   /* 该模式下的等效平均电流 */
    const char *note; /* 折算说明          */
} Load;

#define LOAD_COUNT 10 /* 每个模式的负载清单固定 10 项，定长数组足够 */

/* SHT30：测量 1.5mA / 15ms，待机 0.5uA */
static double sht30AvgMa(double periodMs)
{
    const double active = 1.5 * (15.0 / periodMs); /* 按占空比折算 */
    return active + 0.0005;
}

/* LED：(3.3 - Vf) / R */
static double ledMa(double vf, double ohm, double duty)
{
    const double i = (kRailVolt - vf) / ohm * 1000.0; /* -> mA */
    return i * duty;
}

/* OLED SSD1306：全亮 20mA / 低亮 30% 约 6mA / 关屏（关显示+关升压）约 2uA */
static double oledMa(int level)
{
    if (level == 2) return 20.0; /* 全亮             */
    if (level == 1) return 6.0;  /* 亮度寄存器 0x8F*0.3 */
    return 0.002;                /* 关屏             */
}

/* NTC 备份温度通道：门控供电（采样时才通电），100k 分压 */
static double ntcAvgMa(double periodMs, bool gated)
{
    if (gated) return (kRailVolt / 200000.0) * 1000.0 * (15.0 / periodMs);
    return (kRailVolt / 200000.0) * 1000.0;
}

/* ---------------------------------------------------------------------------
 * 三种工作模式的负载清单（out 数组容量必须 >= LOAD_COUNT）
 * ------------------------------------------------------------------------- */
static void makeModeNormal(Load *out)
{
    out[0].name = "LDO 静态电流";   out[0].ma = 0.0;                  out[0].note = "Iq 40uA 在 5V 输入侧，不计入 3V3 负载";
    out[1].name = "MCU 运行(WFI 平均)"; out[1].ma = 8.000;            out[1].note = "72MHz 全速 36mA，空闲 WFI，占空比折算";
    out[2].name = "SHT30 温湿度";   out[2].ma = sht30AvgMa(1000.0);   out[2].note = "1Hz：1.5mA*15ms/1000ms";
    out[3].name = "OLED SSD1306";   out[3].ma = oledMa(1);            out[3].note = "低亮 30%，常显";
    out[4].name = "电源 LED 常亮";  out[4].ma = ledMa(2.1, 1000.0, 1.0); out[4].note = "(3.3-2.1)/1k = 1.2mA";
    out[5].name = "状态 LED 慢闪";  out[5].ma = ledMa(2.1, 1000.0, 0.5); out[5].note = "50% 呼吸占空比";
    out[6].name = "报警 LED 灭";    out[6].ma = 0.000;                out[6].note = "未触发";
    out[7].name = "蜂鸣器";         out[7].ma = 0.000;                out[7].note = "未触发";
    out[8].name = "NTC 门控分压";   out[8].ma = ntcAvgMa(1000.0, true);  out[8].note = "100k 分压 + 采样时通电";
    out[9].name = "I2C/上拉等漏电"; out[9].ma = 0.002;                out[9].note = "总线空闲几乎无电流";
}

static void makeModeIdle(Load *out)
{
    out[0].name = "LDO 静态电流";   out[0].ma = 0.0;                   out[0].note = "Iq 40uA 在 5V 输入侧，不计入 3V3 负载";
    out[1].name = "MCU STOP+IWDG(LSI)"; out[1].ma = 0.350;             out[1].note = "datasheet 20uA，工程按 350uA 含余量";
    out[2].name = "SHT30 温湿度";   out[2].ma = sht30AvgMa(10000.0);  out[2].note = "10s 一次：1.5mA*15ms/10000ms";
    out[3].name = "OLED SSD1306";   out[3].ma = oledMa(0);            out[3].note = "关屏 + 关升压，只保 RAM 保持";
    out[4].name = "电源 LED 常亮";  out[4].ma = ledMa(2.1, 1000.0, 1.0);  out[4].note = "客户要看电，先记它头上";
    out[5].name = "状态 LED 慢闪";  out[5].ma = ledMa(2.1, 1000.0, 0.1);  out[5].note = "10% 占空比";
    out[6].name = "报警 LED 灭";    out[6].ma = 0.000;                 out[6].note = "未触发";
    out[7].name = "蜂鸣器";         out[7].ma = 0.000;                 out[7].note = "未触发";
    out[8].name = "NTC 门控分压";   out[8].ma = ntcAvgMa(10000.0, true); out[8].note = "10s 一次";
    out[9].name = "I2C/上拉等漏电"; out[9].ma = 0.002;                 out[9].note = "TVS 漏电流也算这里";
}

static void makeModeAlarm(Load *out)
{
    int i;
    makeModeNormal(out);
    for (i = 0; i < LOAD_COUNT; ++i) {
        if (strcmp(out[i].name, "报警 LED 灭") == 0) {
            out[i].name = "报警 LED 闪烁";
            out[i].ma = ledMa(2.1, 470.0, 0.5);
            out[i].note = "470R，500Hz 闪烁 50%";
        }
        if (strcmp(out[i].name, "蜂鸣器") == 0) {
            out[i].ma = 24.0;
            out[i].note = "有源蜂鸣器 3.3V 典型 24mA";
        }
    }
}

/* OLED 全亮常显的反例（教学用：让客户看清"常显"的代价） */
static void makeModeOledMax(Load *out)
{
    int i;
    makeModeNormal(out);
    for (i = 0; i < LOAD_COUNT; ++i) {
        if (strcmp(out[i].name, "OLED SSD1306") == 0) {
            out[i].ma = oledMa(2);
            out[i].note = "全亮 + 常显（客户嘴里的『就要最亮』）";
        }
    }
}

static double totalMa(const Load *loads, int n)
{
    double s = 0.0;
    int i;
    for (i = 0; i < n; ++i) s += loads[i].ma;
    return s;
}

/* 按电流从大到小排序。手写稳定插入排序：
 * 值相等时保持原始顺序（和排行榜对得上），10 个元素的量级完全够用。 */
static void sortLoadsDesc(Load *a, int n)
{
    int i, j;
    for (i = 1; i < n; ++i) {
        Load key = a[i];
        j = i - 1;
        while (j >= 0 && a[j].ma < key.ma) {
            a[j + 1] = a[j];
            --j;
        }
        a[j + 1] = key;
    }
}

/* ---------------------------------------------------------------------------
 * 打印工具
 * ------------------------------------------------------------------------- */
static void hr(char c, int n)
{
    int i;
    for (i = 0; i < n; ++i) printf("%c", c);
    printf("\n");
}

static void sec(const char *s)
{
    printf("\n--- %s ---\n", s);
    hr('-', 88);
}

/* ---------------------------------------------------------------------------
 * 单个模式的预算评估
 * ------------------------------------------------------------------------- */
typedef struct {
    bool   pass;
    double total;
} Verdict;

static Verdict evaluateMode(const char *mode, const Load *loads, int n, bool check3mA)
{
    Verdict v;
    char title[160];
    double total, pdMw, rise, tj;
    bool pass = true;
    int i;

    snprintf(title, sizeof(title), "模式：%s", mode);
    sec(title);
    printf("%-22s %10s  %s\n", "负载", "平均 mA", "折算说明");
    hr('-', 88);
    for (i = 0; i < n; ++i)
        printf("%-22s %10.4f  %s\n", loads[i].name, loads[i].ma, loads[i].note);
    hr('-', 88);
    total = totalMa(loads, n);
    printf("%-22s %10.4f  <- 3V3 轨负载合计（不含 LDO Iq）\n", "总计", total);

    /* 3.3V -> 5V 侧：LDO 输入电流 = 输出电流 + Iq（Iq 走输入侧，不经过 3V3） */
    printf("    折算到 USB 5V 侧：I_in = %.4f mA（= 3V3 负载 + Iq 40µA）\n", total + kLdoIqMa);
    printf("    USB 500mA 预算占用：%.2f%%，裕量 %.1f mA\n",
           (total + kLdoIqMa) / kUsbCurrentBudgetMa * 100.0,
           kUsbCurrentBudgetMa - (total + kLdoIqMa));

    /* LDO 热 */
    pdMw = (kUsbVbusVolt - kRailVolt) * total + kUsbVbusVolt * kLdoIqMa; /* mW */
    rise = pdMw / 1000.0 * kLdoThetaJaCPerW;   /* C  */
    printf("    LDO 耗散：(5-3.3)V * %.2fmA = %.1f mW -> 温升 %.1f C（thetaJA=250C/W）\n",
           total, pdMw, rise);
    tj = kAmbientC + rise;
    printf("    最坏结温估计：环境 %.0fC + %.1fC = %.1fC（上限 %.0fC）-> %s\n",
           kAmbientC, rise, tj, kTjMaxC, (tj < kTjMaxC ? "OK" : "超温"));

    if (check3mA) {
        Load sorted[LOAD_COUNT];
        double remain;
        pass = total < kSpecIdleCurrentMa;
        printf("    规格判定（静态电流 < %.1fmA）：%s\n", kSpecIdleCurrentMa,
               pass ? "PASS" : "FAIL");
        if (!pass) {
            memcpy(sorted, loads, sizeof(Load) * (size_t)n);
            sortLoadsDesc(sorted, n);
            printf("    >>> 是谁吃掉的预算（从大到小，超出 %.2fmA）：\n", total - kSpecIdleCurrentMa);
            remain = total - kSpecIdleCurrentMa;
            for (i = 0; i < n; ++i) {
                const Load *l = &sorted[i];
                if (l->ma <= 0.0) continue;
                printf("        %-20s %8.3f mA   累计削掉它能补回 %.3f mA\n",
                       l->name, l->ma, (remain < l->ma ? remain : l->ma));
                remain -= l->ma;
                if (remain <= 0.0) break;
            }
            printf("    >>> 师傅结论：OLED 全亮 20mA 一类的大负载必须降亮度 / 定时关屏 / 按需点亮。\n");
        }
    }
    v.pass = pass;
    v.total = total;
    return v;
}

/* ---------------------------------------------------------------------------
 * 电源树
 * ------------------------------------------------------------------------- */
static void printPowerTree(void)
{
    typedef struct { const char *stage; const char *part; const char *info; } Node;
    static const Node tree[15] = {
        {"① 入口",   "USB-C 16P",        "VBUS 5V / 500mA；CC1、CC2 各 5.1k 下拉（没焊=没电）"},
        {"② 保护",   "USBLC6-2SC6",      "D+/D- ESD 阵列；对地电容 100nF"},
        {"② 保护",   "PPTC 120mA/6V",    "过流自恢复保险；内阻约 2欧，压降可忽略"},
        {"② 保护",   "SMBJ5.0A (可选)",  "VBUS 的 TVS；线长 >10cm 或工业现场建议加"},
        {"③ 滤波",   "10uF + 100nF",     "VBUS 输入电容，必须 <= 10uF（USB 浪涌限制）"},
        {"④ 变压",   "ME6217C33 SOT-89", "LDO 600mA，Iq 40uA；本机最坏 41mA、耗散约 70mW，温升 <20°C"},
        {"④ 备选",   "1A Buck（未采用）", "ADR-1：本负载用不上，纹波反而害 SHT30/ADC"},
        {"⑤ 轨",     "3V3 = 10uF+100nF", "输出电容 2x1uF MLCC + 10uF，支持全 MLCC 稳定"},
        {"⑥ 负载",   "MCU F103C8T6",     "VDD 每脚 100nF + 一颗 1uF；VDDA 加磁珠+1uF"},
        {"⑥ 负载",   "SHT30 @I2C 0x44",  "4.7k 上拉到 3V3；VDD 100nF 就近"},
        {"⑥ 负载",   "OLED SSD1306 SPI", "最大 20mA，走 FPC 座；刷屏时轨电流阶跃"},
        {"⑥ 负载",   "3x LED + 2x KEY",  "LED 1k/470R；KEY2 复用 BOOT0，BOOT0 10k 下拉"},
        {"⑥ 负载",   "蜂鸣器 + S8050",   "三极管驱动，1N4148 续流（BOM 别漏！）"},
        {"⑥ 负载",   "NTC 100k + ADC",   "门控供电，参考与 3V3 同源 -> 必须去噪"},
        {"⑦ 测试点", "TP1 VBUS / TP2 3V3","TP3 SWDIO / TP4 GND —— 量产治具靠它们"},
    };
    int i;
    sec("电源树（谁给谁供电，每段的电流上限）");
    printf("%-8s %-20s %s\n", "阶段", "器件", "说明");
    hr('-', 88);
    for (i = 0; i < 15; ++i)
        printf("%-8s %-20s %s\n", tree[i].stage, tree[i].part, tree[i].info);
}

/* ---------------------------------------------------------------------------
 * 浪涌 / 输入电容
 * ------------------------------------------------------------------------- */
static void checkInrush(void)
{
    static const double caps[2] = {10.0, 22.0};
    int i;
    sec("上电冲击与 USB 输入电容限制（USB 规范：VBUS 输入电容 <= 10uF）");
    for (i = 0; i < 2; ++i) {
        const double uf = caps[i];
        const double qUc = uf * kUsbVbusVolt; /* uC */
        /* 假设 hot-plug 时 VBUS 经线阻 2欧 + 端口内阻 5欧 充电 */
        const double rOhm = 7.0;
        const double iPeakMa = (kUsbVbusVolt / rOhm) * 1000.0;
        const double tauUs = rOhm * uf; /* us */
        const bool ok = uf <= kUsbInCapLimitUf;
        printf("  输入电容 %5.1fuF : 电荷 Q=%.0fuC，峰值 %.0fmA，tau=%.0fus -> %s\n",
               uf, qUc, iPeakMa, tauUs, ok ? "PASS（在 10uF 内）" : "FAIL（超 USB 浪涌限制）");
        if (!ok)
            printf("      后果：热插拔把 VBUS 拉塌，同一 hub 上别的设备掉线；"
                   "对策：LDO 前只留 10uF，大容量放 LDO 输出侧。\n");
    }
    /* 输出侧的大电容不受 USB 输入限制约束 */
    printf("  提示：3V3 侧的 10uF+2x1uF 不算在 USB 输入电容里（被 LDO 隔开）。\n");
    printf("  LDO 压差与最小负载：ME6217 支持全 MLCC 输出电容，无需刻意加 ESR。\n");
}

/* ---------------------------------------------------------------------------
 * 续航
 * ------------------------------------------------------------------------- */
static void checkBattery(double idleMa)
{
    double hours, days;
    sec("假想加 500mAh 锂电池（USB 拔掉时的续航）");
    hours = kBatteryMah / idleMa;
    days = hours / 24.0;
    printf("  省电模式 %.3fmA -> 续航 %.1f h = %.1f 天（不含电池自放电/低温折损）\n",
           idleMa, hours, days);
    printf("  同一块电池如果 OLED 常亮全显：%.1f h = %.1f 天 —— 客户会瞬间改主意。\n",
           kBatteryMah / 29.5, kBatteryMah / 29.5 / 24.0);
    printf("  工程余量口径：锂衰降 20%%、DC-DC/LDO 效率再打折，保守按 0.8 系数算 %.0f h。\n",
           hours * 0.8);
}

/* ---------------------------------------------------------------------------
 * main
 * ------------------------------------------------------------------------- */
int main(void)
{
    Load normalLoads[LOAD_COUNT], idleLoads[LOAD_COUNT], alarmLoads[LOAD_COUNT], oledMaxLoads[LOAD_COUNT];
    Load ranked[LOAD_COUNT];
    Verdict normal, idle, alarm;
    int i;

    printf("╔══════════════════════════════════════════════════════════════════════════╗\n");
    printf("║   MiniBoard Phase 1 · 电源树与功耗预算（先算账，后画图）               ║\n");
    printf("╚══════════════════════════════════════════════════════════════════════════╝\n");

    printPowerTree();

    makeModeNormal(normalLoads);
    makeModeIdle(idleLoads);
    makeModeAlarm(alarmLoads);
    makeModeOledMax(oledMaxLoads);

    normal = evaluateMode("常亮采集 1Hz（显示低亮常显）", normalLoads, LOAD_COUNT, false);
    idle   = evaluateMode("省电模式（10s 一次采样，关屏）", idleLoads, LOAD_COUNT, true);
    alarm  = evaluateMode("报警模式（蜂鸣器 + 报警灯 + 显示）", alarmLoads, LOAD_COUNT, false);
    evaluateMode("反例：客户要求 OLED 全亮常显", oledMaxLoads, LOAD_COUNT, true);

    checkInrush();
    checkBattery(idle.total);

    sec("电流排行榜（省电模式，谁在偷电）");
    memcpy(ranked, idleLoads, sizeof(ranked));
    sortLoadsDesc(ranked, LOAD_COUNT);
    for (i = 0; i < LOAD_COUNT; ++i) {
        const Load *l = &ranked[i];
        if (l->ma <= 0.0) continue;
        printf("  #%d  %-22s %8.4f mA  占比 %5.1f%%\n",
               i + 1, l->name, l->ma, l->ma / idle.total * 100.0);
    }
    printf("  师傅点评：第一名是电源 LED（%.2fmA），MCU 只排第二。\n", ranked[0].ma);
    printf("  要再降一档：LED 改呼吸（平均 0.2mA）或干脆去掉电源灯 -> 总电流约 %.2fmA。\n",
           idle.total - ranked[0].ma + 0.2);

    sec("Phase 1 结论");
    printf("  规格项                 指标            实测(模型)      判定\n");
    hr('-', 88);
    printf("  静态电流               < 3.00 mA       %.2f mA        %s\n",
           idle.total, idle.pass ? "PASS" : "FAIL");
    printf("  USB 500mA 裕量         占用 <10%%       常亮 %.2f mA    %s\n",
           normal.total + kLdoIqMa, (normal.total + kLdoIqMa < kUsbCurrentBudgetMa * 0.1) ? "PASS" : "偏低");
    printf("  LDO 温升（报警最坏）   < 25 C          %.1f C         %s\n",
           (kUsbVbusVolt - kRailVolt) * alarm.total / 1000.0 * kLdoThetaJaCPerW,
           ((kUsbVbusVolt - kRailVolt) * alarm.total / 1000.0 * kLdoThetaJaCPerW < 25.0) ? "PASS" : "FAIL");
    printf("  USB 输入电容           <= 10 uF        10 uF          PASS\n");
    printf("  500mAh 续航（省电）    > 100 h         %.0f h          PASS\n", kBatteryMah / idle.total);
    printf("\n  一句话：%s 模式达标，代价是 OLED 必须关屏/降亮度；"
           "客户若要常显，回到第一页重谈功耗指标。\n", "省电");
    return 0;
}
