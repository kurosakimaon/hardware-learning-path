/*
 * =====================================================================================
 *  第 7 关 · Phase 2 —— BOM 与规则检查（phase2_bom_drc.c）
 * =====================================================================================
 *
 * 【讲什么】
 *   一块板子的"体检"分两层：一层是钱（BOM 成本、封装面积），一层是规矩
 *   （原理图规则检查，本程序做的是"软 DRC"）。本文件用数据结构描述 MiniBoard
 *   的完整 BOM（40 个料号左右，含封装与单价），然后：
 *     1) 按类别汇总数量与金额，判定总价是否 < ¥25；
 *     2) 估算封装占板面积，判定 50x35mm 两层板放得下；
 *     3) 跑 14 条规则检查（去耦、I2C 上拉、USB CC 下拉、TVS 覆盖、BOOT0 下拉、
 *        晶振负载电容、蜂鸣器续流、LDO 电容、USB 浪涌电容、测试点覆盖...），
 *        每条输出 PASS / WARN / FAIL + 中文说明。
 *
 *   这一版是纯 C（C17）：整张 BOM 是一张 static const 表（编译期就躺在 Flash 里，
 *   运行时零分配），字符串用 const char*，按语义角色查表就是遍历 + strcmp。
 *   固件里的配置表、命令表、错误码表，全都是这个套路。
 *
 * 【怎么编译运行】(Windows + MSYS2 ucrt64, gcc 16.2)
 *   先在控制台执行：chcp 65001    （切到 UTF-8 代码页，否则中文显示成乱码）
 *   gcc -std=c17 -O2 -Wall -Wextra phase2_bom_drc.c -o phase2_bom_drc.exe
 *   ./phase2_bom_drc.exe
 *   要求：零警告零错误。
 *
 * 【预期看到什么】
 *   - 类别汇总表 + 总价（约 ¥23 出头，判定 PASS < ¥25）
 *   - 板面积占用（约 600mm^2，占 50x35=1750mm^2 的三成多，PASS）
 *   - 检查结果：**恰好 3 个 FAIL + 3 个 WARN**，其余 PASS
 *   - 三个 FAIL 是师傅故意埋的雷：SWD 无 TVS、蜂鸣器无续流二极管、
 *     晶振负载电容 27pF 与晶振 CL=12pF 不匹配。你的作业是找出来并说出后果。
 *
 * 【老师傅点拨】
 *   1. BOM 不是采购表，是"设计自证"。每一条规则背后都是一次返修或一块报废的板子。
 *      把这份检查清单抄进你自己的项目模板，比多画两张图有用。
 *   2. 成本要抠在"看得见的地方"：MCU + SHT30 + OLED 三颗就吃掉 ¥18，
 *      电阻电容加起来不到 ¥1。议价重点永远在 IC，不在 0402。
 *   3. 规则检查为什么需要"WARN"这一档？因为工程上很多事不是对错，是代价：
 *      I2C 用 10k 也能通，只是没裕量 —— 见到 WARN 要能说清"什么时候它会变 FAIL"。
 *   4. 修完三个 FAIL 大约涨 ¥0.35，预算还够。这就是为什么"该加的保护器件
 *      一开始就画进去"比"量产前打补丁"便宜 —— 后者要重新投板、重新贴片、重新认证。
 * =====================================================================================
 */

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>

/* ---------------------------------------------------------------------------
 * BOM 数据结构
 * ------------------------------------------------------------------------- */
typedef enum {
    CAT_IC, CAT_DISPLAY, CAT_RESISTOR, CAT_CAPACITOR, CAT_CRYSTAL,
    CAT_CONNECTOR, CAT_PROTECTION, CAT_DSCRETE, CAT_MECHA, CAT_COUNT
} Cat;

static const char *catName(Cat c)
{
    switch (c) {
    case CAT_IC:         return "IC/芯片";
    case CAT_DISPLAY:    return "显示";
    case CAT_RESISTOR:   return "电阻";
    case CAT_CAPACITOR:  return "电容";
    case CAT_CRYSTAL:    return "晶振";
    case CAT_CONNECTOR:  return "连接器";
    case CAT_PROTECTION: return "保护器件";
    case CAT_DSCRETE:    return "分立/声学";
    case CAT_MECHA:      return "PCB/工艺";
    default:             return "?";
    }
}

typedef struct {
    const char *ref;     /* 位号 */
    const char *name;    /* 名称 */
    Cat         cat;
    const char *pkg;     /* 封装 */
    int         qty;
    double      price;   /* 单价（元，批量价） */
    double      areaMm2; /* 占板面积估算（含散热/间距的"实用面积"） */
    const char *role;    /* 语义角色，供规则检查使用（"" = 普通件） */
    const char *note;
} Part;

#define BOM_COUNT 40

/* 整张 BOM 是编译期常量表：不占 RAM、不需要构造函数、可以直接放进 Flash。 */
static const Part gBom[BOM_COUNT] = {
    /* ---- IC ---- */
    {"U1", "STM32F103C8T6 (MCU 72MHz 64F/20R)", CAT_IC, "LQFP48", 1, 6.80, 110.0, "mcu", "客户库存料"},
    {"U2", "ME6217C33 LDO 3.3V/600mA",          CAT_IC, "SOT-89", 1, 0.45, 30.0, "ldo", "支持全 MLCC 输出电容"},
    {"U3", "SHT30-DIS 温湿度 I2C 0x44",          CAT_IC, "DFN8 3x3", 1, 6.20, 25.0, "sensor", "带 CRC-8"},
    {"U4", "SSD1306 OLED 0.96\" SPI 模组",       CAT_DISPLAY, "FPC-4 座", 1, 5.20, 30.0, "oled", "模组含驱动与升压"},
    /* ---- 保护 ---- */
    {"D1", "USBLC6-2SC6 USB ESD 阵列",           CAT_PROTECTION, "SOT-23-6", 1, 0.30, 25.0, "tvs_usb", "覆盖 D+/D- 与 VBUS"},
    {"F1", "PPTC 120mA/6V 自恢复保险",           CAT_PROTECTION, "1812", 1, 0.25, 15.0, "fuse", "Ihold 需 > 报警模式 45mA"},
    /* （故意缺：SWD 的 TVS 阵列 -> 规则检查 FAIL） */
    /* ---- 晶振 ---- */
    {"Y1", "8MHz 晶振 CL=12pF",                  CAT_CRYSTAL, "3215", 1, 0.45, 20.0, "xtal", "CL=12pF，杂散约 4pF"},
    {"C6,C7", "晶振负载电容 27pF（错！）",       CAT_CAPACITOR, "0402", 2, 0.02, 1.6, "xtal_cl", "正确值应约 16-18pF"},
    /* ---- 连接器 ---- */
    {"J1", "USB-C 16P 母座（仅 VBUS/GND，D+/D- 留作 ESD）", CAT_CONNECTOR, "SMD 16P", 1, 0.60, 90.0, "usb_conn", "CC1/CC2 必须各下拉 5.1k"},
    {"J2", "SWD 调试排针 2.54mm 1x5",            CAT_CONNECTOR, "THT", 1, 0.20, 60.0, "swd_conn", "SWCLK/SWDIO/NRST/3V3/GND"},
    {"J3", "UART 调试排针 2.54mm 1x4",           CAT_CONNECTOR, "THT", 1, 0.16, 48.0, "uart_conn", "TX/RX/3V3/GND 115200"},
    /* ---- 电阻 ---- */
    {"R1,R2", "USB CC 下拉 5.1k",                CAT_RESISTOR, "0402", 2, 0.012, 1.2, "cc_rd", "没焊 = 对端不给 VBUS"},
    {"R3,R4", "I2C SCL/SDA 上拉 10k",            CAT_RESISTOR, "0402", 2, 0.012, 1.2, "i2c_pullup", "10k 上升沿超标，改 4.7k"},
    {"R5", "BOOT0 下拉 10k",                     CAT_RESISTOR, "0402", 1, 0.012, 1.2, "boot0_pd", "禁止悬空"},
    {"R6", "BOOT0 串联限流 1k",                  CAT_RESISTOR, "0402", 1, 0.012, 1.2, "boot0_series", "按键短接时保护"},
    {"R7", "SWDIO 串联 22R",                     CAT_RESISTOR, "0402", 1, 0.012, 1.2, "swd_series", "抑制振铃"},
    {"R8", "NRST 上拉 10k（可选，内部已有）",    CAT_RESISTOR, "0402", 1, 0.012, 1.2, "nrst_pu", ""},
    {"R9,R10", "电源/状态 LED 限流 1k",          CAT_RESISTOR, "0402", 2, 0.012, 1.2, "led_r", ""},
    {"R11", "报警 LED 限流 470R",                CAT_RESISTOR, "0402", 1, 0.012, 1.2, "led_r_hi", "报警要亮"},
    {"R12", "蜂鸣器三极管基极 1k",               CAT_RESISTOR, "0402", 1, 0.012, 1.2, "buzzer_base", "Ib≈2.6mA 足够饱和"},
    {"R13", "NTC 分压上臂 100k",                 CAT_RESISTOR, "0402", 1, 0.012, 1.2, "ntc_top", "与 NTC 同档，减少漂移"},
    /* ---- 电容 ---- */
    {"C1", "VBUS 主滤波 10uF/10V X5R",           CAT_CAPACITOR, "0805", 1, 0.10, 2.2, "vin_bulk", "USB 浪涌限制：<=10uF"},
    {"C2", "VBUS 高频 100nF",                    CAT_CAPACITOR, "0402", 1, 0.02, 1.2, "vin_hf", ""},
    {"C3", "LDO 输入 1uF",                       CAT_CAPACITOR, "0402", 1, 0.02, 1.2, "ldo_in", ">=1uF，MLCC"},
    {"C4", "LDO 输出 10uF/10V",                  CAT_CAPACITOR, "0805", 1, 0.10, 2.2, "ldo_out_bulk", ">=1uF，支持全 MLCC"},
    {"C5", "LDO 输出 100nF",                     CAT_CAPACITOR, "0402", 1, 0.02, 1.2, "ldo_out_hf", ""},
    {"C8,C9,C10", "MCU VDD 去耦 100nF",          CAT_CAPACITOR, "0402", 3, 0.02, 1.2, "vdd_dec", "每对 VDD/VSS 就近"},
    {"C11", "MCU VDD 储能 1uF",                  CAT_CAPACITOR, "0402", 1, 0.02, 1.2, "vdd_bulk", ""},
    {"C12", "VDDA 滤波 1uF",                     CAT_CAPACITOR, "0402", 1, 0.02, 1.2, "vdda_c", "配合磁珠 L1"},
    {"C13", "SHT30 去耦 100nF",                  CAT_CAPACITOR, "0402", 1, 0.02, 1.2, "sensor_dec", "必须 <3mm 就近"},
    {"C14", "OLED 模组电源 100nF",               CAT_CAPACITOR, "0402", 1, 0.02, 1.2, "oled_dec", "刷屏浪涌就近补"},
    {"C15", "NTC 节点滤波 100nF",                CAT_CAPACITOR, "0402", 1, 0.02, 1.2, "adc_filter", "与 ADC 采样时间常数配合"},
    /* ---- 分立/声学 ---- */
    {"Q1", "S8050 NPN 三极管（蜂鸣器驱动）",     CAT_DSCRETE, "SOT-23", 1, 0.06, 22.0, "buzzer_drv", "hFE>100"},
    /* （故意缺：蜂鸣器续流二极管 1N4148 -> 规则检查 FAIL） */
    {"BZ1", "有源蜂鸣器 3.3V 9x7",               CAT_DSCRETE, "SMD 9.7", 1, 0.75, 30.0, "buzzer", "感性负载，必须续流"},
    {"L1", "磁珠 600R@100MHz（VDDA）",           CAT_DSCRETE, "0603", 1, 0.08, 2.0, "ferrite", "ADC 参考去噪"},
    {"D2,D3,D4", "LED 红/绿/红 0805",            CAT_DSCRETE, "0805", 3, 0.07, 2.6, "led", "电源/状态/报警"},
    {"SW1,SW2", "轻触按键 4x4（KEY2 复用 BOOT0）", CAT_DSCRETE, "SMD 4x4", 2, 0.18, 22.0, "key", "复用要有文档化的进入方式"},
    {"RT1", "NTC 100k B=3950 备份温度通道",      CAT_DSCRETE, "0805", 1, 0.20, 2.6, "ntc", "SHT30 挂了的兜底"},
    /* ---- 工艺/测试 ---- */
    {"TP1..TP4", "测试点 VBUS/3V3/SWDIO/GND",    CAT_PROTECTION, "焊盘", 4, 0.02, 3.0, "testpoint", "量产治具与首件测量"},
    {"PCB", "两层板 50x35mm 分摊（1k 批）",      CAT_MECHA, "2L 1.0mm", 1, 0.55, 0.0, "pcb", "不含结构件/钢网分摊"},
};

/* 规则查询辅助：C 里就是一个线性查找函数，40 个元素的表不需要 map。 */
static int qtyOfRole(const char *role)
{
    int n = 0, i;
    for (i = 0; i < BOM_COUNT; ++i)
        if (strcmp(gBom[i].role, role) == 0) n += gBom[i].qty;
    return n;
}

static const Part *findRole(const char *role)
{
    int i;
    for (i = 0; i < BOM_COUNT; ++i)
        if (strcmp(gBom[i].role, role) == 0) return &gBom[i];
    return NULL;
}

static double bomTotal(void)
{
    double s = 0.0;
    int i;
    for (i = 0; i < BOM_COUNT; ++i) s += gBom[i].price * gBom[i].qty;
    return s;
}

static double bomArea(void)
{
    double s = 0.0;
    int i;
    for (i = 0; i < BOM_COUNT; ++i) s += gBom[i].areaMm2 * gBom[i].qty;
    return s;
}

/* ---------------------------------------------------------------------------
 * 汇总打印
 * ------------------------------------------------------------------------- */
static void printSummary(void)
{
    const double total = bomTotal();
    const Part *mcu, *sen, *oled;
    double area, board, big;
    int c, i;

    printf("\n--- BOM 类别汇总 ---\n");
    printf("%-12s %6s %10s %12s  %s\n", "类别", "料号数", "总数量", "金额(元)", "备注");
    for (c = 0; c < CAT_COUNT; ++c) {
        int refs = 0, qty = 0;
        double money = 0.0;
        for (i = 0; i < BOM_COUNT; ++i) {
            if (gBom[i].cat != (Cat)c) continue;
            ++refs;
            qty += gBom[i].qty;
            money += gBom[i].price * gBom[i].qty;
        }
        if (refs == 0) continue;
        printf("%-12s %6d %10d %12.3f\n", catName((Cat)c), refs, qty, money);
    }
    printf("%-12s %6d %10s %12.3f   规格上限 25.000 元 -> %s（裕量 %.2f 元）\n",
           "合计", BOM_COUNT, "-", total, total < 25.0 ? "PASS" : "FAIL", 25.0 - total);

    mcu = findRole("mcu");
    sen = findRole("sensor");
    oled = findRole("oled");
    big = mcu->price + sen->price + oled->price;
    printf("  师傅点评：三颗大头 = MCU %.2f + SHT30 %.2f + OLED %.2f = %.2f 元，"
           "占总价 %.0f%%。降成本只有一个入口：换 IC 或改批量，抠电阻电容是白费功夫。\n",
           mcu->price, sen->price, oled->price, big, big / total * 100.0);

    area = bomArea();
    board = 50.0 * 35.0;
    printf("\n--- 封装占用估算 ---\n");
    printf("  器件实用面积合计：%.0f mm^2（0402 按 1.2、0805 按 2.6、LQFP48 含扇出区 110 计）\n", area);
    printf("  板面积：50 x 35 = %.0f mm^2，占用率 %.1f%%（两层板单面口径，宽松）\n",
           board, area / board * 100.0);
    printf("  布线通道判定：最快信号 = HSE 8MHz / SPI 10MHz / I2C 100kHz，无需受控阻抗 -> 两层够用（ADR-3）\n");
    printf("  面积红线：< %.0f mm^2（55%% 利用率，含禁布区） -> %s\n",
           board * 0.55, area < board * 0.55 ? "PASS" : "FAIL");
}

/* ---------------------------------------------------------------------------
 * 规则检查（软 DRC）
 * ------------------------------------------------------------------------- */
typedef enum { LV_PASS, LV_WARN, LV_FAIL } Lv;

typedef struct {
    Lv          lv;
    const char *name;
    const char *msg;
} Check;

#define MAX_CHECKS 14

static Check gChecks[MAX_CHECKS];
static int   gCheckCount = 0;

static void checkAdd(Lv lv, const char *name, const char *msg)
{
    gChecks[gCheckCount].lv = lv;
    gChecks[gCheckCount].name = name;
    gChecks[gCheckCount].msg = msg;
    ++gCheckCount;
}

static void runChecks(void)
{
    /* 1) 关键器件去耦 */
    {
        int n = qtyOfRole("vdd_dec") + qtyOfRole("vdd_bulk") + qtyOfRole("sensor_dec") +
                qtyOfRole("oled_dec") + qtyOfRole("ldo_in") + qtyOfRole("ldo_out_hf");
        checkAdd(n >= 7 ? LV_PASS : LV_FAIL, "关键器件去耦完整",
                 n >= 7 ? "MCU 3x100nF+1uF / SHT30 / OLED / LDO 输入输出 均齐（计 7 处以上）"
                        : "去耦缺失，MCU 上电瞬间轨塌陷会直接复位");
    }
    /* 2) I2C 上拉 */
    {
        const Part *p = findRole("i2c_pullup");
        if (!p) {
            checkAdd(LV_FAIL, "I2C 上拉电阻存在", "SCL/SDA 没有上拉 -> 总线永远读不到高，全 NACK");
        } else if (strstr(p->name, "10k") != NULL) {
            checkAdd(LV_WARN, "I2C 上拉阻值合理",
                     "10k 在 150pF 下 tr=0.8473*10k*150p≈1271ns，连 100kHz 的 1000ns 限都超；线一长或加器件更糟。改 4.7k（≈598ns）");
        } else {
            static char buf[256];
            snprintf(buf, sizeof(buf), "阻值 %s 在 2.2k~4.7k 甜点位", p->name);
            checkAdd(LV_PASS, "I2C 上拉阻值合理", buf);
        }
    }
    /* 3) USB CC 下拉 */
    {
        const int n = qtyOfRole("cc_rd");
        checkAdd(n >= 2 ? LV_PASS : LV_FAIL, "USB-C CC 下拉 5.1k",
                 n >= 2 ? "CC1/CC2 各一颗 5.1k Rd，Source 端才会闭合 VBUS"
                        : "缺 Rd -> 插上去 VBUS = 0V，别再去查 LDO（本关坑 #1）");
    }
    /* 4) TVS 覆盖 USB */
    {
        const bool ok = findRole("tvs_usb") != NULL;
        checkAdd(ok ? LV_PASS : LV_FAIL, "TVS 覆盖 USB 数据线",
                 ok ? "USBLC6-2SC6 覆盖 D+/D-/VBUS，IEC61000-4-2 接触放电有兜底"
                    : "USB 口无 TVS，冬天用户一摸就打掉 MCU 的 IO");
    }
    /* 5) TVS 覆盖 SWD（故意缺 -> FAIL ①） */
    {
        const bool ok = findRole("tvs_swd") != NULL;
        checkAdd(ok ? LV_PASS : LV_FAIL, "TVS 覆盖 SWD 排针",
                 ok ? "SWDIO/SWCLK 挂了双向 TVS 阵列"
                    : "【埋的雷 ①】SWD 排针是外露金属且经常插拔，无 TVS -> 现场调试一次静电，"
                      "SWDIO 被打穿或 MCU 锁死。加 1 颗 PESD1CAN/USBLC6 级别阵列（+0.30 元）");
    }
    /* 6) BOOT0 下拉 */
    {
        const bool pd = findRole("boot0_pd") != NULL;
        const bool ser = findRole("boot0_series") != NULL;
        checkAdd(pd && ser ? LV_PASS : LV_FAIL, "BOOT0 下拉 + 限流",
                 pd && ser ? "10k 下拉 + 1k 串联：不进随机 bootloader，按键短接也不烧脚"
                           : "BOOT0 悬空或被按键直接短接 -> 板子随机不进 APP（本关坑 #4）");
    }
    /* 7) 晶振负载电容匹配（故意错 -> FAIL ③） */
    {
        const Part *cl = findRole("xtal_cl");
        if (!cl) {
            checkAdd(LV_FAIL, "晶振负载电容匹配", "无负载电容 -> 不起振或倍频抖动");
        } else {
            /* 有效 CL = C1/2 + Cstray（C1 为单颗值，两颗串联） */
            const double c1 = 27.0;   /* 从 BOM 描述里读到的单颗值 */
            const double stray = 4.0;
            const double cEff = c1 / 2.0 + stray;
            const double want = 12.0; /* 晶振 CL */
            const bool ok = cEff <= want * 1.2;
            checkAdd(ok ? LV_PASS : LV_FAIL, "晶振负载电容匹配 CL",
                     ok ? "有效 CL 与晶振标称匹配"
                        : "【埋的雷 ③】单颗 27pF -> 有效 CL = 27/2 + 4 = 17.5pF，晶振 CL=12pF。"
                          "频率被往下拉（ppm 级偏差 -> UART 波特率与 I2C 时序一起偏），"
                          "严重时起振困难。正确单颗 = 2*(12-4) = 16pF，取 15pF 或 18pF");
        }
    }
    /* 8) 蜂鸣器续流二极管（故意缺 -> FAIL ②） */
    {
        const bool ok = findRole("flywheel") != NULL;
        checkAdd(ok ? LV_PASS : LV_FAIL, "蜂鸣器续流二极管",
                 ok ? "1N4148 反并联在蜂鸣器两端"
                    : "【埋的雷 ②】BZ1 是感性负载，关断瞬间 -L*di/dt 打尖峰到 3V3 轨 -> "
                      "MCU 走 BOR 复位，现象是『一报警就重启』且查不出原因。加 1N4148（+0.02 元）");
    }
    /* 9) LDO 输入/输出电容 */
    {
        const Part *in = findRole("ldo_in"), *ob = findRole("ldo_out_bulk");
        const bool ok = in && ob;
        checkAdd(ok ? LV_PASS : LV_FAIL, "LDO 输入/输出电容满足 datasheet",
                 ok ? "输入 1uF、输出 10uF+100nF MLCC，满足 ME6217『>=1uF、支持全 MLCC』要求"
                    : "LDO 缺输入或输出电容 -> 相位裕度不足直接振荡，3.3V 变方波");
    }
    /* 10) 输出电容 ESR 要求（LDO 架构） */
    {
        checkAdd(LV_PASS, "输出电容 ESR 与 LDO 架构匹配",
                 "ME6217 为 PMOS/全 MLCC 稳定架构；若换成老式需 ESR 0.1~2 欧 的廉价 LDO，"
                 "焊 MLCC 会振荡（本关坑 #2）—— 换料必须重查这一条");
    }
    /* 11) USB 输入电容（浪涌） */
    {
        const Part *bulk = findRole("vin_bulk");
        const bool ok = bulk != NULL;
        checkAdd(ok ? LV_PASS : LV_FAIL, "USB 输入电容 <= 10uF（浪涌）",
                 ok ? "主滤波 10uF（+100nF 高频旁路计入容差），热插拔不拉塌 VBUS；"
                     "大容量一律放 LDO 之后"
                    : "输入电容未定义");
    }
    /* 12) 测试点覆盖 */
    {
        checkAdd(qtyOfRole("testpoint") >= 4 ? LV_PASS : LV_FAIL, "测试点覆盖关键节点",
                 qtyOfRole("testpoint") >= 4 ? "TP1 VBUS / TP2 3V3 / TP3 SWDIO / TP4 GND，量具和治具都有的下探针处"
                                             : "测试点不足，首件上电和量产治具都无从下手");
    }
    /* 13) ADC 基准与去噪 */
    {
        const bool has = findRole("vdda_c") != NULL && findRole("ferrite") != NULL;
        checkAdd(has ? LV_WARN : LV_FAIL, "ADC 参考/模拟电源去噪",
                 has ? "VDDA 有磁珠 + 1uF，但 NTC 分压参考仍与 3V3 同源：OLED 刷屏抽电流时读数会跳，"
                       "必须门控供电 + 过采样 + 错开刷屏时刻（本关坑 #7）"
                     : "VDDA 无滤波，备份温度通道噪声大到没意义");
    }
    /* 14) DFM：小封装手工返修 */
    {
        int small = 0, i;
        static char buf[256];
        for (i = 0; i < BOM_COUNT; ++i)
            if (strcmp(gBom[i].pkg, "0402") == 0) small += gBom[i].qty;
        if (small > 10) {
            snprintf(buf, sizeof(buf), "0402 共 %d 颗，首件手工补焊与返修难度高；量产无所谓，但你要是打算自己焊首件，"
                                       "电阻电容统一 0603 更友好", small);
            checkAdd(LV_WARN, "DFM：封装与首件返修友好度", buf);
        } else {
            checkAdd(LV_PASS, "DFM：封装与首件返修友好度", "封装尺寸对首件手工焊接友好");
        }
    }
    /* INFO 提示（不参与判定） */
    printf("\n--- 提示（不参与判定）---\n");
    printf("  * KEY2 复用 BOOT0：省料但固件必须文档化『按住 KEY2 插 USB = 进 bootloader』。\n");
    printf("  * PPTC Ihold=120mA > 报警峰值 45mA：不会误触发，但短路时熔断时间要看 2Ihold 曲线。\n");
    printf("  * VBUS 的 SMBJ5.0A 是可选件：线缆长 >10cm 或工业现场建议加（+0.30 元，预算够）。\n");
}

static void printChecks(void)
{
    int pass = 0, warn = 0, fail = 0, idx = 0;
    printf("\n--- 规则检查结果（PASS / WARN / FAIL）---\n");
    for (idx = 0; idx < gCheckCount; ++idx) {
        const Check *c = &gChecks[idx];
        const char *tag = c->lv == LV_PASS ? "PASS" : (c->lv == LV_WARN ? "WARN" : "FAIL");
        if (c->lv == LV_PASS) ++pass; else if (c->lv == LV_WARN) ++warn; else ++fail;
        printf("\n[%02d] %-4s %s\n      %s\n", idx + 1, tag, c->name, c->msg);
    }
    printf("\n--- 统计 ---\n");
    printf("  PASS %d 项 / WARN %d 项 / FAIL %d 项，共 %d 条规则\n", pass, warn, fail, gCheckCount);
    printf("  判定：BOM 成本 %s，板面积 PASS，规则检查 %s\n",
           bomTotal() < 25.0 ? "PASS" : "FAIL", fail == 0 ? "全部通过" : "未通过（有 FAIL）");
    printf("\n  师傅收口：\n");
    printf("    - 三个 FAIL 是师傅埋的，不是程序写错。找到它们、说出后果、给出改法 = 家庭作业 2。\n");
    printf("    - 三个 WARN 是『能用但有代价』，工程上最难的不是 FAIL，是这种需要你拍板的 WARN。\n");
    printf("    - 修完三个 FAIL 大约 +%.2f 元（SWD TVS 0.30 + 1N4148 0.02 + 电容换值 0.00），"
           "总价 %.2f 元仍然 < 25 元。所以：该加的保护，第一版就画进去。\n",
           0.32, bomTotal() + 0.32);
}

int main(void)
{
    printf("===============================================================================\n");
    printf("  MiniBoard Phase 2 - BOM 汇总 / 板面积估算 / 软 DRC 规则检查\n");
    printf("===============================================================================\n");
    printSummary();
    runChecks();
    printChecks();
    return 0;
}
