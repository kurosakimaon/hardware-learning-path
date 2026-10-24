/*
 * ============================================================
 *  第 4 关 · 03_i2c_bus.c
 *  —— I2C 是两根「线与」的绳子：开漏总线模型 + 上拉电阻算账
 *     + 完整时序逐位波形 + 死锁与 9 脉冲救援
 * ============================================================
 *
 * 【讲什么】
 *   1) 开漏 + 上拉的「线与」模型：总线上任何一只管子拉低，线就是低；
 *      所有人都放手，上拉电阻才把线拉回高。上升沿是 RC 充电，
 *      所以又缓又依赖上拉 —— 这条决定了 I2C 的一切脾气。
 *   2) 上拉电阻上下限算账：
 *      Rmin = Vdd / Iol(max)（灌电流别超 3mA，不然拉不到 0.4V 以下）；
 *      Rmax = tr / (0.847 × Cb)（I2C 规范按 30%→70% 量 tr，约等于 0.847·R·C，
 *      容性负载规范上限 400pF，总线长了上到 1000pF 就是深水区）。
 *      程序按几组真实参数算，包括「区间为空」的翻车组，教你读结果。
 *   3) 完整事务逐位波形字符图：START → 从机地址+RW → ACK → 寄存器地址
 *      → 重复 START → 地址+读 → ACK → 数据 → 主机 NACK → STOP。
 *   4) 两种死法与救援：从机拉住 SCL 不放（clock stretching 超时）；
 *      主机被中断打断、从机卡在地址第 8 拍守着 SDA 低 —— 总线死锁，
 *      演示标准的「补 9 个 SCL 脉冲 + STOP」恢复流程。
 *
 * 【怎么编译运行】（MSYS2 / gcc 16.2，Windows，C17 零警告）
 *   chcp 65001            ← 运行前先执行这一句，否则 Windows 控制台中文乱码
 *   gcc -std=c17 -O2 -Wall -Wextra 03_i2c_bus.c -o 03_i2c_bus.exe
 *   .\03_i2c_bus.exe
 *   没有板子也能跑：SDA/SCL 是软件模型，纯打印。
 *   注：本文件不用 M_PI（-std=c17 严格模式下 MSYS2 不暴露它），
 *       需要圆周率就自己 #define PI 3.14159265358979323846 —— 这里算上拉用不到。
 *
 * 【预期看到什么】
 *   - 线与实验：主机放手（ODR=1）、从机拉低 → 总线读回 0。
 *   - 上拉表：3.3V/3mA → Rmin=1100Ω；tr=300ns 且 Cb=1000pF → Rmax≈354Ω，
 *     比 Rmin 还小 —— 区间为空，程序明说这组合根本无解（要么缩短走线
 *     降容性、要么退回 1000ns 慢沿、要么上总线中继器）；常规
 *     100pF/1000ns 组算出 Rmax≈11.8kΩ，推荐 2.2k~4.7k。
 *   - 一张逐位字符波形图：START/地址 0xA0/数据 0xA5/STOP 全画出来，
 *     每段下面标注这一拍在干什么、谁在驱动。
 *   - 死锁演示：SCL 已停、SDA 被从机按在 0；发 9 个 SCL 脉冲后从机放手，
 *     主机补 STOP，总线恢复空闲（SDA=SCL=1）。
 *
 * 【老师傅点拨】
 *   - I2C 不通先量三件事：有没有上拉（断电时量到 VDD 那只是体二极管）、
 *     静态电平是不是高（卡在低 = 有人拉住 = 死锁或有从机没上电）、
 *     地址 8 位发完有没有 ACK（没 ACK 先数地址位，A0~A2 焊反最常见）。
 *   - 上拉不是越小越快越好：太小，从机拉低时灌电流超标、低电平被顶高，
 *     示波器上「低电平 0.9V」就是它；太大，上升沿慢成馒头，
 *     400k 速率下采样点读到半个电平，随机 NACK。
 *   - 死锁救援那 9 个脉冲不是玄学：从机卡在「第 8 拍守 SDA」，
 *     你给它 9 个 SCL，它把这一个字节吐完就放手；STOP 让双方状态机复位。
 *     量产固件里这个恢复函数必须做，而且要在超时后自动跑。
 *   - C 里写这种「总线上的所有设备」模型，别上 new/容器：一个 struct
 *     位图 + 几个静态函数就够；波形段数是可算的（1+9+9+1+9+9+1=39 拍），
 *     开一个固定大小的数组，比 vector 更贴近单片机的真实写法。
 */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>   /* memset / snprintf 场景下的缓冲区清零 */

/* ---------------------------------------------------------- */
/* 一、开漏线与总线模型                                        */
/* ---------------------------------------------------------- */

/* 每条线记录「谁在主动拉低」。开漏设备只有两种状态：拉低 / 放手，
 * 放手时的电平由上拉电阻负责。设备编号：0=主机，1=从机A，2=从机B */
struct I2cBus {
    uint32_t sda_pull_low_by;      /* 哪些设备正在拉低 SDA */
    uint32_t scl_pull_low_by;      /* 哪些设备正在拉低 SCL */
};

static int bus_sda(const struct I2cBus *bus)
{
    return bus->sda_pull_low_by == 0u ? 1 : 0;
}

static int bus_scl(const struct I2cBus *bus)
{
    return bus->scl_pull_low_by == 0u ? 1 : 0;
}

/* 位操作惯用法：拉低就是 |= 置位，放手就是 &= ~ 清位 —— 只动自己那一位 */
static void bus_drive_sda(struct I2cBus *bus, uint32_t dev, bool low)
{
    if (low) { bus->sda_pull_low_by |=  (1u << dev); }
    else     { bus->sda_pull_low_by &= ~(1u << dev); }
}

static void bus_drive_scl(struct I2cBus *bus, uint32_t dev, bool low)
{
    if (low) { bus->scl_pull_low_by |=  (1u << dev); }
    else     { bus->scl_pull_low_by &= ~(1u << dev); }
}

static void demo_wired_and(void)
{
    printf("============================================================\n");
    printf("演示一：线与实验 —— 主机写 1，从机拉 0，总线听谁的？\n");
    printf("============================================================\n");
    struct I2cBus bus = { 0u, 0u };
    printf("  初始（双方都放手，上拉电阻干活）：SDA=%d SCL=%d  ← 空闲态必须是 1 1\n",
           bus_sda(&bus), bus_scl(&bus));
    bus_drive_sda(&bus, 0u, true);
    printf("  主机拉低 SDA（比如发 START）：    SDA=%d SCL=%d\n", bus_sda(&bus), bus_scl(&bus));
    bus_drive_sda(&bus, 0u, false);
    bus_drive_sda(&bus, 1u, true);
    printf("  主机放手（ODR 写 1），从机 1 拉低：SDA=%d SCL=%d  ← 读回 0！\n",
           bus_sda(&bus), bus_scl(&bus));
    printf("  主机这边 ODR=1、IDR=0 —— 这个差值就是现场线索：\n");
    printf("  「我输出的是高，线上却是低」= 有别人在拉 / 没上拉 / 从机没上电。\n");
    bus_drive_sda(&bus, 1u, false);
    printf("  从机也放手：SDA=%d —— 谁都不拉，上拉电阻把线抬回去。\n\n", bus_sda(&bus));
}

/* ---------------------------------------------------------- */
/* 二、上拉电阻上下限                                          */
/* ---------------------------------------------------------- */

static void pullup_case(const char *name, double vdd, double iol_ma,
                        double tr_ns, double cb_pf)
{
    const double rmin = vdd / (iol_ma * 1e-3);            /* 欧姆 */
    const double rmax = (tr_ns * 1e-9) / (0.847 * cb_pf * 1e-12);
    printf("  %-28s Rmin=%7.0fΩ  Rmax=%8.0fΩ  →  ", name, rmin, rmax);
    if (rmax <= rmin) {
        printf("区间为空 ✗（Rmax < Rmin，无解）\n");
    } else {
        printf("允许区间 %.0fΩ ~ %.0fΩ ✓\n", rmin, rmax);
    }
}

static void demo_pullup_calc(void)
{
    printf("============================================================\n");
    printf("演示二：上拉电阻的账 —— Rmin 管「拉得动」，Rmax 管「升得起」\n");
    printf("============================================================\n");
    printf("  Rmin = Vdd / Iol(max)：从机拉低时灌电流 ≤ 3mA，低电平 ≤ 0.4V；\n");
    printf("  Rmax = tr / (0.847 × Cb)：I2C 规范按 30%%→70%% 量 tr ≈ 0.847·R·C，\n");
    printf("         tr 按需求取（标准模式规范 1000ns），Cb 是整条总线总电容。\n\n");
    pullup_case("规范典型 3.3V/3mA/1000ns/150pF", 3.3, 3.0, 1000.0, 150.0);
    pullup_case("快沿保守 3.3V/3mA/1000ns/100pF", 3.3, 3.0, 1000.0, 100.0);
    pullup_case("长总线 3.3V/3mA/300ns/1000pF", 3.3, 3.0, 300.0, 1000.0);
    pullup_case("5V 老系统 5V/3mA/1000ns/200pF", 5.0, 3.0, 1000.0, 200.0);

    printf("\n  读结果：\n");
    printf("   · 第 1 组：1.1k~7.9k —— 教科书区间，所以板子上 4.7k 遍地走。\n");
    printf("   · 第 2 组：1.1k~11.8k —— 短线小板放宽到 10k 也有人用，稳一点取 2.2k~4.7k。\n");
    printf("   · 第 3 组：300ns 快沿 + 1000pF 容性 —— 区间为空！算出来 Rmax≈354Ω，\n");
    printf("     比 Rmin=1100Ω 还小，物理上无解。1000pF 是规范天花板附近（深水区），\n");
    printf("     这种总线要么缩短走线/摘掉闲置从机把 Cb 压回 400pF 以内，\n");
    printf("     要么认命退回 1000ns 慢沿（回到第 1 组），要么上 I2C 总线中继器/缓冲器。\n");
    printf("   · 第 4 组：5V 系统 Rmin 抬到 1.67k —— 换电压先重算，别照抄 3.3V 的 2.2k。\n");
    printf("   · 别忘了静态功耗：1k 上拉挂在 3.3V 上，低电平期间 3mA 常烧着，\n");
    printf("     电池产品上这是肉眼可见的漏电。上拉是「快」和「省电」的对赌协议。\n\n");
}

/* ---------------------------------------------------------- */
/* 三、完整事务 + 逐位波形字符图                               */
/* ---------------------------------------------------------- */

/* 一个「位拍」：SCL 低期间换数据，SCL 高期间被采样 */
struct BitSlot {
    char phase[16];      /* 这一拍在干什么（定长，避免动态字符串）*/
    int  sda;            /* SCL 高期间的 SDA 电平 */
    bool sda_is_ack;     /* 该位由接收方驱动（ACK/NACK）*/
    bool start;          /* SCL 高期间 SDA 下降沿 = START / 重复 START */
    bool stop;           /* SCL 高期间 SDA 上升沿 = STOP */
};

/* 整场事务的拍数是可算的：START(1) + 字节×4(9×4) + 重复START(1) + STOP(1) = 39，
 * 固定数组 64 拍足够，单片机上就这么写，别抱容器 */
#define SLOT_MAX 64

/* 每拍画 8 列。数据拍：SCL 前半低后半高；START/STOP：SCL 全程高 */
static void render_waveform(const struct BitSlot *slots, size_t n)
{
    char sda[8 * 32 + 1];
    char scl[8 * 32 + 1];
    size_t xs = 0, xc = 0;
    for (size_t i = 0; i < n; ++i) {
        const struct BitSlot *b = &slots[i];
        if (b->start) {
            xs += (size_t)snprintf(sda + xs, sizeof(sda) - xs, "----....");
            xc += (size_t)snprintf(scl + xc, sizeof(scl) - xc, "++++++++");
        } else if (b->stop) {
            xs += (size_t)snprintf(sda + xs, sizeof(sda) - xs, "____++++");
            xc += (size_t)snprintf(scl + xc, sizeof(scl) - xc, "++++++++");
        } else {
            const char hi = b->sda_is_ack ? 'a' : '-';
            const char lo = b->sda_is_ack ? 'A' : '_';
            const char c = b->sda ? hi : lo;
            xs += (size_t)snprintf(sda + xs, sizeof(sda) - xs, "....%c%c..", c, c);
            xc += (size_t)snprintf(scl + xc, sizeof(scl) - xc, "....++++");
        }
    }
    printf("   SDA: %s\n", sda);
    printf("   SCL: %s\n", scl);
}

/* 往拍序列尾部追加一个字节（MSB first —— I2C 高位先行）+ ACK 拍 */
static void push_byte(struct BitSlot *slots, size_t *pn, uint8_t byte, const char *tag)
{
    for (int i = 7; i >= 0; --i) {
        struct BitSlot *b = &slots[*pn];
        memset(b, 0, sizeof(*b));
        b->sda = (int)((byte >> i) & 1u);
        snprintf(b->phase, sizeof(b->phase), "%s d%d", tag, i);
        *pn += 1u;
    }
    struct BitSlot *ack = &slots[*pn];
    memset(ack, 0, sizeof(*ack));
    snprintf(ack->phase, sizeof(ack->phase), "ACK(从机)");
    ack->sda = 0;                 /* ACK = 接收方把 SDA 拉低 */
    ack->sda_is_ack = true;
    *pn += 1u;
}

static void demo_full_transaction(void)
{
    printf("============================================================\n");
    printf("演示三：主机写从机(0x50)寄存器 0x11=0xA5，再读回 —— 逐位波形\n");
    printf("============================================================\n");
    printf("   图例：'-'=主机驱动的 1，'_'=主机驱动的 0；'a'/'A'=ACK/NACK 拍（方向见标注）。\n");
    printf("         注意「数据 d7..d0」那一段处在读事务里，其实是【从机】驱动，符号沿用 -/_。\n");
    printf("         数据在 SCL 低（....）期间换，SCL 高（++++）期间被采样。\n\n");

    struct BitSlot slots[SLOT_MAX];
    size_t n = 0u;

    struct BitSlot *st = &slots[n++];
    memset(st, 0, sizeof(*st));
    snprintf(st->phase, sizeof(st->phase), "START");
    st->start = true;

    push_byte(slots, &n, (uint8_t)(0x50u << 1), "地址W");          /* 0xA0 = 0x50 写 */
    push_byte(slots, &n, 0x11u, "寄存器");

    struct BitSlot *rs = &slots[n++];
    memset(rs, 0, sizeof(*rs));
    snprintf(rs->phase, sizeof(rs->phase), "重复START");
    rs->start = true;

    push_byte(slots, &n, (uint8_t)((0x50u << 1) | 1u), "地址R");   /* 0xA1 = 0x50 读 */
    push_byte(slots, &n, 0xA5u, "数据");
    /* 主机收完最后一个字节回 NACK（不要再发），然后 STOP */
    struct BitSlot *last = &slots[n - 1u];
    snprintf(last->phase, sizeof(last->phase), "NACK(主机)");
    last->sda = 1;

    struct BitSlot *sp = &slots[n++];
    memset(sp, 0, sizeof(*sp));
    snprintf(sp->phase, sizeof(sp->phase), "STOP");
    sp->stop = true;

    /* 每 10 拍切一段打印 + 图例，免得一行太宽 */
    for (size_t off = 0; off < n; off += 10u) {
        const size_t len = (n - off < 10u) ? (n - off) : 10u;
        printf("   ---- 第 %zu~%zu 拍 ----\n", off + 1u, off + len);
        render_waveform(&slots[off], len);
        printf("        ");
        for (size_t i = 0; i < len; ++i) {
            printf("[%zu]%s ", off + i + 1u, slots[off + i].phase);
        }
        printf("\n\n");
    }
    printf("   看点：\n");
    printf("   · 地址拍 = 7 位地址 + 1 位 RW。手册写 0x50，总线上跑的是 0xA0/0xA1。\n");
    printf("     扫描工具报 0x50 还是 0xA0，就是「左移一位」这层窗户纸。\n");
    printf("   · 谁敢在 SCL 高期间改 SDA，除了合法的 START/STOP，就是毛刺。\n");
    printf("   · 读事务最后必须 NACK 再 STOP：你不 NACK，从机以为还要，一直占着总线。\n");
    printf("   · 重复 START 不换总线所有权，只是「同一主机换个方向接着聊」，\n");
    printf("     中途不发 STOP = 你没打算撒手，别的设备想插话也插不进来。\n\n");
}

/* ---------------------------------------------------------- */
/* 四、故障注入：stretching 超时 与 SDA 死锁 + 9 脉冲救援      */
/* ---------------------------------------------------------- */

static void demo_stretching(struct I2cBus *bus)
{
    printf("  【故障 A】从机 clock stretching：拉住 SCL 不放\n");
    bus_drive_scl(bus, 1u, true);               /* 从机拉住 SCL */
    const int timeout = 10;
    int waited = 0;
    printf("    主机发完第 8 个 SCL 上升沿想读 ACK，结果 SCL 被顶在 %d ——\n", bus_scl(bus));
    printf("    主机「等 SCL 变高」循环：");
    while (bus_scl(bus) == 0 && waited < timeout) {
        printf("等%d ", waited + 1);
        ++waited;
        if (waited >= 6) { bus_drive_scl(bus, 1u, false); }  /* 从机内部 ADC 忙完了，放手 */
    }
    if (bus_scl(bus) == 1) {
        printf("→ 第 %d 拍从机放手，SCL=%d，事务继续。（合理 stretching：从机忙，合法拖延）\n",
               waited, bus_scl(bus));
    } else {
        printf("→ 超时！SCL 还是 %d，必须走总线恢复，不然整机卡死在这。\n", bus_scl(bus));
    }
    printf("    教训：主机等待必须带超时；从机 stretching 超过你超时的设备，\n");
    printf("    要么调超时要么换货 —— 手册上那句 max stretching time 不是装饰。\n\n");
}

static void demo_deadlock_and_rescue(struct I2cBus *bus)
{
    printf("  【故障 B】经典死锁：主机被中断打断 1ms，回来时从机卡在地址第 8 拍\n");
    /* 现场：主机发完地址位被打断；从机已在 ACK 拍拉住 SDA 等第 8 个 SCL；
     * 主机状态机复位不再发 SCL；主机想重发 START，但 SDA 已被从机按低 */
    bus_drive_sda(bus, 1u, true);               /* 从机守着 ACK：SDA 拉低 */
    printf("    现场电平：SDA=%d（从机拉的） SCL=%d（空闲）\n", bus_sda(bus), bus_scl(bus));
    printf("    主机想发 START：需要 SCL 高期间 SDA 来一次下降沿 ——\n");
    printf("    但 SDA 被按死在 %d，主机 ODR 写破天也没用（线与）→ 死锁。\n", bus_sda(bus));
    printf("    症状复述：I2C 初始化后第一次事务被打断，之后「SCL 一动不动、SDA 常低」，\n");
    printf("    HAL 里表现为 HAL_TIMEOUT / 卡在 WAIT_BUSY，重启才活 —— 就是它。\n\n");
    printf("    【标准救援】把 SCL 引脚临时配成推挽输出，手动打 9 个脉冲：\n");
    for (int i = 1; i <= 9; ++i) {
        printf("    脉冲 %d/9：SCL 给一个高%s\n", i,
               (i == 9) ? " —— 从机吐完这个字节，状态机复位，松开 SDA！" : "，从机状态机往前走一位");
        if (i == 9) { bus_drive_sda(bus, 1u, false); }
    }
    printf("    补一个 STOP（SCL 高期间 SDA 来一次上升沿）：");
    bus_drive_sda(bus, 0u, true);               /* SDA 先拉低（准备 STOP 的起点）*/
    bus_drive_sda(bus, 0u, false);              /* 再放手 → 上拉抬升 = STOP 的上升沿 */
    printf("完成。\n");
    printf("    恢复后总线：SDA=%d SCL=%d —— 回到空闲态，把引脚改回 AF、重新初始化外设即可。\n\n",
           bus_sda(bus), bus_scl(bus));
}

static void demo_faults(void)
{
    printf("============================================================\n");
    printf("演示四：两种死法 —— stretching 超时 与 总线死锁 + 9 脉冲救援\n");
    printf("============================================================\n");
    struct I2cBus bus = { 0u, 0u };
    demo_stretching(&bus);
    demo_deadlock_and_rescue(&bus);
}

/* ---------------------------------------------------------- */
/* 收尾自测                                                    */
/* ---------------------------------------------------------- */

static void demo_quiz(void)
{
    printf("============================================================\n");
    printf("收尾：三个自测问题\n");
    printf("============================================================\n");
    printf("1) 3.3V 系统，总电容 200pF，要求 tr≤1000ns、Iol≤3mA，上拉允许区间？\n");
    printf("   答：Rmin=3.3/0.003=1.1k；Rmax=1000e-9/(0.847×200e-12)≈5.9k。取 2.2k~4.7k。\n");
    printf("2) 板上两颗从机，其中一颗没上电（VDD=0），整条 I2C 直接不通，为什么？\n");
    printf("   答：未上电从机的片上钳位二极管把总线电压拖到它的 VDD+0.3V ≈ 0.3V，\n");
    printf("      上拉电阻拉不起来（等于对地短路）。解决：用模拟开关/隔离器断电隔离，\n");
    printf("      或保证总线上的器件永远有电。手册没写「容忍断电」的引脚别硬接。\n");
    printf("3) 为什么 I2C 引脚必须开漏、SPI 的 MOSI/SCK 却是推挽？\n");
    printf("   答：I2C 要多设备线与（共用一条绳，谁都能踩刹车）；SPI 一主一从各驱各的线，\n");
    printf("      要的是快沿和强驱动。用推挽去驱 I2C = 跟从机对打，\n");
    printf("      高电平被压在 1V 上下、上下管长期直通发热 —— 示波器一看便知。\n");
}

int main(void)
{
    printf("############################################################\n");
    printf("#   第 4 关 · 03 I2C 总线软件模型（PC 上跑的假 SDA/SCL）      #\n");
    printf("############################################################\n\n");
    demo_wired_and();
    demo_pullup_calc();
    demo_full_transaction();
    demo_faults();
    demo_quiz();
    return 0;
}
