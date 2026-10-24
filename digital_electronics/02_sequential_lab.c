/* =====================================================================
 * 02_sequential_lab.c —— 时序逻辑周期精确仿真台（锁存器 vs 触发器 / 纹波 vs 同步）
 *
 * 【这个文件讲什么】
 *   一个纯软件的"周期精确"时序仿真器：每个 D 触发器、每个门都带传播延迟，
 *   每个时钟沿之后按延迟排队产生跳变，然后把每一次跳变的时刻（ns）打印出来。
 *   有了"跳变时刻"这个量，三件平时只能靠背的事就一目了然：
 *     1) 异步（纹波）计数器：第 4 位要等前三位传完才动，
 *        4 级 tpd 串联 → 最高时钟频率被最后那一位卡死，位数翻倍频率减半；
 *     2) 译码毛刺：状态 0111 → 1000 的路上，二进制值会路过 0110、0100、0000，
 *        你那颗"检测 6"的与非门就会输出一个假的尖峰 —— 仿真器直接告诉你
 *        "在 t = xx ns 出现持续 yy ns 的假状态"；
 *     3) 同步 + 并行进位为什么值钱：所有位同一时刻翻，毛刺窗口为 0。
 *   另外还跑：4 位移位寄存器、带使能/异步清空的计数器（演示异步清空的竞争）。
 *
 * 【怎么编译】（MSYS2 / MinGW，gcc 16.2 + Windows 下验证零警告）
 *   gcc -std=c17 -O2 -Wall -Wextra 02_sequential_lab.c -o 02_sequential_lab.exe
 *
 * 【怎么运行】
 *   chcp 65001        ← 先执行！不切 UTF-8 编码，Windows 控制台中文全是乱码
 *   .\02_sequential_lab.exe
 *
 * 【预期看到什么】
 *   - 器件模型：74HC74 的 CLK→Q 取 13 ns，74HC00 取 8 ns（5V、25℃ 典型值）；
 *   - 纹波计数器从 0111 走到 1000：4 次跳变分别在 +13 / +26 / +39 / +52 ns，
 *     中间出现 3 个假状态，其中最坏的是 0110(6) 持续 13 ns；
 *   - 同步 + 并行进位：4 位全部在 +13 ns 同时翻转，假状态 0 个；
 *   - 频率表：纹波 2 位 38.46 MHz → 4 位 19.23 MHz → 8 位 9.62 MHz → 16 位 4.81 MHz；
 *     同步并行进位恒定 76.92 MHz（只算一个 tpd 的稳定时间，位宽无关）；
 *   - 异步清空：清零信号到达时计数器停在 1001，Q0 在 +13 ns 先落地、Q3 在 +37 ns
 *     才落地，中间 24 ns 窗口里读数是 8（谁都没设计过的值）。
 *
 * 【老师傅点拨】
 *   记住这一句：异步电路的"最快"取决于最坏那条链，
 *   而最坏那条链只有在特定状态序列（0111→1000，一路进位到底）才会出现 ——
 *   所以你拿逻辑分析仪抓一百次都对，第一千零一次上板就翻车。
 *   同步设计把最坏情况变成固定情况，这才是它真正的价值，不是"整齐好看"。
 * ===================================================================== */

#include <stdio.h>

#define NBIT 4                       /* 计数器位宽（Q3 是 MSB） */
#define TPD_FF   13.0                /* 74HC74 D 触发器 CLK→Q，5V/25℃ 典型值，单位 ns */
#define TPD_GATE  8.0                /* 74HC00 二输入与非门，单位 ns */

/* 一次跳变事件：某个信号在 t 时刻从 from 变成 to */
typedef struct {
    double t;
    int    bit;
    int    from;
    int    to;
} Edge;

/* ---------------------------------------------------------------- 工具 */

static void print_bits(const char *tag, int v)
{
    printf("%s", tag);
    for (int i = NBIT - 1; i >= 0; i--) printf("%d", (v >> i) & 1);
}

static void print_state_table_head(void)
{
    printf("   # |  时钟沿 t(ns) | 各位跳变时刻（Q0→Q3，- 表示不动）          | 稳定后状态\n");
    printf("  ---+---------------+-------------------------------------------+-----------\n");
}

/*
 * 采样窗口：把一次时钟沿之后产生的所有跳变按时间排序，
 * 逐段读出"此刻二进制值等于几"，与真正的目标状态比较 → 找出假状态。
 * 这一步就是"毛刺"的机器定义：目标状态之外出现过的值。
 */
static int find_glitches(const Edge *ev, int n, int target, double t_edge,
                         int state_before)
{
    int shown = 0;
    int cur = state_before;
    int i = 0;

    while (i < n) {
        double t0 = ev[i].t;
        int    j = i;
        while (j < n && ev[j].t == t0) {                 /* 同一时刻一起变 */
            if (ev[j].to) cur |= (1 << ev[j].bit);
            else          cur &= ~(1 << ev[j].bit);
            j++;
        }
        double t1 = (j < n) ? ev[j].t : (ev[n - 1].t + TPD_FF);
        if (cur != target) {
            printf("       ! 在 t = %6.1f ns 出现假状态 %d%d%d%d (%2d)，持续 %4.1f ns\n",
                   t0 - t_edge + 0.0,
                   (cur >> 3) & 1, (cur >> 2) & 1, (cur >> 1) & 1, cur & 1,
                   cur, t1 - t0);
            shown++;
        }
        i = j;
    }
    return shown;
}

/* ------------------------------------------- 模型 1：异步（纹波）计数器 */

/*
 * 下降沿触发的纹波二进制加法器：
 *   Q0 的时钟 = 系统 CLK；Q1 的时钟 = Q0；Q2 的时钟 = Q1；……
 *   某位原来是 1 → 本位翻成 0（下降沿）→ 触发下一位；原来是 0 → 翻成 1，进位到此为止。
 *   时间：每多穿一位，就多付一个 TPD_FF。这就是"延迟累积"。
 */
static int ripple_step(int state, double t_edge, Edge *ev, int *newstate)
{
    int    n = 0;
    int    s = state;
    double t = t_edge;

    for (int i = 0; i < NBIT; i++) {
        int b = (s >> i) & 1;
        t += TPD_FF;                        /* 时钟边沿传到这一位，就要一个 tpd */
        if (b) {
            s &= ~(1 << i);
            ev[n].t = t; ev[n].bit = i; ev[n].from = 1; ev[n].to = 0; n++;
            /* 1→0 是下降沿，继续往上敲 */
        } else {
            s |= (1 << i);
            ev[n].t = t; ev[n].bit = i; ev[n].from = 0; ev[n].to = 1; n++;
            break;                          /* 0→1 是上升沿，进位链断掉 */
        }
    }
    *newstate = s;
    return n;
}

static void lab_ripple(void)
{
    int st = 0;

    printf("\n============ 实验 1：异步（纹波）4 位二进制计数器 ============\n");
    printf("接法：CLK→Q0 的 CP；Q0→Q1 的 CP；Q1→Q2 的 CP；Q2→Q3 的 CP（串糖葫芦）\n");
    printf("每个 D 触发器 CLK→Q = %.0f ns\n\n", TPD_FF);
    print_state_table_head();

    /* 特意从 6 开始，让 0111→1000 那一路"进位打到底"的最坏情况出现在第 2 行 */
    st = 6;
    for (int k = 1; k <= 6; k++) {
        double t_edge = 100.0 * (double)k;         /* 时钟周期这里取 100 ns，只为看清跳变 */
        Edge ev[NBIT];
        int before = st;
        int n = ripple_step(st, t_edge, ev, &st);
        int target = st;

        printf("   %d | %13.1f | ", k, t_edge);
        for (int i = 0; i < NBIT; i++) {
            int found = -1;
            for (int e = 0; e < n; e++) if (ev[e].bit == i) found = e;
            if (found >= 0) printf("+%.0f ", ev[found].t - t_edge);
            else            printf(" -  ");
        }
        printf("| ");
        print_bits("", target);
        printf("\n");

        if (n >= 2) {
            printf("       进位链穿过了 %d 位，最坏那位在时钟沿后 %.0f ns 才稳定\n",
                   n, ev[n - 1].t - t_edge);
            (void)find_glitches(ev, n, target, t_edge, before);
        }
    }

    printf("\n---- 最坏情况专列：0111 → 1000（进位一路打穿 4 位）----\n");
    {
        int before = 7;
        int after = 0;
        double t_edge = 0.0;
        Edge ev[NBIT];
        int n = ripple_step(before, t_edge, ev, &after);
        for (int e = 0; e < n; e++) {
            printf("   t = %5.1f ns : Q%d %d → %d\n", ev[e].t, ev[e].bit,
                   ev[e].from, ev[e].to);
        }
        printf("   => 译码器在这一路上依次\"看到\"：");
        int cur = before;
        for (int e = 0; e < n; e++) {
            if (ev[e].to) cur |= (1 << ev[e].bit); else cur &= ~(1 << ev[e].bit);
            printf(" %d%d%d%d", (cur >> 3) & 1, (cur >> 2) & 1, (cur >> 1) & 1, cur & 1);
        }
        printf("\n");
        printf("   => 目标状态是 1000(8)，但中间出现了 3 个假状态：\n");
        (void)find_glitches(ev, n, 8, 0.0, before);
        printf("   => 若用一片 74HC21（四输入与门）译码 \"0110\"，\n");
        printf("      你会得到一根宽约 13 ns 的正尖峰 —— 足够让下一级 MCU 的\n");
        printf("      外部中断白触发一次。这就是「毛刺」，不是玄学。\n");
    }
}

/* ------------------------------------- 模型 2：同步计数器（两种进位方式） */

/*
 * 同步：所有位的 CP 都接系统 CLK，所以"何时动"由时钟决定，不由前一级决定。
 *   串行使能（使能信号一级级传）：第 i 位在 时钟沿 + (i+1)×门延迟 后才翻
 *     —— 仍有毛刺，只是比异步纹波小。
 *   并行进位（所有触发器共用时钟，位间不走进位链）：所有使能项同时算好，
 *     4 位全部在 时钟沿 + TPD_FF 同时翻 —— 毛刺窗口 0。
 *   注：74HC393 是异步纹波计数器（实验 1 的模型），不是同步计数器；
 *       74HC161/163 才是所有位共时钟的同步计数器。
 */
static int sync_step(int state, double t_edge, int lookahead, Edge *ev)
{
    int next = (state + 1) & ((1 << NBIT) - 1);
    int n = 0;

    for (int i = 0; i < NBIT; i++) {
        int b_old = (state >> i) & 1;
        int b_new = (next >> i) & 1;
        if (b_old == b_new) continue;
        double t = lookahead ? (t_edge + TPD_FF)
                             : (t_edge + TPD_GATE * (double)(i + 1) + TPD_FF);
        ev[n].t = t; ev[n].bit = i; ev[n].from = b_old; ev[n].to = b_new; n++;
    }
    return n;
}

static void lab_sync(void)
{
    static const char *mode[2] = { "同步 + 串行使能（进位一级级传）",
                                   "同步 + 并行进位（74HC161/163 风格）" };

    for (int m = 0; m < 2; m++) {
        int st = 6;
        printf("\n============ 实验 2%c：%s ============\n",
               m ? 'b' : 'a', mode[m]);
        print_state_table_head();
        for (int k = 1; k <= 3; k++) {
            double t_edge = 100.0 * (double)k;
            Edge ev[NBIT];
            int next = (st + 1) & ((1 << NBIT) - 1);
            int n = sync_step(st, t_edge, m, ev);

            printf("   %d | %13.1f | ", k, t_edge);
            for (int i = 0; i < NBIT; i++) {
                int found = -1;
                for (int e = 0; e < n; e++) if (ev[e].bit == i) found = e;
                if (found >= 0) printf("+%.0f ", ev[found].t - t_edge);
                else            printf(" -  ");
            }
            printf("| ");
            print_bits("", next);
            printf("\n");
            if (k == 2) {                       /* 这一拍正好是 0111→1000 最坏情况 */
                int g = find_glitches(ev, n, next, t_edge, st);
                if (g == 0)
                    printf("       所有位同时翻转，假状态 0 个 —— 译码器干干净净\n");
                else
                    printf("       仍有 %d 个假状态窗口（上表 ! 行），只是比纹波窄\n", g);
            }
            st = next;
        }
    }
}

/* ------------------------------------------------ 模型 3：最高频率的账 */

static void lab_fmax(void)
{
    static const int W[4] = { 2, 4, 8, 16 };

    printf("\n============ 实验 3：位宽怎么吃掉你的最高时钟频率 ============\n");
    printf("规则：时钟周期必须 > 一个时钟沿内最长那条链的稳定时间（再加裕量）\n\n");
    printf("   位宽 |  纹波最长链 (ns) | 纹波 fmax (MHz) | 同步并行链 (ns) | 同步 fmax (MHz)\n");
    printf("  ------+------------------+-----------------+-----------------+---------------\n");
    for (int i = 0; i < 4; i++) {
        double ripple = TPD_FF * (double)W[i];
        double sync   = TPD_FF;               /* 并行进位 + 同一时钟，稳定时间 = 1 个 tpd */
        printf("    %3d |     %7.0f     |     %6.2f      |       %5.0f      |     %6.2f\n",
               W[i], ripple, 1000.0 / ripple, sync, 1000.0 / sync);
    }
    printf("\n师傅的话：纹波计数器每加一位，最高频率对折一次 —— 16 位只剩 %.2f MHz，\n",
           1000.0 / (TPD_FF * 16.0));
    printf("          而且「16 位能跑」的前提是你能容忍最后那一位晚 208 ns 才出来。\n");
    printf("          同步计数器把这个依赖彻底砍掉，代价是每多一位多几个门。\n");
    printf("          这就是「用面积换时序」的第一次见面，后面 FPGA 里天天见。\n");
}

/* ------------------------------------------------ 模型 4：4 位移位寄存器 */

static void lab_shiftreg(void)
{
    static const int din[8] = { 1, 0, 1, 1, 0, 0, 1, 0 };
    int reg = 0;

    printf("\n============ 实验 4：4 位串入并出移位寄存器 ============\n");
    printf("每拍数据进 Q3，整体右移一位。所有位共享同一个 CLK → 同时翻，无累积延迟。\n\n");
    printf("   拍 |  t (ns) |  DIN |  跳变（Q3 Q2 Q1 Q0 相对时钟沿）  |  稳定后 Q3..Q0\n");
    printf("  ----+---------+------+----------------------------------+---------------\n");
    for (int k = 1; k <= 8; k++) {
        double t_edge = 40.0 * (double)k;
        int prev = reg;
        reg = ((reg >> 1) | (din[k - 1] << 3)) & 0xF;

        printf("   %2d | %7.1f |    %d |  ", k, t_edge, din[k - 1]);
        int moving = 0;
        for (int i = NBIT - 1; i >= 0; i--) {
            int a = (prev >> i) & 1, b = (reg >> i) & 1;
            if (a != b) { printf("+%.0f ", TPD_FF); moving = 1; }
            else        printf(" -  ");
        }
        if (!moving) printf("          （本拍无跳变）");
        printf(" | ");
        print_bits("", reg);
        printf("\n");
    }
    printf("\n注意：8 拍之后寄存器里是 %d%d%d%d —— 串行进、并行出，\n",
           (reg >> 3) & 1, (reg >> 2) & 1, (reg >> 1) & 1, reg & 1);
    printf("      这就是 74HC595 的全部原理。它没有累积延迟，是因为它不靠上一位驱动下一位的时钟。\n");
}

/* ------------------------------ 模型 5：带使能与异步清空的计数器（竞争） */

static void lab_enable_clear(void)
{
    printf("\n============ 实验 5：使能门控 + 异步清空的竞争 ============\n");
    printf("74HC393 一类片子有异步清零脚：不等同步沿，脚上一到低，\n");
    printf("所有触发器立刻（各付一个 tpd）被清掉 —— 于是清零瞬间会出现竞争窗口。\n\n");

    double t_clear = 0.0;
    /* 简化模型：清零缓冲器是一棵树，扇出到 4 位；最慢那位多走两级门 */
    double t_fast = t_clear + TPD_FF;             /* Q0 走线短，最先落地 */
    double t_slow = t_clear + TPD_FF + 3 * TPD_GATE; /* Q3 走线最长，最后落地 */

    printf("   清零瞬间计数器停在 1001(9)：Q0=1、Q3=1，其余为 0\n");
    printf("   清零信号到达后各输出跳变（走线长短不一样）：\n");
    printf("     t = %5.1f ns : Q0 1 → 0（走线最短那条）\n", t_fast);
    printf("     t = %5.1f ns : Q3 1 → 0（走线最长那条，多穿 3 级缓冲）\n", t_slow);
    printf("   => 在 %.1f ~ %.1f ns 这 %.1f ns 里，计数器读数是 8 —— 一个谁都没设计过的值。\n",
           t_fast, t_slow, t_slow - t_fast);
    printf("   => 彻底清零要等到 %.1f ns。这一段时间里你去读它、\n", t_slow);
    printf("      或者用它的输出做地址去选东西，就会选到莫名其妙的通道。\n\n");

    printf("   使能门控（同步使能，正确姿势）：EN 经一个与非门进下一级的使能链，\n");
    for (int k = 1; k <= 4; k++) {
        double t_edge = 100.0 * (double)k;
        int en = (k != 3);
        if (en) {
            printf("     拍 %d：EN=1  计数在 %.0f ns 完成（与非门 %.0f + 触发器 %.0f）\n",
                   k, t_edge + TPD_GATE + TPD_FF, TPD_GATE, TPD_FF);
        } else {
            printf("     拍 %d：EN=0  本拍冻结，触发器不翻转（使能链仍付 %.0f ns，但没后果）\n",
                   k, TPD_GATE);
        }
    }
    printf("\n师傅的话：清一个计数器，用同步清；清一整板，用「上电复位 + 延时再释放」。\n");
    printf("          异步清零用来救急（看门狗、急停），不用来做常规逻辑。\n");
}

/* -------------------------------------------- 锁存器 vs 触发器（一句话演示） */

static void lab_latch_vs_flop(void)
{
    printf("\n============ 实验 6：锁存器是「透明」的，触发器才「取样」 ============\n");
    printf("场景：CP 高电平期间，D 上恰好发生一次 0→1→0 的抖动（10 ns 宽）。\n\n");
    printf("   t (ns) |   CLK |    D  |  锁存器 Q（74HC75/HC373） |  触发器 Q（74HC74）\n");
    printf("  -------+-------+-------+---------------------------+--------------------\n");
    static const double tt[8] = { 0, 10, 20, 30, 40, 50, 60, 70 };
    static const int    ck[8] = { 0,  1,  1,  1,  1,  1,  0,  0 };
    static const int    dd[8] = { 0,  0,  1,  0,  0,  0,  0,  0 };
    int lq = 0, fq = 0, pd = 0;
    for (int i = 0; i < 8; i++) {
        if (ck[i] == 1) lq = dd[i];                              /* 高电平期间透明跟随 */
        if (ck[i] == 1 && pd == 0) fq = dd[i];                   /* 上升沿那一瞬取样 */
        printf("   %6.0f |   %d   |   %d   |            %d                 |          %d\n",
               tt[i], ck[i], dd[i], lq, fq);
        pd = ck[i];
    }
    printf("\n结论：锁存器把 D 上的每个毛刺都原样搬到 Q；触发器只认上升沿那一瞬的值。\n");
    printf("        这就是「能用触发器就别用锁存器」的实感版本 ——\n");
    printf("        用锁存器你还得多给一级时钟反相，时序分析直接翻倍复杂度。\n");
}

int main(void)
{
    printf("=================================================================\n");
    printf("      数字电路 · 时序逻辑周期精确仿真台（跳变时刻，单位 ns）\n");
    printf("=================================================================\n");
    printf("器件模型：D 触发器 CLK→Q = %.0f ns；与非门 = %.0f ns（74HC 系列 5V/25℃ 典型）\n",
           TPD_FF, TPD_GATE);
    printf("读表方法：表里的 +13 表示「这个信号在时钟沿之后 13 ns 才翻」。\n");
    printf("          凡是「时钟沿后 52 ns 才稳定」的信号，都不能在那个窗口里被译码使用。\n");

    lab_ripple();
    lab_sync();
    lab_fmax();
    lab_shiftreg();
    lab_enable_clear();
    lab_latch_vs_flop();

    printf("\n=================================================================\n");
    printf(" 收尾三句：\n");
    printf(" 1. 纹波（异步）电路的最坏延迟 = 级数 × tpd，且只在特定状态序列才出现。\n");
    printf(" 2. 毛刺不是「坏了」，是「各位不同时到位」的必然结果，用同步 + 并行进位压掉。\n");
    printf(" 3. 任何一位的读数都有建立/保持时间的问题 —— 下一课 03_setup_hold.c 见。\n");
    printf("=================================================================\n");
    return 0;
}
