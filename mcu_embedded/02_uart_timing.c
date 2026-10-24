/*
 * ============================================================
 *  第 4 关 · 02_uart_timing.c
 *  —— 波特率到底是怎么算出来的：BRR 计算器 + UART 帧时序模拟器
 *     + 无锁环形缓冲区丢包现场
 * ============================================================
 *
 * 【讲什么】
 *   1) USARTDIV = fck / (16 × Baud)，OVERSAMPLING16 下
 *      BRR = ( mantissa << 4 ) | fraction，fraction 只有 4 位（1/16 步进），
 *      所以波特率永远是「近似值」——本程序把 8/16/72/168 MHz ×
 *      9600/115200/921600 共 12 种组合全算一遍，误差 >2% 标红。
 *   2) 误差预算推导：8N1 一帧 10 位，接收端在起始位下降沿重新对齐，
 *      之后每个采样点在「位中点」，离中点最远的一格是停止位（第 9.5 位），
 *      总账 = 0.5 bit / 9.5 bit ≈ ±5.26%，两边时钟平分 → 每边 ±2.6%，
 *      再扣掉起始沿检测误差和晶振温漂 → 工程红线 ±2%。
 *   3) 帧时序模拟：起始位低、8 位数据 LSB first、可选偶校验、停止位高，
 *      用「发送端位栅格 + 接收端采样时刻」模拟时钟偏差，看 3% 链路
 *      是活是死、6% 链路怎么死（帧错误 / 数据被停止位污染）。
 *   4) 无锁环形缓冲区：volatile head/tail、容量 2 的幂（位与代替取模）、
 *      ISR 只写 head、主循环只写 tail。主循环喝口水的功夫，
 *      115200 就给你灌满丢包，现场计数给你看。
 *
 * 【怎么编译运行】（MSYS2 / gcc 16.2，Windows，C17 零警告）
 *   chcp 65001            ← 运行前先执行这一句，否则 Windows 控制台中文乱码
 *   gcc -std=c17 -O2 -Wall -Wextra 02_uart_timing.c -o 02_uart_timing.exe
 *   .\02_uart_timing.exe
 *   没有开发板也能跑：所有 USART 行为都是软件模型，纯打印。
 *   注：本文件不用 M_PI（-std=c17 严格模式下 MSYS2 不暴露它），
 *       需要圆周率就自己 #define PI 3.14159265358979323846 —— 这里算波特率用不到。
 *
 * 【预期看到什么】
 *   - 一张 12 行的 BRR 表：72MHz 跑 115200 误差 0.00%（教科书频率的
 *     秘密）；8MHz 跑 921600 误差约 -3.55%、16MHz 跑 921600 约 +2.12%，
 *     两行标红（终端直跑时有 ANSI 红色，重定向到文件时以 ▲ 代替）。
 *   - 帧模拟：±0% 完美；3% 相对偏差活着但裕量只剩 0.215 bit；
 *     RX 快 6% → 停止位采样落进最后一个数据位 → 帧错误；
 *     RX 慢 6% → 最后一个数据位被停止位污染 → 数据 0x55 变 0xD5。
 *   - 环形缓冲区：主循环每 2 拍才收 1 个字节，16 拍后缓冲区顶满，
 *     后面每拍丢 1 个，共丢 4 个；改成「每拍清空」后丢包 = 0。
 *
 * 【老师傅点拨】
 *   - 串口乱码先别怀疑代码，先怀疑波特率误差：把两边的 BRR 都算出来，
 *     单边误差 >2% 就该查；方向相反、合计接近 4% 就快翻车了。
 *     8MHz 晶振跑 115200 是经典坑中坑。
 *   - 波特率是相对量：两边同偏（都偏 1%）等于没偏；怕的是「一个快一个慢」。
 *   - 缓冲区丢包不报错是最阴的 bug：协议层看到的永远是「缺字节」，
 *     一定要把 overflow 计数器读出来打日志，丢一个也要知道。
 *   - 单生产者单消费者才敢这么「无锁」：head 只有 ISR 碰、tail 只有
 *     主循环碰，各自单向递增，靠无符号回绕和 2 的幂掩码保平安。
 *     多生产者？老实上临界区。
 *   - C 里写环形缓冲就靠三样东西：volatile 的 head/tail、2 的幂容量 +
 *     `& (CAP - 1)` 掩码、无符号减法算占用量。少了 volatile，编译器把
 *     「等 head 变」优化成死循环；少了 2 的幂，你就得用 %， ISR 里多一次除法。
 */

#include <stdio.h>
#include <stdint.h>
#include <inttypes.h>   /* PRIu32 / PRIX32 */
#include <stdbool.h>
#include <stddef.h>
#include <string.h>     /* strlen / memset */
#include <math.h>       /* floor / fabs / llround */
#include <io.h>         /* _isatty / _fileno：判断是不是在终端里直跑 */

/* 一帧最多 11 位（起 + 8 数据 + 偶校验 + 停），固定数组就够，不用动态分配 */
#define FRAME_MAX_BITS 11

struct Frame {
    int bits[FRAME_MAX_BITS];
    size_t n;
};

/* ---------------------------------------------------------- */
/* 一、BRR 计算器                                              */
/* ---------------------------------------------------------- */

static const char *RED = "";
static const char *RST = "";

static void demo_brr_table(void)
{
    printf("============================================================\n");
    printf("演示一：USARTDIV = fck/(16×Baud)，BRR = (M<<4)|F 全表\n");
    printf("============================================================\n");
    printf("  fraction 只有 4 bit，步进 1/16 = 0.0625 —— 所以只能凑，凑不匀就是误差。\n\n");
    printf("  系统时钟    目标波特率   USARTDIV     M     F    BRR     实际波特率    误差\n");
    printf("  ---------  ----------  ----------  ----  ---  ------  -----------  ------\n");

    static const double fck_mhz[4] = {8.0, 16.0, 72.0, 168.0};
    static const uint32_t bauds[3] = {9600u, 115200u, 921600u};

    for (size_t i = 0; i < 4u; ++i) {
        for (size_t j = 0; j < 3u; ++j) {
            const double mhz = fck_mhz[i];
            const uint32_t baud = bauds[j];
            const double fck = mhz * 1e6;
            const double div = fck / (16.0 * (double)baud);
            uint32_t mant = (uint32_t)div;                            /* 整数部分 */
            uint32_t frac = (uint32_t)llround((div - (double)mant) * 16.0);
            if (frac == 16u) { mant += 1u; frac = 0u; }               /* 四舍五入进位 */
            /* BRR 寄存器：高 12 位是整数、低 4 位是分数 —— C 里就是移位拼字 */
            const uint32_t brr = (mant << 4u) | frac;
            const double act_div = (double)mant + (double)frac / 16.0;
            const double act = fck / (16.0 * act_div);
            const double err = (act - (double)baud) / (double)baud * 100.0;
            const bool over = fabs(err) > 2.0;
            printf("  %5.0f MHz  %9" PRIu32 "  %10.4f  %4" PRIu32 "  %3" PRIu32
                   "  0x%03" PRIX32 "  %10.1f  %+6.2f%%  %s%s%s\n",
                   mhz, baud, div, mant, frac, brr, act, err,
                   over ? RED : "", over ? "▲超差(>2%)" : "✓", over ? RST : "");
        }
    }
    printf("\n  看点 1：72MHz 时 115200 的 USARTDIV = 39.0625，分数部分恰好是 1/16 的整数倍，\n");
    printf("          BRR=0x271，误差 0.00%% —— 老芯片选 72M/16M 这类「整数友好」频率不是巧合。\n");
    printf("  看点 2：8MHz 跑 921600 凑出来是 -3.55%%，16MHz 也才 +2.12%%，双双超差。\n");
    printf("          结论：921600 这种高波特率要么换时钟源，要么换 OVERSAMPLING8（见下），\n");
    printf("          要么认命降速。查错时先翻 RCC 确认 fck 真是你以为的那个值。\n");
    printf("  看点 3：STM32 还有 OVERSAMPLING8 模式：公式变 fck/(8×Baud)。\n");
    printf("          它把分频比减半，同样时钟能达到的最高波特率翻倍；\n");
    printf("          代价是分数步进从 1/16 变粗到 1/8，且每 bit 只采 8 次、抗噪变差。\n");
    printf("          它是「高波特率工具」，不是「精度工具」——别指望误差减半。\n");
    printf("  红线：终端里超差是红色（ANSI），文件里用 ▲。记住这句：\n");
    printf("        「波特率对不上，先算 BRR 再查线」。\n\n");
}

/* ---------------------------------------------------------- */
/* 二、误差预算推导                                            */
/* ---------------------------------------------------------- */

static void demo_error_budget(void)
{
    printf("============================================================\n");
    printf("演示二：±2%% 这条红线是怎么推导出来的\n");
    printf("============================================================\n");
    printf("  接收端靠起始位的下降沿重新对齐，之后每个 bit 在「中点」采样。\n");
    printf("  离对齐点最远的采样是停止位，在第 (N - 0.5) 个位周期处（N = 帧总位数）。\n\n");
    const double budget10 = 0.5 / 9.5 * 100.0;   /* 8N1：10 位 */
    const double budget11 = 0.5 / 10.5 * 100.0;  /* 8E1：11 位 */
    printf("  8N1（10 位）：总预算 = 0.5 bit / 9.5 bit = ±%.2f%% —— 这是发送+接收的合计漂移\n", budget10);
    printf("  8E1（11 位）：总预算 = 0.5 bit / 10.5 bit = ±%.2f%% —— 加校验位，预算更紧\n\n", budget11);
    printf("  两边晶振平分总预算：±%.2f%% / 2 ≈ ±2.6%%。\n", budget10);
    printf("  再扣掉：起始沿检测误差（量化 ±1/16 位）、温漂（普通晶振 ±10~30ppm 还好，\n");
    printf("  陶瓷谐振器 ±0.5%% 起步）、老化 —— 工程上把红线定在每边 ±2%%，\n");
    printf("  两边加起来按最坏 ±4%% 留裕量。手册里的「误差 <2%%」通常指单边，\n");
    printf("  不是两边合计；两边同偏 1%% 等于没偏，一快一慢才是灾难。\n\n");
    printf("  波特率是相对量：TX 偏 +1%%、RX 也偏 +1%% → 相对偏差 0 → 一点事没有；\n");
    printf("  TX 偏 +1.5%%、RX 偏 -1.5%% → 相对 3%% → 已经在啃裕量了。\n\n");
}

/* ---------------------------------------------------------- */
/* 三、UART 帧收发时序模拟                                     */
/* ---------------------------------------------------------- */

/* 组帧：起始位 0 + 8 位数据 LSB first + （可选偶校验）+ 停止位 1 */
static struct Frame build_frame(uint8_t data, bool even_parity)
{
    struct Frame f;
    size_t k = 0;
    f.bits[k++] = 0;                                        /* 起始位：低 */
    for (int i = 0; i < 8; ++i) {
        f.bits[k++] = (data >> i) & 1u;                     /* LSB first */
    }
    if (even_parity) {
        int p = 0;
        for (int i = 0; i < 8; ++i) { p ^= (int)((data >> i) & 1u); }
        f.bits[k++] = p;                                    /* 偶校验：让 1 的个数为偶 */
    }
    f.bits[k++] = 1;                                        /* 停止位：高 */
    f.n = k;
    return f;
}

/* 发送端电平：第 i 位占 [i*txT, (i+1)*txT)，帧外为空闲高 */
static int tx_level_at(double t, const struct Frame *f, double txT)
{
    const long i = (long)floor(t / txT + 1e-12);
    if (i < 0 || i >= (long)f->n) { return 1; }
    return f->bits[(size_t)i];
}

struct SimResult {
    uint8_t got;
    bool start_ok;
    bool parity_ok;
    bool frame_err;
    double worst_margin;                 /* 最小的（采样点到位边界距离），单位 bit */
    int sampled[FRAME_MAX_BITS];
    size_t n;
};

/* r = 接收端位周期 / 发送端位周期（相对偏差 = r - 1）。
 * 接收端在起始下降沿处清零自己的计数器，第 k 位在 (k+0.5)*r 处采样。*/
static struct SimResult rx_simulate(const struct Frame *f, double r, bool even_parity)
{
    struct SimResult res;
    res.start_ok = true;
    res.parity_ok = true;
    res.frame_err = false;
    res.worst_margin = 100.0;
    res.n = f->n;
    for (size_t k = 0; k < f->n; ++k) {
        const double pos = ((double)k + 0.5) * r;                    /* 采样时刻（发送位栅格）*/
        const double offset = pos - ((double)k + 0.5);               /* 相对本位中点的漂移 */
        const double margin = 0.5 - fabs(offset);
        if (margin < res.worst_margin) { res.worst_margin = margin; }
        res.sampled[k] = tx_level_at(pos, f, 1.0);
    }
    if (res.sampled[0] != 0) { res.start_ok = false; }
    uint8_t data = 0;
    for (int i = 0; i < 8; ++i) {
        if (res.sampled[1 + (size_t)i] != 0) { data |= (uint8_t)(1u << i); }
    }
    res.got = data;
    size_t stop_idx = 9;
    if (even_parity) {
        stop_idx = 10;
        int p = 0;
        for (int i = 0; i < 8; ++i) { p ^= (int)((data >> i) & 1u); }
        res.parity_ok = (res.sampled[9] == p);
    }
    res.frame_err = (res.sampled[stop_idx] != 1);
    return res;
}

static void print_frame_row(const char *name, const struct Frame *f, double r, uint8_t sent)
{
    const struct SimResult s = rx_simulate(f, r, false);
    printf("  %-24s r=%.3f  收到=0x%02X", name, r, s.got);
    if (s.frame_err)      { printf("  ✗ 帧错误(停止位读到0)"); }
    else if (!s.start_ok) { printf("  ✗ 起始位异常"); }
    else if (s.got != sent) { printf("  ✗ 数据被污染(0x%02X→0x%02X)", sent, s.got); }
    else                  { printf("  ✓ 解出正确"); }
    printf("  最坏裕量=%+.3f bit\n", s.worst_margin);
}

static void demo_frame_sim(void)
{
    printf("============================================================\n");
    printf("演示三：发一个 0x55（8N1），给链路加时钟偏差看它是死是活\n");
    printf("============================================================\n");
    const uint8_t byte = 0x55u;                    /* 0b01010101，bit7=0 —— 帧错误试金石 */
    const struct Frame bits = build_frame(byte, false);
    printf("  帧内容（时间从左到右）：");
    for (size_t i = 0; i < bits.n; ++i) { printf("%d", bits.bits[i]); }
    printf("   （起 + D0..D7 LSB first + 停）\n\n");

    /* 逐位采样表：r = 1.03（RX 慢 3%）*/
    printf("  【r = 1.030：接收端慢 3%%】逐位采样（pos = 采样点落在发送位栅格的位置）\n");
    printf("   位 | 期望 | 采样pos | 落进的发送位 | 读到 | 裕量(bit)\n");
    for (size_t k = 0; k < bits.n; ++k) {
        const double pos = ((double)k + 0.5) * 1.03;
        const long ti = (long)floor(pos);
        const int lvl = tx_level_at(pos, &bits, 1.0);
        const double margin = 0.5 - fabs(pos - ((double)k + 0.5));
        printf("   %2zu |  %d   |  %7.3f |      %2ld     |  %d   |  %+.3f%s\n",
               k, bits.bits[k], pos, ti, lvl, margin,
               lvl != bits.bits[k] ? "   ← 采错！" : (margin < 0.25 ? "   ← 危险边缘" : ""));
    }
    print_frame_row("① r=1.000 双方都准", &bits, 1.000, byte);
    print_frame_row("② r=1.030 RX慢3%", &bits, 1.030, byte);
    print_frame_row("③ r=1.060 RX慢6%", &bits, 1.060, byte);
    print_frame_row("④ r=0.940 RX快6%", &bits, 0.940, byte);

    printf("\n  死法解读：\n");
    printf("   ② 3%%：最坏采样点 9.5×1.03 = 9.785，还在停止位（9~10）里，活着，\n");
    printf("      但裕量只剩 0.215 bit——起始沿检测再抖一点、线上再加毛刺，就是说。\n");
    printf("   ③ 慢 6%%：D7 采样点 8.5×1.06 = 9.01，跨进了停止位（值 1）→ 0x55 被读成\n");
    printf("      0xD5（bit7 由 0 变 1）；停止位采样落在空闲高电平上反而「正常」——\n");
    printf("      所以慢死法是数据错、帧没错，最难查：对端收到的永远是「最后一位翻掉」的字节。\n");
    printf("   ④ 快 6%%：停止位采样点 9.5×0.94 = 8.93，落进 D7（0x55 的 bit7 = 0）→\n");
    printf("      停止位读到 0 → 硬件直接置 FE（帧错误）。快死法反而好查，有标志位。\n");

    /* 校验位场景：预算更紧 */
    const struct Frame bits_e = build_frame(byte, true);
    printf("\n  换成 8E1（11 位帧）再跑 r=1.05：\n");
    const struct SimResult se = rx_simulate(&bits_e, 1.05, true);
    printf("   最坏裕量 %.3f bit，校验%s，帧%s —— 校验位把停止位推远一位，预算变小。\n",
           se.worst_margin, se.parity_ok ? "过" : "错", se.frame_err ? "错" : "对");
    printf("   老师傅：现场看到「校验错误偶发、换 8N1 就好」，八成是偏差踩着红线跑。\n\n");
}

/* ---------------------------------------------------------- */
/* 四、无锁环形缓冲区（ISR 写、主循环读）                      */
/* ---------------------------------------------------------- */

#define RING_CAP 8u                        /* 必须是 2 的幂：& 掩码代替 % */

struct Ring {
    volatile uint32_t head;                /* 只允许生产者（ISR）写 */
    volatile uint32_t tail;                /* 只允许消费者（主循环）写 */
    uint8_t  buf[RING_CAP];
    uint32_t overflow;                     /* 丢包计数，必须暴露出来 */
};

static void ring_init(struct Ring *r)
{
    r->head = 0u; r->tail = 0u; r->overflow = 0u;
    memset(r->buf, 0, sizeof r->buf);
}

static bool ring_push(struct Ring *r, uint8_t b)   /* ISR 上下文调用 */
{
    if (r->head - r->tail >= RING_CAP) {           /* 无符号回绕天然正确 */
        r->overflow++;
        return false;
    }
    r->buf[r->head & (RING_CAP - 1u)] = b;         /* 2 的幂：& 就是 % */
    r->head++;
    return true;
}

static bool ring_pop(struct Ring *r, uint8_t *b)   /* 主循环调用 */
{
    if (r->tail == r->head) { return false; }      /* 空 */
    *b = r->buf[r->tail & (RING_CAP - 1u)];
    r->tail++;
    return true;
}

static void demo_ring_buffer(void)
{
    printf("============================================================\n");
    printf("演示四：115200 进包 vs 主循环「每 2 拍收 1 个」—— 丢包现场\n");
    printf("============================================================\n");
    printf("  115200 波特 8N1 = 11520 字节/秒 ≈ 每 87µs 到 1 个；本模型 1 拍 = 1 个字节到达。\n");
    printf("  容量 %u（2 的幂，head&7 即下标），ISR 只管 push，主循环每 2 拍才 pop 1 个。\n\n", RING_CAP);

    const char pattern[] = "HELLO-STM32";
    const uint32_t plen = (uint32_t)strlen(pattern);

    struct Ring ring;
    ring_init(&ring);
    uint32_t pushed = 0u, popped = 0u;
    printf("   拍 | 收到 | 入队 | 丢包 | 队列 | 主循环动作\n");
    printf("   ---+------+------|------|------|-----------\n");
    for (uint32_t tick = 0; tick < 20u; ++tick) {
        const uint8_t b = (uint8_t)pattern[tick % plen];
        const bool ok = ring_push(&ring, b);       /* UART RX 中断：来一个收一个 */
        if (ok) { pushed++; }
        uint8_t got = 0u;
        const bool busy = (tick % 2u == 0u) && ring_pop(&ring, &got);  /* 主循环手慢 */
        if (busy) { popped++; }
        if ((tick % 2u == 1u) || !ok) {
            printf("   %2" PRIu32 " | 0x%02X |  %c    |  %" PRIu32
                   "    |  %2" PRIu32 "   | %s\n",
                   tick, b, ok ? 'Y' : 'N', ring.overflow, ring.head - ring.tail,
                   busy ? "收 1 个" : "在忙别的");
        }
    }
    printf("   总计：到达 %" PRIu32 "，入队 %" PRIu32 "，收走 %" PRIu32 "，还在队里 %" PRIu32
           "，溢出丢包 %" PRIu32 "\n",
           pushed + ring.overflow, pushed, popped, ring.head - ring.tail, ring.overflow);
    printf("   对账：入队 = 收走 + 在队 → %" PRIu32 " = %" PRIu32 " + %" PRIu32 "？%s（丢包只能由 overflow 计数器记账，\n",
           pushed, popped, ring.head - ring.tail,
           (pushed == popped + (ring.head - ring.tail)) ? "账对得上 ✓" : "账不对！");
    printf("         协议层永远看不见它们——这就是为什么丢包计数必须主动上报）\n");
    printf("\n  改成每拍把队列清空（while 里 pop 到空为止）：\n");
    ring_init(&ring);
    pushed = 0u; popped = 0u;
    for (uint32_t tick = 0; tick < 20u; ++tick) {
        if (ring_push(&ring, (uint8_t)pattern[tick % plen])) { pushed++; }
        uint8_t got = 0u;
        while (ring_pop(&ring, &got)) { popped++; }
    }
    printf("   入队 %" PRIu32 "，收走 %" PRIu32 "，丢包 %" PRIu32 " —— 一个不丢。\n\n",
           pushed, popped, ring.overflow);
    printf("  算账：主循环被高优先级任务压住 50ms，115200 会灌进 11520×0.05 ≈ 576 字节，\n");
    printf("        深度 256 的缓冲都兜不住。所以大流量协议要么 DMA + IDLE 中断，\n");
    printf("        要么把主循环单步耗时的东西（printf！Flash 写！）挪走。\n");
    printf("  三条铁律：容量 2 的幂；单生产者单消费者；overflow 计数器必须被看见。\n\n");
}

/* ---------------------------------------------------------- */
/* 收尾自测                                                    */
/* ---------------------------------------------------------- */

static void demo_quiz(void)
{
    printf("============================================================\n");
    printf("收尾：三个自测问题\n");
    printf("============================================================\n");
    printf("1) 72MHz 跑 9600，BRR 该写多少？\n");
    printf("   答：USARTDIV=468.75，M=468，F=0.75×16=12 → BRR=(468<<4)|12=0x1D4C，误差 0。\n");
    printf("2) 对端 115200，你这板子用 HSI（8MHz RC，精度 ±1%% 起步、温漂大），能长通吗？\n");
    printf("   答：BRR 本身凑出 +0.64%%，再叠 HSI ±1%% 以上漂移，相对偏差奔着 2%% 去 ——\n");
    printf("      短通长挂、天冷挂天热也挂。高波特率请用 HSE 晶振，别让 RC 干这活。\n");
    printf("3) 为什么 ISR 里禁止 printf？跟本文件哪段演示有关？\n");
    printf("   答：printf 一跑几毫秒，UART 中断里做这事 = 把后面所有中断堵死，\n");
    printf("      演示四里主循环只是慢一点，20 拍就丢 2 个包，ISR 里慢 5ms 就是丢 57 个，\n");
    printf("      还可能把更高优先级的 tick/看门狗节拍打断。ISR 只置标志，日志进环形队列。\n");
}

int main(void)
{
    if (_isatty(_fileno(stdout))) {               /* 终端直跑才上色，重定向保持纯文本 */
        RED = "\033[31m";
        RST = "\033[0m";
    }
    printf("############################################################\n");
    printf("#   第 4 关 · 02 UART 波特率与时序软件模型（PC 上跑的假 USART） #\n");
    printf("############################################################\n\n");
    demo_brr_table();
    demo_error_budget();
    demo_frame_sim();
    demo_ring_buffer();
    demo_quiz();
    return 0;
}
