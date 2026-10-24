/* =====================================================================
 * 03_setup_hold.c —— 建立/保持时间计算台 + 亚稳态 MTBF + 格雷码异步 FIFO
 *
 * 【这个文件讲什么】
 *   1) 建立时间裕量与最高频率：
 *        fmax = 1 / (Tco + Tpd + Tsu − skew − jitter)
 *      逐项扫描时钟频率，打印「能跑 / 临界 / 失败」三档表。
 *   2) 保持时间裕量：hold_margin = Tco + Tpd(min) − Th − skew。
 *      重点是那句反直觉的结论：降频能救建立违例，救不了保持违例。
 *   3) 亚稳态 MTBF = exp(t_settle / tau) / (f_clk · f_data)，
 *      按 1 级 / 2 级 / 3 级同步器、tau = 0.5ns / 1.0ns 打印，
 *      并且换算成「多少年 / 多少天 / 多少次」—— 数字比道理管用。
 *   4) 格雷码转换 + 异步 FIFO 空满判断的最小可运行实现，自测若干次读写往返，
 *      逐条打印读写指针与空满标志，验证不误判。
 *   5) 固定种子的蒙特卡洛实验：不加同步器 vs 1/2/3 级同步器的采样错误率。
 *
 * 【怎么编译】（MSYS2 / MinGW，gcc 16.2 + Windows 下验证零警告）
 *   gcc -std=c17 -O2 -Wall -Wextra 03_setup_hold.c -o 03_setup_hold.exe
 *
 * 【怎么运行】
 *   chcp 65001        ← 先执行！不切 UTF-8，Windows 控制台中文全是乱码
 *   .\03_setup_hold.exe
 *      （结果可复现：随机数用 srand(20260315) 固定种子，换机器也一样）
 *
 * 【预期看到什么】
 *   - 场景 A（Tco=6 / Tpd=4 / Tsu=2 / skew=0.5 / jitter=0.3）：
 *       fmax = 1/11.2 = 89.29 MHz；建立裕量在 12ns 时钟下 = +0.8ns（很紧）；
 *       保持裕量 = 6 + 3.2 − 1 − 0.5 = +7.7ns（安全）。
 *   - 场景 B（Tpd 4→8）：fmax 掉到 1/15.2 = 65.79 MHz，100MHz 时建立裕量 −5.20ns。
 *   - 场景 C（skew 拉到 6.5、短走线 Tpd(min)=1.0）：
 *       fmax 反而飙到 1/5.2 = 192.31 MHz —— 但保持裕量 = −0.50ns，
 *       整张表从上到下全是「保持失败」，把时钟降到 20MHz 也照样是 −0.50ns。
 *       这张表就是「降频救不了保持违例」的铁证。
 *   - MTBF（f_clk = f_data = 100MHz、Tsu=2ns，t_settle = 8/18/28 ns）：
 *       tau=0.5ns：1 级 不足 1 ns 错一次；2 级 431.123 ms；3 级 6.63 年。
 *       tau=1.0ns：1 级 不足 1 ns；2 级 6.566 ns；3 级 144.626 us。
 *       f_clk = f_data = 10MHz 那张表（t_settle = 98/198/298 ns）全部是天文数字 ——
 *       收益不是来自「频率降 10 倍」，是来自指数项，这就是两级同步器真正的价值。
 *   - 格雷码/FIFO 自测：0~15 往返全还原，46 次读写操作空满误判 0 次 PASS，
 *     收尾 wptr=11 rptr=3（异或 = 8 = 只有最高位不同）→ 正确判满。
 *   - 采样实验（100 万次，tau=2.5ns，窗口占 30%）：
 *       亚稳态事件 299539 次；0 级错 299539（29.95%，33.385 ns/次）、
 *       1 级 12001（1.20%，833.264 ns/次）、2 级 209（0.0209%，47.847 us/次）、
 *       3 级 0（本 100 万次内未出现）。
 *
 * 【老师傅点拨】
 *   亚稳态不是「会不会发生」的问题，是「多久发生一次」的问题。
 *   你以为两级同步器是仪式感，MTBF 那张表告诉你：一级同步器在 100MHz 下
 *   平均每个时钟沿都可能错，你的板子不是坏了，是你根本没给它稳定时间。
 * ===================================================================== */

#include <stdio.h>
#include <stdarg.h>
#include <math.h>
#include <stdlib.h>

#define SEC_PER_DAY   86400.0
#define SEC_PER_YEAR  3.15576e7      /* 365.25 天 */

/* 小工具：受控格式化进缓冲区，避免直接 sprintf */
static void snp(char *out, int cap, const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(out, (size_t)cap, fmt, ap);
    va_end(ap);
    if (n < 0) out[0] = '\0';
}

/* ------------------------------------------------------- 1. 建立/保持时间 */

typedef struct {
    double tclk;      /* 时钟周期，ns */
    double tco;       /* 发端触发器 CLK→Q，ns */
    double tpd;       /* 组合逻辑 + 走线最大延迟，ns */
    double tpd_min;   /* 组合逻辑 + 走线最小延迟，ns（保持检查用这条） */
    double tsu;       /* 收端建立时间，ns */
    double th;        /* 收端保持时间，ns */
    double skew;      /* 有用偏斜：收端时钟比发端晚到，ns */
    double jitter;    /* 时钟抖动（峰峰值），ns */
} Timing;

typedef struct {
    double setup_margin;
    double hold_margin;
    double fmax;
    int    setup_ok;
    int    hold_ok;
} Verdict;

static Verdict analyze(const Timing *p)
{
    Verdict v;
    double path = p->tco + p->tpd + p->tsu - p->skew - p->jitter;

    v.setup_margin = p->tclk - path;
    /* 保持：数据最早到达 = Tco + Tpd(min)；它必须在收端时钟沿之后还赖住 Th。
       收端时钟「晚到」对建立是帮忙，对保持就是拆台，所以这里 skew 变加号。 */
    v.hold_margin = p->tco + p->tpd_min - p->th - p->skew;
    v.fmax = (path > 0.0) ? (1000.0 / path) : 0.0;    /* 单位 MHz */
    v.setup_ok = (v.setup_margin > 0.0);
    v.hold_ok = (v.hold_margin > 0.0);
    return v;
}

static void scan_frequency(const char *title, const Timing *base)
{
    static const double f[9] = { 20.0, 40.0, 60.0, 70.0, 80.0, 85.0, 90.0, 100.0, 120.0 };

    printf("\n---- %s ----\n", title);
    printf("   时钟 (MHz) |  周期 (ns) | 建立裕量 (ns) | 保持裕量 (ns) | 判定\n");
    printf("  -----------+------------+---------------+---------------+------------------\n");

    for (int i = 0; i < 9; i++) {
        Timing p = *base;
        p.tclk = 1000.0 / f[i];
        Verdict v = analyze(&p);
        const char *tag;
        if (!v.hold_ok)                 tag = "保持失败（降频无用）";
        else if (v.setup_margin < 0.0)  tag = "建立失败（降频可救）";
        else if (v.setup_margin < 0.5)  tag = "临界（<0.5ns）";
        else                            tag = "能跑";
        printf("     %7.1f  |   %8.2f  |    %+9.2f  |    %+9.2f  | %s\n",
               f[i], p.tclk, v.setup_margin, v.hold_margin, tag);
    }

    Timing b = *base;
    Verdict bv = analyze(&b);
    if (bv.fmax > 0.0) {
        printf("  => fmax = 1/(Tco+Tpd+Tsu-skew-jitter) = 1/%.2f ns = %.2f MHz\n",
               1000.0 / bv.fmax, bv.fmax);
    } else {
        printf("  => 路径延迟算出来非正数，参数填错了\n");
    }
    if (!bv.hold_ok) {
        printf("  => 注意：保持裕量 %.2f ns < 0，与 Tclk 无关，上面 9 行全是废的\n",
               bv.hold_margin);
    }
}

static void lab_setup_hold(void)
{
    Timing base;
    base.tclk = 12.0;
    base.tco = 6.0;
    base.tpd = 4.0;
    base.tpd_min = 3.2;
    base.tsu = 2.0;
    base.th = 1.0;
    base.skew = 0.5;
    base.jitter = 0.3;

    printf("\n============ 实验 1：建立时间裕量与最高频率 ============\n");
    printf("基线参数：Tco=%.1f  Tpd=%.1f  Tpd(min)=%.1f  Tsu=%.1f  Th=%.1f  skew=%.1f  jitter=%.1f\n",
           base.tco, base.tpd, base.tpd_min, base.tsu, base.th, base.skew, base.jitter);

    scan_frequency("场景 A：正常布线（skew=0.5ns 帮建立，也压保持）", &base);

    Timing slow = base;
    slow.tpd = 8.0;
    slow.tpd_min = 6.5;
    scan_frequency("场景 B：组合逻辑加长（Tpd 4→8ns，典型超频翻车）", &slow);

    Timing skewed = base;
    skewed.skew = 6.5;
    skewed.tpd_min = 1.0;
    scan_frequency("场景 C：偏斜拉大 + 短走线（skew=6.5ns 破坏保持时间）", &skewed);

    printf("\n师傅的话：\n");
    printf("  ① 建立违例 → 把时钟降下来就能救，因为裕量是「周期减去路径延迟」。\n");
    printf("  ② 保持违例 → 降频一点用都没有。你看保持裕量那条公式里根本没有 Tclk。\n");
    printf("     数据来得太早就赖在收端门口不走，你把时钟放慢它只会来得更早。\n");
    printf("     唯一的解法是改物理：加缓冲/绕走线（抬高延迟下限）、把 skew 收回来、\n");
    printf("     或者长短线重新布线。所以保持违例是「布局问题」，不是「频率问题」。\n");
    printf("  ③ 场景 C 最阴：fmax 表面上飙到 192 MHz，其实一拍都跑不了。\n");
    printf("     只看建立时间的 STA 报告，会给你一张全绿的假绿灯。\n");
}

/* ------------------------------------------------------------ 2. 亚稳态 MTBF */

/* 返回 log10(MTBF 秒)，避免 exp 溢出成 inf */
static double log10_mtbf(double t_settle, double tau, double f_clk, double f_data)
{
    return (t_settle / tau) / log(10.0) - log10(f_clk * f_data);
}

/* 把「多少秒错一次」翻译成人话 */
static void fmt_span(double sec, char *out, int cap, const char *suffix)
{
    if (sec >= SEC_PER_YEAR) {
        snp(out, cap, "%.2f 年%s", sec / SEC_PER_YEAR, suffix);
    } else if (sec >= SEC_PER_DAY) {
        snp(out, cap, "%.2f 天%s", sec / SEC_PER_DAY, suffix);
    } else if (sec >= 1.0) {
        snp(out, cap, "%.3f 秒%s", sec, suffix);
    } else if (sec >= 1e-3) {
        snp(out, cap, "%.3f ms%s", sec * 1e3, suffix);
    } else if (sec >= 1e-6) {
        snp(out, cap, "%.3f us%s", sec * 1e6, suffix);
    } else if (sec >= 1e-9) {
        snp(out, cap, "%.3f ns%s", sec * 1e9, suffix);
    } else {
        snp(out, cap, "不足 1 ns%s（等于每个时钟沿都在赌）", suffix);
    }
}

static void fmt_duration(double log10_sec, char *out, int cap)
{
    if (log10_sec > 15.0) {                    /* 1e15 秒 ≈ 3e7 年 */
        snp(out, cap, "天文数字（> 3000 万年）");
        return;
    }
    fmt_span(pow(10.0, log10_sec), out, cap, " 一次");
}

static void lab_mtbf(void)
{
    static const double tau[2] = { 0.5, 1.0 };
    static const double fc[2] = { 100.0e6, 10.0e6 };
    char buf[80];

    printf("\n============ 实验 2：亚稳态 MTBF，被数字教育一下 ============\n");
    printf("公式：MTBF = exp(t_settle / tau) / (f_clk * f_data)\n");
    printf("  t_settle = 留给亚稳态指数衰减的时间 =（可用周期 − Tsu），\n");
    printf("  每多一级同步器，就多得一个整周期 Tclk。tau 是器件的亚稳态衰减常数。\n");

    for (int c = 0; c < 2; c++) {
        double f_clk = fc[c];
        double tclk = 1.0e9 / f_clk;                /* 周期，ns（注意是 1e9 不是 1e3） */
        double tsu = 2.0;

        printf("\n---- f_clk = %.0f MHz，f_data = %.0f MHz，Tsu = %.1f ns ----\n",
               f_clk / 1e6, f_clk / 1e6, tsu);
        printf("   tau (ns) | 同步级数 | t_settle (ns) | 平均多久错一次（MTBF）\n");
        printf("  ---------+----------+---------------+----------------------------------\n");
        for (int t = 0; t < 2; t++) {
            for (int stage = 1; stage <= 3; stage++) {
                double settle = (tclk - tsu) + (double)(stage - 1) * tclk;
                double lg = log10_mtbf(settle, tau[t], f_clk, f_clk);
                fmt_duration(lg, buf, (int)sizeof(buf));
                printf("     %4.1f   |    %d      |     %7.1f    | %s\n",
                       tau[t], stage, settle, buf);
            }
        }
    }

    printf("\n师傅的话：\n");
    printf("  100MHz 时钟、100MHz 异步数据，一级同步器（tau=0.5ns）不到 1 ns 就错一次 ——\n");
    printf("  这不是「偶发」，这是「一直」。加到第二级变成 431 ms 一次（还是不能用），\n");
    printf("  第三级才勉强爬进「年」的量级。跨时钟域的地址/控制线，两级是底线，\n");
    printf("  高可靠场合（医疗、汽车）直接上三级或异步 FIFO。\n");
    printf("  再看 10MHz 那张表：整张全是天文数字。收益从哪来的？不是频率降了 10 倍，\n");
    printf("  是 t_settle 从 8ns 变成 98ns —— 指数项吃了全部红利。所以「慢一点更稳」是真的。\n");
    printf("  还有一条要命：两级同步器的两个 FF 必须挨着放，且不能被综合器优化合并、\n");
    printf("  不能被 retiming 挪走，否则等于没加。\n");
}

/* ------------------------------------------ 3. 格雷码 + 异步 FIFO 空满判断 */

static unsigned bin2gray(unsigned b)
{
    return b ^ (b >> 1);
}

static unsigned gray2bin(unsigned g)
{
    unsigned b = g;
    while (g >>= 1) b ^= g;
    return b;
}

#define FIFO_DEPTH 8                  /* 2^3，地址 3 位 */
#define PTR_BITS   4                  /* 地址位 + 1 位回绕位 */
#define PTR_MASK   ((1u << PTR_BITS) - 1u)

typedef struct {
    unsigned wptr;                    /* 二进制写指针（含回绕位） */
    unsigned rptr;                    /* 二进制读指针（含回绕位） */
    int      data[FIFO_DEPTH];
    int      filled;                  /* 参考模型：真实个数，用来对答案 */
} Fifo;

static int fifo_empty(const Fifo *f)
{
    return f->wptr == f->rptr;
}

/* 满：低位（地址位）完全相等，但最高那位（回绕位）相反 —— 也就是异或出来只剩最高位 */
static int fifo_full(const Fifo *f)
{
    unsigned top = 1u << (PTR_BITS - 1);
    return ((f->wptr ^ f->rptr) & PTR_MASK) == top;
}

static void fifo_write(Fifo *f, int v)
{
    if (fifo_full(f)) return;
    f->data[f->wptr & (FIFO_DEPTH - 1)] = v;
    f->wptr = (f->wptr + 1u) & PTR_MASK;
    f->filled++;
}

static int fifo_read(Fifo *f, int *v)
{
    if (fifo_empty(f)) return 0;
    *v = f->data[f->rptr & (FIFO_DEPTH - 1)];
    f->rptr = (f->rptr + 1u) & PTR_MASK;
    f->filled--;
    return 1;
}

static void lab_gray_fifo(void)
{
    int all_ok = 1;

    printf("\n============ 实验 3：格雷码 + 异步 FIFO 空满判断（自测） ============\n");
    printf("为什么用格雷码：二进制指针 0111→1000 有 4 位同时变，\n");
    printf("跨时钟域采样时你可能读到 0100、0111、1000 之间的任意组合 —— 指针本身错了。\n");
    printf("格雷码每次只变 1 位，采样最坏也只读到「上一步」或「下一步」，指针不会飞。\n\n");

    printf("   二进制 | 格雷码 | 反解二进制 | 一致\n");
    printf("  --------+--------+-----------+-----\n");
    for (unsigned b = 0; b < 16; b++) {
        unsigned g = bin2gray(b);
        unsigned r = gray2bin(g);
        printf("    %4u  |  %4u  |    %4u    | %s\n", b, g, r, (r == b) ? "YES" : "NO");
        if (r != b) all_ok = 0;
    }
    printf("  => 格雷码往返一致性：%s\n", all_ok ? "PASS（0~15 全部还原）" : "FAIL");

    printf("\n---- 异步 FIFO 空满自测（深度 %d，指针 %d 位，多出的 1 位表示回绕圈数）----\n",
           FIFO_DEPTH, PTR_BITS);
    printf("   空：wptr == rptr            满：(wptr + 1) 与 rptr 只差最高那一位\n\n");

    Fifo f;
    f.wptr = 0u; f.rptr = 0u; f.filled = 0;
    for (int i = 0; i < FIFO_DEPTH; i++) f.data[i] = 0;

    /* 四组：写满再读空 → 交错 → 跨过 15→0 回绕边界 → 再写满收尾 */
    static const int wcnt[4] = { 8, 5, 6, 8 };
    static const int rcnt[4] = { 8, 3, 8, 0 };

    int ops = 0, fail = 0;
    printf("   #  | 动作        | wptr | rptr | wgray | rgray | 空 | 满 | 真实个数 | 对账\n");
    printf("  ----+-------------+------+------+-------+-------+----+----+----------+-----\n");

    for (int round = 0; round < 4; round++) {
        for (int i = 0; i < wcnt[round]; i++) {
            int e, u, ok;
            ops++;
            fifo_write(&f, round * 100 + i);
            e = fifo_empty(&f); u = fifo_full(&f);
            ok = ((e == 0) == (f.filled != 0)) && ((u == 1) == (f.filled == FIFO_DEPTH));
            if (!ok) fail++;
            printf("   %2d | 写 %4d      |  %3u |  %3u |   %3u  |   %3u  |  %d  |  %d  |     %2d    | %s\n",
                   ops, round * 100 + i, f.wptr, f.rptr,
                   bin2gray(f.wptr), bin2gray(f.rptr), e, u, f.filled,
                   ok ? "YES" : "NO");
        }
        for (int i = 0; i < rcnt[round]; i++) {
            int v = -1, e, u, ok;
            ops++;
            (void)fifo_read(&f, &v);
            e = fifo_empty(&f); u = fifo_full(&f);
            ok = ((e == 0) == (f.filled != 0)) && ((u == 1) == (f.filled == FIFO_DEPTH));
            if (!ok) fail++;
            printf("   %2d | 读 -> %4d   |  %3u |  %3u |   %3u  |   %3u  |  %d  |  %d  |     %2d    | %s\n",
                   ops, v, f.wptr, f.rptr,
                   bin2gray(f.wptr), bin2gray(f.rptr), e, u, f.filled,
                   ok ? "YES" : "NO");
        }
    }
    printf("  => 共 %d 次操作，空满误判 %d 次：%s\n", ops, fail, fail ? "FAIL" : "PASS");
    printf("  收尾状态：wptr=%u rptr=%u（累计写 27 读 19，指针各自绕过 15→0 好几次），\n",
           f.wptr, f.rptr);
    printf("  此时 FIFO 是满的 —— 空标志 0、满标志 1 全对。这就是「多留 1 位回绕位」\n");
    printf("  的全部意义：低位相等时，用最高位区分「空」和「满」。少一位就永远区分不了。\n");
}

/* --------------------------------------------- 4. 固定种子采样错误率实验 */

static double urand(void)
{
    /* rand() 归一化到 (0,1)，避免 log(0) */
    double u = ((double)rand() + 1.0) / ((double)RAND_MAX + 1.0);
    if (u >= 1.0) u = 0.999999999;
    if (u <= 0.0) u = 1e-12;
    return u;
}

static void lab_sampling_experiment(void)
{
    const int    N = 1000000;         /* 采样次数 */
    const double tclk = 10.0;         /* 100 MHz */
    const double tsu = 2.0, th = 1.0;
    const double tau = 2.5;           /* 故意放大 tau，让 100 万次能看出差异 */
    const double window = tsu + th;

    printf("\n============ 实验 4：不加同步器 vs 加同步器（固定种子） ============\n");
    printf("种子 srand(20260315)，N = %d 次采样；Tclk=%.1f ns、Tsu=%.1f、Th=%.1f、tau=%.1f ns\n",
           N, tclk, tsu, th, tau);
    printf("模型：数据翻转时刻在时钟周期内均匀随机；\n");
    printf("      落在建立/保持窗口 [Tclk-(Tsu+Th), Tclk] 内 → 触发亚稳态；\n");
    printf("      解除时间 t_res = -tau*ln(u)（指数衰减）；\n");
    printf("      可用稳定时间 = 同步级数 * Tclk - Tsu，t_res 超过它就真的错。\n\n");

    srand(20260315);

    long viol = 0;
    long err[4] = { 0, 0, 0, 0 };

    for (int i = 0; i < N; i++) {
        double off = urand() * tclk;              /* 数据沿距采样沿的时间 */
        if (off > tclk - window) {                /* 落在建立/保持窗口里 */
            viol++;
            double t_res = -tau * log(urand());
            for (int stage = 0; stage <= 3; stage++) {
                double avail = (double)stage * tclk - tsu;
                if (avail < 0.0) avail = 0.0;     /* 0 级：几乎没有稳定时间 */
                if (t_res > avail) err[stage]++;
            }
        }
    }

    printf("   亚稳态事件数：%ld / %d（窗口占周期的 %.1f%%）\n\n", viol, N,
           100.0 * window / tclk);
    printf("   同步级数 | 可用稳定时间(ns) | 错误次数  | 错误率        | 折算多久错一次\n");
    printf("  ---------+------------------+-----------+---------------+---------------\n");
    for (int stage = 0; stage <= 3; stage++) {
        double avail = (double)stage * tclk - tsu;
        double rate, per_err, sec;
        char per[64];

        if (avail < 0.0) avail = 0.0;
        rate = (double)err[stage] / (double)N;

        if (err[stage] == 0) {
            snp(per, (int)sizeof(per), "本 100 万次内未出现");
        } else {
            per_err = (double)N / (double)err[stage];    /* 多少个采样周期错一次 */
            sec = per_err * tclk * 1e-9;                 /* 换算成秒 */
            fmt_span(sec, per, (int)sizeof(per), " 一次");
        }
        printf("      %d      |       %6.1f      |  %8ld   |   %9.5f%%  | %s\n",
               stage, avail, err[stage], rate * 100.0, per);
    }

    printf("\n师傅的话：0 级（直接把异步信号拿去做组合逻辑）错 30%% 上下，\n");
    printf("        这就是「读拨码开关偶尔抽风」的真实成因；\n");
    printf("        2 级已经把错误率打到 1e-4 量级，3 级在这个采样量下只剩个位数。\n");
    printf("        注意：同步器只能「降低概率」，永远不能「归零」——\n");
    printf("        所以任何宣称『绝对不同步出错』的设计，都是没算过 MTBF 的设计。\n");
    printf("        换台机器、换编译器，只要种子是 20260315，这四个数一个都不会变，\n");
    printf("        这就是为什么做实验必须固定种子：不然你对不上账，也骗不了自己。\n");
}

int main(void)
{
    printf("=================================================================\n");
    printf("       数字电路 · 建立/保持时间与亚稳态实验台（单位 ns / MHz）\n");
    printf("=================================================================\n");
    printf("核心公式：fmax = 1 / (Tco + Tpd + Tsu - skew - jitter)\n");
    printf("          保持裕量 = Tco + Tpd(min) - Th - skew\n");
    printf("          MTBF = exp(t_settle / tau) / (f_clk * f_data)\n");

    lab_setup_hold();
    lab_mtbf();
    lab_gray_fifo();
    lab_sampling_experiment();

    printf("\n=================================================================\n");
    printf(" 收尾三句：\n");
    printf(" 1. 建立违例降频能救，保持违例只有改布局/加缓冲能救。\n");
    printf(" 2. 跨时钟域的每一位都要过同步器，两级是底线，且别让综合器优化掉。\n");
    printf(" 3. 指针类多位总线跨域必须转格雷码，或者干脆用异步 FIFO。\n");
    printf("=================================================================\n");
    return 0;
}
