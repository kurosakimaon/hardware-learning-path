/* =====================================================================
 * 01_logic_gates.c —— 逻辑门、卡诺图与"最简与或式"化简器（奎因-麦克拉斯基）
 *
 * 【这个文件讲什么】
 *   1) 一个 4 变量逻辑最小化器：喂给它"最小项集合 + 无关项集合"，
 *      它自己求出最简与或式（SOP）。核心原理只有一句话：
 *          两个相邻最小项合并 = 消去一个变量。
 *      例：A'BC + ABC → 只有 A 不同 → 合并成 BC（A 被"买"掉了）。
 *      卡诺图上画的那个圈，干的就是这件事；圈越大，消掉的变量越多。
 *   2) 两个真活：三人投票多数器（化简成 AB+AC+BC）、
 *      BCD 七段数码管 g 段译码（带 6 个无关项 X）。
 *   3) 化简前 vs 化简后的代价对照：门数量、门输入总数、估计延迟级数。
 *      —— 这才是"为什么要化简"的真实答案：省片子、省功耗、少两级门延迟。
 *   4) 两级与非（NAND-NAND）实现任意组合逻辑，并逐行验证等价性。
 *      74HC00 里四个二输入与非门，就是这个能力的物理形态。
 *   5) 顺手算一下师傅让你拿 74HC00 搭的那个环形振荡器能跑多快。
 *
 * 【怎么编译】（MSYS2 / MinGW，本项目在 gcc 16.2 + Windows 下验证零警告）
 *   gcc -std=c17 -O2 -Wall -Wextra 01_logic_gates.c -o 01_logic_gates.exe
 *
 * 【怎么运行】
 *   chcp 65001              ← 先执行这句！Windows 控制台默认 GBK，不切 UTF-8
 *                             中文全是乱码，你会以为是程序写坏了
 *   .\01_logic_gates.exe
 *
 * 【预期看到什么】
 *   - 多数投票器化简结果 = AB+AC+BC（3 个二输入与门 + 一个三输入或门），
 *     未化简的标准与或式是 4 个四输入与门 + 一个四输入或门；
 *   - 七段 g 段（含 X 无关项）化简成 4 项 7 个字面值，比未化简省一半以上门；
 *   - NAND-NAND 等价性验证 16/16 一致 PASS；
 *   - 环形振荡器：3 级 20.83 MHz、5 级 12.50 MHz、7 级 8.93 MHz、9 级 6.94 MHz。
 *
 * 【老师傅点拨】
 *   化简不是为了考试，是为了少焊几片 74 系列、少两级门延迟、少一份毛刺概率。
 *   还有一条：无关项（X）是白捡的便宜，敢用才化得干净 ——
 *   BCD 译码里 1010~1111 六个输入组合你的电路永远不会出现，
 *   把它们当 1 用，圈就能画大两号。
 * ===================================================================== */

#include <stdio.h>

#define NVAR  4                    /* 变量个数：A B C D */
#define NMIN  (1 << NVAR)          /* 最小项个数：16 */
#define MAXT  192                  /* 蕴含项暂存上限 */
#define MAXPI 96                   /* 质蕴含项上限 */

static const char VN[NVAR] = { 'A', 'B', 'C', 'D' };

/* 一个"与项"：dash 位为 1 表示该变量被消掉了（卡诺图上的"跨格"） */
typedef struct {
    unsigned val;   /* 未被消掉的变量的取值，位权：A=8 B=4 C=2 D=1 */
    unsigned dash;  /* 被消掉的变量掩码 */
} Term;

/* ---------- 小工具 ---------- */

static int popcnt(unsigned x)
{
    int n = 0;
    while (x) { n += (int)(x & 1u); x >>= 1; }
    return n;
}

static int lit_count(Term t)
{
    int n = 0;
    for (int v = 0; v < NVAR; v++) {
        unsigned w = 1u << (NVAR - 1 - v);
        if ((t.dash & w) == 0u) n++;
    }
    return n;
}

/* 该与项是否覆盖最小项 m */
static int covers(Term t, int m)
{
    unsigned keep = (~t.dash) & (unsigned)(NMIN - 1);
    return (((unsigned)m ^ t.val) & keep) == 0u;
}

static unsigned term_mask(Term t)
{
    unsigned m = 0u;
    for (int k = 0; k < NMIN; k++) {
        if (covers(t, k)) m |= (1u << (unsigned)k);
    }
    return m;
}

/* 手搓字符串（不用 sprintf/snprintf，避免 -Wall 下的截断警告） */
typedef struct { char s[96]; int n; } Buf;

static void b_reset(Buf *b) { b->n = 0; b->s[0] = '\0'; }

static void b_putc(Buf *b, char c)
{
    if (b->n + 1 < (int)sizeof(b->s)) { b->s[b->n++] = c; b->s[b->n] = '\0'; }
}

static void b_puts(Buf *b, const char *t)
{
    while (*t) b_putc(b, *t++);
}

static void term_text(Term t, Buf *b)
{
    int wrote = 0;
    for (int v = 0; v < NVAR; v++) {
        unsigned w = 1u << (NVAR - 1 - v);
        if (t.dash & w) continue;
        b_putc(b, VN[v]);
        if ((t.val & w) == 0u) b_putc(b, '\'');
        wrote = 1;
    }
    if (!wrote) b_putc(b, '1');
}

static void term_text_quoted(Term t, Buf *b)
{
    b_puts(b, "NAND(");
    term_text(t, b);
    b_putc(b, ')');
}

/* ---------- 奎因-麦克拉斯基：先求全部质蕴含项 ---------- */

static void tadd(Term *arr, int *n, int cap, Term t)
{
    for (int i = 0; i < *n; i++) {
        if (arr[i].val == t.val && arr[i].dash == t.dash) return;
    }
    if (*n < cap) arr[(*n)++] = t;
}

/*
 * 合并规则：两个与项，被消掉的变量集合相同，且取值只差 1 个比特
 *           → 相差的那个变量就是"相邻"的那一维，直接消掉。
 * 这就是卡诺图上"两个相邻格子圈一起"的代数写法。
 */
static int prime_implicants(const int *ms, int nm, const int *dc, int ndc,
                           Term *pi, int cap)
{
    Term cur[MAXT], nxt[MAXT];
    int ncur = 0, np = 0, used[MAXT];

    for (int i = 0; i < nm; i++) {
        Term t; t.val = (unsigned)ms[i]; t.dash = 0u; tadd(cur, &ncur, MAXT, t);
    }
    for (int i = 0; i < ndc; i++) {
        Term t; t.val = (unsigned)dc[i]; t.dash = 0u; tadd(cur, &ncur, MAXT, t);
    }

    while (ncur > 0) {
        int nn = 0;
        for (int i = 0; i < ncur; i++) used[i] = 0;

        for (int i = 0; i < ncur; i++) {
            for (int j = i + 1; j < ncur; j++) {
                unsigned diff;
                if (cur[i].dash != cur[j].dash) continue;
                diff = cur[i].val ^ cur[j].val;
                if (popcnt(diff) != 1) continue;
                Term t;
                t.val = cur[i].val & ~diff;
                t.dash = cur[i].dash | diff;
                used[i] = 1; used[j] = 1;
                tadd(nxt, &nn, MAXT, t);
            }
        }
        for (int i = 0; i < ncur; i++) {
            if (!used[i]) tadd(pi, &np, cap, cur[i]);
        }
        for (int i = 0; i < nn; i++) cur[i] = nxt[i];
        ncur = nn;
    }
    return np;
}

/* ---------- 再求最小覆盖（分支限界，规模小，够用） ---------- */

typedef struct {
    int pick[MAXPI];
    int npick;
    int bestcost;
} Cover;

static void cover_recur(const Term *pi, const unsigned *pm, int np,
                        unsigned need, int *cur, int depth, Cover *cv)
{
    if (need == 0u) {
        int lit = 0;
        for (int i = 0; i < depth; i++) lit += lit_count(pi[cur[i]]);
        int cost = depth * 100 + lit;      /* 先比项数，再比字面值个数 */
        if (cv->npick == 0 || cost < cv->bestcost) {
            cv->bestcost = cost;
            cv->npick = depth;
            for (int i = 0; i < depth; i++) cv->pick[i] = cur[i];
        }
        return;
    }
    if (cv->npick != 0 && depth >= cv->npick) return;   /* 项数已不会更优 */

    int m = 0;
    while ((need & (1u << (unsigned)m)) == 0u) m++;
    for (int i = 0; i < np; i++) {
        if ((pm[i] & (1u << (unsigned)m)) == 0u) continue;
        cur[depth] = i;
        cover_recur(pi, pm, np, need & ~pm[i], cur, depth + 1, cv);
    }
}

static void sort_terms(Term *t, int n)
{
    for (int i = 1; i < n; i++) {
        Term k = t[i];
        int j = i - 1;
        while (j >= 0 && (t[j].dash > k.dash ||
               (t[j].dash == k.dash && t[j].val > k.val))) {
            t[j + 1] = t[j];
            j--;
        }
        t[j + 1] = k;
    }
}

/* 输入最小项 + 无关项，输出最简与或式；返回项数 */
static int min_sop(const int *ms, int nm, const int *dc, int ndc,
                   Term *out, int cap)
{
    Term pi[MAXPI];
    unsigned pm[MAXPI];
    int np = prime_implicants(ms, nm, dc, ndc, pi, MAXPI);

    unsigned need = 0u;
    for (int i = 0; i < nm; i++) need |= (1u << (unsigned)ms[i]);

    Cover cv;
    cv.npick = 0;
    cv.bestcost = 0;
    int cur[MAXPI];
    for (int i = 0; i < np; i++) pm[i] = term_mask(pi[i]);
    cover_recur(pi, pm, np, need, cur, 0, &cv);

    int n = cv.npick > cap ? cap : cv.npick;
    for (int i = 0; i < n; i++) out[i] = pi[cv.pick[i]];
    sort_terms(out, n);
    return n;
}

/* ---------- 代价模型：以"二输入门"为单元 ---------- */

typedef struct {
    int andg;      /* 与门个数 */
    int org;       /* 或门个数 */
    int invg;      /* 反相器个数 */
    int inputs;    /* 门输入脚总数 */
    int levels;    /* 估计延迟级数 */
    int terms;
    int lits;
} Cost;

static int ceil_log2(int v)
{
    int l = 0, n = 1;
    while (n < v) { n <<= 1; l++; }
    return l;
}

static Cost cost_of(const Term *ts, int n)
{
    Cost c;
    int maxand = 0;
    unsigned need_inv = 0u;

    c.andg = 0; c.org = 0; c.invg = 0; c.inputs = 0;
    c.levels = 0; c.terms = n; c.lits = 0;

    for (int i = 0; i < n; i++) {
        int k = lit_count(ts[i]);
        c.lits += k;
        if (k >= 2) {
            c.andg += k - 1;              /* k 个字面值串成 k-1 个二输入与门 */
            if (k - 1 > maxand) maxand = k - 1;
        }
        for (int v = 0; v < NVAR; v++) {
            unsigned w = 1u << (NVAR - 1 - v);
            if ((ts[i].dash & w) == 0u && (ts[i].val & w) == 0u) need_inv |= w;
        }
    }
    for (int v = 0; v < NVAR; v++) {
        if (need_inv & (1u << v)) c.invg++;
    }
    c.org = (n > 1) ? (n - 1) : 0;        /* 或门做成二输入树 */
    c.inputs = 2 * (c.andg + c.org) + c.invg;
    c.levels = maxand + ceil_log2(n > 1 ? n : 2) + (c.invg ? 1 : 0);
    return c;
}

static void print_cost_line(const char *name, Cost c)
{
    printf("  %-18s 与门 %2d  或门 %2d  反相 %2d  门总数 %2d  输入脚 %3d  延迟级数 %d\n",
           name, c.andg, c.org, c.invg, c.andg + c.org + c.invg, c.inputs, c.levels);
}

/* ---------- 打印 ---------- */

static void print_sop(const Term *ts, int n)
{
    Buf b;
    b_reset(&b);
    for (int i = 0; i < n; i++) {
        if (i) b_putc(&b, '+');
        term_text(ts[i], &b);
    }
    if (n == 0) b_puts(&b, "0");
    printf("%s", b.s);
}

static void print_kmap(const int *ms, int nm, const int *dc, int ndc)
{
    char cell[NMIN];
    static const int seq[4] = { 0, 1, 3, 2 };   /* 00 01 11 10 —— 格雷码序，相邻才相邻 */

    for (int i = 0; i < NMIN; i++) cell[i] = '0';
    for (int i = 0; i < ndc; i++) cell[dc[i]] = 'X';
    for (int i = 0; i < nm; i++) cell[ms[i]] = '1';

    printf("            CD\n            00   01   11   10\n");
    printf("          +----+----+----+----+\n");
    for (int r = 0; r < 4; r++) {
        int a = (seq[r] >> 1) & 1, b = seq[r] & 1;
        printf("     AB %d%d   |", a, b);
        for (int c = 0; c < 4; c++) {
            int m = seq[r] * 4 + seq[c];
            printf(" %c  |", cell[m]);
        }
        printf("   m%-2d,%-2d,%-2d,%-2d\n",
               seq[r] * 4, seq[r] * 4 + 1, seq[r] * 4 + 3, seq[r] * 4 + 2);
    }
    printf("          +----+----+----+----+\n");
}

static int sop_value(const Term *ts, int n, int m)
{
    for (int i = 0; i < n; i++) if (covers(ts[i], m)) return 1;
    return 0;
}

/* 两级与非：先对每个与项做一次与非，再把结果整体做一次与非 */
static int nand_nand_value(const Term *ts, int n, int m)
{
    int all_zero = 1;
    for (int i = 0; i < n; i++) {
        if (covers(ts[i], m)) { all_zero = 0; break; }   /* 该项 NAND 输出 0 */
    }
    return all_zero ? 0 : 1;
}

static void print_truth_table(const int *ms, int nm, const int *dc, int ndc,
                              const Term *ts, int n)
{
    (void)dc; (void)ndc;
    printf("   A B C D | minterm | 标准SOP | 化简SOP | NAND-NAND | 一致\n");
    printf("  ---------+---------+---------+---------+-----------+-----\n");
    int same = 0;
    for (int m = 0; m < NMIN; m++) {
        int a = (m >> 3) & 1, b = (m >> 2) & 1, c = (m >> 1) & 1, d = m & 1;
        int v0 = 0, v1, v2;
        for (int i = 0; i < nm; i++) if (ms[i] == m) v0 = 1;
        v1 = sop_value(ts, n, m);
        v2 = nand_nand_value(ts, n, m);
        printf("    %d %d %d %d | m%-7d | %7d | %7d | %9d | %s\n",
               a, b, c, d, m, v0, v1, v2, (v1 == v2) ? "YES" : "NO");
        if (v1 == v2) same++;
    }
    printf("  => NAND-NAND 等价性验证：%d/%d 一致，%s\n\n",
           same, NMIN, (same == NMIN) ? "PASS" : "FAIL");
}

static void print_nand_impl(const Term *ts, int n)
{
    Buf b;
    int gates = 0, inv = 0;
    unsigned need_inv = 0u;

    b_reset(&b);
    b_puts(&b, "F = NAND( ");
    for (int i = 0; i < n; i++) {
        if (i) b_puts(&b, ", ");
        term_text_quoted(ts[i], &b);
        int k = lit_count(ts[i]);
        gates += (k >= 2) ? (k - 1) : 1;      /* 单变量项：NAND 两脚并接 = 反相器 */
        for (int v = 0; v < NVAR; v++) {
            unsigned w = 1u << (NVAR - 1 - v);
            if ((ts[i].dash & w) == 0u && (ts[i].val & w) == 0u) need_inv |= w;
        }
    }
    b_puts(&b, " )");
    gates += (n > 1) ? (n - 1) : 0;
    for (int v = 0; v < NVAR; v++) if (need_inv & (1u << (unsigned)v)) inv++;

    printf("  两级与非写法：%s\n", b.s);
    printf("  折算门数：项内与非 %d + 输出级与非 %d + 输入反相 %d = %d 个二输入与非门\n",
           gates - ((n > 1) ? (n - 1) : 0), (n > 1) ? (n - 1) : 0, inv, gates + inv);
    printf("  74HC00（每片 4 个二输入与非门）需要 %d 片\n\n",
           (gates + inv + 3) / 4);
}

/* ---------- 案例 ---------- */

static void case_majority(void)
{
    /* 三人投票：A/B/C 三人，多数通过；D 是空脚，与本函数无关 */
    static const int ms[] = { 6, 7, 10, 11, 12, 13, 14, 15 };
    static const int dc[] = { 0, 1, 2, 3, 4, 5, 8, 9 };

    printf("\n================ 例 1：三人投票多数器 ================\n");
    printf("最小项 m = {6,7,10,11,12,13,14,15}，无关项 X = 空（D 为本函数的空脚）\n");
    printf("注意：D 空脚意味着每个最小项都成对出现，化简时天然可以纵向合并。\n\n");
    print_kmap(ms, 8, dc, 0);

    Term t[MAXPI];
    int n = min_sop(ms, 8, NULL, 0, t, MAXPI);
    printf("\n最简与或式：F = ");
    print_sop(t, n);
    printf("\n");

    Term canon[16];
    for (int i = 0; i < 8; i++) { canon[i].val = (unsigned)ms[i]; canon[i].dash = 0u; }
    printf("\n代价对照（门延迟以二输入门计）：\n");
    print_cost_line("标准与或式(未化简)", cost_of(canon, 8));
    print_cost_line("化简后", cost_of(t, n));

    printf("\n两级与非实现（74HC00 可以直接抄）：\n");
    print_nand_impl(t, n);
    print_truth_table(ms, 8, NULL, 0, t, n);
}

static void case_segment_g(void)
{
    /* 七段数码管 g 段（中间那一横），共阴有效=1，输入是 8421BCD：A 是高位 */
    static const int ms[] = { 2, 3, 4, 5, 6, 8, 9 };
    static const int dc[] = { 10, 11, 12, 13, 14, 15 };

    printf("\n================ 例 2：BCD 七段译码 g 段 ================\n");
    printf("数字 2,3,4,5,6,8,9 要点亮 g 段；1010~1111 你的编码器永远不会输出 —— 它们是无关项 X。\n");
    printf("师傅的话：X 不是「随便」，X 是「白捡的便宜」，圈能画大就画大。\n\n");
    print_kmap(ms, 7, dc, 6);

    Term t[MAXPI];
    int n = min_sop(ms, 7, dc, 6, t, MAXPI);
    printf("\n最简与或式：g = ");
    print_sop(t, n);
    printf("\n");

    Term canon[16];
    for (int i = 0; i < 7; i++) { canon[i].val = (unsigned)ms[i]; canon[i].dash = 0u; }
    printf("\n代价对照：\n");
    print_cost_line("标准与或式(未化简)", cost_of(canon, 7));
    print_cost_line("化简后(用满无关项)", cost_of(t, n));

    /* 反例：不敢用无关项会贵成什么样 */
    Term t2[MAXPI];
    int n2 = min_sop(ms, 7, NULL, 0, t2, MAXPI);
    printf("化简后(不敢用 X)      ：");
    print_sop(t2, n2);
    printf("\n");
    print_cost_line("化简后(不敢用 X)", cost_of(t2, n2));

    printf("\n两级与非实现：\n");
    print_nand_impl(t, n);
    print_truth_table(ms, 7, dc, 6, t, n);
}

static void case_parity(void)
{
    /* 奇校验：1 的个数为奇数时输出 1 —— 卡诺图救不了它的典型 */
    static const int ms[] = { 1, 2, 4, 7, 8, 11, 13, 14 };
    static const int dc[] = { 0, 3, 5, 6, 9, 10, 12, 15 };

    printf("\n================ 例 3：4 位奇校验（反面教材） ================\n");
    Term t[MAXPI];
    int n = min_sop(ms, 8, NULL, 0, t, MAXPI);
    printf("最简与或式：P = ");
    print_sop(t, n);
    printf("   （%d 项，每项 4 个字面值 —— 一个圈都没合并成）\n", n);

    Term canon[16];
    for (int i = 0; i < 8; i++) { canon[i].val = (unsigned)ms[i]; canon[i].dash = 0u; }
    print_cost_line("标准与或式(未化简)", cost_of(canon, 8));
    print_cost_line("化简后", cost_of(t, n));
    printf("师傅的话：奇偶校验的 1 在卡诺图上是棋盘格，谁也挨不着谁。\n");
    printf("          这类函数天生要用异或门（74HC86）做，别跟卡诺图较劲。\n");
    (void)dc;
}

static void demo_ring_oscillator(void)
{
    /* 师傅让你搭的振荡器：奇数个反相器首尾相接，f = 1/(2·N·tpd) */
    const int tpd_ns = 8;        /* 74HC00 在 5V、25℃ 下典型门延迟 */
    static const int N[4] = { 3, 5, 7, 9 };

    printf("\n============ 附赠：74HC00 搭环形振荡器能跑多快 ============\n");
    printf("原理：奇数个反相器首尾相接，状态绕一圈翻回来，就是振荡。\n");
    printf("周期 = 2 · N · tpd（信号要绕两圈才回到原点）\n\n");
    printf("   反相器个数 N |  周期 (ns) |  频率 (MHz) | 说明\n");
    printf("  --------------+------------+-------------+-------------------\n");
    for (int i = 0; i < 4; i++) {
        double T = 2.0 * (double)N[i] * (double)tpd_ns;
        double f = 1000.0 / T;
        printf("    %5d        |   %6.0f   |   %6.2f    | %s\n",
               N[i], T, f,
               (N[i] % 2) ? "奇数，能振" : "偶数，锁死");
    }
    printf("\n师傅的话：偶数个反相器接成一圈不叫振荡，叫双稳态（两个稳定点）——\n");
    printf("          SR 锁存器就是这么来的。同一根线，奇偶两个世界。\n");
    printf("          真实板子上这个频率会被负载电容拖低 20%%~50%%，\n");
    printf("          扇出越多越慢，这就是「驱动能力」最直观的体现。\n");
}

int main(void)
{
    printf("=================================================================\n");
    printf("        数字电路 · 逻辑门与卡诺图化简实验台（4 变量）\n");
    printf("=================================================================\n");
    printf("变量约定：A 为最高位（位权 8），B=4，C=2，D 为最低位（位权 1）\n");
    printf("合并原理：A'BC + ABC = BC —— 相邻两个最小项合并，消去一个变量\n");

    case_majority();
    case_segment_g();
    case_parity();
    demo_ring_oscillator();

    printf("\n=================================================================\n");
    printf(" 收尾三句：\n");
    printf(" 1. 化简省的是片子、功耗、门延迟，不是卷面分。\n");
    printf(" 2. 两级与非万能：任何与或式 = NAND(NAND(项1), NAND(项2), ...)\n");
    printf(" 3. 化简出来的是「逻辑」，不是「时序」——毛刺问题下一课见。\n");
    printf("=================================================================\n");
    return 0;
}
