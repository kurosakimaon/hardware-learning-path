/* =====================================================================
 *  01_node_solver.c —— 自己写一个电路求解器（节点电压法 / MNA）
 *  老师傅带徒弟系列 · 第 1 关 · 第 1 课
 * =====================================================================
 *
 *  这个文件讲什么？
 *    它不"手工算"电路，它【自己列方程、自己解方程】。
 *    你只要给它一张网表（哪个元件接在哪两个节点之间、阻值多少），
 *    它就自动：
 *      ① 对每个节点列 KCL 方程；
 *      ② 把每条支路电流写成 (本节点电压 - 邻居电压) / 电阻（欧姆定律）；
 *      ③ 组装成线性方程组 G·x = i；
 *      ④ 用带部分主元选取的高斯-约当消元解出来；
 *      ⑤ 打印各节点电压、各支路电流、各电阻功耗和封装建议。
 *    全程纯 C（C17）、零第三方库、零网络、零硬件，方程求解也是手写的。
 *
 *  为什么"节点电压法"就是 KCL + 欧姆定律？（师傅逼你读懂这段）
 *    KCL 说：流入任一节点的电流之和 = 流出之和，也就是
 *            "流出该节点的所有支路电流加起来 = 0"。这一条给你【方程的个数】：
 *            有几个独立节点，就有几个方程。
 *    但方程里的未知量本来是"每条支路的电流"，支路比节点多，未知量太多。
 *    欧姆定律说：一条电阻支路上的电流 = 两端电压差 / 阻值。
 *    把每条电流都换成"节点电压之差 × 电导"，未知量就【只剩节点电压】了，
 *    而且方程立刻变成【线性】的——这就是节点电压法的全部秘密。
 *    碰到电压源怎么办？电压源的电流不能由欧姆定律表达（R = 0，除不了），
 *    所以多拉一个未知量出来：把这个源的电流当未知数，
 *    再补一条方程"这个源两端电压差 = 标称值"。
 *    这就是 MNA（改进节点法，Modified Nodal Analysis）。
 *    你以后用的 SPICE、LTspice、Multisim，内核就是它，没有别的魔法。
 *
 *  怎么编译（PowerShell，在本目录下）：
 *      gcc -std=c17 -O2 -Wall -Wextra 01_node_solver.c -o 01_node_solver.exe
 *  怎么运行：
 *      chcp 65001          （先把控制台切到 UTF-8，否则中文会乱码）
 *      .\01_node_solver.exe
 *
 *  预期看到什么？（四组，逐条对着 tutorial.md 第四节看）
 *    ① LED 限流：5V + 330Ω + LED(恒压降 2.1V) → 8.788mA、电阻 25.49mW；
 *       换 100Ω → 29mA、电阻 84.1mW（0402 只有 62.5mW 额定，已经在烤）；
 *       最后附一张 LED 的指数 I-V 表：1.9V→0.18mA，2.1V→8.7mA，2.3V→418mA。
 *    ② 分压带负载（本关最值钱的一张表）：3.3V、10k/10k，空载 1.650V；
 *       挂 1M 掉 0.5%、挂 100k 掉 4.8%、挂 10k 掉 33.3%、挂 1k 掉 83.3%。
 *       戴维南内阻 Rth = 5.000kΩ —— 记住这个数，第二关运放还要用它。
 *    ③ 惠斯通电桥：四臂都 10k 时检测支路电流 = 0（打印出 0.00e+00 A，
 *       也就是说矩阵真的解出了"零"）；
 *       一个臂变 10.1k（模拟应变片形变 1%），高阻检测下差分输出 -12.44mV；
 *       用 1k 的"检流计"去读只剩 -1.13mV —— 仪表放大器存在的理由。
 *    ④ 电源内阻：2Ω 内阻挂 332Ω 负载（15mA）只掉 30mV；
 *       同样 2Ω 挂 5Ω 负载（约 714mA，等价 WiFi 发射瞬间）直接塌到 3.571V，
 *       内阻自己烧 1.02W。最后再并一颗"100µF"：稳态数字一模一样，
 *       因为直流稳态下电容 = 开路。
 *
 *  老师傅点拨两句：
 *    第一，程序里的 LED 是"恒压降电压源"，这是最粗糙的模型。
 *          粗糙模型的用处是：让你 10 秒算出一个能用的电阻值，误差 10% 以内，
 *          工程上够用。先粗糙后精确，别一上来就牛顿迭代。
 *    第二，"空载"在这个程序里 = "网表里没有这条支路"。
 *          你在板子上想造开路，得真的把元件焊下来。所以你以为的"空载"，
 *          往往是 PCB 上那个还没焊的位置（DNP），或者一片你压根没注意的负载。
 * =====================================================================
 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* 容量上限：本文件的电路都很小，固定数组足够，不做动态分配 */
#define MAX_NODES 8   /* 独立节点数上限（不含 GND=0） */
#define MAX_RES 16    /* 电阻支路上限 */
#define MAX_SRC 8     /* 电压源上限 */
#define MAX_SZ (MAX_NODES + MAX_SRC) /* 方程组维度上限 */
#define BUFSZ 512

/* =====================================================================
 * 一、线性方程组求解：高斯-约当消元 + 部分主元选取
 *     为什么要有"部分主元"？因为如果主对角线上那个数是 1e-18，
 *     拿它去除别的数，舍入误差会被放大到没法收拾（数值爆炸）。
 *     做法很土：在当前列里挑绝对值最大的那一行，换上来当主元。
 *     这段三重循环，就是你以后做 SPICE / 电力电子 / 热网络仿真的老祖宗。
 * ===================================================================== */
static void fail(const char *msg) {
    printf("\n[出错了] %s\n", msg);
    exit(1);
}

static void solve_linear(double a[MAX_SZ][MAX_SZ], double b[MAX_SZ], size_t n) {
    for (size_t col = 0; col < n; ++col) {
        size_t pivot = col;
        double best = fabs(a[col][col]);
        for (size_t r = col + 1; r < n; ++r) {
            const double v = fabs(a[r][col]);
            if (v > best) {
                best = v;
                pivot = r;
            }
        }
        if (best < 1e-15) {
            fail("方程组奇异（解不出来）。现场常见三种原因："
                 "1) 有节点浮空，没有回路回地；"
                 "2) 两个理想电压源并联或成环；"
                 "3) 该节点没有被任何电阻拉住，只有电压源。");
        }
        if (pivot != col) {
            for (size_t c = 0; c < n; ++c) {
                const double t = a[pivot][c];
                a[pivot][c] = a[col][c];
                a[col][c] = t;
            }
            const double tb = b[pivot];
            b[pivot] = b[col];
            b[col] = tb;
        }
        const double diag = a[col][col];
        for (size_t c = col; c < n; ++c) a[col][c] /= diag;
        b[col] /= diag;
        for (size_t r = 0; r < n; ++r) {
            if (r == col) continue;
            const double factor = a[r][col];
            if (factor == 0.0) continue;
            for (size_t c = col; c < n; ++c) a[r][c] -= factor * a[col][c];
            b[r] -= factor * b[col];
        }
    }
}

/* =====================================================================
 * 二、网表（netlist）：把"一张图"翻译成"一串元件"
 *     节点编号约定：0 = 参考地 GND（永远 0V），1..nodeCount 是独立节点。
 *     注意：GND 只是【参考点】，不是"大地的绝对零电位"。
 *     你在板子上量两个 GND 焊盘之间差了几 mV，这程序算不出来——
 *     因为它把 GND 当理想 0V，而真实的铜皮有电阻（tutorial.md 坑 1）。
 *     这是第一层理想化，认清它，别把它当现实。
 * ===================================================================== */
typedef struct {
    char name[32];
    int a;
    int b;
    double ohm;
} Resistor;

typedef struct {
    char name[32];
    int plus;
    int minus;
    double volts;
} Source;

typedef struct {
    int nodeCount;                  /* 独立节点数（不含 GND=0） */
    Resistor resistors[MAX_RES];
    int nRes;
    Source sources[MAX_SRC];
    int nSrc;
} Netlist;

typedef struct {
    double nodeV[MAX_NODES + 1];    /* nodeV[k] = 节点 k 对 GND 的电压（k 从 1 起，[0] 恒为 0） */
    double resCurrent[MAX_RES];     /* 每条电阻支路：从 a 流向 b 的电流（A） */
    double resPower[MAX_RES];       /* 每条电阻支路功耗（W） */
    double srcCurrent[MAX_SRC];     /* 每个电压源：流入其 + 端的电流（A） */
                                    /*   负值 = 正在向外供电（电源） */
                                    /*   正值 = 正在吸收电流（LED、假负载） */
} Solution;

static void net_init(Netlist *net, int nodes) {
    memset(net, 0, sizeof *net);
    net->nodeCount = nodes;
}

static void net_addR(Netlist *net, const char *name, int a, int b, double ohm) {
    if (ohm <= 0.0) {
        fail("电阻必须是正值：要导线请直接把两个节点合并，要电源请用 net_addV");
    }
    if (net->nRes >= MAX_RES) fail("电阻支路超出容量上限");
    Resistor *r = &net->resistors[net->nRes++];
    snprintf(r->name, sizeof r->name, "%s", name);
    r->a = a;
    r->b = b;
    r->ohm = ohm;
}

static void net_addV(Netlist *net, const char *name, int plus, int minus, double volts) {
    if (net->nSrc >= MAX_SRC) fail("电压源数量超出容量上限");
    Source *s = &net->sources[net->nSrc++];
    snprintf(s->name, sizeof s->name, "%s", name);
    s->plus = plus;
    s->minus = minus;
    s->volts = volts;
}

/* ---------------------------------------------------------------------
 * solve：核心中的核心。看懂这个函数，你就看懂了节点电压法。
 *
 *   未知量向量 x = [ v1 .. vn | iE1 .. iEm ]
 *                  前 n 个是节点电压，后 m 个是电压源支路电流。
 *
 *   行 0..n-1 是 KCL：   Σ_k (vp - vk)·g  ±  iE  =  0
 *                        ↑ "欧姆定律代入 KCL" 就发生在这里
 *   行 n..n+m-1 是 KVL： v(plus) - v(minus) = E   ← 每个电压源补的一条方程
 *
 *   一条电导 g = 1/R 在矩阵里落四个子，正是 KCL 展开的样子：
 *     (p,p) += g    流出 p 的电流与 vp 成正比，系数 g
 *     (k,k) += g    对称地，节点 k 那边同样
 *     (p,k) -= g    流出 p 的电流里含 -vk 这一项
 *     (k,p) -= g
 *   于是矩阵天然对称、对角占优——这也是它数值稳定的根本原因。
 * --------------------------------------------------------------------- */
static void solve(const Netlist *net, Solution *sol) {
    const size_t n = (size_t)net->nodeCount;
    const size_t m = (size_t)net->nSrc;
    const size_t sz = n + m;

    double g[MAX_SZ][MAX_SZ];
    double rhs[MAX_SZ];
    memset(g, 0, sizeof g);
    memset(rhs, 0, sizeof rhs);

    /* 节点号 k（1..n）对应矩阵的行/列下标 k-1；GND(0) 不参与，因为它恒等于 0V。 */
    /* rowOf：见下面每处的 (node - 1) */

    /* ---- 电阻：只往 KCL 行里盖章（欧姆定律把电流换成电压差 × 电导）---- */
    for (int k = 0; k < net->nRes; ++k) {
        const Resistor *r = &net->resistors[k];
        const double cond = 1.0 / r->ohm;
        if (r->a > 0) g[r->a - 1][r->a - 1] += cond;
        if (r->b > 0) g[r->b - 1][r->b - 1] += cond;
        if (r->a > 0 && r->b > 0) {
            g[r->a - 1][r->b - 1] -= cond;
            g[r->b - 1][r->a - 1] -= cond;
        }
    }

    /* ---- 电压源：在 KCL 行里挂一个 ±1 的电流系数，另外自己占一行写 KVL ---- */
    for (size_t k = 0; k < m; ++k) {
        const Source *s = &net->sources[k];
        const size_t col = n + k;  /* 这个源对应的未知电流在哪一列 */
        if (s->plus > 0) {
            g[s->plus - 1][col] += 1.0;  /* KCL：这股电流从 + 端流进节点 */
            g[col][s->plus - 1] += 1.0;  /* KVL：v(+) */
        }
        if (s->minus > 0) {
            g[s->minus - 1][col] -= 1.0;
            g[col][s->minus - 1] -= 1.0;  /* KVL：- v(-) */
        }
        rhs[col] = s->volts;  /* v(+) - v(-) = E */
    }

    solve_linear(g, rhs, sz);

    memset(sol, 0, sizeof *sol);
    for (size_t i = 1; i <= n; ++i) sol->nodeV[i] = rhs[i - 1];

    for (int k = 0; k < net->nRes; ++k) {
        const Resistor *r = &net->resistors[k];
        const double va = sol->nodeV[r->a];
        const double vb = sol->nodeV[r->b];
        const double cur = (va - vb) / r->ohm;  /* 欧姆定律：支路电流就是这么来的 */
        sol->resCurrent[k] = cur;
        sol->resPower[k] = fabs(va - vb) * fabs(cur);
    }

    for (size_t k = 0; k < m; ++k) sol->srcCurrent[k] = rhs[n + k];
}

/* =====================================================================
 * 三、打印辅助
 *     把数字换算成工程师真正说的单位。这不是化妆，是防错：
 *     工程上没人说"0.008788 安"，都说"8.79 毫安"。
 *     单位一换，数量级错没错，一眼就露馅。
 * ===================================================================== */

/* 终端显示宽度：ASCII 占 1 列，UTF-8 多字节字符（中日韩）占 2 列 */
static size_t disp_width(const char *s) {
    size_t w = 0;
    for (const char *p = s; *p; ++p) {
        const unsigned char c = (unsigned char)*p;
        if (c < 0x80) {
            w += 1;
        } else if ((c & 0xC0) != 0x80) {
            w += 2;  /* 多字节字符的首字节；续字节是字符的一部分，跳过 */
        }
    }
    return w;
}

/* 打印 s 并用空格补到显示宽度 width */
static void pad(const char *s, size_t width) {
    fputs(s, stdout);
    const size_t w = disp_width(s);
    if (w < width) printf("%*s", (int)(width - w), "");
}

static void fmt_v(char *out, double v) {
    snprintf(out, BUFSZ, "%.3f V", v);
}

static void fmt_i(char *out, double amp) {
    const double a = fabs(amp);
    if (a >= 1.0) {
        snprintf(out, BUFSZ, "%.3f A", amp);
    } else if (a >= 1e-3) {
        snprintf(out, BUFSZ, "%.3f mA", amp * 1e3);
    } else if (a >= 1e-6) {
        snprintf(out, BUFSZ, "%.3f uA", amp * 1e6);
    } else if (a >= 1e-9) {
        snprintf(out, BUFSZ, "%.3f nA", amp * 1e9);
    } else {
        snprintf(out, BUFSZ, "%.2e A", amp);
    }
}

static void fmt_r(char *out, double ohm) {
    if (ohm >= 1e6) {
        snprintf(out, BUFSZ, "%.3f MOhm", ohm / 1e6);
    } else if (ohm >= 1e3) {
        snprintf(out, BUFSZ, "%.3f kOhm", ohm / 1e3);
    } else {
        snprintf(out, BUFSZ, "%.2f Ohm", ohm);
    }
}

static void fmt_p(char *out, double watt) {
    if (watt >= 1.0) {
        snprintf(out, BUFSZ, "%.3f W", watt);
    } else if (watt >= 1e-3) {
        snprintf(out, BUFSZ, "%.2f mW", watt * 1e3);
    } else {
        snprintf(out, BUFSZ, "%.2f uW", watt * 1e6);
    }
}

/* 封装额定功率（典型值）：0402=1/16W、0603=1/10W、0805=1/8W、1206=1/4W。
 * 工程上还要降额到 60% 以下用，所以判据是 P <= 额定 * 0.6。 */
static const char *package_advice(double watt) {
    if (watt <= 0.0625 * 0.6) return "0402 够用";
    if (watt <= 0.100 * 0.6) return "0603 够用";
    if (watt <= 0.125 * 0.6) return "0805 够用";
    if (watt <= 0.250 * 0.6) return "1206 够用";
    return "至少 1210 / 插件，或多颗并联";
}

static void rule(const char *title) {
    printf("\n============================================================\n");
    printf("  %s\n", title);
    printf("============================================================\n");
}

static void node_label(char *out, size_t cap, int node) {
    if (node == 0) {
        snprintf(out, cap, "GND");
    } else {
        snprintf(out, cap, "N%d", node);
    }
}

static void print_nodes(const Netlist *net, const Solution *sol) {
    char buf[BUFSZ];
    printf("  [节点电压]\n");
    printf("    ");
    pad("GND", 8);
    printf("= 0.000 V  (参考点，不是大地的绝对电位)\n");
    for (int i = 1; i <= net->nodeCount; ++i) {
        printf("    ");
        snprintf(buf, BUFSZ, "N%d", i);
        pad(buf, 8);
        fmt_v(buf, sol->nodeV[i]);
        printf("%s\n", buf);
    }
}

static void print_resistors(const Netlist *net, const Solution *sol) {
    char buf[BUFSZ];
    char na[16];
    char nb[16];
    printf("  [支路电流 / 功耗]  电流正方向 a -> b\n");
    for (int k = 0; k < net->nRes; ++k) {
        const Resistor *r = &net->resistors[k];
        printf("    ");
        pad(r->name, 7);
        node_label(na, sizeof na, r->a);
        node_label(nb, sizeof nb, r->b);
        snprintf(buf, BUFSZ, "%s->%s", na, nb);
        pad(buf, 11);
        fmt_r(buf, r->ohm);
        pad(buf, 13);
        fmt_i(buf, sol->resCurrent[k]);
        pad(buf, 14);
        fmt_p(buf, sol->resPower[k]);
        pad(buf, 12);
        printf("%s\n", package_advice(sol->resPower[k]));
    }
}

/* =====================================================================
 * 四、演示 ①：LED 限流电阻
 *     LED 用最粗暴的"恒压降模型"：导通就当成一颗固定 2.1V 的电压源。
 *     为什么敢这么糙？看本节末尾那张指数 I-V 表就明白了：
 *     电流从 1mA 变到 20mA，压降只挪了 0.1V 出头。
 *     既然压降几乎不动，就把它当"不动"，多出来的电压全归限流电阻扛。
 * ===================================================================== */
static void demo_led(void) {
    rule("演示 1 LED 限流：为什么必须有这颗电阻，为什么取 330");

    const double vcc = 5.0;
    const double vf = 2.1;  /* 红色 LED 典型导通压降 */

    printf("  电路：VCC(5.000V) --[R]-- N2 --[LED 恒压降 2.1V]-- GND\n");
    printf("  理论：I = (5.000 - 2.100) / R，方向是 VCC -> R -> LED -> GND -> VCC（一整圈）\n\n");

    printf("  ");
    pad("R", 13);
    pad("回路电流", 13);
    pad("R 上压降", 13);
    pad("R 功耗", 12);
    printf("封装建议\n");
    printf("  --------------------------------------------------------------------------\n");

    const double values[] = {10.0, 100.0, 330.0, 1000.0, 4700.0, 10000.0};
    char buf[BUFSZ];
    for (size_t k = 0; k < sizeof values / sizeof values[0]; ++k) {
        const double r = values[k];
        Netlist net;
        net_init(&net, 2);
        net_addV(&net, "VCC", 1, 0, vcc);   /* 节点 1 被理想电源钉在 5V */
        net_addR(&net, "R", 1, 2, r);       /* N1 -> N2 */
        net_addV(&net, "LED", 2, 0, vf);    /* 节点 2 被 LED 钉在 2.1V */
        Solution sol;
        solve(&net, &sol);
        const double cur = sol.resCurrent[0];
        printf("  ");
        fmt_r(buf, r);
        pad(buf, 13);
        fmt_i(buf, cur);
        pad(buf, 13);
        fmt_v(buf, cur * r);
        pad(buf, 13);
        fmt_p(buf, sol.resPower[0]);
        pad(buf, 12);
        printf("%s\n", package_advice(sol.resPower[0]));
    }

    printf("  ");
    pad("不限流", 11);
    printf("→ 恒压降模型直接无解\n");
    printf("  ");
    pad("", 11);
    printf("（5V 和 2.1V 被导线硬顶在一起，方程矛盾）。\n");
    printf("  ");
    pad("", 11);
    printf("模型失效的真实含义：电流只由电源内阻、\n");
    printf("  ");
    pad("", 11);
    printf("线路电阻和 LED 自己的动态电阻决定，\n");
    printf("  ");
    pad("", 11);
    printf("实际会冲到 1~3A，引脚发烫、几秒内光衰甚至开路。\n");

    printf("\n  师傅批注：\n");
    printf("    330Ω 是教科书答案：8.788mA、25.49mW，0402 都够用。\n");
    printf("    100Ω 那行 29mA：数据手册的 20mA 是【最大绝对值】不是【推荐值】。\n");
    printf("                而且如果这颗电阻接在 MCU 的 IO 口上，\n");
    printf("                GPIO 一般只有 8~20mA 能力（绝对最大 25mA），\n");
    printf("                烧的是 MCU 的引脚，不是 LED。\n");
    printf("    10k 那行 290uA：黑暗环境里看得见，做【待机指示】时\n");
    printf("                很多人故意用 4.7k~10k 省电，这是合理的设计取舍。\n");

    /* ---- LED 指数 I-V：解释"0.4V 之差就是两千倍电流" ---- */
    printf("\n  [附] 真实 LED 是指数关系 I = Is*(e^(V/(n*VT)) - 1)，取 Is=2e-20A、n=2、VT=25.85mV：\n");
    printf("    ");
    pad("Vf", 11);
    printf("I\n");
    const double vfs[] = {1.6, 1.8, 1.9, 2.0, 2.1, 2.2, 2.3};
    for (size_t k = 0; k < sizeof vfs / sizeof vfs[0]; ++k) {
        const double v = vfs[k];
        const double i = 2e-20 * (exp(v / (2.0 * 0.02585)) - 1.0);
        printf("    ");
        fmt_v(buf, v);
        pad(buf, 11);
        fmt_i(buf, i);
        printf("%s\n", buf);
    }
    printf("    → 1.9V 到 2.3V 只涨了 0.4V，电流从 0.18mA 涨到 418mA（两千多倍）。\n");
    printf("      所以【用电压源直接驱动 LED】这件事从根上就是错的：\n");
    printf("      必须用电阻（或恒流源）把【电压差】变成【电流的决定者】。\n");
}

/* =====================================================================
 * 五、演示 ②：分压带负载（负载效应）—— 本关最值钱的一张表
 *     空载：Vout = Vin·R2/(R1+R2) = 1.650V
 *     带载：把 R2 换成 R2∥RL 再分压
 *     戴维南：从输出端看进去 = 1.650V 串 5.000kΩ
 *     判据：后级阻抗 ≥ 100 × Rth，这个分压才算"没人打扰"。
 * ===================================================================== */
static void demo_divider(void) {
    rule("演示 2 分压带负载：输出电压被拉低了多少（负载效应）");

    const double vin = 3.3;
    const double r1 = 10e3;
    const double r2 = 10e3;

    Netlist open_net;
    net_init(&open_net, 2);
    net_addV(&open_net, "VIN", 1, 0, vin);
    net_addR(&open_net, "R1", 1, 2, r1);
    net_addR(&open_net, "R2", 2, 0, r2);
    Solution open_sol;
    solve(&open_net, &open_sol);
    const double v_open = open_sol.nodeV[2];

    char buf[BUFSZ];
    printf("  电路：VIN(3.300V) --[R1=10k]-- N2 --[R2=10k]-- GND，输出从 N2 取\n");
    print_nodes(&open_net, &open_sol);
    printf("  空载输出 = ");
    fmt_v(buf, v_open);
    printf("%s，戴维南内阻 Rth = R1∥R2 = ", buf);
    fmt_r(buf, r1 * r2 / (r1 + r2));
    printf("%s\n", buf);
    printf("  也就是说：这个分压在电气上等价于一颗 1.650V 的电池串 5.000kΩ 电阻。\n");
    print_resistors(&open_net, &open_sol);
    printf("  （注意 R1 和 R2 的电流一模一样：空载时没有第三条路可走，\n");
    printf("    这就是串联的定义——同一条支路，同一个电流。）\n\n");

    printf("  ");
    pad("负载 RL", 11);
    pad("实际输出", 13);
    pad("误差", 10);
    pad("负载电流", 13);
    pad("负载功耗", 12);
    printf("结论\n");
    printf("  ------------------------------------------------------------------------------------\n");

    struct Case {
        double rl;        /* <0 表示空载（网表里不加这条支路） */
        const char *name;
    };
    const struct Case cases[] = {
        {-1.0, "空载"},
        {1e6, "1 MOhm"},
        {100e3, "100 kOhm"},
        {10e3, "10 kOhm"},
        {1e3, "1 kOhm"},
        {100.0, "100 Ohm"},
    };

    for (size_t k = 0; k < sizeof cases / sizeof cases[0]; ++k) {
        const struct Case *c = &cases[k];
        Netlist net;
        net_init(&net, 2);
        net_addV(&net, "VIN", 1, 0, vin);
        net_addR(&net, "R1", 1, 2, r1);
        net_addR(&net, "R2", 2, 0, r2);
        if (c->rl > 0.0) net_addR(&net, "RL", 2, 0, c->rl);  /* 空载 = 干脆不加这条支路 */
        Solution sol;
        solve(&net, &sol);
        const double v_out = sol.nodeV[2];
        const double err = (v_out - v_open) / v_open * 100.0;
        const double i_load = (c->rl > 0.0) ? v_out / c->rl : 0.0;
        const double p_load = (c->rl > 0.0) ? v_out * v_out / c->rl : 0.0;

        const char *verdict;
        if (c->rl < 0.0) verdict = "纸上算出来的那个值";
        else if (err > -1.0) verdict = "误差 <1%，可以用";
        else if (err > -5.0) verdict = "误差 <5%，看用途勉强";
        else verdict = "负载效应吃掉一切：降阻值 或 加跟随器";

        char es[BUFSZ];
        snprintf(es, BUFSZ, "%.1f%%", err);
        printf("  ");
        pad(c->name, 11);
        fmt_v(buf, v_out);
        pad(buf, 13);
        pad(es, 10);
        fmt_i(buf, i_load);
        pad(buf, 13);
        fmt_p(buf, p_load);
        pad(buf, 12);
        printf("%s\n", verdict);
    }

    printf("\n  师傅批注：\n");
    printf("    挂 10k 的负载，输出从 1.650V 掉到 1.100V，误差 -33.3%%。\n");
    printf("    这就是【照抄网上一张分压图去接 ADC，读数永远偏低】的全部原因。\n");
    printf("    两条出路：\n");
    printf("      1) R1、R2 一起降 10 倍（1k/1k）：误差跟着降约 10 倍，\n");
    printf("         代价是静态电流从 165uA 涨到 1.65mA——电池供电的设备会骂人。\n");
    printf("      2) 后级加一级运放电压跟随（第三关）：跟随器输入阻抗 1e12 级，\n");
    printf("         分压再也没人打扰，1M/1M 也敢用。\n");
    printf("    自查口诀：先算 Rth = R1∥R2，再看后级要吃多少电流。\n");
}

/* =====================================================================
 * 六、演示 ③：惠斯通电桥 —— 平衡、不平衡、以及"被自己加载掉"
 *     平衡条件 R1/R2 = R3/R4 → 中间支路电流为 0（数值上 1e-16 级）
 *     不平衡时输出是【毫伏级】，而且你一挂上低阻检测元件，
 *     信号立刻被自己的检测电阻加载掉——这就是仪表放大器的存在理由。
 * ===================================================================== */
static void demo_bridge(void) {
    rule("演示 3 惠斯通电桥：平衡 / 不平衡 / 检测元件加载掉信号");

    /* 节点：1 = VBUS（钉在 5V）、2 = 左中点 L、3 = 右中点 R */
    struct BridgeCase {
        double r1, r2, r3, r4, rg;
        const char *name;
    };
    const struct BridgeCase cases[] = {
        {10e3, 10e3, 10e3, 10e3, 1e3, "四臂全 10k（平衡）"},
        {10.1e3, 10e3, 10e3, 10e3, 1e3, "R1→10.1k，检测 1k"},
        {10.1e3, 10e3, 10e3, 10e3, 1e6, "R1→10.1k，检测 1M"},
        {10.1e3, 10e3, 10e3, 10e3, 1e9, "R1→10.1k，仪放 1G"},
    };

    printf("  电路：VBUS(5.000V) 分两列。左列 R1(上)/R2(下) 中点 L=N2；\n");
    printf("        右列 R3(上)/R4(下) 中点 R=N3。Rg 接在 L 与 R 之间。\n\n");
    printf("  ");
    pad("情况", 28);
    pad("V(L)", 12);
    pad("V(R)", 12);
    pad("差分 V(L)-V(R)", 18);
    printf("Rg 电流\n");
    printf("  --------------------------------------------------------------------------------\n");

    char buf[BUFSZ];
    for (size_t k = 0; k < sizeof cases / sizeof cases[0]; ++k) {
        const struct BridgeCase *c = &cases[k];
        Netlist net;
        net_init(&net, 3);
        net_addV(&net, "VBUS", 1, 0, 5.0);
        net_addR(&net, "R1", 1, 2, c->r1);
        net_addR(&net, "R2", 2, 0, c->r2);
        net_addR(&net, "R3", 1, 3, c->r3);
        net_addR(&net, "R4", 3, 0, c->r4);
        net_addR(&net, "Rg", 2, 3, c->rg);
        Solution sol;
        solve(&net, &sol);
        const double vL = sol.nodeV[2];
        const double vR = sol.nodeV[3];
        const double diff = vL - vR;
        char ds[BUFSZ];
        snprintf(ds, BUFSZ, "%.3f mV", diff * 1e3);
        printf("  ");
        pad(c->name, 28);
        fmt_v(buf, vL);
        pad(buf, 12);
        fmt_v(buf, vR);
        pad(buf, 12);
        pad(ds, 18);
        fmt_i(buf, sol.resCurrent[4]);
        printf("%s\n", buf);
    }

    printf("\n  师傅批注：\n");
    printf("    平衡那一行的电流是 1e-16 ~ 1e-18 A 量级，那是浮点舍入噪声，\n");
    printf("    不是【还有一丁点电流】。物理结论是：电桥平衡时中间支路【真的】没有电流，\n");
    printf("    所以你可以把那根线直接剪断，电路行为一点都不变——\n");
    printf("    这就是【平衡】二字的工程含义，也是天平式/电桥式传感器精度高的根子。\n");
    printf("    而 R1 只变了 1%%（10k→10.1k），差分输出就只有 -12.44mV。\n");
    printf("    你用 1k 的表去读，只剩 -1.13mV —— 信号被你自己的表笔加载吃掉了。\n");
    printf("    换成 1G 的仪表放大器输入，才拿得到 -12.44mV 的满信号。\n");
    printf("    应变片、称重传感器后面为什么要挂 HX711 / INA128 这类仪放，\n");
    printf("    这张表就是答案：不是【要放大】这么简单，是【要高输入阻抗地放大】。\n");
}

/* =====================================================================
 * 七、演示 ④：电源内阻 —— 理想值全线崩塌的地方
 *     同一个电源，挂小电流负载没事，挂大电流负载直接塌。
 *     判据：负载电阻和电源内阻【同数量级】时，电压分掉一大半。
 * ===================================================================== */
static void demo_source_impedance(void) {
    rule("演示 4 电源内阻：为什么【5V 电源】挂了负载就不是 5V");

    printf("  电路：理想 5V --[Rs = 电源内阻]-- N2 --[Rload]-- GND\n\n");
    printf("  ");
    pad("Rs", 11);
    pad("Rload", 13);
    pad("V(load)", 12);
    pad("回路电流", 13);
    pad("内阻发热", 12);
    printf("评价\n");
    printf("  ----------------------------------------------------------------------------------------\n");

    struct Case {
        double rs;
        double rl;
        const char *name;
    };
    const struct Case cases[] = {
        {0.0, 332.0, "理想"},
        {2.0, 332.0, "2 Ohm"},
        {0.1, 5.0, "0.1 Ohm"},
        {2.0, 5.0, "2 Ohm"},
        {2.0, 1000.0, "2 Ohm"},
    };
    char buf[BUFSZ];
    for (size_t k = 0; k < sizeof cases / sizeof cases[0]; ++k) {
        const struct Case *c = &cases[k];
        /* 注意：Rs = 0（理想电源）时【不能】再建一个 N2 让负载挂上去——
         * 导线两端就是同一个节点，N2 会变成一个只连了一根的浮空节点，
         * 方程组直接奇异。这本身就是一课：没有回路就没有方程。
         * 所以理想电源这一行只需要一个节点：负载直接挂在被钉死的 N1 上。 */
        Netlist net;
        net_init(&net, c->rs > 0.0 ? 2 : 1);
        net_addV(&net, "IDEAL", 1, 0, 5.0);
        if (c->rs > 0.0) {
            net_addR(&net, "Rs", 1, 2, c->rs);
            net_addR(&net, "Rload", 2, 0, c->rl);
        } else {
            net_addR(&net, "Rload", 1, 0, c->rl);  /* 没有内阻，负载直接挂在被钉住的 N1 上 */
        }
        Solution sol;
        solve(&net, &sol);
        const double v_load = (c->rs > 0.0) ? sol.nodeV[2] : sol.nodeV[1];
        const double i_loop = v_load / c->rl;
        const double p_rs = (c->rs > 0.0) ? (5.0 - v_load) * (5.0 - v_load) / c->rs : 0.0;

        char verdict[BUFSZ * 3];
        if (c->rs <= 0.0) {
            snprintf(verdict, sizeof verdict, "理想电源：永远 5.000V（这种电源不存在）");
        } else if (v_load < 4.0) {
            const int eff = (int)(v_load / 5.0 * 100.0);
            snprintf(verdict, sizeof verdict, "崩了：负载与内阻同数量级，效率只有 %d%%", eff);
        } else if (i_loop > 0.1) {
            snprintf(verdict, sizeof verdict, "注意内阻发热，路径上每一欧都要算压降");
        } else {
            char dv[BUFSZ];
            fmt_v(dv, 5.0 - v_load);
            snprintf(verdict, sizeof verdict, "轻载：内阻吃掉 %s，基本可忽略", dv);
        }
        printf("  ");
        pad(c->name, 11);
        fmt_r(buf, c->rl);
        pad(buf, 13);
        fmt_v(buf, v_load);
        pad(buf, 12);
        fmt_i(buf, i_loop);
        pad(buf, 13);
        fmt_p(buf, p_rs);
        pad(buf, 12);
        printf("%s\n", verdict);
    }

    printf("\n  师傅批注：\n");
    printf("    挂 332Ω（约 15mA，等价一颗 LED 支路）时，2Ω 内阻只吃掉 30mV，\n");
    printf("    你根本感觉不到。挂 LED 那种 9mA 的支路，2Ω 上只有 18mV。\n");
    printf("    同样这颗 2Ω，挂 5Ω 负载（714mA，约等于 ESP32 WiFi 发射瞬间、\n");
    printf("    一个小型直流电机、或者一路满载的 3.3V LDO），\n");
    printf("    输出直接塌到 3.571V，而且内阻上烧 1.020W——\n");
    printf("    如果这颗【内阻】是一根杜邦线或者一段细铜皮，它现在就烫手了。\n");
    printf("    现场表现：MCU 一联网就复位、电机一转 ADC 就乱跳、\n");
    printf("    摄像头一启动就掉线。根子全在这张表里。\n");

    /* ---- 直流稳态下电容 = 开路：跑两遍让你亲眼看见数字完全一样 ---- */
    printf("\n  [附] 在这个【纯直流稳态】求解器里，并一颗 100uF 会怎样？\n");
    printf("  ");
    pad("", 14);
    pad("N2", 15);
    printf("R 上电流\n");
    for (int with_cap = 0; with_cap < 2; ++with_cap) {
        Netlist net;
        net_init(&net, 2);
        net_addV(&net, "VCC", 1, 0, 5.0);
        net_addR(&net, "R", 1, 2, 330.0);
        net_addV(&net, "LED", 2, 0, 2.1);
        /* 电容在直流稳态下就是开路，所以稳态网表里【根本不该出现它】。
         * 这里用 1e12 欧近似"开路"来演示"加了等于没加"。 */
        if (with_cap) net_addR(&net, "C100u", 2, 0, 1e12);
        Solution sol;
        solve(&net, &sol);
        printf("  ");
        pad(with_cap ? "并了 100uF" : "没并电容", 14);
        fmt_v(buf, sol.nodeV[2]);
        pad(buf, 15);
        fmt_i(buf, sol.resCurrent[0]);
        printf("%s\n", buf);
    }
    printf("    → 两行数字完全相同。直流稳态下电容 = 开路，它【不改变稳态】，\n");
    printf("      只改变【到达稳态要多久】和【突变时能顶多久】。\n");
    printf("      那叫暂态，去看 02_rc_transient.c。\n");
}

int main(void) {
    printf("================================================================\n");
    printf("  第 1 关 · 电路基础 · 节点电压法求解器（手写内核，零第三方库）\n");
    printf("  内核 = KCL + 欧姆定律 + MNA + 高斯约当消元 = SPICE 的心脏\n");
    printf("================================================================\n");

    demo_led();
    demo_divider();
    demo_bridge();
    demo_source_impedance();

    printf("\n================================================================\n");
    printf("  收工。留三个问题（答不上就回去读 tutorial.md 第三节）：\n");
    printf("  1) 把演示 2 的 R1、R2 都改成 1M，空载还是 1.650V 吗？\n");
    printf("     为什么此时挂 10M 的负载误差也只有 4.8%%，\n");
    printf("     但你用万用表（10M 内阻）去量，读数会掉到 1.57V 附近（−4.8%%）？\n");
    printf("     （答案就在【表笔也是电路的一部分】这一条。）\n");
    printf("  2) 把演示 3 的 Rg 那一整条支路删掉（等于剪断那根线），\n");
    printf("     平衡电桥的 V(L)、V(R) 会变吗？不平衡的会变吗？自己改代码验证。\n");
    printf("  3) 把演示 1 里 LED 的模型从 2.1V 改成 2.0V 和 2.2V，\n");
    printf("     电流各变成多少？这个【模型误差】折算成电阻容差是百分之几？\n");
    printf("     （提示：E24 系列的电阻本身就有 ±5%%，你算那么准给谁看？）\n");
    printf("================================================================\n");
    return 0;
}
