/* =====================================================================
 * 04_fsm_traffic.c —— 表驱动状态机引擎 + 十字路口交通灯 + 按键消抖
 *
 * 【这个文件讲什么】
 *   1) 一个通用的「表驱动 FSM 引擎」：状态枚举 + 事件枚举 + 一张
 *      const 转移表（当前态 / 事件 / 条件 / 次态 / 动作），
 *      引擎只做一件事：查表、查不到就走 default。
 *      这就是实际项目里状态机的正确写法：状态逻辑是「数据」，不是「if-else 面条」。
 *   2) 十字路口交通灯：红 → 红黄 → 绿 → 黄 →（行人请求则进行人态）→ 红，
 *      带车流传感器加速、带故障注入，任何非法组合一律被 default 拉进 SAFE（全向黄闪）。
 *   3) 按键消抖状态机：10ms 采样一次，连续 3 次（30ms）稳定才算真按下，
 *      按住到阈值报「长按」，之后按连发间隔重复报；抖动毛刺直接被作废。
 *   4) 对照组：把 default 分支删掉会怎样。同一段激励重跑一遍，
 *      状态变量直接跑飞成枚举里根本不存在的值，然后永久卡死。
 *      这就是硬件里没有 default 时综合器给你锁存器 / 未定义状态的真相。
 *
 * 【怎么编译】（MSYS2 / MinGW，gcc 16.2 + Windows 下验证零警告）
 *   gcc -std=c17 -O2 -Wall -Wextra 04_fsm_traffic.c -o 04_fsm_traffic.exe
 *
 * 【怎么运行】
 *   chcp 65001        ← 先执行！不切 UTF-8，Windows 控制台中文全是乱码
 *   .\04_fsm_traffic.exe
 *      （没有随机数，纯确定性状态机，跑一百遍输出完全一样）
 *
 * 【预期看到什么】
 *   - 交通灯完整转移表 23 条规则（其中 FAULT 显式占 6 条），40 个 tick 逐拍输出；
 *   - 0~3 拍红灯计时，第 4 拍进红+黄，第 6 拍放绿灯；第 9 拍行人按钮被记录（标志=1）
 *     但绝不打断绿灯；第 12 拍车流传感说本向无车 → 绿灯提前收尾进黄灯；
 *     黄灯计满 2 拍，因为标志=1，第 15 拍进行人态，第 19 拍回红灯；
 *   - 第 28 拍注入 FAULT → 显式规则进 SAFE（全向黄闪），第 32 拍 CLEAR 才回红灯；
 *   - 第 36 拍注入未定义事件 99 → 表里一条都查不到，default 兜住照样进 SAFE，
 *     第 38 拍 CLEAR 又能出得来。全程 default 命中 1 次、非法状态 0 次：PASS；
 *   - 对照组（删掉 default）：同样喂 99，状态立刻变成枚举里根本不存在的值，
 *     后面 3 拍喂的是完全正常的 TICK，一条规则都匹配不上，永久卡死，非法状态 4 次；
 *   - 消抖：第 4 个样本那个 10ms 毛刺被判为抖动直接作废（0 次误报按下）；
 *     第二次真正按下要等到第 9 个样本（第 6/7/8/9 连续 4 次为 1）才报「按下」，
 *     第 13 个样本松手报「短按（单击）」；第三次从第 20 拍按住不放，
 *     第 43 拍（按住第 20 次）报「长按有效」，第 51 拍报第一次连发。
 *     汇总：毛刺作废 1 次、确认按下 2 次、短按 1 次、长按 1 次、连发 1 次。
 *
 * 【老师傅点拨】
 *   状态机最贵的不是状态多，是「漏了一个组合」。产品上电时按下按键、
 *   传感器同时报错、复位刚释放外设还没稳 —— 这些组合你在实验室见不到，
 *   用户能给你造出来。default 分支就是留给它们的门，删了就等于赌运气。
 * ===================================================================== */

#include <stdio.h>

/* ================================================================== 引擎 */

typedef struct {
    int         s;         /* 当前态 */
    int         ev;        /* 事件（或采样电平） */
    int         ns;        /* 次态 */
    int         guard;     /* 计数器 >= guard 才允许匹配（0 = 无条件） */
    int         need;      /* 需要的标志位：1 = 必须为 1，-1 = 必须为 0，0 = 不关心 */
    int         set;       /* 命中后对标志位做什么：1 置位，-1 清零，0 不动 */
    int         reset;     /* 命中后计数器是否强制清零 */
    const char *act;       /* 输出动作 */
} Rule;

/*
 * 走一步。
 *   tick_evt ：哪个事件编号算「时钟节拍」（只有它会让计数器 +1）
 *   has_default：0 = 模拟「把 default 分支删掉」
 * 返回值：>=0 = 命中的规则下标；-1 = 未命中但被 default 兜住；
 *         -2 = 未命中且没有 default（状态跑飞）
 */
static int fsm_step(int *st, int *cnt, int *flag, int ev,
                    const Rule *tab, int ntab, int safe, int tick_evt,
                    const char **act, int has_default)
{
    int i;

    for (i = 0; i < ntab; i++) {
        const Rule *r = &tab[i];

        if (r->s != *st || r->ev != ev) continue;
        if (*cnt < r->guard) continue;
        if (r->need == 1 && *flag == 0) continue;
        if (r->need == -1 && *flag == 1) continue;

        *act = r->act;
        if (r->set == 1) *flag = 1;
        else if (r->set == -1) *flag = 0;

        if (r->reset || r->ns != *st) {
            *cnt = 0;                     /* 换状态：计时从头再来 */
        } else if (ev == tick_evt) {
            *cnt += 1;                    /* 自环且是节拍事件：计时累加 */
        }
        *st = r->ns;
        return i;
    }

    *act = NULL;
    if (has_default) {
        *st = safe;
        *cnt = 0;
        *flag = 0;
        return -1;
    }

    /* 没有 default：硬件里等价于「这个组合没有任何逻辑覆盖」，
       状态寄存器被未定义的组合逻辑改写。这里就让它真的飞一下。 */
    *st = *st + ev + 1;
    *cnt = 0;
    return -2;
}

static const char *name_of(int v, const char *const *tbl, int n)
{
    if (v < 0 || v >= n) return "<未定义>";
    return tbl[v];
}

static void dump_table(const Rule *tab, int ntab,
                       const char *const *snames, int ns,
                       const char *const *enames, int ne)
{
    printf("   #  | 当前态      | 事件        | 计数>= | 条件 | 次态        | 动作\n");
    printf("  ----+-------------+-------------+--------+------+-------------+-----------------\n");
    for (int i = 0; i < ntab; i++) {
        printf("  %3d | %-11s | %-11s |  %4d  | %-4s | %-11s | %s\n",
               i + 1,
               name_of(tab[i].s, snames, ns),
               name_of(tab[i].ev, enames, ne),
               tab[i].guard,
               tab[i].need == 1 ? "有标志" : (tab[i].need == -1 ? "无标志" : "—"),
               name_of(tab[i].ns, snames, ns),
               tab[i].act);
    }
}

/* ==================================================== 例 1：交通灯状态机 */

enum { L_RED = 0, L_REDY, L_GREEN, L_YELLOW, L_PED, L_SAFE, L_NSTATE };

enum { EV_TICK = 0, EV_PED_REQ, EV_SENSOR, EV_FAULT, EV_CLEAR, EV_NEVENT };

static const char *const LNAME[L_NSTATE] = {
    "RED", "RED+Y", "GREEN", "YELLOW", "PED", "SAFE"
};
static const char *const ENAME[EV_NEVENT] = {
    "TICK", "PED_REQ", "SENSOR", "FAULT", "CLEAR"
};

/*
 * 计时约定：进入状态时计数 = 0，每个 TICK 自环 +1，
 * 所以 guard = N 表示「进入本状态后的第 N+1 拍」才真正转移。
 * 红 4 拍 / 红黄 1 拍 / 绿 6 拍 / 黄 2 拍 / 行人 3 拍。
 *
 * 注意 FAULT 那五条：每个状态都要单独写一条。
 * 这就是表驱动的好处 —— 故障路径是「看得见的行」，评审时逐行数得完；
 * 写成 if-else 的话，你根本不确定是不是每个分支都处理了故障。
 */
static const Rule LIGHT_TABLE[] = {
    /* 红灯：到点先亮红黄（起步预备），没人按按钮就直接回环继续红 */
    { L_RED,    EV_TICK,     L_REDY,   4,  0,  0, 0, "红灯计时到 → 红+黄预备"     },
    { L_RED,    EV_PED_REQ,  L_RED,    0,  0,  1, 0, "红灯：记下行人请求"          },
    { L_RED,    EV_SENSOR,   L_RED,    0,  0,  0, 0, "红灯：传感只是记录"          },
    { L_RED,    EV_TICK,     L_RED,    0,  0,  0, 0, "红灯全停"                    },

    /* 红黄：一拍后放行 */
    { L_REDY,   EV_TICK,     L_GREEN,  1,  0,  0, 0, "红+黄 → 绿灯放行"            },
    { L_REDY,   EV_TICK,     L_REDY,   0,  0,  0, 0, "红+黄（起步预备）"           },

    /* 绿灯：正常计时到点转黄；传感说没车就提前转黄；行人请求只记录不打断 */
    { L_GREEN,  EV_TICK,     L_YELLOW, 6,  0,  0, 0, "绿灯计时到 → 黄灯"           },
    { L_GREEN,  EV_SENSOR,   L_YELLOW, 0,  0,  0, 0, "传感：本向无车，提前收尾"    },
    { L_GREEN,  EV_PED_REQ,  L_GREEN,  0,  0,  1, 0, "绿灯：记下行人请求，不打断"  },
    { L_GREEN,  EV_TICK,     L_GREEN,  0,  0,  0, 0, "绿灯通行"                    },

    /* 黄灯：有人请求就进行人态，没人就回红灯。两条规则靠 need 分流 */
    { L_YELLOW, EV_TICK,     L_PED,    2,  1, -1, 0, "有人请求 → 行人通行态"       },
    { L_YELLOW, EV_TICK,     L_RED,    2, -1,  0, 0, "无人请求 → 回红灯"           },
    { L_YELLOW, EV_TICK,     L_YELLOW, 0,  0,  0, 0, "黄灯（清空路口）"            },

    /* 行人态：给足时间，然后回红灯 */
    { L_PED,    EV_TICK,     L_RED,    3,  0,  0, 0, "行人放行结束 → 红灯"         },
    { L_PED,    EV_TICK,     L_PED,    0,  0,  0, 0, "行人绿灯（车辆全停）"        },

    /* 故障：显式覆盖每一个工作状态，一条都不能漏 */
    { L_RED,    EV_FAULT,    L_SAFE,   0,  0, -1, 0, "自检故障 → 安全态"           },
    { L_REDY,   EV_FAULT,    L_SAFE,   0,  0, -1, 0, "自检故障 → 安全态"           },
    { L_GREEN,  EV_FAULT,    L_SAFE,   0,  0, -1, 0, "自检故障 → 安全态"           },
    { L_YELLOW, EV_FAULT,    L_SAFE,   0,  0, -1, 0, "自检故障 → 安全态"           },
    { L_PED,    EV_FAULT,    L_SAFE,   0,  0, -1, 0, "自检故障 → 安全态"           },

    /* 安全态下只有黄闪，只有 CLEAR 能放出来 */
    { L_SAFE,   EV_FAULT,    L_SAFE,   0,  0,  0, 0, "故障持续，继续黄闪"          },
    { L_SAFE,   EV_CLEAR,    L_RED,    0,  0, -1, 0, "故障排除 → 回红灯复位"       },
    { L_SAFE,   EV_TICK,     L_SAFE,   0,  0,  0, 0, "全向黄闪（安全态）"          },
};
#define NLIGHT ((int)(sizeof(LIGHT_TABLE) / sizeof(LIGHT_TABLE[0])))

/* default 分支的动作 —— 表里查不到的组合，唯一去处 */
static const char *const DEFAULT_ACT = "!! 表外组合 → default 拉进安全态（黄闪）";

static void lab_traffic_light(void)
{
    int  seq[40];
    int  st = L_RED, cnt = 0, flag = 0;
    int  def_hits = 0, illegal = 0;

    printf("\n============ 例 1：十字路口交通灯（表驱动 FSM） ============\n");
    printf("计时档位：红 4 拍 / 红黄 1 拍 / 绿 6 拍 / 黄 2 拍 / 行人 3 拍，每拍 = 1 tick\n\n");
    dump_table(LIGHT_TABLE, NLIGHT, LNAME, L_NSTATE, ENAME, EV_NEVENT);

    for (int i = 0; i < 40; i++) seq[i] = EV_TICK;
    seq[9]  = EV_PED_REQ;    /* 行人按下按钮 */
    seq[12] = EV_SENSOR;     /* 车流传感：本向没车，绿灯提前收尾 */
    seq[28] = EV_FAULT;      /* 传感器自检报错 */
    seq[32] = EV_CLEAR;      /* 故障排除，回到红灯 */
    seq[36] = 99;            /* 用户造出来的非法事件：表里根本没有这个编码 */
    seq[38] = EV_CLEAR;      /* default 兜进 SAFE 之后，还得能出得来 */

    printf("\n---- 40 个 tick 逐拍输出（含 default 分支）----\n");
    printf("  tick | 事件        | 状态转移        | 计数 | 标志 | 输出\n");
    printf("  -----+-------------+-----------------+------+------+----------------------\n");
    for (int t = 0; t < 40; t++) {
        const char *act = NULL;
        int before = st;
        int r = fsm_step(&st, &cnt, &flag, seq[t],
                         LIGHT_TABLE, NLIGHT, L_SAFE, EV_TICK, &act, 1);

        if (r < 0) {                  /* 表里一条都没查到 —— 全靠 default 兜住 */
            act = DEFAULT_ACT;
            def_hits++;
        }
        if (st < 0 || st >= L_NSTATE) illegal++;

        printf("  %4d | %-11s | %-5s → %-5s |  %4d |   %d   | %s\n",
               t, name_of(seq[t], ENAME, EV_NEVENT),
               name_of(before, LNAME, L_NSTATE), name_of(st, LNAME, L_NSTATE),
               cnt, flag, act ? act : "（无）");
    }
    printf("  => default 分支命中 %d 次，非法状态出现 %d 次：%s\n",
           def_hits, illegal, illegal ? "FAIL" : "PASS（状态永远合法）");

    /* ---------------- 对照组：把 default 分支删掉 ---------------- */
    printf("\n---- 对照组：删掉 default 分支，同样喂一个未定义事件 ----\n");
    st = L_GREEN; cnt = 0; flag = 0;
    illegal = 0;
    printf("  tick | 事件        | 状态转移        | 输出\n");
    printf("  -----+-------------+-----------------+-------------------------------\n");
    for (int t = 0; t < 7; t++) {
        const char *act = NULL;
        int before = st;
        int ev = (t == 3) ? 99 : EV_TICK;         /* 第 3 拍喂进未定义编码 */
        int r = fsm_step(&st, &cnt, &flag, ev,
                         LIGHT_TABLE, NLIGHT, L_SAFE, EV_TICK, &act, 0);

        if (r < 0) act = "!! 无任何规则覆盖：状态寄存器被未定义逻辑改写";
        if (st < 0 || st >= L_NSTATE) illegal++;

        printf("  %4d | %-11s | %-5s → %-5s | %s\n",
               t, name_of(ev, ENAME, EV_NEVENT),
               name_of(before, LNAME, L_NSTATE), name_of(st, LNAME, L_NSTATE),
               act ? act : "（无）");
    }
    printf("  => 非法状态出现 %d 次。注意第 4~6 拍喂的是完全正常的 TICK，\n", illegal);
    printf("     可状态已经不在表里了，一条规则都匹配不上 —— 机器就停在那儿不动了。\n");
    printf("     硬件里一模一样：没有 default 的 case，综合器给你锁存器 + 未定义状态，\n");
    printf("     上电、单粒子翻转、外部干扰把状态推进未编码的组合，就再也爬不出来。\n");
}

/* ====================================================== 例 2：按键消抖 */

enum { DB_IDLE = 0, DB_WAIT, DB_DOWN, DB_LONG, DB_NSTATE };

#define DB_TICK_MS    10    /* 每 10ms 采样一次 */
#define DB_DEBOUNCE    3    /* 连续 3 次 = 30ms 稳定才算真按下 */
#define DB_LONG_TH    20    /* 按住 20 次 = 200ms 报长按 */
#define DB_REPEAT      8    /* 长按后每 8 次 = 80ms 连发一次 */

static const char *const DBNAME[DB_NSTATE] = { "IDLE", "WAIT", "DOWN", "LONG" };
static const char *const DBEV[2] = { "电平0", "电平1" };

static const Rule DB_TABLE[] = {
    { DB_IDLE, 0, DB_IDLE, 0, 0, 0, 0, "空闲（电平低）" },
    { DB_IDLE, 1, DB_WAIT, 0, 0, 0, 0, "采到 1，开始确认 —— 先别当真" },

    /* WAIT：掉回去就是抖动，直接作废，一点不留恋 */
    { DB_WAIT, 0, DB_IDLE, 0, 0, 0, 0, "抖动！毛刺作废，回空闲" },
    { DB_WAIT, 1, DB_DOWN, DB_DEBOUNCE - 1, 0, 0, 0, "连续稳定 → 判定「按下」" },
    { DB_WAIT, 1, DB_WAIT, 0, 0, 0, 0, "消抖确认中…" },

    /* DOWN：提前松手 = 短按；按住到阈值 = 长按 */
    { DB_DOWN, 0, DB_IDLE, 0, 0, 0, 0, "松手 → 短按（单击）" },
    { DB_DOWN, 1, DB_LONG, DB_LONG_TH - 1, 0, 0, 0, "按住不放 → 长按有效" },
    { DB_DOWN, 1, DB_DOWN, 0, 0, 0, 0, "按住中…" },

    /* LONG：到连发间隔就报一次，注意 reset=1，不然计数器一直累加会连发不停 */
    { DB_LONG, 1, DB_LONG, DB_REPEAT - 1, 0, 0, 1, "连发一次（repeat）" },
    { DB_LONG, 1, DB_LONG, 0, 0, 0, 0, "长按保持…" },
    { DB_LONG, 0, DB_IDLE, 0, 0, 0, 0, "松手 → 结束长按" },
};
#define NDB ((int)(sizeof(DB_TABLE) / sizeof(DB_TABLE[0])))

/* 表里那几条「要数一数发生了几次」的规则，按下标认领。
   写成枚举而不是硬编码数字，改表的时候记得同步这里。 */
enum {
    R_JUNK   = 2,   /* WAIT 电平掉回 0 → 抖动作废        */
    R_PRESS  = 3,   /* WAIT 连续稳定   → 判定按下        */
    R_SHORT  = 5,   /* DOWN 提前松手   → 短按（单击）    */
    R_LONG   = 6,   /* DOWN 按住到阈值 → 长按有效        */
    R_REPEAT = 8    /* LONG 到连发间隔 → 连发一次        */
};

static void lab_debounce(void)
{
    static int raw[70];
    int n = 0;
    int st = DB_IDLE, cnt = 0, flag = 0;
    int press = 0, short_press = 0, long_press = 0, repeat = 0, junk = 0;

    printf("\n============ 例 2：按键消抖状态机（每 %d ms 采样一次） ============\n", DB_TICK_MS);
    printf("阈值：消抖 %d 次（%dms）、长按 %d 次（%dms）、连发每 %d 次（%dms）\n\n",
           DB_DEBOUNCE, DB_DEBOUNCE * DB_TICK_MS,
           DB_LONG_TH, DB_LONG_TH * DB_TICK_MS,
           DB_REPEAT, DB_REPEAT * DB_TICK_MS);
    dump_table(DB_TABLE, NDB, DBNAME, DB_NSTATE, DBEV, 2);

    /* 构造激励：
       4: 一个 10ms 毛刺（抖动）；5 掉回 0
       6~12: 真按下（6,7,8 三次稳定 → 第 8 次判定按下）
       13: 松手 → 短按
       20~55: 第二次按下，按住不放 → 长按 + 连发
       56: 松手                                                     */
    for (int i = 0; i < 60; i++) raw[i] = 0;
    raw[4] = 1;
    for (int i = 6; i <= 12; i++) raw[i] = 1;
    for (int i = 20; i <= 55; i++) raw[i] = 1;
    n = 60;

    printf("\n---- 逐次采样输出（%d 个采样点 = %d ms）----\n", n, n * DB_TICK_MS);
    printf("  采样 | 时刻(ms) | 原始电平 | 状态转移       | 输出\n");
    printf("  -----+----------+----------+----------------+------------------------\n");
    for (int t = 0; t < n; t++) {
        const char *act = NULL;
        int before = st;

        /* 分类一律按「命中的是表里第几条规则」来数。
           千万别拿 act 字符串的首字节去比 —— 中文在 UTF-8 里是 3 个字节，
           act[0] 只是个 0xE8/0xE6 之类的非法值，比了也是白比；
           而 '连' 这种多字节字符常量，gcc 直接给你 warning。 */
        int r = fsm_step(&st, &cnt, &flag, raw[t],
                         DB_TABLE, NDB, DB_IDLE, 1, &act, 1);

        if (r == R_JUNK)   junk++;
        else if (r == R_PRESS)  press++;
        else if (r == R_SHORT)  short_press++;
        else if (r == R_LONG)   long_press++;
        else if (r == R_REPEAT) repeat++;
        printf("  %4d |   %5d  |    %d     | %-5s → %-5s | %s\n",
               t, t * DB_TICK_MS, raw[t],
               name_of(before, DBNAME, DB_NSTATE), name_of(st, DBNAME, DB_NSTATE),
               act ? act : "（无）");
    }
    printf("  => 毛刺作废 %d 次，确认按下 %d 次，短按 %d 次，长按 %d 次，连发 %d 次\n",
           junk, press, short_press, long_press, repeat);

    printf("\n师傅的话：\n");
    printf("  ① 机械按键的抖动是 1~5 ms 量级，而且每次都不一样。你要是「读电平 = 按下」，\n");
    printf("     一次按键 MCU 能读到七八个边沿 —— 音量一下跳七格就是这么来的。\n");
    printf("  ② 只加 100nF 电容消抖，测出来抖 15 ms 打穿 —— 电容容量按 RC 算出来是 1 ms，\n");
    printf("     可按键datasheet写的抖动是「典型值」，不是上限。要么 RC 取到 10~20 ms，\n");
    printf("     要么硬件给个施密特（74HC14）+ 软件再做 20~30 ms 的稳定判定，双保险。\n");
    printf("  ③ 注意 LONG 那条连发规则里的 reset：计数器不清零，连发就会变成「每拍都发」。\n");
    printf("     状态机里「计时器什么时候清零」，比状态本身更容易写出 bug。\n");
    printf("  ④ 消抖计数用的是采样次数不是真时间。一旦你改了采样周期，\n");
    printf("     所有阈值都得跟着改 —— 所以阈值一律写成「毫秒 / 采样周期」，别写死数字。\n");
}

int main(void)
{
    printf("=================================================================\n");
    printf("        数字电路 · 表驱动状态机（交通灯 + 按键消抖）\n");
    printf("=================================================================\n");
    printf("一句话：状态机的正确形态是「一张 const 表 + 一个引擎 + 一个 default」，\n");
    printf("        不是三百行 if-else。查不到的组合必须有地方去，那就是 default。\n");

    lab_traffic_light();
    lab_debounce();

    printf("\n=================================================================\n");
    printf(" 收尾三句：\n");
    printf(" 1. 状态、事件、动作全部写进表里，代码只负责查表 —— 评审时一眼看完全部逻辑。\n");
    printf(" 2. default 分支必须存在，而且必须指向一个「安全态」，不是指向「什么都不做」。\n");
    printf(" 3. 状态变量要能自检：上电先归安全态，运行中发现越界立刻复位，别硬撑。\n");
    printf("=================================================================\n");
    return 0;
}
