/*
 * ============================================================================
 * 文件名：04_scheduler_irq.c
 * ----------------------------------------------------------------------------
 * 【讲什么】
 *   用纯软件模拟一颗 Cortex-M3 的中断控制器（NVIC）：
 *     - 优先级分组（抢占位 + 子优先级位）
 *     - pending 位图、正在执行栈、抢占仲裁规则
 *     - 三个中断源：TIM 1kHz 定时、UART RX 收包、ADC EOC 转换完成
 *     - 主循环任务表（周期任务 + 关中断临界区任务 + 喂狗任务）
 *   跑 2100 tick（1 tick = 1 µs）统计每个 ISR 的响应延迟与最坏情况；
 *   然后现场复刻三大惨案：
 *     1) 高优先级 ISR 里做 5ms 忙等 → 低优先级中断饿死 + IWDG 复位
 *     2) 修正版：ISR 只置标志，重活扔给主循环延迟队列
 *     3) 优先级反转：中优先级把低优先级踢下去，害得高优先级等更久
 *   最后附赠 IWDG 超时计算器（LSI 40kHz、预分频、12 位重装值）。
 *
 * 【怎么编译运行】（MSYS2 / gcc 16.2，Windows，C17 零警告）
 *   chcp 65001            ← 运行前先执行这一句，否则 Windows 控制台中文乱码
 *   gcc -std=c17 -O2 -Wall -Wextra 04_scheduler_irq.c -o 04_scheduler_irq.exe
 *   .\04_scheduler_irq.exe
 *   不需要任何硬件，时间轴是虚拟的，仿真步长 1 µs。
 *   注：本文件不用 M_PI（-std=c17 严格模式下 MSYS2 不暴露它），
 *       需要圆周率就自己 #define PI 3.14159265358979323846 —— 这里用不到。
 *
 * 【预期看到什么】
 *   - 演示一：调度表。TIM 最坏延迟 40 µs——事件恰好落进主循环刚起跑的
 *     40 µs 关中断临界区，只能在窗口外干等；抢占次数为 0，因为同级事件
 *     只排队不抢占。
 *   - 演示二：TIM 的 ISR 忙等 5000 µs，周期才 1000 µs，债越还越多。
 *     t=1000 风暴开始后 UART/ADC 全部饿死，主循环 0 次喂狗，
 *     t=100000 µs（IWDG 标称 100ms）准时复位。
 *   - 演示三：ISR 缩到 2 µs 只置标志，重活搬到主循环，跑满 100ms 不复位。
 *   - 演示四：同一份代码只改低优先级的优先级值，高优先级请求的
 *     完成延迟从约 1401 µs 降到约 601 µs。
 *   - 演示五：IWDG 超时对照表 + 一道「LSI 实际 30~60 kHz」的陷阱题。
 *
 * 【老师傅点拨】
 *   1. 中断响应延迟从来不是玄学，它 = 关中断窗口剩余 + 同级/上级 ISR 尾巴
 *      + 入栈时间（真机 12 个周期，本模型近似 1 µs，偏保守）。
 *   2. ISR 的黄金守则：进去、清挂起位、置标志、出来。超过 10 µs 的活
 *      都要问一句「这活能不能挪到主循环」。
 *   3. 看门狗要「有条件地喂」：只在该干的活干完之后喂。无条件喂狗 = 没喂。
 *   4. 裸机上没有优先级继承，优先级反转只能靠「给干活的人排高优先级」
 *      或「临界区真的把中断关掉」来治。
 *   5. C 里写这种状态机仿真，别用类：struct 放数据、函数指针都省了——
 *      直接把 step/enter/exit_top 写成收 struct Sim * 的普通函数，
 *      这正是裸机固件里 module_init/module_step 的标准组织方式。
 * ============================================================================
 */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>   /* memset */

/* ──────────────────────────────────────────────────────────────────────────── */
/* 模型常量                                                                     */
/* ──────────────────────────────────────────────────────────────────────────── */
#define ENTRY_LAT 1LL      /* 入栈+取向量近似 1 µs（真机 12 周期@72MHz≈0.17 µs，取保守整） */
#define MAX_IRQ   3
#define MAX_STACK 4
#define MAX_TASK  8
#define MAX_FLAG  4
#define MAX_FIRE  4

/* ──────────────────────────────────────────────────────────────────────────── */
/* NVIC 优先级分组：group ∈ {0,1,2,3,4}                                         */
/*   STM32F1 的 NVIC_PriorityGroup_x 决定 4 位优先级里高位几 bit 是抢占、       */
/*   低位几 bit 是子优先级。group=2 就是常见的「2 位抢占 × 2 位子优先级」。     */
/* ──────────────────────────────────────────────────────────────────────────── */
static int g_priority_group = 2;

static int eff_pre(int raw)
{
    /* 高 group 位当抢占值；group=0 时所有人抢占值相同（没人能抢占，只排队） */
    if (g_priority_group == 0) return 0;
    return raw >> (4 - g_priority_group);
}

static int eff_sub(int raw)
{
    if (g_priority_group == 4) return 0;
    return raw & ((1 << (4 - g_priority_group)) - 1);
}

/* ──────────────────────────────────────────────────────────────────────────── */
/* 配置结构                                                                     */
/* ──────────────────────────────────────────────────────────────────────────── */
struct IrqCfg {
    const char *name;
    long long period;      /* >0：每 period µs 自动产生一次事件；0：不自动 */
    long long work;        /* ISR 本体执行时长（µs） */
    int raw;               /* 4 位原始优先级，0 最高 */
    int exit_set_flag;     /* ISR 退出时置位 flags[i]，-1 表示无 */
    int exit_fire_irq;     /* ISR 退出时触发另一个 IRQ，-1 表示无 */
};

struct TaskCfg {
    const char *name;
    long long first;       /* 首次到期时刻（µs） */
    long long period;      /* >0：完成后隔 period 再来一轮；0：仅由 gate 触发 */
    long long cost;        /* 需要连续攒够的执行时间（µs，可被中断打断后续跑） */
    bool critical;         /* true：执行期间 PRIMASK 关中断（模拟临界区） */
    bool feeds_dog;        /* true：完成时喂一次 IWDG */
    int gate_flag;         /* >=0：flags[gate] 置位才准跑，跑完清掉（延迟队列） */
    int done_set_flag;     /* 完成时置位 flags[i]，-1 表示无 */
};

/* ──────────────────────────────────────────────────────────────────────────── */
/* 仿真器                                                                       */
/* ──────────────────────────────────────────────────────────────────────────── */
struct IrqState {
    bool enabled, pending;
    long long event_t;      /* 本次事件发生时刻 */
    long long enters, lat_sum, lat_max, busy_us;
    long long starve_peak;  /* 未响应事件的最长挂起时间（演示饿死用） */
};

struct TaskState {
    long long rem;          /* 本轮剩余 µs */
    long long next_due;
    long long crit_rem;     /* 临界区剩余（>0 表示 PRIMASK=1） */
    long long done_cnt;
};

/* 抢占栈的一帧与一次性点火事件：C 里没有嵌套类型名，提到文件作用域 */
struct Frame { int id; long long rem; };
struct Fire  { long long when; int id; };

struct Sim {
    long long t;
    long long t_end;
    long long iwdg_timeout; /* 0 = 不看门狗 */
    long long wdg_deadline; /* t 到点没喂就复位 */
    bool reset_fired;

    struct IrqState irq[MAX_IRQ];
    struct IrqCfg icfg[MAX_IRQ];
    int n_irq;

    struct TaskState task[MAX_TASK];
    struct TaskCfg tcfg[MAX_TASK];
    int n_task;

    bool flags[MAX_FLAG];

    struct Frame stack[MAX_STACK];
    int top;
    long long preempt_cnt;

    /* 一次性手动点火列表 */
    struct Fire fires[MAX_FIRE];
    int n_fire;
};

static void sim_setup(struct Sim *s,
                      const struct IrqCfg *ics, int nic,
                      const struct TaskCfg *tcs, int ntc,
                      long long tend, long long wdt)
{
    memset(s, 0, sizeof(*s));               /* 全字段一次清零，t/top/flags 等全归零 */
    s->t = 0; s->t_end = tend; s->iwdg_timeout = wdt;
    s->wdg_deadline = (wdt > 0) ? wdt : 0;
    s->n_irq = nic;
    for (int i = 0; i < nic; ++i) {
        s->icfg[i] = ics[i];
        s->irq[i].enabled = true;
    }
    s->n_task = ntc;
    for (int i = 0; i < ntc; ++i) {
        s->tcfg[i] = tcs[i];
        s->task[i].next_due = tcs[i].first;
    }
}

static bool sim_irq_active(const struct Sim *s, int id)
{
    for (int i = 0; i < s->top; ++i) if (s->stack[i].id == id) return true;
    return false;
}

static bool sim_primask(const struct Sim *s)
{
    for (int i = 0; i < s->n_task; ++i) if (s->task[i].crit_rem > 0) return true;
    return false;
}

/* 选出当前 pending 里优先级最高的（抢占值小 → 子优先级小 → id 小） */
static int sim_pick(const struct Sim *s)
{
    int best = -1;
    for (int i = 0; i < s->n_irq; ++i) {
        if (!s->irq[i].enabled || !s->irq[i].pending) continue;
        if (best < 0) { best = i; continue; }
        int pa = eff_pre(s->icfg[i].raw), pb = eff_pre(s->icfg[best].raw);
        if (pa < pb) best = i;
        else if (pa == pb && eff_sub(s->icfg[i].raw) < eff_sub(s->icfg[best].raw)) best = i;
    }
    return best;
}

static void sim_enter(struct Sim *s, int id)
{
    s->irq[id].pending = false;
    long long lat = s->t - s->irq[id].event_t + ENTRY_LAT;
    s->irq[id].enters++; s->irq[id].lat_sum += lat;
    if (lat > s->irq[id].lat_max) s->irq[id].lat_max = lat;
    if (s->top > 0) s->preempt_cnt++;
    s->stack[s->top].id = id;
    s->stack[s->top].rem = s->icfg[id].work;
    s->top++;
}

static void sim_exit_top(struct Sim *s)
{
    int id = s->stack[--s->top].id;
    if (s->icfg[id].exit_set_flag >= 0) s->flags[s->icfg[id].exit_set_flag] = true;
    if (s->icfg[id].exit_fire_irq >= 0) {
        int fid = s->icfg[id].exit_fire_irq;
        if (!s->irq[fid].pending && !sim_irq_active(s, fid)) {
            s->irq[fid].pending = true; s->irq[fid].event_t = s->t;
        }
    }
}

static void sim_step(struct Sim *s)
{
    s->t++;
    /* 自动周期事件（正在执行或已挂起时新事件被硬件语义吞掉：不重复置位） */
    for (int i = 0; i < s->n_irq; ++i) {
        if (s->icfg[i].period > 0 && s->t % s->icfg[i].period == 0 &&
            !s->irq[i].pending && !sim_irq_active(s, i)) {
            s->irq[i].pending = true; s->irq[i].event_t = s->t;
        }
    }
    /* 手动一次性事件 */
    for (int i = 0; i < s->n_fire; ++i) {
        if (s->fires[i].when == s->t && !s->irq[s->fires[i].id].pending &&
            !sim_irq_active(s, s->fires[i].id)) {
            s->irq[s->fires[i].id].pending = true;
            s->irq[s->fires[i].id].event_t = s->t;
        }
    }
    /* 饿死统计：pending 但没被服务的挂起时长 */
    for (int i = 0; i < s->n_irq; ++i) {
        if (s->irq[i].pending) {
            long long age = s->t - s->irq[i].event_t;
            if (age > s->irq[i].starve_peak) s->irq[i].starve_peak = age;
        }
    }
    /* 看门狗 */
    if (s->iwdg_timeout > 0 && s->t >= s->wdg_deadline) { s->reset_fired = true; return; }

    /* ---- NVIC 仲裁 ---- */
    if (!sim_primask(s) && s->top < MAX_STACK) {
        int cand = sim_pick(s);
        if (cand >= 0) {
            bool may = (s->top == 0) ||
                (eff_pre(s->icfg[cand].raw) < eff_pre(s->icfg[s->stack[s->top - 1].id].raw));
            if (may) {
                sim_enter(s, cand);
                s->irq[cand].busy_us++;
                s->stack[s->top - 1].rem--;
                if (s->stack[s->top - 1].rem <= 0) sim_exit_top(s);
                return; /* 本 µs 被 ISR 占用 */
            }
        }
    }
    /* ---- 正在执行的 ISR 继续跑 1 µs ---- */
    if (s->top > 0) {
        s->irq[s->stack[s->top - 1].id].busy_us++;
        s->stack[s->top - 1].rem--;
        if (s->stack[s->top - 1].rem <= 0) sim_exit_top(s);
        return;
    }
    /* ---- 主循环：1 µs 一片 ---- */
    int run = -1;
    for (int i = 0; i < s->n_task; ++i)
        if (s->task[i].rem > 0) { run = i; break; }
    if (run < 0) {
        for (int i = 0; i < s->n_task; ++i) {
            if (s->tcfg[i].gate_flag >= 0) {
                if (!s->flags[s->tcfg[i].gate_flag]) continue;
            } else if (s->tcfg[i].period > 0 && s->t < s->task[i].next_due) {
                continue;
            }
            s->task[i].rem = s->tcfg[i].cost;
            if (s->tcfg[i].critical) s->task[i].crit_rem = s->tcfg[i].cost;
            run = i; break;
        }
    }
    if (run >= 0) {
        s->task[run].rem--;
        if (s->task[run].crit_rem > 0) s->task[run].crit_rem--;
        if (s->task[run].rem == 0) {
            s->task[run].done_cnt++;
            if (s->tcfg[run].gate_flag >= 0) s->flags[s->tcfg[run].gate_flag] = false;
            if (s->tcfg[run].done_set_flag >= 0) s->flags[s->tcfg[run].done_set_flag] = true;
            if (s->tcfg[run].feeds_dog && s->iwdg_timeout > 0)
                s->wdg_deadline = s->t + s->iwdg_timeout;
            if (s->tcfg[run].period > 0) s->task[run].next_due = s->t + s->tcfg[run].period;
        }
    }
}

/* ──────────────────────────────────────────────────────────────────────────── */
/* 打印工具                                                                     */
/* ──────────────────────────────────────────────────────────────────────────── */
static void hr(const char *title)
{
    printf("\n");
    printf("════════════════════════════════════════════════════════════════════\n");
    printf("  %s\n", title);
    printf("════════════════════════════════════════════════════════════════════\n");
}

static void dump_irq_stats(const struct Sim *s)
{
    printf("   %-10s %-6s %-10s %-12s %-12s %s\n",
           "中断源", "进入", "平均延迟", "最坏延迟", "占用时间", "最长挂起");
    printf("   ──────────────────────────────────────────────────────────────\n");
    for (int i = 0; i < s->n_irq; ++i) {
        const struct IrqState *q = &s->irq[i];
        printf("   %-10s %-6lld %-9.1f  %-10lld  %-10lld  %lld\n",
               s->icfg[i].name, q->enters,
               q->enters > 0 ? (double)q->lat_sum / (double)q->enters : 0.0,
               q->lat_max, q->busy_us, q->starve_peak);
    }
    printf("   （延迟单位 µs；最长挂起 = 事件到来越久没被服务的峰值）\n");
    printf("   抢占发生次数：%lld\n", s->preempt_cnt);
}

static void dump_task_stats(const struct Sim *s)
{
    printf("   主循环任务表：\n");
    for (int i = 0; i < s->n_task; ++i) {
        printf("     - %-18s 完成 %lld 次", s->tcfg[i].name, s->task[i].done_cnt);
        if (s->tcfg[i].critical) printf("（执行期间关中断）");
        if (s->tcfg[i].feeds_dog) printf("（喂狗）");
        if (s->tcfg[i].gate_flag >= 0) printf("（延迟队列：等标志）");
        printf("\n");
    }
}

/* ──────────────────────────────────────────────────────────────────────────── */
/* 演示一：正常调度，看响应延迟从哪来                                           */
/* ──────────────────────────────────────────────────────────────────────────── */
static void demo_normal(void)
{
    hr("演示一：软件 NVIC 正常调度（跑 2100 tick，1 tick = 1 µs）");

    struct IrqCfg ics[3] = {
        /* name,        period, work, raw,     exit_set, exit_fire */
        {"TIM 1kHz", 1000, 25, 0b0010, -1, -1},   /* 抢占 0 子 2（最高抢占级） */
        {"UART RX",   150, 20, 0b1100, -1, -1},   /* 抢占 3 子 0 */
        {"ADC EOC",   300, 15, 0b1101, -1, -1},   /* 抢占 3 子 1 */
    };
    struct TaskCfg tcs[3] = {
        /* name,               first, period, cost, crit,  feed, gate, done */
        {"采样提交(临界区)",   999, 1000, 40, true,  false, -1, -1},
        {"状态上报",           700,  700, 60, false, false, -1, -1},
        {"喂狗",               800,  800,  3, false, true,  -1, -1},
    };

    struct Sim s;
    sim_setup(&s, ics, 3, tcs, 3, 2100, 0);
    printf("   优先级分组 = %d（%d 位抢占 × %d 位子优先级）\n",
           g_priority_group, g_priority_group, 4 - g_priority_group);
    printf("   TIM raw=0b0010→抢占0；UART raw=0b1100→抢占3子0；ADC raw=0b1101→抢占3子1\n");
    printf("   主循环 t=999 起跑一个 40µs 的关中断临界区「采样提交」。\n\n");

    while (!s.reset_fired && s.t < s.t_end) sim_step(&s);

    dump_irq_stats(&s);
    printf("\n");
    dump_task_stats(&s);
    printf("\n   师傅解读：\n");
    printf("   - TIM 在 t=1000 的事件，恰好落在临界区起跑后第 1µs，整段 40µs 窗口\n");
    printf("     都关着中断，只能等到 t=1039 才进 ISR：最坏延迟 40µs。\n");
    printf("     最坏响应延迟公式：临界区剩余 + 上级ISR尾巴 + 入栈时间，就这三项。\n");
    printf("   - t=300/600/... UART 和 ADC 同一拍到达：抢占级相同（都是3），\n");
    printf("     子优先级 0 < 1 → UART 先走，ADC 排队多等约 20µs。子优先级\n");
    printf("     只决定同拍排队顺序，永远不能互相抢占——所以抢占次数是 0。\n");
    printf("   - 想把 40µs 压下去：拆小临界区，或者先关中断拷数据、开中断再提交。\n");
    printf("     不信？把 ADC 的 raw 改成 0b0100（抢占1），它就能把 UART 拦腰砍断。\n");
}

/* ──────────────────────────────────────────────────────────────────────────── */
/* 演示二：高优先级 ISR 里 5ms 忙等 → 饿死 + 看门狗复位                         */
/* ──────────────────────────────────────────────────────────────────────────── */
static void demo_busywait(void)
{
    hr("演示二：事故现场——TIM 的 ISR 里忙等 5ms（周期才 1ms）");

    struct IrqCfg ics[3] = {
        {"TIM 1kHz", 1000, 5000, 0b0010, -1, -1},  /* 灾难：work=5000µs > period */
        {"UART RX",   150,   20, 0b1100, -1, -1},
        {"ADC EOC",   300,   15, 0b1101, -1, -1},
    };
    struct TaskCfg tcs[3] = {
        {"采样提交",  200,  200, 40, false, false, -1, -1},
        {"状态上报",  700,  700, 60, false, false, -1, -1},
        {"喂狗",     10000, 10000, 3, false, true,  -1, -1},
    };

    struct Sim s;
    sim_setup(&s, ics, 3, tcs, 3, 130000, 100000);  /* IWDG 标称 100ms */
    printf("   配置：TIM ISR 忙等 5000µs，但 1kHz 每 1000µs 就来一发；\n");
    printf("         IWDG：预分频 /4、重装值 1000 → 标称超时 100ms。\n\n");

    while (!s.reset_fired && s.t < s.t_end) sim_step(&s);

    dump_irq_stats(&s);
    printf("\n");
    dump_task_stats(&s);
    printf("\n");
    if (s.reset_fired) {
        printf("   ✗ t=%lld µs：IWDG 超时，芯片复位！\n", s.t);
    }
    printf("\n   师傅解读：\n");
    printf("   - 风暴（t=1000）之前 UART/ADC 还各跑了几次；风暴一开始就全灭了：\n");
    printf("     TIM ISR 要还 5ms 的债：t=1000 进去，t=5999 才出来，t=6000 下一发\n");
    printf("     立刻挂起、当拍二次进入——TIM 连占 99000µs 无缝衔接，\n");
    printf("     主循环一步都没跑到，喂狗 0 次（表里那几次都是风暴前跑完的）。\n");
    printf("   - UART/ADC 抢占级比 TIM 低，NVIC 的规则是「低的给高的让路」，\n");
    printf("     于是挂起时间一路涨到约 10 万µs——这就叫饿死（starvation）。\n");
    printf("   - 真机现象：串口偶发不响应，几十毫秒后整机重启。查 RCC->CSR 的\n");
    printf("     IWDGRSTF 复位标志就能确诊是看门狗咬的，不是玄学死机。\n");
}

/* ──────────────────────────────────────────────────────────────────────────── */
/* 演示三：修正版——ISR 只置标志，重活进延迟队列                                 */
/* ──────────────────────────────────────────────────────────────────────────── */
static void demo_fixed(void)
{
    hr("演示三：修正版——ISR 只干 2µs，重活扔给主循环延迟队列");

    struct IrqCfg ics[3] = {
        {"TIM 1kHz", 1000, 2,  0b0010, 0, -1},    /* 置 flags[0]，重活交给任务 */
        {"UART RX",   150, 20, 0b1100, -1, -1},
        {"ADC EOC",   300, 15, 0b1101, -1, -1},
    };
    struct TaskCfg tcs[4] = {
        /* name,                first, period, cost, crit,  feed, gate, done */
        {"TIM重活(延迟队列)",   0,     0,    300, false, false,  0, -1},
        {"采样提交",          200,   200,   40, false, false, -1, -1},
        {"状态上报",          700,   700,   60, false, false, -1, -1},
        {"喂狗",            10000, 10000,    3, false, true,  -1, -1},
    };

    struct Sim s;
    sim_setup(&s, ics, 3, tcs, 4, 100000, 100000);
    printf("   ISR 里只清挂起+置标志（2µs 出来），「TIM 重活」作为主循环任务，\n");
    printf("   攒到标志就跑 300µs，可被 UART/ADC 打断后续跑。\n\n");

    while (!s.reset_fired && s.t < s.t_end) sim_step(&s);

    dump_irq_stats(&s);
    printf("\n");
    dump_task_stats(&s);
    printf("\n");
    if (s.reset_fired) {
        printf("   ✗ 还是复位的？检查主循环吞吐是不是超了 CPU 预算。\n");
    } else {
        printf("   ✓ 跑满 %lld µs 没有复位：中断全部响应，狗也被正常喂到。\n", s.t);
    }
    printf("\n   师傅解读：\n");
    printf("   - ISR 从 5000µs 缩到 2µs，CPU 立刻还给主循环和低优先级中断。\n");
    printf("   - 代价是重活晚几十µs 才被处理——对 1kHz 定时器来说无所谓。\n");
    printf("     真要硬实时，再谈 DMA 和 RTOS，别拿 ISR 硬扛。\n");
    printf("   - 延迟队列两个要点：标志要用 volatile；本模型跑完才清标志，\n");
    printf("     相当于「一次只攒一单」，真机上要么这样，要么上环形队列。\n");
}

/* ──────────────────────────────────────────────────────────────────────────── */
/* 演示四：优先级反转（裸机版）与治法                                           */
/* ──────────────────────────────────────────────────────────────────────────── */
static void inversion_case(const char *tag, int low_raw, long long t_end)
{
    /* 场景：REQ（抢占0）收到采集请求，2µs 后甩给 LOW 异步干 600µs，干完置
     *       flags[0] 表示数据就绪；MED（抢占1）在 LOW 干到一半时触发 800µs。 */
    struct IrqCfg ics[3] = {
        /* name,        period, work, raw,      exit_set, exit_fire */
        {"REQ 高",   0,    2, 0b0001, -1,  1},   /* 退出时触发 LOW */
        {"LOW 干活", 0,  600, low_raw,  0, -1},   /* 完成时置 flags[0] */
        {"MED 中",   0,  800, 0b0100, -1, -1},
    };
    struct TaskCfg tcs[1] = {
        {"空转", 1, 10, 1, false, false, -1, -1},
    };

    struct Sim s;
    sim_setup(&s, ics, 3, tcs, 1, t_end, 0);
    s.fires[0].when = 300; s.fires[0].id = 0;   /* t=300 REQ 到达 */
    s.fires[1].when = 400; s.fires[1].id = 2;   /* t=400 MED 插队（LOW 才干了 90 多µs） */
    s.n_fire = 2;

    long long req_done = -1;
    while (!s.reset_fired && s.t < s.t_end) {
        sim_step(&s);
        if (s.flags[0]) { req_done = s.t; break; }
    }

    printf("   %s：REQ 抢占级 %d｜LOW 抢占级 %d｜MED 抢占级 %d\n",
           tag, eff_pre(ics[0].raw), eff_pre(ics[1].raw), eff_pre(ics[2].raw));
    if (req_done >= 0) {
        printf("     请求 t=300 发出，数据就绪 t=%lld → 完成延迟 %lld µs（LOW 本体只要 600µs）\n",
               req_done, req_done - 300);
    } else {
        printf("     仿真窗口内活没跑完（检查 t_end）\n");
    }
}

static void demo_inversion(void)
{
    hr("演示四：优先级反转——高优先级被「中优先级」间接拖死");

    printf("   场景：REQ（抢占0）收到采集请求，甩给 LOW（抢占3）异步干 600µs；\n");
    printf("         MED（抢占1）在 LOW 干到 98µs 时触发，要占 800µs。\n");
    printf("         NVIC 只认抢占级：MED(1) 比 LOW(3) 高 → 合法插队。\n\n");
    inversion_case("方案A 原始优先级", 0b1110, 4000);
    printf("\n");
    inversion_case("方案B 把 LOW 提到抢占0", 0b0010, 4000);
    printf("\n   师傅解读：\n");
    printf("   - 方案A：REQ 自己 2µs 就跑完了，但它依赖的 LOW 被 MED 压了一头，\n");
    printf("     数据要等约 1400µs 才齐——从 REQ 的看，是「替我干活的人」变慢了，\n");
    printf("     这就是优先级反转的裸机形态。\n");
    printf("   - 方案B 只改一个数：给【干活的人】排高优先级（MED 插不进来），\n");
    printf("     延迟回到约 600µs。RTOS 里这叫「优先级天花板/继承」，裸机没这\n");
    printf("     机制，只能你排优先级时替它想好：谁共享资源，谁跟着资源走。\n");
    printf("   - 排优先级口诀：不是「谁的事件吓人谁优先级高」，而是\n");
    printf("     「谁在关键路径上、谁拖累了别人，谁优先级高」。\n");
}

/* ──────────────────────────────────────────────────────────────────────────── */
/* 演示五：IWDG 超时计算器                                                      */
/* ──────────────────────────────────────────────────────────────────────────── */
static void demo_iwdg_calc(void)
{
    hr("演示五：IWDG 超时计算器（LSI 标称 40 kHz，重装值 12 位 ≤ 4095）");

    printf("   超时时间（秒）= 预分频 × (重装值 + 1) ÷ 40000；下表数值已换算成毫秒\n\n");
    printf("   %-8s %-8s %-14s %s\n", "预分频", "重装值", "标称超时(ms)", "备注");
    printf("   ──────────────────────────────────────────────────────────────\n");

    struct Row { int ps; int rl; const char *note; };
    struct Row rows[5] = {
        {  4,    0, "重装值 0：只数 1 拍（/4 档 = 0.1ms），几乎立刻复位" },
        {  4,  999, "100 ms：(999+1)×4/40000" },
        {  4, 4000, "400.1 ms：(4000+1)×4/40000，常见安全档" },
        { 32, 4000, "3.2 s：宽松档，给慢流程留余地" },
        {256, 4095, "26.2 s：本表最长 = 256×4096/40000" },
    };
    for (int i = 0; i < 5; ++i) {
        double ms = (double)rows[i].ps * (double)(rows[i].rl + 1) / 40.0;
        printf("   /%-7d %-8d %-13.1f %s\n", rows[i].ps, rows[i].rl, ms, rows[i].note);
    }

    printf("\n   陷阱题：LSI 的真实频率在 30~60 kHz 之间漂（手册写着 −25%%~+50%%），\n");
    printf("   标称 400ms 的狗，实际超时 = 16000 拍 ÷ 实际频率，落在 267ms ~ 533ms。\n");
    printf("   所以喂狗窗口要按最坏 267ms 留余量：主循环最坏路径 150ms，\n");
    printf("   就别选 200ms 档，选 400ms 档，让「最快咬的那次」也咬不到活人。\n");
    printf("   还有：喂狗只准放在「所有关键任务都过了一遍」之后；\n");
    printf("   放在 ISR 里喂、或进循环先喂再干活，都等于给僵尸续命。\n");
}

/* ──────────────────────────────────────────────────────────────────────────── */
int main(void)
{
    printf("╔══════════════════════════════════════════════════════════════════════╗\n");
    printf("║  04 · 软件 NVIC：调度、饿死、反转、看门狗（无硬件仿真，1µs 一步）    ║\n");
    printf("╚══════════════════════════════════════════════════════════════════════╝\n");

    demo_normal();
    demo_busywait();
    demo_fixed();
    demo_inversion();
    demo_iwdg_calc();

    printf("\n════════════════════════════════════════════════════════════════════\n");
    printf("  收工。带走一句话：中断是「插队的特权」，特权越大，进去越要快出来。\n");
    printf("════════════════════════════════════════════════════════════════════\n");
    return 0;
}
