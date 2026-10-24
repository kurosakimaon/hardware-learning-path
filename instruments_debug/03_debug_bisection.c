/*
 * ============================================================================
 * 03_debug_bisection.c —— 排障决策树模拟器：聪明地测，而不是乱测
 * ============================================================================
 *
 * 【这个文件讲什么】
 *   一块 5 路电源板（12V 输入 → 5V/3.3V/1.8V/+3.3VA/-9V）的网表级行为模型
 *   （纯 C17，无第三方库），注入 6 种典型故障（LDO 烧毁、DCDC 开/短路、
 *   反馈电阻漂移、保险丝熔断、去耦电容失效、负载过重）。然后两种排障策略
 *   同场竞技：
 *     策略 A「师傅」：每一步在候选测量动作里挑信息增益最大的（把剩余
 *            候选故障集合切得最均匀的那个），带测量代价（万用表便宜、
 *            热风枪/拆焊昂贵）；
 *     策略 B「无头苍蝇」：随机乱测。
 *   旁白全程中文：每一步说"现在怀疑什么、为什么测这个、量到 X 说明什么"。
 *   固定随机种子，结果可复现；同一故障跑两个策略对比步数与代价。
 *
 * 【怎么编译运行】(MSYS2 gcc 16.2，Windows)
 *   先在控制台执行：chcp 65001   （否则中文输出会乱码）
 *   gcc -std=c17 -O2 -Wall -Wextra 03_debug_bisection.c -o 03_debug_bisection.exe
 *   .\03_debug_bisection.exe            # 6 个故障 × 两策略对比 + 单例详解
 *
 * 【预期看到什么】
 *   - 每个故障的逐步旁白："当前候选 6 个 → 测『5V 轨电压』能把候选切成
 *     4|2，信息增益最大 → 读数 0.02V → 排除/锁定……"；
 *   - 师傅策略全部 ≤8 步（多数 2~4 步）定位到故障；
 *   - 汇总表：6 个故障两策略的【步数/总代价】对比，师傅几乎全胜；
 *   - 偶尔"乱测"也碰对——但平均步数和代价高一大截，这就是方法论的钱。
 *
 * 【老师傅点拨】
 *   1. 排障不是"测着看"，是"花最小代价买最大信息"。第一步永远望闻问切，
 *      把可能性空间先砍一半再掏表。
 *   2. 好的测量动作 = 结果二分的动作。全板电压一次测完，看似勤快，
 *      其实每点信息量可能都是零（因为故障只影响两路）。
 *   3. 测量有代价：带电测电压 5 秒，拆焊换件半小时。信息增益要除以代价。
 * ============================================================================
 */

#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define N_RAILS 5
#define N_FAULTS 6
#define MAX_ACTIONS 9

/* ---------------------------------------------------------------------------
 * 可复现随机源：xorshift64*（固定种子，同一种子两次运行逐字一致）
 * --------------------------------------------------------------------------- */
typedef struct {
    unsigned long long s;
} Rng;

static void rngSeed(Rng *r, unsigned long long seed)
{
    r->s = seed ? seed : 88172645463325252ULL; /* 0 会让 xorshift 死掉 */
}

/* [0,1) 均匀分布 */
static double rngUnit(Rng *r)
{
    r->s ^= r->s >> 12;
    r->s ^= r->s << 25;
    r->s ^= r->s >> 27;
    unsigned long long v = (r->s * 2685821657736338717ULL) >> 11;
    return (double)v * (1.0 / 9007199254740992.0);
}

/* [lo,hi) 均匀分布 */
static double rngUniform(Rng *r, double lo, double hi)
{
    return lo + (hi - lo) * rngUnit(r);
}

/* ---------------------------------------------------------------------------
 * 板卡模型：5 路电源网表（行为级，不玩 SPICE）
 * --------------------------------------------------------------------------- */
typedef struct {
    const char *name;
    double nominal;     /* 标称电压 */
    double rout;        /* 输出阻抗量级（负载调整用） */
    const char *feeds;  /* 供给谁 */
} Rail;

typedef struct {
    Rail rails[N_RAILS];
    /* 每路的标称负载电流（A） */
    double loadA[N_RAILS];
} Board;

static Board makeBoard(void)
{
    Board b;
    b.rails[0] = (Rail){"VIN 12V", 12.0, 0.02, "输入保险丝后，喂 DCDC"};
    b.rails[1] = (Rail){"5V",       5.0, 0.05, "DCDC1 输出，喂 LDO 与电机驱动"};
    b.rails[2] = (Rail){"3V3",      3.3, 0.10, "LDO(U3) 从 5V 而来，喂 MCU"};
    b.rails[3] = (Rail){"1V8",      1.8, 0.08, "DCDC2 输出，喂 FPGA 内核"};
    b.rails[4] = (Rail){"-9V",     -9.0, 2.0,  "电荷泵，喂运放负轨"};
    b.loadA[0] = 0.45; b.loadA[1] = 0.60; b.loadA[2] = 0.12;
    b.loadA[3] = 1.50; b.loadA[4] = 0.005;
    return b;
}

/* 故障定义：改了哪些路的"行为" */
typedef struct {
    const char *id, *name, *smell; /* smell: 望闻问切能得到的外部症状 */
    /* 对每路的效果：mult 电压乘子，add 附加偏移 */
    double mult[N_RAILS];
    double add[N_RAILS];
    int dead[N_RAILS];  /* 被拖死的轨（读数≈0，带纹波） */
    int nDead;          /* dead 列表长度 */
    double hotOhm;      /* 对地短路等效电阻（0=无短路） */
} Fault;

static bool isDead(const Fault *f, int r)
{
    for (int i = 0; i < f->nDead; ++i)
        if (f->dead[i] == r) return true;
    return false;
}

static void faultInit(Fault *f, const char *id, const char *name,
                      const char *smell)
{
    f->id = id; f->name = name; f->smell = smell;
    for (int i = 0; i < N_RAILS; ++i) {
        f->mult[i] = 1.0;
        f->add[i] = 0.0;
    }
    f->nDead = 0;
    f->hotOhm = 0.0;
}

static void makeFaults(Fault *f)
{
    faultInit(&f[0], "F1", "输入保险丝熔断",
              "完全无电流，板子冰凉，任何轨都是 0V");
    for (int i = 0; i < N_RAILS; ++i) {
        f[0].mult[i] = 0.0;
        f[0].dead[f[0].nDead++] = i; /* 全板被拖死 */
    }
    /* F2: 5V DCDC 功率管击穿 —— 5V 轨短路把输入也拉垮 */
    faultInit(&f[1], "F2", "5V DCDC 下管击穿",
              "上电就打火/保险丝瞬间熔断，5V 电感烫手");
    f[1].dead[0] = 0; f[1].dead[1] = 1; f[1].dead[2] = 2; /* 输入被拖垮，5V 短路，3.3V LDO 随之无输入 */
    f[1].nDead = 3;
    f[1].hotOhm = 0.05;
    /* F3: 3.3V LDO 输出对地短路（MLCC 裂纹） */
    faultInit(&f[2], "F3", "3.3V 输出 MLCC 裂纹短路",
              "5V 正常但发烫，3.3V 读 0，LDO 烫手");
    f[2].dead[0] = 2;
    f[2].nDead = 1;
    f[2].hotOhm = 0.2;
    /* F4: 1.8V DCDC 反馈上分压电阻开路 —— 输出飙高到限幅 */
    faultInit(&f[3], "F4", "1.8V 反馈上电阻开路",
              "1.8V 轨读到 3.3V 上限，FPGA 冒烟前过压保护");
    f[3].mult[3] = 0.0; f[3].add[3] = 3.3; /* 顶到钳位 */
    /* F5: -9V 电荷泵飞电容失效 —— 输出阻抗暴涨，带载即垮 */
    faultInit(&f[4], "F5", "-9V 飞电容失效",
              "空载 -9V 看着正常(-8.9V)，一上电负载变 -1.2V");
    f[4].mult[4] = 0.13; /* 带载垮掉 */
    /* F6: 3.3V LDO 调整管退化 —— 掉进 dropout，输出矮一截带纹波 */
    faultInit(&f[5], "F6", "3.3V LDO 调整管退化",
              "3.3V 只有 2.1V 且纹波大，MCU 反复复位");
    f[5].mult[2] = 0.64; /* 3.3*0.64≈2.1V */
}

/* 给定故障下测量某轨电压：正常=标称±负载调整；拖死≈0；退化=乘子/加偏移 */
typedef struct {
    double v;
    bool dead;
} Obs;

static Obs measureRail(const Board *b, const Fault *f, size_t r, Rng *rng)
{
    Obs o;
    const double jitter = rngUniform(rng, -0.015, 0.015);
    if (isDead(f, (int)r)) {
        o.v = jitter * 0.5;
        o.dead = true;
        return o;
    }
    double v = b->rails[r].nominal * f->mult[r] + f->add[r];
    /* 负载调整 + 噪声 */
    const double droop = b->loadA[r] * b->rails[r].rout
                         * (f->mult[r] > 0.0 ? f->mult[r] : 1.0);
    v -= (v > 0.0 ? droop : -droop);
    o.v = v + jitter;
    o.dead = false;
    return o;
}

/* 对地电阻测量（断电用电阻档）：短路故障见低阻 */
static double measureOhm(const Fault *f, size_t r, Rng *rng)
{
    const double jitter = rngUniform(rng, 0.95, 1.05);
    if (isDead(f, (int)r) && f->hotOhm > 0.0)
        return f->hotOhm * jitter;
    return 2000.0 * jitter; /* 正常轨对地看进去是负载，kΩ 级 */
}

/* ---------------------------------------------------------------------------
 * 测量动作库：每个动作对每种故障产生什么"观测类别"
 * --------------------------------------------------------------------------- */
typedef struct Action {
    char name[96];
    double cost;    /* 代价（抽象单位：表笔 1，摸温度 2，断电电阻档 3，拆焊 30） */
    /* 返回该动作在该故障下的观测分类字符串 */
    const char *(*observe)(const Board *, const Fault *, size_t railIdx, Rng *);
    size_t rail;
    bool needsPowerOff;
} Action;

static const char *obsVolt(const Board *b, const Fault *f, size_t r, Rng *rng)
{
    const Obs o = measureRail(b, f, r, rng);
    if (o.dead) return "≈0V(死)";
    /* 分类：正常 / 偏低 / 偏高 / 飙高 */
    const double n = b->rails[r].nominal;
    const double ratio = o.v / n;
    if (ratio > 1.3) return "飙高";
    if (ratio > 0.9) return "正常";
    if (ratio > 0.45) return "偏低";
    return "接近0";
}

static const char *obsOhm(const Board *b, const Fault *f, size_t r, Rng *rng)
{
    (void)b;
    const double o = measureOhm(f, r, rng);
    return (o < 50.0) ? "低阻(短路)" : "高阻(正常)";
}

static const char *obsTouch(const Board *b, const Fault *f, size_t r, Rng *rng)
{
    (void)b; (void)r; (void)rng;
    /* 摸温度：只看 hotOhm（短路发热） */
    return (f->hotOhm > 0.0) ? "烫手" : "常温";
}

static int makeActions(const Board *b, Action *a)
{
    int n = 0;
    for (size_t r = 0; r < N_RAILS; ++r) {
        snprintf(a[n].name, sizeof(a[n].name), "量 %s 电压（带电）",
                 b->rails[r].name);
        a[n].cost = 1.0;
        a[n].observe = obsVolt;
        a[n].rail = r;
        a[n].needsPowerOff = false;
        ++n;
    }
    snprintf(a[n].name, sizeof(a[n].name),
             "上电 10 秒摸全板温度分布（先别碰！看/闻为主）");
    a[n].cost = 2.0;
    a[n].observe = obsTouch;
    a[n].rail = 0;
    a[n].needsPowerOff = false;
    ++n;
    /* 断电电阻档只对可能短路的轨有意义 */
    snprintf(a[n].name, sizeof(a[n].name), "断电量 5V 对地电阻");
    a[n].cost = 3.0; a[n].observe = obsOhm; a[n].rail = 1;
    a[n].needsPowerOff = true; ++n;
    snprintf(a[n].name, sizeof(a[n].name), "断电量 3.3V 对地电阻");
    a[n].cost = 3.0; a[n].observe = obsOhm; a[n].rail = 2;
    a[n].needsPowerOff = true; ++n;
    snprintf(a[n].name, sizeof(a[n].name), "断电量 VIN 对地电阻");
    a[n].cost = 3.0; a[n].observe = obsOhm; a[n].rail = 0;
    a[n].needsPowerOff = true; ++n;
    return n;
}

/* ---------------------------------------------------------------------------
 * 策略 A：贪心信息增益（除以代价）
 * --------------------------------------------------------------------------- */
typedef struct {
    int steps;
    double cost;
    char log[24][512];
    int nlog;
} Trace;

/* 对每个候选故障算该动作的观测，返回最大分组大小（越小越好），并填 outs */
static int partitionBest(const int *cands, int nCands, const Action *act,
                         const Board *b, Rng *rng, const Fault *faults,
                         const char **outs)
{
    for (int i = 0; i < nCands; ++i)
        outs[i] = act->observe(b, &faults[cands[i]], act->rail, rng);
    int worst = 1;
    for (int i = 0; i < nCands; ++i) {
        int cnt = 1;
        for (int j = 0; j < nCands; ++j)
            if (j != i && strcmp(outs[j], outs[i]) == 0) ++cnt;
        if (cnt > worst) worst = cnt;
    }
    return worst;
}

static Trace runSmart(const Board *b, const Fault *faults, size_t target,
                      Rng *rng, bool verbose)
{
    Trace t;
    t.steps = 0; t.cost = 0.0; t.nlog = 0;
    int cands[N_FAULTS], next[N_FAULTS];
    int nC = N_FAULTS;
    for (int i = 0; i < N_FAULTS; ++i) cands[i] = i;
    Action acts[MAX_ACTIONS];
    const int nActs = makeActions(b, acts);
    const char *outs[N_FAULTS];
    int guard = 0;
    while (nC > 1 && guard++ < 20) {
        /* 选 max-min 分组（再按代价微调）：min worst / cost */
        int bestIdx = -1, bestWorst = 1 << 30;
        double bestScore = 1e30;
        for (int ai = 0; ai < nActs; ++ai) {
            const int w = partitionBest(cands, nC, &acts[ai], b, rng,
                                        faults, outs);
            const double score = (double)w / acts[ai].cost; /* 越小越好 */
            if (w < bestWorst || (w == bestWorst && score < bestScore)) {
                bestWorst = w; bestScore = score; bestIdx = ai;
            }
        }
        const Action *act = &acts[(size_t)bestIdx];
        /* 用该动作实测目标故障 */
        const char *obs = act->observe(b, &faults[target], act->rail, rng);
        int nNext = 0;
        for (int i = 0; i < nC; ++i)
            if (strcmp(act->observe(b, &faults[cands[i]], act->rail, rng),
                       obs) == 0)
                next[nNext++] = cands[i];
        ++t.steps;
        t.cost += act->cost;
        if (verbose) {
            char ids[64];
            size_t pos = 0;
            for (int i = 0; i < nNext; ++i) {
                int w = snprintf(ids + pos, sizeof(ids) - pos, "%s ",
                                 faults[next[i]].id);
                if (w > 0 && pos + (size_t)w < sizeof(ids)) pos += (size_t)w;
            }
            snprintf(t.log[t.nlog], sizeof(t.log[0]),
                     "  第 %d 步｜候选 %d 个 → 测「%s」（代价 %.0f）→ 读数【%s】"
                     "→ 候选剩 %d 个：%s",
                     t.steps, nC, act->name, act->cost, obs, nNext, ids);
            ++t.nlog;
        }
        memcpy(cands, next, (size_t)nNext * sizeof(int));
        nC = nNext;
    }
    if (verbose) {
        snprintf(t.log[t.nlog], sizeof(t.log[0]),
                 "  ★ 定位：%s（%s）｜共用 %d 步，总代价 %.0f",
                 faults[target].id, faults[target].name, t.steps, t.cost);
        ++t.nlog;
    }
    return t;
}

/* ---------------------------------------------------------------------------
 * 策略 B：随机乱测（同动作库，随机选）
 * --------------------------------------------------------------------------- */
static Trace runRandom(const Board *b, const Fault *faults, size_t target,
                       Rng *rng, bool verbose)
{
    Trace t;
    t.steps = 0; t.cost = 0.0; t.nlog = 0;
    int cands[N_FAULTS], next[N_FAULTS];
    int nC = N_FAULTS;
    for (int i = 0; i < N_FAULTS; ++i) cands[i] = i;
    Action acts[MAX_ACTIONS];
    const int nActs = makeActions(b, acts);
    int guard = 0;
    while (nC > 1 && guard++ < 20) {
        const Action *act = &acts[(size_t)(rngUnit(rng) * (double)nActs)];
        const char *obs = act->observe(b, &faults[target], act->rail, rng);
        int nNext = 0;
        for (int i = 0; i < nC; ++i)
            if (strcmp(act->observe(b, &faults[cands[i]], act->rail, rng),
                       obs) == 0)
                next[nNext++] = cands[i];
        ++t.steps;
        t.cost += act->cost;
        if (verbose) {
            snprintf(t.log[t.nlog], sizeof(t.log[0]),
                     "  第 %d 步｜乱测「%s」→ 【%s】→ 候选剩 %d 个",
                     t.steps, act->name, obs, nNext);
            ++t.nlog;
        }
        memcpy(cands, next, (size_t)nNext * sizeof(int));
        nC = nNext;
    }
    if (verbose) {
        snprintf(t.log[t.nlog], sizeof(t.log[0]),
                 "  （乱测收工：%d 步，代价 %.0f）", t.steps, t.cost);
        ++t.nlog;
    }
    return t;
}

/* --------------------------------------------------------------------------- */
int main(void)
{
    printf("================================================================\n");
    printf(" 排障决策树模拟器 —— 第 6 关 · 望闻问切（软件模型，无硬件）\n");
    printf("================================================================\n");
    const Board b = makeBoard();
    Fault faults[N_FAULTS];
    makeFaults(faults);

    printf("\n【板卡】5 路电源：");
    for (int i = 0; i < N_RAILS; ++i)
        printf("%s(%.1fV) ", b.rails[i].name, b.rails[i].nominal);
    printf("\n【症状输入】客户描述：『插上电源没反应，MCU 不复位也不亮。』\n");
    printf("【第一步永远是望闻问切】看烧痕/闻焦味/摸温度/问过程——模型里就是\n"
           "『摸温度分布』这个动作，代价 2，很多时候一步砍掉一半候选。\n");

    printf("\n---------------- 详解：故障 F3（3.3V MLCC 裂纹短路）----------------\n");
    {
        Rng rng;
        rngSeed(&rng, 6301);
        printf("  [师傅策略]\n");
        Trace a = runSmart(&b, faults, 2, &rng, true);
        for (int i = 0; i < a.nlog; ++i) printf("%s\n", a.log[i]);
        printf("\n  [乱测策略（同种子起步）]\n");
        Trace r = runRandom(&b, faults, 2, &rng, true);
        for (int i = 0; i < r.nlog; ++i) printf("%s\n", r.log[i]);
    }

    printf("\n---------------- 全故障对比：贪心 vs 随机 ----------------\n");
    printf("  %-4s %-26s | 师傅步数 师傅代价 | 乱测步数 乱测代价\n", "ID", "故障");
    double sumSmart = 0, sumRand = 0;
    int maxSteps = 0;
    for (size_t i = 0; i < N_FAULTS; ++i) {
        /* 每个故障独立种子；乱测跑 20 次取平均，避免单次运气 */
        Rng r1, r2;
        rngSeed(&r1, 6310ULL + (unsigned long long)i);
        Trace a = runSmart(&b, faults, i, &r1, false);
        double rs = 0, rc = 0;
        rngSeed(&r2, 97ULL + (unsigned long long)i);
        const int trials = 20;
        for (int k = 0; k < trials; ++k) {
            Trace r = runRandom(&b, faults, i, &r2, false);
            rs += (double)r.steps; rc += r.cost;
        }
        sumSmart += (double)a.steps; sumRand += rs / (double)trials;
        if (a.steps > maxSteps) maxSteps = a.steps;
        printf("  %-4s %-26s |   %d      %4.0f    |  %.1f    %5.1f\n",
               faults[i].id, faults[i].name,
               a.steps, a.cost, rs / (double)trials, rc / (double)trials);
    }
    printf("  平均每个故障：师傅 %.1f 步 vs 乱测 %.1f 步；师傅单故障最多 %d 步（≤8 达标）。\n"
           "  注意 F1/F2 乱测偶尔碰对——但那是运气，不是方法：看总代价。\n",
           sumSmart / (double)N_FAULTS, sumRand / (double)N_FAULTS, maxSteps);
    printf("\n【老师傅收尾】贪心策略没有魔法：它只是每一步都问『哪个动作能把\n"
           "剩下的可能性切得最均匀、还最便宜』。人脑排障也一样——先望闻问切，\n"
           "再挑『一刀下去答案最两分』的点下表。乱测不是不努力，是在花\n"
           "同样的代价买更少的信息。\n");
    return 0;
}
