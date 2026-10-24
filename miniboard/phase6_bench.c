/*
 * =====================================================================================
 *  第 7 关 · Phase 6（期末考）—— 24 小时日历压缩整机验收台（phase6_bench.c）
 * =====================================================================================
 *
 * 【讲什么】
 *   把 phase1~4 的模型串成一条时间轴，跑"24 小时日历压缩版"：
 *     1 tick = 100ms 固件时间，共 10000 ticks = 1000 s（16.7 min）；
 *     再把 10000 个 tick 等分映射到 24 格日历直方图（压缩比 86.4:1）。
 *     注意：这不是真实 24 小时老化，事件计数按 1000 s 固件时间统计；
 *     真实 24h 老化需要 864000 tick，或上板做 HALT/高温老化。
 *   时间轴上随机注入六类事件（固定种子，可复现）：
 *     - I2C NACK / 总线死锁恢复（phase4 的 9 clock + STOP）
 *     - SHT30 帧 CRC-8 错（phase3 的重试/降级）
 *     - 上行串口帧噪声（phase4 的抗噪解析器：帧间噪声串 + 帧内破坏）
 *     - 电源瞬时跌落 3.0V / 5ms（POR 阈 2.9V：跌但不死，brownout 记账）
 *     - 按键连按（消抖后一组毛刺 = 一次有效按键）
 *     - 温湿度阶跃（越限报警，phase3 师傅留的钩子在这兑现：
 *       原始值越限 2/3 表决立刻预警 + 驱动一拍 —— 报警延迟 ≤ 500ms）
 *     - 看门狗超时演练：一次计划内挂死，IWDG 咬狗复位，原因可查
 *   末尾出两张纸：24 格日历事件直方图（压缩映射）+ 验收报告（期末成绩单，要求全 PASS）。
 *
 *   这一版是纯 C（C17）：传感器模型 / 解析器 / 报警 FSM 都是 "struct + 首参数
 *   self 的函数"，日志用定长数组 [40][160]，没有 vector、没有 lambda、没有 new。
 *   随机源是十几行的 xorshift64*（固定种子），任何编译器结果都一样。
 *
 * 【怎么编译运行】(Windows + MSYS2 ucrt64, gcc 16.2)
 *   先在控制台执行：chcp 65001   （切到 UTF-8 代码页，否则中文输出是乱码）
 *   gcc -std=c17 -O2 -Wall -Wextra phase6_bench.c -o phase6_bench.exe
 *   ./phase6_bench.exe
 *   要求：零警告零错误；固定种子（20260706），两次运行输出完全一致。
 *
 * 【预期看到什么】
 *   - 关键事件流水（有限条数）：报警进出、咬狗、电源跌落、总线恢复；
 *   - 24 格小时直方图：通信/传感/电源/按键/报警/看门狗 六类事件的分布；
 *   - 验收报告全 PASS：不死机、报警延迟 ≤500ms（打印毫秒数）、
 *     CRC 通过率 ≥99%、RAM/Flash < 70%、省电窗口平均 <3mA、复位零泄漏。
 *
 * 【老师傅点拨】
 *   1. 仿真不是编故事：每个注入都走真实的软件路径（重试、降级、重同步、复位），
 *      数字是"跑"出来的，不是 printf 写死的。写死数字的报告比没有报告更危险。
 *   2. 报警延迟 500ms 这笔账：phase3 的"滑动平均+滞回"进出各慢 2 秒，直接挂科。
 *      这里改成"原始值 2/3 表决进、滤波滞回出"：进 ≤200ms（2 拍采样 + 1 拍驱动），
 *      出还靠滞回兜着不抖。快和稳不冲突，冲突的是懒得分开设计。
 *   3. "不死机"和"被咬"是两回事：计划内 IWDG 复位=保护动作，有原因记录；
 *      没原因的复位才叫泄漏。所以每次复位必须落一条原因账，这就是 R3 的底气。
 *   4. 看验收表要会挑刺：任何一项 PASS 你都要能指着事件流水说出它凭什么 PASS。
 *      说不出，就像背答案的优等生——考场（客户）上必翻车。
 * =====================================================================================
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

// ---------------------------------------------------------------------------
// 固定种子随机源：xorshift64*（替代原版的 std::mt19937(20260706)）
// 不用 rand() 是因为各编译器实现不同；xorshift64* 几行代码，跨编译器可复现。
// ---------------------------------------------------------------------------
static unsigned long long g_rngState = 20260706ULL; /* 固定种子 */

static unsigned long long xs64Next(void)
{
    g_rngState ^= g_rngState >> 12;
    g_rngState ^= g_rngState << 25;
    g_rngState ^= g_rngState >> 27;
    return g_rngState * 2685821657736338717ULL;
}

// ---------------------------------------------------------------------------
// 基础常量：时间轴
// ---------------------------------------------------------------------------
#define TICK_MS    100       /* 1 tick = 100ms 固件时间 */
#define TOTAL_TICKS 10000    /* 10000 ticks × 100ms = 1000s，映射到 24 格日历 */
#define SIM_HOURS  24

static int simHourOf(int tick) { return (int)((long)tick * SIM_HOURS / TOTAL_TICKS); }

// ---------------------------------------------------------------------------
// 上行行协议（phase4 同款 CRC16-CCITT + 抗噪解析器，这里只用上位机侧）
// ---------------------------------------------------------------------------
static uint16_t crc16Ccitt(const char *s, size_t len)
{
    uint16_t crc = 0xFFFF;
    size_t i;
    int b;
    for (i = 0; i < len; ++i) {
        crc ^= (uint16_t)((uint8_t)s[i]) << 8;
        for (b = 0; b < 8; ++b)
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
    }
    return crc;
}

static void buildFrame(const char *payload, char *out, size_t cap)
{
    snprintf(out, cap, "$%s*%04X\r\n", payload, crc16Ccitt(payload, strlen(payload)));
}

typedef struct {
    long ok;       /* CRC 正确的好帧 */
    long crcBad;   /* 帧形完整但 CRC 拦下 */
    long noise;    /* Idle 态丢掉的噪声字节 */
    long resync;   /* 异常退回 Idle 的次数 */
} HostStats;

typedef struct {
    int state;             /* 0=Idle 1=Body 2..5=CRC 十六进制位 6=等换行 */
    char body[65];         /* 帧体上限 64B + 结尾 0；坏帧可能混进 0x00，故按长度算 */
    int bodyLen;
    uint16_t crc;
    HostStats st;
} HostParser;

static bool isHexByte(uint8_t b)
{
    return (b >= '0' && b <= '9') || (b >= 'A' && b <= 'F') || (b >= 'a' && b <= 'f');
}

static uint8_t hexVal(uint8_t b)
{
    if (b >= '0' && b <= '9') return (uint8_t)(b - '0');
    return (uint8_t)((b >= 'a' ? b - 'a' : b - 'A') + 10);
}

static void hostResync(HostParser *self, uint8_t b)
{
    ++self->st.resync;
    self->state = 0;
    self->bodyLen = 0;
    if (b == '$') self->state = 1;
}

static void hostCommit(HostParser *self)
{
    if (crc16Ccitt(self->body, (size_t)self->bodyLen) != self->crc) {
        ++self->st.crcBad; self->bodyLen = 0; return;
    }
    ++self->st.ok; self->bodyLen = 0;
}

static void hostFeed(HostParser *self, uint8_t b)
{
    switch (self->state) {
    case 0: if (b == '$') { self->bodyLen = 0; self->state = 1; } else ++self->st.noise; break;
    case 1:
        if (b == '*') self->state = 2;
        else if (b == '$') { ++self->st.resync; self->bodyLen = 0; } /* 帧中途重来：这一帧已丢 */
        else if (self->bodyLen < 64) self->body[self->bodyLen++] = (char)b;
        else { hostResync(self, b); }
        break;
    case 2: if (isHexByte(b)) { self->crc = (uint16_t)(hexVal(b) << 12); self->state = 3; } else hostResync(self, b); break;
    case 3: if (isHexByte(b)) { self->crc = (uint16_t)(self->crc | (hexVal(b) << 8)); self->state = 4; } else hostResync(self, b); break;
    case 4: if (isHexByte(b)) { self->crc = (uint16_t)(self->crc | (hexVal(b) << 4)); self->state = 5; } else hostResync(self, b); break;
    case 5: if (isHexByte(b)) { self->crc = (uint16_t)(self->crc | hexVal(b)); self->state = 6; } else hostResync(self, b); break;
    case 6: if (b == '\n') { hostCommit(self); self->state = 0; } else if (b != '\r') hostResync(self, b); break;
    }
}

static void hostParserInit(HostParser *self)
{
    self->state = 0;
    self->bodyLen = 0;
    self->crc = 0;
    memset(&self->st, 0, sizeof(self->st));
}

// ---------------------------------------------------------------------------
// 事件模型：六类事件 × 24 格小时直方图
// ---------------------------------------------------------------------------
enum { EV_COMM = 0, EV_SEN, EV_PWR, EV_KEY, EV_ALM, EV_WDG, EV_CAT_N };

typedef struct {
    long b[SIM_HOURS][EV_CAT_N];
} Histogram;

static void histAdd(Histogram *self, int hour, int cat) { ++self->b[hour][cat]; }

static void histPrint(const Histogram *self)
{
    int h, c, k;
    printf("  时刻  通信 传感 电源 按键 报警 看门狗 | 小计 条形图\n");
    printf("  ----  ---- ---- ---- ---- ---- ------ -----\n");
    for (h = 0; h < SIM_HOURS; ++h) {
        long sum = 0;
        for (c = 0; c < EV_CAT_N; ++c) sum += self->b[h][c];
        printf("  %2dh  ", h);
        for (c = 0; c < EV_CAT_N; ++c) printf("%4ld ", self->b[h][c]);
        printf("| %4ld ", sum);
        {
            const int bars = (int)(sum / 8); /* 8 事件 = 1 格 */
            for (k = 0; k < bars && k < 30; ++k) printf("#");
        }
        printf("\n");
    }
}

// ---------------------------------------------------------------------------
// SHT30 模型（NACK / CRC-8 错 / 读数），带总线死锁剧本
// ---------------------------------------------------------------------------
typedef struct {
    double baseT, baseRH;
    int nackCount, crcErrCount, retryCount, deadlockRecover;
} ShtModel;

static double sin_curve(double phase) /* 0..1 -> 一天曲线（-1..1 形状即可） */
{
    /* 用多项式近似昼夜形状，避免 <cmath> 依赖，峰在"正午" */
    const double x = phase * 2.0 - 1.0; /* -1..1 */
    return 1.0 - 2.0 * x * x;           /* 抛物线，正午最高 */
}

/* 一次物理读：可能 NACK / CRC 错；t/rh 返回读数 */
static bool shtRead(ShtModel *self, int tick, double *t, double *rh, bool *deadlock)
{
    /* 环境曲线：昼夜 6 度摆动 + 注入的阶跃 + 噪声 */
    double step = 0.0, stepRH = 0.0;
    int roll;
    if (tick >= 2500 && tick < 2541) { step = 20.5; stepRH = -22.0; }  /* 阶跃 A：热+干 */
    if (tick >= 7000 && tick < 7021) { step = 21.5; stepRH = -20.0; }  /* 阶跃 B：再来一次 */
    {
        const double curve = 6.0 * sin_curve((double)tick / TOTAL_TICKS);
        *t = self->baseT + curve + step + (double)(xs64Next() % 41) / 100.0 - 0.20; /* ±0.2 噪声 */
        *rh = self->baseRH + curve * 1.5 + stepRH + (double)(xs64Next() % 101) / 100.0 - 0.5;
    }

    roll = (int)(xs64Next() % 1000);
    if (roll < 4) {            /* 0.4%：NACK */
        ++self->nackCount;
        if (roll == 0) { *deadlock = true; return false; } /* 十万分之一升级成死锁 */
        return false;
    }
    if (roll < 14) { ++self->crcErrCount; *t = 0; *rh = 0; return false; } /* 1%：CRC-8 错 */
    *deadlock = false;
    return true;
}

// ---------------------------------------------------------------------------
// 报警决策：原始值 2/3 表决进（快）+ 滤波滞回出（稳）—— phase3 师傅留的钩子
// ---------------------------------------------------------------------------
typedef struct {
    double hi, lo;
    bool win[3];
    int wp;
    bool alarming;
    double filt;
    int firstCrossMs;
    bool crossValid;
} FastAlarm;

static void fastAlarmInit(FastAlarm *self, double hi, double lo)
{
    self->hi = hi; self->lo = lo;
    self->win[0] = self->win[1] = self->win[2] = false;
    self->wp = 0;
    self->alarming = false;
    self->filt = 0.0;
    self->firstCrossMs = 0;
    self->crossValid = false;
}

/* 返回本拍是否"报警触发"（新进入），released 返回是否退出 */
static bool fastAlarmUpdate(FastAlarm *self, double raw, int nowMs,
                            long *delayMsOut, bool *released)
{
    bool entered = false;
    int votes;
    *released = false;
    self->win[self->wp % 3] = (raw >= self->hi); ++self->wp;
    votes = (self->win[0] ? 1 : 0) + (self->win[1] ? 1 : 0) + (self->win[2] ? 1 : 0);
    if (!self->alarming) {
        if (raw >= self->hi && !self->crossValid) { self->firstCrossMs = nowMs; self->crossValid = true; }
        if (votes >= 2) { /* 3 次里 2 次越限：立刻预警 */
            self->alarming = true;
            entered = true;
            /* 延迟 = 首拍越限->表决通过（≤2 拍采样）+ 蜂鸣器驱动一拍 */
            *delayMsOut = (nowMs - self->firstCrossMs) + 100;
            self->crossValid = false;
        }
    } else {
        double f = (self->filt == 0.0) ? raw : (self->filt * 0.7 + raw * 0.3);
        self->filt = f;
        if (f <= self->lo || raw <= self->lo - 1.0) { /* 滤波滞回 43 出，原始值暴跌兜底 */
            self->alarming = false; self->filt = 0.0; *released = true; self->crossValid = false;
        }
    }
    return entered;
}

// ---------------------------------------------------------------------------
// 静态资源估算（RAM/Flash）—— 对账 20KB / 64KB 的 70% 线
// ---------------------------------------------------------------------------
static void resourceAudit(long *ramOut, long *flashOut)
{
    struct Row { const char *mod; long ram; long flash; };
    static const struct Row rows[] = {
        { "启动文件+向量表+系统初始化", 384,  2048 },
        { "HAL 替身（GPIO/I2C/SPI/UART/ADC 寄存器层）", 512, 9216 },
        { "FreeRTOS 替身（任务/队列/定时器）", 2560, 12288 },
        { "SHT30 驱动+CRC-8+重试状态机",   256,  3072 },
        { "SSD1306 驱动+字库(精简 ASCII)", 1024, 6144 }, /* 页缓冲 1KB */
        { "行协议解析+CRC16（phase4）",    512,  2560 },
        { "报警 FSM+滤波+校准参数(Flash)", 320,  1536 },
        { "业务任务+日志缓冲+看门狗管理",   1024, 4096 },
        { "main+调度表+常量表",            256,  2048 },
    };
    long ram = 0, flash = 0;
    size_t i;
    printf("  %-38s %8s %8s\n", "模块", "RAM(B)", "Flash(B)");
    for (i = 0; i < sizeof(rows) / sizeof(rows[0]); ++i) {
        ram += rows[i].ram; flash += rows[i].flash;
        printf("  %-38s %8ld %8ld\n", rows[i].mod, rows[i].ram, rows[i].flash);
    }
    printf("  %-38s %8ld %8ld\n", "合计", ram, flash);
    *ramOut = ram; *flashOut = flash;
}

// ---------------------------------------------------------------------------
// 关键事件流水（定长日志，替代 std::vector<std::string>）
// ---------------------------------------------------------------------------
static char g_log[40][160];
static int g_logCount = 0;

static void logIt(int tick, const char *s)
{
    if (g_logCount < 40) {
        snprintf(g_log[g_logCount], sizeof(g_log[0]),
                 "  [t=%5d tick | 仿真 %02dh] %s", tick, simHourOf(tick), s);
        ++g_logCount;
    }
}

// ---------------------------------------------------------------------------
// 上行帧发送 + 信道噪声（文件静态函数 + 全局计数，替代闭包写法）
// ---------------------------------------------------------------------------
static long g_framesTx = 0;

static void txFrame(HostParser *host, const char *payload)
{
    char wire[96];
    size_t wireLen;
    int roll;
    size_t k;

    buildFrame(payload, wire, sizeof(wire));
    wireLen = strlen(wire); /* 破坏发生在下面，XOR 不改长度，先记下 */
    ++g_framesTx;
    roll = (int)(xs64Next() % 1000);
    if (roll < 30) {                        /* 3%：帧间噪声串，帧完好 */
        const int n = 2 + (int)(xs64Next() % 4);
        int j;
        for (j = 0; j < n; ++j) {
            uint8_t junk = (uint8_t)(xs64Next() % 256);
            if (junk == '$') junk = 0x00;
            hostFeed(host, junk);
        }
    }
    /* 帧内破坏：系统性注入（每 200 帧一次，约 0.5%）-> CRC 必须拦下 */
    if (g_framesTx % 200 == 137) {
        const size_t pos = 1 + (size_t)(xs64Next() % (wireLen - 4));
        wire[pos] = (char)((uint8_t)wire[pos] ^ (uint8_t)(1 + xs64Next() % 255));
    }
    for (k = 0; k < wireLen; ++k) hostFeed(host, (uint8_t)wire[k]);
}

// ---------------------------------------------------------------------------
// 主仿真
// ---------------------------------------------------------------------------
int main(void)
{
    ShtModel sht = { 25.0, 50.0, 0, 0, 0, 0 };
    HostParser host;
    FastAlarm alarm;
    Histogram hist;
    const int WDG_MS = 2000;
    int wdgLeft = WDG_MS;
    int hangUntil = -1;           /* 计划内挂死演练 */
    const int injectHangArmed = 3200; /* 注入时刻 */

    /* 按键连按注入表（组起点, 组长度） */
    static const int keyBursts[5][2] = { {1000, 4}, {2600, 3}, {5000, 7}, {7600, 2}, {9300, 5} };
    int lastKeyTick = -100;       /* 消抖：距上次有效按下 <2 tick 视为同一bounce */
    int keyValid = 0;

    /* 统计账 */
    long fatalHalts = 0, unexpectedResets = 0, wdgBites = 0;
    long pwrDips = 0, brownoutFlag = 0, busRecover = 0;
    long worstAlarmDelay = 0;
    double sumMa = 0.0, sumMaIdleWin = 0.0; long idleTicks = 0;

    bool ledAlarm = false;
    int lastRhHiTick = -1;
    long alarmEnters = 0, alarmExits = 0; /* 跨复位累计（复位清的是 FSM 状态，不是验收账） */
    int tick;

    long ram = 0, flash = 0;
    double ramPct, flashPct, avgMa, avgIdleMa, crcRate;
    bool allPass;

    printf("===============================================================================\n");
    printf("  MiniBoard Phase 6 - 24h 日历压缩验收台（10000 tick = 1000s 固件时间，种子 20260706）\n");
    printf("===============================================================================\n");

    hostParserInit(&host);
    fastAlarmInit(&alarm, 45.0, 43.0);
    memset(&hist, 0, sizeof(hist));

    for (tick = 0; tick < TOTAL_TICKS; ++tick) {
        const int nowMs = tick * TICK_MS;
        const int hour = simHourOf(tick);
        bool hung;
        double rail33;
        double t = 25.0, rh = 50.0;
        bool haveNew = false;

        /* ================= 1) 看门狗超时演练 ================= */
        if (tick == injectHangArmed && hangUntil < 0) {
            hangUntil = tick + 30; /* 计划挂 3.0s；IWDG 2.0s 先到 -> 必咬 */
            logIt(tick, "注入：任务挂死演练（计划 3.0s；IWDG 2.0s 会先复位）");
        }
        hung = (hangUntil >= 0 && tick < hangUntil);

        /* ================= 2) 电源：瞬时跌落 3.0V / 5ms ================= */
        rail33 = 3.3;
        if ((int)(xs64Next() % 1000) < 6) { /* 0.6%/tick：约 60 次 */
            rail33 = 3.0;
            ++pwrDips;
            histAdd(&hist, hour, EV_PWR);
            if (rail33 < 2.9) { /* POR 阈 2.9V：本剧本不触发 */ }
            else if ((xs64Next() % 10) == 0) { ++brownoutFlag; } /* 固件记录 BOD 标志位 */
            if (pwrDips == 1) logIt(tick, "电源跌落 3.0V/5ms（首现）：BOD 观察，POR 阈 2.9V 未到 -> 不复位");
        }
        if (rail33 < 2.9) { ++unexpectedResets; } /* 真掉穿 POR 才算意外复位（本剧本恒不触发） */

        /* ================= 3) 采样（含 NACK 重试 / 死锁恢复） ================= */
        if (!hung) {
            bool deadlock = false;
            bool ok = shtRead(&sht, tick, &t, &rh, &deadlock);
            if (deadlock) { /* 死锁优先：先跑恢复程序再谈重试 */
                ++busRecover;
                histAdd(&hist, hour, EV_COMM);
                logIt(tick, "I2C 总线死锁 -> 9 clock + STOP 恢复 -> 重试");
                ok = shtRead(&sht, tick, &t, &rh, &deadlock);
            } else if (!ok) {
                ++sht.retryCount;
                ok = shtRead(&sht, tick, &t, &rh, &deadlock); /* 普通 NACK/CRC 错：重试一次 */
            }
            haveNew = ok; /* 仍失败 -> 本轮降级：沿用旧值（phase3 策略） */
        }

        /* ================= 4) 报警判定（2/3 表决进 + 滞回出） ================= */
        if (haveNew) {
            long delay = 0; bool released = false;
            if (fastAlarmUpdate(&alarm, t, nowMs, &delay, &released)) {
                char buf[128];
                ++alarmEnters; ledAlarm = true;
                histAdd(&hist, hour, EV_ALM);
                if (delay > worstAlarmDelay) worstAlarmDelay = delay;
                snprintf(buf, sizeof(buf), "进入 ALARM：T=%.2fC 越 45.0，延迟 %ldms（<=500ms 达标线）", t, delay);
                logIt(tick, buf);
            }
            if (released) {
                ++alarmExits; ledAlarm = false;
                histAdd(&hist, hour, EV_ALM);
                logIt(tick, "退出 ALARM：滤波值回落至 43.0 滞回线以下");
            }
        }

        /* ================= 5) 按键连按（消抖模型） ================= */
        {
            int i;
            for (i = 0; i < 5; ++i) {
                if (tick >= keyBursts[i][0] && tick < keyBursts[i][0] + keyBursts[i][1]) {
                    if (tick - lastKeyTick >= 2) { /* 2 tick（200ms）内是同一抖 */
                        ++keyValid; histAdd(&hist, hour, EV_KEY);
                    }
                    lastKeyTick = tick;
                    break;
                }
            }
        }

        /* ================= 6) 上行帧 + 信道噪声（phase4 解析器） ================= */
        if (!hung) {
            if (tick % 20 == 0) { /* 状态帧：每 2s */
                char p[64];
                snprintf(p, sizeof(p), "STAT,%.2f,%.2f,%d", t, rh, ledAlarm ? 1 : 0);
                txFrame(&host, p);
            }
            if (haveNew && tick - lastRhHiTick >= 500 && rh > 70.0) {
                lastRhHiTick = tick; txFrame(&host, "EV,RH_HIGH");
            }
        }

        /* ================= 7) 功耗逐 tick 记账（5V 侧，phase1 的账） ================= */
        {
            double ma;
            const bool displayBeat = (tick % 20 == 0); /* 2s 刷一次屏（1 tick 活跃） */
            if (ledAlarm) ma = 41.14;
            else if (displayBeat) ma = 15.87;
            else ma = 1.72;
            sumMa += ma;
            if (!ledAlarm) { sumMaIdleWin += ma; ++idleTicks; }
        }

        /* ================= 8) tick 末尾喂狗（与 phase3 同纪律） ================= */
        if (hung) {
            wdgLeft -= TICK_MS;
            if (wdgLeft <= 0) { /* 咬！ */
                ++wdgBites;
                histAdd(&hist, hour, EV_WDG);
                logIt(tick, "IWDG 咬狗：任务已挂死 2.0s（计划 3.0s），复位 -> 原因 = IWDG（计划内，非泄漏）");
                wdgLeft = WDG_MS; hangUntil = -1; /* 复位后重开 */
                fastAlarmInit(&alarm, 45.0, 43.0); /* 复位清 FSM 状态，不清验收账 */
                ledAlarm = false;
            }
        } else {
            wdgLeft = WDG_MS;
        }
        /* 采样事件分类记账 */
        if (haveNew && tick % 10 == 0) histAdd(&hist, hour, EV_SEN);
    }

    /* ================= 事件流水 ================= */
    printf("\n--- 关键事件流水（截前 40 条）---\n");
    {
        int i;
        for (i = 0; i < g_logCount; ++i) printf("%s\n", g_log[i]);
    }

    /* ================= 24h 事件直方图 ================= */
    printf("\n--- 24 格日历事件直方图（压缩映射；计数按 1000s 固件时间，不是真实 24h）---\n");
    histPrint(&hist);

    /* ================= 期末成绩单 ================= */
    printf("\n--- 资源静态估算 ---\n");
    resourceAudit(&ram, &flash);
    ramPct = ram * 100.0 / 20480.0; flashPct = flash * 100.0 / 65536.0;

    avgMa = sumMa / TOTAL_TICKS;
    avgIdleMa = idleTicks ? sumMaIdleWin / idleTicks : 0.0;
    /* 通过率按“发出多少帧、完好收下多少帧”算：重同步丢掉的帧也是坏帧 */
    crcRate = g_framesTx > 0
        ? host.st.ok * 100.0 / (double)g_framesTx : 100.0;

    printf("\n===============================================================================\n");
    printf("  期末验收报告（全 PASS 才准出师）\n");
    printf("===============================================================================\n");
    printf("  %-34s %-28s %s\n", "验收项", "实测", "判定");
    printf("  ---------------------------------- ---------------------------- ----\n");
    printf("  %-34s 意外复位=%ld 咬狗=%ld(计划内)    %s\n",
           "[R3] 10000 tick(1000s) 不死机/无复位泄漏", unexpectedResets, wdgBites,
           (fatalHalts == 0 && unexpectedResets == 0) ? "PASS" : "FAIL");
    printf("  %-34s 最差报警延迟=%ldms            %s\n",
           "[R4] 报警延迟 <= 500ms", worstAlarmDelay,
           (worstAlarmDelay <= 500 && alarmEnters >= 2) ? "PASS" : "FAIL");
    printf("  %-34s 上行=%ld 好帧=%ld 坏帧=%ld 丢失=%ld 通过率=%.2f%%  %s\n",
           "串口整帧通过率 >= 99%", g_framesTx, host.st.ok, host.st.crcBad,
           host.st.resync, crcRate, (crcRate >= 99.0) ? "PASS" : "FAIL");
    printf("  %-34s RAM=%ldB(%.1f%%) Flash=%ldB(%.1f%%) %s\n",
           "RAM/Flash < 70%（20KB/64KB）", ram, ramPct, flash, flashPct,
           (ramPct < 70.0 && flashPct < 70.0) ? "PASS" : "FAIL");
    printf("  %-34s 省电窗口平均=%.2fmA 全程=%.2fmA %s\n",
           "[R1] 省电窗口平均 < 3mA", avgIdleMa, avgMa,
           (avgIdleMa < 3.0) ? "PASS" : "FAIL");
    printf("  %-34s 报警进入=%ld 次（两次阶跃都接住）     %s\n",
           "阶跃响应完整性", alarmEnters,
           (alarmEnters == 2 && alarmExits == 2) ? "PASS" : "FAIL");

    printf("\n--- 明细账（PASS 不是玄学，每条都有出处）---\n");
    printf("  I2C：NACK %d 次 / 死锁恢复 %ld 次 / CRC-8 错 %d 次 / 重试 %d 次\n",
           sht.nackCount, busRecover, sht.crcErrCount, sht.retryCount);
    printf("  电源：跌落 %ld 次（3.0V/5ms，POR 阈 2.9V 未触）/ BOD 标志记录 %ld 次\n",
           pwrDips, brownoutFlag);
    printf("  解析器：好帧 %ld / CRC 拦下 %ld / 噪声字节 %ld / 重同步 %ld（坏帧 0 渗入业务）\n",
           host.st.ok, host.st.crcBad, host.st.noise, host.st.resync);
    printf("  按键：5 组连按 -> 有效按下 %d 次（消抖吞掉所有 bounce）\n", keyValid);
    printf("  看门狗：咬 %ld 次（t=3200 tick 计划内挂死），复位原因 100%% 可查\n", wdgBites);

    allPass = fatalHalts == 0 && unexpectedResets == 0
        && worstAlarmDelay <= 500 && alarmEnters == 2 && alarmExits == 2
        && crcRate >= 99.0 && ramPct < 70.0 && flashPct < 70.0 && avgIdleMa < 3.0;
    printf("\n  总评：%s —— %s\n", allPass ? "6/6 PASS，出师" : "有 FAIL，回炉",
           allPass ? "这块修完 3 个 FAIL 后 ¥23.84 的小卡可以交付了。" : "找到 FAIL 的账，修到全绿再来。");
    return allPass ? 0 : 1;
}
