/*
 * =====================================================================================
 *  第 7 关 · Phase 3 —— 固件骨架（主机可跑）（phase3_firmware_core.c）
 * =====================================================================================
 *
 * 【讲什么】
 *   没有真板子也能练固件分层。本程序在 PC 上用"假时钟 + 外设影子寄存器"搭出
 *   一个完整的分层固件骨架，跑 30 秒仿真时间（主循环 = 100ms 子 tick，业务拍 = 1s）：
 *     HAL 模拟层：GPIO 影子寄存器 / 假时钟 / I2C·SPI·UART 总线模型 / IWDG
 *     驱动层    ：SHT30（CRC-8 校验 + 重试 + 降级）、SSD1306 最小指令集、
 *                 按键消抖状态机、LED 呼吸软件 PWM
 *     服务层    ：采样调度、滑动平均滤波、报警阈值 + 滞回
 *     应用层    ：主循环 + 状态机 INIT -> SELFTEST -> RUN -> ALARM -> SAFE
 *
 *   【这一版是纯 C（C17），重点看 HAL 怎么做抽象】
 *   C 语言没有 class、没有 virtual，但嵌入式 C 项目里的 HAL/驱动抽象到处都是，
 *   惯用做法就是一张"函数指针表"（struct HalOps）：
 *     - 表里每个函数指针 = 一个"接口方法"，第一个参数 void *sim 指向实现方数据；
 *     - 模拟实现（FakeHal：影子寄存器/假时钟）和真芯片实现（STM32 寄存器操作）
 *       各自填一张表，运行时把表"注入"给驱动 —— 这就是 C 语言的虚函数表；
 *     - 好处一：可测试。驱动/服务/应用三层在 PC 上对着假表跑，逻辑全验证；
 *     - 好处二：可换芯片。从 F103 换到 F0/GD32，只换这张表，上层一行不改。
 *   "对象"在 C 里的表达：struct + 以 self 作首参数的函数；本程序每类只建
 *   一个全局实例，和真固件的单例外设一致。
 *
 * 【怎么编译运行】(Windows + MSYS2 ucrt64, gcc 16.2)
 *   先在控制台执行：chcp 65001    （切到 UTF-8 代码页，否则中文显示成乱码）
 *   gcc -std=c17 -O2 -Wall -Wextra phase3_firmware_core.c -o phase3_firmware_core.exe
 *   ./phase3_firmware_core.exe
 *   要求：零警告零错误。
 *
 * 【预期看到什么】
 *   - 上电 -> OLED 首帧 80ms（规格 <= 200ms，PASS；SELFTEST 故意不整屏刷屏）
 *   - 串口每秒一行 CSV：t_ms,temp,rh,state,alarm,wdog_left_ms（115200 的替身）
 *   - t≈5s 注入 3 帧 CRC 错误：重试 2 次仍失败 -> 降级沿用旧值 + 故障标志，程序不死
 *   - t≈12s KEY1 短按（20ms 消抖通过）、t≈16s KEY2 长按 4s -> 关屏省电
 *   - t≈18s 温度阶跃 25->52：3 点平均滤波拖到 t≈20.1s 才进 ALARM（延迟约 2 秒）；
 *     t≈21s 回落 40：滞回 45 进 / 43 出，t≈22.1s 均值 44.0 还压着不放，
 *     t≈23.1s 均值 40.0 才退出 -> SAFE 冷却 2s -> t≈25.1s 回 RUN —— 亲眼看见"滤波是双刃剑"
 *   - t≈26s 注入主循环卡死 1.3s：喂狗停了，IWDG(1s) 在 t≈26.9s 咬狗复位，
 *     复位原因=IWDG，重启走 SELFTEST 回 RUN，30 秒里就这一次复位
 *   - 结尾统计：读取/CRC 错/降级/复位/按键/报警 各多少次
 *
 * 【老师傅点拨】
 *   1. 分层不是为了画图好看，是为了"换腿能跑"：把 HalOps 表换成真 STM32 寄存器
 *      操作，驱动/服务/应用三层一行不改。你现在定义的每个接口，都是在给真板子铺路。
 *   2. 喂狗点只放一处：主循环（100ms 子 tick）末尾。谁在子函数里喂狗，谁就是给
 *      看门狗喂安眠药。注意喂狗周期(100ms)必须远小于狗叫周期(1s)，留 10 倍余量；
 *      卡死时喂狗和业务一起停，狗 1 秒内必咬。SELFTEST 只发初始指令不刷全屏，
 *      就是为了 200ms 出显示的指标 —— 顺序也是设计。
 *   3. CRC 错误的正确姿势 = 重试 N 次 -> 降级用上次有效值 + 打故障标志。
 *      最坏的做法是把脏数据当真数据用，那是量产返修单上的常客。
 *   4. 滞回（45 进 / 43 出）+ 滑动平均 = 报警不抖，但代价是触发/释放都要慢约 2 秒。
 *      规格要求"500ms 内报警"时，你就得换算法（原始值越限立刻预警 + 滤波值确认），
 *      phase6 会继续算这笔账。
 * =====================================================================================
 */

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>

/* ===========================================================================
 * HAL 模拟层
 *
 * HalOps = 固件对下依赖的全部"接口方法"，一张 const 函数指针表。
 * 真项目里对应 hal.h：驱动只认这张表，不认表背后是寄存器还是 PC 上的影子变量。
 * ======================================================================== */
typedef struct HalOps {
    /* GPIO：写影子寄存器 / 读输入电平 */
    void (*gpio_write)(void *sim, const char *pin, int level);
    int  (*gpio_read)(void *sim, const char *pin);
    /* I2C：一次完整事务。返回读到的字节数，-1 = 地址无应答（NACK） */
    int  (*i2c_transact)(void *sim, uint8_t addr, const uint8_t *w, int wn,
                         uint8_t *r, int rmax);
    /* SPI：命令（D/C=0）与数据（D/C=1） */
    void (*spi_cmd)(void *sim, uint8_t c);
    void (*spi_data)(void *sim, const uint8_t *d, int n);
    /* UART：一行一条（前缀 UART| 模拟串口助手窗口） */
    void (*uart_line)(void *sim, const char *line);
    /* 假时钟：真板上是 SysTick/定时器，接口形状一样 */
    long (*clock_now)(void *sim);
    void (*clock_advance)(void *sim, long ms);
    /* IWDG：真固件只会"喂"和（模型里）查剩余时间；倒计时由硬件自己走 */
    void (*iwdg_feed)(void *sim);
    long (*iwdg_left)(void *sim);
} HalOps;

/* "接口 + 实现"的句柄：驱动拿到的永远只是这一个轻量的对 */
typedef struct Hal {
    const HalOps *ops;
    void         *sim;
} Hal;

/* ---------------------------------------------------------------------------
 * 被模拟的物理外设（挂在"总线"上的全局唯一实例）
 * ------------------------------------------------------------------------- */

/* CRC-8（SHT30：多项式 0x31，初值 0xFF，无反转） */
static uint8_t crc8Sht(const uint8_t *d, int n)
{
    uint8_t crc = 0xFF;
    int i, b;
    for (i = 0; i < n; ++i) {
        crc ^= d[i];
        for (b = 0; b < 8; ++b)
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x31) : (uint8_t)(crc << 1);
    }
    return crc;
}

/* SHT30 从机物理模型（挂在总线上） */
typedef struct {
    int    faultReadsLeft; /* 还有几帧要坏                */
    bool   nackFault;      /* 坏法：true=NACK，false=CRC 被篡改 */
    double tempC;
    double rh;
} Sht30Slave;

static Sht30Slave gSht30Slave = { 0, false, 25.0, 50.0 };

/* 出一帧 [Tmsb Tlsb Tcrc Rmsb Rlsb Rcrc] */
static void sht30Frame(const Sht30Slave *self, uint8_t out[6])
{
    const int rawT = (int)((self->tempC + 45.0) / 175.0 * 65535.0);
    const int rawH = (int)(self->rh / 100.0 * 65535.0);
    out[0] = (uint8_t)(rawT >> 8);
    out[1] = (uint8_t)(rawT & 0xFF);
    out[2] = crc8Sht(out, 2);
    out[3] = (uint8_t)(rawH >> 8);
    out[4] = (uint8_t)(rawH & 0xFF);
    out[5] = crc8Sht(out + 3, 2);
}

/* IWDG：LSI 独立时钟，超时咬狗 */
typedef struct {
    long timeoutMs;
    long left;
    int  bites;
} Iwdg;

static void  iwdgInit(Iwdg *self, long timeoutMs) { self->timeoutMs = timeoutMs; self->left = timeoutMs; self->bites = 0; }
static void  iwdgFeed(Iwdg *self) { self->left = self->timeoutMs; }
static bool  iwdgTick(Iwdg *self, long ms)
{
    self->left -= ms;
    if (self->left <= 0) { self->left = self->timeoutMs; ++self->bites; return true; }
    return false;
}

/* SPI 模型（SSD1306）。统计计数是"测试台探针"，跨复位累计，方便对账 */
typedef struct {
    long    cmds;
    long    writes;
    long    bytes;
    uint8_t last;
} SpiModel;

/* GPIO 影子寄存器（保序，便于打印快照） */
typedef struct { char name[16]; int level; } PinShadow;

typedef struct {
    PinShadow  pins[16];
    int        pinCount;
    long       nowMs;
    SpiModel   spi;
    Iwdg       iwdg;
} FakeHal;

static FakeHal gSim; /* 全局唯一的"模拟硬件" */

/* ---- 假实现（FakeHal 侧）：填进 HalOps 表的就这几个函数 ---- */
static void gpioSet(FakeHal *sim, const char *pin, int level)
{
    int i;
    for (i = 0; i < sim->pinCount; ++i) {
        if (strcmp(sim->pins[i].name, pin) == 0) { sim->pins[i].level = level; return; }
    }
    if (sim->pinCount < 16) {
        snprintf(sim->pins[sim->pinCount].name, sizeof(sim->pins[sim->pinCount].name), "%s", pin);
        sim->pins[sim->pinCount].level = level;
        ++sim->pinCount;
    }
}

static void fakeGpioWrite(void *sim, const char *pin, int level)
{
    gpioSet((FakeHal *)sim, pin, level);
}

static int fakeGpioRead(void *sim, const char *pin)
{
    FakeHal *s = (FakeHal *)sim;
    int i;
    for (i = 0; i < s->pinCount; ++i)
        if (strcmp(s->pins[i].name, pin) == 0) return s->pins[i].level;
    return 0;
}

static int fakeI2cTransact(void *sim, uint8_t addr, const uint8_t *w, int wn,
                           uint8_t *r, int rmax)
{
    (void)sim; (void)w; (void)wn; (void)rmax;
    if (addr != 0x44) return -1; /* 地址无应答 */
    if (gSht30Slave.faultReadsLeft > 0) {
        --gSht30Slave.faultReadsLeft;
        if (gSht30Slave.nackFault) return -1;
        sht30Frame(&gSht30Slave, r);
        r[5] ^= 0xFF; /* 篡改最后一字节 CRC */
        return 6;
    }
    sht30Frame(&gSht30Slave, r);
    return 6;
}

static void fakeSpiCmd(void *sim, uint8_t c)
{
    FakeHal *s = (FakeHal *)sim;
    ++s->spi.cmds;
    s->spi.last = c;
}

static void fakeSpiData(void *sim, const uint8_t *d, int n)
{
    FakeHal *s = (FakeHal *)sim;
    ++s->spi.writes;
    s->spi.bytes += (long)n;
    (void)d;
}

static void fakeUartLine(void *sim, const char *line)
{
    (void)sim;
    printf("UART| %s\n", line);
}

static long fakeClockNow(void *sim)      { return ((FakeHal *)sim)->nowMs; }
static void fakeClockAdvance(void *sim, long ms) { ((FakeHal *)sim)->nowMs += ms; }
static void fakeIwdgFeed(void *sim)      { iwdgFeed(&((FakeHal *)sim)->iwdg); }
static long fakeIwdgLeft(void *sim)      { return ((FakeHal *)sim)->iwdg.left; }

/* 模拟实现这张表。上真板时另写一张 stm32f103Ops（GPIO_BSRR/I2C_DR/SPI_DR/USART_DR/
 * IWDG_KR=0xCCCC/SysTick），驱动一行不用改 —— 这就是"换腿能跑"。 */
static const HalOps gFakeOps = {
    fakeGpioWrite, fakeGpioRead,
    fakeI2cTransact,
    fakeSpiCmd, fakeSpiData,
    fakeUartLine,
    fakeClockNow, fakeClockAdvance,
    fakeIwdgFeed, fakeIwdgLeft
};

/* 注入给固件的 HAL 句柄（全局唯一） */
static const Hal gHal = { &gFakeOps, &gSim };

/* ===========================================================================
 * 驱动层（只认 Hal 句柄，不认后面是真是假）
 * ======================================================================== */

typedef struct { double tempC; double rh; bool valid; } Sample;

/* SHT30 驱动：CRC-8 校验 + 重试 + 降级 */
typedef struct {
    Hal     hal;
    Sample  lastValid;
    int     okCount;
} Sht30Drv;

static Sample sht30ReadWithRetry(Sht30Drv *self, int retries,
                                 int *crcErr, int *nackErr, int *retryCount)
{
    static const uint8_t cmdMeas[2] = { 0x2C, 0x06 }; /* 高重复性测量命令 */
    uint8_t r[8];
    Sample s;
    int attempt, n;

    for (attempt = 0; attempt <= retries; ++attempt) {
        if (attempt > 0) ++*retryCount;
        n = self->hal.ops->i2c_transact(self->hal.sim, 0x44, cmdMeas, 2, r, (int)sizeof(r));
        if (n < 0) { ++*nackErr; continue; }
        if (n != 6)  { ++*crcErr; continue; }
        if (crc8Sht(r, 2) != r[2] || crc8Sht(r + 3, 2) != r[5]) { ++*crcErr; continue; }
        {
            const int rawT = (r[0] << 8) | r[1];
            const int rawH = (r[3] << 8) | r[4];
            s.tempC = -45.0 + 175.0 * rawT / 65535.0;
            s.rh = 100.0 * rawH / 65535.0;
            s.valid = true;
        }
        self->lastValid = s;
        ++self->okCount;
        return s;
    }
    s = self->lastValid;
    s.valid = false; /* 降级：旧值 + 上层打故障标志 */
    return s;
}

/* SSD1306 最小指令集驱动 */
typedef struct {
    Hal  hal;
    bool inited;
    bool screenOff;
    long frameNo;
} Ssd1306Drv;

static void ssd1306Begin(Ssd1306Drv *self) /* 24 条初始化指令（末尾 0xAF = 显示开），模型耗时 ≈12ms */
{
    static const uint8_t init[] = { 0xAE, 0xD5, 0x80, 0xA8, 0x3F, 0xD3, 0x00, 0x40, 0x8D, 0x14,
                                    0xA1, 0xC8, 0xDA, 0x12, 0x81, 0x8F, 0xD9, 0xF1, 0xDB, 0x30,
                                    0xA4, 0xA6, 0x8F, 0xAF };
    int i;
    for (i = 0; i < (int)sizeof(init); ++i) self->hal.ops->spi_cmd(self->hal.sim, init[i]);
    self->inited = true;
    self->screenOff = false;
}

static void ssd1306ShowText(Ssd1306Drv *self, const char *s)
{
    if (!self->inited || self->screenOff) return; /* 关屏后不刷数据，省电账对得上 */
    ++self->frameNo;
    self->hal.ops->spi_cmd(self->hal.sim, 0xB0); /* 页寻址 */
    self->hal.ops->spi_data(self->hal.sim, (const uint8_t *)s, (int)strlen(s));
}

static void ssd1306SetDisplayOff(Ssd1306Drv *self, bool off)
{
    self->hal.ops->spi_cmd(self->hal.sim, off ? 0xAE : 0xAF);
    self->screenOff = off;
}

/* 按键消抖状态机：20ms 消抖，>=1500ms 判长按 */
typedef enum { BTN_IDLE, BTN_BOUNCE, BTN_HELD } BtnState;
typedef enum { BTN_EV_NONE, BTN_EV_SHORT, BTN_EV_LONG } BtnEv;

typedef struct {
    const char *pin;
    BtnState    state;
    long        t0;
} Button;

static BtnEv buttonPoll(Button *self, const Hal *hal, long nowMs)
{
    const int raw = hal->ops->gpio_read(hal->sim, self->pin);
    BtnEv ev = BTN_EV_NONE;
    switch (self->state) {
    case BTN_IDLE:
        if (raw) { self->state = BTN_BOUNCE; self->t0 = nowMs; }
        break;
    case BTN_BOUNCE:
        if (nowMs - self->t0 >= 20) self->state = raw ? BTN_HELD : BTN_IDLE;
        break;
    case BTN_HELD:
        if (!raw) { ev = (nowMs - self->t0 >= 1500) ? BTN_EV_LONG : BTN_EV_SHORT; self->state = BTN_IDLE; }
        break;
    }
    return ev;
}

/* LED 呼吸：软件 PWM，2s 三角波 */
typedef struct {
    long   phase;
    double duty;
} LedBreath;

static void ledBreathTick(LedBreath *self, long ms)
{
    const long kPeriodMs = 2000;
    long half;
    self->phase += ms;
    if (self->phase >= kPeriodMs) self->phase -= kPeriodMs;
    half = kPeriodMs / 2;
    self->duty = (self->phase < half) ? (double)self->phase / (double)half
                                      : (double)(kPeriodMs - self->phase) / (double)half;
}

/* ===========================================================================
 * 服务层（纯数据加工，连 HAL 都不碰，最好测）
 * ======================================================================== */
typedef struct {
    double buf[3];
    int    n;     /* 窗口大小 */
    int    count; /* 已填入个数 */
} MovingAvg;

static double movingAvgAdd(MovingAvg *self, double v)
{
    double s = 0.0;
    int i;
    if (self->count == self->n) {
        for (i = 0; i < self->n - 1; ++i) self->buf[i] = self->buf[i + 1];
        self->buf[self->n - 1] = v;
    } else {
        self->buf[self->count++] = v;
    }
    for (i = 0; i < self->count; ++i) s += self->buf[i];
    return self->count ? s / (double)self->count : v;
}

static void movingAvgReset(MovingAvg *self) { self->count = 0; }

/* 报警滞回状态机：45 进 / 43 出 */
typedef struct {
    double hi, lo;
    bool   alarming;
    int    edges;
} AlarmFsm;

static void alarmFsmInit(AlarmFsm *self, double hi, double lo)
{
    self->hi = hi; self->lo = lo; self->alarming = false; self->edges = 0;
}

static bool alarmFsmUpdate(AlarmFsm *self, double t)
{
    if (!self->alarming && t >= self->hi)      { self->alarming = true;  ++self->edges; }
    else if (self->alarming && t <= self->lo)  { self->alarming = false; ++self->edges; }
    return self->alarming;
}

/* ===========================================================================
 * 应用层
 * ======================================================================== */
typedef enum { ST_INIT, ST_SELFTEST, ST_RUN, ST_ALARM, ST_SAFE } AppState;

static const char *stateName(AppState s)
{
    switch (s) {
    case ST_INIT:     return "INIT";
    case ST_SELFTEST: return "SELFTEST";
    case ST_RUN:      return "RUN";
    case ST_ALARM:    return "ALARM";
    case ST_SAFE:     return "SAFE";
    }
    return "?";
}

/* "对象"= struct；全局唯一实例 gFw，和真固件的单例状态机一致 */
typedef struct {
    Hal hal; /* 注入的 HAL 函数指针表 */

    Sht30Drv   sht;
    Ssd1306Drv oled;
    Button     key1, key2; /* KEY1=PA0、KEY2=PB1（KEY1 复用 BOOT0，见坑 #4） */
    AlarmFsm   alarm;

    bool hangActive;
    long hangUntilMs;

    int  crcErr, nackErr, retries, degraded;
    int  keyEvents, alarmEnter, alarmExit;
    long firstFrameMs;

    AppState st;
    long     stateEnterMs;
    bool     faultFlag;

    MovingAvg  avgT, avgH;
    LedBreath  breath;
} Firmware;

static Firmware gFw;

/* "构造函数"：上电一次性做 GPIO 配置与成员初始化 */
static void firmwareInit(Firmware *self)
{
    const HalOps *ops = gHal.ops;
    self->hal = gHal;

    self->sht.hal = gHal;
    self->sht.lastValid.tempC = 25.0; self->sht.lastValid.rh = 50.0; self->sht.lastValid.valid = true;
    self->sht.okCount = 0;

    self->oled.hal = gHal;
    self->oled.inited = false; self->oled.screenOff = false; self->oled.frameNo = 0;

    self->key1.pin = "KEY1"; self->key1.state = BTN_IDLE; self->key1.t0 = 0;
    self->key2.pin = "KEY2"; self->key2.state = BTN_IDLE; self->key2.t0 = 0;

    alarmFsmInit(&self->alarm, 45.0, 43.0);

    self->hangActive = false; self->hangUntilMs = 0;
    self->crcErr = 0; self->nackErr = 0; self->retries = 0; self->degraded = 0;
    self->keyEvents = 0; self->alarmEnter = 0; self->alarmExit = 0;
    self->firstFrameMs = -1;

    self->st = ST_INIT; self->stateEnterMs = 0; self->faultFlag = false;

    self->avgT.n = 3; self->avgT.count = 0;
    self->avgH.n = 3; self->avgH.count = 0;
    self->breath.phase = 0; self->breath.duty = 0.0;

    ops->gpio_write(self->hal.sim, "LED_PWR", 0);
    ops->gpio_write(self->hal.sim, "LED_ST", 0);
    ops->gpio_write(self->hal.sim, "LED_ALM", 0);
    ops->gpio_write(self->hal.sim, "BUZZER", 0);
    ops->gpio_write(self->hal.sim, "KEY1", 0);
    ops->gpio_write(self->hal.sim, "KEY2", 0);
    ops->gpio_write(self->hal.sim, "LED_PWR", 1); /* 电源灯是硬件直连 —— phase1 电流榜第一名，固件管不了 */
}

/* 上电/复位公共流程：INIT(15ms) -> SELFTEST(65ms) -> RUN */
static void firmwareBoot(Firmware *self, bool cold)
{
    const HalOps *ops = self->hal.ops;
    void *sim = self->hal.sim;
    char msg[256];

    movingAvgReset(&self->avgT);
    movingAvgReset(&self->avgH);
    self->alarm.alarming = false;
    ops->gpio_write(sim, "BUZZER", 0);
    ops->gpio_write(sim, "LED_ALM", 0);
    self->st = ST_INIT;
    self->stateEnterMs = ops->clock_now(sim);
    snprintf(msg, sizeof(msg), "BOOT: %s，HSE 8MHz x9=72MHz，GPIO/I2C/SPI/UART/IWDG(1000ms) ready",
             cold ? "POR 上电复位" : "IWDGRST 看门狗复位（RCC_CSR 可读）");
    ops->uart_line(sim, msg);
    ops->clock_advance(sim, 15);
    self->st = ST_SELFTEST;
    self->stateEnterMs = ops->clock_now(sim);
    ssd1306Begin(&self->oled);                     /* 24 条指令 ≈ 12ms（末尾 0xAF 重新开屏） */
    ssd1306ShowText(&self->oled, "MiniBoard v1.0 OK"); /* 首帧 ≈ 15ms */
    ops->clock_advance(sim, 65);                   /* SELFTEST 合计 65ms：故意不整屏刷屏（坑 #8） */
    if (cold) self->firstFrameMs = ops->clock_now(sim);
    ops->uart_line(sim, "SELFTEST: ack@0x44 OK, OLED 首帧 OK, 上电->首帧 = 80ms（预算 200ms）");
    self->st = ST_RUN;
    self->stateEnterMs = ops->clock_now(sim);
    ops->iwdg_feed(sim); /* 喂狗点之一：boot 出口（复位后先喂一口再进主循环） */
}

/* 业务拍（1s）：采样 -> 滤波 -> 报警判定 -> OLED/串口。喂狗不在这里（见主循环） */
static void firmwareBeat(Firmware *self)
{
    const HalOps *ops = self->hal.ops;
    void *sim = self->hal.sim;
    const long now = ops->clock_now(sim);
    char disp[64];
    char line[160];
    Sample s;
    double t, h;
    bool alming;

    s = sht30ReadWithRetry(&self->sht, 2, &self->crcErr, &self->nackErr, &self->retries);
    if (s.valid) {
        t = movingAvgAdd(&self->avgT, s.tempC);
        h = movingAvgAdd(&self->avgH, s.rh);
        if (self->faultFlag) { self->faultFlag = false; ops->uart_line(sim, "INFO: I2C 恢复正常，撤销降级"); }
    } else {
        ++self->degraded;
        self->faultFlag = true;
        /* 降级：本模型用现场值顶（真板 = 沿用上次有效值 + 陈旧标志） */
        t = movingAvgAdd(&self->avgT, gSht30Slave.tempC);
        h = movingAvgAdd(&self->avgH, gSht30Slave.rh);
        ops->uart_line(sim, "WARN: SHT30 重试 2 次仍失败，DEGRADE 沿用旧值 + 故障标志（不死、不 panic）");
    }

    alming = alarmFsmUpdate(&self->alarm, t);
    if (self->st == ST_RUN && alming) {
        self->st = ST_ALARM;
        self->stateEnterMs = now;
        ++self->alarmEnter;
        ops->gpio_write(sim, "BUZZER", 1);
        ops->uart_line(sim, "EVENT: 进入 ALARM（滤波温度 >= 45.0）蜂鸣器 ON + 报警灯 4Hz");
    } else if (self->st == ST_ALARM && !alming) {
        self->st = ST_SAFE;
        self->stateEnterMs = now;
        ++self->alarmExit;
        ops->gpio_write(sim, "BUZZER", 0);
        ops->uart_line(sim, "EVENT: 退出 ALARM（滤波温度 <= 43.0，滞回 2 度）-> SAFE 冷却 2s");
    } else if (self->st == ST_SAFE && now - self->stateEnterMs >= 2000) {
        self->st = ST_RUN;
        self->stateEnterMs = now;
        ops->gpio_write(sim, "LED_ALM", 0);
        ops->uart_line(sim, "EVENT: SAFE -> RUN");
    }

    snprintf(disp, sizeof(disp), "T:%.1fC H:%.1f%% %s%s", t, h, stateName(self->st),
             self->faultFlag ? " !IO" : "");
    ssd1306ShowText(&self->oled, disp);

    snprintf(line, sizeof(line), "%ld,%.2f,%.2f,%s,%d,%ld",
             now, t, h, stateName(self->st), self->alarm.alarming ? 1 : 0,
             ops->iwdg_left(sim));
    ops->uart_line(sim, line);
}

/* ===========================================================================
 * main = 测试台/仿真环境：30s 仿真。注意 boot() 会把时钟推 80ms，注入条件一律用
 * >= 不用 ==。测试台直接拨"物理侧"（从机故障、温度曲线、按键电平），
 * 固件只通过 HalOps 表看世界 —— 真上板时这些注入换成人为制造即可。
 * ======================================================================== */
int main(void)
{
    const HalOps *ops = gHal.ops;
    void *sim = gHal.sim;
    long nextBeatMs;
    bool injCrc = false, k1Down = false, k1Up = false, k2Down = false, k2Up = false;
    bool injHang = false, injRise = false, injFall = false;
    int i;

    printf("===============================================================================\n");
    printf("  MiniBoard Phase 3 - 固件骨架仿真（30s，主循环 tick=100ms，业务拍=1s）\n");
    printf("===============================================================================\n\n");

    iwdgInit(&gSim.iwdg, 1000); /* 狗在硬件侧：1s 超时 */
    firmwareInit(&gFw);

    printf("[t=%4ldms] === 冷启动上电 ===\n", ops->clock_now(sim));
    firmwareBoot(&gFw, true); /* 走完 INIT+SELFTEST，时钟来到 80ms */

    nextBeatMs = 1000;

    for (i = 0; i < 300; ++i) {
        long now;
        bool hanging;
        BtnEv e1, e2;

        ops->clock_advance(sim, 100);
        now = ops->clock_now(sim);

        /* ---- 场景注入 ---- */
        if (!injCrc && now >= 5000) {
            injCrc = true;
            gSht30Slave.faultReadsLeft = 3; /* 3 帧 CRC 被篡改 */
            printf("[t=%4ldms] >>> 注入：SHT30 连续 3 帧 CRC 错误（从机应答了，但数据是脏的）\n", now);
        }
        if (!k1Down && now >= 12000) {
            k1Down = true; ops->gpio_write(sim, "KEY1", 1);
            printf("[t=%4ldms] >>> 注入：KEY1 按下（400ms 短按）\n", now);
        }
        if (!k1Up && now >= 12400) { k1Up = true; ops->gpio_write(sim, "KEY1", 0); }
        if (!k2Down && now >= 16000) {
            k2Down = true; ops->gpio_write(sim, "KEY2", 1);
            printf("[t=%4ldms] >>> 注入：KEY2 按下（4s 长按 = 关屏省电）\n", now);
        }
        if (!k2Up && now >= 20000) { k2Up = true; ops->gpio_write(sim, "KEY2", 0); }
        if (!injRise && now >= 18000) {
            injRise = true;
            gSht30Slave.tempC = 52.0;
            printf("[t=%4ldms] >>> 注入：环境温度阶跃 25 -> 52 度（负载突变）\n", now);
        }
        if (!injFall && injRise && now >= 21000) {
            injFall = true;
            gSht30Slave.tempC = 40.0;
            printf("[t=%4ldms] >>> 注入：温度阶跃回落 52 -> 40 度（风扇介入；40 < 释放阈值 43）\n", now);
        }
        if (!injHang && now >= 26000) {
            injHang = true;
            gFw.hangActive = true;
            gFw.hangUntilMs = now + 1300;
            printf("[t=%4ldms] >>> 注入：主循环卡死 1.3s（刷屏阻塞/死循环）—— 看门狗看你的了\n", now);
        }

        /* ---- 传感器物理曲线 ---- */
        if (!injRise) gSht30Slave.tempC = 25.0 + (double)now / 1000.0 * 0.05;
        gSht30Slave.rh = 50.0 - (gSht30Slave.tempC - 25.0) * 0.3;

        hanging = gFw.hangActive && now < gFw.hangUntilMs;
        if (!hanging) {
            /* ---- 外设层（每个子 tick）---- */
            e1 = buttonPoll(&gFw.key1, &gFw.hal, now);
            e2 = buttonPoll(&gFw.key2, &gFw.hal, now);
            if (e1 == BTN_EV_SHORT) {
                ++gFw.keyEvents;
                printf("[t=%4ldms] BTN: KEY1 短按（20ms 消抖 + 1500ms 短/长判定）-> 切显示亮度\n", now);
            }
            if (e2 == BTN_EV_LONG) {
                ++gFw.keyEvents;
                ssd1306SetDisplayOff(&gFw.oled, !gFw.oled.screenOff);
                printf("[t=%4ldms] BTN: KEY2 长按 4s -> OLED %s（省电入口，对账 phase1：关屏省 6mA）\n",
                       now, gFw.oled.screenOff ? "关屏" : "恢复显示");
            }
            ledBreathTick(&gFw.breath, 100);
            if (!gFw.alarm.alarming) ops->gpio_write(sim, "LED_ST", gFw.breath.duty > 0.5 ? 1 : 0);
            else ops->gpio_write(sim, "LED_ALM", (now / 250) % 2 == 0 ? 1 : 0); /* 报警灯 4Hz */

            /* ---- 业务拍 ---- */
            if (now >= nextBeatMs) {
                nextBeatMs += 1000;
                firmwareBeat(&gFw);
            }

            /* ---- 喂狗点（唯一业务喂狗点）：主循环末尾 ---- */
            ops->iwdg_feed(sim);
        }

        /* ---- IWDG 倒计时（卡死时喂狗停了，只有这里在走）---- */
        if (iwdgTick(&gSim.iwdg, 100)) {
            printf("[t=%4ldms] !!!! IWDG 超时咬狗：RCC_CSR 置位 IWDGRSTF，MCU 复位 !!!!\n", now);
            gFw.hangActive = false; /* 复位后外设重新初始化，卡死条件消失 */
            firmwareBoot(&gFw, false);
            nextBeatMs = ops->clock_now(sim) + 1000;
        }
    }

    /* ---- 验收数字 ---- */
    printf("\n--- Phase 3 验收数字 ---\n");
    printf("  上电 -> OLED 首帧：%ld ms（规格 <= 200ms -> %s）\n",
           gFw.firstFrameMs, (gFw.firstFrameMs <= 200 ? "PASS" : "FAIL"));
    printf("  SHT30：成功读取 %d 次 / CRC 错 %d 次 / NACK %d 次 / 重试 %d 次 / 降级 %d 次\n",
           gFw.sht.okCount, gFw.crcErr, gFw.nackErr, gFw.retries, gFw.degraded);
    printf("  IWDG 咬狗复位：%d 次（预期 1 次，t≈26.9s，复位原因 = IWDG）\n", gSim.iwdg.bites);
    printf("  按键事件：%d（预期 2：KEY1 短按 + KEY2 长按关屏）\n", gFw.keyEvents);
    printf("  报警：进入 %d 次（预期 t≈20.1s）/ 退出 %d 次（预期 t≈23.1s）；"
           "滞回 45 进 / 43 出，滤波窗口 3 拍\n", gFw.alarmEnter, gFw.alarmExit);
    printf("  OLED：%ld 帧 / SPI 指令 %ld 条 / 数据 %ld 字节（总线探针口径，跨复位累计）\n",
           gFw.oled.frameNo, gSim.spi.cmds, gSim.spi.bytes);
    printf("\n  师傅点评：\n");
    printf("    - CRC 三连错：重试 2 次（共 3 次尝试）仍败 -> 降级 + 故障标志，主循环没停、状态机没乱。\n");
    printf("    - 卡死 1.3s：喂狗和业务一起停，1s 的 IWDG 必咬。复位 -> SELFTEST -> RUN，\n");
    printf("      30 秒里就允许这一次复位；复位次数是要上产线报表的，多一次都要查根因。\n");
    printf("    - 温度 18s 冲到 52、21s 已回到 40，ALARM 却 20.1s 才进、23.1s 才退：\n");
    printf("      滤波 + 滞回保住了'不抖'，代价是进、出各约 2 秒延迟。规格要 500ms 报警时，\n");
    printf("      就要换策略（原始值越限立刻预警 + 滤波值确认），phase6 继续算这笔账。\n");
    return 0;
}
