/*
 * =====================================================================================
 *  第 7 关 · Phase 4 —— I2C 时序模型 + 上位机行协议（phase4_protocol_i2c.c）
 * =====================================================================================
 *
 * 【讲什么】
 *   两条"通信线"一起练，一条往板子里看（I2C），一条往板子外看（UART 行协议）：
 *     A. I2C 物理时序模型：开漏线 + 位级 master/slave，演示四种真实场景——
 *        正常事务、clock stretching（从机拉住 SCL）、地址 NACK、
 *        从机卡死拉住 SDA 的死锁，以及标准恢复程序：放线 + 9 个 SCL 脉冲 + STOP。
 *     B. 上位机行协议：$命令,参数*CRC16-CCITT(0x1021)\r\n，命令 READ_T /
 *        SET_THRESH / STATUS / CAL；抗噪解析器状态机逐字节喂入；
 *        回环测试：5% 的字节是噪声（帧间噪声串 + 少量帧内破坏），看解析器
 *        崩不崩、重同步后帧 CRC 对不对。
 *     C. 上拉电阻 × 总线电容上升时间表：tr = 0.8473 · Rp · Cb（I2C 规范按 30%→70% 定义），
 *        2.2k/4.7k/10k × 50/150/400pF 九宫格，顺便算每根线的"直流电费"。
 *
 *   这一版是纯 C（C17）：I2C 的 master/slave 用 struct + 函数表达，"成员变量"
 *   就是 struct 字段，"成员函数"就是首参数拿指针的静态函数；解析器状态机用
 *   enum + switch，字符串全用定长 char 数组 + snprintf，没有 new、没有容器。
 *
 * 【怎么编译运行】(Windows + MSYS2 ucrt64, gcc 16.2)
 *   先在控制台执行：chcp 65001   （切到 UTF-8 代码页，否则中文输出是乱码）
 *   gcc -std=c17 -O2 -Wall -Wextra phase4_protocol_i2c.c -o phase4_protocol_i2c.exe
 *   ./phase4_protocol_i2c.exe
 *   要求：零警告零错误；回环测试固定随机种子，结果可复现。
 *
 * 【预期看到什么】
 *   - 死锁演示：STOP 之后 SDA 仍被拉住（BUS_BUSY），9 clock + STOP 恢复后
 *     总线重新 ACK —— 这就是真板子上"总线锁死、重烧固件也没用"的软件解药。
 *   - 回环统计：发出 200 帧，成功帧率、丢弃噪声字节数、重同步次数三项打印；
 *     收到的每一帧 CRC 全部正确（坏帧全被拦下，没有一个字节混进业务层）。
 *   - 上升时间表：板上典型 150pF 时，10k 的 tr≈1271ns，连 100kHz 的 1000ns 限都超——
 *     这就是 phase2 给 10k 挂 WARN 的原因；4.7k@150pF≈598ns，板载 100kHz 的甜点位；
 *     要上 400kHz（限 300ns），150pF 下得换 2.2k（≈280ns），灌流代价也跟着上来。
 *
 * 【老师傅点拨】
 *   1. I2C 的开漏 = 所有人都只会"往下拽"，拉高全靠电阻。所以线被拉住不是 bug
 *      而是特性：stretching 是从机说"慢点"，死锁是从机发疯说"我就不撒手"。
 *      master 的每一句"等等"都要真等（检查线真的回到高），否则时序全是假的。
 *   2. 死锁恢复那 9 个时钟不是玄学：I2C 从机状态机最坏卡在 8 位数据中间，
 *      再给够 9 个 SCL（够它走完当前字节 + 一个 ACK 位）它就走到"空闲"了，
 *      补一个 STOP 归零。协议文档里写好的救命招，背下来。
 *   3. 串口解析器的铁律：任何一条异常路径（噪声、溢出、CRC 错、换行错）都必须
 *      回到 Idle 且**清空缓冲**。解析器不许崩、不许死循环、不许把半截帧递给业务。
 *   4. 上拉电阻是"速度 vs 电费"的跷跷板：2.2k 最快，但每个低电平烧 1.3mA——
 *      你 phase1 里 3mA 的静态预算，一根 SCL 就吃掉 44%。板载 100kHz 用 4.7k，
 *      算完表你就明白 phase2 为什么给 10k 挂 WARN——它 150pF 下 tr 超了 1000ns。
 * =====================================================================================
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

/* ---------------------------------------------------------------------------
 * 固定种子随机源：xorshift64*（替代原版的 std::mt19937(20260707)）
 * 为什么不用 rand()：各编译器实现不同，换个机器结果就变；xorshift64* 只有
 * 几行，任何编译器上同种子输出都一样，回环统计因此可复现。
 * ------------------------------------------------------------------------- */
static unsigned long long g_rngState = 20260707ULL; /* 固定种子 */

static unsigned long long xs64Next(void)
{
    g_rngState ^= g_rngState >> 12;
    g_rngState ^= g_rngState << 25;
    g_rngState ^= g_rngState >> 27;
    return g_rngState * 2685821657736338717ULL;
}

static void section(const char *t)
{
    printf("\n===============================================================================\n");
    printf("  %s\n", t);
    printf("===============================================================================\n\n");
}

// ---------------------------------------------------------------------------
// A. I2C 物理时序模型：开漏线 + 位级 master/slave
// ---------------------------------------------------------------------------
typedef struct {
    /* true = 对应引脚在往下拽（开漏只能拽，不能推）；线高 = 上拉电阻的功劳 */
    bool mSdaLow, mSclLow;
    bool sSdaLow, sSclLow;
} I2cLines;

/* 线状态：true = 高（没人拽）。C 没有 const 成员函数，写成拿指针的普通函数 */
static bool lineSda(const I2cLines *ln) { return !(ln->mSdaLow || ln->sSdaLow); }
static bool lineScl(const I2cLines *ln) { return !(ln->mSclLow || ln->sSclLow); }

typedef enum { SLV_NORMAL, SLV_STRETCH_ON_ADDR, SLV_DEADLOCKED } SlaveMode;

typedef struct {
    uint8_t addr;
    uint8_t respByte; /* 读事务时从机往 SDA 上驱动的数据（模型给个固定值） */
    SlaveMode mode;
    int stretchLeft;        /* 还剩几个时钟拉 SCL */
    int sclRisesSinceStuck;
    I2cLines *lines;
} PhysSlave;

/* 死锁：发完 STOP 后耍赖不撒手，把 SDA 拽死 */
static void slaveEnterDeadlock(PhysSlave *self)
{
    self->mode = SLV_DEADLOCKED;
    self->lines->sSdaLow = true;
    self->lines->sSclLow = false;
}

/* 每个 SCL 下降沿从机看一眼总线（恢复流程数时钟用） */
static void slaveOnSclFalling(PhysSlave *self, I2cLines *ln)
{
    if (self->mode == SLV_DEADLOCKED) {
        ++self->sclRisesSinceStuck;
        if (self->sclRisesSinceStuck >= 9) { /* 够 9 个时钟：状态机走完 -> 撒手 */
            self->mode = SLV_NORMAL;
            ln->sSdaLow = false;
            self->sclRisesSinceStuck = 0;
        }
    }
}

/* 地址字节传输期间，从机要不要拉住 SCL 要求慢点 */
static bool slaveWantsStretch(const PhysSlave *self)
{
    return self->mode == SLV_STRETCH_ON_ADDR && self->stretchLeft > 0;
}

static void slaveReleaseOneStretch(PhysSlave *self)
{
    if (self->stretchLeft > 0) --self->stretchLeft;
}

/* 地址字节：只有地址对上才 ACK；数据字节：只要没死机就 ACK */
static bool slaveAckFor(const PhysSlave *self, uint8_t b, bool isAddr)
{
    if (self->mode == SLV_DEADLOCKED) return false;
    return isAddr ? ((uint8_t)(b >> 1) == self->addr) : true;
}

static void slaveOnStop(PhysSlave *self)
{
    if (self->mode != SLV_DEADLOCKED) return;
    /* 恢复流程末尾的 STOP 也能叫醒它（和 9 clock 双保险） */
    self->mode = SLV_NORMAL;
    self->lines->sSdaLow = false;
    self->sclRisesSinceStuck = 0;
}

static void masterSda(I2cLines *ln, bool hi) { ln->mSdaLow = !hi; }
static void masterScl(I2cLines *ln, bool hi) { ln->mSclLow = !hi; }
static void delay(void) { /* 半个时钟周期，模型里 100kHz 下每 bit 10us，打日志已经够了 */ }

static bool masterStart(I2cLines *ln, PhysSlave *sl)
{
    (void)sl;
    masterSda(ln, true); delay();
    masterScl(ln, true); delay();
    if (!lineSda(ln)) { printf("    [I2C] START 前 SDA 已是低 -> BUS_BUSY，总线被拉住\n"); return false; }
    masterSda(ln, false); delay();  /* SCL 高时 SDA 下降 = START */
    masterScl(ln, false); delay();
    return true;
}

static void masterStop(I2cLines *ln, PhysSlave *sl)
{
    masterSda(ln, false); delay();
    masterScl(ln, true); delay();
    masterSda(ln, true); delay();   /* SCL 高时 SDA 上升 = STOP */
    if (!lineSda(ln)) {
        printf("    [I2C] STOP 失败：SCL 已高但 SDA 仍被拽低（总线死锁实锤）\n");
        slaveOnStop(sl);            /* 真协议里从机不该听到这个 STOP；模型里让它有机会被叫醒 */
    }
}

/* 写一字节，返回 ACK(true)。从机 stretch 时真的等。isAddr=true 表示这是地址字节 */
static bool masterWriteByte(I2cLines *ln, PhysSlave *sl, uint8_t b,
                            int *stretchWaitClocks, bool isAddr)
{
    bool ack;
    int i;
    for (i = 7; i >= 0; --i) {
        masterSda(ln, (b >> i) & 1); delay();
        masterScl(ln, true); delay();
        masterScl(ln, false); delay();
    }
    masterSda(ln, true); delay();   /* 第 9 个时钟：master 释放 SDA，轮到从机说话 */
    *stretchWaitClocks = 0;
    /* 从机若要 stretching：先拽住 SCL。master 在等待 ACK 前检查 SCL 是否真回到高 */
    if (slaveWantsStretch(sl)) {
        printf("    [I2C] 从机拉住 SCL（clock stretching），master 等待...");
        while (slaveWantsStretch(sl)) {
            slaveReleaseOneStretch(sl);
            ++*stretchWaitClocks;
        }
        printf("等了 %d 个时钟后释放\n", *stretchWaitClocks);
    }
    /* 从机视角：字节收齐，地址匹配（或普通数据）就拽低 SDA 回 ACK（开漏，只会拽） */
    ln->sSdaLow = slaveAckFor(sl, b, isAddr);
    masterScl(ln, true); delay();
    ack = (lineSda(ln) == false); /* 从机拽低 = ACK */
    masterScl(ln, false); delay();
    ln->sSdaLow = false; /* ACK 位说完，从机撒手 */
    return ack;
}

/* sendAck=true：master 拽低回 ACK；false：释放回 NACK */
static uint8_t masterReadByte(I2cLines *ln, PhysSlave *sl, bool sendAck)
{
    uint8_t b = 0;
    int i;
    masterSda(ln, true); delay();
    for (i = 7; i >= 0; --i) {
        /* 开漏：数据位 0 = 从机拽低，数据位 1 = 从机释放（上拉抬高） */
        ln->sSdaLow = (((sl->respByte >> i) & 1) == 0);
        delay();
        masterScl(ln, true); delay();
        if (lineSda(ln)) b |= (uint8_t)(1u << i);
        masterScl(ln, false); delay();
    }
    ln->sSdaLow = false;            /* 从机释放数据线 */
    masterSda(ln, !sendAck); delay(); /* master 回 ACK（拽低）/ NACK（释放） */
    masterScl(ln, true); delay();
    masterScl(ln, false); delay();
    masterSda(ln, true); delay();
    return b;
}

/* 总线恢复：放掉所有线，用 GPIO 方式打 9 个 SCL 脉冲，再补一个 STOP */
static void masterRecoverBus(I2cLines *ln, PhysSlave *sl, int *pulsesOut)
{
    int pulses = 0;
    int i;
    printf("    [I2C] 恢复程序：释放 SDA/SCL -> 打 9 个 SCL 脉冲 -> 补 STOP\n");
    ln->mSdaLow = false; ln->mSclLow = false;
    for (i = 0; i < 9; ++i) {
        ln->mSclLow = true;  delay();
        ln->mSclLow = false; delay();
        slaveOnSclFalling(sl, ln);
        ++pulses;
    }
    *pulsesOut = pulses;
    masterSda(ln, false); delay();
    masterScl(ln, true); delay();
    masterSda(ln, true); delay(); /* 手动 STOP */
}

static void demoI2c(void)
{
    I2cLines ln = { false, false, false, false };
    PhysSlave slave;
    int st = 0;

    slave.addr = 0x44;
    slave.respByte = 0xA5;
    slave.mode = SLV_NORMAL;
    slave.stretchLeft = 0;
    slave.sclRisesSinceStuck = 0;
    slave.lines = &ln;

    section("A. I2C 物理时序模型（开漏 + 位级模拟）");

    printf("  [场景 1] 正常事务：写命令 0x2C -> 重复 START -> 读 1 字节\n");
    if (masterStart(&ln, &slave)) {
        bool ack = masterWriteByte(&ln, &slave, 0x88, &st, true); /* 0x44 << 1 | W（地址字节） */
        printf("    addr 0x44+W -> %s\n", ack ? "ACK" : "NACK");
        ack = masterWriteByte(&ln, &slave, 0x2C, &st, false);
        printf("    cmd 0x2C -> %s\n", ack ? "ACK" : "NACK");
        masterStart(&ln, &slave);                    /* 重复 START */
        ack = masterWriteByte(&ln, &slave, 0x89, &st, true); /* 0x44 << 1 | R（地址字节） */
        {
            const uint8_t v = masterReadByte(&ln, &slave, false); /* 读 1 字节后 NACK 收尾 */
            printf("    addr 0x44+R -> %s，读回 0x%02X\n", ack ? "ACK" : "NACK", v);
        }
        masterStop(&ln, &slave);
        printf("    STOP 完成，总线空闲（SDA=%d SCL=%d）\n\n", (int)lineSda(&ln), (int)lineScl(&ln));
    }

    printf("  [场景 2] clock stretching：从机没准备好，拉住 SCL 说『慢点』\n");
    slave.mode = SLV_STRETCH_ON_ADDR;
    slave.stretchLeft = 3;
    if (masterStart(&ln, &slave)) {
        const bool ack = masterWriteByte(&ln, &slave, 0x88, &st, true);
        printf("    addr 传输：stretch 等待 %d 个时钟，最终 %s\n\n", st, ack ? "ACK" : "NACK");
        masterStop(&ln, &slave);
    }
    slave.mode = SLV_NORMAL;

    printf("  [场景 3] 地址 NACK：总线上没有 0x44（虚焊/上电没就绪），第 9 拍 SDA 保持高\n");
    slave.addr = 0x45; /* 假装 0x44 这颗不存在 */
    if (masterStart(&ln, &slave)) {
        const bool ack = masterWriteByte(&ln, &slave, 0x88, &st, true);
        printf("    addr 0x44+W -> %s -> 正确姿势：STOP + 重试/降级（phase3 演过）\n\n",
               ack ? "ACK" : "NACK（无人应答）");
        masterStop(&ln, &slave);
    }
    slave.addr = 0x44;

    printf("  [场景 4] 死锁：从机状态机发疯，STOP 后仍把 SDA 拽在低\n");
    slaveEnterDeadlock(&slave);
    printf("    （从机已卡死拉住 SDA：此刻 SDA=%d，SCL=%d）\n", (int)lineSda(&ln), (int)lineScl(&ln));
    masterStart(&ln, &slave); /* 尝试通信 -> 检测到 BUS_BUSY */
    printf("    应用层怎么办？重烧固件没用、复位 master 没用（线是从机拽着的）。\n");
    {
        int pulses = 0;
        masterRecoverBus(&ln, &slave, &pulses);
        printf("    打了 %d 个 SCL 脉冲 + STOP -> 从机状态机走完，撒手了\n", pulses);
        printf("    验证：此刻 SDA=%d SCL=%d（%s）\n", (int)lineSda(&ln), (int)lineScl(&ln),
               (lineSda(&ln) && lineScl(&ln)) ? "总线恢复" : "仍卡死");
        if (masterStart(&ln, &slave)) {
            const bool ack = masterWriteByte(&ln, &slave, 0x88, &st, true);
            printf("    恢复后首笔事务：addr 0x44+W -> %s\n", ack ? "ACK，总线满血复活" : "NACK");
            masterStop(&ln, &slave);
        }
    }
}

// ---------------------------------------------------------------------------
// B. 上位机行协议：$payload*CRC16(4hex)\r\n + 抗噪解析器
// ---------------------------------------------------------------------------
/* 0x1021 / 初值 0xFFFF / 无反转。按长度算（帧体理论上可能出现 0x00 噪声字节） */
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

/* 设备端：收到合法命令 -> 产出响应（回环测试把响应帧喂回解析器） */
static void deviceDispatch(const char *body, char *out, size_t cap)
{
    if (strcmp(body, "READ_T") == 0)     { snprintf(out, cap, "T:25.31,RH:49.86"); return; }
    if (strcmp(body, "STATUS") == 0)     { snprintf(out, cap, "RUN,ALARM:0,IWDG:900"); return; }
    if (strncmp(body, "SET_THRESH;", 11) == 0) { snprintf(out, cap, "OK,THRESH=%s", body + 11); return; }
    if (strncmp(body, "CAL;", 4) == 0)    { snprintf(out, cap, "OK,CAL"); return; }
    snprintf(out, cap, "ERR,UNKNOWN");
}

typedef struct {
    long okFrames;        /* CRC 正确并交付业务 */
    long crcBad;          /* 帧形完整但 CRC 拦下 */
    long noiseBytes;      /* Idle 态丢掉的噪声字节 */
    long resyncs;         /* 非 Idle 态因异常退回 Idle 的次数 */
    long overflow;        /* 帧体超长（>64B）丢弃 */
    long unknownCmd;      /* CRC 对但不是已知命令 */
} ParserStats;

/* 抗噪解析器状态机：任何异常路径都回 Idle 并清空缓冲，绝不让半截帧漏进业务层。
 * C 里这就是一个 struct（状态 + 缓冲 + 统计）+ 一个 feed 函数，够用且零分配。 */
typedef enum { ST_IDLE, ST_BODY, ST_CRC_HI, ST_CRC_LO, ST_CRC3, ST_CR, ST_WAIT_LF } LineState;

typedef struct {
    bool isDevice;        /* true=设备侧（解析命令并应答） */
    LineState state;
    char body[65];        /* 帧体上限 64B + 结尾 0 */
    int bodyLen;
    uint16_t crc;
    char lastResp[80];    /* 设备响应帧，回环测试再喂回来（帧体 64 + 帧头尾 8 还富余） */
    ParserStats stats;
} LineParser;

static bool isHexByte(uint8_t b)
{
    return (b >= '0' && b <= '9') || (b >= 'A' && b <= 'F') || (b >= 'a' && b <= 'f');
}

static uint8_t hexVal(uint8_t b)
{
    if (b >= '0' && b <= '9') return (uint8_t)(b - '0');
    return (uint8_t)((b >= 'a' ? b - 'a' : b - 'A') + 10);
}

static void parserResync(LineParser *self, uint8_t b)
{
    ++self->stats.resyncs;
    self->state = ST_IDLE;
    self->bodyLen = 0;
    if (b == '$') { self->state = ST_BODY; } /* 噪声恰好是新帧头：立刻重同步，一个字节都不误 */
}

static void parserCommit(LineParser *self)
{
    const uint16_t calc = crc16Ccitt(self->body, (size_t)self->bodyLen);
    if (calc != self->crc) { ++self->stats.crcBad; self->bodyLen = 0; return; } /* 坏帧到此为止 */
    ++self->stats.okFrames;
    self->body[self->bodyLen] = '\0';
    if (self->isDevice) { /* 只有设备侧解析命令并产生响应；上位机侧收的是应答帧 */
        char resp[64];
        deviceDispatch(self->body, resp, sizeof(resp));
        if (strcmp(resp, "ERR,UNKNOWN") == 0) ++self->stats.unknownCmd;
        buildFrame(resp, self->lastResp, sizeof(self->lastResp));
    }
    self->bodyLen = 0;
}

static void parserFeed(LineParser *self, uint8_t b)
{
    switch (self->state) {
    case ST_IDLE:
        if (b == '$') { self->bodyLen = 0; self->state = ST_BODY; }
        else ++self->stats.noiseBytes;
        break;
    case ST_BODY:
        if (b == '*') self->state = ST_CRC_HI;
        else if (b == '$') { self->bodyLen = 0; }   /* 帧头重现：旧的半截帧作废 */
        else if (self->bodyLen < 64) self->body[self->bodyLen++] = (char)b;
        else { ++self->stats.overflow; parserResync(self, b); } /* 超长：整个作废 */
        break;
    case ST_CRC_HI:
        if (isHexByte(b)) { self->crc = (uint16_t)(hexVal(b) << 12); self->state = ST_CRC_LO; }
        else parserResync(self, b);
        break;
    case ST_CRC_LO:
        if (isHexByte(b)) { self->crc = (uint16_t)(self->crc | (hexVal(b) << 8)); self->state = ST_CRC3; }
        else parserResync(self, b);
        break;
    case ST_CRC3:
        if (isHexByte(b)) { self->crc = (uint16_t)(self->crc | (hexVal(b) << 4)); self->state = ST_CR; }
        else parserResync(self, b);
        break;
    case ST_CR:
        if (isHexByte(b)) { self->crc = (uint16_t)(self->crc | hexVal(b)); self->state = ST_WAIT_LF; }
        else parserResync(self, b);
        break;
    case ST_WAIT_LF:
        /* 协议是 *XXXX\r\n */
        if (b == '\n') { parserCommit(self); self->state = ST_IDLE; }
        else if (b != '\r') parserResync(self, b);   /* 容忍多余 \r */
        break;
    }
}

static void parserInit(LineParser *self, bool isDevice)
{
    self->isDevice = isDevice;
    self->state = ST_IDLE;
    self->bodyLen = 0;
    self->crc = 0;
    self->lastResp[0] = '\0';
    memset(&self->stats, 0, sizeof(self->stats));
}

static void demoProtocol(void)
{
    LineParser dev, host;
    static const char *const cmds[4] = {"READ_T", "SET_THRESH;45", "STATUS", "CAL;1"};
    int framesSent = 0;
    long respSent = 0;
    int i;

    section("B. 行协议 + 抗噪解析器（200 帧回环，5% 噪声字节，固定种子）");

    parserInit(&dev, true);   /* 设备侧：解析命令帧并产出响应 */
    parserInit(&host, false); /* 上位机侧：只收响应帧，不做命令分发 */

    for (i = 0; i < 200; ++i) {
        char wire[64];
        size_t wireLen;
        int roll;
        long before, b2;

        buildFrame(cmds[i % 4], wire, sizeof(wire));
        wireLen = strlen(wire);

        /* ---- 信道噪声：5% 字节是噪声（上行方向）---- */
        roll = (int)(xs64Next() % 100);
        if (roll < 3) {
            /* 3%：帧间噪声串（USB 转串口毛刺、热插拔毛头），帧本身完好 */
            const int n = 2 + (int)(xs64Next() % 4);
            int k;
            for (k = 0; k < n; ++k) {
                uint8_t junk = (uint8_t)(xs64Next() % 256);
                if (junk == '$') junk = 0x00;
                parserFeed(&dev, junk);
            }
        } else if (roll < 5) {
            /* 2%：帧内破坏（某字节被翻掉）-> 解析器必须用 CRC 拦下，然后重同步 */
            const size_t pos = 1 + (size_t)(xs64Next() % (wireLen - 4));
            wire[pos] = (char)((uint8_t)wire[pos] ^ (uint8_t)(1 + xs64Next() % 255));
        }

        /* ---- 发出命令帧给设备；设备响应喂给上位机（回环）---- */
        before = dev.stats.okFrames;
        {
            size_t k;
            for (k = 0; k < wireLen; ++k) parserFeed(&dev, (uint8_t)wire[k]);
        }
        ++framesSent;
        if (dev.stats.okFrames > before) { /* 命令帧解析成功设备才会应答 */
            const size_t respLen = strlen(dev.lastResp);
            b2 = host.stats.okFrames;
            {
                size_t k;
                for (k = 0; k < respLen; ++k) parserFeed(&host, (uint8_t)dev.lastResp[k]);
            }
            if (host.stats.okFrames > b2) ++respSent;
        }
    }

    printf("  发出命令帧 %d -> 设备侧成功解析 %ld -> 应答回传上位机 %ld（上位机收到 %ld）\n",
           framesSent, dev.stats.okFrames, respSent, host.stats.okFrames);
    printf("  设备侧 CRC 拦下的坏帧：%ld（一个字节都没混进业务层）\n", dev.stats.crcBad);
    printf("  Idle 态丢弃噪声字节：%ld\n", dev.stats.noiseBytes);
    printf("  重同步次数：%ld   帧体超长丢弃：%ld   未知命令：%ld（应为 0：只发过已知命令）\n",
           dev.stats.resyncs, dev.stats.overflow, dev.stats.unknownCmd);
    printf("  结论：解析器不崩不死循环；每一笔交付给业务的帧 CRC 都正确，\n");
    printf("        坏帧的代价只是重发 —— 这就是 phase6 里『整帧通过率 >= 99%%』的底气。\n");
}

// ---------------------------------------------------------------------------
// C. 上拉电阻 × 总线电容：tr = 0.8473 · Rp · Cb
// ---------------------------------------------------------------------------
static void demoRiseTable(void)
{
    const double rpK[3] = {2.2, 4.7, 10.0};
    const double cbPf[3] = {50, 150, 400};
    int i, j;

    section("C. 上升时间表：tr = 0.8473 x Rp x Cb（30%->70%，I2C 规范口径）");

    printf("  Rp\\Cb      50pF          150pF(板上)     400pF(长线/多器件)\n");
    printf("  --------   ------------- --------------- ------------------\n");
    for (i = 0; i < 3; ++i) {
        printf("  %4.1fk     ", rpK[i]);
        for (j = 0; j < 3; ++j) {
            /* I2C 规范口径：tr 从 0.3·Vdd 升到 0.7·Vdd，tr = ln(7/3)·Rp·Cb ≈ 0.8473·Rp·Cb。
             * 记口诀：0.8473 × Rp(kΩ) × Cb(pF) 的数值就是 tr(ns)。 */
            const double tr = 0.8473 * rpK[i] * 1000.0 * cbPf[j] * 1e-12 * 1e9; /* ns */
            /* 限值：标准模式 100kHz tr<=1000ns，快速模式 400kHz tr<=300ns */
            const char *tag = (tr <= 300.0) ? "OK" : (tr <= 1000.0 ? "仅100k" : "FAIL");
            printf(" %6.1fns %-4s ", tr, tag);
        }
        /* 直流电费：低电平灌入电流 I = (3.3 - VOL 0.4) / Rp */
        printf(" 低电平灌流 %5.2fmA\n", (3.3 - 0.4) / rpK[i]);
    }
    printf("\n  读数姿势（本板 I2C 按 100kHz 标准模式跑，限 tr<=1000ns）：\n");
    printf("  - 板上典型 Cb≈150pF（MCU 10p + SHT30 80p + 走线/过孔 ~50p + OLED 隔离）：\n");
    printf("    10k 的 tr≈1271ns，连 100kHz 的 1000ns 限都超 —— 这就是 phase2 的 WARN。\n");
    printf("  - 4.7k@150pF≈598ns，100kHz 下余量充足，灌流 0.62mA 可接受 -> 板载甜点位。\n");
    printf("  - 要上 400kHz（限 300ns）：150pF 下只有 2.2k（≈280ns）过线，但每根被拉低的\n");
    printf("    线烧 1.32mA —— 3mA 静态预算直接被啃掉 44%%。设计不是选最快，是选都活。\n");
}

int main(void)
{
    printf("===============================================================================\n");
    printf("  MiniBoard Phase 4 - I2C 时序模型 + 上位机行协议（纯软件，离线可跑）\n");
    printf("===============================================================================\n");
    demoI2c();
    demoProtocol();
    demoRiseTable();
    printf("\n--- 师傅收个尾 ---\n");
    printf("  1. stretching 要真等、NACK 要降级、死锁要会 9 clock + STOP 救场；\n");
    printf("  2. 行协议 = 帧头 + 长度上限 + CRC + 铁打的回 Idle，四样少一样迟早翻车；\n");
    printf("  3. 上拉表背下来：0.8473·Rp·Cb。3.3V 板载 100kHz 下 4.7k@150pF≈598ns 是甜点位，\n");
    printf("     10k@150pF≈1271ns 连 100k 都超标；想跑 400k 得换 2.2k 并付灌流的电费。\n");
    return 0;
}
