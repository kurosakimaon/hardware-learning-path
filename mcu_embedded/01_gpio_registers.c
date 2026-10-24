/* =====================================================================
 * 01_gpio_registers.c —— 在 PC 上跑一个「假 STM32」的 GPIO
 *
 * 【这个文件讲什么】
 *   控制一个引脚，说白了就是往 7 个寄存器里塞几个 2 位的格子：
 *     MODER   每引脚 2 位 —— 方向（输入/通用输出/复用/模拟）
 *     OTYPER  每引脚 1 位 —— 输出类型（0 推挽 / 1 开漏）
 *     OSPEEDR 每引脚 2 位 —— 速度档（也代表驱动能力档位）
 *     PUPDR   每引脚 2 位 —— 上下拉（无/上拉/下拉/保留）
 *     IDR     只读        —— 引脚真实电平
 *     ODR     可读可写    —— 输出数据锁存器
 *     BSRR    只写        —— 低 16 位写 1 置位、高 16 位写 1 复位
 *   本文件用软件影子寄存器把这些都搭出来，再加一个假时钟和外部世界模型，
 *   让没有板子的你也能亲手看见「每一位是谁改的、什么时候改的」。
 *
 *   位操作全部用 C 的看家写法：寄存器名是宏（REG_MODER），改一个字段就是
 *   「掩码 + 移位 + 读-改-写」三件套：
 *     置位：  REG_MODER  |=  (3u << (2u * n));
 *     清位：  REG_MODER  &= ~MASK2(n);
 *     改字段：REG_MODER   = (REG_MODER & ~MASK2(n)) | ((v & 3u) << (2u * n));
 *   每次改动都同时打印十六进制和「按字节用下划线分组」的 32 位二进制展开，
 *   让你看见位而不是猜位。
 *
 * 【怎么编译运行】（MSYS2 / MinGW，gcc 16.2 实测零警告）
 *   chcp 65001            ← 运行前先执行这一句，否则 Windows 控制台中文乱码
 *   gcc -std=c17 -O2 -Wall -Wextra 01_gpio_registers.c -o 01_gpio_registers.exe
 *   .\01_gpio_registers.exe
 *   没有开发板也能跑：寄存器是内存里的影子变量，纯打印。
 *
 * 【预期看到什么】
 *   1) MODER 复位值是 0xA8000000（PA13/14/15 = 0b10 复用，其余 0b00 输入），为什么不是 0
 *   2) 四种配置后的完整寄存器快照：推挽输出 2MHz / 开漏+上拉 / 输入+下拉 / AF1
 *   3) 100 个 tick 的 LED 闪烁 + 按键软件消抖主循环，含按键抖动逐 tick 打印
 *   4) 事故一：GPIOA_ODR = (1u << 3) 把 PA5 的 LED 一起灭掉了
 *   5) 事故二：读-改-写 ODR 的过程中被「中断」插队，插队方写的内容被覆盖
 *   6) 事故三：对只读的 IDR 赋值，下一个 tick 立刻被打回原形
 *
 * 【老师傅点拨】
 *   - BSRR 是「原子」的：一次写进去，硬件内部自己拆成置位/复位，中间没人能插队。
 *     ODR 的 |= 是三条总线动作（读、或、写），中间一旦被中断插队，你就把
 *     别人刚写的位覆盖掉了。这种 bug 上线才发作，仿真查不出，所以今天先在这
 *     个模型里把它演一遍，让你以后写 ODR |= 的时候手抖一下。
 *   - 开漏输出「写 1」只是把 N-MOS 关掉（释放总线），它自己产生不了高电平。
 *     忘了外部上拉，IDR 永远读回 0 —— 别怀疑芯片，量一下上拉电阻焊没焊。
 *   - 寄存器编码别跨系列抄：F1 用 CRL/CRH，4 位一格；F4 才是 MODER 2 位一格，
 *     而且 F1 的速度编码是 01=10MHz、10=2MHz、11=50MHz，跟 F4 完全不是一个顺序。
 *   - C 里改寄存器位只有三招：`|=` 置位、`&= ~` 清位、`^=` 翻转。`=` 是整字覆盖，
 *     除非你确定这 32 位全归你管，否则它就是本文件演示三的那场事故。
 * ===================================================================== */

#include <stdio.h>
#include <stdint.h>
#include <inttypes.h>   /* PRIu32 / PRIX32：定宽整数打印 */
#include <stdbool.h>

/* ------------------------- 影子寄存器（GPIOA） ------------------------- */
/* 复位值直接取 STM32F4 GPIOA_MODER 的手册值 0xA8000000：最高三个 2 位组是 10
 * ——PA15/PA14/PA13（SWDIO / SWCLK / JTDI）复位即复用，其余 13 脚是 00 输入。
 * 这既是「上电就能用 SWD 烧录」的原因，也是「你把 13/14 改成普通 GPIO 之后
 * 调试器连不上」的原因。OSPEEDR 等其余寄存器模型里简化成全 0（真机上
 * SWD 那几档是最高速，查手册「GPIO register map and reset values」）。 */
static volatile struct {
    uint32_t moder;
    uint32_t otyper;
    uint32_t ospeedr;
    uint32_t pupdr;
    uint32_t idr;     /* 只读：只有外设模型能写 */
    uint32_t odr;
    uint32_t bsrr;    /* 只写：读完恒为 0 */
    uint32_t afrl;    /* PA0~PA7  每引脚 4 位 */
    uint32_t afrh;    /* PA8~PA15 每引脚 4 位 */
} g_gpioa = {
    0xA8000000u, 0x00000000u, 0x00000000u, 0x00000000u,
    0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u
};

/* 寄存器名写成宏：看起来就像手册里的 GPIOA->MODER，实际是内存里的影子字 */
#define REG_MODER   (g_gpioa.moder)
#define REG_OTYPER  (g_gpioa.otyper)
#define REG_OSPEEDR (g_gpioa.ospeedr)
#define REG_PUPDR   (g_gpioa.pupdr)
#define REG_IDR     (g_gpioa.idr)
#define REG_ODR     (g_gpioa.odr)
#define REG_BSRR    (g_gpioa.bsrr)
#define REG_AFRL    (g_gpioa.afrl)
#define REG_AFRH    (g_gpioa.afrh)

/* ---------------------------- 位域宏（C 惯用法） ---------------------------- */
/* 2 位一格的寄存器：MODER / OSPEEDR / PUPDR，字段位置 = 引脚号 × 2 */
#define POS2(pin)      ((uint32_t)(pin) * 2u)
#define MASK2(pin)     (3u << POS2(pin))
#define GET2(reg, pin) (((reg) >> POS2(pin)) & 3u)
/* 读-改-写：先掩掉旧的 2 位，再把新值挪到位上塞回去 —— 一个都不能多碰 */
#define PUT2(reg, pin, v) \
    ((reg) = ((reg) & ~MASK2(pin)) | (((uint32_t)(v) & 3u) << POS2(pin)))

/* 1 位一格的寄存器（OTYPER）就直接用 |= / &= ~ 的移位掩码写法 */
#define OT_OD_SET(pin)   (REG_OTYPER  |=  (1u << (pin)))
#define OT_OD_CLR(pin)   (REG_OTYPER  &= ~(1u << (pin)))

/* ------------------------------ 模式常量 ------------------------------ */
enum {
    MODE_INPUT  = 0u,   /* 0b00 输入             */
    MODE_OUTPUT = 1u,   /* 0b01 通用输出         */
    MODE_AF     = 2u,   /* 0b10 复用功能（外设） */
    MODE_ANALOG = 3u    /* 0b11 模拟（ADC/DAC）  */
};
enum { OT_PP = 0u, OT_OD = 1u };                       /* 推挽/开漏 */
enum { PUP_NONE = 0u, PUP_UP = 1u, PUP_DOWN = 2u, PUP_NA = 3u };
enum { SPD_LOW = 0u, SPD_MED = 1u, SPD_FAST = 2u, SPD_ULTRA = 3u };

/* ----------------------- 假时钟 / 外部世界模型 ----------------------- */
static uint32_t g_tick = 0;                  /* 1 tick = 1ms 的软件假时钟 */

/* 外部世界对每个引脚做了什么：0 悬空、1 外部拉到地、2 外部拉到 VDD */
static uint32_t g_ext[16];
/* 板子上有没有焊外部上拉电阻（I2C 那种 4.7k 到 VDD） */
static bool g_ext_pullup[16];
/* 浮空引脚的电平「不定」，模型里就保持上次的值，另外打个标记 */
static bool g_float_flag[16];
/* 复用功能引脚由哪个外设驱动：0=未驱动（当输入用），1=驱动低，2=驱动高 */
static uint32_t g_af_drive[16];

/* ---------------------------- 外设模型 ---------------------------- */
/* 每个 tick 由「外设」更新 IDR：引脚真实电平不是 ODR 说了算，是电路说了算 */
static void sim_update_idr(void)
{
    uint32_t idr = 0;
    for (uint32_t pin = 0; pin < 16u; ++pin) {
        const uint32_t mode  = GET2(REG_MODER, pin);
        const uint32_t otype = (REG_OTYPER >> pin) & 1u;
        const uint32_t pupdr = GET2(REG_PUPDR, pin);
        int level;
        g_float_flag[pin] = false;      /* 只有下面明确判为浮空的分支才重新置位 */
        if (mode == MODE_OUTPUT && otype == OT_PP) {
            level = (int)((REG_ODR >> pin) & 1u);     /* 推挽：引脚 == ODR */
            g_float_flag[pin] = false;
        } else if (mode == MODE_OUTPUT && otype == OT_OD) {
            /* 开漏：写 0 拉低，写 1 只是「放手」，高电平得靠外部上拉 */
            if (((REG_ODR >> pin) & 1u) == 0u) {
                level = 0;
            } else {
                const bool pull_hi = g_ext_pullup[pin] || (pupdr == PUP_UP);
                if (g_ext[pin] == 1u)      level = 0;
                else if (pull_hi)          level = 1;
                else { level = (int)((idr >> pin) & 1u); g_float_flag[pin] = true; }
            }
        } else if (mode == MODE_AF && g_af_drive[pin] != 0u) {
            /* AF 输出：引脚由片上外设（USART/SPI/TIM…）推挽驱动，ODR 无关 */
            level = (g_af_drive[pin] == 2u);
            g_float_flag[pin] = false;
        } else if (mode == MODE_INPUT || mode == MODE_AF) {
            if (g_ext[pin] == 1u)        level = 0;
            else if (g_ext[pin] == 2u)   level = 1;
            else if (pupdr == PUP_UP)    level = 1;
            else if (pupdr == PUP_DOWN)  level = 0;
            else { level = (int)((idr >> pin) & 1u); g_float_flag[pin] = true; }
        } else {                          /* 模拟模式：数字输入通路断开 */
            level = 0;
            g_float_flag[pin] = false;
        }
        if (level != 0) idr |= (1u << pin);
    }
    REG_IDR = idr;
}

/* BSRR 的硬件行为：低 16 位写 1 置位、高 16 位写 1 复位，寄存器本身读完即清 */
static void bsrr_write(uint32_t val)
{
    REG_BSRR = val;                  /* 总线上写进去，CPU 侧读回来是 0      */
    REG_ODR |= (val & 0xFFFFu);      /* BSrr = 1 → ODR 置 1（原子）         */
    REG_ODR &= ~(val >> 16);         /* BRsr = 1 → ODR 清 0（原子）         */
    REG_BSRR = 0u;                   /* 硬件自清，读出来永远是 0             */
}

/* ------------------------------ 打印工具 ------------------------------ */
static void hr(const char *title)
{
    printf("\n============================================================\n");
    printf("%s\n", title);
    printf("============================================================\n");
}

/* 自写的二进制打印 helper：32 位全展开，每 8 位（一个字节）用下划线分隔。
 * 手册里给你的是十六进制，脑子里要能立刻换成二进制 —— 多打几遍就会了。 */
static void print_bin32(const char *label, uint32_t v)
{
    printf("      %s = 0x%08" PRIX32 " = ", label, v);
    for (unsigned b = 0; b < 32u; ++b) {
        if (b != 0u && (b % 8u) == 0u) putchar('_');
        putchar(((v >> (31u - b)) & 1u) ? '1' : '0');
    }
    printf("\n");
}

static const char *mode_name(uint32_t m)
{
    switch (m) {
        case MODE_INPUT:  return "输入";
        case MODE_OUTPUT: return "输出";
        case MODE_AF:     return "复用";
        default:          return "模拟";
    }
}
static const char *speed_name(uint32_t s)
{
    switch (s) {
        case SPD_LOW:   return "低速 2MHz / 约2mA ";
        case SPD_MED:   return "中速 25MHz / 约8mA";
        case SPD_FAST:  return "快速 50MHz / 12mA ";
        default:        return "超高速 100MHz      ";
    }
}
static const char *pupdr_name(uint32_t p)
{
    switch (p) {
        case PUP_NONE: return "无上下拉  ";
        case PUP_UP:   return "内部上拉  ";
        case PUP_DOWN: return "内部下拉  ";
        default:       return "保留      ";
    }
}

static void dump_regs(const char *tag)
{
    printf("  %-16s MODER=%08" PRIX32 " OTYPER=%08" PRIX32 " OSPEEDR=%08" PRIX32 "\n",
           tag, REG_MODER, REG_OTYPER, REG_OSPEEDR);
    printf("  %-16s PUPDR =%08" PRIX32 " IDR    =%08" PRIX32 " ODR    =%08" PRIX32 "\n",
           "", REG_PUPDR, REG_IDR, REG_ODR);
    print_bin32("MODER 展开", REG_MODER);
    print_bin32("ODR    展开", REG_ODR);
}

static void describe_pin(uint32_t pin)
{
    const uint32_t mode  = GET2(REG_MODER, pin);
    const uint32_t otype = (REG_OTYPER >> pin) & 1u;
    const uint32_t spd   = GET2(REG_OSPEEDR, pin);
    const uint32_t pup   = GET2(REG_PUPDR, pin);
    const uint32_t odr   = (REG_ODR >> pin) & 1u;
    const uint32_t idr   = (REG_IDR >> pin) & 1u;
    uint32_t af = 0u;
    if (pin < 8u) af = (REG_AFRL >> (pin * 4u)) & 0xFu;
    else          af = (REG_AFRH >> ((pin - 8u) * 4u)) & 0xFu;

    printf("  PA%-2u %s %s %s %s | MODER[%u:%u]=0b%02u ODR=%u IDR=%u",
           pin, mode_name(mode),
           (mode == MODE_OUTPUT) ? (otype == OT_OD ? "开漏" : "推挽") : "    ",
           (mode == MODE_OUTPUT) ? speed_name(spd) : "                  ",
           pupdr_name(pup),
           pin * 2u + 1u, pin * 2u, mode, odr, idr);
    if (mode == MODE_AF)   printf("  AF=%u", af);
    if (g_float_flag[pin]) printf("   ← 电平不定（浮空）");
    printf("\n");
}

/* ======================= 演示一：复位与四种典型配置 ======================= */
static void demo_configure(void)
{
    hr("演示一：复位值 + 四种典型配置");

    printf("\n[复位后] MODER = 0x%08" PRIX32 "\n", REG_MODER);
    print_bin32("MODER 二进制", REG_MODER);
    printf("  为什么不是 0？拆开看：A8 = 10101000，最高三个 2 位组全是 10 ——\n");
    printf("  PA15/PA14/PA13（SWDIO/SWCLK/JTDI）复位就是 0b10 复用，其余 0b00 输入。\n");
    printf("  这是 STM32F4 GPIOA_MODER 的真实手册值：调试引脚上电就归调试外设管，\n");
    printf("  所以「上电就能用 SWD 烧录」；反过来说，把 13/14 改成普通 GPIO 之后\n");
    printf("  调试器就连不上了。每个口的复位值不同（GPIOB_MODER 就是全 0），\n");
    printf("  查手册「GPIO register map and reset values」，别拿一个口的值当全口的。\n");
    printf("  共同点：上电默认不驱动、无上下拉，让引脚「什么都不干」，\n");
    printf("  避免上电就把下游电路带歪、或者跟外部驱动对打烧管子。\n");

    /* --- PA5：LED，推挽输出，低速 2MHz --- */
    PUT2(REG_MODER, 5u, MODE_OUTPUT);       /* bit11:10 = 0b01 通用输出 */
    OT_OD_CLR(5u);                          /* 0 = 推挽： &= ~(1u << 5) */
    PUT2(REG_OSPEEDR, 5u, SPD_LOW);
    PUT2(REG_PUPDR, 5u, PUP_NONE);
    sim_update_idr();
    printf("\n[配置 1] PA5 = 推挽输出 + 低速档（点 LED，够用就别提速）\n");
    printf("  MODER  bit11:10 = 0b01（通用输出） OTYPER bit5 = 0（推挽）\n");
    describe_pin(5u);

    /* --- PA6：I2C 风格，开漏输出 + 内部上拉 + 板上有 4.7k 外部上拉 --- */
    PUT2(REG_MODER, 6u, MODE_OUTPUT);
    OT_OD_SET(6u);                          /* |= (1u << 6)：只点这一位，别的不动 */
    PUT2(REG_OSPEEDR, 6u, SPD_LOW);
    PUT2(REG_PUPDR, 6u, PUP_UP);
    g_ext_pullup[6] = true;
    /* --- PA7：同样开漏，但板子上忘了焊上拉电阻 --- */
    PUT2(REG_MODER, 7u, MODE_OUTPUT);
    OT_OD_SET(7u);
    PUT2(REG_OSPEEDR, 7u, SPD_LOW);
    PUT2(REG_PUPDR, 7u, PUP_NONE);          /* 内部上拉也没开 */

    /* 两个都「写 1」：注意开漏写 1 = 释放总线，靠上拉电阻抬起来 */
    bsrr_write(1u << 6);
    bsrr_write(1u << 7);
    sim_update_idr();
    printf("\n[配置 2] PA6 = 开漏 + 上拉（有外部 4.7k）；PA7 = 开漏 + 谁都没上拉\n");
    printf("  两个都执行了 BSRR 置位（ODR bit6 = bit7 = 1）：\n");
    describe_pin(6u);
    describe_pin(7u);
    printf("  两个 ODR 都是 1，为什么 PA7 的 IDR 读回 0 / 不定？\n");
    printf("  因为开漏写 1 只是关掉 N-MOS（放手），高电平必须由上拉电阻从 VDD 拉上来。\n");
    printf("  PA7 内外上拉都没有 → 浮空 → 读回不定。\n");
    printf("  现场表现就是「SDA 一直是低 / 读不到 ACK」，先量上拉再怀疑协议。\n");

    /* --- PA0：按键，输入 + 内部下拉 --- */
    PUT2(REG_MODER, 0u, MODE_INPUT);
    PUT2(REG_PUPDR, 0u, PUP_DOWN);
    g_ext[0] = 0;              /* 按键另一端接 VDD；松开时靠内部下拉，不是悬空 */
    sim_update_idr();
    printf("\n[配置 3] PA0 = 输入 + 内部下拉（按键接 VDD，按下读到 1）\n");
    describe_pin(0u);

    /* --- PA9：复用功能 AF1 = USART1_TX --- */
    PUT2(REG_MODER, 9u, MODE_AF);
    /* PA8 用 AFRH[3:0]，PA9 用 AFRH[7:4]：AFRn 的偏移 = (n-8)*4 */
    REG_AFRH = (REG_AFRH & ~(0xFu << 4u)) | (1u << 4u);  /* PA9 = AF1 = USART1_TX */
    PUT2(REG_OSPEEDR, 9u, SPD_FAST);
    OT_OD_CLR(9u);                          /* AF 输出在芯片内部仍是推挽结构 */
    g_af_drive[9] = 2u;                     /* 模型：USART1_TX 空闲态由外设驱动为高 */
    sim_update_idr();
    printf("\n[配置 4] PA9 = 复用功能 AF1（USART1_TX，模型里空闲态为高）\n");
    describe_pin(9u);
    printf("  注意：模式选 0b10 只是「把引脚交给外设」，输出什么电平由 USART 的\n");
    printf("  移位寄存器决定，你写 ODR 一点用都没有。AF 编号查数据手册的\n");
    printf("  Alternate functions 表，抄错 AF 号 = 引脚静默不动。\n");

    sim_update_idr();
    dump_regs("四种配置完成后");
}

/* ==================== 演示二：100 tick 固件主循环 ==================== */
/* 按键抖动模型：按下/松开后 3 个 tick 内电平乱跳，之后稳定 */
struct ButtonModel {
    bool pressed;      /* 物理状态：是否被按下 */
    int  edge_tick;    /* 发生跳变的 tick      */
    bool raw_level;    /* 引脚上实际看到的电平 */
};
static struct ButtonModel g_btn = { false, -100, false };

static void button_model_update(uint32_t tick)
{
    if (tick == 20u)  { g_btn.pressed = true;  g_btn.edge_tick = (int)tick; }
    if (tick == 45u)  { g_btn.pressed = false; g_btn.edge_tick = (int)tick; }
    if (tick == 70u)  { g_btn.pressed = true;  g_btn.edge_tick = (int)tick; }
    if (tick == 92u)  { g_btn.pressed = false; g_btn.edge_tick = (int)tick; }

    if ((int)tick - g_btn.edge_tick < 3) {
        /* 抖动期：电平每 tick 翻一次，这就是机械触点弹跳 */
        g_btn.raw_level = (((int)tick - g_btn.edge_tick) % 2 == 1)
                              ? g_btn.pressed : !g_btn.pressed;
    } else {
        g_btn.raw_level = g_btn.pressed;
    }
    g_ext[0] = g_btn.raw_level ? 2u : 0u;   /* 按下=接到 VDD；松开=悬空（靠下拉回 0） */
}

static void demo_main_loop(void)
{
    hr("演示二：100 个 tick 固件主循环（LED 每 50ms 闪、按键软件消抖）");
    printf("tick | 原始 | 消抖 | 事件        | 寄存器快照\n");
    printf("-----+------+------+-------------+------------------------------\n");

    uint32_t led_on_tick = 0u;
    uint32_t stable = 0u, last_raw = 0u, stable_cnt = 0u, press_cnt = 0u;
    const uint32_t DEBOUNCE_TICKS = 3u;     /* 连续 3 次一致才认（1ms 采样 = 3ms 窗口） */

    for (g_tick = 1u; g_tick <= 100u; ++g_tick) {
        button_model_update(g_tick);
        sim_update_idr();

        /* LED：用 BSRR 翻转，永远不要用 ODR 的读-改-写 */
        if (g_tick - led_on_tick >= 50u) {
            led_on_tick = g_tick;
            if ((REG_ODR >> 5) & 1u) bsrr_write(1u << (5u + 16u));  /* 复位 PA5 */
            else                      bsrr_write(1u << 5u);         /* 置位 PA5 */
        }

        /* 按键消抖：采样值和上次一样就累加，不一样就重新数 */
        const uint32_t raw = (REG_IDR >> 0) & 1u;
        const char *event = "-";
        if (raw == last_raw) {
            if (++stable_cnt >= DEBOUNCE_TICKS && stable != raw) {
                stable = raw;
                event  = raw ? ">>> 确认按下" : "<<< 确认松开";
                if (raw != 0u) ++press_cnt;
            }
        } else {
            stable_cnt = 0;
        }
        last_raw = raw;

        if (g_tick % 10u == 0u || event[0] == '>') {
            printf("%4u |   %" PRIu32 "  |   %" PRIu32
                   "  | %-11s | ODR=%08" PRIX32 " IDR=%08" PRIX32 "\n",
                   g_tick, raw, stable, event, REG_ODR, REG_IDR);
        }
    }
    printf("-----+------+------+-------------+------------------------------\n");
    printf("消抖后有效按下次数 = %" PRIu32 "（物理按了 2 次，触点弹跳全被吃掉）\n", press_cnt);
    printf("老师傅：不做消抖，一次按下能给你报出 5~8 个事件，菜单「按一下跳三格」\n");
    printf("        就是这么来的。消抖要么硬件 RC + 施密特，要么软件「连续 N 次一致」，\n");
    printf("        别用 ISR 里 delay_ms(20) 那种土办法——那叫在急诊室里睡大觉。\n");
}

/* ==================== 演示三：ODR = 1<<3 事故 ==================== */
static void demo_odr_clobber(void)
{
    hr("演示三：经典事故 —— GPIOA_ODR = (1u << 3) 把整个端口别的引脚全清了");

    /* 先立好现场：LED 亮、SDA 释放、TX 空闲高（都用 BSRR，这是规范写法） */
    bsrr_write(1u << 5);
    bsrr_write(1u << 6);
    bsrr_write(1u << 9);
    sim_update_idr();
    printf("事发前 ODR = 0x%08" PRIX32 "  → PA5=%u（LED 亮）PA6=%u（SDA 释放）PA9=%u（TX 空闲高）\n",
           REG_ODR, (REG_ODR >> 5) & 1u, (REG_ODR >> 6) & 1u, (REG_ODR >> 9) & 1u);
    print_bin32("事发前", REG_ODR);

    printf("\n[错误写法] GPIOA_ODR = (1u << 3);   // 我只想开一下 PA3 的继电器\n");
    REG_ODR = (1u << 3);
    sim_update_idr();
    printf("事发后 ODR = 0x%08" PRIX32 "  → PA5=%u（LED 灭了）PA6=%u（SDA 被拉低=I2C 卡死）PA9=%u（TX 拉低=对端收到 BREAK）\n",
           REG_ODR, (REG_ODR >> 5) & 1u, (REG_ODR >> 6) & 1u, (REG_ODR >> 9) & 1u);
    print_bin32("事发后", REG_ODR);
    printf("  一行代码，16 个引脚被你重写了 15 个。\n");
    printf("  C 里这三行的区别必须刻进骨头：\n");
    printf("    REG_ODR  =  (1u << 3);   整字覆盖 —— 别的位全被写成 0（本行事故）\n");
    printf("    REG_ODR |= (1u << 3);    读-改-写 —— 只碰一位，但「读」有竞态（见演示四）\n");
    printf("    bsrr_write(1u << 3);     原子写   —— 没有「读」这一步，唯一正确姿势\n");

    printf("\n[正确写法] bsrr_write(1u << 3);     // 只碰 PA3，别的一个 bit 都不动\n");
    REG_ODR = 0u;
    bsrr_write(1u << 5);                    /* 恢复现场 */
    bsrr_write(1u << 6);
    bsrr_write(1u << 9);
    bsrr_write(1u << 3);                    /* 现在才是「只开 PA3」 */
    sim_update_idr();
    printf("恢复后 ODR = 0x%08" PRIX32 "  → PA3=%u PA5=%u PA6=%u PA9=%u  全都在，谁也没被误伤\n",
           REG_ODR, (REG_ODR >> 3) & 1u, (REG_ODR >> 5) & 1u,
           (REG_ODR >> 6) & 1u, (REG_ODR >> 9) & 1u);
    print_bin32("恢复后", REG_ODR);
    printf("  BSRR 的原子性来自硬件：一次总线写，内部组合逻辑直接置位/复位，\n");
    printf("  没有「读」这一步，所以不可能读到过期数据，也不可能覆盖别人。\n");
}

/* ============= 演示四：读-改-写被中断插队（竞态） ============= */
static void demo_rmw_race(void)
{
    hr("演示四：读-改-写竞态 —— 中断插队把你的写入吃掉了");

    REG_ODR = 0u;
    bsrr_write(1u << 5);
    sim_update_idr();
    printf("初始 ODR = 0x%08" PRIX32 "（PA5 亮）\n", REG_ODR);
    print_bin32("初始", REG_ODR);

    printf("\n任务 A 要开 PA3，写成 GPIOA_ODR |= (1u << 3); 它在总线上是三步：\n");
    const uint32_t snapshot = REG_ODR;                  /* 1) 读 */
    printf("  第 1 步 读：snapshot = 0x%08" PRIX32 "\n", snapshot);
    printf("  ---- 就在这个空隙，UART 中断进来了，它要点亮 PA7：----\n");
    REG_ODR |= (1u << 7);                               /* 中断插入 */
    printf("  中断里写：ODR = 0x%08" PRIX32 "（PA7 亮）\n", REG_ODR);
    print_bin32("中断后", REG_ODR);
    REG_ODR = snapshot | (1u << 3);                     /* 2) 或 3) 写回 */
    printf("  第 3 步 写回：ODR = snapshot | 0x8 = 0x%08" PRIX32 "\n", REG_ODR);
    print_bin32("写回后", REG_ODR);

    sim_update_idr();
    printf("\n结果：PA3=%u（A 点上了）  PA7=%u（中断点的那个被抹掉了）\n",
           (REG_ODR >> 3) & 1u, (REG_ODR >> 7) & 1u);
    printf("  现场症状：主循环一忙，中断里点的灯就灭一下；关掉 UART 中断就正常——\n");
    printf("  于是你去查 UART，查三天。解法：改用 BSRR（无读、天然原子），\n");
    printf("  或者 RMW 期间短关中断（PRIMASK），或者把该引脚只交给一个执行上下文。\n");

    printf("\n[同样逻辑改用 BSRR] 主循环 bsrr_write(1u<<3)，中断 bsrr_write(1u<<7)：\n");
    REG_ODR = 0u;
    bsrr_write(1u << 5);
    bsrr_write(1u << 3);                    /* 主循环 */
    bsrr_write(1u << 7);                    /* 中断插队：仍然是独立的一次总线写 */
    printf("  ODR = 0x%08" PRIX32 " → PA3=%u PA5=%u PA7=%u 三个都在，相安无事\n",
           REG_ODR, (REG_ODR >> 3) & 1u, (REG_ODR >> 5) & 1u, (REG_ODR >> 7) & 1u);
    print_bin32("BSRR 后", REG_ODR);
}

/* =============== 演示五：IDR 只读，写它没用 =============== */
static void demo_idr_readonly(void)
{
    hr("演示五：IDR 只读 —— 想靠写 IDR 控制引脚，等于对墙说话");

    g_ext[0] = 0u;                          /* 按键松开，下拉把 PA0 拉到 0 */
    sim_update_idr();
    printf("外部按键松开：IDR bit0 = %u\n", (REG_IDR >> 0) & 1u);
    printf("有人写了 GPIOA_IDR |= (1u << 0); 想「读回一个 1」……\n");
    REG_IDR |= (1u << 0);                   /* 软件模型允许你写，真芯片上这次写被忽略 */
    printf("  写完你读到 IDR bit0 = %u（模型让你写进去了，硬件上不会）\n",
           (REG_IDR >> 0) & 1u);
    sim_update_idr();                       /* 外设模型立刻按真实电路刷新 */
    printf("  下一个 tick 外设按真实电路刷新：IDR bit0 = %u ← 打回原形\n",
           (REG_IDR >> 0) & 1u);
    printf("  IDR 是输入缓冲器拍的「照片」，只反映引脚上的真实电平。\n");
    printf("  要输出请用 ODR/BSRR；要读「我自己输出成什么样」请读 ODR，别读 IDR——\n");
    printf("  开漏引脚被外部拉住时，ODR 和 IDR 是两个不一样的值，这个差异就是现场线索。\n");
}

/* ============================== main ============================== */
int main(void)
{
    /* 五个演示排成函数指针表，加一段演示只要往表里添一行 */
    static void (*const demos[])(void) = {
        demo_configure, demo_main_loop, demo_odr_clobber, demo_rmw_race, demo_idr_readonly
    };

    printf("############################################################\n");
    printf("#   第 4 关 · 01 GPIO 寄存器软件模型（PC 上跑的假 STM32）   #\n");
    printf("############################################################\n");

    for (size_t i = 0; i < sizeof demos / sizeof demos[0]; ++i) demos[i]();

    hr("收尾：三个自测问题");
    printf("1) 现在 MODER = 0x%08" PRIX32 "：口算 PA5 的 2 位是 0b 几、在 bit 几。\n", REG_MODER);
    printf("   答：PA5 → bit11:10 = 0b01（通用输出）。算不出就去玩 register_playground.html。\n");
    printf("2) 把 PA6 从开漏改成推挽会怎样？\n");
    printf("   答：I2C 从机拉低期间你也输出高 → 跟上管直通、跟从机对打。\n");
    printf("      示波器上看到 SDA 高电平被压在 1V 左右，就是这儿。I2C 引脚永远开漏。\n");
    printf("3) LED 直接接 PA 口，串多大电阻？\n");
    printf("   答：(3.3-2.0)V / 2mA ≈ 650Ω，取 680Ω 稳妥。手册在 8mA 下考核 VOL/VOH，\n");
    printf("      长期别超过 8~10mA；25mA 是绝对最大（灾难值，不是工作值），\n");
    printf("      整个端口合计还有 150mA 上限，还要看 VDD 源的总预算。\n");
    printf("      低速档驱动能力最小，直推 LED 请把 OSPEEDR 调高一档。\n");
    return 0;
}
