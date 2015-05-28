#define ARRAY_AND_SIZE(x)	(x), ARRAY_SIZE(x)

struct sys_timer;

extern struct sys_timer pxa168_timer;
extern void __init pxa168_init_irq(void);

extern void __init icu_init_irq(void);
extern void __init pxa_map_io(void);
#define REG32(x)       (*(volatile unsigned long *)(x))
#define RIPC0_STATUS   REG32(0xfe03D000)
extern void release_RIPC(void);
extern void get_RIPC(void);
