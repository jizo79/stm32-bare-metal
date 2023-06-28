#include <inttypes.h>
#include <stdbool.h>

#define F302R8

typedef volatile uint32_t config_register;

// RM0365 10 (p.158)
struct gpio
{
    config_register MODER;      //port x configuration
    config_register OTYPER;     //output type
    config_register OSPEEDR;    //output speed
    config_register PUPDR;      //pull-up/pull-down
    config_register IDR;        //input data
    config_register ODR;        //output data
    config_register BSRR;       //bit set/reset
    config_register LCKR;       //configuration locl
    config_register AFR[2];     //alternate function low/high
};

// RM0365 9 t.29 (p.155)
struct rcc 
{
    config_register CR;
    config_register CFGR;
    config_register CIR;
    config_register APB2RSTR;
    config_register APB1RSTR;
    config_register AHBENR;
    config_register APB2ENR;
    config_register APB1ENR;
    config_register BDCR;
    config_register CSR;
    config_register AHBRSTR;
    config_register CFGR2;
    config_register CFGR3;
};

// ARM Cortex-M4 core specific
// Cortex-M4 Devices Generic User Guide
// DUI0553 4.4 p.4-33
struct systick 
{
    config_register CTRL;   // SYST_CSR - control and status
    config_register LOAD;   // SYST_RVR - reload value
    config_register VAL;    // SYST_CVR - current value
    config_register CALIB;  // SYST_CALIB - calibration value
};

// for STM32F302R8 boundaries are:
// RM0365 t.4 (p.58)
#ifdef F302R8
    #define ADDR_GPIO_A 0x48000000
    #define ADDR_RCC    0x40021000
#endif //#ifdef F302R8

// ARM Cortex-M4 core specific
#define ADDR_SYSTICK 0xE000E010

#define GPIO_A ((struct gpio *) ADDR_GPIO_A)
#define RCC ((struct rcc *) ADDR_RCC)
#define SYSTICK ((struct systick *) ADDR_SYSTICK)

enum {
    GPIO_MODE_INPUT = 0,
    GPIO_MODE_OUTPUT = 1,
    GPIO_MODE_ALTERNATE_FUNCTION = 2,
    GPIO_MODE_ANALOG = 3
} GPIO_MODE;

#define GPIO(bank) ((struct gpio *) (ADDR_GPIO_A + 0x400 * (bank)))
#define BIT(x) (0x1UL << (x))
#define PIN(bank, num) ((((bank) - 'A') << 8) | num)
#define PINNO(pin) ((uint8_t)pin & 255)
#define PINBANK(pin) (pin >> 8)

static volatile uint32_t s_ticks;
void SysTick_Handler(void)
{
    s_ticks++;
}

static inline void systick_init(uint32_t ticks)
{
    // check if ticks is bounded to 24bits
    if((ticks - 1) > 0xffffff)
    {
        return;
    }
    SYSTICK->LOAD = ticks - 1;
    SYSTICK->VAL = 0;
    SYSTICK->CTRL = BIT(0) | BIT(1) | BIT(2);
    RCC->APB2ENR |= BIT(0);
}
static inline void gpio_set_mode(uint16_t pin, uint8_t mode)
{
    struct gpio * gpio = GPIO(PINBANK(pin));
    uint8_t n = PINNO(pin);
    gpio->MODER &= ~(0x11UL << (n * 2)); // reset existing settings
    gpio->MODER |= (mode & 0x11) << (n * 2);
}

static inline void gpio_write(uint16_t pin, bool val)
{
    struct gpio * gpio = GPIO(PINBANK(pin));
    gpio->BSRR = (0x1UL << PINNO(pin)) << (val ? 0 : 16);
}

static inline void spin(volatile uint32_t count)
{
    while (count--)
    {
        (void)0;
    }
}

bool timer_expired(uint32_t *expiration_time, uint32_t period, uint32_t current_time)
{
    if(current_time + period < *expiration_time)
    {
        *expiration_time = 0;
    }

    if(*expiration_time == 0)
    {
        *expiration_time = current_time + period;
    }

    if(*expiration_time > current_time)
    {
        return false;
    }
    
    *expiration_time = (current_time - *expiration_time) > period ?
                        current_time + period :
                        *expiration_time + period;
    return true;
}

int main(void)
{
    uint16_t led = PIN('B', 13);
    RCC->AHBENR |= BIT(18);
    systick_init(8000000 / 1000); // tick every 1 ms
    gpio_set_mode(led, GPIO_MODE_OUTPUT);
    uint32_t timer = 0;
    uint32_t period = 1000;
    for(;;)
    {
        
        if(timer_expired(&timer, period, s_ticks))
        {
            static bool on;
            gpio_write(led, on);
            on = !on;
        }
        /*
        gpio_write(led, true);
        spin(99999);
        gpio_write(led, false);
        spin(999999);
        */
    }
    return 0;
}

__attribute__((naked, noreturn)) void _reset(void) 
{
    extern long _sbss;
    extern long _ebss;
    extern long _sdata;
    extern long _edata;
    extern long _sidata;
    //memset .bss to zero, and copy .data section to RAM region
    for(long * src = &_sbss; src < &_ebss; src++)
    {
        *src = 0;
    }
    for(long * src = &_sdata, * dst = &_sidata; src < &_edata;)
    {
        *src++ = *dst++;
    }
    main();
    for (;;)
    {
        (void)0;
    }
}

extern void _estack(void);

// RM0365 t.40 (p.213)
// for STM32F302R8 66 maskable interrupt channels + 16 Cortex-M4 interrupt lines

#ifdef F302R8
    #define MASK_INTER  66
    #define CM4_INTER   16
#endif //#ifdef F302R8

__attribute__((section(".vectors"))) void (*const tab[MASK_INTER + CM4_INTER]) (void) = {
    _estack, _reset, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, SysTick_Handler
};