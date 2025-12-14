#include <inttypes.h>
#include <stdbool.h>

#define BIT(x) (1UL << (x))
#define PIN(bank,num) ((((bank) - 'A') << 8) | (num))
#define PINNO(pin) (pin & 255)
#define PINBANK(pin) (pin >> 8)

struct gpio{
	volatile uint32_t MODER, OTYPER, OSPEEDR, PUPDR, IDR, ODR, BSRR, LCKR, AFR[2];
};
#define GPIO(bank) ((struct gpio *)(0x48000000 + 0x400 * (bank)))

struct rcc {
    volatile uint32_t CR;
    volatile uint32_t ICSCR;
    volatile uint32_t CFGR;
    volatile uint32_t PLLCFGR;
    volatile uint32_t PLLSAI1CFGR;
    uint32_t RESERVED0;
    volatile uint32_t CIER;
    volatile uint32_t CIFR;
    volatile uint32_t CICR;
    uint32_t RESERVED1;
    volatile uint32_t AHB1RSTR;
    volatile uint32_t AHB2RSTR;
    volatile uint32_t AHB3RSTR;
    uint32_t RESERVED2;
    volatile uint32_t APB1RSTR1;
    volatile uint32_t APB1RSTR2;
    volatile uint32_t APB2RSTR;
    uint32_t RESERVED3;
    volatile uint32_t AHB1ENR;
    volatile uint32_t AHB2ENR;
    volatile uint32_t AHB3ENR;
    uint32_t RESERVED4;
    volatile uint32_t APB1ENR1;
    volatile uint32_t APB1ENR2;
    volatile uint32_t APB2ENR;
};
#define RCC ((struct rcc*) 0x40021000)

struct systick {
	volatile uint32_t STCSR, STRVR, STCVR, STCR;
};
#define SYSTICK ((struct systick*) 0xE000E010)

struct lp_uart {
	volatile uint32_t LPUART_CR1, LPUART_CR2, LPUART_CR3, LPUART_BRR, RESERVED, RESERVED, LPUART_RQR, LPUART_ISR, LPUART_ICR, LPUART_RDR, LPUART_TDR, LPUART_PRESC
};

#define LP_UART ((struct lp_uart*) 0x40008000)


//enum values datasheet specific
enum { GPIO_MODE_INPUT, GPIO_MODE_OUTPUT, GPIO_MODE_AF, GPIO_MODE_ANALOG };

static inline void gpio_set_mode(uint16_t pin, uint8_t mode) {
	struct gpio* gpio = GPIO(PINBANK(pin));
	int n = PINNO(pin);
	gpio->MODER &= ~(3U <<(n*2));
	gpio->MODER |= (mode & 3) << (n * 2);
	RCC->AHB2ENR |= BIT(PINBANK(pin));
}
static inline gpio_set_af(uint16_t pin, uint8_t af_num){
	struct gpio* gpio = GPIO(PINBANK(pin));
	int n = PINNO(pin);
	gpio->AFR[n >> 3] &= ~(15UL ((n & 7) * 4));
	gpio->AFR[n >> 3] |= ((uint32_t) af_num) << ((n & 7) * 4);

}

#define FREQ 16000000 // CPU Frequency 16MHz
static inline void uart_init(struct uart* uart, unsigned long baud) {
	uint8_t af = 7;
	uint16_t rx = 0, tx = 0;	
	RCC->APB1ENR2 |= BIT(4);

}

static inline void gpio_write(uint16_t pin, bool val){
	struct gpio* gpio = GPIO(PINBANK(pin));
 	gpio->BSRR = (1U << PINNO(pin)) << (val ? 0 : 16);

}
static inline void spin(volatile uint32_t count) {
	while (count--) (void) 0;
}

static inline void systick_init(uint32_t ticks) {
	if((ticks - 1) > 0xFFFFFF) return;
	SYSTICK->STRVR = ticks - 1;
	SYSTICK->STCVR = 0;
	SYSTICK->STCSR = BIT(0) | BIT(1) | BIT(2); //use internal clock source, enable systick exception, enable systick
}

static volatile uint32_t s_ticks;
void SysTick_Handler(void) {
	s_ticks++;
}

bool timer_expired(uint32_t* t, uint32_t prd, uint32_t now) {
	if(now + prd < *t) *t = 0; //reset timer if the time has wrapped
	if(*t == 0) *t = now + prd; // set the timer if first poll
	if(*t > now) return false; // not expired yet
	*t = (now - *t) > prd ? now + prd : *t + prd; // if timer expired return true and add the period to the timer
	return true;

}

int main(void) {
	uint16_t led = PIN('A',5);
	systick_init(16000000 / 1000);
	gpio_set_mode(led, GPIO_MODE_OUTPUT);
	uint32_t timer, period = 5000;
 	for(;;){
		if(timer_expired(&timer, period, s_ticks)) {
			static bool on;
			gpio_write(led,on);
			on = !on;
		}
	}
	return 0;

}

__attribute__((naked, noreturn)) void _reset(void){
	//memset .bss to zero and copy the .data section to RAM 
	extern long _sbss, _ebss, _sdata, _edata, _sidata;
	for (long *dst = &_sbss; dst < &_ebss; dst++) *dst = 0;
	for (long *dst = &_sdata, *src = &_sidata; dst < &_edata;) *dst++ = *src++;
	main();
	for(;;) (void) 0;
}
extern void _estack(void);  // Defined in link.ld

// 16 standard and 91 STM32-specific handlers
__attribute__((section(".vectors"))) void (*const tab[16 + 91])(void) = {
  _estack, _reset, 0,0,0,0,0,0,0,0,0,0,0,0,0,SysTick_Handler
};
