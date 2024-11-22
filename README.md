# Микропроцессорная техника
### Автор: Романюк А.А

## лабораторные работы 

### Используемое ПО
1. arm-none-eabi-g++ --- кросс-компилятор.
1. make --- система сборки проекта.
1. cmake --- система управления проектом.
1. git --- система контроля версий.

uint8_t c{'a'};
Ring_buffer buf;
void setup () {
    rcc_periph_clock_enable(RCC_GPIOA);                          
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO2 | GPIO3);  
    gpio_set_af(GPIOA,GPIO_AF7, GPIO2 | GPIO3);                           

    rcc_periph_clock_enable(RCC_USART2);                    

    usart_set_baudrate(USART2, 19200);                       
    usart_set_databits(USART2, 8);                            
    usart_set_stopbits(USART2, USART_STOPBITS_1);            
    usart_set_parity(USART2, USART_PARITY_NONE);              
    usart_set_mode(USART2, USART_MODE_TX_RX);                 
    usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);   
    usart_enable(USART2);

    usart_enable_rx_interrupt(USART2);
    nvic_enable_irq(NVIC_USART2_IRQ);   

    usart_enable(USART2);

    rcc_periph_clock_enable(RCC_GPIOE);
    gpio_mode_setup(GPIOE, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO9 | GPIO11);




}


void loop () {
    
   // uint16_t c = usart_recv_blocking(USART2);
   //usart_send_blocking(USART2,c);
   // usart_send_blocking(USART2, 'H');
   // usart_send_blocking(USART2, 'E');
   // usart_send_blocking(USART2, 'L');
   // usart_send_blocking(USART2, 'P');
   // usart_send_blocking(USART2, '\n');
   // usart_send_blocking(USART2, '\r');
   
   if (!buf.empty()) {c = buf.get();}
   usart_send_blocking(USART2, c);
   for(volatile uint32_t i=0; i<200000; ++i);
   gpio_toggle(GPIOE, GPIO9);


}
//------------------------------------------------------------------------------------------------------------
int main () {
    setup();
    gpio_set(GPIOE, GPIO15);

    while (true) {
        loop();
    }
}
//-------------------------------------------------------------------------------------------------------------
void usart2_exti26_isr (void){

    USART_SR(USART2) &= ~(USART_SR_RXNE);
    buf.put(c = static_cast<uint8_t>(usart_recv(USART2)));
    gpio_toggle(GPIOE, GPIO11);

//USART_ISR(USART2) &= ~(0x20);
//[[maybe_unused]]
//static uint8_t c = static_cast<uint8_t>(usart_recv(USART2));
//gpio_toggle(GPIOE, GPIO11);

    
}

///////////////////////////////////////////////////////

#define F_CPU_MHz       8
#define BAUDRATE        115200

uint32_t u32tod(uint32_t num, uint8_t *s) {
    uint8_t *p = s + 11;
    uint32_t i = 0, cnt = 0;
    do {  *(--p) = num % 10 + '0';  num /= 10;  cnt++;  } while (num);
    for (; i<cnt; i++) { s[i] = p[i]; };  s[cnt++] = '\n';
    return cnt;
}

void uart_send_blocking(uint8_t *s, uint32_t cnt) {
    while (cnt--) {
        while (!(USART1->SR & USART_SR_TXE));
        USART1->DR = *s++;
    }
}

int main() {
    RCC->APB2ENR = RCC_APB2ENR_USART1EN | RCC_APB2ENR_IOPAEN | RCC_APB2ENR_TIM1EN;

    /* Pins; PA8 is TI1, pulled low; PA9 is UART TX */
    CONFIGURE_PIN(GPIOA, 8, I_PULLED);
    CONFIGURE_PIN(GPIOA, 9, O_ALT_PUSH_PULL);

    /* Timer 1; CC2 captures on falling TI1; timer reset on rising TI1 */
    #define CC1S_IC1_ON_TI1     (0b01<<0)     /* CC1 as input capture on TI1 */
    #define CC2S_IC2_ON_TI1     (0b10<<8)     /* CC2 as input capture also on TI1 */
    #define FILTER_IC1          (0b1111<<4)   /* Input 1 filter config = f_DTS/32, N=8 */
    #define FILTER_IC2          (0b1111<<12)  /* Input 2 filter config = f_DTS/32, N=8 */
    #define CC1EP_NOCAP_RISING  (0b00<<0)     /* CC1 polarity is positive; actual capture not enabled */
    #define CC2EP_CAP_FALLING   (0b11<<4)     /* CC2 polarity is negative; THIS IS OUR CAPTURE CHANNEL */
    #define SLAVEMODE_TRIG_TI1  (0b100<<4)    /* Timer is in slave mode, triggered by filtered/polarized TI1 (CC1) */
    #define SLAVEMODE_RESET     (0b100<<0)    /* Trigger resets the timer but doesn't stop it */

    TIM1->CCMR1 = FILTER_IC2 | CC2S_IC2_ON_TI1 | FILTER_IC1 | CC1S_IC1_ON_TI1;
    TIM1->CCER = CC2EP_CAP_FALLING | CC1EP_NOCAP_RISING;
    TIM1->SMCR = SLAVEMODE_TRIG_TI1 | SLAVEMODE_RESET;
    TIM1->PSC = F_CPU_MHz * 1000;   /* 1 ms resolution */
    TIM1->CR1 = 1;

    /* UART; transmission only */
    USART1->BRR = F_CPU_MHz * 1000000 / BAUDRATE;
    USART1->CR1 = USART_CR1_UE | USART_CR1_TE;

    uint8_t buf[11];
    uint32_t i, cnt, cap, old_cap = 0;

    while (1) {
        cap = TIM1->CCR2;
        if (cap != old_cap && cap > 10) {
            uart_send_blocking(buf, u32tod(cap, buf));
            old_cap = cap;
        }
    }
}

$ make
$ stty < /dev/ttyUSB0 115200
$ cat /dev/ttyUSB0
1546
3347
^C

$ base64 tim1capture.bin
AFAAIAkCACAPShBLmkII0A9Ig0IF0hL4ARsD+AEbmEL50QxLDEqTQgTSACED+AEbk0L70U/w4CMI
SAlJiGDD+PgtAPBHuAC/XAMAIFwDACBcAwAgXAMAIFwDACAAAgAgAO0A4HC1ACQSTgHxCwKGRqb7
AFPbCAPrgwyg60wAMDBf+oD8vvEJDyVGGEYE8QEEAvgBzevYRLFLHhgZAeAS+AHPA/gBz4NC+dEK
I6gcC1VwvQC/zczMzEmxBUoBRBNoGwb81RD4ATuIQlNg99FwRwA4AUBE9gQCHUuAtTAmT/LxJ0/w
RA5P9PpcASRFIELyCAGaYQAjF02EsNX4BCgi8A8CQvAIAsX4BCjV+AQoIvDwAkLwkALF+AQoD0oF
9TBVr2EuYsX4CODF+CjALGCQYNFgrGujQvzQCiz62QGpIEb/95D/AUYBqP/3tv8jRvDnABACQAAA
AUAAOAFA

////////////////////////////////////////////////////////