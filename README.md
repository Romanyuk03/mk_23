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

#include <libopencm3/stm32/f4/rcc.h>
#include <libopencm3/stm32/f4/gpio.h>
#include <libopencm3/stm32/f4/timer.h>
#include <libopencm3/stm32/f4/nvic.h>
#include <libopencm3/stm32/f4/usart.h>
#include <stdio.h>

volatile uint32_t pulse_width = 0;

void tim2_isr(void) {
    if (TIM_SR(TIM2) & TIM_SR_CC1IF) {
        pulse_width = TIM_CCR1(TIM2);
        TIM_SR(TIM2) &= ~TIM_SR_CC1IF;
    }
}

void usart_setup(void) {
    rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_USART2);
    rcc_peripheral_enable_clock(&RCC_AHB1ENR, RCC_GPIOE);

    // Настройка PA2 как альтернативной функции (USART2_TX)
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO2);
    gpio_set_af(GPIOA, GPIO_AF7, GPIO2); // USART2_TX

    // Настройка USART
    USART_CR1(USART2) &= ~USART_CR1_UE; // Отключаем USART
    USART_BRR(USART2) = 115200; // Настройка скорости 115200
    USART_CR1(USART2) |= USART_CR1_TE; // Включаем передатчик
    USART_CR1(USART2) |= USART_CR1_UE; // Включаем USART
}

void usart_send_string(const char *str) {
    while (*str) {
        while (!(USART_SR(USART2) & USART_SR_TXE)); // Ждем, пока TXE не станет 1
        USART_DR(USART2) = *str++;
    }
}

void setup_timer(void) {
    rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_TIM2);
    rcc_peripheral_enable_clock(&RCC_AHB1ENR, RCC_GPIOE);

    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO0);
    gpio_set_af(GPIOA, GPIO_AF1, GPIO0); // TIM2_CH1

    TIM_CR1(TIM2) &= ~TIM_CR1_CEN; // Отключаем таймер
    TIM_PSC(TIM2) = 83; // Делитель (для 1мс при 84MHz)
    TIM_ARR(TIM2) = 0xFFFF; // Автозагрузка

    TIM_CCER(TIM2) |= TIM_CCER_CC1E; // Включаем захват на канале 1
    TIM_SMCR(TIM2) |= TIM_SMCR_SMS_GM; // Режим захвата

    TIM_DIER(TIM2) |= TIM_DIER_CC1IE; // Разрешаем прерывание от канала 1
    nvic_enable_irq(NVIC_TIM2_IRQ); // Разрешаем прерывание в NVIC

    TIM_CR1(TIM2) |= TIM_CR1_CEN; // Включаем таймер
}

int main(void) {
    usart_setup(); // Настройка UART
    setup_timer(); // Настройка таймера

    while (1) {
        char buffer[50];
        // Преобразуем pulse_width в строку и отправляем по UART
        snprintf(buffer, sizeof(buffer), "Pulse Width: %lu\n", pulse_width);
        usart_send_string(buffer);
        
        // Добавим небольшую задержку, чтобы избежать слишком частого вывода
        for (volatile int i = 0; i < 100000; i++);
    }

    return 0;
}

//////////////////////////////////////////////////////////////

#include <libopencm3/stm32/f4/rcc.h>
#include <libopencm3/stm32/f4/gpio.h>
#include <libopencm3/stm32/f4/timer.h>
#include <libopencm3/stm32/f4/nvic.h>
#include <libopencm3/stm32/f4/usart.h>
#include <stdio.h>

volatile uint32_t pulse_width = 0;

void tim2_isr(void) {
    if (TIM_SR(TIM2) & TIM_SR_CC1IF) {
        pulse_width = TIM_CCR1(TIM2);
        TIM_SR(TIM2) &= ~TIM_SR_CC1IF;
    }
}

void usart_setup(void) {
    rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_USART2);
    rcc_peripheral_enable_clock(&RCC_AHB1ENR, RCC_GPIOE);

    // Настройка PA2 как альтернативной функции (USART2_TX)
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO2);
    gpio_set_af(GPIOA, GPIO_AF7, GPIO2); // USART2_TX

    // Настройка USART
    USART_CR1(USART2) &= ~USART_CR1_UE; // Отключаем USART
    USART_BRR(USART2) = 115200; // Настройка скорости 115200
    USART_CR1(USART2) |= USART_CR1_TE; // Включаем передатчик
    USART_CR1(USART2) |= USART_CR1_UE; // Включаем USART
}

void usart_send_string(const char *str) {
    while (*str) {
        while (!(USART_SR(USART2) & USART_SR_TXE)); // Ждем, пока TXE не станет 1
        USART_DR(USART2) = *str++;  ///это отправка в юсарт можно ее использовать 
    }
}

void setup_timer(void) {
    rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_TIM2);
    rcc_peripheral_enable_clock(&RCC_AHB1ENR, RCC_GPIOE);

    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO0);
    gpio_set_af(GPIOA, GPIO_AF1, GPIO0); // TIM2_CH1

    TIM_CR1(TIM2) &= ~TIM_CR1_CEN; // Отключаем таймер
    TIM_PSC(TIM2) = 83; // Делитель (для 1мс при 84MHz)
    TIM_ARR(TIM2) = 0xFFFF; // Автозагрузка

    TIM_CCER(TIM2) |= TIM_CCER_CC1E; // Включаем захват на канале 1
    TIM_SMCR(TIM2) |= TIM_SMCR_SMS_GM; // Режим захвата

    TIM_DIER(TIM2) |= TIM_DIER_CC1IE; // Разрешаем прерывание от канала 1
    nvic_enable_irq(NVIC_TIM2_IRQ); // Разрешаем прерывание в NVIC

    TIM_CR1(TIM2) |= TIM_CR1_CEN; // Включаем таймер
}

int main(void) {
    usart_setup(); // Настройка UART
    setup_timer(); // Настройка таймера

    while (1) {    ////дописать включение светодиода   
        char buffer[50];
        // Преобразуем pulse_width в строку и отправляем по UART
        snprintf(buffer, sizeof(buffer), "Pulse Width: %lu\n", pulse_width);
        USART_DR(USART2) = 85;  /////вместо этого написать отправку числа 85 в юсарт 
        
        // Добавим небольшую задержку, чтобы избежать слишком частого вывода
        for (volatile int i = 0; i < 100000; i++);
    }

    return 0;
}

//////////////////////////////////////////////////////////////////////////////////////////////////


// #include <libopencm3/stm32/f4/rcc.h>
// #include <libopencm3/stm32/f4/gpio.h>
// #include <libopencm3/stm32/f4/timer.h>
// #include <libopencm3/stm32/f4/nvic.h>
// #include <libopencm3/stm32/f4/usart.h>
// #include <stdio.h>

// volatile uint32_t pulse_width = 0;

// void tim2_isr(void) {
//     if (TIM_SR(TIM2) & TIM_SR_CC1IF) {
//         pulse_width = TIM_CCR1(TIM2);
//         TIM_SR(TIM2) &= ~TIM_SR_CC1IF;
//     }
// }

// void usart_setup(void) {
//     // Включаем тактирование для GPIOA и USART2
//     rcc_periph_clock_enable(RCC_GPIOA);
//     rcc_periph_clock_enable(RCC_USART2);

//     // Настраиваем пины PA2 (TX) и PA3 (RX) для USART2
//     gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO2 | GPIO3);
//     gpio_set_af(GPIOA, GPIO_AF7, GPIO2); // TX
//     gpio_set_af(GPIOA, GPIO_AF7, GPIO3); // RX

//     // Настраиваем USART2
//     usart_set_baudrate(USART2, 115200); // Устанавливаем скорость бит/с 
//     usart_set_databits(USART2, 8); // 8 бит данных
//     usart_set_stopbits(USART2, USART_STOPBITS_1); // 1 стоп-бит
//     usart_set_mode(USART2, USART_MODE_TX_RX); // Режим TX и RX
//     usart_set_parity(USART2, USART_PARITY_NONE); // Без четности
//     usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE); // Без управления потоком

//     usart_enable_rx_interrupt(USART2);
//     nvic_enable_irq(NVIC_USART2_IRQ);
    
//     // Включаем USART2
//     usart_enable(USART2);
// }

// void usart_send_string(const char *str) {
//     while (*str) {
        
//         while (!(USART_SR(USART2) & USART_SR_TXE)); // Ждем, пока TXE не станет 1
//         USART_DR(USART2) = *str++;  // Отправка символа
//     }

//     gpio_toggle(GPIOD, GPIO15); // Настройка мигания
// }

// // Инициализация всех светодиодов 
// void setup_LED(void){
//     rcc_periph_clock_enable(RCC_GPIOD);
//     gpio_mode_setup(GPIOD, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO15 | GPIO14 | GPIO13 | GPIO12);
// }




// void init_timer(void) {
//     // Включаем тактирование для TIM2
//     rcc_periph_clock_enable(RCC_TIM2);
//     rcc_periph_clock_enable(RCC_AHB1ENR); // Включаем тактирование для GPIOA

//     // Настраиваем PA0 как альтернативную функцию (TIM2_CH1)
//     gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO0);
//     gpio_set_af(GPIOA, GPIO_AF1, GPIO0); // TIM2_CH1

//     // Настраиваем таймер
//     timer_set_prescaler(TIM2, 8399); // Предделитель (84 МГц / (8399 + 1) = 10 кГц)
//     timer_set_period(TIM2, 0xFFFF); // Устанавливаем период таймера

//     // Настройка режима захвата
//     TIM_CCER(TIM2) |= TIM_CCER_CC1E; // Включаем захват на канале 1
//     TIM_SMCR(TIM2) |= TIM_SMCR_SMS_GM; // Режим захвата (в данном случае - режим захвата на фронте)

//     // Разрешаем прерывание от канала 1
//     TIM_DIER(TIM2) |= TIM_DIER_CC1IE; 
//     nvic_enable_irq(NVIC_TIM2_IRQ); // Разрешаем прерывание в NVIC

//     timer_enable_counter(TIM2); // Запускаем таймер
// }

// void setup_timer(void) {
//     rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_TIM2);
//     rcc_peripheral_enable_clock(&RCC_AHB1ENR, RCC_GPIOE);

//     gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO0);
    
//     gpio_set_af(GPIOA, GPIO_AF1, GPIO0); // TIM2_CH1

//     TIM_CR1(TIM2) &= ~TIM_CR1_CEN; // Отключаем таймер
//     TIM_PSC(TIM2) = 83; // Делитель (для 1мс при 84MHz)
//     TIM_ARR(TIM2) = 0xFFFF; // Автозагрузка

//     TIM_CCER(TIM2) |= TIM_CCER_CC1E; // Включаем захват на канале 1
//     TIM_SMCR(TIM2) |= TIM_SMCR_SMS_GM; // Режим захвата

//     TIM_DIER(TIM2) |= TIM_DIER_CC1IE; // Разрешаем прерывание от канала 1
//     nvic_enable_irq(NVIC_TIM2_IRQ); // Разрешаем прерывание в NVIC

//     TIM_CR1(TIM2) |= TIM_CR1_CEN; // Включаем таймер
// }


// void read(void){
// tim2_isr();
// char buffer[150];

// snprintf(buffer, sizeof(buffer), "Pulse Width: %lu\n", pulse_width);
// usart_send_string(buffer);  // Отправка ширины импульса по UART
        

// gpio_toggle(GPIOD, GPIO14);

// for (volatile int i = 0; i < 100000; i++); // Задержка


// // usart_recv(USART2)

// // snprintf(number_buffer, sizeof(number_buffer), " %d\n",52); // Форматируем число в строку
        
// // usart_send_string(number_buffer);  // Отправка числа 85 в виде строки
        
// // for (volatile int i = 0; i < 100000; i++); // Задержка


// }

// int main(void) {
//     usart_setup(); // Настройка UART
//     init_timer(); // Настройка таймера
//     setup_LED();   // Инициализация светодиодов
//     tim2_isr();

//     while (1) {
//         read();
//     }

//     return 0;
// }

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include <libopencm3/stm32/f4/rcc.h>
#include <libopencm3/stm32/f4/gpio.h>
#include <libopencm3/stm32/f4/timer.h>
#include <libopencm3/stm32/f4/nvic.h>
#include <libopencm3/stm32/f4/usart.h>
#include <stdio.h>

volatile uint32_t pulse_width = 0;

void tim2_isr(void) {
    if (TIM_SR(TIM2) & TIM_SR_CC1IF) {
        pulse_width = TIM_CCR1(TIM2); // Считываем значение захвата
        TIM_SR(TIM2) &= ~TIM_SR_CC1IF; // Сбрасываем флаг прерывания
    }
}

void usart_setup(void) {
    // Включаем тактирование для GPIOA и USART2
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_USART2);

    // Настраиваем пины PA2 (TX) и PA3 (RX) для USART2
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO2 | GPIO3);
    gpio_set_af(GPIOA, GPIO_AF7, GPIO2); // TX
    gpio_set_af(GPIOA, GPIO_AF7, GPIO3); // RX

    // Настраиваем USART2
    usart_set_baudrate(USART2, 115200); // Устанавливаем скорость бит/с 
    usart_set_databits(USART2, 8); // 8 бит данных
    usart_set_stopbits(USART2, USART_STOPBITS_1); // 1 стоп-бит
    usart_set_mode(USART2, USART_MODE_TX_RX); // Режим TX и RX
    usart_set_parity(USART2, USART_PARITY_NONE); // Без четности
    usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE); // Без управления потоком

    usart_enable_rx_interrupt(USART2);
    nvic_enable_irq(NVIC_USART2_IRQ);
    
    // Включаем USART2
    usart_enable(USART2);
}

void usart_send_string(const char *str) {
    // Ждем, пока TXE не станет 1
        USART_DR(USART2) = *str++;  // Отправка символа
 

    gpio_toggle(GPIOD, GPIO15); // Настройка мигания
}

// Инициализация всех светодиодов 
void setup_LED(void){
    rcc_periph_clock_enable(RCC_GPIOD);
    gpio_mode_setup(GPIOD, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO15 | GPIO14 | GPIO13 | GPIO12);
}

void init_timer(void) {
    // Включаем тактирование для TIM2
    rcc_periph_clock_enable(RCC_TIM2);
    
    // Настраиваем PA0 как альтернативную функцию (TIM2_CH1)
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO0);
    gpio_set_af(GPIOA, GPIO_AF1, GPIO0); // TIM2_CH1

    // Настраиваем таймер
    timer_set_prescaler(TIM2, 8399); // Предделитель (84 МГц / (8399 + 1) = 10 кГц)
    timer_set_period(TIM2, 0xFFFF);   // Устанавливаем период таймера

    // Настройка режима захвата
    TIM_CCER(TIM2) |= TIM_CCER_CC1E;   // Включаем захват на канале 1
    TIM_SMCR(TIM2) |= TIM_SMCR_SMS_GM;  // Режим захвата на фронте

    // Разрешаем прерывание от канала 1
    TIM_DIER(TIM2) |= TIM_DIER_CC1IE; 
    nvic_enable_irq(NVIC_TIM2_IRQ);     // Разрешаем прерывание в NVIC

    timer_enable_counter(TIM2);          // Запускаем таймер
}

void read(void){
    char buffer[150];
    tim2_isr();
   
    snprintf(buffer, sizeof(buffer), "Pulse Width: %lu\n", pulse_width);
    usart_send_string(buffer);            // Отправка ширины импульса по UART

    gpio_toggle(GPIOD, GPIO14);           // Мигаем светодиодом на GPIOD14

    for (volatile int i = 0; i < 100000; i++); // Задержка
}

int main(void) {
    usart_setup();   // Настройка UART
    init_timer();   // Настройка таймера
    setup_LED();     // Инициализация светодиодов

    while (1) {
        read();      // Чтение и отправка ширины импульса по UART
        for (volatile int i = 0; i < 100000; i++); // Задержка между отправками
    }

    return 0;
}



////////////////////////////////////////////////////////////////////////////////