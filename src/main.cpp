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
    while (*str) {
        while (!(USART_SR(USART2) & USART_SR_TXE)); // Ждем, пока TXE не станет 1
        USART_DR(USART2) = *str++;  // Отправка символа
    }

    gpio_toggle(GPIOD, GPIO15); // Настройка мигания
}

// Инициализация всех светодиодов 
void setup_LED(void){
    rcc_periph_clock_enable(RCC_GPIOD);
    gpio_mode_setup(GPIOD, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO15 | GPIO14 | GPIO13 | GPIO12);
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
    setup_LED();   // Инициализация светодиодов

    while (1) {
        char number_buffer[20];
        snprintf(number_buffer, sizeof(number_buffer), "Number: %d\n", 85); // Форматируем число в строку
        
        usart_send_string(number_buffer);  // Отправка числа 85 в виде строки
        
        for (volatile int i = 0; i < 100000; i++); // Задержка
    }

    return 0;
}
