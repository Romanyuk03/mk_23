
//#include <libopencm3/cm3/nvic.h>
//#include <libopencm3/stm32/rcc.h>
//#include <libopencm3/stm32/gpio.h>
//#include <libopencm3/stm32/timer.h>
//#include <libopencm3/stm32/usart.h>
//#include <stdio.h>

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
        usart_send_string(buffer);  /////вместо этого написать отправку числа 85 в юсарт 
        
        // Добавим небольшую задержку, чтобы избежать слишком частого вывода
        for (volatile int i = 0; i < 100000; i++);
    }

    return 0;
}
