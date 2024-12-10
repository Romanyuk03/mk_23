
#include <libopencm3/stm32/f4/rcc.h>
#include <libopencm3/stm32/f4/gpio.h>
#include <libopencm3/stm32/f4/timer.h>
#include <libopencm3/stm32/f4/nvic.h>
#include <libopencm3/stm32/f4/usart.h>
#include <stdio.h>

volatile uint32_t pulse_width;

void tim2_isr(void) {
    if (TIM_SR(TIM2) & TIM_SR_CC1IF) {
        pulse_width = TIM_CCR1(TIM2); // Считываем значение захвата
        TIM_SR(TIM2) &= ~TIM_SR_CC1IF; // Сбрасываем флаг прерывания
        gpio_toggle(GPIOD, GPIO12); // Мигаем светодиодом при захвате
        for (volatile int i = 0; i < 100000; i++);
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

void usart_send_string(const char *str, uint32_t cnt) {
    // Ждем, пока TXE не станет 1
    
    while (cnt--) {
        while (!(USART_SR(USART2) & USART_SR_TXE));
        USART_DR(USART2) = *str++;  // Отправка символа
 
    }
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
    TIM_DIER(TIM2) |= TIM_DIER_CC1IE;   //разрешаем прерывание в самом таймере
    
    timer_enable_counter(TIM2);          // Запускаем таймер
    nvic_enable_irq(NVIC_TIM2_IRQ);     // Разрешаем прерывание в NVIC
}

void read(void){
    char buffer[25];
    //tim2_isr();
    uint32_t current_pulse_width = pulse_width;    

    uint32_t n = snprintf(buffer, sizeof(buffer), "Pulse Width: %lu\n\r ", current_pulse_width);
    usart_send_string(buffer,n);            // Отправка ширины импульса по UART

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
