// Підключення стандартних бібліотек для AVR
#include <avr/io.h>        // Доступ до портів вводу/виводу
#include <util/delay.h>    // Функції затримки
#include <stdlib.h>        // Стандартні функції (itoa, abs тощо)

// Визначення пінів підключення LCD дисплея
#define LCD_RS PB4
#define LCD_RW PB3
#define LCD_E  PB2
#define LCD_D4 PB1
#define LCD_D5 PB0
#define LCD_D6 PD7
#define LCD_D7 PD6

// Піни сенсорів тиску HX710B
#define HX_SCK PC4   // Тактовий сигнал
#define HX_DT1 PC2   // Канал 1
#define HX_DT2 PC3   // Канал 2

// Піни для керування кроковим двигуном
#define MOTOR_STEP_PD PD3 // Крок
#define MOTOR_DIR_PC  PC5 // Напрямок обертання
#define MOTOR_ENA_PC  PC0 // Дозвіл (ввімкнення драйвера)

//Піни для передачі даних по протоколу RS485
#define USART_RX PD0        
#define USART_TX PD1
#define RS485_DA PD2

// Коефіцієнт масштабування (визначається експериментально, перетворює значення ΔP у кПа)
#define SCALE_FACTOR 10000.0

// Надсилання сигналу Enable на дисплей (короткий імпульс для прийому команди)
static void lcd_enable() {
    PORTB |= (1 << LCD_E);
    _delay_us(1);
    PORTB &= ~(1 << LCD_E);
    _delay_us(100);
}

//Ініціалізація інтерфейсу RS485
static void rs485_init(void)
{   
    //Налаштовуємо пін як вихід
    DDRD |= (1 << RS485_DA);
    //Налаштовуємо в режим прийому
    PORTD &= ~(1 << RS485_DA);

    //Налаштування UART: 9600 baud, 8N1, F_CPU = 16meg
    static uint16_t ubrr = 103;
    UBRR0H = (ubrr >> 8);
    UBRR0L = ubrr;
    UCSR0B = (1 << RXEN0) | (1 << TXEN0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);    
}

//Функція відправки повідомлення
static void rs485_send(const char *msg)
{
    PORTD |= (1 << RS485_DA);
    _delay_us(50);

    while(*msg)
    {
        while(!(UCSR0A & (1 << UDRE0)));
        UDR0 = *msg++;
    }
    while (!(UCSR0A & (1 << TXC0)));
    _delay_us(5);

    PORTD &= ~(1 << RS485_DA);
}


//Функція отримання повідомлення
static int rs485_readln(char *buf, int maxlen)
{
    int i = 0;
    while(1)
    {
        while (!(UCSR0A & (1 << RXC0)));
        char c = UDR0;
        if (i < maxlen - 1) buf[i++] = c;
        if (c == '\n') break;
    }
    buf[i] = '\0'; 
    return i;
}

// Надсилання 4-бітного півбайту (nibble) на LCD
static void lcd_send_nibble(uint8_t nibble) {
    // Маскування портів і передача значущих бітів
    PORTB = (PORTB & ~((1 << LCD_D4) | (1 << LCD_D5))) |
            ((nibble & 0x01) << LCD_D4) | ((nibble & 0x02) << LCD_D5);
    PORTD = (PORTD & ~((1 << LCD_D6) | (1 << LCD_D7))) |
            ((nibble & 0x04) << (LCD_D6 - 2)) | ((nibble & 0x08) << (LCD_D7 - 3));
    lcd_enable();
}

// Відправлення команди на LCD
static void lcd_command(uint8_t cmd) {
    PORTB &= ~(1 << LCD_RS); // Режим команд
    lcd_send_nibble(cmd >> 4); // Старші біти
    lcd_send_nibble(cmd & 0x0F); // Молодші біти
    _delay_ms(2);
}

// Вивід символу на LCD
static void lcd_data(uint8_t data) {
    PORTB |= (1 << LCD_RS); // Режим даних
    lcd_send_nibble(data >> 4);
    lcd_send_nibble(data & 0x0F);
    _delay_ms(2);
}

// Ініціалізація LCD дисплея
static void lcd_init() {
    DDRB |= (1 << LCD_RS) | (1 << LCD_RW) | (1 << LCD_E) | (1 << LCD_D4) | (1 << LCD_D5);
    DDRD |= (1 << LCD_D6) | (1 << LCD_D7);
    PORTB &= ~(1 << LCD_RW); // Встановлення режиму запису
    _delay_ms(40); // Затримка після включення живлення
    lcd_send_nibble(0x03); _delay_ms(5);
    lcd_send_nibble(0x03); _delay_us(150);
    lcd_send_nibble(0x03); _delay_us(150);
    lcd_send_nibble(0x02); _delay_us(150); // Перехід у 4-бітний режим
    lcd_command(0x28); // 2 рядки, 5x8 точок, 4-біт
    lcd_command(0x0C); // Увімкнення дисплея, без курсору
    lcd_command(0x06); // Рух курсору вправо
    lcd_command(0x01); // Очистка екрану
}

// Встановлення позиції курсору на LCD
static void lcd_set_cursor(uint8_t row, uint8_t col) {
    lcd_command(0x80 | (row ? 0x40 : 0x00) | col);
}

// Виведення рядка на дисплей
static void lcd_write_string(const char *str) {
    while (*str) lcd_data(*str++);
}

// Ініціалізація сенсорів тиску HX710B
static void hx710b_init() {
    DDRC |= (1 << HX_SCK);
    DDRC &= ~((1 << HX_DT1) | (1 << HX_DT2)); // Лінії даних — вхід
    PORTC &= ~(1 << HX_SCK); // Початковий рівень SCK = 0
}

// Зчитування 24-бітного значення з HX710B
static uint32_t hx710b_read(uint8_t dout_pin) {
    uint32_t value = 0;
    while (PINC & (1 << dout_pin)); // Чекаємо, поки DOUT стане LOW
    for (uint8_t i = 0; i < 24; i++) {
        PORTC |= (1 << HX_SCK);
        value <<= 1;
        if (PINC & (1 << dout_pin)) value++;
        PORTC &= ~(1 << HX_SCK);
    }
    PORTC |= (1 << HX_SCK); // Додатковий 25-й імпульс (для режиму 128)
    value ^= 0x800000;       // Інверсія знаку (2-комплементарний код)
    PORTC &= ~(1 << HX_SCK);
    return value;
}

// Середнє ковзне значення з фільтрацією
#define FILTER_DEPTH 8
static uint32_t filtered_avg(uint8_t pin) {
    static uint32_t buffer1[FILTER_DEPTH] = {0};
    static uint8_t i1 = 0;

    uint32_t sum = 0;
    buffer1[i1] = hx710b_read(pin); // Запис нового значення в буфер
    i1 = (i1 + 1) % FILTER_DEPTH;   // Перехід до наступного індексу

    for (uint8_t i = 0; i < FILTER_DEPTH; i++) sum += buffer1[i];
    return sum / FILTER_DEPTH; // Середнє значення
}

// Ініціалізація пінів для крокового двигуна
static void motor_init() {
    DDRD |= (1 << MOTOR_STEP_PD);
    DDRC |= (1 << MOTOR_DIR_PC) | (1 << MOTOR_ENA_PC);
    PORTC &= ~(1 << MOTOR_ENA_PC); // Увімкнення драйвера (LOW)
}

// Один крок двигуна
static void motor_step() {
    PORTD |= (1 << MOTOR_STEP_PD);
    _delay_us(800);
    PORTD &= ~(1 << MOTOR_STEP_PD);
    _delay_us(800);
}

// Головна функція
int main(void) {
    lcd_init();       // Запуск LCD
    hx710b_init();    // Ініціалізація сенсорів тиску
    motor_init();     // Підготовка двигуна

    lcd_set_cursor(0, 0);
    lcd_write_string("Calibrating..."); // Повідомлення на дисплеї
    uint32_t offset1 = filtered_avg(HX_DT1); // Збереження нульових значень
    uint32_t offset2 = filtered_avg(HX_DT2);
    lcd_command(0x01); // Очистити екран

    char buf[16]; // Буфер для тексту

    while (1) {
        _delay_ms(500); // Період оновлення 500 мс

        // Зчитування та розрахунок різниці тиску
        uint32_t r1 = filtered_avg(HX_DT1);
        uint32_t r2 = filtered_avg(HX_DT2);
        int32_t diff = ((int32_t)(r1 - offset1)) - ((int32_t)(r2 - offset2));
        float pressure_kpa = diff / SCALE_FACTOR;

        // Виведення тиску на LCD
        lcd_set_cursor(0, 0);
        lcd_write_string("dP: ");
        dtostrf(pressure_kpa, 5, 2, buf); // Перетворення float → текст
        lcd_write_string(buf);
        lcd_write_string(" kPa ");

        // Якщо тиск перевищує поріг — прокручуємо мотор
        if (pressure_kpa > 0.10 || pressure_kpa < -0.10) {
            if (pressure_kpa > 0)
                PORTC |= (1 << MOTOR_DIR_PC);  // Обертання в один бік
            else
                PORTC &= ~(1 << MOTOR_DIR_PC); // Обертання в інший бік

            // Кількість кроків пропорційна тиску (обмеження 50)
            uint8_t steps = (uint8_t)(fabs(pressure_kpa) * 10);
            if (steps > 50) steps = 50;
            for (uint8_t i = 0; i < steps; i++) motor_step();
        }
    }
}
