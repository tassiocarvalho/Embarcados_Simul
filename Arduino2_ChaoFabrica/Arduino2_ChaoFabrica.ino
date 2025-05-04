// Arduino 2 (Chão de Fábrica) - Versão para SimulIDE
#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

// Definições de pinos
#define LED_VERDE PB0    // D8
#define LED_VERMELHO PB1 // D9
#define BUZZER PB2       // D10
#define MOTOR1 PD3       // D3 (OC2B)
#define MOTOR2 PD5       // D5 (OC0B)
#define SERVO PD6        // D6 (OC0A)
#define SENSOR_PRESENCA PD7 // D7

// Variáveis globais
volatile uint8_t parada_solicitada = 0;
uint8_t vel_motor1 = 0;
uint8_t vel_motor2 = 0;
uint8_t temperatura = 0;
uint8_t inclinacao = 0;
uint8_t nivel_tanque = 0;
uint8_t presenca = 0;
uint16_t contador_blocos = 0;

// UART
void UART_Init(void) {
    UBRR0H = 0;
    UBRR0L = 103;
    UCSR0B = (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

// Enviar byte pela UART
void UART_EnviarByte(uint8_t data) {
    while (!(UCSR0A & (1 << UDRE0)));
    UDR0 = data;
}

// Enviar string pela UART
void UART_EnviarString(const char* str) {
    while (*str) {
        UART_EnviarByte(*str++);
    }
}

// ADC
void ADC_Init(void) {
    ADMUX = (1 << REFS0);
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
}

// Ler ADC
uint16_t ADC_Ler(uint8_t canal) {
    ADMUX = (ADMUX & 0xF0) | canal;
    ADCSRA |= (1 << ADSC);
    while (ADCSRA & (1 << ADSC));
    return ADC;
}

// PWM
void PWM_Init(void) {
    // Timer0 - Fast PWM (Motor2 e Servo)
    TCCR0A = (1 << COM0A1) | (1 << COM0B1) | (1 << WGM01) | (1 << WGM00);
    TCCR0B = (1 << CS01);
    OCR0A = 0;
    OCR0B = 0;
    
    // Timer2 - Fast PWM (Motor1)
    TCCR2A = (1 << COM2B1) | (1 << WGM21) | (1 << WGM20);
    TCCR2B = (1 << CS21);
    OCR2B = 0;
}

// I/O
void IO_Init(void) {
    // LEDs e Buzzer
    DDRB |= (1 << LED_VERDE) | (1 << LED_VERMELHO) | (1 << BUZZER);
    // Motores e Servo
    DDRD |= (1 << MOTOR1) | (1 << MOTOR2) | (1 << SERVO);
    // Sensor de presença
    DDRD &= ~(1 << SENSOR_PRESENCA);
    PORTD |= (1 << SENSOR_PRESENCA);
    // LED verde inicialmente ON
    PORTB |= (1 << LED_VERDE);
}

// Interrupção INT0
void INT_Init(void) {
    DDRD &= ~(1 << PD2);
    PORTD |= (1 << PD2);
    EICRA = (1 << ISC01);
    EIMSK = (1 << INT0);
}

// ISR para INT0
ISR(INT0_vect) {
    parada_solicitada = 1;
}

// ISR para recepção UART
ISR(USART_RX_vect) {
    uint8_t data = UDR0;
    
    // Código simplificado para processar comandos
    static uint8_t cmd = 0;
    static uint8_t step = 0;
    
    if (step == 0 && data == 'V') {
        cmd = 'V';
        step = 1;
    } else if (step == 1 && data == ':') {
        step = 2;
    } else if (step == 2) {
        vel_motor1 = data;
        step = 3;
    } else if (step == 3 && data == ',') {
        step = 4;
    } else if (step == 4) {
        vel_motor2 = data;
        step = 0;
    } else if (data == 'P') {
        parada_solicitada = 1;
        step = 0;
    } else {
        step = 0;
    }
}

int main(void) {
    // Inicializações
    UART_Init();
    ADC_Init();
    PWM_Init();
    IO_Init();
    INT_Init();
    sei();
    
    // Mensagem inicial
    UART_EnviarString("Arduino 2 OK\r\n");
    
    while (1) {
        // Ler sensores
        temperatura = ADC_Ler(0) >> 2;
        inclinacao = ADC_Ler(1) >> 2;
        nivel_tanque = ADC_Ler(2) >> 2;
        presenca = !(PIND & (1 << SENSOR_PRESENCA));
        
        // Verificar condições
        if (temperatura < 10 || temperatura > 200) { // Ajustado para simulação
            PORTB |= (1 << LED_VERMELHO) | (1 << BUZZER);
            parada_solicitada = 1;
        } else {
            PORTB &= ~((1 << LED_VERMELHO) | (1 << BUZZER));
        }
        
        // Controle do servo
        if (inclinacao < 100 || inclinacao > 150) {
            OCR0A = 90; // Posição de correção
        } else {
            OCR0A = 45; // Posição normal
        }
        
        // Controle dos motores
        if (parada_solicitada || presenca) {
            OCR0B = 0;   // Motor 2 OFF
            OCR2B = 0;   // Motor 1 OFF
            PORTB &= ~(1 << LED_VERDE);
        } else {
            OCR0B = vel_motor2; // Motor 2
            OCR2B = vel_motor1; // Motor 1
            PORTB |= (1 << LED_VERDE);
        }
        
        // Reset parada
        if (parada_solicitada) {
            UART_EnviarString("PARADA OK\r\n");
            parada_solicitada = 0;
        }
        
        // Status simples
        UART_EnviarByte('S');
        UART_EnviarByte(':');
        UART_EnviarByte(temperatura);
        UART_EnviarByte(',');
        UART_EnviarByte(inclinacao);
        UART_EnviarByte(',');
        UART_EnviarByte(nivel_tanque);
        UART_EnviarByte('\r');
        UART_EnviarByte('\n');
        
        _delay_ms(100);
    }
    
    return 0;
}