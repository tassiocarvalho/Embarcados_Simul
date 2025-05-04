// Arduino 1 (Supervisor) - Controle de Planta Industrial
// UEFS - MI Sistemas Digitais - Problema #2

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

// Variáveis globais
volatile uint8_t parada_solicitada = 0;
uint16_t valor_pot1 = 0;
uint16_t valor_pot2 = 0;
uint8_t velocidade_motor1 = 0;
uint8_t velocidade_motor2 = 0;
uint8_t contador_3s = 0;

// Função para inicializar UART
void UART_Init(void) {
    // 9600 baud rate para 16MHz
    UBRR0H = 0;
    UBRR0L = 103;
    
    // Habilitar TX e RX
    UCSR0B = (1 << RXEN0) | (1 << TXEN0);
    
    // Formato 8N1
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

// Função para enviar byte pela UART
void UART_Transmit(uint8_t data) {
    while (!(UCSR0A & (1 << UDRE0)));
    UDR0 = data;
}

// Função para enviar string pela UART
void UART_PrintString(const char* str) {
    while (*str) {
        UART_Transmit(*str++);
    }
}

// Função para inicializar ADC
void ADC_Init(void) {
    // Referência AVCC
    ADMUX = (1 << REFS0);
    
    // Habilitar ADC, prescaler 128
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
}

// Função para ler ADC
uint16_t ADC_Read(uint8_t channel) {
    // Selecionar canal
    ADMUX = (ADMUX & 0xF8) | (channel & 0x07);
    
    // Iniciar conversão
    ADCSRA |= (1 << ADSC);
    
    // Aguardar conversão
    while (ADCSRA & (1 << ADSC));
    
    return ADC;
}

// Função para configurar interrupção
void INT_Init(void) {
    // PD2 (INT0) como entrada com pull-up
    DDRD &= ~(1 << PD2);
    PORTD |= (1 << PD2);
    
    // Interrupção na borda de descida
    EICRA = (1 << ISC01);
    
    // Habilitar INT0
    EIMSK = (1 << INT0);
}

// ISR para botão de emergência
ISR(INT0_vect) {
    parada_solicitada = 1;
}

void setup() {
    // Inicializar componentes
    UART_Init();
    ADC_Init();
    INT_Init();
    
    // Habilitar interrupções globais
    sei();
    
    UART_PrintString("Arduino 1 (Supervisor) iniciado\r\n");
}

void loop() {
    // Ler potenciômetros
    valor_pot1 = ADC_Read(0);
    valor_pot2 = ADC_Read(1);
    
    // Mapear valores (0-1023 para 0-255)
    velocidade_motor1 = valor_pot1 >> 2;
    velocidade_motor2 = valor_pot2 >> 2;
    
    // Enviar comando de velocidade
    char comando[20];
    sprintf(comando, "VEL:%d,%d\r\n", velocidade_motor1, velocidade_motor2);
    UART_PrintString(comando);
    
    // Verificar parada de emergência
    if (parada_solicitada) {
        UART_PrintString("PARADA\r\n");
        UART_PrintString("Parada solicitada\r\n");
        parada_solicitada = 0;
    }
    
    // Contador para status a cada 3 segundos
    contador_3s++;
    if (contador_3s >= 30) { // 30 * 100ms = 3s
        UART_PrintString("STATUS_REQUEST\r\n");
        contador_3s = 0;
    }
    
    _delay_ms(100);
}