/*
 * ATmega16A UART Communication
 * 4x4 keypad + 16x2 LCD
 * Bidirectional number exchange with ATmega8A
 *
 * Atmel Studio 6 / AVR-GCC
 */

#define F_CPU 8000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <util/atomic.h>
#include <stdint.h>
#include <stdlib.h>

/* =========================================================
   UART PACKET

   Byte 0: 0xA5 header
   Byte 1: number high byte
   Byte 2: number low byte
   Byte 3: XOR checksum
   ========================================================= */

#define UART_HEADER 0xA5
#define UART_UBRR   51U       /* 9600 baud at 8 MHz */

/* =========================================================
   LCD — PORTC

   RS -> PC0
   EN -> PC1
   D4 -> PC4
   D5 -> PC5
   D6 -> PC6
   D7 -> PC7
   RW -> GND
   ========================================================= */

#define LCD_PORT PORTC
#define LCD_DDR  DDRC

#define LCD_RS PC0
#define LCD_EN PC1
#define LCD_D4 PC4
#define LCD_D5 PC5
#define LCD_D6 PC6
#define LCD_D7 PC7

/* =========================================================
   4x4 KEYPAD — PORTA

   R1 -> PA0
   R2 -> PA1
   R3 -> PA2
   R4 -> PA3

   C1 -> PA4
   C2 -> PA5
   C3 -> PA6
   C4 -> PA7
   ========================================================= */

#define KEYPAD_PORT PORTA
#define KEYPAD_DDR  DDRA
#define KEYPAD_PIN  PINA

/* Number received from ATmega8A */
static volatile uint16_t receivedNumber = 0;

/* UART receive-state variables */
static volatile uint8_t uartReceiveState = 0;
static volatile uint8_t uartHighByte = 0;
static volatile uint8_t uartLowByte = 0;

/* =========================================================
   JTAG DISABLE

   ATmega16A JTAG can interfere with PORTC pins PC2-PC5.
   ========================================================= */

static void Disable_JTAG(void)
{
#ifdef JTD
    MCUCSR |= (1 << JTD);
    MCUCSR |= (1 << JTD);
#endif
}

/* =========================================================
   LCD FUNCTIONS
   ========================================================= */

static void LCD_EnablePulse(void)
{
    LCD_PORT |= (1 << LCD_EN);
    _delay_us(1);

    LCD_PORT &= ~(1 << LCD_EN);
    _delay_us(100);
}

static void LCD_SendNibble(uint8_t nibble)
{
    LCD_PORT &= ~(
        (1 << LCD_D4) |
        (1 << LCD_D5) |
        (1 << LCD_D6) |
        (1 << LCD_D7)
    );

    if (nibble & 0x01)
        LCD_PORT |= (1 << LCD_D4);

    if (nibble & 0x02)
        LCD_PORT |= (1 << LCD_D5);

    if (nibble & 0x04)
        LCD_PORT |= (1 << LCD_D6);

    if (nibble & 0x08)
        LCD_PORT |= (1 << LCD_D7);

    LCD_EnablePulse();
}

static void LCD_SendByte(uint8_t value, uint8_t isData)
{
    if (isData)
        LCD_PORT |= (1 << LCD_RS);
    else
        LCD_PORT &= ~(1 << LCD_RS);

    LCD_SendNibble(value >> 4);
    LCD_SendNibble(value & 0x0F);

    _delay_us(50);
}

static void LCD_Command(uint8_t command)
{
    LCD_SendByte(command, 0);

    if ((command == 0x01) || (command == 0x02))
        _delay_ms(2);
}

static void LCD_Character(char character)
{
    LCD_SendByte((uint8_t)character, 1);
}

static void LCD_Init(void)
{
    LCD_DDR |=
        (1 << LCD_RS) |
        (1 << LCD_EN) |
        (1 << LCD_D4) |
        (1 << LCD_D5) |
        (1 << LCD_D6) |
        (1 << LCD_D7);

    LCD_PORT &= ~(
        (1 << LCD_RS) |
        (1 << LCD_EN)
    );

    _delay_ms(40);

    LCD_SendNibble(0x03);
    _delay_ms(5);

    LCD_SendNibble(0x03);
    _delay_us(150);

    LCD_SendNibble(0x03);
    LCD_SendNibble(0x02);

    LCD_Command(0x28);
    LCD_Command(0x0C);
    LCD_Command(0x06);
    LCD_Command(0x01);
}

static void LCD_Goto(uint8_t row, uint8_t column)
{
    uint8_t address;

    if (row == 0)
        address = column;
    else
        address = 0x40 + column;

    LCD_Command(0x80 | address);
}

static void LCD_Print(const char *text)
{
    while (*text != '\0')
    {
        LCD_Character(*text);
        text++;
    }
}

static void LCD_PrintNumber(uint16_t number)
{
    char buffer[6];

    itoa((int)number, buffer, 10);
    LCD_Print(buffer);

    if (number < 1000)
        LCD_Character(' ');

    if (number < 100)
        LCD_Character(' ');

    if (number < 10)
        LCD_Character(' ');
}

/* =========================================================
   KEYPAD FUNCTIONS
   ========================================================= */

static const char keypadMap[4][4] =
{
    {'1', '2', '3', 'A'},
    {'4', '5', '6', 'B'},
    {'7', '8', '9', 'C'},
    {'*', '0', '#', 'D'}
};

static void Keypad_Init(void)
{
    /* PA0-PA3 outputs, PA4-PA7 inputs */
    KEYPAD_DDR = 0x0F;

    /* Rows HIGH, column pull-ups enabled */
    KEYPAD_PORT = 0xFF;
}

static char Keypad_ScanRaw(void)
{
    uint8_t row;
    uint8_t column;

    for (row = 0; row < 4; row++)
    {
        KEYPAD_PORT |= 0x0F;
        KEYPAD_PORT &= ~(1 << row);

        _delay_us(5);

        for (column = 0; column < 4; column++)
        {
            if (!(KEYPAD_PIN & (1 << (column + 4))))
                return keypadMap[row][column];
        }
    }

    return 0;
}

static char Keypad_GetKey(void)
{
    char key = Keypad_ScanRaw();

    if (key != 0)
    {
        _delay_ms(20);

        if (Keypad_ScanRaw() == key)
        {
            while (Keypad_ScanRaw() != 0)
                _delay_ms(5);

            return key;
        }
    }

    return 0;
}

/* =========================================================
   UART FUNCTIONS
   ========================================================= */

static void UART_Init(void)
{
    UBRRH = (uint8_t)(UART_UBRR >> 8);
    UBRRL = (uint8_t)UART_UBRR;

    /*
     * RXEN  = receiver enabled
     * TXEN  = transmitter enabled
     * RXCIE = receive interrupt enabled
     */
    UCSRB =
        (1 << RXEN) |
        (1 << TXEN) |
        (1 << RXCIE);

    /*
     * URSEL must be 1 when writing UCSRC.
     * 8 data bits, no parity, one stop bit.
     */
    UCSRC =
        (1 << URSEL) |
        (1 << UCSZ1) |
        (1 << UCSZ0);
}

static void UART_SendByte(uint8_t data)
{
    while (!(UCSRA & (1 << UDRE)))
    {
        /* Wait for empty transmit register */
    }

    UDR = data;
}

static uint8_t UART_Checksum(
    uint8_t header,
    uint8_t highByte,
    uint8_t lowByte
)
{
    return header ^ highByte ^ lowByte;
}

static void UART_SendNumber(uint16_t number)
{
    uint8_t highByte;
    uint8_t lowByte;
    uint8_t checksum;

    highByte = (uint8_t)(number >> 8);
    lowByte = (uint8_t)(number & 0xFF);

    checksum = UART_Checksum(
        UART_HEADER,
        highByte,
        lowByte
    );

    UART_SendByte(UART_HEADER);
    UART_SendByte(highByte);
    UART_SendByte(lowByte);
    UART_SendByte(checksum);
}

/* =========================================================
   UART RECEIVE INTERRUPT
   ========================================================= */

ISR(USART_RXC_vect)
{
    uint8_t receivedByte;
    uint8_t expectedChecksum;

    receivedByte = UDR;

    switch (uartReceiveState)
    {
        case 0:
            if (receivedByte == UART_HEADER)
                uartReceiveState = 1;
            break;

        case 1:
            uartHighByte = receivedByte;
            uartReceiveState = 2;
            break;

        case 2:
            uartLowByte = receivedByte;
            uartReceiveState = 3;
            break;

        case 3:
            expectedChecksum = UART_Checksum(
                UART_HEADER,
                uartHighByte,
                uartLowByte
            );

            if (receivedByte == expectedChecksum)
            {
                receivedNumber =
                    ((uint16_t)uartHighByte << 8) |
                    (uint16_t)uartLowByte;
            }

            uartReceiveState = 0;
            break;

        default:
            uartReceiveState = 0;
            break;
    }
}

/* =========================================================
   KEYPAD NUMBER PROCESSING
   ========================================================= */

static void ProcessKey(
    char key,
    uint16_t *inputNumber,
    uint8_t *digitCount
)
{
    if ((key >= '0') && (key <= '9'))
    {
        if (*digitCount < 4)
        {
            *inputNumber =
                (*inputNumber * 10U) +
                (uint16_t)(key - '0');

            (*digitCount)++;
        }
    }
    else if (key == '*')
    {
        *inputNumber = 0;
        *digitCount = 0;
    }
    else if (key == 'A')
    {
        *inputNumber /= 10U;

        if (*digitCount > 0)
            (*digitCount)--;
    }
    else if (key == 'B')
    {
        if (*inputNumber < 9999)
            (*inputNumber)++;
    }
    else if (key == 'C')
    {
        if (*inputNumber > 0)
            (*inputNumber)--;
    }
    else if (key == 'D')
    {
        *inputNumber = 0;
        *digitCount = 0;
    }
    else if (key == '#')
    {
        UART_SendNumber(*inputNumber);
    }
}

/* =========================================================
   MAIN
   ========================================================= */

int main(void)
{
    char key;
    uint16_t inputNumber = 0;
    uint16_t receivedNumberCopy = 0;
    uint8_t digitCount = 0;

    Disable_JTAG();
    LCD_Init();
    Keypad_Init();
    UART_Init();

    sei();

    LCD_Goto(0, 0);
    LCD_Print("M16:");

    LCD_Goto(1, 0);
    LCD_Print("M8 :");

    while (1)
    {
        key = Keypad_GetKey();

        if (key != 0)
        {
            ProcessKey(
                key,
                &inputNumber,
                &digitCount
            );
        }

        ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
        {
            receivedNumberCopy = receivedNumber;
        }

        LCD_Goto(0, 5);
        LCD_PrintNumber(inputNumber);

        LCD_Goto(1, 5);
        LCD_PrintNumber(receivedNumberCopy);

        _delay_ms(20);
    }

    return 0;
}