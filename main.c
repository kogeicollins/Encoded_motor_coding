#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#define LCD_Dir DDRD
#define LCD_Port PORTD
#define RS PD0
#define EN PD1

void LCD_Cmd(unsigned char cmd);
void LCD_Char(unsigned char char_data);
void LCD_Init(void);
void LCD_Clear(void);
void LCD_String(char *str);
void LCD_String_xy(char row, char pos, char *str);

void LCD_Cmd(unsigned char cmd){
	/*Sending the first nibble of data (Higher 4 bits)*/
	LCD_Port = (LCD_Port & 0x0F) | (cmd & 0xF0);/* Sending upper nibble */
	LCD_Port &= ~ (1<<RS); /* RS=0, command reg. */
	LCD_Port |= (1<<EN); /* Enable pulse ON */
	_delay_us(1);
	LCD_Port &= ~ (1<<EN); /* Enable pulse OFF */
	_delay_us(200);
	/*Sending the second nibble of data (Lower 4 bits)*/
	LCD_Port = (LCD_Port & 0x0F) | (cmd << 4);/* Sending lower nibble */
	LCD_Port |= (1<<EN); /* Enable pulse ON */
	_delay_us(1);
	LCD_Port &= ~ (1<<EN); /* Enable pulse OFF */
	_delay_ms(2);
}
void LCD_Char (unsigned char char_data){
	/*Sending the first nibble of data (Higher 4 bits)*/
	LCD_Port = (LCD_Port & 0x0F) | (char_data & 0xF0);/* Sending upper nibble */
	LCD_Port |= (1<<RS); /* RS=1, data reg. */
	LCD_Port |= (1<<EN); /* Enable pulse ON */
	_delay_us(1);
	LCD_Port &= ~ (1<<EN); /* Enable pulse OFF */
	_delay_us(200);
	/*Sending the second nibble of data (Lower 4 bits)*/
	LCD_Port = (LCD_Port & 0x0F) | (char_data << 4); /* Sending lower nibble */
	LCD_Port |= (1<<EN); /* Enable pulse ON */
	_delay_us(1);
	LCD_Port &= ~ (1<<EN); /* Enable pulse OFF */
	_delay_ms(2);
}
void LCD_Init (void){
	LCD_Dir = 0xFF; /* Make LCD command port direction as output pins*/
	_delay_ms(20); /* LCD Power ON delay always > 15ms */
	LCD_Cmd(0x02); /* Return display to its home position */
	LCD_Cmd(0x28); /* 2 line 4bit mode */
	LCD_Cmd(0x0C); /* Display ON Cursor OFF */
	LCD_Cmd(0x06); /* Auto Increment cursor */
	LCD_Cmd(0x01); /* Clear display */
}
void LCD_Clear(void){
	LCD_Cmd(0x01); /* clear display */
	LCD_Cmd(0x02); /* Return display to its home position */
}
/*Send string to LCD function */
void LCD_String (char *str){
	int i;
	/* Send each char of string till the NULL */
	for(i=0;str[i]!=0;i++){
		LCD_Char(str[i]);
	}
}
void LCD_String_xy (char row, char pos, char *str){
	if (row == 0 && pos<16){
		LCD_Cmd((pos & 0x0F)|0x80);/* Command of first row and required
		position<16 */
	}
	else if (row == 1 && pos<16){
		LCD_Cmd((pos & 0x0F)|0xC0);/* Command of second row and required
		position<16 */
	}
	LCD_String(str); /* Call LCD string function */
}

// Motor and encoder definitions
#define PWM_PIN PB3
#define IN1_PIN PB2
#define IN2_PIN PB1
#define ENCODER_A_PIN PD2
#define ENCODER_B_PIN PD3

volatile int encoder_count = 0;


ISR(INT0_vect) {
	if (PIND & (1 << ENCODER_B_PIN)) {
		encoder_count++;
		} else {
		encoder_count--;
	}
}

ISR(INT1_vect) {
	if (PIND & (1 << ENCODER_A_PIN)) {
		encoder_count++;
		} else {
		encoder_count--;
	}
}

void pwm_init() {
	DDRB |= (1 << PWM_PIN) | (1 << IN1_PIN) | (1 << IN2_PIN);
	TCCR2 = (1 << WGM20) | (1 << WGM21) | (1 << COM21) | (1 << CS21); // Fast PWM, non-inverting, prescaler 8
	OCR2 = 0;
}

void set_motor_speed(uint8_t speed) {
	OCR2 = speed;
	PORTB |= (1 << IN1_PIN);
	PORTB &= ~(1 << IN2_PIN);
}

void adc_init() {
	ADMUX = (1 << REFS0); // AVcc with external capacitor at AREF pin
	ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1); // Enable ADC, prescaler 64
}

uint16_t adc_read(uint8_t channel) {
	ADMUX = (ADMUX & 0xF8) | (channel & 0x07);
	ADCSRA |= (1 << ADSC); // Start conversion
	while (ADCSRA & (1 << ADSC)); // Wait for conversion to complete
	return ADCW;
}
void lcd_print(const char *str) {
	while (*str) {
		LCD_Char(*str++);
	}
}
void lcd_print_int(int value) {
	char buffer[16];
	itoa(value, buffer, 10);
	LCD_Char(buffer);
}

int main(void) {
	// Initialize LCD
	//LCD_DDRD |= (1 << LCD_RS) | (1 << LCD_EN); // Set control pins as output
	//LCD_DDRC |= (1 << LCD_D4) | (1 << LCD_D5) | (1 << LCD_D6) | (1 << LCD_D7); // Set data pins as output
	//lcd_init();

	LCD_Init(); /* Initialize LCD */
	LCD_String("MSP II:EMT 3202"); /* Write a string on 1st line of LCD*/
	LCD_Cmd(0xC0); /* Go to 2nd line*/
	lcd_print_int(set_motor_speed();
	// Initialize motor control
	pwm_init();
	adc_init();
	
	// Initialize encoder
	DDRD &= ~((1 << ENCODER_A_PIN) | (1 << ENCODER_B_PIN)); // Set encoder pins as input
	MCUCR |= (1 << ISC00) | (1 << ISC10); // Trigger on any logical change
	GICR |= (1 << INT0) | (1 << INT1); // Enable external interrupts INT0 and INT1
	sei(); // Enable global interrupts
	
//LCD_String("PWM:"); /* Write a string on 1st line of LCD*/
//LCD_Cmd(0xC0); /* Go to 2nd line*/
//LCD_String("Hello World");
//LCD_Cmd(0XC0);
//LCD_String("new line"); /* Write string on 2nd line*/
//LCD_String("PWM: ");
//LCD_Cmd(0XC0);
//lcd_print_int(speed);
	while (1) {
		uint16_t adc_value = adc_read(0); // Read potentiometer value from PA0
		uint8_t speed = adc_value / 4; // Convert ADC value to 0-255 range for PWM
		set_motor_speed(speed);

		// Display the PWM value on the LCD
		//lcd_clear();
		//lcd_set_cursor(0, 0);
		//LCD_String("Hello World");
		//lcd_print_int(speed);
		//LCD_String("pwm")
		
		_delay_ms(500); // Update display every 500ms
	}
}
