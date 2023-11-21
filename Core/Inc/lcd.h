/*
 * lcd.h
 *
 *
 *      Author: rahu7p
 */

#ifndef INC_LCD_H_
#define INC_LCD_H_

/* **************************************************************************** */
/* Completar la lista de definicion de pines del LCD (Puerto B)                 *
 * Ejemplo: D4 (LCD) conectado al pin PB2 del microcontrolador...               *
 *          #define LCD_D4_PIN_HIGH   GPIO_BSRR_BS2_Pos // Set pin DATA4_LCD    *
 *          #define LCD_D4_PIN_LOW    GPIO_BSRR_BR2_Pos // Reset pin DATA4_LCD  *
 * Conexión recomendada de los pines:                                           *
 * RS->PC6, RW->PC7, EN->PC8, D4->PC9, D5->PC10, D6->PC11, D7->PC12             */

#define LCD_RS_PIN_HIGH 6                 	//		Set pin RS_LCD
#define LCD_RS_PIN_LOW 	22	                  //		Reset pin RS_LCD
#define LCD_RW_PIN_HIGH 7	                  //		Set pin RW_LCD
#define LCD_RW_PIN_LOW 	23	                  //		Reset pin RW_LCD
#define LCD_EN_PIN_HIGH 8	                  //		Set pin EN_LCD
#define LCD_EN_PIN_LOW 	24	                  //		Reset pin EN_LCD

#define LCD_D4_PIN_HIGH	9	                  //		Set pin DATA4_LCD
#define LCD_D5_PIN_HIGH 10	                  //		Set pin DATA5_LCD
#define LCD_D6_PIN_HIGH 11	                  //		Set pin DATA6_LCD
#define LCD_D7_PIN_HIGH 12	                  //		Set pin DATA7_LCD

#define LCD_D4_PIN_LOW 	25	                  //		Reset pin DATA4_LCD
#define LCD_D5_PIN_LOW 	26	                  //		Reset pin DATA5_LCD
#define LCD_D6_PIN_LOW 	27	                  //		Reset pin DATA6_LCD
#define LCD_D7_PIN_LOW 	28	                  //		Reset pin DATA7_LCD
/* *********************************************************************** */

//Definimos los nombres de los comandos para el LCD
#define LCD_Clear( )            LCD_Write_Cmd( 0x01U )//	Borra la pantalla
#define LCD_Display_ON( )	LCD_Write_Cmd( 0x0EU )//	Pantalla LCD activa
#define LCD_Display_OFF( )	LCD_Write_Cmd( 0x08U )//	Pantalla LCD inactiva
#define LCD_Cursor_Home( )	LCD_Write_Cmd( 0x02U )//	Establecer el cursor a 'Home'
#define LCD_Cursor_Blink( )	LCD_Write_Cmd( 0x0FU )//	Cursor intermitente
#define LCD_Cursor_ON( )	LCD_Write_Cmd( 0x0EU )//	Cursor visible activo
#define LCD_Cursor_OFF( )	LCD_Write_Cmd( 0x0CU )//	Cursor inactivo
#define LCD_Cursor_Left( )	LCD_Write_Cmd( 0x10U )//	Movimiento hacia la izquierda del cursor
#define LCD_Cursor_Right( )	LCD_Write_Cmd( 0x14U )//	Movimiento hacia la derecha del cursor
#define LCD_Cursor_SLeft( )	LCD_Write_Cmd( 0x18U )//	Movimiento hacia la izquierda de la pantalla
#define LCD_Cursor_SRight( )    LCD_Write_Cmd( 0x1CU )//	Movimiento hacia la derecha de la pantalla

//Lista de funciones
void LCD_Data_Out4(unsigned char val);
void LCD_Write_Byte(unsigned char val);
void LCD_Write_Cmd(unsigned char val);
void LCD_Put_Char(unsigned char c);
void LCD_Init(void);
void LCD_Set_Cursor(unsigned char line, unsigned char column);
void LCD_Put_Str(char* str);
void LCD_Put_Num(int num);
char LCD_Busy(void);
void LCD_Pulse_EN(void);
void LCD_BarGraphic(int value, int size);
void LCD_BarGraphicXY(int pos_x, int pos_y, int value);


#endif /* INC_LCD_H_ */
