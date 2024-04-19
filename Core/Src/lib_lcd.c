/*
 * lib_lcd.c
 *
 *  Created on: janvier 2020
 *      Author:
 */

#include <lib_lcd.h>
#include "main.h"
#include "usart.h"
#include "i2c.h"
//I2C_HandleTypeDef hi2c3;

//UART_HandleTypeDef huart2;

void lcd_init(I2C_HandleTypeDef* I2Cx, rgb_lcd* DataStruct)
{
	I2C_HandleTypeDef* Handle = I2Cx;
	DataStruct->_displayfunction |= LCD_2LINE | LCD_5x10DOTS; // MODE 2 LIGNES
	uint8_t data[2];
	data[0] = 0x80;
	data[1] = LCD_FUNCTIONSET | DataStruct->_displayfunction;

	HAL_UART_Transmit(&huart2,(uint8_t *)data,2,10);
	char newline[2] = "\r\n";
	HAL_UART_Transmit(&huart2, (uint8_t *) newline, 2, 10);

	HAL_StatusTypeDef status = HAL_I2C_IsDeviceReady(Handle,LCD_ADDRESS,5,100);
	HAL_UART_Transmit(&huart2,&status,1,10);
	HAL_Delay(50);

	HAL_StatusTypeDef status2 = HAL_I2C_Master_Transmit(Handle, LCD_ADDRESS, data,2,5000);
	HAL_UART_Transmit(&huart2,&status2,1,10);
	HAL_Delay(50);

	HAL_I2C_Master_Transmit(Handle,LCD_ADDRESS,data,2,5000);
	HAL_Delay(5);

	HAL_I2C_Master_Transmit(Handle,LCD_ADDRESS,data,2,5000);

	// CONTROL
	DataStruct->_displaycontrol = LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF;

	data[1] = LCD_DISPLAYCONTROL | DataStruct->_displaycontrol;
	HAL_I2C_Master_Transmit(Handle,LCD_ADDRESS, data,2,1000);

	data[1] = LCD_CLEARDISPLAY;
	HAL_I2C_Master_Transmit(Handle, (uint16_t)LCD_ADDRESS, (uint8_t *)data,2,1000);
	HAL_Delay(2);

	// MODE
	DataStruct->_displaymode = LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT;
	data[1] = LCD_ENTRYMODESET | DataStruct->_displaymode;
	HAL_I2C_Master_Transmit(Handle, (uint16_t)LCD_ADDRESS, (uint8_t *)data,2,1000);

	// initialisation du lcd_rgb_Backight
		uint8_t data_backlight[2];
		data_backlight[0] = REG_MODE1;
		data_backlight[1] = 0;
		HAL_I2C_Master_Transmit(Handle, (uint16_t)RGB_ADDRESS, (uint8_t *)data_backlight,2,1000);

		data_backlight[0] = REG_OUTPUT;
		data_backlight[1] = 0xFF;
		HAL_I2C_Master_Transmit(Handle, (uint16_t)RGB_ADDRESS, (uint8_t *)data_backlight,2,1000);

		data_backlight[0] = REG_MODE2;
		data_backlight[1] = 0x20;
		HAL_I2C_Master_Transmit(Handle, (uint16_t)RGB_ADDRESS, (uint8_t *)data_backlight,2,1000);

		//couleur du LCD en Blanc
		uint8_t data_rgb[2];
		data_rgb[0] = REG_RED;
		data_rgb[1] = 255;
		HAL_I2C_Master_Transmit(Handle, (uint16_t)RGB_ADDRESS, (uint8_t *)data_rgb,2,1000);

		data_rgb[0] = REG_GREEN;
		data_rgb[1] = 255;
		HAL_I2C_Master_Transmit(Handle, (uint16_t)RGB_ADDRESS, (uint8_t *)data_rgb,2,1000);

		data_rgb[0] = REG_BLUE;
		data_rgb[1] = 255;
		HAL_I2C_Master_Transmit(Handle, (uint16_t)RGB_ADDRESS, (uint8_t *)data_rgb,2,1000);


}



void clearlcd(void) // permet d'effacer tout ce qui s'affiche sur le lcd
{
	lcd_position(&hi2c1,0,0);
	lcd_print(&hi2c1,"                ");
	lcd_position(&hi2c1,0,1);
	lcd_print(&hi2c1,"                ");
}

void lcd_write(I2C_HandleTypeDef* I2Cx, uint8_t value) // permet d'écrire sur le lcd
{
	I2C_HandleTypeDef* Handle = I2Cx;
    unsigned char data[2] = {0x40, value};
    HAL_I2C_Master_Transmit(Handle,LCD_ADDRESS, data,2,1000);
}

void lcd_print(I2C_HandleTypeDef* I2Cx, char *str)
{
	I2C_HandleTypeDef* Handle = I2Cx;
    char data[2];
    data[0] = 0x40;
    int i=0;
    while(str[i] != '\0')
    {
            data[1] = str[i];
            HAL_I2C_Master_Transmit(Handle,LCD_ADDRESS,(uint8_t *) data, 2,1000);
            i++;
   }
}

void lcd_position(I2C_HandleTypeDef* I2Cx,char col, char row) // position du curseur le lcd
{
	I2C_HandleTypeDef* Handle = I2Cx;
    if(row == 0)
    {
        col = col | 0x80;
    }
    else
    {
        col = col | 0xc0;
    }

    char data[2];
    data[0] = 0x80;
    data[1] = col;
    HAL_I2C_Master_Transmit(Handle,LCD_ADDRESS,(uint8_t *) data, 2,1000);
}

void reglagecouleur(uint8_t R,uint8_t G,uint8_t B)
{
	I2C_HandleTypeDef* Handle = &hi2c1;
	uint8_t data_r[2];
	uint8_t data_g[2];
	uint8_t data_b[2];
	data_r[0] = REG_RED;
	data_r[1] = R;
	HAL_I2C_Master_Transmit(Handle, (uint16_t)RGB_ADDRESS, (uint8_t *)data_r,2,1000);

	data_g[0] = REG_GREEN;
	data_g[1] = G;
	HAL_I2C_Master_Transmit(Handle, (uint16_t)RGB_ADDRESS, (uint8_t *)data_g,2,1000);

	data_b[0] = REG_BLUE;
	data_b[1] = B;
	HAL_I2C_Master_Transmit(Handle, (uint16_t)RGB_ADDRESS, (uint8_t *)data_b,2,1000);
}


void variable_Transmit(uint16_t address, uint8_t *Data, uint16_t len)
{
	 HAL_I2C_Master_Transmit(&hi2c1, address, Data, 2, 1000);
}

