#include "HC-06.h"

HAL_StatusTypeDef HC06_check(HC06_t *hc06)
{
	char *packet = "AT";
	char *answer;
	HAL_UART_Transmit(hc06->usart, (uint8_t*)packet, 2, 100);
	HAL_UART_Receive(hc06->usart, (uint8_t*)answer, 2, 100);
	if (answer[0] == 'O' && answer[1] == 'K')
		return HAL_OK;
	return HAL_ERROR;
}

HAL_StatusTypeDef HC06_set_name(HC06_t *hc06)
{
	char *packet = "AT+NAMEBT_Car";
	
	uint8_t size;
	for (size = 0; packet[size] != '\0'; size++) ;
	
	
	char *answer;
	HAL_UART_Transmit(hc06->usart, (uint8_t*)packet, size + size, 1000);
	HAL_UART_Receive(hc06->usart, (uint8_t*)answer, 2, 1000);
	if (answer[0] == 'O' && answer[1] == 'K')
		return HAL_OK;
	return HAL_ERROR;
}

HAL_StatusTypeDef HC06_set_pass(HC06_t *hc06)
{
	char *packet = "AT+PIN1111";
	
	uint8_t size;
	for (size = 0; packet[size] != '\0'; size++) ;
	
	char *answer;
	HAL_UART_Transmit(hc06->usart, (uint8_t*)packet, size, 1000);
	HAL_UART_Receive(hc06->usart, (uint8_t*)answer, 2, 1000);
	if (answer[0] == 'O' && answer[1] == 'K')
		return HAL_OK;
	return HAL_ERROR;
}

HAL_StatusTypeDef HC06_get_version(HC06_t *hc06)
{
	char *packet = "AT+VERSION";
	uint8_t cmd_len = 10;
	
	char *answer;
	HAL_UART_Transmit(hc06->usart, (uint8_t*)packet, cmd_len, 1000);
	HAL_UART_Receive(hc06->usart, (uint8_t*)answer, 5, 1000);
	if (answer[0] == 'O' && answer[1] == 'K')
		return HAL_OK;
	return HAL_ERROR;
}

HAL_StatusTypeDef HC06_set_parity(HC06_t *hc06, uint8_t mode)
{
	
	char *packet;
	if (mode == 0)
		packet = "AT+PN";
	else if (mode == 1)
		packet = "AT+PO";
	else
		packet = "AT+PE";
	uint8_t cmd_len = 5;
	
	char *answer;
	HAL_UART_Transmit(hc06->usart, (uint8_t*)packet, cmd_len, 1000);
	HAL_UART_Receive(hc06->usart, (uint8_t*)answer, 2, 1000);
	if (answer[0] == 'O' && answer[1] == 'K')
		return HAL_OK;
	return HAL_ERROR;
}

HAL_StatusTypeDef HC06_set_baud(HC06_t *hc06, HC06_RATE baudrate)
{
	
	char *packet;
	switch (baudrate)
	{
	case HC06_BAUD_1200:
		packet = "AT+BAUD1";
		break;
	case HC06_BAUD_2400:
		packet = "AT+BAUD2";
		break;
	case HC06_BAUD_4800:
		packet = "AT+BAUD3";
		break;
	case HC06_BAUD_9600:
		packet = "AT+BAUD4";
		break;
	case HC06_BAUD_19200:
		packet = "AT+BAUD5";
		break;
	case HC06_BAUD_38400:
		packet = "AT+BAUD6";
		break;
	case HC06_BAUD_57600:
		packet = "AT+BAUD7";
		break;
	case HC06_BAUD_115200:
		packet = "AT+BAUD8";
		break;
	case HC06_BAUD_230400:
		packet = "AT+BAUD9";
		break;
	}
	uint8_t cmd_len = 8;
	
	char *answer;
	HAL_UART_Transmit(hc06->usart, (uint8_t*)packet, cmd_len, 1000);
	HAL_UART_Receive(hc06->usart, (uint8_t*)answer, 2, 1000);
	if (answer[0] == 'O' && answer[1] == 'K')
		return HAL_OK;
	return HAL_ERROR;
}

