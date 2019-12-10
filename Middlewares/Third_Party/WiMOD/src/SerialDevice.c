//------------------------------------------------------------------------------
//
//	File:		SerialDevice.cpp
//
//	Abstract:	Serial Device Abstraction
//
//	Version:	0.1
//
//	Date:		18.05.2016
//
//	Disclaimer:	This example code is provided by IMST GmbH on an "AS IS" basis
//				without any warranties.
//
//	Maintain by : Anol Paisal <anol.paisal@emone.co.th> @ 2018
//
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//
//  Include Files
//
//------------------------------------------------------------------------------

#include "SerialDevice.h"
#include "WiMOD_LoRaWAN_API.h"
#include "main.h"
//------------------------------------------------------------------------------
//
//  Section RAM
//
//------------------------------------------------------------------------------

#ifdef Q_OS_WIN

// File Handle
static HANDLE   ComHandle = INVALID_HANDLE_VALUE;

#else
static UART_HandleTypeDef *hWiModUart;
#endif

#define USART_STATUS_REGISTER   SR                     //!< HAL USART status register name adapter.
#define USART_DATA_REGISTER     DR                     //!< HAL UART data register name adapter.

//------------------------------------------------------------------------------
//
//  Section Code
//
//------------------------------------------------------------------------------
uint8_t UsartTextString;
//------------------------------------------------------------------------------
//
//  Open
//
//  @brief: open serial device
//
//------------------------------------------------------------------------------

bool
SerialDevice_Open(
#ifdef Q_OS_WIN
		const char*   comPort,
#else
		UART_HandleTypeDef * huart,
#endif
                  UINT32        baudRate,
                  int           dataBits,
                  UINT8         parity)
{

#ifdef Q_OS_WIN
    // handle valid ?
    if (ComHandle != INVALID_HANDLE_VALUE)
        SerialDevice_Close();

    char devName[80];

    // windows workaround for COM Ports higher than COM9
    strcpy(devName, "\\\\.\\");
    strcat(devName, comPort);


    ComHandle = CreateFileA(devName,
                            GENERIC_WRITE | GENERIC_READ,
                            0,
                            NULL,
                            OPEN_EXISTING,
                            0, //FILE_FLAG_WRITE_THROUGH, //0
                            NULL);

    // handle valid ?
    if (ComHandle != INVALID_HANDLE_VALUE)
    {
        DCB dcb;
        if (GetCommState(ComHandle, &dcb))
        {
            dcb.DCBlength           = sizeof(DCB);
            dcb.BaudRate            = baudRate;
            dcb.ByteSize            = dataBits;
            dcb.Parity              = parity; //EVENPARITY;// NOPARITY;
            dcb.StopBits            = ONESTOPBIT;
            dcb.fOutxCtsFlow        = FALSE;
            dcb.fOutxDsrFlow        = FALSE;
            dcb.fDtrControl         = DTR_CONTROL_DISABLE;
            dcb.fDsrSensitivity     = FALSE;
            dcb.fTXContinueOnXoff   = FALSE;
            dcb.fOutX               = FALSE; // no XON/XOFF
            dcb.fInX                = FALSE; // no XON/XOFF
            dcb.fErrorChar          = FALSE;
            dcb.fNull               = FALSE;
            dcb.fRtsControl         = RTS_CONTROL_DISABLE;
            dcb.fAbortOnError       = FALSE;

            if (SetCommState(ComHandle, &dcb))
            {
                COMMTIMEOUTS commTimeouts;
                commTimeouts.ReadIntervalTimeout 		= MAXDWORD;
                commTimeouts.ReadTotalTimeoutMultiplier = 0;
                commTimeouts.ReadTotalTimeoutConstant 	= 0;

                commTimeouts.WriteTotalTimeoutMultiplier = 10;
                commTimeouts.WriteTotalTimeoutConstant  = 1;

                SetCommTimeouts(ComHandle, &commTimeouts);

                // ok
                return true;
            }
        }
        // close device
        SerialDevice_Close();
    }
#else
    // TODO : add your own platform specific code here
    if (huart != NULL) {
    		hWiModUart = huart;
  	}
    //    if (hWiModUart->gState == HAL_UART_STATE_READY) {
//		return true;
//	}
	hWiModUart->Instance = USART6;
	hWiModUart->Init.Mode = UART_MODE_TX_RX;
	hWiModUart->Init.HwFlowCtl = UART_HWCONTROL_NONE;
	hWiModUart->Init.OverSampling = UART_OVERSAMPLING_16;
//	hWiModUart->RxCpltCallback = USART_ITCharManager;
	hWiModUart->Init.BaudRate = baudRate;
	hWiModUart->Init.WordLength = dataBits;
	hWiModUart->Init.StopBits = UART_STOPBITS_1;
	hWiModUart->Init.Parity = parity;

	if (HAL_UART_Init(hWiModUart) == HAL_OK)
	{
		HAL_UART_Receive_IT(hWiModUart, &UsartTextString, 1);
		return true;
	}

	SerialDevice_Close();
	Error_Handler();
//	if (HAL_UARTEx_SetTxFifoThreshold(&huart, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
//	{
//	_Error_Handler(__FILE__, __LINE__);
//	}
//
//	if (HAL_UARTEx_SetRxFifoThreshold(&huart, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
//	{
//	_Error_Handler(__FILE__, __LINE__);
//	}
//
//	if (HAL_UARTEx_DisableFifoMode(&huart) != HAL_OK)
//	{
//	_Error_Handler(__FILE__, __LINE__);
//	}

#endif
    // error
    return false;
}

//------------------------------------------------------------------------------
//
//  Close
//
//  @brief: close serial device
//
//------------------------------------------------------------------------------

bool
SerialDevice_Close()
{
#ifdef Q_OS_WIN
    // handle valid ?
    if (ComHandle != INVALID_HANDLE_VALUE)
    {
        // cancel last operation
        CancelIo(ComHandle);

        // wait 100us
        Sleep(100);

        // close device
        CloseHandle(ComHandle);

        // invalidate handle
        ComHandle = INVALID_HANDLE_VALUE;

        // ok
        return true;
    }
#else
    // Todo : add your own platform specific code here
    HAL_UART_DeInit(hWiModUart);
#endif
    // error
    return false;
}

//------------------------------------------------------------------------------
//
//  SendData
//
//  @brief: send data
//
//------------------------------------------------------------------------------

int
SerialDevice_SendData(UINT8* txBuffer, size_t txLength)
{
#ifdef Q_OS_WIN
    // handle valid ?
    if (ComHandle == INVALID_HANDLE_VALUE)
        return -1;

    UINT32  numTxBytes;

    // write chunk of data
    if (!WriteFile(ComHandle, txBuffer, txLength, (DWORD*)&numTxBytes, 0))
    {
        // error
        return -1;
    }
    // all bytes written ?
    if (numTxBytes == (UINT32)txLength)
    {
        // ok
        return numTxBytes;
    }
#else
    // Todo : add your own platform specific code here
	if(USART_Transmit(hWiModUart, txBuffer, txLength) != HAL_ERROR)
	{
		return txLength;
	}
#endif
    // error
    return -1;
}

//------------------------------------------------------------------------------
//
//  SendByte
//
//  @brief: send single byte
//
//------------------------------------------------------------------------------

int
SerialDevice_SendByte(UINT8 txByte)
{
#ifdef Q_OS_WIN
    // handle valid ?
    if (ComHandle == INVALID_HANDLE_VALUE)
        return -1;

    UINT32  numTxBytes;

    // write chunk of data
    if (!WriteFile(ComHandle, &txByte, 1, (DWORD*)&numTxBytes, 0))
    {
        // error
        return -1;
    }
    // all bytes written ?
    if (numTxBytes == 1)
    {
        // ok
        return numTxBytes;
    }
#else
    // Todo : add your own platform specific code here
    if(USART_Transmit(hWiModUart, &txByte, 1) != HAL_ERROR)
	{
		return 1;
	}
#endif
    // error
    return -1;
}

//------------------------------------------------------------------------------
//
//  ReadData
//
//  @brief: read data
//
//------------------------------------------------------------------------------

int
SerialDevice_ReadData(UINT8* rxBuffer, size_t rxBufferSize)
{
#ifdef  Q_OS_WIN
    // handle ok ?
    if (ComHandle == INVALID_HANDLE_VALUE)
        return -1;

    DWORD numRxBytes = 0;

    // read chunk of data
    if (ReadFile(ComHandle, rxBuffer, rxBufferSize, &numRxBytes, 0))
    {
        // return number of bytes read
        return (int)numRxBytes;
    }
#else
    // Todo : add your own platform specific code here
    *rxBuffer = UsartTextString;
	rxBufferSize = 1;
	return rxBufferSize;
//    if(HAL_UART_Receive(hWiModUart, rxBuffer, rxBufferSize, 100) != HAL_ERROR)
//    {
//    	return rxBufferSize;
//    }
#endif
    // error
//    return -1;
}

/**
 * @brief  Send a text string via USART.
 * @param  huart       pointer to a UART_HandleTypeDef structure that contains
 *                     the configuration information for the specified UART module.
 * @param  TextString  The text string to be sent.
 * @note It use the HAL_UART_Transmit function.
 */
HAL_StatusTypeDef USART_Transmit(UART_HandleTypeDef* huart, uint8_t* data, size_t len) {
	return HAL_UART_Transmit(huart, data, len, 500);
}

/**
 * @brief  Handle text character just received.
 * @param  huart pointer to a UART_HandleTypeDef structure that contains
 *               the configuration information for the specified UART module.
 * @note To use inside USARx_IRQHandler function.
 */
void USART_ITCharManager(UART_HandleTypeDef* huart) {
	uint8_t UART_Receive_IT_Char;

	UART_Receive_IT_Char = (uint8_t) (huart->Instance->USART_DATA_REGISTER);
	/* Checks the buffer full or return carriage  */
	if ((huart->RxXferCount == 1) || (UART_Receive_IT_Char == '\r')) {
		huart->RxXferCount += 1;
		huart->pRxBuffPtr -= 1;
		*(huart->pRxBuffPtr) = '\0';

		USART_Transmit(huart, (uint8_t *) "\r\n" , 2);

		while (HAL_IS_BIT_SET(huart->Instance->USART_STATUS_REGISTER,
				UART_FLAG_RXNE)) {
		}
		__HAL_UART_DISABLE_IT(huart, UART_IT_RXNE);

		/* Check if a transmit process is ongoing or not */
		if (huart->gState == HAL_UART_STATE_BUSY_TX_RX) {
			huart->gState = HAL_UART_STATE_BUSY_TX;
		} else {
			/* Disable the UART Parity Error Interrupt */
			__HAL_UART_DISABLE_IT(huart, UART_IT_PE);

			/* Disable the UART Error Interrupt: (Frame error, noise error, overrun error) */
			__HAL_UART_DISABLE_IT(huart, UART_IT_ERR);

			huart->gState = HAL_UART_STATE_READY;
		}
	}
}

/**
 * @brief  This function converts a text string into a number.
 * @param  str       The pointer to the text string to convert.
 * @param  pnum      The pointer to the numerical variable.
 * @retval FlagStatus  SET or RESET related to the conversion.
 */
FlagStatus str2num(uint8_t* str, uint32_t* pnum) {
	uint8_t TxtStr_digit, digit;
	uint32_t tenpwr;
	uint32_t number;

	digit = 0;

	while (*(str + digit) != '\0') {
		if (((*(str + digit) >= '0') && (*(str + digit) <= '9'))) {
			digit++;
		} else {
			*pnum = 0;
			return RESET;
		}
	}

	tenpwr = 1;
	number = 0;

	do {
		TxtStr_digit = (*(str + (--digit)));
		number += ((TxtStr_digit - '0') * tenpwr);
		tenpwr *= 10;
	} while (digit);

	*pnum = number;
	return SET;
}

/**
 * @brief  Convert a number nbr into a string str with 7 characters.
 * @param  nbr The number to be converted.
 * @param  str The container of the converted number into a text in decimal
 *         format.
 * @note   The decimal digits of the number must be maximum 7 so str has to be
 *         able to store at least 7 characters plus '\0'.
 */
void num2str(uint32_t nbr, uint8_t *str) {
	uint8_t k;
	uint8_t *pstrbuff;
	uint32_t divisor;

	pstrbuff = str;

	/* Reset the text string */
	for (k = 0; k < 7; k++)
		*(pstrbuff + k) = '\0';

	divisor = 1000000;

	if (nbr) // if nbr is different from zero then it is processed
	{
		while (!(nbr / divisor)) {
			divisor /= 10;
		}

		while (divisor >= 10) {
			k = nbr / divisor;
			*pstrbuff++ = '0' + k;
			nbr = nbr - (k * divisor);
			divisor /= 10;
		}
	}

	*pstrbuff++ = '0' + nbr;
	*pstrbuff++ = '\0';
}

/**
 * @brief  Convert an integer number into hexadecimal format.
 *
 * @param  num         The integer number to convert.
 * @param  HexFormat   The output format about hexadecimal number.
 *
 * @retval uint8_t*    The address of the string text for the converted hexadecimal number.
 */
uint8_t* num2hex(uint32_t num, eHexFormat HexFormat) {
	static uint8_t HexValue[8 + 1];
	uint8_t i;
	uint8_t dummy;
	uint8_t HexDigits = 0;

	switch (HexFormat) {
	case HALFBYTE_F:
		HexDigits = 1;
		break;
	case BYTE_F:
		HexDigits = 2;
		break;
	case WORD_F:
		HexDigits = 4;
		break;
	case DOUBLEWORD_F:
		HexDigits = 8;
		break;
	default:
		HexDigits = 2;
		break;
	}

	for (i = 0; i < HexDigits; i++) {
		HexValue[i] = '\0';
		dummy = (num & (0x0F << (((HexDigits - 1) - i) * 4)))
				>> (((HexDigits - 1) - i) * 4);
		if (dummy < 0x0A) {
			HexValue[i] = dummy + '0';
		} else {
			HexValue[i] = (dummy - 0x0A) + 'A';
		}
	}
	HexValue[i] = '\0';

	return HexValue;
}

//------------------------------------------------------------------------------
// end of file
//------------------------------------------------------------------------------
