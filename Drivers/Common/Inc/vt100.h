/*
 * vt100.h
 *
 *  Created on: Apr 7, 2021
 *      Author: Margit Robert
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef COMMON_INC_VT100_H_
#define COMMON_INC_VT100_H_
/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/*
 * https://www.csie.ntu.edu.tw/~r92094/c++/VT100.html
 * http://www.termsys.demon.co.uk/vtansi.htm
 */
#define VT100_CLEARSCREEN   "\033[2J"
#define VT100_CURSORHOME    "\033[H"
#define VT100_ATTR_RESET    "\033[0m"
#define VT100_ATTR_RED      "\033[31m"
#define VT100_ATTR_GREEN    "\033[32m"
#define VT100_ATTR_YELLOW   "\033[33m"


/* Exported macro ------------------------------------------------------------*/
#define VT100_CUP(__v__,__h__)    ("\033["__v__";"__h__"H") /*Cursor Position*/
/* Exported functions ------------------------------------------------------- */

#endif /* COMMON_INC_VT100_H_ */

/************************ (C) COPYRIGHT KonvolucioBt ***********END OF FILE****/
