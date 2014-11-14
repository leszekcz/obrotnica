/**************************************************************************//*****
 * @file     stdio.c
 * @brief    Implementation of newlib syscall
 ********************************************************************************/

#include <stdio.h>
#include <stdarg.h>
#include <sys/types.h>
#include <sys/stat.h>
#include "cmsis_lib/include/stm32f4xx_usart.h"

#undef errno
extern int errno;
extern int  _end;

/*This function is used for handle heap option*/
__attribute__ ((used))
caddr_t _sbrk ( int incr )
{
    static unsigned char *heap = NULL;
    unsigned char *prev_heap;

    if (heap == NULL) {
        heap = (unsigned char *)&_end;
    }
    prev_heap = heap;

    heap += incr;

    return (caddr_t) prev_heap;
}

__attribute__ ((used))
int link(char *old, char *new)
{
    return -1;
}

__attribute__ ((used))
int _close(int file)
{
    return -1;
}

__attribute__ ((used))
int _fstat(int file, struct stat *st)
{
    st->st_mode = S_IFCHR;
    return 0;
}

__attribute__ ((used))
int _isatty(int file)
{
    return 1;
}

__attribute__ ((used))
int _lseek(int file, int ptr, int dir)
{
    return 0;
}

/*Low layer read(input) function*/
__attribute__ ((used))
int _read(int file, char *ptr, int len)
{

#if 0
     //user code example
     int i;
     (void)file;

     for(i = 0; i < len; i++)
     {
        // UART_GetChar is user's basic input function
        *ptr++ = UART_GetChar();
     }

#endif

    return len;
}


/*Low layer write(output) function*/
__attribute__ ((used))
int _write( int file, char *ptr, int len )

{

int txCount;

  (void)file;
  GPIOD->BSRRL = GPIO_Pin_10;
  for ( txCount = 0; txCount < len; txCount++)

  {

  USART_SendData(USART3, ptr[txCount]);   //Don't forget to include "stm32f10x_usart.h"

  /* Loop until the end of transmission */

   	   while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET)
   {
   		GPIOD->BSRRL = GPIO_Pin_10;
   }
//   	GPIOD->BSRRH = GPIO_Pin_10;
  }

  return len;

}

__attribute__ ((used))
void abort(void)
{
    /* Abort called */
    while(1);
}

/* --------------------------------- End Of File ------------------------------ */
