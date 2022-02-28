#include "cy_pdl.h"
#include <stdio.h>
#include "book.h"

// Function to print all of the book data
// Params:
// IN:
// base			- The pointer to the UART SCB instance
// bookToPrint	- The book to print
void printBookInfo(CySCB_Type *base, struct book bookToPrint){
	Cy_SCB_UART_PutString(base, "Title: ");
	Cy_SysLib_Delay(10);
	Cy_SCB_UART_PutArray(base, bookToPrint.title, strlen(bookToPrint.title));
	Cy_SysLib_Delay(10);
	Cy_SCB_UART_PutString(base, "\r\n");
	Cy_SysLib_Delay(10);
	Cy_SCB_UART_PutString(base, "Author: ");
	Cy_SysLib_Delay(10);
	Cy_SCB_UART_PutArray(base, bookToPrint.author, strlen(bookToPrint.author));
	Cy_SysLib_Delay(10);
	Cy_SCB_UART_PutString(base, "\r\n");
	Cy_SysLib_Delay(10);
	Cy_SCB_UART_PutString(base, "Page Count: ");
	Cy_SysLib_Delay(10);
	char buffer[10];
	sprintf(buffer, "%d", bookToPrint.pageCount);
	Cy_SCB_UART_PutArray(base, buffer, strlen(buffer));
	Cy_SysLib_Delay(10);
	Cy_SCB_UART_PutString(base, "\r\n\r\n");
	Cy_SysLib_Delay(10);
}
