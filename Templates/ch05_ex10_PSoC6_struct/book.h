#ifndef BOOK_H
#define BOOK_H

struct book {
	char title[50];
	char author[50];
	uint16_t pageCount;
};

void printBookInfo(CySCB_Type *base, struct book bookToPrint); // defined in book.c

#endif
