CC = gcc
CFLAGS = -Wall
SOURCES = $(shell echo src/*c)
INCLUDES = $(shell echo include/*.h)
LIBDIR = $(lib/)

all:
	$(CC) $(CFLAGS) -o test main.c $(SOURCES) 

clean:
	-rm -f *.o
