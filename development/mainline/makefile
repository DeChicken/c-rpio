# Define what compiler to use
CC = gcc

# All source files currenly in the working directory
SRCS := $(wildcard *.c)

# Names of all possible object files
OBJS := $(SRCS:%.c=%.o)

# Names of all possible executables
BINS := $(SRCS:%.c=%)

all: ${OBJS} main

# Compile sources into object files
%.o: %.c
	${CC} -c $<

main: ${OBJS}
	${CC} -o main ${OBJS} -lm

clean:
	@echo 'Removing all objects and executables...'
	@rm -rvf *.o ${BINS}
