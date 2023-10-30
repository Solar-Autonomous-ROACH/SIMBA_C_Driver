# Makefile

# Compiler options
CC = gcc
CFLAGS = -Wall -Wextra -Werror

# Source files
SRCS = main.c mmio.c

# Object files
OBJS = $(SRCS:.c=.o)

# Executable file
TARGET = test

# Compile Everything
all: $(TARGET)

# Link the program
$(TARGET): $(OBJS)
	$(CC) $(CFLAGS) $(OBJS) -o $(TARGET)

# Compile the C source files
%.o: %.c
	$(CC) $(CFLAGS) -c $<

# Remove unneeded files
clean:
	rm -f $(OBJS) $(TARGET)
