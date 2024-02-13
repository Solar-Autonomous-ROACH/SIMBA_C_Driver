# Makefile

# Compiler options
CC = gcc
CFLAGS = -Wall -Wextra -Werror -g

# Source files
SRCS = main.c mmio.c pid.c motor.c servo.c rover.c

# Object files
OBJS = $(SRCS:.c=.o)

# Executable file
TARGET = rover

# Compile Everything
all: $(TARGET) librover.a

# Compile Library
librover.a: $(OBJS)
	ar r librover.a $(OBJS)

# Link the program
$(TARGET): $(OBJS)
	$(CC) $(CFLAGS) $(OBJS) -lm -o $(TARGET)

# Compile the C source files
%.o: %.c
	$(CC) $(CFLAGS) -c $<

# Remove unneeded files
clean:
	rm -f $(OBJS) $(TARGET)
