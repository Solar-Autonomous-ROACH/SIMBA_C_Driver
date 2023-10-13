
# Makefile

# Compiler options
CC = gcc
CFLAGS = -Wall -Wextra

# Source files
SRCS = main.c mmio.c led.c move.c isr.c motor.c steering_motor.c rover.c

# Object files
OBJS = $(SRCS:.c=.o)

# Executable file
TARGET = roverCore

# Targets
all: $(TARGET)

$(TARGET): $(OBJS)
	$(CC) $(CFLAGS) $(OBJS) -o $(TARGET)

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -f $(OBJS) $(TARGET)
