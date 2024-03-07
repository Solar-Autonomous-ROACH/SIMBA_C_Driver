# Makefile

# Compiler options
CC = gcc
CFLAGS = -Wall -Wextra -Werror -g

# Source files
LIBSRCS = mmio.c pid.c motor.c servo.c rover.c steering_motor.c isr.c
SRCS = $(LIBSRCS) main.c

# Object files
OBJS = $(SRCS:.c=.o)
LIBOBJS = $(LIBSRCS:.c=.o)

LIBNAME = librover.a

# Executable file
TARGET = rover

# Compile Everything
all: $(TARGET) librover.a

# Compile Library
$(LIBNAME): $(LIBOBJS)
	ar r $(LIBNAME) $(LIBOBJS)

# Link the program
$(TARGET): $(OBJS)
	$(CC) $(CFLAGS) $(OBJS) -lm -o $(TARGET)

# Compile the C source files
%.o: %.c
	$(CC) $(CFLAGS) -c $<

# Remove unneeded files
clean:
	rm -f $(OBJS)

cleanall: clean
	rm -f $(OBJS) $(TARGET) $(LIBNAME)
