main: main.o ../rover/librover.a
	gcc -Wall -g -L../rover -o main main.o -lrover

main.o: main.c
	gcc -Wall -g -I ../rover -c main.c

clean:
	rm -f main.o
