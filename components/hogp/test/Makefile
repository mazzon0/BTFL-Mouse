CC = gcc
CFLAGS = -I../include -I../src -DHOGP_TEST

all: run_tests

run_tests: test_serializer.c ../src/hogp_msg_serializer.c
	$(CC) $(CFLAGS) $^ -o $@

run: run_tests
	./run_tests