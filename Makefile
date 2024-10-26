# Makefile to build the project
# NOTE: This file must not be changed

# Parameters
OS := $(shell uname -s)

CC = gcc
CFLAGS = -Wall

SRC = src/
INCLUDE = include/
BIN = bin/
CABLE_DIR = cable/

ifeq ($(OS), Linux)
	TX_SERIAL_PORT = /dev/ttyS10
	RX_SERIAL_PORT = /dev/ttyS11
else
	TX_SERIAL_PORT = /tmp/ttyS10
	RX_SERIAL_PORT = /tmp/ttyS11
endif

BAUD_RATE = 9600

TX_FILE = penguin.gif
RX_FILE = penguin-received.gif

# Targets
.PHONY: all
all: $(BIN)/main $(BIN)/cable

$(BIN)/main: main.c $(SRC)/*.c
	$(CC) $(CFLAGS) -o $@ $^ -I$(INCLUDE)

ifeq ($(OS), Linux)
$(BIN)/cable: $(CABLE_DIR)/cable.c
else
$(BIN)/cable: $(CABLE_DIR)/cable_macos.c
endif
	$(CC) $(CFLAGS) -o $@ $^

.PHONY: run_tx
run_tx: $(BIN)/main
	./$(BIN)/main $(TX_SERIAL_PORT) $(BAUD_RATE) tx $(TX_FILE)

.PHONY: run_rx
run_rx: $(BIN)/main
	./$(BIN)/main $(RX_SERIAL_PORT) $(BAUD_RATE) rx $(RX_FILE)

.PHONY: run_cable
run_cable: $(BIN)/cable
	./$(BIN)/cable

.PHONY: check_files
check_files:
	diff -s $(TX_FILE) $(RX_FILE) || exit 0

.PHONY: clean
clean:
	rm -f $(BIN)/main
	rm -f $(BIN)/cable
	rm -f $(RX_FILE)
