HOME=/home/olivier/projets
PROJET=ardpilot

CC=gcc
CFLAGS=-c -g
LDFLAGS=-lm

SRC=$(HOME)/$(PROJET)/src
OBJ=$(HOME)/$(PROJET)/obj
BIN=$(HOME)/$(PROJET)/bin

ALL_OBJS=$(OBJ)/$(PROJET).o \
	$(OBJ)/environment.o \
	$(OBJ)/read_config.o \
	$(OBJ)/drawing.o \
	$(OBJ)/calculs.o \
	$(OBJ)/navigation.o \
	$(OBJ)/commandes.o

$(BIN)/$(PROJET) : $(ALL_OBJS)
	$(CC) $(LDFLAGS) -o $@ $(ALL_OBJS) $(LDFLAGS)

$(OBJ)/%.o : $(SRC)/%.c ardpilot.h
	$(CC) $(CFLAGS) -o $@ $<
