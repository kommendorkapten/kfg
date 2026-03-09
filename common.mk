CC = cc
CFLAGS = -D_POSIX_C_SOURCE=200809L -D_DARWIN_C_SOURCE \
         -Wall -W -Wextra -Wconversion -Wsign-conversion -Werror \
         -pedantic -O2 \
         -std=c11
OBJCFLAGS = -D_POSIX_C_SOURCE=200809L -D_DARWIN_C_SOURCE \
         -Wall -W -Wextra -Wconversion -Wsign-conversion -Werror \
         -pedantic -O2 \
         -fobjc-arc

LINT_FLAGS = --analyze \
	  -Xanalyzer -analyzer-checker=core \
	  -Xanalyzer -analyzer-checker=deadcode \
	  -Xanalyzer -analyzer-checker=security \
	  -Xanalyzer -analyzer-checker=unix \
	  -Xanalyzer -analyzer-checker=valist \
	  -Xanalyzer -analyzer-checker=nullability \
	  -Xanalyzer -analyzer-checker=optin

#DEBUG=1
LDFLAGS =
LIBS =

ifdef DEBUG
    CFLAGS += -g -DDEBUG
    OBJCFLAGS += -g -DDEBUG
endif

SRCS ?= $(wildcard *.c)
OBJS = $(SRCS:%.c=objs/%.o)

objs/%.o: %.c
	$(CC) $(CFLAGS) -c -o $@ $<

lint:
	cc -I../src $(LINT_FLAGS) $(SRCS)

.PHONY: all clean lint
