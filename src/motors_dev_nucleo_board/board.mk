# List of all the board related files.
BOARDSRC = ./motors_dev_nucleo_board/board.c

# Required include directories
BOARDINC = ./motors_dev_nucleo_board/

# Shared variables
ALLCSRC += $(BOARDSRC)
ALLINC  += $(BOARDINC)
