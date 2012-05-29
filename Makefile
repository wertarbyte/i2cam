MCU = attiny2313
F_CPU = 8000000
TARGET = i2cam
SRC = i2cam.c usiTwiSlave.c
COMBINE_SRC = 0

include avr-tmpl.mk
