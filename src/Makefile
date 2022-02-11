CC=g++

DEFINES= -D$(MODEL) -D$(IF) -D$(PLATFORM)
INCLUDES= .
CFLAGS= -I$(INCLUDES) $(DEFINES) -Wall
LIBS=
DEPS= hcl.h hcl_gpio.h sensor_epsonCommon.h main_helper.h
DEPS_IMUMODEL= sensor_epsonG354.h sensor_epsonG364PDC0.h sensor_epsonG364PDCA.h sensor_epsonV340.h sensor_epsonG320.h sensor_epsonG365PDC0.h sensor_epsonG365PDF0.h sensor_epsonG370PDF0.h sensor_epsonG325PDF0.h sensor_epsonG365PDC1.h sensor_epsonG365PDF1.h sensor_epsonG370PDF1.h sensor_epsonG325PDF1.h
_OBJ= sensor_epsonCommon.o main_helper.o

# If no MODEL macro is defined when calling make it defaults to G365PDF0
MODEL ?= G365PDF0

# defaults Interface to SPI
IF = SPI

# defaults to RPI
PLATFORM = RPI

####### Adding MODEL Specific Files 
ifeq ($(MODEL), G354)
	_OBJ+= sensor_epsonG354.o
endif

ifeq ($(MODEL), G364PDC0)
	_OBJ+= sensor_epsonG364.o
endif

ifeq ($(MODEL), G364PDCA)
	_OBJ+= sensor_epsonG364.o
endif

ifeq ($(MODEL), V340)
	_OBJ+= sensor_epsonV340.o
endif

ifeq ($(MODEL), G320)
	_OBJ+= sensor_epsonG320.o
endif

ifeq ($(MODEL), G365PDC0)
	_OBJ+= sensor_epsonG365.o
endif

ifeq ($(MODEL), G365PDF0)
	_OBJ+= sensor_epsonG365.o
endif

ifeq ($(MODEL), G370PDF0)
	_OBJ+= sensor_epsonG370.o
endif

ifeq ($(MODEL), G325PDF0)
	_OBJ+= sensor_epsonG325.o
endif

ifeq ($(MODEL), G365PDC1)
	_OBJ+= sensor_epsonG365.o
endif

ifeq ($(MODEL), G365PDF1)
	_OBJ+= sensor_epsonG365.o
endif

ifeq ($(MODEL), G370PDF1)
	_OBJ+= sensor_epsonG370.o
endif

ifeq ($(MODEL), G325PDF1)
	_OBJ+= sensor_epsonG325.o
endif

####### Adding Interface Specific Files 
ifeq ($(IF), SPI)
	_OBJ+= sensor_epsonSpi.o
	DEPS+= hcl_spi.h
endif

####### Adding PLATFORM Specific Files 
ifeq ($(PLATFORM), RPI)
	_OBJ+= hcl_rpi.o hcl_spi_rpi.o hcl_gpio_rpi.o

	LIBS+= -lwiringPi -lpthread -lcrypt -lrt

endif

OBJ_SCREEN= main_screen.o $(_OBJ)
OBJ_CSV= main_csvlogger.o $(_OBJ)
OBJ_REG= main_regdump.o $(_OBJ)

%.o: %.c $(DEPS) $(DEPS_IMUMODEL)
	$(CC) -c -o $@ $< $(CFLAGS) $(LIBS)

screen: $(OBJ_SCREEN)
	$(CC) -o $@ $^ $(CFLAGS) $(LIBS)

csvlogger: $(OBJ_CSV)
	$(CC) -o $@ $^ $(CFLAGS) $(LIBS)

regdump: $(OBJ_REG)
	$(CC) -o $@ $^ $(CFLAGS) $(LIBS)

.PHONY: clean all tar help

clean:
	rm -f *.o *~ core *~
	rm -f csvlogger screen regdump

all: screen csvlogger regdump

tar:
	tar cvzf archive.tar.gz *.c *.h readme.txt Makefile

help:
	@echo "supported make commands are:"
	@echo "\tmake clean"
	@echo "\tmake <targets> MODEL=<model>\n"
	@echo "valid <targets> are: csvlogger screen or regdump\n"
	@echo "valid <models> are:"
	@echo "\tG354 G364PDC0 G364PDCA G320 V340 G365PDC0 G365PDF0(default) G370PDF0 G325PDF0"
	@echo "\tG365PDC1 G365PDF1 G370PDF1 G325PDF1\n"
	@echo "example:\n\tmake csvlogger MODEL=G364PDC0"
	@echo "\tmake screen (defaults to MODEL=G365PDF0)"
