#*************************************************************************
#Title:    MRB-AVRTEMPLATE Makefile
#Authors:  Michael Petersen <railfan@drgw.net>
#          Nathan Holmes <maverick@drgw.net>
#File:     MRB-AVRTEMPLATE Makefile
#License:  GNU General Public License v3
#
#LICENSE:
#    Copyright (C) 2012 Nathan Holmes and Michael Petersen
#
#    Derived from example WinAVR project Makefile
#
#    This program is free software; you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation; either version 3 of the License, or
#    any later version.
#
#    This program is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.
#    
#    You should have received a copy of the GNU General Public License along 
#    with this program. If not, see http://www.gnu.org/licenses/
#    
#*************************************************************************

DEFINES =
INCS = 

INCLUDES = -I.
CFLAGS  = $(INCLUDES) -Wall -Os -std=gnu99

COMPILE = gcc $(DEFINES) $(CFLAGS)

all: panel test

panel: 
	$(COMPILE) panel.c -o panel

test: 
	$(COMPILE) test.c -o test

# rule for deleting dependent files (those which can be built by Make):
clean:
	rm -f $(OBJS) *.o *~ panel test

PWD := $(shell pwd)
