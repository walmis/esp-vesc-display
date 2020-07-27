#
# "main" pseudo-component makefile.
#
# (Uses default behaviour of compiling all source files in directory, adding 'include' to include path.)

COMPONENT_SRCDIRS += vesc fonts img
CFLAGS += -DNO_LIBOPENCM3=1 -DFREERTOS 
CPPFLAGS += -Wno-error=narrowing
