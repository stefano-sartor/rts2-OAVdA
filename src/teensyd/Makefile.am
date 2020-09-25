bin_PROGRAMS = rts2-teensy

rts2_teensy_SOURCES = connection_tcp.cpp server_tcp.cpp teensy.cpp
rts2_teensy_LDFLAGS =  -L../../lib/oavda -lrts2oavda  -lboost_system -lpthread ${LDADD}
rts2_teensy_CPPFLAGS = -I../../include
