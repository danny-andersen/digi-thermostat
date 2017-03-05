include Makefile.inc

PROGRAMS = masterstation

SOURCES = $(PROGRAMS:=.cpp) ../digi-thermostat.h

all: $(PROGRAMS)

$(PROGRAMS): $(SOURCES)
	$(CXX) $(CFLAGS) $(INC_HEADER_DIR) -I.. -L$(LIB_DIR) $@.cpp $(LIBS) -o $@

clean:
	@echo "[Cleaning]"
	rm -rf $(PROGRAMS)


