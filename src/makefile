SYSTEM = x86-64_linux
LIBFORMAT = static_pic

# ---------------------------------------------------------------------
# Compiler options
# ---------------------------------------------------------------------
CCC = g++ -O0 -std=c++11
CCOPT = -m64 -O -fPIC -fno-strict-aliasing -fexceptions -DNDEBUG -DIL_STD -Wno-ignored-attributes 
# ---------------------------------------------------------------------
# Cplex, Concert, Lemon paths : USER SHOULD CHANGE IT ACCORDING TO ITS NEEDS
# ---------------------------------------------------------------------
CONCERTVERSION = concert
CPLEXVERSION = CPLEX_Studio1210

CONCERTDIR = /opt/ibm/ILOG/$(CPLEXVERSION)/$(CONCERTVERSION)
CONCERTINCDIR = $(CONCERTDIR)/include/
CONCERTLIBDIR = $(CONCERTDIR)/lib/$(SYSTEM)/$(LIBFORMAT)

CPLEXDIR = /opt/ibm/ILOG/$(CPLEXVERSION)/cplex
CPLEXINCDIR = $(CPLEXDIR)/include/
CPLEXLIBDIR = $(CPLEXDIR)/lib/$(SYSTEM)/$(LIBFORMAT)

LEMONINCDIR = /opt/lemon/include/
LEMONLIBDIR = /opt/lemon/lib/
LEMONCFLAGS = -I$(LEMONINCDIR)
LEMONCLNFLAGS = -L$(LEMONLIBDIR) -lemon


# ---------------------------------------------------------------------
# Flags
# ---------------------------------------------------------------------
CCLNFLAGS = -L$(CPLEXLIBDIR) -lilocplex -lcplex -L$(CONCERTLIBDIR) -lconcert -lm -lpthread -ldl
CLNFLAGS  = -L$(CPLEXLIBDIR) -lcplex -lm -lpthread
CFLAGS  = $(COPT)  -I$(CPLEXINCDIR)
CCFLAGS = $(CCOPT) -I$(CPLEXINCDIR) -I$(CONCERTINCDIR)

#---------------------------------------------------------
# .cpp Files
#---------------------------------------------------------
CPPFILES = main.cpp instance/*.cpp network/*.cpp solver/*.cpp tools/*.cpp

# ---------------------------------------------------------------------
# Comands
# ---------------------------------------------------------------------
PRINTLN = echo

#---------------------------------------------------------
# Files
#---------------------------------------------------------
all: main

main:
	#doxygen ../dconfig
	$(CCC) -c -Wall -g $(CCFLAGS) $(LEMONCFLAGS) $(CPPFILES)
	$(CCC) $(CCFLAGS) *.o -g -o exec $(CCLNFLAGS) $(LEMONCLNFLAGS)
	rm -rf *.o *~ ^

clean:
	rm -rf *.o main ../Output/LP/* ../Output/*.csv ../doc/* out

