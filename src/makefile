
# ---------------------------------------------------------------------
# Cplex, Concert, Lemon paths : USER SHOULD CHANGE IT ACCORDING TO ITS NEEDS
# ---------------------------------------------------------------------
SYSTEM = x86-64_linux
LIBFORMAT = static_pic
CPLEX_INSTALL_DIR = /opt/ibm/ILOG/CPLEX_Studio2211

CONCERT_DIR = $(CPLEX_INSTALL_DIR)/concert
CONCERT_INC_DIR = $(CONCERT_DIR)/include
CONCERT_LIB_DIR = $(CONCERT_DIR)/lib/$(SYSTEM)/$(LIBFORMAT)

CPLEX_DIR = $(CPLEX_INSTALL_DIR)/cplex
CPLEX_INC_DIR = $(CPLEX_DIR)/include
CPLEX_LIB_DIR = $(CPLEX_DIR)/lib/$(SYSTEM)/$(LIBFORMAT)

CPLEX_INC_FLAGS = $(CCOPT) -I$(CPLEX_INC_DIR) -I$(CONCERT_INC_DIR)
CPLEX_LIB_FLAGS = -L$(CPLEX_LIB_DIR) -lilocplex -lcplex -L$(CONCERT_LIB_DIR) -lconcert -lm -m64 -lpthread -ldl

LEMON_FLAGS = -lemon

# ---------------------------------------------------------------------
# Compiler options
# ---------------------------------------------------------------------
CCC = g++ -O2 -std=c++11
CCOPT = -m64 -O -fPIC -fno-strict-aliasing -fexceptions -DNDEBUG -DIL_STD -Wno-ignored-attributes 


#---------------------------------------------------------
# .cpp Files
#---------------------------------------------------------
CPPFILES = main.cpp instance/*.cpp network/*.cpp solver/*.cpp tools/*.cpp

#---------------------------------------------------------
# Files
#---------------------------------------------------------
all: main

main:
	$(CCC) -c -Wall -g $(CPLEX_INC_FLAGS) $(CPPFILES)
	$(CCC) $(CPLEX_INC_FLAGS) *.o -g -o exec $(CPLEX_LIB_FLAGS) $(LEMON_FLAGS)
	rm -rf *.o *~ ^

clean:
	rm -rf *.o exec