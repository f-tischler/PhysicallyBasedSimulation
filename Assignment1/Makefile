CC = g++
LD = g++

SRC = $(wildcard *.cpp)
OBJ = $(patsubst %.cpp,%.o,$(SRC))
TARGET = MassSpring

CFLAGS = -O3 -Wall
LDLIBS = -lGL -lglut
INCLUDES = 

SRC_DIR = 
BUILD_DIR = 
VPATH = 

# Rules
all: $(TARGET)

$(TARGET).o: $(TARGET).cpp
	$(CC) $(CFLAGS) $(INCLUDES) -c $^ -o $@

%.o: %.cpp
	$(CC) $(CFLAGS) $(INCLUDES) -c $^ -o $@

clean:
	rm -f *.o $(TARGET)

.PHONY: clean

# Dependencies
$(TARGET): $(OBJ) 

