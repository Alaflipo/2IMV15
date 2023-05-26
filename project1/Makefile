INC_WINDOWS = include/windows
INC_LINUX   = include/linux
INC_COMMON  = include/common

ifeq ($(OS),Windows_NT)
	RM = rm -vf # might have to be adjusted to del for some Windows distros
	INCLUDE = $(INC_WINDOWS)
	CXX_EXTRA_FLAGS = -Llib -lfreeglut -lglu32 -lopengl32 -lpng12
else
	RM = rm -vf
	INCLUDE = $(INC_LINUX)
	CXX_EXTRA_FLAGS = -lglut -lGLU -lGL -lpng
endif

CXXFLAGS = -g -O2 -Wall -Wno-sign-compare -I$(INCLUDE) -I$(INC_COMMON) -DHAVE_CONFIG_H 
CXX = g++

SRC_DIR = src
OBJ_DIR = obj
BIN_DIR = bin

SOURCES = $(wildcard $(SRC_DIR)/*cpp)
OBJECTS = $(patsubst %.cpp, $(OBJ_DIR)/%.o, $(notdir $(SOURCES)))
EXECUTABLE = $(BIN_DIR)/project1.exe

all: $(OBJ_DIR) $(BIN_DIR) $(EXECUTABLE)

$(OBJ_DIR):
	mkdir -v $@

$(BIN_DIR):
	mkdir -v $@

obj/%.o: $(SRC_DIR)/%.cpp
	$(CXX) $(CXXFLAGS) -o $@ -c $<

$(EXECUTABLE): $(OBJECTS)
	$(CXX) -o $@ $^ $(CXX_EXTRA_FLAGS)

clean:
	@$(RM) $(OBJ_DIR)/*
	@$(RM) $(BIN_DIR)/*exe
