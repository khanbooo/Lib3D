CXX = g++
PY = python
CXXFLAGS = -std=c++17 -Wall -Wextra -O2 -I./Core/lib
AR = ar
ARFLAGS = rcs

CORE_LIB_DIR = ./Core/lib
TEST_DIR = ./Test
BUILD_DIR = build
LIB_DIR = $(BUILD_DIR)/lib

CORE_SOURCES = $(CORE_LIB_DIR)/vector3d.cc \
               $(CORE_LIB_DIR)/segment3d.cc \
               $(CORE_LIB_DIR)/segment3d_utils.cc \
               $(CORE_LIB_DIR)/common_utils.cc

TEST_SCRIPT = ${TEST_DIR}/run_test.py

CORE_OBJECTS = $(patsubst $(CORE_LIB_DIR)/%.cc,$(BUILD_DIR)/%.o,$(CORE_SOURCES))

STATIC_LIB = $(LIB_DIR)/libcore.a

TARGET = $(BUILD_DIR)/main.exe

all: $(TARGET)

$(BUILD_DIR):
	@if not exist "$(BUILD_DIR)" mkdir "$(BUILD_DIR)"

$(LIB_DIR): $(BUILD_DIR)
	@if not exist "$(LIB_DIR)" mkdir "$(LIB_DIR)"

$(BUILD_DIR)/%.o: $(CORE_LIB_DIR)/%.cc | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(STATIC_LIB): $(CORE_OBJECTS) | $(LIB_DIR)
	$(AR) $(ARFLAGS) $@ $^
	@echo Static library created: $(STATIC_LIB)

$(TARGET): ./Console/main.cc $(STATIC_LIB)
	$(CXX) $(CXXFLAGS) $< -L$(LIB_DIR) -lcore -o $@
	@echo Executable created: $(TARGET)

clean:
	@if exist "$(BUILD_DIR)" rmdir /s /q "$(BUILD_DIR)"
	@echo Cleaned build directory

run: $(TEST_SCRIPT)
	${PY} $(TEST_SCRIPT)

