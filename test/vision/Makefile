# Define the compiler
CXX = g++

# Define the target executable
TARGET = vision

# Define the source file
SRC = open_cv_test.cpp

# Define the pkg-config command for OpenCV
PKG_CONFIG = `pkg-config --cflags --libs opencv4`

# Define the build rule
all: $(TARGET)

$(TARGET): $(SRC)
    $(CXX) -o $(TARGET) $(SRC) $(PKG_CONFIG)

# Define the clean rule
clean:
    rm -f $(TARGET)