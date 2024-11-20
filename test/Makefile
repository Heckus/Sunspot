# Define the compiler
CXX = g++

# Define the target executables
TARGETS = vision detection

# Define the source files
SRC_DETECTION = /home/hecke/GitRepos/projects/Sunspot/test/vision/detectionexample.cpp
SRC_SAVING = /home/hecke/GitRepos/projects/Sunspot/test/vision/savingexample.cpp

# Define the pkg-config command for OpenCV
PKG_CONFIG = $(shell pkg-config --cflags --libs opencv4)

# Define the build rule
all: $(TARGETS)

detection: $(SRC_DETECTION)
	$(CXX) -o detection $(SRC_DETECTION) $(PKG_CONFIG)

saving: $(SRC_SAVING)
	$(CXX) -o saving $(SRC_SAVING) $(PKG_CONFIG)

# Define the clean rule
clean:
	rm -f $(TARGETS)