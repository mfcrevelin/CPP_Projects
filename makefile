CXX = g++
CXXFLAGS = -std=c++17 -Wall -Wextra -Werror
INCLUDES = -I /usr/include/nlohmann/
TARGET = a_star.bin

all: $(TARGET)

$(TARGET): main.cpp
	$(CXX) $(CXXFLAGS) -o $@ $< $(INCLUDES)

clean:
	rm -f $(TARGET)
