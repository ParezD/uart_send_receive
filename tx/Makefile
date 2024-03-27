CXX = g++

CXXFLAGS = -Wall

SRCS := $(wildcard *.cpp)

OBJS := $(SRCS:.cpp=.o)

EXEC = out

$(EXEC): $(OBJS)
	$(CXX) $(CXXFLAGS) $(OBJS) -o $(EXEC)

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	rm -f $(OBJS) $(EXEC)
