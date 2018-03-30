CC=gcc
CXX=g++
CFLAGS=`pkg-config --cflags opencv`
CXXFLAGS=-std=c++14 $(CFLAGS)
EXTRAFLAGS=-std=gnu++14
LIBS=`pkg-config --libs opencv`
INCLUDES=

% : %.cpp
	$(CXX) $(CXXFLAGS) -o $@.o $< $(LIBS) 

% : %.c
	$(CC) $(CFLAGS) -o $@.o $< $(LIBS) 