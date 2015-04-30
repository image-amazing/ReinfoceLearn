# Example makefile for Player client programs

PLAYERDIR = /usr

CXX = g++
INCLUDES = -I$(PLAYERDIR)/include/player-3.0/ -I$(PLAYERDIR)/include/boost/
CFLAGS = -Wall -g $(INCLUDES)

Zhang-Project3:   
	$(CXX) $(CFLAGS) -o Zhang-Project3 `pkg-config --cflags playerc++`Zhang-Project3.cc `pkg-config --libs playerc++`

clean:
	rm -f a.out core Zhang-Project3 *.o
