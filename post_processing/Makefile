CC = g++

.SUFFIXES: .cpp

CC = g++
CFLAGS = `pkg-config --cflags opencv`
LDLIBS = `pkg-config --libs opencv`
LDFLAGS = -L. -L../../lib -L/usr/lib/ -L/usr/X11R6/lib/

OBJS   = im_seg.o
TARGET = segm 
	
.cpp.o:
	$(CC) ${CFLAGS} -c $<

$(TARGET): $(OBJS)
	$(CC) $(CFLAGS) $(OBJS) -o $(TARGET) $(LDFLAGS) $(LDLIBS) -std=c++11

run: $(TARGET)
	$(TARGET)

clean: 
	rm -f *~ $(OBJS) $(TARGET)
	

