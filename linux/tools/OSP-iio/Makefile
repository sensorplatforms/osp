CC=gcc
ARMCC=/opt/ndk-tool/bin/arm-linux-androideabi-gcc
CFLAGS=-Wall -I../../../include/linux

all: osp-iio-x86 osp-iio

osp-iio-arm.o: osp-iio.c
	$(ARMCC) $(CFLAGS) -c $^ -o $@

osp-iio-x86.o: osp-iio.c
	$(CC) $(CFLAGS) -c $^ -o $@

osp-iio-x86: osp-iio-x86.o
	$(CC) $(CFLAGS) $^ -o $@

osp-iio: osp-iio-arm.o
	$(ARMCC) -static $(CFLAGS) $^ -o $@


clean:
	rm -f osp-iio osp-iio-x86 *.o
