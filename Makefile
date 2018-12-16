obj-m := clunet.o

all:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules

clean:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean

install: all
	cp -f clunet.ko /lib/modules/$(shell uname -r)/
