
SDK_PATH = /home/leesum/Documents/huawei_soft2024/LinuxRelease.part01/LinuxRelease
JUGDE_PROG = $(SDK_PATH)/PreliminaryJudge
MAP_PATH = ./maps
REPLAYER_PROG = $(SDK_PATH)/replayer/CodeCraft_2024_Replay.x86_64



RAND_SEED = 12312
MAP_SEL = map39.txt
TARGET = $(PWD)/build/main


all: 
	if [ ! -d "build" ]; then mkdir build && cd build && cmake .. && cd ..; fi
	make -C build


run:all
	$(JUGDE_PROG) -s $(RAND_SEED)   -m $(MAP_PATH)/$(MAP_SEL)  $(TARGET)

test:all
	$(TARGET)

replay:all
	$(REPLAYER_PROG)

gdb:all
	gdb $(TARGET)

clean:
	rm -rf build