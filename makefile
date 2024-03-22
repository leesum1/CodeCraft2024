
SDK_PATH = /home/leesum/Documents/huawei_soft2024/LinuxReleasev1.2.part1/LinuxReleasev1.2/LinuxReleasev1.2
SDK_PATH2 = /home/leesum/Documents/huawei_soft2024/LinuxRelease.part01/LinuxRelease
JUGDE_PROG = $(SDK_PATH)/PreliminaryJudge
MAP_PATH = ./maps
REPLAYER_PROG = $(SDK_PATH)/replayer/CodeCraft_2024_Replay.x86_64

# 87911441231234 有一个港口距离特别短

RAND_SEED = 87911921366
MAP_SEL := map312.txt
TARGET = $(PWD)/build/main
# 获取时间戳
TIMESTAMP = $(shell date +%s)
ZIP_NAME = robot-$(TIMESTAMP).zip

all: 
	if [ ! -d "build" ]; then mkdir build && cd build && cmake .. && cd ..; fi
	make -C build


run:all
	$(JUGDE_PROG) -s $(RAND_SEED)   -m $(MAP_PATH)/$(MAP_SEL)  $(TARGET)

test:all
	$(TARGET)

zip:
	if [ -f "log.txt" ]; then rm log.txt; fi
	zip zip/$(ZIP_NAME)  *

replay:all
	$(REPLAYER_PROG)

gdb:all
	gdb $(TARGET)

clean:
	rm -rf build cmake-build-debug

.PHONY: zip clean