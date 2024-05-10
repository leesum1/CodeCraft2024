
SDK_PATH = /home/leesum/Documents/huawei_soft2024/LinuxRelease
JUGDE_PROG = $(SDK_PATH)/SemiFinalJudge
REPLAYER_PROG = $(SDK_PATH)/replayer/CodeCraft_2024_Replayer_linux_semifinal/CodeCraft_2024_Replayer_v2.x86_64

MAP_PATH = ./maps


RAND_SEED = 2249455883
MAP_SEL := map1.txt
TARGET = $(PWD)/build/main
# 获取时间戳
TIMESTAMP = $(shell date +%s)
ZIP_NAME = robot-$(TIMESTAMP).zip

all: 
	if [ ! -d "build" ]; then mkdir build && cd build && cmake .. && cd ..; fi
	make -C build


run:all
	sed -i 's/^\/\/#define LOG_ENABLE/#define LOG_ENABLE/' config.h
	$(JUGDE_PROG) -s $(RAND_SEED)   -m $(MAP_PATH)/$(MAP_SEL)  $(TARGET)

test:all
	$(TARGET)

zip:
	sed -i 's/^#define LOG_ENABLE/\/\/#define LOG_ENABLE/' config.h
	if [ -f "log.txt" ]; then rm log.txt; fi
	zip zip/$(ZIP_NAME)  *

replay:all
	$(REPLAYER_PROG)

gdb:all
	gdb $(TARGET)

clean:
	rm -rf build cmake-build-debug

.PHONY: zip clean