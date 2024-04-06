
SDK_PATH = /home/leesum/Documents/huawei_soft2024/linux
JUGDE_PROG = $(SDK_PATH)/SemiFinalJudge
REPLAYER_PROG = $(SDK_PATH)/replayer/CodeCraft_2024_Replayer_v2.x86_64

MAP_PATH = ./maps41


RAND_SEED = 1423
MAP_SEL := map3.txt
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