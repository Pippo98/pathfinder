MAKEFLAGS += --no-print-directory
BUILD_DIR = build
BIN_DIR = bin
.PHONY: build run format

bf: build format

build:
	@mkdir -p $(BUILD_DIR) && cd $(BUILD_DIR) && cmake .. && make -j4

run: build
	@echo "\n\n--> Running RRTStarTest1"
	@./$(BIN_DIR)/RRTStarTest1

format:
	@clang-format -i src/*.c* inc/*.h* -style=file:.clang-format