MAKEFLAGS += --no-print-directory
BUILD_DIR = build
BIN_DIR = bin
.PHONY: build run format debug release setup

bf: build format

build: setup
	cd $(BUILD_DIR) && cmake .. && make -j4
setup:
	@mkdir -p $(BUILD_DIR) $(BIN_DIR)
debug: setup
	cd $(BUILD_DIR) && cmake -DCMAKE_BUILD_TYPE=Debug .. -DBUILD_EXAMPLES=True && make -j4
release: setup
	cd $(BUILD_DIR) && cmake -DCMAKE_BUILD_TYPE=Release .. -DBUILD_EXAMPLES=True && make -j4

format:
	@clang-format -i src/*.c* inc/*.h* -style=file:.clang-format