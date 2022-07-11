UTEST=OFF
BUILD_EXAMPLES=OFF
GPU=OFF
CMAKE_ARGS:=$(CMAKE_ARGS)

default:
	@mkdir -p build
	@cd build && cmake .. -DBUILDING_TEST=$(UTEST) -DBUILD_EXAMPLES=$(BUILD_EXAMPLES) -DUSE_GPU=$(GPU) -DCMAKE_BUILD_TYPE=Release $(CMAKE_ARGS) && make

debug:
	@mkdir -p build
	@cd build && cmake .. -DBUILD_EXAMPLES=$(BUILD_EXAMPLES) -DCMAKE_BUILD_TYPE=Debug $(CMAKE_ARGS) && make

apps:
	@make default BUILD_EXAMPLES=ON

gpu_apps:
	@make apps GPU=ON

debug_apps:
	@make debug BUILD_EXAMPLES=ON

unittest:
	@make default UTEST=ON
	@cd build && make test

clean:
	@rm -rf build*
