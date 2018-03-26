all: build run
build: main.cc
	cd src && g++ main.cc -o main.o
	mkdir -p build && mv src/main.o build/
run: build
	cd build && ./main.o
clean:
	rm -rf build/