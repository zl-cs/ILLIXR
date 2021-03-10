include common/common.mk
nproc=$(shell python3 -c 'import multiprocessing; print( max(multiprocessing.cpu_count() - 1, 1))')

CXX := clang++-10
CC := clang-10

.PHONY: plugin.opt.so
plugin.opt.so: build/Makefile
	make -C build "-j$(nproc)" && \
	rm -f $@ && \
	ln -s build/libplugin.so $@ && \
	true


build/Makefile:
	mkdir build && \
	cd build && \
	cmake -DCMAKE_CXX_COMPILER=$(CXX) -DCMAKE_C_COMPILER=$(CC) .. && \
	true




.PHONY: clean
clean:
	touch build && rm -rf build *.so
