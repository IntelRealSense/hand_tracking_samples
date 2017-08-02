DEBUG := 0
GLFW_EXISTS = $(shell sh -c 'ldconfig -p|grep libglfw.so.3|wc -l')
ifeq ($(GLFW_EXISTS),0)
BUILD_GLFW := 1
else
BUILD_GLFW := 0
endif
#BUILD_GLFW := 1 #override
LIBREALSENSE_EXISTS = $(shell sh -c 'ldconfig -p|grep librealsense|wc -l')
ifeq ($(LIBREALSENSE_EXISTS),0)
BUILD_LIBREALSENSE := 1
else
BUILD_LIBREALSENSE := 0
endif
#BUILD_LIBREALSENSE := 1 #override

# Detect OS and CPU
uname_S := $(shell sh -c 'uname -s 2>/dev/null || echo not')
machine := $(shell sh -c "$(CC) -dumpmachine || echo unknown")

CXX = clang++
CXXFLAGS := -std=c++14 -Wno-narrowing -Iinclude
CXXFLAGS += -fdelayed-template-parsing
ifeq ($(DEBUG),0)
CXXFLAGS += -Ofast -march=native
else
CXXFLAGS += -O0 -g
endif

ifeq ($(uname_S),Darwin)
CXXFLAGS += -I/usr/local/include
GLFW3_FLAGS := -lglfw -framework OpenGL
else
GLFW3_FLAGS := `pkg-config --cflags --libs glfw3 gl`
endif
LDFLAGS := -lpthread
INCLUDES := $(wildcard include/*.hpp)
INCLUDES += $(wildcard include/*.h)
OBJECTS  = $(notdir $(basename $(wildcard */*.cpp)))
BINARIES := $(addprefix bin/, $(OBJECTS))
OBJECTS := $(addprefix obj/, $(addsuffix .o, $(OBJECTS)))

ifeq ($(BUILD_LIBREALSENSE),0)
REALSENSE_FLAGS := -lrealsense
else
REALSENSE_FLAGS := third_party/librealsense/build/librealsense.a
DEPS += third_party/librealsense/build/librealsense.a 
LDFLAGS += -lusb-1.0 
endif

ifeq ($(BUILD_GLFW),1)
DEPS += /usr/local/lib/libglfw.so 
endif 

all: $(BINARIES) 

clean:
	rm -rf obj
	rm -rf bin

obj bin:
	mkdir -p bin
	mkdir -p obj

obj/%.o: */%.cpp $(INCLUDES) | obj
	$(CXX) $< $(CXXFLAGS) -c -o $@

bin/%: obj/%.o $(DEPS) | bin
	$(CXX) $< $(REALSENSE_FLAGS) $(GLFW3_FLAGS) $(LDFLAGS) -o $@

/usr/local/lib/libglfw.so:
	cd third_party/librealsense/scripts/; \
	./install_glfw3.sh

third_party/librealsense/build/librealsense.a:
	cd third_party/librealsense; mkdir -p build; cd build; \
	cmake .. -DBUILD_UNIT_TESTS:bool=OFF -DBUILD_SHARED_LIBS:bool=OFF; \
	make -j4 -l4
