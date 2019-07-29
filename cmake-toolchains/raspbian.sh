export TARGET_ARCH=arm

export CROSS_COMPILER_C="arm-linux-gnueabihf-gcc-6"
export CROSS_COMPILER_CXX="arm-linux-gnueabihf-g++-6"

export SYSROOT="/root/sysroot"
export PYTHON_SOABI="cpython-36m-arm-linux-gnueabihf"

export TARGET_C_FLAGS="-mcpu=cortex-a7 -mtune=cortex-a7 -mfpu=neon-vfpv4 -mfloat-abi=hard -w -O2 -Wl,-rpath-link=/root/ws/install/lib"
export TARGET_CXX_FLAGS="-mcpu=cortex-a7 -mtune=cortex-a7 -mfpu=neon-vfpv4 -mfloat-abi=hard -w -O2 -Wl,-rpath-link=/root/ws/install/lib"