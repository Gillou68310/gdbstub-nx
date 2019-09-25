GDB stub library for the nintendo switch

Usage
=====

* Call `GDBStub_Init` to initialize USB serial connection
* Call `GDBStub_Breakpoint` to give control to GDB
* Call `GDBStub_Shutdown` to close USB serial connection
* Link your application with `-Wl,--whole-archive -lgdbstub -Wl,--no-whole-archive -lfmt -lstdc++ -lnx`
* Run your application and connect with `aarch64-none-elf-gdb`

<br>

    file "your-homebrew.elf"
    set remotetimeout 999
    set serial baud 115200
    target "remote your-COM-port"

Build
=====

    mkdir build && cd "$_"
    source ${DEVKITPRO}/switchvars.sh
    cmake -G"Unix Makefiles" \
    -DCMAKE_TOOLCHAIN_FILE=${DEVKITPRO}/switch.cmake \
    -DPKG_CONFIG_EXECUTABLE=${DEVKITPRO}/portlibs/switch/bin/aarch64-none-elf-pkg-config \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=${DEVKITPRO}/portlibs/switch \
    ..

Limitation
=====

* Won't work with application using USB API

Todo
=====

* Implement TCP/IP connection
* Port to C to get rid of libstdc++ and libfmt dependencies
* libgdbstub needs the whole-archive linker option is order to replace the weak `__libnx_exception_entry` symbol from libnx

Docs
=====

https://www.embecosm.com/appnotes/ean4/embecosm-howto-rsp-server-ean4-issue-2.html#sec_exchange_target_remote

Credits
=====

`gdbstub.cpp` has been ported from:
https://github.com/yuzu-emu/yuzu
