cmake_minimum_required(VERSION 2.6)
project(gdbstub)

# All includes are relative to the "include" directory
include_directories ("${PROJECT_SOURCE_DIR}/src")

enable_language(ASM)

add_library (gdbstub
 src/gdbstub.h
 src/logger/Logger.h
 src/nxlink/nxlink.h
 src/usb/usb.h
 src/usb/USBInterface.h
 src/usb/USBSerial.h
 src/usb/USBSerialComInterface.h
 src/usb/USBSerialDataInterface.h
 src/exception.s
 src/gdbstub.cpp
 src/logger/Logger.cpp
 src/nxlink/nxlink.cpp
 src/usb/usb.c
 src/usb/USBSerial.cpp
 src/usb/USBSerialComInterface.cpp
 src/usb/USBSerialDataInterface.cpp
)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)
configure_file ("cmake/gdbstub.pc.in" "${PROJECT_BINARY_DIR}/gdbstub.pc" @ONLY)
install(TARGETS gdbstub DESTINATION "${CMAKE_INSTALL_LIBDIR}")
install(FILES src/gdbstub.h DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}")
install(FILES "${PROJECT_BINARY_DIR}/gdbstub.pc" DESTINATION "${CMAKE_INSTALL_LIBDIR}/pkgconfig")
