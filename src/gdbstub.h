// Copyright 2013 Dolphin Emulator Project
// Licensed under GPLv2+
// Refer to the license.txt file included.

// Originally written by Sven Peter <sven@fail0verflow.com> for anergistic.

#ifndef __GDBSTUB_H
#define __GDBSTUB_H

#ifdef __cplusplus
extern "C" {
#endif

    void GDBStub_Init();
    void GDBStub_Shutdown();
    void GDBStub_Breakpoint();

#ifdef __cplusplus
} //end extern "C"
#endif

#endif /* __GDBSTUB_H */