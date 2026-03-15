// Copyright (c) 2026 Mario T
// 
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#pragma once

#include "../Common.h"
#include "../hid/Packets.h"

Option<SRGBColor> ParseHexColor(String p_colorStr);
Option<Set<WString>> ParseSerialArgs(int p_argc, char *p_pArgv[]);
bool ParseVerbose(int p_argc, char *p_pArgv[]);
bool ParseHelp(int p_argc, char *p_pArgv[]);
void PrintHelp(const char *p_pProgramName);
