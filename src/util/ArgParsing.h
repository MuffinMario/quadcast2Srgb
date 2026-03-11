#pragma once

#include "../Common.h"
#include "../hid/Packets.h"

Option<SRGBColor> ParseHexColor(String p_colorStr);
Option<Set<WString>> ParseSerialArgs(int p_argc, char *p_pArgv[]);
bool ParseVerbose(int p_argc, char *p_pArgv[]);
