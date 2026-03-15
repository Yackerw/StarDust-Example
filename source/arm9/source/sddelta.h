#pragma once
#include "sdmath.h"

class SDTime {
private:
    SDTime();
public:
    static Fixed deltaTime;
    static bool deltaTimeEngine;
    static void InitDeltaTime();
    static void UpdateDeltaTime();
    static void StartBenchmark();
    static int StopBenchmark();
};