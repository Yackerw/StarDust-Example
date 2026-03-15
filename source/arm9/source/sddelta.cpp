#include <nds.h>
#include "sddelta.h"
#include "sdmath.h"
#include <stdio.h>

Fixed SDTime::deltaTime;
bool SDTime::deltaTimeEngine;

void SDTime::InitDeltaTime() {
	timerStart(1, ClockDivider_1024, 0, NULL);
	timerElapsed(1);
}

void SDTime::UpdateDeltaTime() {
	deltaTime = timerElapsed(1);
	deltaTime = deltaTime.value / 8;
	deltaTime = Min(deltaTime.value, 409);
}

// TODO: needs to be updated to get calico's timer!
void SDTime::StartBenchmark() {
	timerStart(1, ClockDivider_64, 0, NULL);
	//tickGetCount(); // TODO: USE THIS
}

int SDTime::StopBenchmark() {
	int retValue = timerElapsed(1);
	timerStop(1);
	return retValue * 128;
}