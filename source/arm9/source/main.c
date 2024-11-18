#include <stdio.h>
#include <nds.h>
#include <string.h>
#ifndef _NOTDS
#include <filesystem.h>
#endif
#include "sdcollision.h"
#include "sdrender.h"
#include "sdobject.h"
#include "sddelta.h"
#include "sdsound.h"
#include "sdfile.h"
#include "sdinput.h"
#include "player.h"

#ifdef _WIN32

char* LoadShaderInclude(char* includePath) {
	char* path = (char*)malloc(strlen(includePath) + 1 + strlen("shaders/include/"));
	sprintf(path, "%s%s", "shaders/include/", includePath);
	FILE* f = fopen(path, "rb");
	free(path);
	if (f == NULL) {
		return NULL;
	}
	fseek(f, 0, SEEK_END);
	int fsize = ftell(f);
	fseek(f, 0, SEEK_SET);
	char* retValue = (char*)malloc(fsize + 1);
	retValue[fsize] = 0;
	fread(retValue, fsize, 1, f);
	fclose(f);
	return retValue;
}

void WindowsInitialization() {
	shaderIncludeCallback = LoadShaderInclude;
	InitializeGraphics();
	InitializeWindow(640, 480, 4, true, false);
	defaultShader = LoadShader("shaders/default.vert", "shaders/default.frag");
	defaultRiggedShader = LoadShader("shaders/defaultRigged.vert", "shaders/default.frag");
	defaultSpriteShader = LoadShader("shaders/defaultSprite.vert", "shaders/defaultSprite.frag");
}

#endif

FILE* testfile;

void AsyncReadTestCallback(void* data, bool success) {
	//if (data == NULL) return;
	fseek(testfile, 0, SEEK_SET);
	// i think it's being read in 0 time...
	fread_Async(data, 0x100000, 1, testfile, 0, AsyncReadTestCallback, data);
}

int main() {
	InitSound();
#ifndef _NOTDS
	
	defaultExceptionHandler();
	if (!nitroFSInit(NULL)) {
		printf("NitroFSInit failure");
	}

#endif

#ifdef _WIN32
	WindowsInitialization();
#endif

	Initialize3D(true, true);

	// set 3D to top screen
	Set3DOnTop();

	InitializeNetworking(1, 1);

	InitializeSubBG();
	
	// be sure to set up lighting
	SetLightColor(RGB15(16, 16, 16));
	SetAmbientColor(RGB15(8, 8, 8));
	SetLightDir(0, -4096, 0);

	// set up basic collision layers
	AddCollisionBetweenLayers(1, 1);
	AddCollisionBetweenLayers(1, 2);

	// register object type 0 to execute no code so we can use it for misc things
	AddObjectType(NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, false);
	// register object type 1 to be our player
	int playerId = AddObjectType(PlayerUpdate, PlayerStart, PlayerCollide, PlayerLateUpdate, PlayerDestroy, NULL, NULL, NULL, false);

	Vec3 zero = { 0, 0, 0 };
	Object* world = CreateObject(0, &zero, false);
	world->mesh = LoadModel("nitro:/testmap/testmap.sdm");
	world->meshCol = MeshColliderFromMesh(world->mesh);
	world->scale.x = world->mesh->defaultScale;
	world->scale.y = world->mesh->defaultScale;
	world->scale.z = world->mesh->defaultScale;
	world->position = world->mesh->defaultOffset;
	world->solid = true;

	Vec3 up = { 0, 4096, 0 };

	Object* player = CreateObject(playerId, &up, false);
	
	InitDeltaTime();
	deltaTimeEngine = true;

	UpdateDeltaTime();
	
	PlayMusic("nitro:/music/battle.wav", 0);

	while (1) {
		UpdateDeltaTime();
		printf("%f\n", f32tofloat(deltaTime));
		UpdateInput();
		
		ProcessObjects();

		//FinalizeSprites();

		//printf("%i\n%i\n", GetTouchScreenX(TOUCH_ORIGIN_LEFT), GetTouchScreenY(TOUCH_ORIGIN_TOP));
#ifdef _WIN32
		if (GetWindowClosing()) {
			break;
		}
#endif
	}
#ifdef _WIN32
	DestroyGameWindow();
	DestroyGraphics();
#endif
	UninitializeAudio();
	return 0;
}