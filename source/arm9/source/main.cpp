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

int main() {
	InitSound();
#ifndef _NOTDS
	
	defaultExceptionHandler();
	if (!nitroFSInit(NULL)) {
		printf("NitroFSInit failure");
		while (true);
	}

#endif

#ifdef _WIN32
	WindowsInitialization();
#endif

	
	Initialize3D(false, false);

	// set 3D to top screen
	Set3DOnTop();

	InitializeSubBG();

	consoleDemoInit();
	/*volatile int* POS_TEST = (volatile int*)0x040005C4;
	volatile int* GXSTAT = (volatile int*)0x04000600;
	StartBenchmark();
	for (int i = 0; i < 4096; ++i) {
		*POS_TEST = 1;
		*POS_TEST = 2;
		while (((*GXSTAT) & 1));
	}
	int cycleCount = StopBenchmark();
	printf("%i\n", cycleCount);
	while (true);*/
	
	// be sure to set up lighting
	SetLightColor(0, 16, 16, 16);
	SetAmbientColor(8, 8, 8);
	SetLightDir(0, 0, -4096, 0);
	EnableLight(0);
	EnableLight(1);
	SetLightColor(1, 0, 16, 16);
	SetLightDir(1, 0, 4096, 0);

	// set up basic collision layers
	AddCollisionBetweenLayers(1, 1);
	AddCollisionBetweenLayers(1, 2);

	Vec3 zero = { 0, 0, 0 };
	Object* world = new Object();
	world->mesh = LoadModel("nitro:/testmap/testmap.sdm");
	world->meshCol = MeshColliderFromMesh(world->mesh);
	world->scale.x = world->mesh->defaultScale;
	world->scale.y = world->mesh->defaultScale;
	world->scale.z = world->mesh->defaultScale;
	world->position = world->mesh->defaultOffset;
	world->solid = true;
	for (int i = 0; i < world->mesh->materialCount; ++i) {
		world->mesh->defaultMats[i].stencilPack = 0x1;
	}

	Player* player = new Player();

	Vec3 up = { 0, 4096, 0 };

	//Object* player = CreateObject(playerId, &up, false);
	
	SDTime::InitDeltaTime();
	SDTime::deltaTimeEngine = true;

	SDTime::UpdateDeltaTime();
	
	PlayMusic("nitro:/music/battle.wav", 0);

	while (1) {
		SDTime::UpdateDeltaTime();
		//printf("%f\n", f32tofloat(deltaTime));
		UpdateInput();
		//printf("%i %i %i\n", player->position.x, player->position.y, player->position.z);
		
		Object::ProcessObjects();

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