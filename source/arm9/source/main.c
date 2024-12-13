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

void QuatTimesVec3Cached(Quaternion* quat, Vec3* vec, Vec3* out, f32 s) {
	// huh
	Vec3* u = (Vec3*)quat;
	f32 dot = DotProduct(u, vec) * 2;
	Vec3 temp;
	temp.x = mulf32(u->x, dot);
	temp.y = mulf32(u->y, dot);
	temp.z = mulf32(u->z, dot);

	temp.x += mulf32(s, vec->x);
	temp.y += mulf32(s, vec->y);
	temp.z += mulf32(s, vec->z);
	s = quat->w * 2;
	Vec3 cross;
	CrossProduct(u, vec, &cross);
	out->x = temp.x + mulf32(cross.x, s);
	out->y = temp.y + mulf32(cross.y, s);
	out->z = temp.z + mulf32(cross.z, s);
}

void CrossProductNormal(Vec3* left, Vec3* right, Vec3* out) {
	out->x = ((left->y * right->z) - (left->z * right->y)) >> 12;
	out->y = -((left->x * right->z) - (left->z * right->x)) >> 12;
	out->z = ((left->x * right->y) - (left->y * right->x)) >> 12;
}

void QuatTimesNormalCached(Quaternion* quat, Vec3* vec, Vec3* out, f32 s) {
	// huh
	Vec3* u = (Vec3*)quat;
	f32 dot = DotProductNormal(u, vec) * 2;
	Vec3 temp;
	temp.x = u->x * dot;
	temp.y = u->y * dot;
	temp.z = u->z * dot;

	temp.x += s * vec->x;
	temp.y += s * vec->y;
	temp.z += s * vec->z;
	s = quat->w * 2;
	Vec3 cross;
	CrossProductNormal(u, vec, &cross);
	out->x = (temp.x + (cross.x * s)) >> 12;
	out->y = (temp.y + (cross.y * s)) >> 12;
	out->z = (temp.z + (cross.z * s)) >> 12;
}

Vec3 BoxSupportTest(void* shape, Vec3* normal) {
	CollisionBox* box = shape;
	Vec3 tmpNormal;
	Quaternion tmpQuat;
	QuaternionInverse(box->rotation, &tmpQuat);
	QuatTimesNormalCached(&tmpQuat, normal, &tmpNormal, box->cachedMagnitude);
	Vec3 tmpRetValue;
	if (tmpNormal.x >= 0) {
		tmpRetValue.x = box->extents.x;
	}
	else {
		tmpRetValue.x = -box->extents.x;
	}
	if (tmpNormal.y >= 0) {
		tmpRetValue.y = box->extents.y;
	}
	else {
		tmpRetValue.y = -box->extents.y;
	}
	if (tmpNormal.z >= 0) {
		tmpRetValue.z = box->extents.z;
	}
	else {
		tmpRetValue.z = -box->extents.z;
	}

	Vec3 retValue;
	QuatTimesVec3Cached(box->rotation, &tmpRetValue, &retValue, box->cachedMagnitude);
	Vec3Addition(&retValue, box->position, &retValue);
	return retValue;
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

	Initialize3D(false, true);

	// set 3D to top screen
	Set3DOnTop();

	InitializeNetworking(1, 1);

	InitializeSubBG();

	consoleDemoInit();
	
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
	for (int i = 0; i < world->mesh->materialCount; ++i) {
		world->mesh->defaultMats[i].stencilPack = 0x1;
	}

	Vec3 up = { 0, 4096, 0 };

	Object* player = CreateObject(playerId, &up, false);

	Vec3 cubeSpawn = { 4096, 0, 4096*4 };

	Object* cube = CreateObject(0, &cubeSpawn, false);
	cube->mesh = LoadModel("nitro:/testcube0.sdm");
	cube->mesh->defaultMats[0].stencilPack = 0x1;
	cube->mesh->defaultMats[0].materialFlags0 = FRONT_CULLING;
	cube->mesh->defaultMats[0].alpha = 0x0F;
	cube->boxCol = (CollisionBox*)malloc(sizeof(CollisionBox));
	cube->boxCol->position = &cube->position;
	cube->boxCol->rotation = &cube->rotation;
	cube->boxCol->extents.x = 4096;
	cube->boxCol->extents.y = 4096;
	cube->boxCol->extents.z = 4096;
	EulerToQuat(0, 6000, 0, &cube->rotation);
	CollisionBox* tmpBox = cube->boxCol;
	tmpBox->cachedMagnitude = ((cube->rotation.w * cube->rotation.w) >> 12) - DotProductNormal((Vec3*)&cube->rotation, (Vec3*)&cube->rotation);
	//cubeSpawn.x = 4096 * 6;
	cubeSpawn.x += 40;
	cube = CreateObject(0, &cubeSpawn, false);
	cube->mesh = LoadModel("nitro:/testcube1.sdm");
	cube->mesh->defaultMats[0].stencilPack = 0x2;
	cube->mesh->defaultMats[0].alpha = 0x0F;
	cube->boxCol = (CollisionBox*)malloc(sizeof(CollisionBox));
	cube->boxCol->position = &cube->position;
	cube->boxCol->rotation = &cube->rotation;
	cube->boxCol->extents.x = 4096;
	cube->boxCol->extents.y = 4096;
	cube->boxCol->extents.z = 4096;
	
	InitDeltaTime();
	deltaTimeEngine = true;

	UpdateDeltaTime();
	
	PlayMusic("nitro:/music/battle.wav", 0);

	while (1) {
		UpdateDeltaTime();
		//printf("%f\n", f32tofloat(deltaTime));
		UpdateInput();
		
		ProcessObjects();

		FinalizeSprites();

		Vec3 tmpVec;

		cube->position = player->position;

		cube->rotation = player->rotation;

		Quaternion* quat = &cube->rotation;

		cube->boxCol->cachedMagnitude = ((quat->w * quat->w) >> 12) - DotProductNormal((Vec3*)quat, (Vec3*)quat);

		Vec3Subtraction(tmpBox->position, cube->boxCol->position, &tmpVec);

		Normalize(&tmpVec, &tmpVec);

		Simplex tmpSimplex = { 0 };

		printf("%i\n", BasicGJK(cube->boxCol, tmpBox, &tmpVec, &tmpSimplex, BoxSupportTest, BoxSupportTest));

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