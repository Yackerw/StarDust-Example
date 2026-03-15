#include <nds.h>
#include <stdio.h>
#include "sdrender.h"
#include "sdmath.h"
#include "sdfile.h"
#include <stdlib.h>
#include <stdint.h>
#include <calico.h>

typedef struct {
	Vec3 position;
	Fixed relativeZ;
	Vec3 scale;
	Quaternion rotation;
	SDMaterial* materials;
	Animator* animator;
	Model* model;
	int renderPriority;
	char hasShadow;
} ModelDrawCall;

typedef struct {
	Vec3 position;
	Vec3 scale;
	Quaternion rotation;
	SDMaterial* materials;
	Animator* animator;
	Model* model;
	int renderPriority;
} QueueRenderModel;

typedef struct {
	Sprite* sprite;
	short x;
	short y;
	char spriteAlignX;
	char spriteAlignY;
	bool flipX;
	bool flipY;
	bool scaled;
	short xScale;
	short yScale;
} SpriteDrawCall;

SpriteDrawCall mainSpriteCalls[128];
SpriteDrawCall subSpriteCalls[128];
int mainSpriteCallCount;
int subSpriteCallCount;
int spriteMatrixId;

ModelDrawCall* modelDrawCalls;
int modelDrawCallCount;
int modelDrawCallAllocated;

QueueRenderModel* modelRenderQueue;
int modelRenderQueueCount;
int modelRenderQueueAllocated;

Vec3 cameraRecentering;

bool touch3D = false;

bool multipassRendering = false;

Vec3 cameraPosition;
Quaternion cameraRotation = {0, 0, 0, 4096};
Fixed cameraFOV = 8192;
Fixed cameraNear = 900;
Fixed cameraFar = 1409600;
Mat4x4 cameraMatrix;
ViewFrustum frustum;

Vec3 lightNormal[4];
Vec3 nativeLightNormal[4];
int lightColor[4];
bool lightEnabled[4];
bool lightsDirty[4]; // used basically only for DS light overrides
Vec3 ambientColor;

int bgID;

Texture startTexture;

typedef struct {
	FILE* f;
	Model* model;
	void (*callBack)(void* data, Model* model);
	void* callBackData;
	char* texDir;
} ModelCallbackData;

typedef struct {
	FILE* f;
	Texture* texture;
	void (*callBack)(void* data, Texture* model);
	void* callBackData;
} TextureCallbackData;

typedef struct {
	SetupModelFromMemoryCallback callBack;
	void* callBackData;
	Model* modelToTexture;
	int matId;
}ModelTexturesCallbackData;

typedef struct TextureQueue TextureQueue;

struct TextureQueue {
	char* textureToLoad;
	bool upload;
	LoadTexturesCallback callBack;
	void* callBackData;
	FILE* f;
	Texture* tex;
	TextureQueue* prev;
	TextureQueue* next;
};

typedef struct {
	FILE* f;
	Sprite* sprite;
	bool sub;
	bool upload;
	void (*callBack)(void* data, Sprite* sprite);
	void* callBackData;
} SpriteCallbackData;

typedef struct {
	FILE* f;
	Animation* anim;
	void (*callBack)(void* data, Animation* anim);
	void* callBackData;
} AnimationCallbackData;

TextureQueue* firstTextureQueue;

ITCM_CODE int TransparentSortFunction(void const *aa, void const *ba) {
	ModelDrawCall* a = (ModelDrawCall*)aa;
	ModelDrawCall* b = (ModelDrawCall*)ba;
	if (a->renderPriority != b->renderPriority) {
		return a->renderPriority > b->renderPriority ? -1 : 1;
	}
#ifdef _NOTDS
	// always prioritize a 0 shadow stencil over others in the same priority group
	if ((a->subMat.stencilPack & (STENCIL_SHADOW_COMPARE_WRITE | STENCIL_VALUE)) == STENCIL_SHADOW_COMPARE_WRITE && a->subMat.stencilPack != b->subMat.stencilPack) {
		return -1;
	}
	return a->position.z - b->position.z > 0 ? -1 : 1;
#else
	// initialize if the draw call is a shadow
	if (a->hasShadow == 0) {
		a->hasShadow = -1;
		for (int i = 0; i < a->model->materialCount; ++i) {
			if ((a->materials[i].stencilPack & (STENCIL_SHADOW_COMPARE_WRITE | STENCIL_VALUE)) == STENCIL_SHADOW_COMPARE_WRITE) {
				a->hasShadow = 1;
				break;
			}
		}
	}
	if (b->hasShadow == 0) {
		b->hasShadow = -1;
		for (int i = 0; i < a->model->materialCount; ++i) {
			if ((b->materials[i].stencilPack & (STENCIL_SHADOW_COMPARE_WRITE | STENCIL_VALUE)) == STENCIL_SHADOW_COMPARE_WRITE) {
				b->hasShadow = 1;
				break;
			}
		}
	}
	// always prioritize a 0 shadow stencil over others in the same priority group
	if (a->hasShadow == 1 && b->hasShadow != 1) {
		return -1;
	}
	return a->relativeZ - b->relativeZ > 0 ? -1 : 1;
#endif
}

void LoadModelTexturesCallback(void* data, Texture* texture) {
	ModelTexturesCallbackData* mtcbd = (ModelTexturesCallbackData*)data;

	if (texture != NULL) {
		mtcbd->modelToTexture->defaultMats[mtcbd->matId].texture = texture;
		++texture->numReferences;
	}

	if (mtcbd->callBack != NULL) {
		mtcbd->callBack(mtcbd->callBackData);
	}
	free(data);
}

void SetupModelFromMemory(Model* model, char* textureDir, bool asyncTextures, SetupModelFromMemoryCallback callback, void* asyncCallbackData) {
	Model* retValue = model;
	char* input = textureDir;
	retValue->vertexGroups = (VertexHeader*)((unsigned int)retValue->vertexGroups + (unsigned int)retValue);
	retValue->defaultMats = (SDMaterial*)((uint32_t)retValue + (uint32_t)retValue->defaultMats);
	retValue->materialTextureNames = (char*)((uint32_t)retValue + (uint32_t)retValue->materialTextureNames);
	if (retValue->skeleton != NULL)
	{
		retValue->skeleton = (Bone*)((uint32_t)retValue + (uint32_t)retValue->skeleton);
		for (int i = 0; i < retValue->skeletonCount; ++i) {
			retValue->skeleton[i].inverseMatrix = retValue->skeleton[i].inverseMatrix.Transpose();
			int* temp = (int*)&retValue->skeleton[i].rotation;
			retValue->skeleton[i].rotation = Quaternion(temp[0],temp[1],temp[2],temp[3]);
		}
	}
	char* currString = retValue->materialTextureNames;
	// do this before the loop for optimization
	int inputLen = strlen(input);
	// cut off everything after the last / or backslash
	for (; inputLen > 0; --inputLen) {
		if (input[inputLen] == "/"[0]) {
			inputLen++;
			break;
		}
	}
	char tmpString[512];
	memcpy(tmpString, input, inputLen);
	for (int i = 0; i < retValue->materialCount; ++i) {
		strcpy(tmpString + inputLen, currString);
		currString += strlen(currString);
		currString += 1;
		if (!asyncTextures) {
			retValue->defaultMats[i].texture = LoadTexture(tmpString, true);
			if (retValue->defaultMats[i].texture != NULL) {
				retValue->defaultMats[i].texture->numReferences += 1;
			}
		}
		else {
			retValue->defaultMats[i].texture = NULL;

			ModelTexturesCallbackData* callbackData = (ModelTexturesCallbackData*)malloc(sizeof(ModelTexturesCallbackData));

			if (i == retValue->materialCount - 1) {
				callbackData->callBack = callback;
				callbackData->callBackData = asyncCallbackData;
			}
			else {
				callbackData->callBack = NULL;
			}
			callbackData->matId = i;
			callbackData->modelToTexture = retValue;

			LoadTextureAsync(tmpString, true, (LoadTexturesCallback)LoadModelTexturesCallback, callbackData);
		}
	}

	CacheModel(retValue);
	//retValue->NativeModel = NULL;

#ifdef _NOTDS
	UpdateModel(model);

#endif
}

Model *LoadModel(char *input) {
	char* nativeDir = DirToNative(input);
	FILE *f = fopen(nativeDir, "rb");
	free(nativeDir);
	if (f == NULL) {
		return NULL;
	}
	fseek(f, 0, SEEK_END);
	int fsize = ftell(f);
	fseek(f, 0, SEEK_SET);
	Model *retValue = (Model*)malloc(fsize);
	fread_MusicYielding(retValue, fsize, 1, f);
	fclose(f);
	SetupModelFromMemory(retValue, input, false, NULL, NULL);
	return retValue;
}

Model* FreeModelKeepCache(Model* model) {
	if (model->NativeModel == NULL) {
		// no cache...
		return model;
	}
	// duplicate materials
	SDMaterial* mats = (SDMaterial*)malloc(sizeof(SDMaterial) * model->materialCount);
	memcpy(mats, model->defaultMats, sizeof(SDMaterial) * model->materialCount);
	Model* retValue = (Model*)calloc(sizeof(Model), 1);
	retValue->defaultMats = mats;
	retValue->materialCount = model->materialCount;
	retValue->NativeModel = model->NativeModel;
	retValue->boundsMin = model->boundsMin;
	retValue->boundsMax = model->boundsMax;
	retValue->defaultOffset = model->defaultOffset;
	retValue->defaultScale = model->defaultScale;
	retValue->skeletonCount = model->skeletonCount;
	if (retValue->skeletonCount != 0) {
		retValue->skeleton = (Bone*)malloc(sizeof(Bone) * retValue->skeletonCount);
		memcpy(retValue->skeleton, model->skeleton, sizeof(Bone) * retValue->skeletonCount);
	}
	// set up vertex headers
	retValue->vertexGroupCount = model->vertexGroupCount;
	retValue->vertexGroups = (VertexHeader*)malloc((sizeof(VertexHeader) * model->vertexGroupCount) - (sizeof(Vertex) * model->vertexGroupCount));
	VertexHeader* currHeader = retValue->vertexGroups;
	VertexHeader* modelHeader = model->vertexGroups;
	for (int i = 0; i < retValue->vertexGroupCount; ++i) {
		currHeader->material = modelHeader->material;
		currHeader->bitFlags = modelHeader->bitFlags;
		currHeader->count = 0;
		currHeader = (VertexHeader*)&currHeader->vertices;
		modelHeader = (VertexHeader*)((uint32_t)(&(modelHeader->vertices)) + (uint32_t)(sizeof(Vertex) * (modelHeader->count)));
	}
	retValue->version = 0x80000000 | model->version;
	free(model);
	return retValue;
}

void LoadModelAsyncInitCallback(void* data) {
	ModelCallbackData* cbd = (ModelCallbackData*)data;
	cbd->callBack(cbd->callBackData, cbd->model);
	free(cbd);
}

void LoadModelAsyncCallback(void* data, bool success) {
	ModelCallbackData* cbd = (ModelCallbackData*)data;
	fclose(cbd->f);
	if (success) {
		SetupModelFromMemory(cbd->model, cbd->texDir, true, (SetupModelFromMemoryCallback)LoadModelAsyncInitCallback, cbd);
	}
	else {
		cbd->callBack(cbd->callBackData, NULL);
		free(cbd);
	}
}

int LoadModelAsync(char* input, void (*callBack)(void* data, Model* model), void* callBackData) {
	if (callBack == NULL) {
		// ?
		return -1;
	}
	char* nativeDir = DirToNative(input);
	FILE* f = fopen(nativeDir, "rb");
	free(nativeDir);
	if (f == NULL) {
		callBack(callBackData, NULL);
		return -1;
	}
	fseek(f, 0, SEEK_END);
	int fsize = ftell(f);
	fseek(f, 0, SEEK_SET);
	Model* retValue = (Model*)malloc(fsize);

	ModelCallbackData* cbd = (ModelCallbackData*)malloc(sizeof(ModelCallbackData));
	cbd->f = f;
	cbd->model = retValue;
	cbd->callBack = callBack;
	cbd->callBackData = callBackData;
	cbd->texDir = (char*)malloc(strlen(input) + 1);
	strcpy(cbd->texDir, input);

	return fread_Async((void*)retValue, fsize, 1, f, 0, (AsyncFileCallback)LoadModelAsyncCallback, cbd);
}

void LoadTextureFromQueue();

void UpdateTextureQueue() {
	TextureQueue* tmp = firstTextureQueue;
	firstTextureQueue = firstTextureQueue->next;
	if (firstTextureQueue != NULL)
		firstTextureQueue->prev = NULL;
	free(tmp->textureToLoad);
	free(tmp);
	if (firstTextureQueue != NULL)
		LoadTextureFromQueue();
}

void TextureAsyncCallback(void* data, bool success) {
	TextureQueue* tq = (TextureQueue*)data;
	fclose(tq->f);
	// once more...!
	Texture* tex = startTexture.next;
	while (tex != NULL) {
		if (strcmp(tq->textureToLoad, tex->name) == 0) {
			tq->callBack(tq->callBackData, tex);
			free(tq->tex);
			UpdateTextureQueue();
			return;
		}
		tex = tex->next;
	}

	// okay, we're good, initialize the texture properly
	tq->tex = LoadTextureFromRAM(tq->tex, tq->upload, tq->textureToLoad);

	tq->callBack(tq->callBackData, tq->tex);
	UpdateTextureQueue();
}

void LoadTextureFromQueue() {
	Texture* tex = startTexture.next;
	while (tex != NULL) {
		if (strcmp(firstTextureQueue->textureToLoad, tex->name) == 0) {
			firstTextureQueue->callBack(firstTextureQueue->callBackData, tex);
			UpdateTextureQueue();
			return;
		}
		tex = tex->next;
	}
	FILE* f = fopen(firstTextureQueue->textureToLoad, "rb");
	if (f == NULL) {
		firstTextureQueue->callBack(firstTextureQueue->callBackData, NULL);
		UpdateTextureQueue();
		return;
	}
	fseek(f, 0, SEEK_END);
	int fsize = ftell(f);
	fseek(f, 0, SEEK_SET);
	Texture* newTex = (Texture*)malloc(fsize);
	firstTextureQueue->f = f;
	firstTextureQueue->tex = newTex;
	fread_Async(newTex, fsize, 1, f, 0, (AsyncFileCallback)TextureAsyncCallback, firstTextureQueue);
}

void LoadTextureAsync(char* input, bool upload, LoadTexturesCallback callBack, void* callBackData) {
	if (callBack == NULL) {
		// ?
		return;
	}
	Texture* tex = startTexture.next;
	while (tex != NULL) {
		if (strcmp(input, tex->name) == 0) {
			callBack(callBackData, tex);
			return;
		}
		tex = tex->next;
	}

	// okay, so this is a little dumb, but we don't want to repeatedly load textures we already have loaded. so, we set up a new queue here.
	// if there's no textures currently loading, we go ahead. otherwise, we add to the queue.
	// we have this in a custom queue rather than in the texture itself so that we can mix and match async texture loading and synchronous texture loading
	TextureQueue* newQueue = (TextureQueue*)malloc(sizeof(TextureQueue));
	newQueue->textureToLoad = DirToNative(input);
	newQueue->callBack = callBack;
	newQueue->upload = upload;
	newQueue->callBackData = callBackData;
	newQueue->next = NULL;
	newQueue->prev = NULL;
	if (firstTextureQueue != NULL) {
		TextureQueue* tq = firstTextureQueue;
		while (tq->next != NULL) {
			tq = tq->next;
		}
		tq->next = newQueue;
		newQueue->prev = tq;
	}
	else {
		firstTextureQueue = newQueue;
		LoadTextureFromQueue();
	}

}

#ifndef _NOTDS
unsigned int FIFOLookup[] = { FIFO_COMMAND_PACK(FIFO_NORMAL, FIFO_TEX_COORD, FIFO_VERTEX16, FIFO_NORMAL), FIFO_COMMAND_PACK(FIFO_TEX_COORD, FIFO_VERTEX16, FIFO_NORMAL, FIFO_TEX_COORD), FIFO_COMMAND_PACK(FIFO_VERTEX16, FIFO_NORMAL, FIFO_TEX_COORD, FIFO_VERTEX16) };
#define FIFO_MTX_RESTORE (0x50 >> 2)
#define FIFO_VTX_DIFF (0xA0 >> 2)

#define CMD_TEXCOORD 1
#define CMD_NORMAL 2
#define CMD_POS 3
#define CMD_MTX 4
#define CMD_TRI 5
#define CMD_TRI_STRIP 6
#define CMD_QUAD 7
#define CMD_QUAD_STRIP 8

struct GFXFIFOBuilder {
	unsigned int* commands;
	int usedMemory;
	int allocatedMemory;
	bool DTCM;
	bool initializedVert;
	char currCommand;
	char prevMtx; // placing it here makes better use of memory
	int commandInd;
	unsigned int prevUV;
	unsigned int prevNormal;
	Vec3s prevVert;
};

struct GFXFIFOBuilder InitFIFO() {
	struct GFXFIFOBuilder ret;
	ret.commands = (unsigned int*)malloc(1024 * 16);//0x02FF0000; // DTCM base
	ret.allocatedMemory = 1024 * 16 / 4; // 16KB DTCM
	ret.usedMemory = 2;
	ret.DTCM = false;
	ret.currCommand = 0;
	ret.commandInd = 1;
	ret.commands[1] = 0;
	ret.prevUV = 0xFFFFFFFF; // lousy
	ret.prevNormal = 0;
	ret.prevVert.x = 0;
	ret.prevVert.y = 0;
	ret.prevVert.z = 0;
	ret.initializedVert = false;
	ret.prevMtx = -1;
	return ret;
}

void IncrementFIFO(struct GFXFIFOBuilder* fifo) {
	fifo->usedMemory += 1;
	if (fifo->usedMemory == fifo->allocatedMemory) {
		int oldMemorySize = fifo->allocatedMemory;
		fifo->allocatedMemory *= 1.5f; // generally a decent scale
		if (fifo->DTCM) {
			unsigned int* newAlloc = (unsigned int*)malloc(sizeof(unsigned int) * fifo->allocatedMemory);
			memcpy(newAlloc, fifo->commands, sizeof(unsigned int) * oldMemorySize);
			fifo->commands = newAlloc;
			fifo->DTCM = false;
		}
		else {
			fifo->commands = (unsigned int*)realloc(fifo->commands, sizeof(unsigned int)*fifo->allocatedMemory);
		}
	}
}

void PushFIFOCommand(struct GFXFIFOBuilder* fifo, unsigned char command, int commandArg1, int commandArg2, int commandArg3) {
	int packBitShift = fifo->currCommand * 8;
	switch (command) {
	case CMD_TEXCOORD: {
		unsigned int currUV = (commandArg1 & 0xFFFF) | (commandArg2 << 16);
		// ensure the first vertex always writes a UV
		if (fifo->prevUV != currUV || !fifo->initializedVert) {
			fifo->commands[fifo->commandInd] |= FIFO_TEX_COORD << packBitShift;
			fifo->commands[fifo->usedMemory] = currUV;
			fifo->prevUV = currUV;
			IncrementFIFO(fifo);
		}
		else {
			return;
		}
		// TODO: add a mode to force this to always write due to how sphere mapping works
	}
		break;
	case CMD_NORMAL:
		if (fifo->prevNormal != commandArg1) {
			fifo->commands[fifo->commandInd] |= FIFO_NORMAL << packBitShift;
			fifo->commands[fifo->usedMemory] = commandArg1;
			fifo->prevNormal = commandArg1;
			IncrementFIFO(fifo);
		}
		else {
			return;
		}
		// TODO: add a mode to force this to always write due to how sphere mapping works
		break;
	case CMD_POS: {
		int diffX = (commandArg1 - fifo->prevVert.x);
		int diffY = (commandArg2 - fifo->prevVert.y);
		int diffZ = (commandArg3 - fifo->prevVert.z);
		if (abs(diffX) < 512 && abs(diffY) < 512 && abs(diffZ) < 512 && fifo->initializedVert) {
			fifo->commands[fifo->commandInd] |= FIFO_VTX_DIFF << packBitShift;
			fifo->commands[fifo->usedMemory] = (diffX & 0x3FF) | ((diffY & 0x3FF) << 10) | ((diffZ & 0x3FF) << 20);
			IncrementFIFO(fifo);
		}
		else {
			fifo->commands[fifo->commandInd] |= FIFO_VERTEX16 << packBitShift;
			fifo->commands[fifo->usedMemory] = (commandArg1 & 0xFFFF) | (commandArg2 << 16);
			IncrementFIFO(fifo);
			fifo->commands[fifo->usedMemory] = commandArg3;
			IncrementFIFO(fifo);
			fifo->initializedVert = true;
		}
		fifo->prevVert.x = commandArg1;
		fifo->prevVert.y = commandArg2;
		fifo->prevVert.z = commandArg3;
	}
		break;
	case CMD_MTX:
		if (fifo->prevMtx != commandArg1) {
			fifo->commands[fifo->commandInd] |= FIFO_MTX_RESTORE << packBitShift;
			fifo->commands[fifo->usedMemory] = commandArg1;
			IncrementFIFO(fifo);
			fifo->prevMtx = commandArg1;
			// invalidate normal
			fifo->prevNormal = 0;
		}
		else {
			return;
		}
		break;
	case CMD_TRI:
		fifo->commands[fifo->commandInd] |= FIFO_BEGIN << packBitShift;
		fifo->commands[fifo->usedMemory] = GL_TRIANGLE;
		IncrementFIFO(fifo);
		break;
	case CMD_TRI_STRIP:
		fifo->commands[fifo->commandInd] |= FIFO_BEGIN << packBitShift;
		fifo->commands[fifo->usedMemory] = GL_TRIANGLE_STRIP;
		IncrementFIFO(fifo);
		break;
	case CMD_QUAD:
		fifo->commands[fifo->commandInd] |= FIFO_BEGIN << packBitShift;
		fifo->commands[fifo->usedMemory] = GL_QUAD;
		IncrementFIFO(fifo);
		break;
	case CMD_QUAD_STRIP:
		fifo->commands[fifo->commandInd] |= FIFO_BEGIN << packBitShift;
		fifo->commands[fifo->usedMemory] = GL_QUAD_STRIP;
		IncrementFIFO(fifo);
		break;
	default:
		fifo->commands[fifo->commandInd] |= FIFO_NOP << packBitShift;
		break;
	}
	fifo->currCommand += 1;
	if (fifo->currCommand == 4) {
		fifo->currCommand = 0;
		fifo->commandInd = fifo->usedMemory;
		fifo->commands[fifo->commandInd] = 0;
		IncrementFIFO(fifo);
	}
}

unsigned int* FinalizeFIFO(struct GFXFIFOBuilder* fifo) {
	fifo->commands[0] = fifo->usedMemory - 1;
	if (fifo->currCommand != 0) {
		while (fifo->currCommand != 4) {
			fifo->commands[fifo->commandInd] |= FIFO_NOP << (fifo->currCommand * 8);
			++fifo->currCommand;
		}
	}
	if (!fifo->DTCM) {
		return (unsigned int*)realloc(fifo->commands, sizeof(unsigned int) * fifo->usedMemory);
	}
	else {
		unsigned int* retValue = (unsigned int*)malloc(sizeof(unsigned int) * fifo->usedMemory);
		memcpy(retValue, fifo->commands, sizeof(unsigned int) * fifo->usedMemory);
		return retValue;
	}
}


#endif

#ifndef _NOTDS
void CacheRiggedModel(Model* reference) {
	if (reference->skeletonCount > 30) {
		return;
	}
	DSNativeModel dsnm;
	dsnm.FIFOCount = reference->vertexGroupCount;
	dsnm.FIFOBatches = (unsigned int**)malloc(sizeof(unsigned int*) * reference->vertexGroupCount);
	const VertexHeader* currHeader = &reference->vertexGroups[0];
	for (int i = 0; i < reference->vertexGroupCount; ++i) {
		if (currHeader->count == 0) {
			dsnm.FIFOBatches[i] = NULL;
			uint32_t toAdd = (sizeof(Vertex) * (currHeader->count));
			currHeader = (VertexHeader*)(((uint32_t)(&(currHeader->vertices))) + toAdd);
			continue;
		}
		struct GFXFIFOBuilder fifo = InitFIFO();
		if (currHeader->bitFlags & VTX_QUAD) {
			if (currHeader->bitFlags & VTX_STRIPS) {
				PushFIFOCommand(&fifo, CMD_QUAD_STRIP, 0, 0, 0);
			}
			else {
				PushFIFOCommand(&fifo, CMD_QUAD, 0, 0, 0);
			}
		}
		else {
			if (currHeader->bitFlags & VTX_STRIPS) {
				PushFIFOCommand(&fifo, CMD_TRI_STRIP, 0, 0, 0);
			}
			else {
				PushFIFOCommand(&fifo, CMD_TRI, 0, 0, 0);
			}
		}

		const Vertex* vertices = &currHeader->vertices;

		for (int j = 0; j < currHeader->count; ++j) {
			PushFIFOCommand(&fifo, CMD_MTX, vertices[j].boneID, 0, 0);
			PushFIFOCommand(&fifo, CMD_TEXCOORD, vertices[j].u, vertices[j].v, 0);
			PushFIFOCommand(&fifo, CMD_NORMAL, vertices[j].normal, 0, 0);
			PushFIFOCommand(&fifo, CMD_POS, vertices[j].x, vertices[j].y, vertices[j].z);
		}

		dsnm.FIFOBatches[i] = FinalizeFIFO(&fifo);
		uint32_t toAdd = (sizeof(Vertex) * (currHeader->count));
		currHeader = (VertexHeader*)(((uint32_t)(&(currHeader->vertices))) + toAdd);
		DC_FlushRange(dsnm.FIFOBatches[i], sizeof(unsigned int) * (dsnm.FIFOBatches[i][0] + 1));
	}
	reference->NativeModel = malloc(sizeof(DSNativeModel));
	DSNativeModel* dsnmptr = (DSNativeModel*)reference->NativeModel;
	dsnmptr[0] = dsnm;
}
#endif

void CacheModel(Model* reference) {
	if (reference->skeletonCount > 0) {
		CacheRiggedModel(reference);
		return;
	}
	DSNativeModel dsnm;
	dsnm.FIFOCount = reference->vertexGroupCount;
	dsnm.FIFOBatches = (unsigned int**)malloc(sizeof(unsigned int*) * reference->vertexGroupCount);
	// get vertex count to try and calculate FIFO count
	const VertexHeader* currHeader = &reference->vertexGroups[0];
	for (int i = 0; i < reference->vertexGroupCount; ++i) {
		if (currHeader->count == 0) {
			dsnm.FIFOBatches[i] = NULL;
			uint32_t toAdd = (sizeof(Vertex) * (currHeader->count));
			currHeader = (VertexHeader*)(((uint32_t)(&(currHeader->vertices))) + toAdd);
			continue;
		}
		struct GFXFIFOBuilder fifo = InitFIFO();
		if (currHeader->bitFlags & VTX_QUAD) {
			if (currHeader->bitFlags & VTX_STRIPS) {
				PushFIFOCommand(&fifo, CMD_QUAD_STRIP, 0, 0, 0);
			}
			else {
				PushFIFOCommand(&fifo, CMD_QUAD, 0, 0, 0);
			}
		}
		else {
			if (currHeader->bitFlags & VTX_STRIPS) {
				PushFIFOCommand(&fifo, CMD_TRI_STRIP, 0, 0, 0);
			}
			else {
				PushFIFOCommand(&fifo, CMD_TRI, 0, 0, 0);
			}
		}

		const Vertex* vertices = &currHeader->vertices;

		for (int j = 0; j < currHeader->count; ++j) {
			PushFIFOCommand(&fifo, CMD_TEXCOORD, vertices[j].u, vertices[j].v, 0);
			PushFIFOCommand(&fifo, CMD_NORMAL, vertices[j].normal, 0, 0);
			PushFIFOCommand(&fifo, CMD_POS, vertices[j].x, vertices[j].y, vertices[j].z);
		}

		dsnm.FIFOBatches[i] = FinalizeFIFO(&fifo);
		uint32_t toAdd = (sizeof(Vertex) * (currHeader->count));
		currHeader = (VertexHeader*)(((uint32_t)(&(currHeader->vertices))) + toAdd);
		DC_FlushRange(dsnm.FIFOBatches[i], sizeof(unsigned int) * (dsnm.FIFOBatches[i][0] + 1));
	}
	reference->NativeModel = malloc(sizeof(DSNativeModel));
	DSNativeModel* dsnmptr = (DSNativeModel*)reference->NativeModel;
	dsnmptr[0] = dsnm;
}

ITCM_CODE void QueueModelRender(Model* model, Vec3* position, Vec3* scale, Quaternion* rotation, SDMaterial* mats, Animator* animator, int renderPriority) {
	if (modelRenderQueueAllocated == 0) {
		modelRenderQueueAllocated = 32;
		modelRenderQueue = (QueueRenderModel*)malloc(sizeof(QueueRenderModel) * 32);
	}
	// expand if we need more ram
	if (modelRenderQueueAllocated == modelRenderQueueCount) {
		modelRenderQueueAllocated *= 1.5f;
		modelRenderQueue = (QueueRenderModel*)realloc(modelRenderQueue, sizeof(QueueRenderModel) * modelRenderQueueAllocated);
	}
	modelRenderQueue[modelRenderQueueCount].position = position[0];
	modelRenderQueue[modelRenderQueueCount].scale = scale[0];
	modelRenderQueue[modelRenderQueueCount].rotation = rotation[0];
	modelRenderQueue[modelRenderQueueCount].materials = mats;
	modelRenderQueue[modelRenderQueueCount].animator = animator;
	modelRenderQueue[modelRenderQueueCount].renderPriority = renderPriority;
	modelRenderQueue[modelRenderQueueCount].model = model;
	++modelRenderQueueCount;
}

ITCM_CODE void PosTest(short x, short y, short z) {
	GFX_POS_TEST = VERTEX_PACK(x, y);
	GFX_POS_TEST = z;
	while ((GFX_STATUS & 1) != 0);
}

ITCM_CODE int PosTestWResult() {
	return GFX_POS_RESULT[3];
}

ITCM_CODE void AppendDrawCall(Model* model, Vec3* position, Vec3* scale, Quaternion* rotation, SDMaterial* mats, Animator* animator, int renderPriority) {
	if (modelDrawCallAllocated == 0) {
		modelDrawCallAllocated = 32;
		modelDrawCalls = (ModelDrawCall*)malloc(sizeof(ModelDrawCall) * 32);
	}
	// expand if we need more ram
	if (modelDrawCallAllocated == modelDrawCallCount) {
		modelDrawCallAllocated *= 1.5f;
		modelDrawCalls = (ModelDrawCall*)realloc(modelDrawCalls, sizeof(ModelDrawCall) * modelDrawCallAllocated);
	}
	modelDrawCalls[modelDrawCallCount].position = position[0];
	modelDrawCalls[modelDrawCallCount].scale = scale[0];
	modelDrawCalls[modelDrawCallCount].rotation = rotation[0];
	modelDrawCalls[modelDrawCallCount].materials = mats;
	modelDrawCalls[modelDrawCallCount].animator = animator;
	modelDrawCalls[modelDrawCallCount].renderPriority = renderPriority;
	modelDrawCalls[modelDrawCallCount].model = model;
	modelDrawCalls[modelDrawCallCount].hasShadow = 0;
	// we don't need to pass in anything but 0s since we just want model origin anyways. this will *usually* be the center of the model given how the exporter works, but we may want to change this
	// to the -offset defined in the model if we wanted to play it safe, albeit that'd cause issues if it fell out of range...
	PosTest(0, 0, 0);
	modelDrawCalls[modelDrawCallCount].relativeZ = PosTestWResult();
	++modelDrawCallCount;
}

ITCM_CODE bool SetupMaterial(SDMaterial* mat, bool rigged) {
	// lighting notes for me:
	// vertex colors are capped at 31 (well, 63 internally, but it's equivalent to an input of 31)
	// highlight mode is toon shading, but RGB get used again to add to the color. this means it can actually hue shift! it's clumsy to use, however, as RGB gets used initially, then
	// the toon lighting table is added using R as the index. as a result, you'd have to be VERY clever with how you use it outside of pure white lighting.
	// another of note is that the lighting calculation is (light * diffuse * dot) + (ambient * light)
	// this means that light functions more like diffuse than...well, LIGHT. so multiple lights functioning in the same manner as a modern renderer is IMPOSSIBLE,
	// and DIFFUSE acts more like a light. this means light calculation is extremely cumbersome and, to be honest, probably not worth implementing in a broader case.
	// HOWEVER! emission would be functional as ambient in a replacement, so if you set diffuse to proper, emission to (emiss + (ambient * color)) and ambient to pure 0...you get modern lights. weird!

	// alpha, stencil ID, stencil compare, don't omit polygons that intersect far plane
	uint32_t flags = POLY_ALPHA(mat->alpha) | POLY_ID(mat->stencilPack & STENCIL_VALUE) | (((mat->stencilPack & STENCIL_SHADOW_COMPARE_WRITE) != 0) ? (3 << 4) : 0) | (1 << 12);
	bool isTransparent = mat->alpha < 31;
	if ((mat->materialFlags0 & CULLING_MASK) == BACK_CULLING) {
#ifdef FLIP_X
		flags |= POLY_CULL_FRONT;
#else
		flags |= POLY_CULL_BACK;
#endif
	}
	else if ((mat->materialFlags0 & CULLING_MASK) == FRONT_CULLING) {
#ifdef FLIP_X
		flags |= POLY_CULL_BACK;
#else
		flags |= POLY_CULL_FRONT;
#endif
	}
	else {
		flags |= POLY_CULL_NONE;
	}
	if (mat->lightingFlags & LIGHT_ENABLE) {
		for (int i = 0; i < 4; ++i) {
			if (lightEnabled[i]) {
				flags |= 1 << i; // light enabled are stored in bottom 4 bits, so we can just do this
			}
		}
		// allow light overrides to occur
		// reset matrix because for SOME REASON the light is rotated by that when set up
		if (mat->lightingFlags & LIGHT_OVERRIDEMASK) {
			glMatrixMode(GL_MODELVIEW);
			if (rigged) {
				glLoadIdentity();
			}
			else {
				glPushMatrix();
				glLoadIdentity();
			}

			if (mat->lightingFlags & LIGHT_OVERRIDE0) {
				flags |= 1;
				glLight(0, mat->lightOverride0, (mat->lightNormal0 & 0x1F) * 32, ((mat->lightNormal0 >> 5) & 0x1F) * 32, ((mat->lightNormal0 >> 10) & 0x1F) * 32);
				lightsDirty[0] = true;
			}
			else {
				if (lightsDirty[0]) {
					glLight(0, lightColor[0], lightNormal[0].x, lightNormal[0].y, lightNormal[0].z);
					lightsDirty[0] = false;
				}
			}
			if (mat->lightingFlags & LIGHT_OVERRIDE1) {
				flags |= 2;
				glLight(1, mat->lightOverride1, (mat->lightNormal1 & 0x1F) * 32, ((mat->lightNormal1 >> 5) & 0x1F) * 32, ((mat->lightNormal1 >> 10) & 0x1F) * 32);
				lightsDirty[1] = true;
			}
			else {
				if (lightsDirty[1]) {
					glLight(1, lightColor[1], lightNormal[1].x, lightNormal[1].y, lightNormal[1].z);
					lightsDirty[1] = false;
				}
			}
			if (mat->lightingFlags & LIGHT_OVERRIDE2) {
				flags |= 4;
				// hell world backwards compatibility with materials
				unsigned short currLightNormal = mat->lightNormal2Pt0 | (mat->lightNormal2Pt1 << 8);
				glLight(2, mat->lightOverride2, (currLightNormal & 0x1F) * 32, ((currLightNormal >> 5) & 0x1F) * 32, ((currLightNormal >> 10) & 0x1F) * 32);
				lightsDirty[2] = true;
			}
			else {
				if (lightsDirty[2]) {
					glLight(2, lightColor[2], lightNormal[2].x, lightNormal[2].y, lightNormal[2].z);
					lightsDirty[3] = false;
				}
			}
			if (mat->lightingFlags & LIGHT_OVERRIDE3) {
				flags |= 8;
				// hell world backwards compatibility with materials
				unsigned short currLightNormal = mat->lightNormal3Pt0 | (mat->lightNormal3Pt1 << 8);
				glLight(3, mat->lightOverride3, (currLightNormal & 0x1F) * 32, ((currLightNormal >> 5) & 0x1F) * 32, ((currLightNormal >> 10) & 0x1F) * 32);
				lightsDirty[3] = true;
			}
			else {
				if (lightsDirty[3]) {
					glLight(3, lightColor[3], lightNormal[3].x, lightNormal[3].y, lightNormal[3].z);
					lightsDirty[3] = false;
				}
			}

			if (!rigged) {
				glPopMatrix(1);
			}
		}
		int diffR = mat->colorR * ambientColor.x.value;
		int diffG = mat->colorG * ambientColor.y.value;
		int diffB = mat->colorB * ambientColor.z.value;
		diffR = diffR ? ((diffR >> 5) + 1) : 0;
		diffG = diffG ? ((diffG >> 5) + 1) : 0;
		diffB = diffB ? ((diffB >> 5) + 1) : 0;
		diffR += mat->emissionR;
		diffG += mat->emissionG;
		diffB += mat->emissionB;
		if (diffR > 0x1F) diffR = 0x1F;
		if (diffG > 0x1F) diffG = 0x1F;
		if (diffB > 0x1F) diffB = 0x1F;
		//glMaterialf(GL_EMISSION, RGB15(diffR, diffG, diffB));
		//glMaterialf(GL_DIFFUSE, RGB15(mat->colorR, mat->colorG, mat->colorB));
		GFX_DIFFUSE_AMBIENT = RGB15(mat->colorR, mat->colorG, mat->colorB);
		GFX_SPECULAR_EMISSION = RGB15(diffR, diffG, diffB) << 16;
	}
	else {
		//glMaterialf(GL_EMISSION, RGB15(mat->colorR, mat->colorG, mat->colorB));
		//glMaterialf(GL_DIFFUSE, 0);
		GFX_DIFFUSE_AMBIENT = 0;
		GFX_SPECULAR_EMISSION = RGB15(mat->colorR, mat->colorG, mat->colorB) << 16;
	}
	// specular is OFFICIALLY un-supported. sorry. DS GPU sucks.
	//glMaterialf(GL_SPECULAR, RGB15(((lightColor & 0x1F) * mat->specular) >> 8, (((lightColor >> 5) & 0x1F) * mat->specular) >> 8, (((lightColor >> 10) & 0x1F) * mat->specular) >> 8));
	glPolyFmt(flags);
	Texture* currTex = mat->texture;
	//glBindTexture(0, currTex->textureId);
	//glAssignColorTable(0, currTex->paletteId);
	if (currTex != NULL) {
		unsigned int addTexFlags = 0;
		if (mat->materialFlags0 & TEXTURE_TRANSFORM) {
			addTexFlags = 1 << 30;
		}
		GFX_TEX_FORMAT = (currTex->textureWrite & 0x3FFFFFFF) | addTexFlags; // this is a minor optimization, but glBindTexture and glAssignColorTable accounted for about half the call time for material setup, and material setup needs to be called a lot.
		GFX_PAL_FORMAT = currTex->paletteWrite;

		if (mat->materialFlags0 & TEXTURE_TRANSFORM) {
			glMatrixMode(GL_TEXTURE);
			// TODO: change to a m4x3?
			Mat4x4 textureMatrix;
			Fixed c = cosLerp(mat->texRotation);
			Fixed s = sinLerp(mat->texRotation);
			textureMatrix.m[r1x] = c * mat->texScaleX;
			textureMatrix.m[r2x] = s * mat->texScaleX;
			textureMatrix.m[r1y] = -s * mat->texScaleY;
			textureMatrix.m[r2y] = c * mat->texScaleY;
			// this must be multiplied by 16, because that's how the gpu handles it for some reason
			textureMatrix.m[r1w] = mat->texOffsX.value * 16;
			textureMatrix.m[r2w] = mat->texOffsY.value * 16;
			textureMatrix.m[r1z] = 0;
			textureMatrix.m[r2z] = 0;
			textureMatrix.m[r3x] = 0;
			textureMatrix.m[r3y] = 0;
			textureMatrix.m[r3z] = 0;
			textureMatrix.m[r3w] = 0;
			textureMatrix.m[r4x] = 0;
			textureMatrix.m[r4y] = 0;
			textureMatrix.m[r4z] = 0;
			textureMatrix.m[r4w] = 0;
			glLoadMatrix4x4((m4x4*)&textureMatrix);
		}
		glMatrixMode(GL_MODELVIEW);
		isTransparent = isTransparent || currTex->type == GL_RGB32_A3 || currTex->type == GL_RGB8_A5;
	}
	else {
		GFX_TEX_FORMAT = 0;
		GFX_PAL_FORMAT = 0;
	}

	return isTransparent || (mat->stencilPack & STENCIL_FORCE_OPAQUE_ORDERING) != 0;
}

// created primarily to prevent flushing the data every time, since we seldom update that
ITCM_CODE void DrawList(unsigned int* list) {
	while ((DMA_CR(3) & DMA_BUSY));
	DMA_SRC(3) = ((unsigned int)list) + 4;
	DMA_DEST(3) = 0x4000400;
	DMA_CR(3) = DMA_FIFO | (*list);
	while (DMA_CR(3) & DMA_BUSY);
}

ITCM_CODE void RenderModelRigged(Model *model, Vec3 *position, Vec3 *scale, Quaternion *rotation, SDMaterial *mats, Animator *animator, int renderPriority) {
	//threadSleep(1000000);
	// set current matrix to be model matrix
	glMatrixMode(GL_MODELVIEW);
	// ensure materials are valid...
	VertexHeader *currVertexGroup = model->vertexGroups;
	if (mats == NULL) {
		mats = model->defaultMats;
	}
	while (GFX_STATUS & (1 << 14));
	// a little silly, but lets push until the end of the matrix, then store our base matrix in the last bone we would use.
	const int lastBone = Min(31, model->skeletonCount) - 1;
	for (int i = 0; i <= lastBone; ++i) {
		glPushMatrix();
	}
	// set up base object matrix
	Mat4x4 rotationMatrix = Mat4x4::Rotation(*rotation);
	rotationMatrix.m[r1w] = position->x - cameraRecentering.x;
	rotationMatrix.m[r2w] = position->y - cameraRecentering.y;
	rotationMatrix.m[r3w] = position->z - cameraRecentering.z;
	glLoadMatrix4x4((m4x4*)&rotationMatrix);
	glScalef32(scale->x, scale->y, scale->z);
	glStoreMatrix(lastBone);

	// hardware AABB test
	if (!BoxTest(model->boundsMin.x, model->boundsMin.y, model->boundsMin.z, model->boundsMax.x - model->boundsMin.x, model->boundsMax.y - model->boundsMin.y, model->boundsMax.z - model->boundsMin.z)) {
		if (31 > model->skeletonCount) {
			glPopMatrix(model->skeletonCount);
		}
		else {
			glPopMatrix(31);
		}
		return;
	}

	// cache up to 31 bones.
	for (int i = 0; i <= lastBone; ++i) {
		//glRestoreMatrix(lastBone);
		// get all parents
		// if parent is within stack already, then don't recalculate it!
		if (model->skeleton[i].parent < i) {
			// no parent! use model transform!
			if (model->skeleton[i].parent < 0) {
				glRestoreMatrix(lastBone);
			} else {
				glRestoreMatrix(model->skeleton[i].parent);
			}
		} else {
			glRestoreMatrix(lastBone);
			int parentQueue[128];
			int parentQueueSlot = 0;
			for (int parent = model->skeleton[i].parent; parent != -1; parent = model->skeleton[parent].parent) {
				parentQueue[parentQueueSlot] = parent;
				++parentQueueSlot;
			}
			for (int parent = parentQueueSlot - 1; parent >= 0; --parent) {
				glMultMatrix4x4((m4x4*)&animator->items[parentQueue[parent]].matrix);
			}
		}
		glMultMatrix4x4((m4x4*)&animator->items[i].matrix);
		glStoreMatrix(i);
	}

	for (int i = 0; i <= lastBone; ++i) {
		// apply inverse matrices now
		glRestoreMatrix(i);
		glMultMatrix4x4((m4x4*)&model->skeleton[i].inverseMatrix);
		glStoreMatrix(i);
	}

	// if a mesh has > 31 bones, then we need to set up the basic matrix too
	Mat4x4 matrix;
	if (model->skeletonCount > 31) {
		Mat4x4 scaleMatrix = Mat4x4::Scale(*scale);
		matrix = scaleMatrix.Multiply4x3(rotationMatrix);
		matrix.m[r1w] = position->x;
		matrix.m[r2w] = position->y;
		matrix.m[r3w] = position->z;
	}
	bool skipTransparentOrOpaque = renderPriority == RENDER_PRIO_RESERVED;
	bool transparentSkip = skipTransparentOrOpaque;
	bool modelQueued = false;
	if (model->NativeModel == NULL) {
		int currBone = -1;
		const int vertGroupCount = model->vertexGroupCount;
		for (int i = 0; i < vertGroupCount; ++i) {
			if (currVertexGroup->bitFlags & VTX_MATERIAL_CHANGE) {
				transparentSkip = skipTransparentOrOpaque;
				if (SetupMaterial(&mats[currVertexGroup->material], true)) {
					transparentSkip = !skipTransparentOrOpaque;
					if (renderPriority != RENDER_PRIO_RESERVED) {
						if (!modelQueued) {
							AppendDrawCall(model, position, scale, rotation, mats, animator, renderPriority);
							modelQueued = true;
						}
						currVertexGroup = (VertexHeader*)((uint32_t)(&(currVertexGroup->vertices)) + (uint32_t)(sizeof(Vertex) * (currVertexGroup->count)));
						continue;
					}
				}
				currBone = -1;
			}
			if (transparentSkip) {
				currVertexGroup = (VertexHeader*)((uint32_t)(&(currVertexGroup->vertices)) + (uint32_t)(sizeof(Vertex) * (currVertexGroup->count)));
				continue;
			}
			if (currVertexGroup->bitFlags & VTX_QUAD) {
				if (currVertexGroup->bitFlags & VTX_STRIPS) {
					glBegin(GL_QUAD_STRIP);
				}
				else {
					glBegin(GL_QUAD);
				}
			}
			else {
				if (currVertexGroup->bitFlags & VTX_STRIPS) {
					glBegin(GL_TRIANGLE_STRIP);
				}
				else {
					glBegin(GL_TRIANGLE);
				}
			}
			const int vertCount = currVertexGroup->count;
			for (int i2 = 0; i2 < vertCount; ++i2) {
				const Vertex* currVert = &((&(currVertexGroup->vertices))[i2]);
				if (currVert->boneID != currBone) {
					currBone = currVert->boneID;
					if (currBone > 31) {
						glLoadMatrix4x4((m4x4*)&matrix);
						// get all parents
						int parentQueue[128];
						int parentQueueSlot = 0;
						for (int parent = model->skeleton[currBone].parent; parent != -1; parent = model->skeleton[parent].parent) {
							parentQueue[parentQueueSlot] = parent;
							++parentQueueSlot;
						}
						for (int parent = parentQueueSlot - 1; parent >= 0; --parent) {
							glMultMatrix4x4((m4x4*)&animator->items[parentQueue[parent]].matrix);
						}
						glMultMatrix4x4((m4x4*)&animator->items[currBone].matrix);
						glMultMatrix4x4((m4x4*)&model->skeleton[currBone].inverseMatrix);
					}
					else {
						glRestoreMatrix(currBone);
					}
				}
				glNormal(currVert->normal);
				glTexCoord2t16(currVert->u, currVert->v);
				glVertex3v16(currVert->x, currVert->y, currVert->z);
			}
			currVertexGroup = (VertexHeader*)((uint32_t)(&(currVertexGroup->vertices)) + (uint32_t)(sizeof(Vertex) * (currVertexGroup->count)));
			//glEnd();
		}
	}
	else {
		DSNativeModel* dsnm = (DSNativeModel*)model->NativeModel;
		for (int i = 0; i < dsnm->FIFOCount; ++i) {
			// uh oh!! we can free except the cache!! assume in order then!!
			if (currVertexGroup == NULL) {
				transparentSkip = skipTransparentOrOpaque;
				if (SetupMaterial(&mats[i], true)) {
					transparentSkip = !skipTransparentOrOpaque;
					if (renderPriority != RENDER_PRIO_RESERVED) {
						if (!modelQueued) {
							AppendDrawCall(model, position, scale, rotation, mats, animator, renderPriority);
						}
						continue;
					}
				}
			}
			else {
				if (currVertexGroup->bitFlags & VTX_MATERIAL_CHANGE) {
					transparentSkip = skipTransparentOrOpaque;
					if (SetupMaterial(&mats[currVertexGroup->material], true)) {
						transparentSkip = !skipTransparentOrOpaque;
						if (renderPriority != RENDER_PRIO_RESERVED) {
							if (!modelQueued) {
								AppendDrawCall(model, position, scale, rotation, mats, animator, renderPriority);
							}
							currVertexGroup = (VertexHeader*)((uint32_t)(&(currVertexGroup->vertices)) + (uint32_t)(sizeof(Vertex) * (currVertexGroup->count)));
							continue;
						}
					}
				}
			}
			if (transparentSkip) {
				if (currVertexGroup != NULL) {
					currVertexGroup = (VertexHeader*)((uint32_t)(&(currVertexGroup->vertices)) + (uint32_t)(sizeof(Vertex) * (currVertexGroup->count)));
				}
				continue;
			}
			if (dsnm->FIFOBatches[i] != NULL) {
				//glCallList((u32*)dsnm->FIFOBatches[i]);
				DrawList((unsigned int*)dsnm->FIFOBatches[i]);
			}
			if (currVertexGroup != NULL) {
				currVertexGroup = (VertexHeader*)((uint32_t)(&(currVertexGroup->vertices)) + (uint32_t)(sizeof(Vertex) * (currVertexGroup->count)));
			}
		}
	}
	if (31 > model->skeletonCount) {
		glPopMatrix(model->skeletonCount);
	} else {
		glPopMatrix(31);
	}
}

ITCM_CODE void RenderModel(Model *model, Vec3 *position, Vec3 *scale, Quaternion *rotation, SDMaterial *mats, int renderPriority) {
	// have to work around the DS' jank by omitting scale from the MODELVIEW matrix for normals, but not the POSITION matrix
	//Vec3 matrixSize;
	//GetMatrixLengths(matrix, &matrixSize);
	// set current matrix to be model matrix
	glMatrixMode(GL_MODELVIEW);
	//glPushMatrix();
	Mat4x4 rotationMatrix = Mat4x4::Rotation(*rotation);
	rotationMatrix.m[r1w] = position->x - cameraRecentering.x;
	rotationMatrix.m[r2w] = position->y - cameraRecentering.y;
	rotationMatrix.m[r3w] = position->z - cameraRecentering.z;
	glLoadMatrix4x4((m4x4*)&rotationMatrix);
	glScalef32(scale->x, scale->y, scale->z);

	// hardware AABB test
	if (!BoxTest(model->boundsMin.x, model->boundsMin.y, model->boundsMin.z, model->boundsMax.x - model->boundsMin.x, model->boundsMax.y - model->boundsMin.y, model->boundsMax.z - model->boundsMin.z)) {
		return;
	}

	VertexHeader *currVertexGroup = model->vertexGroups;
	if (mats == NULL) {
		mats = model->defaultMats;
	}
	bool skipTransparentOrOpaque = renderPriority == RENDER_PRIO_RESERVED;
	bool transparentSkip = skipTransparentOrOpaque;
	bool modelQueued = false;
	if (model->NativeModel == NULL) {
		const int vertGroupCount = model->vertexGroupCount;
		for (int i = 0; i < vertGroupCount; ++i) {
			if (currVertexGroup->bitFlags & VTX_MATERIAL_CHANGE) {
				transparentSkip = skipTransparentOrOpaque;
				// update our material
				if (SetupMaterial(&mats[currVertexGroup->material], false)) {
					transparentSkip = !skipTransparentOrOpaque;
					if (renderPriority != RENDER_PRIO_RESERVED) {
						if (!modelQueued) {
							AppendDrawCall(model, position, scale, rotation, mats, NULL, renderPriority);
							modelQueued = true;
						}
						currVertexGroup = (VertexHeader*)((uint32_t)(&(currVertexGroup->vertices)) + (uint32_t)(sizeof(Vertex) * (currVertexGroup->count)));
						continue;
					}
				}
			}
			if (transparentSkip) {
				currVertexGroup = (VertexHeader*)((uint32_t)(&(currVertexGroup->vertices)) + (uint32_t)(sizeof(Vertex) * (currVertexGroup->count)));
				continue;
			}
			if (!(currVertexGroup->bitFlags & VTX_QUAD)) {
				if (currVertexGroup->bitFlags & VTX_STRIPS) {
					glBegin(GL_TRIANGLE_STRIP);
				}
				else {
					glBegin(GL_TRIANGLE);
				}
			}
			else {
				if (currVertexGroup->bitFlags & VTX_STRIPS) {
					glBegin(GL_QUAD_STRIP);
				}
				else {
					glBegin(GL_QUAD);
				}
			}
			const int vertCount = currVertexGroup->count;
			for (int i2 = 0; i2 < vertCount; ++i2) {
				Vertex* currVert = &((&(currVertexGroup->vertices))[i2]);
				glNormal(currVert->normal);
				glTexCoord2t16(currVert->u, currVert->v);
				glVertex3v16(currVert->x, currVert->y, currVert->z);
			}

			currVertexGroup = (VertexHeader*)((uint32_t)(&(currVertexGroup->vertices)) + (uint32_t)(sizeof(Vertex) * (currVertexGroup->count)));
			//glEnd();
		}
	 }
	else {
		DSNativeModel* dsnm = (DSNativeModel*)model->NativeModel;
		for (int i = 0; i < dsnm->FIFOCount; ++i) {
			// uh oh!! we can free except the cache!! assume in order then!!
			if (currVertexGroup == NULL) {
				transparentSkip = skipTransparentOrOpaque;
				if (SetupMaterial(&mats[i], false)) {
					transparentSkip = !skipTransparentOrOpaque;
					if (renderPriority != RENDER_PRIO_RESERVED) {
						if (!modelQueued) {
							AppendDrawCall(model, position, scale, rotation, mats, NULL, renderPriority);
							modelQueued = true;
						}
						continue;
					}
				}
			}
			else {
				if (currVertexGroup->bitFlags & VTX_MATERIAL_CHANGE) {
					transparentSkip = skipTransparentOrOpaque;
					if (SetupMaterial(&mats[currVertexGroup->material], false)) {
						transparentSkip = !skipTransparentOrOpaque;
						if (renderPriority != RENDER_PRIO_RESERVED) {
							if (!modelQueued) {
								AppendDrawCall(model, position, scale, rotation, mats, NULL, renderPriority);
								modelQueued = true;
							}
							currVertexGroup = (VertexHeader*)((uint32_t)(&(currVertexGroup->vertices)) + (uint32_t)(sizeof(Vertex) * (currVertexGroup->count)));
							continue;
						}
					}
				}
			}
			if (transparentSkip) {
				if (currVertexGroup != NULL) {
					currVertexGroup = (VertexHeader*)((uint32_t)(&(currVertexGroup->vertices)) + (uint32_t)(sizeof(Vertex) * (currVertexGroup->count)));
				}
				continue;
			}
			if (dsnm->FIFOBatches[i] != NULL) {
				//glCallList((u32*)dsnm->FIFOBatches[i]);
				DrawList((unsigned int*)dsnm->FIFOBatches[i]);
			}
			if (currVertexGroup != NULL) {
				currVertexGroup = (VertexHeader*)((uint32_t)(&(currVertexGroup->vertices)) + (uint32_t)(sizeof(Vertex) * (currVertexGroup->count)));
			}
		}
	}
}

void UploadTexture(Texture* input) {
	// get palette size
	int paletteSize = 0;
	switch (input->type) {
	case 1:
		paletteSize = 32;
		break;
	case 2:
		paletteSize = 4;
		break;
	case 3:
		paletteSize = 16;
		break;
	case 4:
		paletteSize = 256;
		break;
	case 6:
		paletteSize = 8;
		break;
	case 7:
	case 8:
		paletteSize = 0;
		break;
	}
	// send the palette to VRAM if it exists
	if (paletteSize != 0) {
		glGenTextures(1, &input->paletteId);
		glBindTexture(0, input->paletteId);
		glColorTableEXT(0, 0, paletteSize, 0, 0, input->palette);
	}
	// now send the texture data to vram
	glGenTextures(1, &input->textureId);
	glBindTexture(0, input->textureId);
	unsigned int flagValue = TEXGEN_TEXCOORD;
	switch (input->mapTypeU) {
	case 1:
		flagValue |= GL_TEXTURE_FLIP_S;
	case 0:
		flagValue |= GL_TEXTURE_WRAP_S;
		break;
	}
	switch (input->mapTypeV) {
	case 1:
		flagValue |= GL_TEXTURE_FLIP_T;
	case 0:
		flagValue |= GL_TEXTURE_WRAP_T;
		break;
	}
	if (!(input->palette[0] & 0x8000)) {
		flagValue |= GL_TEXTURE_COLOR0_TRANSPARENT;
	}
	glTexImage2D(0, 0, (GL_TEXTURE_TYPE_ENUM)input->type, input->width, input->height, 0, flagValue, input->image);
	input->uploaded = true;
	// re-assign so we can get the palette format here...
	if (paletteSize != 0) {
		glAssignColorTable(0, input->paletteId);
	}
	// ugh, derive the data from libnds now
	gl_texture_data *tex = (gl_texture_data*)DynamicArrayGet(&glGlob->texturePtrs, input->textureId);
	input->textureWrite = tex->texFormat;
	input->paletteWrite = ((gl_palette_data*)DynamicArrayGet( &glGlob->palettePtrs, tex->palIndex ))->addr;
}

Texture *LoadTextureFromRAM(Texture* newTex, bool upload, char* name) {
	char* input = name;

	int flushRange = 0x30;

	int texMultiplier = 1 * 4096;

	switch (newTex->type) {
	case 1:
		texMultiplier = 1 * 4096;
		flushRange += 32 * 2;
		break;
	case 2:
		texMultiplier = 1 * 4096 / 4;
		flushRange += 4 * 2;
		break;
	case 3:
		texMultiplier = 1 * 4096 / 2;
		flushRange += 16 * 2;
		break;
	case 4:
		texMultiplier = 1 * 4096;
		flushRange += 256 * 2;
		break;
	case 6:
		texMultiplier = 1 * 4096;
		flushRange += 8 * 2;
		break;
	case 7:
	case 8:
		texMultiplier = 2 * 4096;
		break;
	}

	int width = Pow(2 * 4096, (newTex->width + 3) * 4096);
	int height = Pow(2 * 4096, (newTex->height + 3) * 4096);

	flushRange += mulf32(width, mulf32(height, texMultiplier)) / 4096;

	DC_FlushRange(newTex, flushRange);
	newTex->palette = (unsigned short*)((uint32_t)newTex->palette + (uint32_t)newTex);
	newTex->image = (unsigned char*)((uint32_t)newTex->image + (uint32_t)newTex);
	if (upload) {
		UploadTexture(newTex);
	}
	// save the name
	newTex->name = (char*)malloc(strlen(input) + 1);
	strcpy(newTex->name, input);

	Texture* retTex = newTex;

	if (!newTex->dontReleaseFromRAM) {
		retTex = (Texture*)malloc(sizeof(Texture));
		*retTex = (*newTex);
		retTex->palette = NULL;
		retTex->image = NULL;
		free(newTex);
	}

	// place in linked list
	if (startTexture.next != NULL) {
		startTexture.next->prev = retTex;
	}

	retTex->next = startTexture.next;
	startTexture.next = retTex;
	retTex->prev = &startTexture;

	return retTex;
}

Texture *LoadTexture(char *input, bool upload) {
	// first check if we have the texture cached
	Texture *tex = startTexture.next;
	while (tex != NULL) {
		if (strcmp(input, tex->name) == 0) {
			return tex;
		}
		tex = tex->next;
	}
	// otherwise try opening it
	FILE *f = fopen(input, "rb");
	if (f == NULL) {
		return NULL;
	}
	fseek(f, 0, SEEK_END);
	int fsize = ftell(f);
	fseek(f, 0, SEEK_SET);
	Texture *newTex = (Texture*)malloc(fsize);
	fread_MusicYielding(newTex, fsize, 1, f);
	fclose(f);
	return LoadTextureFromRAM(newTex, upload, input);
}

void UnloadTexture(Texture *tex) {
	// very simple, just remove it from the linked list
	tex->prev->next = tex->next;
	tex->next->prev = tex->prev;
	free(tex->name);
	if (tex->uploaded) {
		glDeleteTextures(1, &tex->paletteId);
		glDeleteTextures(1, &tex->textureId);
	}
	free(tex);
}

ITCM_CODE void RenderTransparentModels() {
	qsort(modelDrawCalls, modelDrawCallCount, sizeof(ModelDrawCall), TransparentSortFunction);
	for (int i = 0; i < modelDrawCallCount; ++i) {
		if (modelDrawCalls[i].animator != NULL) {
			RenderModelRigged(modelDrawCalls[i].model, &modelDrawCalls[i].position, &modelDrawCalls[i].scale, &modelDrawCalls[i].rotation, modelDrawCalls[i].materials, modelDrawCalls[i].animator, RENDER_PRIO_RESERVED);
		}
		else {
			RenderModel(modelDrawCalls[i].model, &modelDrawCalls[i].position, &modelDrawCalls[i].scale, &modelDrawCalls[i].rotation, modelDrawCalls[i].materials, RENDER_PRIO_RESERVED);
		}
	}
	modelDrawCallCount = 0;
}

void SetLightDir(int lightId, Fixed x, Fixed y, Fixed z) {
	if (lightId < 0 || lightId > 3) return;
	nativeLightNormal[lightId].x = x;
	nativeLightNormal[lightId].y = y;
	nativeLightNormal[lightId].z = z;
	x.value /= 8;
	y.value /= 8;
	z.value /= 8;
	if (x >= 512) {
		x = 511;
	}
	if (y >= 512) {
		y = 511;
	}
	if (z >= 512) {
		z = 511;
	}
	lightNormal[lightId].x = x;
	lightNormal[lightId].y = y;
	lightNormal[lightId].z = z;
	// set light dir
	glLoadIdentity();
	glLight(lightId, lightColor[lightId], lightNormal[lightId].x, lightNormal[lightId].y, lightNormal[lightId].z);
}

void SetLightColor(int lightId, char R, char G, char B) {
	if (lightId < 0 || lightId > 3) return;
	lightColor[lightId] = RGB15(R, G, B);
	glLoadIdentity();
	glLight(lightId, lightColor[lightId], lightNormal[lightId].x, lightNormal[lightId].y, lightNormal[lightId].z);
}

void SetAmbientColor(char R, char G, char B) {
	ambientColor.x = R & 0x1F;
	ambientColor.y = G & 0x1F;
	ambientColor.z = B & 0x1F;
}

void EnableLight(int lightId) {
	if (lightId < 0 || lightId > 3) return;
	lightEnabled[lightId] = true;
}

void DisableLight(int lightId) {
	if (lightId < 0 || lightId > 3) return;
	lightEnabled[lightId] = false;
}

void LoadAnimationFromRAM(Animation* anim) {
	for (int i = 0; i < anim->keyframeSetCount; ++i) {
		anim->sets[i] = (KeyframeSet*)((uint32_t)anim->sets[i] + (uint32_t)anim);
		for (int j = 0; j < anim->sets[i]->keyframeCount; ++j) {
			// conversion to 3 bit decimal for optimized animator updating...
			anim->sets[i]->keyframes[j].frame >>= 9;
		}
	}
}

Animation *LoadAnimation(char *input) {
	char* fileDir = DirToNative(input);
	FILE *f = fopen(fileDir, "rb");
	free(fileDir);
	if (f == NULL) {
		return NULL;
	}
	fseek(f, 0, SEEK_END);
	int fsize = ftell(f);
	fseek(f, 0, SEEK_SET);
	Animation *retValue = (Animation*)malloc(fsize);
	fread_MusicYielding(retValue, fsize, 1, f);
	fclose(f);
	LoadAnimationFromRAM(retValue);
	return retValue;
}

void LoadAnimationAsyncCallback(void* data, bool success) {
	AnimationCallbackData* acd = (AnimationCallbackData*)data;
	fclose(acd->f);
	if (!success) {
		acd->callBack(acd->callBackData, NULL);
		free(acd->anim);
	}
	else {
		acd->callBack(acd->callBackData, acd->anim);
	}
	free(acd);
}

int LoadAnimationAsync(char* input, void (*callBack)(void* data, Animation* anim), void* callBackData) {
	if (callBack == NULL) {
		// ?
		return -1;
	}
	char* fileDir = DirToNative(input);
	FILE* f = fopen(fileDir, "rb");
	free(fileDir);
	if (f == NULL) {
		callBack(callBackData, NULL);
		return -1;
	}
	fseek(f, 0, SEEK_END);
	int fsize = ftell(f);
	fseek(f, 0, SEEK_SET);
	Animation* retValue = (Animation*)malloc(fsize);
	AnimationCallbackData* acd = (AnimationCallbackData*)malloc(sizeof(AnimationCallbackData));
	acd->f = f;
	acd->anim = retValue;
	acd->callBack = callBack;
	acd->callBackData = callBackData;
	return fread_Async(retValue, fsize, 1, f, 0, (AsyncFileCallback)LoadAnimationAsyncCallback, acd);
}

Animator *CreateAnimator(Model *referenceModel) {
	Animator *retValue = (Animator*)malloc(sizeof(Animator));
	retValue->speed = 4096;
	retValue->items = (AnimatorItem*)malloc(sizeof(AnimatorItem)*referenceModel->skeletonCount);
	retValue->itemCount = referenceModel->skeletonCount;
	retValue->currFrame = 0;
	retValue->lerpPrevTime = 0;
	retValue->lerpPrevTimeTarget = 0;
	retValue->currAnimation = NULL;
	retValue->queuedAnimCount = 0;
	retValue->loop = true;
	retValue->paused = false;
	// set up default animation values...
	for (int i = 0; i < referenceModel->skeletonCount; ++i) {
		memcpy(&retValue->items[i].currRotation, &referenceModel->skeleton[i].rotation, sizeof(Quaternion));
		memcpy(&retValue->items[i].currPosition, &referenceModel->skeleton[i].position, sizeof(Vec3));
		memcpy(&retValue->items[i].currScale, &referenceModel->skeleton[i].scale, sizeof(Vec3));
		AnimatorItem *currItem = &retValue->items[i];
		// and also matrix
		Mat4x4 w1;
		Mat4x4 w2;
		Mat4x4 w3;
		w1 = Mat4x4::Scale(currItem->currScale);
		w2 = Mat4x4::Rotation(currItem->currRotation);
		w3 = w1.Multiply3x3(w2);
		w3.m[r1w] = currItem->currPosition.x;
		w3.m[r2w] = currItem->currPosition.y;
		w3.m[r3w] = currItem->currPosition.z;
		memcpy(&retValue->items[i].matrix, &w3, sizeof(m4x4));
	}
	return retValue;
}

ITCM_CODE int LerpAnimator(int left, int right, i29d3 t) {
	return left + ((t * (right - left)) >> 3);
}

__attribute__((target("arm")))
Quaternion QuatLerpAnimator(Quaternion* left, Quaternion* right, i29d3 t) {
	Quaternion retValue;
	Fixed dot = (left->x * right->x) + (left->y * right->y) + (left->z * right->z) + (left->w * right->w);
	if (dot < 0) {
		retValue.x = left->x + ((t * (-right->x - left->x)) >> 3);
		retValue.y = left->y + ((t * (-right->y - left->y)) >> 3);
		retValue.z = left->z + ((t * (-right->z - left->z)) >> 3);
		retValue.w = left->w + ((t * (-right->w - left->w)) >> 3);
	}
	else {
		retValue.x = left->x + ((t * (right->x - left->x)) >> 3);
		retValue.y = left->y + ((t * (right->y - left->y)) >> 3);
		retValue.z = left->z + ((t * (right->z - left->z)) >> 3);
		retValue.w = left->w + ((t * (right->w - left->w)) >> 3);
	}
	retValue.Normalize();
	return retValue;
}

const int* CLIPMTX_RESULT = (int*)0x4000640;

ITCM_CODE void UpdateAnimator(Animator *animator, Model *referenceModel) {
	if (animator->currAnimation == NULL || animator->paused) {
		return;
	}
	animator->currFrame += animator->speed;
	animator->lerpPrevTime += animator->speed;
	if (animator->lerpPrevTime > animator->lerpPrevTimeTarget) {
		animator->lerpPrevTime = animator->lerpPrevTimeTarget;
	}
	if (animator->queuedAnimCount == 0) {
		// no queued animations, handle end of animation normally
		if (animator->loop) {
			animator->currFrame = animator->currFrame % animator->currAnimation->lastFrame;
		}
		else {
			animator->currFrame = Min(animator->currFrame, animator->currAnimation->lastFrame);
		}
	}
	else {
		// queued animation(s), switch to them
		if (animator->currFrame >= animator->currAnimation->lastFrame) {
			Fixed currFrame = animator->currFrame - animator->currAnimation->lastFrame;
			PlayAnimation(animator, animator->queuedAnims[0], animator->queuedLerpTimes[0]);
			animator->currFrame = currFrame;
			for (int i = 0; i < animator->queuedAnimCount - 1; ++i) {
				animator->queuedAnims[i] = animator->queuedAnims[i + 1];
				animator->queuedLerpTimes[i] = animator->queuedLerpTimes[i + 1];
			}
			--animator->queuedAnimCount;
		}
	}
	int animCurrFrame = animator->currFrame >> 9;
	int currKeyFrame = 0;
	for (int i = 0; i < animator->currAnimation->keyframeSetCount; ++i) {
		KeyframeSet *currSet = animator->currAnimation->sets[i];
		// catch animations wanting to animate more bones than we have
		if (currSet->target >= animator->itemCount) {
			continue;
		}

		// optimization: cache the current keyframe between sets. typically works, unless animation is code generated
		if (currKeyFrame >= currSet->keyframeCount) {
			currKeyFrame = 0;
		}
		Keyframe *leftKeyframe = &currSet->keyframes[currKeyFrame];
		Keyframe *rightKeyframe = &currSet->keyframes[currKeyFrame];
		// catch it out if it's wrong
		if (leftKeyframe->frame >= animCurrFrame) {
			currKeyFrame = 0;
		}
		for (int i2 = currKeyFrame; i2 < currSet->keyframeCount; ++i2) {
			if (animCurrFrame < currSet->keyframes[i2].frame) {
				rightKeyframe = &currSet->keyframes[i2];
				int maxKeyframe = i2 - 1;
				if (maxKeyframe < 0) {
					maxKeyframe = i2;
				}
				leftKeyframe = &currSet->keyframes[maxKeyframe];
				currKeyFrame = maxKeyframe;
				break;
			}
		}
		// previous and next key frame acquired, now simply lerp between them
		int lerpAmnt = ((animCurrFrame - leftKeyframe->frame) << 3) / (rightKeyframe->frame - leftKeyframe->frame); //divf32(animator->currFrame - leftKeyframe->frame, rightKeyframe->frame - leftKeyframe->frame);
		switch (currSet->type) {
			case 0: {
				/*Quaternion leftRot = Quaternion(leftKeyframe->data.temp.x,leftKeyframe->data.temp.y,leftKeyframe->data.temp.z,leftKeyframe->data.temp.w);
				Quaternion rightRot = Quaternion(rightKeyframe->data.temp.x,rightKeyframe->data.temp.y,rightKeyframe->data.temp.z,rightKeyframe->data.temp.w);
				animator->items[currSet->target].currRotation = leftRot.Slerp(rightRot, lerpAmnt << 9);*/
				Quaternion leftRot = Quaternion(leftKeyframe->data.temp.x,leftKeyframe->data.temp.y,leftKeyframe->data.temp.z,leftKeyframe->data.temp.w);
				Quaternion rightRot = Quaternion(rightKeyframe->data.temp.x,rightKeyframe->data.temp.y,rightKeyframe->data.temp.z,rightKeyframe->data.temp.w);
				animator->items[currSet->target].currRotation = QuatLerpAnimator(&leftRot, &rightRot, lerpAmnt);
				}
				break;
			case 1: {
				animator->items[currSet->target].currPosition.x = LerpAnimator(leftKeyframe->data.position.x, rightKeyframe->data.position.x, lerpAmnt);
				animator->items[currSet->target].currPosition.y = LerpAnimator(leftKeyframe->data.position.y, rightKeyframe->data.position.y, lerpAmnt);
				animator->items[currSet->target].currPosition.z = LerpAnimator(leftKeyframe->data.position.z, rightKeyframe->data.position.z, lerpAmnt);
				}
				break;
			case 2: {
				animator->items[currSet->target].currScale.x = LerpAnimator(leftKeyframe->data.scale.x, rightKeyframe->data.scale.x, lerpAmnt);
				animator->items[currSet->target].currScale.y = LerpAnimator(leftKeyframe->data.scale.y, rightKeyframe->data.scale.y, lerpAmnt);
				animator->items[currSet->target].currScale.z = LerpAnimator(leftKeyframe->data.scale.z, rightKeyframe->data.scale.z, lerpAmnt);
				}
				break;
		}
	}
	
	// matrix time
	Fixed prevLerpAmnt;
	if (animator->lerpPrevTimeTarget == 0) {
		prevLerpAmnt = 4096;
	}
	else {
		prevLerpAmnt = divf32f(animator->lerpPrevTime, animator->lerpPrevTimeTarget);
	}
	if (prevLerpAmnt < 0) {
		prevLerpAmnt = 4096;
	}
	if (prevLerpAmnt >= 4096) {
		for (int i = 0; i < animator->itemCount; ++i) {
			AnimatorItem *currItem = &animator->items[i];
			// i choose...optimization, here.
			Mat4x4 w1;
			Mat4x4 w2;
			w1 = Mat4x4::Scale(currItem->currScale);
			w2 = Mat4x4::Rotation(currItem->currRotation);
			// use the matrix hardware!
			glLoadMatrix4x4((m4x4*)&w2);
			// do 4x4 for now...
			glMultMatrix4x4((m4x4*)&w1);
			for (int j = 0; j < 16; ++j) {
				animator->items[i].matrix.m[j] = CLIPMTX_RESULT[j];
			}
			animator->items[i].matrix.m[r1w] = currItem->currPosition.x;
			animator->items[i].matrix.m[r2w] = currItem->currPosition.y;
			animator->items[i].matrix.m[r3w] = currItem->currPosition.z;
			//MatrixToDSMatrix(&w3, &animator->items[i].matrix);
		}
	} else {
		for (int i = 0; i < animator->itemCount; ++i) {
			AnimatorItem *currItem = &animator->items[i];
			// i choose...optimization, here.
			Mat4x4 w1;
			Mat4x4 w2;
			Quaternion slerpedQuat;
			slerpedQuat = currItem->prevRotation.Slerp(currItem->currRotation, prevLerpAmnt);
			w1 = Mat4x4::Scale(Vec3(Lerp(currItem->prevScale.x, currItem->currScale.x, prevLerpAmnt),
				Lerp(currItem->prevScale.y, currItem->currScale.y, prevLerpAmnt),
				Lerp(currItem->prevScale.z, currItem->currScale.z, prevLerpAmnt)));
			w2 = Mat4x4::Rotation(slerpedQuat);
			// use the matrix hardware!
			glLoadMatrix4x4((m4x4*)&w2);
			// do 4x4 for now...
			glMultMatrix4x4((m4x4*)&w1);
			for (int j = 0; j < 16; ++j) {
				animator->items[i].matrix.m[j] = CLIPMTX_RESULT[j];
			}
			animator->items[i].matrix.m[r1w] = Lerp(currItem->prevPosition.x, currItem->currPosition.x, prevLerpAmnt);
			animator->items[i].matrix.m[r2w] = Lerp(currItem->prevPosition.y, currItem->currPosition.y, prevLerpAmnt);
			animator->items[i].matrix.m[r3w] = Lerp(currItem->prevPosition.z, currItem->currPosition.z, prevLerpAmnt);
		}
	}
}

void PlayAnimation(Animator *animator, Animation *animation, Fixed lerpTime) {
	if (animator == NULL) {
		return;
	}
	animator->currFrame = 0;
	animator->lerpPrevTime = 0;
	animator->lerpPrevTimeTarget = lerpTime;
	animator->currAnimation = animation;
	for (int i = 0; i < animator->itemCount; ++i) {
		memcpy(&animator->items[i].prevPosition, &animator->items[i].currPosition, sizeof(Vec3));
		memcpy(&animator->items[i].prevScale, &animator->items[i].currScale, sizeof(Vec3));
		memcpy(&animator->items[i].prevRotation, &animator->items[i].currRotation, sizeof(Quaternion));
	}
}

void SetSDMaterialTexture(SDMaterial *mat, Texture *texture) {
	if (mat->texture != NULL) {
		mat->texture->numReferences -= 1;
		if (mat->texture->numReferences == 0) {
			UnloadTexture(mat->texture);
		}
	}
	mat->texture = texture;
	if (texture != NULL)
		texture->numReferences += 1;
}

// NOTE: THIS FUNCTION DOES NOT FREE IT
void DestroySDMaterial(SDMaterial *mat) {
	if (mat->texture != NULL) {
		mat->texture->numReferences -= 1;
		if (mat->texture->numReferences == 0) {
			UnloadTexture(mat->texture);
		}
	}
}

void DestroyModel(Model* m) {
	for (int i = 0; i < m->materialCount; ++i) {
		DestroySDMaterial(&m->defaultMats[i]);
	}
#ifdef _WIN32
	if (m->NativeModel != NULL) {
		DeleteMesh(m->NativeModel);
		free(m->NativeModel);
	}
#endif
#ifndef _NOTDS
	if (m->NativeModel != NULL) {
		DSNativeModel* dsnm = (DSNativeModel*)m->NativeModel;
		for (int i = 0; i < dsnm->FIFOCount; ++i) {
			if (dsnm->FIFOBatches[i] != NULL) {
				free(dsnm->FIFOBatches[i]);
			}
		}
		free(dsnm->FIFOBatches);
		free(m->NativeModel);
	}
#endif
	// check for freed model
	if (m->version & 0x80000000) {
		free(m->defaultMats);
		free(m->vertexGroups);
		if (m->skeleton != NULL) {
			free(m->skeleton);
		}
	}
	free(m);
}

// UNLIKE DESTROYMODEL, THIS WILL NOT FREE THE MODEL ITSELF
void DestroyGeneratedModel(Model* m) {
	for (int i = 0; i < m->materialCount; ++i) {
		DestroySDMaterial(&m->defaultMats[i]);
	}
#ifdef _WIN32
	if (m->NativeModel != NULL) {
		DestroyMesh(m->NativeModel);
		free(m->NativeModel);
	}
#endif
#ifndef _NOTDS
	if (m->NativeModel != NULL) {
		DSNativeModel* dsnm = (DSNativeModel*)m->NativeModel;
		for (int i = 0; i < dsnm->FIFOCount; ++i) {
			if (dsnm->FIFOBatches[i] != NULL) {
				free(dsnm->FIFOBatches[i]);
			}
}
		free(dsnm->FIFOBatches);
		free(m->NativeModel);
	}
#endif
	free(m->vertexGroups);
	if (m->skeleton != NULL)
		free(m->skeleton);
}

void LoadBGTexture(Sprite* input) {
#ifdef _NOTDS
	UploadSprite(input, true, true);
	BGTexture = input;
#else
	float paletteMultiplier = 1;
	switch (input->format) {
	case 0:
		paletteMultiplier = 0.5f;
		break;
	case 2:
		paletteMultiplier = 2.0f;
		break;
	}
	int paletteSize = 0;
	switch (input->format) {
	case 0:
		paletteSize = 16;
		break;
	case 1:
		paletteSize = 256;
		break;
	case 2:
		paletteSize = 0;
		break;
	}
	int width = input->width;
	int height = input->height;
	dmaCopy(input->image, bgGetGfxPtr(bgID), width * height * paletteMultiplier);
	dmaCopy(input->palette, BG_PALETTE_SUB, paletteSize * 2);
#endif
}

void InitializeSubBG() {
#ifndef _NOTDS
	bgID = bgInitSub_call(0, BgType_Text8bpp, BgSize_T_256x256, 0, 1);
#else
	subScreenTexture = CreateRenderTexture(256, 192, RENDERTEXTURE_TYPE_BYTE, false, 0, 1);
#endif
}

#ifndef _NOTDS
unsigned int spriteNativeResolutions[] = {
	SpriteSize_8x8,
	SpriteSize_16x16,
	SpriteSize_32x32,
	SpriteSize_64x64,
	SpriteSize_16x8,
	SpriteSize_32x8,
	SpriteSize_32x16,
	SpriteSize_64x32,
	SpriteSize_8x16,
	SpriteSize_8x32,
	SpriteSize_16x32,
	SpriteSize_32x64
};
#endif

void UploadSprite(Sprite* input, bool sub, bool BG) {
#ifndef _NOTDS
	input->DSResolution = spriteNativeResolutions[(int)input->resolution];
	input->gfx = (char*)oamAllocateGfx(sub ? &oamSub : &oamMain, (SpriteSize)input->DSResolution, input->format == 2 ? SpriteColorFormat_Bmp : (SpriteColorFormat)input->format);
	float multiplier = input->format == 0 ? 0.5f : input->format == 1 ? 1.0f : 2.0f;
	dmaCopy(input->image, input->gfx, multiplier * (input->width * input->height));
	input->paletteOffset = 15;
	// TODO: 8 & 4 bit sprites
#else
	TextureRGBA* nativeColors = malloc(sizeof(TextureRGBA) * input->width * input->height);
	switch (input->format) {
	case 1:
		Convert256Palette(input->image, input->palette, nativeColors, input->width, input->height);
		break;
	case 2:
		ConvertPaletteless(input->image, nativeColors, input->width, input->height);
		break;
	}
	NativeTexture* texture = malloc(sizeof(NativeTexture));
	InitializeTexture(texture);
	texture->color = nativeColors;
	texture->width = input->width;
	texture->height = input->height;
	texture->texRef = NULL;
	texture->WrapU = TexWrapClamp;
	texture->WrapV = TexWrapClamp;

	// make first color transparent
	if (!BG && input->format != 2) {
		for (int i = 0; i < input->width * input->height; ++i) {
			if (nativeColors[i].r == nativeColors[0].r && nativeColors[i].g == nativeColors[0].g && nativeColors[i].b == nativeColors[i].b) {
				nativeColors[i].a = 0;
			}
		}
	}
	else if (BG) {
		// un-fuck the texture
		TextureRGBA* newColors = malloc(sizeof(TextureRGBA) * input->width * input->height);
		for (int i = 0; i < input->height; i += 8) {
			for (int j = 0; j < input->width; j += 8) {
				for (int k = 0; k < 8; ++k) {
					for (int l = 0; l < 8; ++l) {
						newColors[(i + k) * input->width + j + l] = nativeColors[i * input->width + ((j + k) * 8) + l];
					}
				}
			}
		}
		free(texture->color);
		texture->color = newColors;
	}

	UpdateTexture(texture, false, MAG_NEAREST, MIN_NEAREST);
	input->nativeSprite = texture;
#endif
	input->uploaded = true;
	input->sub = sub;
}

void LoadSpriteFromRAM(Sprite* sprite) {
	sprite->image = (unsigned char*)((uint32_t)sprite->image + (uint32_t)sprite);
	sprite->palette = (unsigned short*)((uint32_t)sprite->palette + (uint32_t)sprite);
}

Sprite* LoadSprite(char* input, bool sub, bool upload) {
	char* newInput = DirToNative(input);
	FILE* f = fopen(newInput, "rb");
	free(newInput);
	if (f == NULL) {;
		return NULL;
	}
	fseek(f, 0, SEEK_END);
	int fsize = ftell(f);
	fseek(f, 0, SEEK_SET);
	Sprite* newSprite = (Sprite*)malloc(fsize);
	fread_MusicYielding(newSprite, fsize, 1, f);
	fclose(f);
	LoadSpriteFromRAM(newSprite);
	// that's it really, not much setup to be done here
	if (upload) {
		UploadSprite(newSprite, sub, false);
	}
	return newSprite;
}

void LoadSpriteAsyncCallback(void* data, bool success) {
	SpriteCallbackData* scd = (SpriteCallbackData*)data;
	fclose(scd->f);
	if (!success) {
		free(scd->sprite);
		scd->callBack(scd->callBackData, NULL);
	}
	else {
		LoadSpriteFromRAM(scd->sprite);
		if (scd->upload) {
			UploadSprite(scd->sprite, scd->sub, false);
		}
		scd->callBack(scd->callBackData, scd->sprite);
	}
	free(scd);
}

int LoadSpriteAsync(char* input, bool sub, bool upload, void (*callBack)(void* data, Sprite* sprite), void* callBackData) {
	if (callBack == NULL) {
		// ?
		return -1;
	}
	char* newInput = DirToNative(input);
	FILE* f = fopen(newInput, "rb");
	free(newInput);
	if (f == NULL) {
		callBack(callBackData, NULL);
		return -1;
	}
	fseek(f, 0, SEEK_END);
	int fsize = ftell(f);
	fseek(f, 0, SEEK_SET);
	Sprite* newSprite = (Sprite*)malloc(fsize);

	SpriteCallbackData* scd = (SpriteCallbackData*)malloc(sizeof(SpriteCallbackData));
	scd->callBack = callBack;
	scd->callBackData = callBackData;
	scd->sprite = newSprite;
	scd->sub = sub;
	scd->upload = upload;
	scd->f = f;

	return fread_Async(newSprite, fsize, 1, f, 0, (AsyncFileCallback)LoadSpriteAsyncCallback, scd);
}

void UnloadSprite(Sprite* input) {
	if (input->uploaded) {
#ifndef _NOTDS
		oamFreeGfx(input->sub ? &oamSub : &oamMain, input->gfx);
#else
		if (input->uploaded) {
			DeleteTexture(input->nativeSprite);
			free(input->nativeSprite);
		}
#endif
	}
	free(input);
}

void RenderSprite(Sprite* sprite, int x, int y, bool flipX, bool flipY, int xAlign, int yAlign) {
	SpriteDrawCall* drawList;
	int drawListCount;
	if (sprite->sub) {
		if (subSpriteCallCount >= 128) {
			return;
		}
		drawList = subSpriteCalls;
		drawListCount = subSpriteCallCount;
		++subSpriteCallCount;
}
	else {
		if (mainSpriteCallCount >= 128) {
			return;
		}
		drawList = mainSpriteCalls;
		drawListCount = mainSpriteCallCount;
		++mainSpriteCallCount;
	}
	drawList[drawListCount].scaled = false;
	drawList[drawListCount].sprite = sprite;
	drawList[drawListCount].x = x;
	drawList[drawListCount].y = y;
	drawList[drawListCount].flipX = flipX;
	drawList[drawListCount].flipY = flipY;
	drawList[drawListCount].spriteAlignX = xAlign;
	drawList[drawListCount].spriteAlignY = yAlign;
}

void RenderSpriteScaled(Sprite* sprite, int x, int y, bool flipX, bool flipY, Fixed xScale, Fixed yScale, int xAlign, int yAlign) {
	SpriteDrawCall* drawList;
	int drawListCount;
	if (sprite->sub) {
		if (subSpriteCallCount >= 128) {
			return;
		}
		drawList = subSpriteCalls;
		drawListCount = subSpriteCallCount;
		++subSpriteCallCount;
	}
	else {
		if (mainSpriteCallCount >= 128) {
			return;
		}
		drawList = mainSpriteCalls;
		drawListCount = mainSpriteCallCount;
		++mainSpriteCallCount;
	}
	drawList[drawListCount].scaled = true;
	drawList[drawListCount].sprite = sprite;
	drawList[drawListCount].x = x;
	drawList[drawListCount].y = y;
	drawList[drawListCount].xScale = xScale;
	drawList[drawListCount].yScale = yScale;
	drawList[drawListCount].flipX = flipX;
	drawList[drawListCount].flipY = flipY;
	drawList[drawListCount].spriteAlignX = xAlign;
	drawList[drawListCount].spriteAlignY = yAlign;
}

int oamCount = 0;

void oamSetSD(OamState* oam, int id, int x, int y, int priority,
	int palette_alpha, SpriteSize size, SpriteColorFormat format,
	const void* gfxOffset,
	int affineIndex,
	bool sizeDouble, bool hide, bool hflip, bool vflip, bool mosaic) {
	SpriteEntry s;

	if (hide) {
		s.attribute[0] = ATTR0_DISABLED;
		return;
	}

	s.shape = (ObjShape)SPRITE_SIZE_SHAPE(size);
	s.size = (ObjSize)SPRITE_SIZE_SIZE(size);
	s.x = x;
	s.y = y;
	s.palette = palette_alpha;
	s.priority = (ObjPriority)priority;
	s.hFlip = hflip;
	s.vFlip = vflip;
	s.isMosaic = mosaic;
	s.gfxIndex = oamGfxPtrToOffset(oam, gfxOffset);


	if (affineIndex >= 0 && affineIndex < 32) {
		s.rotationIndex = affineIndex;
		s.isSizeDouble = sizeDouble;
		s.isRotateScale = true;
	}
	else {
		s.isSizeDouble = false;
		s.isRotateScale = false;
	}

	if (format != SpriteColorFormat_Bmp) {
		s.colorMode = (ObjColMode)format;
	}
	else {
		s.blendMode = (ObjBlendMode)format;
		s.colorMode = (ObjColMode)0;
	}
	oam->oamMemory[id] = s;
}

void RenderSpriteInternal(SpriteDrawCall* sprite) {
	int realDrawPosX = 0;
	int realDrawPosY = 0;
	if (sprite->spriteAlignX == SpriteAlignLeft) {
		realDrawPosX = sprite->x;
	}
	else if (sprite->spriteAlignX == SpriteAlignCenter) {
		realDrawPosX = sprite->x + 127;
	}
	else if (sprite->spriteAlignX == SpriteAlignRight) {
		realDrawPosX = sprite->x + 255;
	}
	if (sprite->spriteAlignY == SpriteAlignTop) {
		realDrawPosY = sprite->y;
	}
	else if (sprite->spriteAlignY == SpriteAlignCenter) {
		realDrawPosY = sprite->y + 96;
	}
	else if (sprite->spriteAlignY == SpriteAlignBottom) {
		realDrawPosY = sprite->y + 192;
	}
	// TODO: this can only draw one sprite, it'll immediately overwrite future ones...
	if (sprite->sprite->sub) {
		int affineId = -1;
		if (sprite->scaled && spriteMatrixId < 32) {
			oamAffineTransformation(&oamSub, spriteMatrixId, sprite->xScale, 0, 0, sprite->yScale);
			affineId = spriteMatrixId;
			++spriteMatrixId;
		}
		oamSetSD(&oamSub, oamCount, realDrawPosX, realDrawPosY, 0, sprite->sprite->paletteOffset, (SpriteSize)sprite->sprite->DSResolution, (SpriteColorFormat)sprite->sprite->format, sprite->sprite->gfx, affineId, true, false, sprite->flipX, sprite->flipY, false);
	}
	else {
		int affineId = -1;
		if (sprite->scaled && spriteMatrixId < 32) {
			oamAffineTransformation(&oamMain, spriteMatrixId, sprite->xScale, 0, 0, sprite->yScale);
			affineId = spriteMatrixId;
			++spriteMatrixId;
		}
		oamSetSD(&oamMain, oamCount, realDrawPosX, realDrawPosY, 0, sprite->sprite->paletteOffset, (SpriteSize)sprite->sprite->DSResolution, (SpriteColorFormat)sprite->sprite->format, sprite->sprite->gfx, affineId, true, false, sprite->flipX, sprite->flipY, false);
	}
	++oamCount;
}

void FinalizeSprites() {
#ifndef _NOTDS
	oamClear(&oamMain, 0, 0);
	oamClear(&oamSub, 0, 0);
	oamCount = 0;
#else
	ClearDepth();
#endif
	spriteMatrixId = 0;
	for (int i = 0; i < mainSpriteCallCount; ++i) {
		RenderSpriteInternal(&mainSpriteCalls[i]);
	}
	spriteMatrixId = 0;
#ifdef _NOTDS
	UseRenderTexture(subScreenTexture);
	if (BGTexture != NULL) {
		ClearColor();
		RenderBackground();
	}
	else {
		ClearColor();
	}
#else
	oamCount = 0;
#endif
	for (int i = 0; i < subSpriteCallCount; ++i) {
		RenderSpriteInternal(&subSpriteCalls[i]);
	}
#ifdef _NOTDS
	UseRenderTexture(NULL);
	RenderBottomScreen();
#endif
	mainSpriteCallCount = 0;
	subSpriteCallCount = 0;
#ifndef _NOTDS
	oamUpdate(&oamMain);
	oamUpdate(&oamSub);
#endif
}

void SetBackgroundTile(int x, int y, int id) {
#ifndef _NOTDS
	bgGetMapPtr(bgID)[x + y * 32] = id;
#else
	subBackground[x + y * 32] = id;
#endif
}

void SetupCameraMatrix() {
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	#ifdef FLIP_X
	m4x4 tmpMat;
	MakeScaleMatrix(-4096, 4096, 4096, &tmpMat);
	glMultMatrix4x4(&tmpMat);
	#endif
	gluPerspectivef32(cameraFOV, 5461, cameraNear, cameraFar);
	Vec3 cameraPositionMod;
	cameraPositionMod.x = cameraPosition.x.value % 4096;
	cameraPositionMod.y = cameraPosition.y.value % 4096;
	cameraPositionMod.z = cameraPosition.z.value % 4096;
	cameraRecentering.x = cameraPosition.x - cameraPositionMod.x;
	cameraRecentering.y = cameraPosition.y - cameraPositionMod.y;
	cameraRecentering.z = cameraPosition.z - cameraPositionMod.z;
	Vec3 camPosInverse;
	camPosInverse.x = -cameraPositionMod.x;
	camPosInverse.y = -cameraPositionMod.y;
	camPosInverse.z = -cameraPositionMod.z;
	Mat4x4 camTransform = Mat4x4::Translation(camPosInverse);
	// rotation
	Quaternion inverseCamRot = cameraRotation.Inverse();
	Mat4x4 camRotation = Mat4x4::Rotation(inverseCamRot);
	cameraMatrix = camRotation.Multiply4x3(camTransform);
	//m4x4 trueCameraMatrix;
	//MatrixToDSMatrix(&cameraMatrix, &trueCameraMatrix);
	glMultMatrix4x4((m4x4*)&cameraMatrix);

	// set viewport to be size of screen
	glViewport(0, 0, 255, 191);
}

void SetupCameraMatrixPartial(int x, int y, int width, int height) {
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	Mat4x4 screenAdjustMatrix;
	// offset target...note: don't need to multiply by 4096 for divf32! it'll convert to base-4096 post division for us, due to...well, 0.1/0.1 = 1.0
	int oneMinusWidth = (4096 - divf32f(width, 256)) * 2;
	int oneMinusHeight = (4096 - divf32f(height, 192)) * 2;
	screenAdjustMatrix.m[12] = Lerp(oneMinusWidth, -oneMinusWidth, divf32f(x, 256 - width));
	screenAdjustMatrix.m[13] = Lerp(oneMinusHeight, -oneMinusHeight, divf32f(y, 192 - height));
	screenAdjustMatrix.m[0] = 4096;
	screenAdjustMatrix.m[5] = 4096;
	screenAdjustMatrix.m[10] = 4096;
	screenAdjustMatrix.m[15] = 4096;
	glMultMatrix4x4((m4x4*)&screenAdjustMatrix);
#ifdef FLIP_X
	m4x4 tmpMat;
	MakeScaleMatrix(-4096, 4096, 4096, &tmpMat);
	glMultMatrix4x4(&tmpMat);
#endif
	gluPerspectivef32(cameraFOV, divf32f(width, height), cameraNear, cameraFar);
	Vec3 cameraPositionMod;
	cameraPositionMod.x = cameraPosition.x.value % 4096;
	cameraPositionMod.y = cameraPosition.y.value % 4096;
	cameraPositionMod.z = cameraPosition.z.value % 4096;
	cameraRecentering.x = cameraPosition.x - cameraPositionMod.x;
	cameraRecentering.y = cameraPosition.y - cameraPositionMod.y;
	cameraRecentering.z = cameraPosition.z - cameraPositionMod.z;
	Vec3 camPosInverse;
	camPosInverse.x = -cameraPositionMod.x;
	camPosInverse.y = -cameraPositionMod.y;
	camPosInverse.z = -cameraPositionMod.z;
	Mat4x4 camTransform = Mat4x4::Translation(camPosInverse);
	// rotation
	Mat4x4 camRotation = Mat4x4::Rotation(cameraRotation.Inverse());
	cameraMatrix = camRotation.Multiply4x3(camTransform);
	glMultMatrix4x4((m4x4*)&cameraMatrix);
	if (x == 128) {
		x = 0;
	}
	else {
		x = 128;
	}
	glViewport(x, y, (x+width)-1, (y+height)-1);
}

bool AABBInCamera(Vec3* min, Vec3* max, Mat4x4* transform) {
	glMatrixMode(GL_MODELVIEW);
	glLoadMatrix4x4((m4x4*)transform);
	return BoxTest(min->x, min->y, min->z, max->x - min->x, max->y - min->y, max->z - min->z);
}

bool QueueAnimation(Animator* animator, Animation* animation, Fixed lerpTime) {
	if (animator->queuedAnimCount >= 8) return false;
	animator->queuedAnims[animator->queuedAnimCount] = animation;
	animator->queuedLerpTimes[animator->queuedAnimCount] = lerpTime;
	++animator->queuedAnimCount;
	return true;
}

void DestroyAnimator(Animator* animator) {
	free(animator->items);
	free(animator);
}

void Set3DOnTop() {
	touch3D = false;
#ifndef _NOTDS
	lcdMainOnTop();
#endif
}

void Set3DOnBottom() {
	touch3D = true;
#ifndef _NOTDS
	lcdMainOnBottom();
#endif
}

unsigned short* frameBuffer1;
unsigned short* frameBuffer2;
int frameBufferToRead = 0;

void DisplayIRQ() {
	unsigned short* readBuffer = frameBuffer1;
	if (frameBufferToRead == 1) {
		readBuffer = frameBuffer2;
	}
	REG_DMAxCNT(1) = 0;
	REG_DMAxSAD(1) = (unsigned int)&readBuffer[0];
	REG_DMAxDAD(1) = (unsigned int)BG_GFX;
	REG_DMAxCNT(1) = (128) | ((DMA_MODE_DST(DmaMode_IncrReload) | DMA_MODE_SRC(DmaMode_Increment) | DMA_UNIT_32 | DMA_TIMING(DmaTiming_Immediate) | DMA_START) << 16);

	dmaBusyWait(1);
	REG_DMAxSAD(1) = (unsigned int)&readBuffer[256];
	REG_DMAxDAD(1) = (unsigned int)BG_GFX; // BG memory
	REG_DMAxCNT(1) = (128) | ((DMA_MODE_DST(DmaMode_IncrReload) | DMA_MODE_SRC(DmaMode_Increment) | DMA_UNIT_32 | DMA_TIMING(DmaTiming_HBlank) | DMA_START | DMA_MODE_REPEAT) << 16);
	return;
}

void Initialize3D(bool multipass, bool subBGFull) {
	// initialize gl engine
	glInit();

	videoSetMode(MODE_0_3D);
	videoSetModeSub(MODE_0_2D);

	// AA because why not
	//glEnable(GL_ANTIALIAS);

	glEnable(GL_TEXTURE_2D);

	glEnable(GL_BLEND);
	glEnable(GL_ALPHA_TEST);

	vramSetBankA(VRAM_A_TEXTURE);
	vramSetBankB(VRAM_B_TEXTURE);
	if (subBGFull) {
		vramSetBankC(VRAM_C_SUB_BG);
	}
	else {
		vramSetBankC(VRAM_C_TEXTURE);
		vramSetBankH(VRAM_H_SUB_BG);
	}

	vramSetBankE(VRAM_E_MAIN_SPRITE);
	vramSetBankI(VRAM_I_SUB_SPRITE);

	if (multipass) {
		vramSetBankD(VRAM_D_LCD);
		vramSetBankF(VRAM_F_MAIN_BG_0x06000000);
		vramSetBankG(VRAM_G_TEX_PALETTE_SLOT0);
		videoSetMode(MODE_5_3D);
		int bgId = bgInit(3, BgType_Bmp16, BgSize_B16_256x256, 0, 0);
		int bgId2 = bgInit(2, BgType_Bmp16, BgSize_B16_256x256, 0, 0);
		bgSetScale(bgId,256,0);
		bgSetScale(bgId2, 256, 0);
		lcdSetVBlankIrq(true);
		irqSet(IRQ_VBLANK, (IrqHandler)DisplayIRQ);
		irqEnable(IRQ_VBLANK);
		frameBuffer1 = new unsigned short[256*192];
		frameBuffer2 = new unsigned short[256*192];
		bgSetPriority(3, 0);
		bgSetPriority(2, 1);
		bgSetPriority(0, 3);
		// TODO: implement below for HDR
		//REG_BLDCNT = (1 << 3) | (1 << 10) | (1 << 2) | (1 << 11) | (1 << 6);
		//REG_BLDALPHA = (16 << 0) | (16 << 8);
	} else {
		vramSetBankF(VRAM_F_TEX_PALETTE_SLOT0);
		vramSetBankG(VRAM_G_TEX_PALETTE_SLOT5);
	}

	glMaterialShinyness();
	oamInit(&oamMain, SpriteMapping_Bmp_1D_128, false);
	oamInit(&oamSub, SpriteMapping_Bmp_1D_128, false);

	multipassRendering = multipass;
}

void SetMaterialLightOverride(SDMaterial* material, int id, char R, char G, char B, Fixed normalX, Fixed normalY, Fixed normalZ) {
	unsigned short lightColor = RGB15(R, G, B);
	unsigned short lightNormal = ((normalX.value / 273) & 0x1F) | (((normalY.value / 273) & 0x1F) << 5) | (((normalZ.value / 273) & 0x1F) << 10);
	switch (id) {
	case 0:
		material->lightOverride0 = lightColor;
		material->lightNormal0 = lightNormal;
		break;
	case 1:
		material->lightOverride1 = lightColor;
		material->lightNormal1 = lightNormal;
		break;
	case 2:
		material->lightOverride2 = lightColor;
		material->lightNormal2Pt0 = lightNormal & 0xFF;
		material->lightNormal2Pt1 = (lightNormal >> 8) & 0xFF;
		break;
	case 3:
		material->lightOverride3 = lightColor;
		material->lightNormal3Pt0 = lightNormal & 0xFF;
		material->lightNormal3Pt1 = (lightNormal >> 8) & 0xFF;
		break;
	}
}

void RenderModelQueue(bool flush) {
	for (int i = 0; i < modelRenderQueueCount; ++i) {
		QueueRenderModel* currRender = &modelRenderQueue[i];
		if (currRender->animator != NULL) {
			RenderModelRigged(currRender->model, &currRender->position, &currRender->scale, &currRender->rotation, currRender->materials, currRender->animator, currRender->renderPriority);
		}
		else {
			RenderModel(currRender->model, &currRender->position, &currRender->scale, &currRender->rotation, currRender->materials, currRender->renderPriority);
		}
	}
	if (flush) {
		modelRenderQueueCount = 0;
	}
}

// functions only used for debugging multipass...
void SaveLCD() {
#ifndef _NOTDS
	// dma copying from the LCD storage to our temporary texture for multipass
	dmaBusyWait(1);
	//REG_DMAxCNT_H(2) = 0; // disable the DMA...
	//dmaCopy(VRAM_D, storageTexture, sizeof(unsigned short) * 256 * 192);
	//dmaCopyWordsAsynch(1, VRAM_D, storageTexture, sizeof(unsigned short) * 256 * 192);
	dmaBusyWait(1);
	//DC_FlushRange(storageTexture, sizeof(unsigned short) * 256 * 192);
#endif
}

/*void RestoreLCD() {
	//dmaBusyWait(1);
	//dmaBusyWait(2);
	// now we have to DMA copy to the screen! no built in function gives us enough control, so write the registers ourselves...
	REG_DMAxSAD(2) = (unsigned int)storageTexture;
	REG_DMAxDAD(2) = 0x04000068;
	REG_DMAxCNT_L(2) = 4; // copy 8 pixels per copy; 4 int32s
	REG_DMAxCNT_H(2) = DMA_MODE_DST(DmaMode_Fixed) | DMA_MODE_SRC(DmaMode_Increment) | DMA_UNIT_32 | DMA_TIMING(DmaTiming_MemDisp) | DMA_START | DMA_MODE_REPEAT; // has to be set to repeat so it continues outputting it
}*/