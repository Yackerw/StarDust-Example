#ifndef PLAYER
#define PLAYER
#include "sdobject.h"

typedef struct {
	Vec3 normal;
	f32 cameraAngle;
	bool onGround;
	f32 vSpeed;
} PlayerValues;

void PlayerStart(Object* obj);

void PlayerUpdate(Object* obj);

void PlayerLateUpdate(Object* obj);

void PlayerDestroy(Object* obj);

bool PlayerCollide(Object* obj, CollisionHit* hitInfo);
#endif