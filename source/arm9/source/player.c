#include <nds.h>
#include "sddelta.h"
#include "player.h"
#include "sdsound.h"

Animation* idleAnim, * walkAnim, * jumpUpAnim, * jumpDownAnim;
SoundEffect* jumpSound;

void PlayerStart(Object* obj) {
	// obj->data is a free slot to place a pointer to our own values for the object
	obj->data = calloc(sizeof(PlayerValues), 1);
	// obj->mesh is the mesh to be rendered by this object
	obj->mesh = LoadModel("nitro:/sonic/Sonic.sdm");
	// obj->animator is the animator for the objects mesh
	obj->animator = CreateAnimator(obj->mesh);
	idleAnim = LoadAnimation("nitro:/sonic/Stand.sda");
	walkAnim = LoadAnimation("nitro:/sonic/Walk.sda");
	jumpUpAnim = LoadAnimation("nitro:/sonic/Springing.sda");
	jumpDownAnim = LoadAnimation("nitro:/sonic/Falling.sda");
	// spherecol gives us a sphere collider for our object to collide with the world with
	obj->sphereCol = calloc(sizeof(CollisionSphere), 1);
	obj->sphereCol->position = &obj->position;
	obj->sphereCol->radius = Fixed32ToNative(2048);
	// layer indicates the collision layer it's on, for now we'll just place the player on layer 2 and the world on layer 1 for organizational purposes
	obj->layer = 2;
	// scale impacts mesh rendering size and mesh collider size. it does NOT impact other colliders
	obj->scale.x = Fixed32ToNative(250);
	obj->scale.y = Fixed32ToNative(250);
	obj->scale.z = Fixed32ToNative(250);
	// solid is necessary for collision checks to be ran, moves is necessary for it to be able to conduct collision checks while moving
	obj->solid = true;
	obj->moves = true;
	jumpSound = LoadWav("nitro:/sfx/jump.wav");
	PlayAnimation(obj->animator, idleAnim, 0);
}

void PlayerUpdate(Object* obj) {
	PlayerValues* pv = obj->data;
	u16 keys = keysHeld();
	Vec3 moveAxis;
	moveAxis.x = ((keys & KEY_RIGHT) != 0) * Fixed32ToNative(4096);
	moveAxis.x += ((keys & KEY_LEFT) != 0) * -Fixed32ToNative(4096);
	moveAxis.z = ((keys & KEY_UP) != 0) * -Fixed32ToNative(4096);
	moveAxis.z += ((keys & KEY_DOWN) != 0) * Fixed32ToNative(4096);
	moveAxis.y = 0;
	if (moveAxis.x != 0 || moveAxis.z != 0) {
		Normalize(&moveAxis, &moveAxis);
	}
	Quaternion tmpQuat;
	EulerToQuat(0, pv->cameraAngle, 0, &tmpQuat);
	Vec3 tmpMoveAxis;
	QuatTimesVec3(&tmpQuat, &moveAxis, &tmpMoveAxis);

	if (keysDown() & KEY_A && pv->onGround) {
		pv->onGround = false;
		pv->vSpeed = Fixed32ToNative(14 * 4096);

		SoundData *sd = malloc(sizeof(SoundData));
		sd->volume = Fixed32ToNative(2048);
		sd->loop = false;
		// 0 for left, 0.5 for middle, 1 for right
		sd->pan = Fixed32ToNative(2048);
		sd->pitch = Fixed32ToNative(4096);
		sd->sound = jumpSound;
		PlaySound(sd);
		free(sd);
	}
	
	// set speed
	obj->velocity.x = mulf32(tmpMoveAxis.x, Fixed32ToNative(4096 * 7));
	obj->velocity.z = mulf32(tmpMoveAxis.z, Fixed32ToNative(4096 * 7));
	if (pv->onGround) {
		obj->velocity.y = 0;
		Vec3 newDown = { mulf32(-pv->normal.x, Fixed32ToNative(4096 * 7)), mulf32(-pv->normal.y, Fixed32ToNative(4096 * 7)), mulf32(-pv->normal.z, Fixed32ToNative(4096 * 7)) };
		Vec3Addition(&newDown, &obj->velocity, &obj->velocity);;
		pv->vSpeed = 0;
	}
	else {
		pv->vSpeed = Max(pv->vSpeed - mulf32(Fixed32ToNative(4096 * 14), deltaTime), -Fixed32ToNative(20 * 4096));
		obj->velocity.y = pv->vSpeed;
	}

	// animate
	f32 magnitude = Magnitude(&tmpMoveAxis);
	if (magnitude > 0) {
		EulerToQuat(0, Atan2(tmpMoveAxis.x, tmpMoveAxis.z), 0, &obj->rotation);
	}
	if (pv->onGround) {
		if (magnitude == 0 && obj->animator->currAnimation != idleAnim) {
			// animator lerp time is in frames, assuming a 60 fps game. IT IS DELTA TIMED!! 60 lerp time will always equal 1 second of lerp time
			PlayAnimation(obj->animator, idleAnim, Fixed32ToNative(7 * 4096));
		}
		if (magnitude != 0 && obj->animator->currAnimation != walkAnim) {
			PlayAnimation(obj->animator, walkAnim, Fixed32ToNative(7 * 4096));
		}

	}
	else {
		if (pv->vSpeed > 0 && obj->animator->currAnimation != jumpUpAnim) {
			PlayAnimation(obj->animator, jumpUpAnim, Fixed32ToNative(7 * 4096));
		}
		if (pv->vSpeed < 0 && obj->animator->currAnimation != jumpDownAnim) {
			PlayAnimation(obj->animator, jumpDownAnim, Fixed32ToNative(7 * 4096));
		}
	}

	pv->onGround = false;
}

void PlayerLateUpdate(Object* obj) {
	PlayerValues* pv = obj->data;
	// update camera to follow us here
	u16 keys = keysHeld();
	if (keys & KEY_R) {
		// FixedDegreesToRotation can be multiplied by to convert from 0-360*4096 degree fixed point to 0-32767 fixed point angles used by trigonometric functions
		pv->cameraAngle -= mulf32(mulf32(Fixed32ToNative(4096 * 110), deltaTime), FixedDegreesToRotation);
	}
	else if (keys & KEY_L) {
		pv->cameraAngle += mulf32(mulf32(Fixed32ToNative(4096 * 110), deltaTime), FixedDegreesToRotation);
	}
	pv->cameraAngle = f32Mod(pv->cameraAngle, mulf32(Fixed32ToNative(4096 * 360), FixedDegreesToRotation));

	// cameraRotation is the rotation of the camera, cameraPosition is the cameras position
	EulerToQuat(0, pv->cameraAngle, 0, &cameraRotation);
	Vec3 back = { 0, Fixed32ToNative(1 * 4096), Fixed32ToNative(3 * 4096) };
	QuatTimesVec3(&cameraRotation, &back, &cameraPosition);
	Vec3Addition(&cameraPosition, &obj->position, &cameraPosition);
}

void PlayerDestroy(Object* obj) {
	free(obj->data);
	DestroyAnimator(obj->animator);
	DestroyModel(obj->mesh);
	free(obj->sphereCol);
}

bool PlayerCollide(Object* obj, CollisionHit* hitInfo) {
	PlayerValues* pv = obj->data;
	Vec3 up = { 0, Fixed32ToNative(4096), 0 };
	if (DotProduct(&up, &hitInfo->normal) >= Fixed32ToNative(2048) && pv->vSpeed <= 0) {
		pv->normal = hitInfo->normal;
		pv->onGround = true;
	}
	return true;
}