#include "sddelta.h"
#include "player.h"
#include "sdsound.h"
#include "sdinput.h"

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
	obj->sphereCol->radius = 2048;
	// layer indicates the collision layer it's on, for now we'll just place the player on layer 2 and the world on layer 1 for organizational purposes
	obj->layer = 2;
	// scale impacts mesh rendering size and mesh collider size. it does NOT impact other colliders
	obj->scale.x = 250;
	obj->scale.y = 250;
	obj->scale.z = 250;
	// solid is necessary for collision checks to be ran, moves is necessary for it to be able to conduct collision checks while moving
	obj->solid = true;
	obj->moves = true;
	jumpSound = LoadWav("nitro:/sfx/jump.wav");
	PlayAnimation(obj->animator, idleAnim, 0);

	// LIGHTING OVERRIDE!
	for (int i = 0; i < obj->mesh->materialCount; ++i) {
		SetMaterialLightOverride(&obj->mesh->defaultMats[i], 1, 0x1F, 0, 0, 4096, 0, 0);
		obj->mesh->defaultMats[i].lightingFlags |= LIGHT_OVERRIDE1;
	}
}

void PlayerUpdate(Object* obj) {
	PlayerValues* pv = obj->data;
	Vec3 moveAxis;
	moveAxis.x = GetKey(INPUT_RIGHT) * 4096;
	moveAxis.x += GetKey(INPUT_LEFT) * -4096;
	moveAxis.z = GetKey(INPUT_UP) * -4096;
	moveAxis.z += GetKey(INPUT_DOWN) * 4096;
	moveAxis.y = 0;
	if (moveAxis.x != 0 || moveAxis.z != 0) {
		Normalize(&moveAxis, &moveAxis);
	}
	Quaternion tmpQuat;
	EulerToQuat(0, pv->cameraAngle, 0, &tmpQuat);
	Vec3 tmpMoveAxis;
	QuatTimesVec3(&tmpQuat, &moveAxis, &tmpMoveAxis);

	if (GetKeyDown(INPUT_A) && pv->onGround) {
		pv->onGround = false;
		pv->vSpeed = 14 * 4096;

		SoundData *sd = malloc(sizeof(SoundData));
		sd->volume = 2048;
		sd->loop = false;
		// 0 for left, 0.5 for middle, 1 for right
		sd->pan = 2048;
		sd->pitch = 4096;
		sd->sound = jumpSound;
		PlaySound(sd);
		free(sd);
	}
	
	// set speed
	obj->velocity.x = mulf32(tmpMoveAxis.x, 4096 * 7);
	obj->velocity.z = mulf32(tmpMoveAxis.z, 4096 * 7);
	if (pv->onGround) {
		obj->velocity.y = 0;
		Vec3 newDown = { mulf32(-pv->normal.x, 4096 * 7), mulf32(-pv->normal.y, 4096 * 7), mulf32(-pv->normal.z, 4096 * 7) };
		Vec3Addition(&newDown, &obj->velocity, &obj->velocity);;
		pv->vSpeed = 0;
	}
	else {
		pv->vSpeed = Max(pv->vSpeed - mulf32(4096 * 14, deltaTime), -20 * 4096);
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
			PlayAnimation(obj->animator, idleAnim, 7 * 4096);
		}
		if (magnitude != 0 && obj->animator->currAnimation != walkAnim) {
			PlayAnimation(obj->animator, walkAnim, 7 * 4096);
		}

	}
	else {
		if (pv->vSpeed > 0 && obj->animator->currAnimation != jumpUpAnim) {
			PlayAnimation(obj->animator, jumpUpAnim, 7 * 4096);
		}
		if (pv->vSpeed < 0 && obj->animator->currAnimation != jumpDownAnim) {
			PlayAnimation(obj->animator, jumpDownAnim, 7 * 4096);
		}
	}

	pv->onGround = false;
}

void PlayerLateUpdate(Object* obj) {
	PlayerValues* pv = obj->data;
	// update camera to follow us here
	if (GetKey(INPUT_R)) {
		// FixedDegreesToRotation can be multiplied by to convert from 0-360*4096 degree fixed point to 0-32767 fixed point angles used by trigonometric functions
		pv->cameraAngle -= mulf32(mulf32(4096 * 110, deltaTime), FixedDegreesToRotation);
	}
	else if (GetKey(INPUT_L)) {
		pv->cameraAngle += mulf32(mulf32(4096 * 110, deltaTime), FixedDegreesToRotation);
	}
	pv->cameraAngle = f32Mod(pv->cameraAngle, mulf32(4096 * 360, FixedDegreesToRotation));

	// cameraRotation is the rotation of the camera, cameraPosition is the cameras position
	EulerToQuat(0, pv->cameraAngle, 0, &cameraRotation);
	Vec3 back = { 0, 1 * 4096, 3 * 4096 };
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
	Vec3 up = { 0, 4096, 0 };
	if (DotProduct(&up, &hitInfo->normal) >= 2048 && pv->vSpeed <= 0) {
		pv->normal = hitInfo->normal;
		pv->onGround = true;
	}
	return true;
}