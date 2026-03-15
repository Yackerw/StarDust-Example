#include "sddelta.h"
#include "player.h"
#include "sdsound.h"
#include "sdinput.h"

Animation* idleAnim, * walkAnim, * jumpUpAnim, * jumpDownAnim;
SoundEffect* jumpSound;
Model* testModel;

Player::Player() {
	// mesh is the mesh to be rendered by this object
	mesh = LoadModel("nitro:/sonic/Sonic.sdm");
	testModel = LoadModel("nitro:/testcube0.sdm");
	testModel->defaultMats[0].stencilPack = 0;
	testModel->defaultMats[0].alpha = 31;
	// animator is the animator for the objects mesh
	animator = CreateAnimator(mesh);
	idleAnim = LoadAnimation("nitro:/sonic/Stand.sda");
	walkAnim = LoadAnimation("nitro:/sonic/Walk.sda");
	jumpUpAnim = LoadAnimation("nitro:/sonic/Springing.sda");
	jumpDownAnim = LoadAnimation("nitro:/sonic/Falling.sda");
	// spherecol gives us a sphere collider for our object to collide with the world with
	sphereCol = new SphereCollider(&this->position, 2048);
	sphereCol->position = &position;
	sphereCol->radius = 2048;
	// layer indicates the collision layer it's on, for now we'll just place the player on layer 2 and the world on layer 1 for organizational purposes
	layer = 2;
	// scale impacts mesh rendering size and mesh collider size. it does NOT impact other colliders
	scale.x = 250;
	scale.y = 250;
	scale.z = 250;
	// solid is necessary for collision checks to be ran, moves is necessary for it to be able to conduct collision checks while moving
	solid = true;
	moves = true;
	jumpSound = LoadWav("nitro:/sfx/jump.wav");
	PlayAnimation(animator, idleAnim, 0);

	// LIGHTING OVERRIDE!
	for (int i = 0; i < mesh->materialCount; ++i) {
		SetMaterialLightOverride(&mesh->defaultMats[i], 2, 0x1F, 0, 0, 4096, 0, 0);
		mesh->defaultMats[i].lightingFlags |= LIGHT_OVERRIDE2;
	}

	normal = Vec3(0,0,0);
	cameraAngle = 0;
	onGround = false;
	vSpeed = 0;
	position.y = 4096*5;
}

void Player::Update() {
	Vec3 moveAxis;
	moveAxis.x = GetKey(INPUT_RIGHT) * 4096;
	moveAxis.x += GetKey(INPUT_LEFT) * -4096;
	moveAxis.z = GetKey(INPUT_UP) * -4096;
	moveAxis.z += GetKey(INPUT_DOWN) * 4096;
	moveAxis.y = 0;
	if (moveAxis.x != 0 || moveAxis.z != 0) {
		moveAxis = moveAxis.Normalize();
	}
	
	Quaternion tmpQuat = Quaternion::FromEuler(0,cameraAngle,0);
	Vec3 tmpMoveAxis = tmpQuat * moveAxis;

	if (GetKeyDown(INPUT_A) && onGround) {
		onGround = false;
		vSpeed = 14 * 4096;

		SoundData *sd = (SoundData*)malloc(sizeof(SoundData));
		sd->volume = 2048;
		sd->loop = false;
		// 0 for left, 0.5 for middle, 1 for right
		sd->pan = 2048;
		sd->pitch = 4096;
		sd->sound = jumpSound;
		PlaySound(sd);
		free(sd);
	}
	
	Vec3 down = { 0,-4096,0 };
	CollisionHit hitInfo;
	RayCast ray(position, down, 4096*5);
	bool hit = RaycastWorld(ray, 1, &hitInfo);
	//printf("%i %i %i\n", position.x, position.y, position.z);
	//printf("%i %i %i\n", hitInfo.hitPos.x, hitInfo.hitPos.y, hitInfo.hitPos.z);
	Vec3 one = { 1024,1024,1024 };
	RenderModel(testModel, &hitInfo.position, &one, &rotation, NULL, 0);


	// set speed
	velocity.x = mulf32(tmpMoveAxis.x, 4096 * 7);
	velocity.z = mulf32(tmpMoveAxis.z, 4096 * 7);
	if (onGround) {
		velocity.y = 0;
		Vec3 newDown = { mulf32(-normal.x.value, 4096 * 7), mulf32(-normal.y.value, 4096 * 7), mulf32(-normal.z.value, 4096 * 7) };
		velocity = newDown + velocity;
		vSpeed = 0;
	}
	else {
		vSpeed = Max(vSpeed.value - mulf32(4096 * 14, SDTime::deltaTime.value), -20 * 4096);
		velocity.y = vSpeed;
	}

	// animate
	Fixed magnitude = tmpMoveAxis.Magnitude();
	if (magnitude > 0) {
		rotation = Quaternion::FromEuler(0, Atan2(tmpMoveAxis.x, tmpMoveAxis.z), 0);
	}
	if (onGround) {
		if (magnitude == 0 && animator->currAnimation != idleAnim) {
			// animator lerp time is in frames, assuming a 60 fps game. IT IS DELTA TIMED!! 60 lerp time will always equal 1 second of lerp time
			PlayAnimation(animator, idleAnim, 7 * 4096);
		}
		if (magnitude != 0 && animator->currAnimation != walkAnim) {
			PlayAnimation(animator, walkAnim, 7 * 4096);
		}

	}
	else {
		if (vSpeed > 0 && animator->currAnimation != jumpUpAnim) {
			PlayAnimation(animator, jumpUpAnim, 7 * 4096);
		}
		if (vSpeed < 0 && animator->currAnimation != jumpDownAnim) {
			PlayAnimation(animator, jumpDownAnim, 7 * 4096);
		}
	}

	onGround = false;
}

void Player::LateUpdate() {
	// update camera to follow us here
	if (GetKey(INPUT_R)) {
		// FixedDegreesToRotation can be multiplied by to convert from 0-360*4096 degree fixed point to 0-32767 fixed point angles used by trigonometric functions
		cameraAngle -= mulf32(mulf32(4096 * 110, SDTime::deltaTime), FixedDegreesToRotation);
	}
	else if (GetKey(INPUT_L)) {
		cameraAngle += mulf32(mulf32(4096 * 110, SDTime::deltaTime), FixedDegreesToRotation);
	}
	cameraAngle = cameraAngle.value % mulf32(4096 * 360, FixedDegreesToRotation);

	// cameraRotation is the rotation of the camera, cameraPosition is the cameras position
	cameraRotation = Quaternion::FromEuler(0, cameraAngle, 0);
	Vec3 back = { 0, 1 * 4096, 3 * 4096 };
	cameraPosition = (cameraRotation * back) + position;
}

Player::~Player() {
	DestroyAnimator(animator);
	DestroyModel(mesh);
	delete sphereCol;
}

bool Player::Collide(const CollisionHit& hitInfo) {
	Vec3 up = { 0, 4096, 0 };
	if (up.Dot(hitInfo.normal) >= 2048 && vSpeed <= 0) {
		normal = hitInfo.normal;
		onGround = true;
	}
	return true;
}