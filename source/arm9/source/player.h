
#pragma once
#include <nds.h>
#include "sdobject.h"


class Player : public Object {
private:
	Vec3 normal;
	Fixed cameraAngle;
	bool onGround;
	Fixed vSpeed;
public:
	Player();
	void Update();
	bool Collide(const CollisionHit& hitInfo);
	void LateUpdate();

	~Player();
};