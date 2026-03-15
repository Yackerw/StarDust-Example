#pragma once
#include <nds.h>
#include "sdmath.h"
#include "sdrender.h"
#include "sdcollision.h"

#define COLLIDER_SPHERE 1
#define COLLIDER_MESH 2
#define COLLIDER_BOX 3

struct ObjectPtr;

class Object;

class ObjectPtr {
private:
	Object* object;
	ObjectPtr* prev;
	ObjectPtr* next;
public:
	//friend Object::Object();
	//friend void Object::CleanupObjPtr();
	friend class Object;
	ObjectPtr() {
		object = NULL;
		prev = NULL;
		next = NULL;
	}
	ObjectPtr(Object* obj);
};

struct CollisionHit : BasicCollisionInfo {
	Object* hitObject;
	int hitTri;
	int colliderType;
};

class Object {
private:
	Object *previous;
	Object *next;
	int objectType;
	ObjectPtr *references;

	static Object* first;
	static void** objTypes;
	static int objTypeCount;
	static int objTypeAlloc;

	void CleanupObjPtr();

public:
	Vec3 position;
	Quaternion rotation;
	Vec3 scale;
	Vec3 velocity;
	Model *mesh;
	SphereCollider *sphereCol;
	MeshCollider *meshCol;
	BoxCollider* boxCol;
	unsigned int flags;
	bool solid;
	bool moves;
	bool culled;
	bool destroy;
	bool active;
	Animator *animator;
	unsigned int layer;
	int renderPriority;
	int netId;

	friend ObjectPtr::ObjectPtr(Object* obj);

	virtual void Update() {};
	virtual bool Collide(const CollisionHit& hit) {
		return true;
	};
	virtual void LateUpdate() {};

	Object() {
		if (first == NULL) {
			previous = NULL;
			next = NULL;
			first = this;
		} else {
			previous = NULL;
			next = first;
			first->previous = this;
			first = this;
		}
		mesh = NULL;
		sphereCol = NULL;
		meshCol = NULL;
		boxCol = NULL;
		animator = NULL;

		position = Vec3(0,0,0);
		scale = Vec3(4096,4096,4096);
		rotation = Quaternion(0,0,0,4096);
		layer = 1;
		active = true;
		references = NULL;
	};

	ObjectPtr GetObjPtr() {
		return ObjectPtr(this);
	}

	virtual ~Object() {
		if (first == this) {
			first = next;
		}
		if (next != NULL) {
			next->previous = previous;
		}
		if (previous != NULL) {
			previous->next = next;
		}

		CleanupObjPtr();
	};

	int GetObjectType() const {
		return objectType;
	}

	static void ProcessObjects();
	static bool RaycastWorld(RayCast& rayObject, unsigned int layerMask, CollisionHit* hitInfo);
	static int SphereCollisionCheck(SphereCollider *sphere, unsigned int layerMask, CollisionHit* hitInfos, int maxHit);
	static int GetObjectsOfType(int type, Object **out, int maxObjects);
};

void AddCollisionBetweenLayers(int layer1, int layer2);

void DestroyObject(Object *object);

void DestroyObjectImmediate(Object* object);