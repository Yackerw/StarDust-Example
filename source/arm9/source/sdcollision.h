#pragma once
#include <nds.h>
#include "sdmath.h"
#include "sdrender.h"

struct BasicCollisionInfo {
	union {
		Fixed t;
		Fixed penetration;
	};
	Vec3 position;
	Vec3 normal;
};

struct CollisionTriangle {
	Vec3s verts[3];
	Vec3s normal;
	Vec3s boundsMin;
	Vec3s boundsMax;
};

class CollisionBlock {
public:
	bool subdivided;
	unsigned short triCount;
	Vec3s boundsMin;
	Vec3s boundsMax;
	union {
		unsigned short *triangleList;
		CollisionBlock *blocks;
	};

	int GetQuadTreeCount() const;
};

class MeshCollider {
private:
	void FindTrianglesFromOctreeInternal(const Vec3& min, const Vec3& max, CollisionBlock* block, unsigned short** retValue, int* maxSize, int* currSize);
public:
	int triCount;
	CollisionTriangle* triangles;
	Vec3 AABBPosition;
	Vec3 AABBBounds;
	// blockmap sorta
	CollisionBlock blocks[8];

	unsigned short* FindTrianglesFromOctree(const Vec3& min, const Vec3& max, int *totalTris);
	static void ReleaseTriangleOctreeAllocation(unsigned short* tris);
	int GetQuadTreeCount() const;

	~MeshCollider();
};

class BoxCollider {
public:
	Vec3* position;
	Vec3 extents;
	Quaternion* rotation;

	BoxCollider() {
		position = NULL;
		extents = Vec3(0,0,0);
		rotation = NULL;
	}

	BoxCollider(Vec3* position, const Vec3& extents, Quaternion* rotation) {
		this->position = position;
		this->extents = extents;
		this->rotation = rotation;
	}
};

class SphereCollider {
private:
	bool OnLine(const Vec3& p1, const Vec3& p2, Vec3* closestPoint);
	Fixed OnPoint(Fixed sphereSquareMagnitude, const Vec3& point);
public:
	enum TriIntersectStatus {NO_COLLIDE, YES_COLLIDE, ON_PLANE};

	Vec3* position;
	Fixed radius;

	SphereCollider() {
		position = NULL;
		radius = 0;
	}

	SphereCollider(Vec3* position, Fixed radius) {
		this->position = position;
		this->radius = radius;
	}

	bool OnTriangleLines(const CollisionTriangle& tri, BasicCollisionInfo* infoOut);
	bool OnTriangleVertices(const CollisionTriangle& tri, BasicCollisionInfo* infoOut);
	TriIntersectStatus OnTrianglePlane(const CollisionTriangle& tri, BasicCollisionInfo* infoOut);
	bool OnSphere(const SphereCollider& other, BasicCollisionInfo* infoOut);
	bool OnOBB(const BoxCollider& box, BasicCollisionInfo* infoOut);
};

class RayCast {
private:
	bool OnPlane(const Vec3& normal, const Fixed planeDistance, Fixed *t, Vec3* hitPos);
	void OnQuadTreeSub(const Vec3& AABBMin, const Vec3& AABBMax, const CollisionBlock* block, const CollisionBlock** hitBlocks, int* hitBlockPosition, int* triCount);
	void OnQuadTree(const Vec3& AABBMin, const Vec3& AABBMax, const MeshCollider& mesh, CollisionBlock** hitBlocks, int* hitBlockPosition, int* triCount);
public:
	Vec3 point;
	Vec3 dir;
	Fixed length;
	RayCast(const Vec3& point, const Vec3& dir, const Fixed length) {
		this->point = point;
		this->dir = dir;
		this->length = length;
	}

	bool OnAABB(const Vec3& boxMin, const Vec3& boxMax, BasicCollisionInfo* infoOut);
	bool OnSphere(const SphereCollider& sphere, BasicCollisionInfo* infoOut);
	bool OnTriangle(const CollisionTriangle& triangle, BasicCollisionInfo* infoOut);
	// TODO: maybe incorporate matrix transforms into this instead of the transforms
	int OnMesh(const MeshCollider& mesh, const Vec3& meshOffset, const Vec3& meshScale, const Quaternion& meshRotation, BasicCollisionInfo* infoOut);
};

MeshCollider *LoadCollisionMesh(char *input);

MeshCollider *MeshColliderFromMesh(Model *input);