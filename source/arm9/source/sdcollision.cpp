#include <stdio.h>
#include "sdcollision.h"
#include <nds.h>
#include "sdmath.h"
#include <stdint.h>
#define OCTREE_MAX_DEPTH 10
#define OCTREE_MAX_TRIS 40

#define GJK_LENIENCY 32
// note: barely noticeable slowdown from this, and fixes a bug with large polygons
#define FLOATBARY
//#define BARY64

#define FACE_INTERIOR -1
#define FACE_EDGE0 0
#define FACE_EDGE1 1
#define FACE_EDGE2 2
#define FACE_VERT0 3
#define FACE_VERT1 4
#define FACE_VERT2 5

unsigned short staticCollisionAllocation[512];

// these are the same, but they're different defines for clarity anyways
#define ShortToVec3(sh, ve) (ve).x = (sh).x; (ve).y = (sh).y; (ve).z = (sh).z
#define Vec3ToShort(ve, sh) (sh).x = (ve).x; (sh).y = (ve).y; (sh).z = (ve).z

ITCM_CODE float dotf(float x1, float y1, float z1, float x2, float y2, float z2) {
	return x1 * x2 + y1 * y2 + z1 * z2;
}

#if defined(BARY64_DEBUG)
// debug functions
long long dot64(Vec3* left, Vec3* right) {
	// note: i've decided to extend the precision for these by 12 bits for the barycentric calculations. it's still a net gain of 20 bits before overflow
	long long work = ((long long)(left->x * 4096.0f) * (long long)(right->x * 4096.0f));
	long long work2 = ((long long)(left->y * 4096.0f) * (long long)(right->y * 4096.0f));
	long long work3 = ((long long)(left->z * 4096.0f) * (long long)(right->z * 4096.0f));
	return work + work2 + work3;
}

long long mulf64(long long left, long long right) {
	return (left * right) >> 24;
}

long long divf64(long long left, long long right) {
	return (left << 12) / right;
}
#else
ITCM_CODE long long dot64(Vec3* left, Vec3* right) {
	// note: i've decided to extend the precision for these by 12 bits for the barycentric calculations. it's still a net gain of 20 bits before overflow
	long long work = ((long long)left->x * (long long)right->x);
	long long work2 = ((long long)left->y * (long long)right->y);
	long long work3 = ((long long)left->z * (long long)right->z);
	return work + work2 + work3;
}

ITCM_CODE long long mulf64(long long left, long long right) {
	return (left * right) >> 24;
}

ITCM_CODE long long divf64(long long left, long long right) {
#ifndef _NOTDS
	REG_DIVCNT = DIV_64_64;

	while (REG_DIVCNT & DIV_BUSY);

	// really hope this still works
	REG_DIV_NUMER = left << 12;
	REG_DIV_DENOM = right;

	while (REG_DIVCNT & DIV_BUSY);

	return (REG_DIV_RESULT);
#else
	return (left << 12) / right;
#endif
}
#endif

void Vec3Subtractions(const Vec3s& left, const Vec3s& right, Vec3* out) {
	out->x = left.x - right.x;
	out->y = left.y - right.y;
	out->z = left.z - right.z;
}

Vec3 BarycentricCoords(const CollisionTriangle& tri, const Vec3& point) {
	Vec3 v0, v1, v2;
	Vec3Subtractions(tri.verts[1], tri.verts[0], &v0);
	Vec3Subtractions(tri.verts[2], tri.verts[0], &v1);
	ShortToVec3(tri.verts[0], v2);
	v2 = point-v2;
	// FLOATBARY is...probably a big bottleneck. maybe.
	#ifdef FLOATBARY
	float v0x = f32tofloat(v0.x);
	float v0y = f32tofloat(v0.y);
	float v0z = f32tofloat(v0.z);
	float v1x = f32tofloat(v1.x);
	float v1y = f32tofloat(v1.y);
	float v1z = f32tofloat(v1.z);
	float v2x = f32tofloat(v2.x);
	float v2y = f32tofloat(v2.y);
	float v2z = f32tofloat(v2.z);
	float d00 = dotf(v0x, v0y, v0z, v0x, v0y, v0z);
	float d01 = dotf(v0x, v0y, v0z, v1x, v1y, v1z);
	float d11 = dotf(v1x, v1y, v1z, v1x, v1y, v1z);
	float d20 = dotf(v2x, v2y, v2z, v0x, v0y, v0z);
	float d21 = dotf(v2x, v2y, v2z, v1x, v1y, v1z);
	float denom = d00 * d11 - d01 * d01;
	Vec3 retValue;
	retValue.x = floattof32((d11 * d20 - d01 * d21) / denom);
	retValue.y = floattof32((d00 * d21 - d01 * d20) / denom);
	retValue.z = 4096 - retValue.x - retValue.y;
	return retValue;
	#else
#if defined(BARY64)
	long long d00 = dot64(&v0, &v0);
	long long d01 = dot64(&v0, &v1);
	long long d11 = dot64(&v1, &v1);
	long long d20 = dot64(&v2, &v0);
	long long d21 = dot64(&v2, &v1);
	long long denom = mulf64(d00, d11) - mulf64(d01, d01);
	Vec3 retValue;
	retValue.x = divf64(mulf64(d11, d20) - mulf64(d01, d21), denom);
	retValue.y = divf64(mulf64(d00, d21) - mulf64(d01, d20), denom);
	retValue.z = 4096 - retValue.x - retValue.y;
	return retValue;
#elif defined (BARY64_DEBUG)
	long long d00 = dot64(&v0, &v0);
	long long d01 = dot64(&v0, &v1);
	long long d11 = dot64(&v1, &v1);
	long long d20 = dot64(&v2, &v0);
	long long d21 = dot64(&v2, &v1);
	long long denom = mulf64(d00, d11) - mulf64(d01, d01);
	Vec3 retValue;
	retValue.x = divf64(mulf64(d11, d20) - mulf64(d01, d21), denom) / 4096.0f;
	retValue.y = divf64(mulf64(d00, d21) - mulf64(d01, d20), denom) / 4096.0f;;
	retValue.z = 4096 - retValue.x - retValue.y;
	return retValue;
#else
	Fixed d00 = DotProduct(&v0, &v0);
	Fixed d01 = DotProduct(&v0, &v1);
	Fixed d11 = DotProduct(&v1, &v1);
	Fixed d20 = DotProduct(&v2, &v0);
	Fixed d21 = DotProduct(&v2, &v1);
	Fixed denom = mulf32(d00, d11) - mulf32(d01, d01);
	Vec3 retValue;
	retValue.x = divf32f(mulf32(d11, d20) - mulf32(d01, d21), denom);
	retValue.y = divf32f(mulf32(d00, d21) - mulf32(d01, d20), denom);
	retValue.z = 4096 - retValue.x - retValue.y;
	
	return retValue;
#endif
	#endif
}

Vec3 BarycentricCoordsVectors(Vec3* a, Vec3* b, Vec3* c, Vec3* point) {
	Vec3 v0, v1, v2;
	v0 = *b-*a;
	v1 = *c-*a;
	v2 = *point-*a;
	// FLOATBARY is...probably a big bottleneck. maybe.
#ifdef FLOATBARY
	float v0x = f32tofloat(v0.x);
	float v0y = f32tofloat(v0.y);
	float v0z = f32tofloat(v0.z);
	float v1x = f32tofloat(v1.x);
	float v1y = f32tofloat(v1.y);
	float v1z = f32tofloat(v1.z);
	float v2x = f32tofloat(v2.x);
	float v2y = f32tofloat(v2.y);
	float v2z = f32tofloat(v2.z);
	float d00 = dotf(v0x, v0y, v0z, v0x, v0y, v0z);
	float d01 = dotf(v0x, v0y, v0z, v1x, v1y, v1z);
	float d11 = dotf(v1x, v1y, v1z, v1x, v1y, v1z);
	float d20 = dotf(v2x, v2y, v2z, v0x, v0y, v0z);
	float d21 = dotf(v2x, v2y, v2z, v1x, v1y, v1z);
	float denom = d00 * d11 - d01 * d01;
	Vec3 retValue;
	retValue.x = floattof32((d11 * d20 - d01 * d21) / denom);
	retValue.y = floattof32((d00 * d21 - d01 * d20) / denom);
	retValue.z = (4096 - retValue.x) - retValue.y;
	return retValue;
#else
#if defined(BARY64)
	long long d00 = dot64(&v0, &v0);
	long long d01 = dot64(&v0, &v1);
	long long d11 = dot64(&v1, &v1);
	long long d20 = dot64(&v2, &v0);
	long long d21 = dot64(&v2, &v1);
	long long denom = mulf64(d00, d11) - mulf64(d01, d01);
	Vec3 retValue;
	retValue.x = divf64(mulf64(d11, d20) - mulf64(d01, d21), denom);
	retValue.y = divf64(mulf64(d00, d21) - mulf64(d01, d20), denom);
	retValue.z = 4096 - retValue.x - retValue.y;
	return retValue;
#elif defined (BARY64_DEBUG)
	long long d00 = dot64(&v0, &v0);
	long long d01 = dot64(&v0, &v1);
	long long d11 = dot64(&v1, &v1);
	long long d20 = dot64(&v2, &v0);
	long long d21 = dot64(&v2, &v1);
	long long denom = mulf64(d00, d11) - mulf64(d01, d01);
	Vec3 retValue;
	retValue.x = divf64(mulf64(d11, d20) - mulf64(d01, d21), denom) / 4096.0f;
	retValue.y = divf64(mulf64(d00, d21) - mulf64(d01, d20), denom) / 4096.0f;;
	retValue.z = 4096 - retValue.x - retValue.y;
	return retValue;
#else
	Fixed d00 = DotProduct(&v0, &v0);
	Fixed d01 = DotProduct(&v0, &v1);
	Fixed d11 = DotProduct(&v1, &v1);
	Fixed d20 = DotProduct(&v2, &v0);
	Fixed d21 = DotProduct(&v2, &v1);
	Fixed denom = mulf32(d00, d11) - mulf32(d01, d01);
	Vec3 retValue;
	retValue.x = divf32f(mulf32(d11, d20) - mulf32(d01, d21), denom);
	retValue.y = divf32f(mulf32(d00, d21) - mulf32(d01, d20), denom);
	retValue.z = 4096 - retValue.x - retValue.y;

	return retValue;
#endif
#endif
}

Fixed SphereCollider::OnPoint(Fixed sphereSquareMagnitude, const Vec3& point) {
	Vec3 distance = *position - point;
	Fixed mag = distance.SqrMagnitude();
	if (mag < sphereSquareMagnitude) {
		return radius.value - sqrtf32f(mag);
	}
	return -1;
}

__attribute__((target("arm")))
bool SphereCollider::OnLine(const Vec3& p1, const Vec3& p2, Vec3* closestPoint) {
	Vec3 working;
	Vec3 working2;
	Vec3 working3;
	Fixed t;
	Fixed t2;
	working3 = p2 - p1;
	working2 = p1 - *position;
	t = working3.Dot(working2);
	t2 = working3.Dot(working3);
	t = divf32f(t, t2);
	
	working = working3 * t;
	*closestPoint  = p1-working;
	
	// if distance to closest point is greater than the spheres radius, no collision
	working = *position-*closestPoint;
	if (working.SqrMagnitude() >= radius*radius) {
		return false;
	}
	// okay, get magnitude between the points. if
	// the distance between the two points of the line and the closest point is equal to the distance between
	// the two points of the line, then it's on the line and we return true
	Fixed magLine = working3.Magnitude();
	working = p1-*closestPoint;
	Fixed magPoint1 = working.Magnitude();
	working = p2-*closestPoint;
	Fixed magPoint2 = working.Magnitude();
	magPoint1 += magPoint2;
	// add a little leniency for, uh...lack of precision
	if (magPoint1.value <= magLine.value + 16 && magPoint1.value >= magLine.value - 16) {
		return true;
	}
	
	return false;
}

bool SphereCollider::OnTriangleLines(const CollisionTriangle& tri, BasicCollisionInfo* infoOut) {
	// and finally, line collision
	for (int i = 0; i < 3; ++i) {
		Vec3 closestPoint;
		Vec3 v1;
		Vec3 v2;
		ShortToVec3(tri.verts[i], v1);
		ShortToVec3(tri.verts[(i + 1) % 3], v2);
		if (OnLine(v1, v2, &closestPoint)) {
			// get penetration
			Vec3 lineDiff = *position-closestPoint;
			infoOut->penetration = radius - lineDiff.Magnitude();
			// and finally the normal
			infoOut->normal = lineDiff.Normalize();
			
			return true;
		}
	}
	return false;
}

__attribute__((target("arm")))
bool SphereCollider::OnTriangleVertices(const CollisionTriangle& tri, BasicCollisionInfo* infoOut) {
	Fixed sphereSquareMagnitude = radius*radius;
	for (int i = 0; i < 3; ++i) {
		Fixed pointMag;
		Vec3 v;
		ShortToVec3(tri.verts[i], v);
		// TODO: cache sphere square radius so we aren't recalculating it every time. that 64 bit multiply isn't cheap!
		if (pointMag = OnPoint(sphereSquareMagnitude, v) > 0) {
			// get penetration and normal
			infoOut->penetration = pointMag;
			// normal; subtract the two vectors, then normalize
			infoOut->normal = (*position-v).Normalize();
			return true;
		}
	}
	return false;
}

__attribute__((target("arm")))
SphereCollider::TriIntersectStatus SphereCollider::OnTrianglePlane(const CollisionTriangle& tri, BasicCollisionInfo* infoOut) {
	// adjust sphere so triangle is origin
	Vec3 newSpherePosition;
	newSpherePosition.x = position->x.value - tri.verts[0].x;
	newSpherePosition.y = position->y.value - tri.verts[0].y;
	newSpherePosition.z = position->z.value - tri.verts[0].z;
	// finally, just apply a dot product
	Vec3 n;
	ShortToVec3(tri.normal, n);
	Fixed dot = newSpherePosition.Dot(n);
	// don't collide if we're beneath the polygon!
	if (dot >= radius || dot < 0) {
		return NO_COLLIDE;
	}
	infoOut->penetration = radius - dot;
	// get closest position on the plane
	Vec3 spotOnPlane;
	spotOnPlane.x = mulf32fast(dot, -tri.normal.x);
	spotOnPlane.y = mulf32fast(dot, -tri.normal.y);
	spotOnPlane.z = mulf32fast(dot, -tri.normal.z);
	spotOnPlane = spotOnPlane + *position;
	// now calculate the barycentric coordinates on the triangle
	Vec3 barycentric = BarycentricCoords(tri, spotOnPlane);
	// if any are < 0 or > 1, no collision here. if all arent, then yes collision. also add some leniency
	if (barycentric.x >= 0 && barycentric.x <= 4096
	&& barycentric.y >= 0 && barycentric.y <= 4096
	&& barycentric.z >= 0 && barycentric.z <= 4096) {
		infoOut->normal.x = tri.normal.x;
		infoOut->normal.y = tri.normal.y;
		infoOut->normal.z = tri.normal.z;
		return YES_COLLIDE;
	}
	return ON_PLANE;
}

MeshCollider *LoadCollisionMesh(char *input) {
	FILE *f = fopen(input, "rb");
	fseek(f, 0, SEEK_END);
	int fileSize = ftell(f);
	fseek(f, 0, SEEK_SET);
	MeshCollider *mesh = (MeshCollider*)malloc(fileSize);
	fread(mesh, fileSize, 1, f);
	fclose(f);
	mesh->triangles = (CollisionTriangle*)((uint32_t)mesh + (uint32_t)mesh->triangles);
	return mesh;
}

bool SphereCollider::OnSphere(const SphereCollider& other, BasicCollisionInfo* infoOut) {
	// simple, just get distance
	Vec3 sub = *position - *other.position;
	Fixed mag = sub.Magnitude();
	if (mag < radius + other.radius) {
		infoOut->normal = sub.Normalize();
		infoOut->penetration = (radius + other.radius) - mag;
		infoOut->position = *position + (infoOut->normal * (infoOut->penetration - radius));
		return true;
	}
	return false;
}

void GenerateBoundsForBlocks(Vec3s *min, Vec3s *max, CollisionBlock* blocks) {
	Vec3 middle;
	middle.x = (min->x + max->x) / 2;
	middle.y = (min->y + max->y) / 2;
	middle.z = (min->z + max->z) / 2;
	// generate each block w/ a loop
	for (int i = 0; i < 8; ++i) {
		if (i % 2 == 1) {
			// right
			blocks[i].boundsMin.x = middle.x;
			blocks[i].boundsMax.x = max->x;
		}
		else {
			// left
			blocks[i].boundsMin.x = min->x;
			blocks[i].boundsMax.x = middle.x;
		}
		if (i % 4 >= 2) {
			// front
			blocks[i].boundsMin.z = middle.z;
			blocks[i].boundsMax.z = max->z;
		}
		else {
			// back
			blocks[i].boundsMin.z = min->z;
			blocks[i].boundsMax.z = middle.z;
		}
		if (i >= 4) {
			// top
			blocks[i].boundsMin.y = middle.y;
			blocks[i].boundsMax.y = max->y;
		}
		else {
			// bottom
			blocks[i].boundsMin.y = min->y;
			blocks[i].boundsMax.y = middle.y;
		}
	}
}

bool AABBCheck(const Vec3& minA, const Vec3& maxA, const Vec3& minB, const Vec3& maxB) {
	return (minA.x <= maxB.x &&
		maxA.x >= minB.x &&
		minA.y <= maxB.y &&
		maxA.y >= minB.y &&
		minA.z <= maxB.z &&
		maxA.z >= minB.z);
}

ITCM_CODE bool AABBCheckLeniency(Vec3* minA, Vec3* maxA, Vec3* minB, Vec3* maxB, Fixed leniency) {
	return (minA->x <= maxB->x + leniency &&
		maxA->x >= minB->x - leniency &&
		minA->y <= maxB->y + leniency &&
		maxA->y >= minB->y - leniency &&
		minA->z <= maxB->z + leniency &&
		maxA->z >= minB->z - leniency);
}

ITCM_CODE bool AABBCheckLeniencyShort(Vec3s* minA, Vec3s* maxA, Vec3s* minB, Vec3s* maxB, Fixed leniency) {
	return (minA->x <= maxB->x + leniency &&
		maxA->x >= minB->x - leniency &&
		minA->y <= maxB->y + leniency &&
		maxA->y >= minB->y - leniency &&
		minA->z <= maxB->z + leniency &&
		maxA->z >= minB->z - leniency);
}

void GenerateOctree(CollisionBlock *currBlock, MeshCollider *currMesh, int currDepth) {
	++currDepth;
	currBlock->subdivided = false;
	currBlock->triCount = 0;
	int maxTris = OCTREE_MAX_TRIS;
	currBlock->triangleList = (unsigned short*)malloc(sizeof(unsigned short) * maxTris);
	// iterate over all the collision triangles in the mesh and see if they fall within the block
	for (int i = 0; i < currMesh->triCount; ++i) {
		// fixes occasionally broken faces on DS
		if (AABBCheckLeniencyShort(&currBlock->boundsMin, &currBlock->boundsMax, &currMesh->triangles[i].boundsMin, &currMesh->triangles[i].boundsMax, 1)) {
			currBlock->triangleList[currBlock->triCount] = i;
			++currBlock->triCount;
			// max tris, either start a new subdivision or increase tri count
			if (currBlock->triCount >= maxTris && currDepth < OCTREE_MAX_DEPTH) {
				free(currBlock->triangleList);
				currBlock->subdivided = true;
				currBlock->triCount = 0;
				break;
			}
			else if (currBlock->triCount >= maxTris) {
				maxTris += OCTREE_MAX_TRIS;
				currBlock->triangleList = (unsigned short*)realloc(currBlock->triangleList, sizeof(unsigned short) * maxTris);
			}
		}
	}

	if (currBlock->subdivided) {
		currBlock->blocks = (CollisionBlock*)malloc(sizeof(CollisionBlock) * 8);
		GenerateBoundsForBlocks(&currBlock->boundsMin, &currBlock->boundsMax, currBlock->blocks);
		for (int i = 0; i < 8; ++i) {
			GenerateOctree(&currBlock->blocks[i], currMesh, currDepth);
		}
	}

}

MeshCollider *MeshColliderFromMesh(Model *input) {
	if (input == NULL) {
		return NULL;
	}
	MeshCollider *retValue = (MeshCollider*)malloc(sizeof(MeshCollider));
	// start with the AABB
	retValue->AABBPosition.x = (input->boundsMin.x + input->boundsMax.x).value / 2;
	retValue->AABBPosition.y = (input->boundsMin.y + input->boundsMax.y).value / 2;
	retValue->AABBPosition.z = (input->boundsMin.z + input->boundsMax.z).value / 2;
	retValue->AABBBounds.x = input->boundsMax.x - retValue->AABBPosition.x;
	retValue->AABBBounds.y = input->boundsMax.y - retValue->AABBPosition.y;
	retValue->AABBBounds.z = input->boundsMax.z - retValue->AABBPosition.z;
	// and then move onto the triangles
	int triCount = 0;
	VertexHeader* currVertexGroup = input->vertexGroups;
	for (int i = 0; i < input->vertexGroupCount; ++i) {
		if (!(currVertexGroup->bitFlags & VTX_QUAD)) {
			if (currVertexGroup->bitFlags & VTX_STRIPS) {
				triCount += 1 + (currVertexGroup->count - 3);
			}
			else {
				triCount += currVertexGroup->count / 3;
			}
		}
		else {
			if (currVertexGroup->bitFlags & VTX_STRIPS) {
				// two verts to make one quad, or two triangles.
				triCount += 2 + (currVertexGroup->count - 4);
			}
			else {
				triCount += (currVertexGroup->count / 4) * 2;
			}
		}
		currVertexGroup = (VertexHeader*)((uint32_t)(&(currVertexGroup->vertices)) + (uint32_t)(sizeof(Vertex) * (currVertexGroup->count)));
	}
	retValue->triangles = (CollisionTriangle*)malloc(sizeof(CollisionTriangle)*triCount);
	retValue->triCount = triCount;
	int currTri = 0;
	currVertexGroup = input->vertexGroups;
	for (int i = 0; i < input->vertexGroupCount; ++i) {
		Vertex *currVerts = &currVertexGroup->vertices;
		for (int j = 0; j < currVertexGroup->count; j += 3) {
			// get verts...
			if (!(currVertexGroup->bitFlags & VTX_QUAD)) {
				if (!(currVertexGroup->bitFlags & VTX_STRIPS) || j < 3) {
					for (int k = 0; k < 3; ++k) {
						retValue->triangles[currTri].verts[k].x = currVerts[j + k].x;
						retValue->triangles[currTri].verts[k].y = currVerts[j + k].y;
						retValue->triangles[currTri].verts[k].z = currVerts[j + k].z;
					}
				}
				else {
					// not sure why i have to reverse the winding order for these, but i do
					if ((j & 1) == 0) {
						retValue->triangles[currTri].verts[1].x = currVerts[j - 2].x;
						retValue->triangles[currTri].verts[1].y = currVerts[j - 2].y;
						retValue->triangles[currTri].verts[1].z = currVerts[j - 2].z;
						retValue->triangles[currTri].verts[2].x = currVerts[j - 1].x;
						retValue->triangles[currTri].verts[2].y = currVerts[j - 1].y;
						retValue->triangles[currTri].verts[2].z = currVerts[j - 1].z;
						retValue->triangles[currTri].verts[0].x = currVerts[j].x;
						retValue->triangles[currTri].verts[0].y = currVerts[j].y;
						retValue->triangles[currTri].verts[0].z = currVerts[j].z;
					}
					else {
						retValue->triangles[currTri].verts[2].x = currVerts[j - 2].x;
						retValue->triangles[currTri].verts[2].y = currVerts[j - 2].y;
						retValue->triangles[currTri].verts[2].z = currVerts[j - 2].z;
						retValue->triangles[currTri].verts[1].x = currVerts[j - 1].x;
						retValue->triangles[currTri].verts[1].y = currVerts[j - 1].y;
						retValue->triangles[currTri].verts[1].z = currVerts[j - 1].z;
						retValue->triangles[currTri].verts[0].x = currVerts[j].x;
						retValue->triangles[currTri].verts[0].y = currVerts[j].y;
						retValue->triangles[currTri].verts[0].z = currVerts[j].z;
					}
					// fix j to increment for 1 vert instead of 1 tri
					j -= 2;
				}
				retValue->triangles[currTri].boundsMax.x = -32768;
				retValue->triangles[currTri].boundsMax.y = -32768;
				retValue->triangles[currTri].boundsMax.z = -32768;
				retValue->triangles[currTri].boundsMin.x = 32767;
				retValue->triangles[currTri].boundsMin.y = 32767;
				retValue->triangles[currTri].boundsMin.z = 32767;
				// calculate bounds extents now
				for (int k = 0; k < 3; ++k) {
					retValue->triangles[currTri].boundsMax.x = Max(retValue->triangles[currTri].boundsMax.x, currVerts[j + k].x);
					retValue->triangles[currTri].boundsMax.y = Max(retValue->triangles[currTri].boundsMax.y, currVerts[j + k].y);
					retValue->triangles[currTri].boundsMax.z = Max(retValue->triangles[currTri].boundsMax.z, currVerts[j + k].z);
					retValue->triangles[currTri].boundsMin.x = Min(retValue->triangles[currTri].boundsMin.x, currVerts[j + k].x);
					retValue->triangles[currTri].boundsMin.y = Min(retValue->triangles[currTri].boundsMin.y, currVerts[j + k].y);
					retValue->triangles[currTri].boundsMin.z = Min(retValue->triangles[currTri].boundsMin.z, currVerts[j + k].z);
				}
				// calculate normal
				retValue->triangles[currTri].normal = Vec3s::NormalFromVertsFloat(retValue->triangles[currTri].verts[0],retValue->triangles[currTri].verts[1],retValue->triangles[currTri].verts[2]);
				currTri += 1;
			}
			else {
				if (!(currVertexGroup->bitFlags & VTX_STRIPS) || j < 4) {
					for (int k = 0; k < 3; ++k) {
						retValue->triangles[currTri].verts[k].x = currVerts[j + k].x;
						retValue->triangles[currTri].verts[k].y = currVerts[j + k].y;
						retValue->triangles[currTri].verts[k].z = currVerts[j + k].z;
					}
					retValue->triangles[currTri].boundsMax.x = -32768;
					retValue->triangles[currTri].boundsMax.y = -32768;
					retValue->triangles[currTri].boundsMax.z = -32768;
					retValue->triangles[currTri].boundsMin.x = 32767;
					retValue->triangles[currTri].boundsMin.y = 32767;
					retValue->triangles[currTri].boundsMin.z = 32767;
					// calculate bounds extents now
					for (int k = 0; k < 3; ++k) {
						retValue->triangles[currTri].boundsMax.x = Max(retValue->triangles[currTri].boundsMax.x, currVerts[j + k].x);
						retValue->triangles[currTri].boundsMax.y = Max(retValue->triangles[currTri].boundsMax.y, currVerts[j + k].y);
						retValue->triangles[currTri].boundsMax.z = Max(retValue->triangles[currTri].boundsMax.z, currVerts[j + k].z);
						retValue->triangles[currTri].boundsMin.x = Min(retValue->triangles[currTri].boundsMin.x, currVerts[j + k].x);
						retValue->triangles[currTri].boundsMin.y = Min(retValue->triangles[currTri].boundsMin.y, currVerts[j + k].y);
						retValue->triangles[currTri].boundsMin.z = Min(retValue->triangles[currTri].boundsMin.z, currVerts[j + k].z);
					}
					// calculate normal
					retValue->triangles[currTri].normal = Vec3s::NormalFromVertsFloat(retValue->triangles[currTri].verts[0],retValue->triangles[currTri].verts[1],retValue->triangles[currTri].verts[2]);
					currTri += 1;
					++j;
					retValue->triangles[currTri].verts[0].x = currVerts[j + 2].x;
					retValue->triangles[currTri].verts[0].y = currVerts[j + 2].y;
					retValue->triangles[currTri].verts[0].z = currVerts[j + 2].z;
					retValue->triangles[currTri].verts[1] = retValue->triangles[currTri - 1].verts[0];
					retValue->triangles[currTri].verts[2] = retValue->triangles[currTri - 1].verts[2];
					// reverse winding if it's a strip
					if (currVertexGroup->bitFlags & VTX_STRIPS) {
						Vec3s tmp = retValue->triangles[currTri].verts[0];
						retValue->triangles[currTri].verts[0] = retValue->triangles[currTri - 1].verts[1];
						retValue->triangles[currTri].verts[1] = tmp;
						retValue->triangles[currTri].verts[2] = retValue->triangles[currTri - 1].verts[2];
					}
					retValue->triangles[currTri].boundsMax.x = -32768;
					retValue->triangles[currTri].boundsMax.y = -32768;
					retValue->triangles[currTri].boundsMax.z = -32768;
					retValue->triangles[currTri].boundsMin.x = 32767;
					retValue->triangles[currTri].boundsMin.y = 32767;
					retValue->triangles[currTri].boundsMin.z = 32767;
					// calculate bounds extents now
					for (int k = 0; k < 3; ++k) {
						retValue->triangles[currTri].boundsMax.x = Max(retValue->triangles[currTri].boundsMax.x, retValue->triangles[currTri].verts[k].x);
						retValue->triangles[currTri].boundsMax.y = Max(retValue->triangles[currTri].boundsMax.y, retValue->triangles[currTri].verts[k].y);
						retValue->triangles[currTri].boundsMax.z = Max(retValue->triangles[currTri].boundsMax.z, retValue->triangles[currTri].verts[k].z);
						retValue->triangles[currTri].boundsMin.x = Min(retValue->triangles[currTri].boundsMin.x, retValue->triangles[currTri].verts[k].x);
						retValue->triangles[currTri].boundsMin.y = Min(retValue->triangles[currTri].boundsMin.y, retValue->triangles[currTri].verts[k].y);
						retValue->triangles[currTri].boundsMin.z = Min(retValue->triangles[currTri].boundsMin.z, retValue->triangles[currTri].verts[k].z);
					}
					// calculate normal
					retValue->triangles[currTri].normal = Vec3s::NormalFromVertsFloat(retValue->triangles[currTri].verts[0],retValue->triangles[currTri].verts[1],retValue->triangles[currTri].verts[2]);
				}
				else {
					// create virtual quad
					Vec3s quad[4];
					quad[0].x = currVerts[j - 1].x;
					quad[0].y = currVerts[j - 1].y;
					quad[0].z = currVerts[j - 1].z;
					quad[1].x = currVerts[j - 2].x;
					quad[1].y = currVerts[j - 2].y;
					quad[1].z = currVerts[j - 2].z;
					quad[2].x = currVerts[j + 1].x;
					quad[2].y = currVerts[j + 1].y;
					quad[2].z = currVerts[j + 1].z;
					quad[3].x = currVerts[j].x;
					quad[3].y = currVerts[j].y;
					quad[3].z = currVerts[j].z;
					// adjust j position
					j -= 1;

					retValue->triangles[currTri].verts[0] = quad[2];
					retValue->triangles[currTri].verts[1] = quad[1];
					retValue->triangles[currTri].verts[2] = quad[0];
					retValue->triangles[currTri].boundsMax.x = -32768;
					retValue->triangles[currTri].boundsMax.y = -32768;
					retValue->triangles[currTri].boundsMax.z = -32768;
					retValue->triangles[currTri].boundsMin.x = 32767;
					retValue->triangles[currTri].boundsMin.y = 32767;
					retValue->triangles[currTri].boundsMin.z = 32767;
					// calculate bounds extents now
					for (int k = 0; k < 3; ++k) {
						retValue->triangles[currTri].boundsMax.x = Max(retValue->triangles[currTri].boundsMax.x, retValue->triangles[currTri].verts[k].x);
						retValue->triangles[currTri].boundsMax.y = Max(retValue->triangles[currTri].boundsMax.y, retValue->triangles[currTri].verts[k].y);
						retValue->triangles[currTri].boundsMax.z = Max(retValue->triangles[currTri].boundsMax.z, retValue->triangles[currTri].verts[k].z);
						retValue->triangles[currTri].boundsMin.x = Min(retValue->triangles[currTri].boundsMin.x, retValue->triangles[currTri].verts[k].x);
						retValue->triangles[currTri].boundsMin.y = Min(retValue->triangles[currTri].boundsMin.y, retValue->triangles[currTri].verts[k].y);
						retValue->triangles[currTri].boundsMin.z = Min(retValue->triangles[currTri].boundsMin.z, retValue->triangles[currTri].verts[k].z);
					}
					// calculate normal
					retValue->triangles[currTri].normal = Vec3s::NormalFromVertsFloat(retValue->triangles[currTri].verts[0],retValue->triangles[currTri].verts[1],retValue->triangles[currTri].verts[2]);
					currTri += 1;
					retValue->triangles[currTri].verts[0] = quad[1];
					retValue->triangles[currTri].verts[1] = quad[2];
					retValue->triangles[currTri].verts[2] = quad[3];
					retValue->triangles[currTri].boundsMax.x = -32768;
					retValue->triangles[currTri].boundsMax.y = -32768;
					retValue->triangles[currTri].boundsMax.z = -32768;
					retValue->triangles[currTri].boundsMin.x = 32767;
					retValue->triangles[currTri].boundsMin.y = 32767;
					retValue->triangles[currTri].boundsMin.z = 32767;
					// calculate bounds extents now
					for (int k = 0; k < 3; ++k) {
						retValue->triangles[currTri].boundsMax.x = Max(retValue->triangles[currTri].boundsMax.x, retValue->triangles[currTri].verts[k].x);
						retValue->triangles[currTri].boundsMax.y = Max(retValue->triangles[currTri].boundsMax.y, retValue->triangles[currTri].verts[k].y);
						retValue->triangles[currTri].boundsMax.z = Max(retValue->triangles[currTri].boundsMax.z, retValue->triangles[currTri].verts[k].z);
						retValue->triangles[currTri].boundsMin.x = Min(retValue->triangles[currTri].boundsMin.x, retValue->triangles[currTri].verts[k].x);
						retValue->triangles[currTri].boundsMin.y = Min(retValue->triangles[currTri].boundsMin.y, retValue->triangles[currTri].verts[k].y);
						retValue->triangles[currTri].boundsMin.z = Min(retValue->triangles[currTri].boundsMin.z, retValue->triangles[currTri].verts[k].z);
					}
					// calculate normal
					retValue->triangles[currTri].normal = Vec3s::NormalFromVertsFloat(retValue->triangles[currTri].verts[0],retValue->triangles[currTri].verts[1],retValue->triangles[currTri].verts[2]);
				}
				currTri += 1;
			}
		}
		currVertexGroup = (VertexHeader*)((uint32_t)(&(currVertexGroup->vertices)) + (uint32_t)(sizeof(Vertex) * (currVertexGroup->count)));
	}

	Vec3 AABBMin = retValue->AABBPosition - retValue->AABBBounds;
	Vec3 AABBMax = retValue->AABBPosition + retValue->AABBBounds;
	Vec3s AABBMins, AABBMaxs;
	Vec3ToShort(AABBMin, AABBMins);
	Vec3ToShort(AABBMax, AABBMaxs);
	GenerateBoundsForBlocks(&AABBMins, &AABBMaxs, retValue->blocks);
	for (int i = 0; i < 8; ++i) {
		GenerateOctree(&retValue->blocks[i], retValue, 0);
	}

	return retValue;
}

void MeshCollider::FindTrianglesFromOctreeInternal(const Vec3& min, const Vec3& max, CollisionBlock* block, unsigned short** retValue, int* maxSize, int* currSize) {
	if (block->subdivided) {
		for (int i = 0; i < 8; ++i) {
			Vec3 blockMin, blockMax;
			ShortToVec3(block->blocks[i].boundsMin, blockMin);
			ShortToVec3(block->blocks[i].boundsMax, blockMax);
			if (AABBCheck(min, max, blockMin, blockMax)) {
				FindTrianglesFromOctreeInternal(min, max, &block->blocks[i], retValue, maxSize, currSize);
			}
		}
	}
	else {
		// ensure we overlap the triangles
		for (int i = 0; i < block->triCount; ++i) {
			const CollisionTriangle* currTri = &triangles[block->triangleList[i]];
			Vec3 vMin, vMax;
			ShortToVec3(currTri->boundsMin, vMin);
			ShortToVec3(currTri->boundsMax, vMax);
			if (AABBCheck(min, max, vMin, vMax)) {
				// omit duplicates
				bool toContinue = false;
				// this DEFINITELY needs some form of optimization. iterating over the whole thing is garbage. allocating ANY extra memory is likely slower, though, so i'm unsure how to approach.
				// form the data into a binary tree instead? though i fear that to be harder to read back
				for (int j = 0; j < *currSize; ++j) {
					if (retValue[0][j] == block->triangleList[i]) {
						toContinue = true;
						break;
					}
				}
				if (toContinue) {
					continue;
				}
				retValue[0][*currSize] = block->triangleList[i];
				++*currSize;
				if (*currSize >= *maxSize) {
#ifndef COLLISION_DTCM
					*maxSize += 512;
					retValue[0] = (unsigned short*)realloc(retValue[0], sizeof(unsigned short) * *maxSize);
#else
					if (*maxSize & 0x40000000) {
						*maxSize &= ~0x40000000;
						*maxSize += 512;
						retValue[0] = (unsigned short*)malloc(sizeof(unsigned short) * *maxSize);
						memcpy(retValue[0], staticCollisionAllocation, COLLISION_DTCM_SIZE);
					}
					else {
						*maxSize += 512;
						retValue[0] = (unsigned short*)realloc(retValue[0], sizeof(unsigned short) * *maxSize);
					}
#endif
				}
			}
		}
	}
}

unsigned short* MeshCollider::FindTrianglesFromOctree(const Vec3& min, const Vec3& max, int *totalTris) {
#ifndef COLLISION_DTCM
	unsigned short* retValue = staticCollisionAllocation;//(unsigned short*)malloc(sizeof(unsigned short) * 512);
	int maxSize = 512 | 0x40000000;
#else
	unsigned short* retValue = staticCollisionAllocation;
	int maxSize = COLLISION_DTCM_SIZE | 0x40000000;
#endif
	int currSize = 0;
	for (int i = 0; i < 8; ++i) {
		Vec3 blockMin, blockMax;
		ShortToVec3(blocks[i].boundsMin, blockMin);
		ShortToVec3(blocks[i].boundsMax, blockMax);
		if (AABBCheck(min, max, blockMin, blockMax)) {
			FindTrianglesFromOctreeInternal(min, max, &blocks[i], &retValue, &maxSize, &currSize);
		}
	}
	*totalTris = currSize;
	// dummy
	/*free(retValue);
	*totalTris = meshCollider->triCount;
	retValue = (unsigned int*)malloc(sizeof(unsigned int) * meshCollider->triCount);
	for (int i = 0; i < meshCollider->triCount; ++i) {
		retValue[i] = i;
	}*/
	return retValue;
}

void MeshCollider::ReleaseTriangleOctreeAllocation(unsigned short *tris) {
	// no need to do anything...
	if (tris == staticCollisionAllocation) return;
	free(tris);
}

void DestroyOctree(CollisionBlock* block) {
	if (block->subdivided) {
		for (int i = 0; i < 8; ++i) {
			DestroyOctree(&block->blocks[i]);
		}
	}
	// tomato tomato, both are ptrs here
	free(block->blocks);
}

MeshCollider::~MeshCollider() {
	// destroy all the octrees...
	for (int i = 0; i < 8; ++i) {
		DestroyOctree(&blocks[i]);
	}
	free(triangles);
}

// doesn't return position by default since this should rarely be used by actual game code
bool RayCast::OnAABB(const Vec3& boxMin, const Vec3& boxMax, BasicCollisionInfo* infoOut) {
	Vec3 workVec;
	Vec3 newDir = dir;
	// division by 0 fix
	if (newDir.x == 0) {
		newDir.x = 1;
	}
	if (newDir.y == 0) {
		newDir.y = 1;
	}
	if (newDir.z == 0) {
		newDir.z = 1;
	}
	long long tMin1, tMin2, tMin3;
	long long tMax1, tMax2, tMax3;
	workVec = boxMin - point;
	tMin1 = Int64Div(workVec.x, newDir.x);
	tMin2 = Int64Div(workVec.y, newDir.y);
	tMin3 = Int64Div(workVec.z, newDir.z);
	workVec = boxMax - point;
	tMax1 = Int64Div(workVec.x, newDir.x);
	tMax2 = Int64Div(workVec.y, newDir.y);
	tMax3 = Int64Div(workVec.z, newDir.z);
	long long t11, t12, t13, t21, t22, t23;
	// min...
	t11 = tMin1 > tMax1 ? tMax1 : tMin1;
	t12 = tMin2 > tMax2 ? tMax2 : tMin2;
	t13 = tMin3 > tMax3 ? tMax3 : tMin3;
	// max...
	t21 = tMin1 > tMax1 ? tMin1 : tMax1;
	t22 = tMin2 > tMax2 ? tMin2 : tMax2;
	t23 = tMin3 > tMax3 ? tMin3 : tMax3;

	long long tNear, tFar;

	tNear = t11;
	if (t12 > tNear) {
		tNear = t12;
	}
	if (t13 > tNear) {
		tNear = t13;
	}

	tFar = t21;
	if (t22 < tFar) {
		tFar = t22;
	}
	if (t23 < tFar) {
		tFar = t23;
	}
	// who cares if it gets truncated
	long long dist;
	if (tNear < 0) {
		dist = tFar;
	}
	else {
		dist = tNear;
	}

	if (dist > length) {
		return false;
	}

	// return normal as well
	if (infoOut != NULL) {
		infoOut->t = dist;
		Fixed tSeries[] = { tMin1, tMax1, tMin2, tMax2, tMin3, tMax3 };
		Vec3 normals[] = { {-4096, 0, 0 }, {4096, 0, 0},
			{0, -4096, 0}, {0, 4096, 0},
			{0, 0, -4096}, {0, 0, 4096} };
		for (int i = 0; i < 6; ++i) {
			if (dist == tSeries[i]) {
				infoOut->normal = normals[i];
				infoOut->position = (dir * dist) + point;
				break;
			}
		}
	}

	return tNear <= tFar && tFar >= 0;
}

bool RayCast::OnSphere(const SphereCollider& sphere, BasicCollisionInfo* infoOut) {
	// TODO: int64 version for DS?
	Vec3 dist = point - *sphere.position;
	Fixed b = dist.Dot(dir);
	Fixed c = dist.Dot(dist) - (sphere.radius * sphere.radius);
	// return if it's outside of the sphere and pointing the wrong way
	if (c > 0 && b > 0) return false;
	Fixed discr = b * b - c;
	// negative discriminant means a miss
	if (discr < 0) return false;
	// ray found to intersect sphere, compute intersection
	Fixed hitDist = -b.value - sqrtf32f(discr);
	// if t is negative, started in sphere
	if (hitDist < 0) {
		hitDist = 0;
	}
	if (infoOut != NULL) {
		infoOut->t = hitDist;
		infoOut->position = point + (dir * hitDist);
		infoOut->normal = (infoOut->position - *sphere.position).Normalize();
	}
	return true;
}

bool RayCast::OnPlane(const Vec3& normal, const Fixed planeDistance, Fixed *t, Vec3* hitPos) {
	// two normals, can safely not use 64 bit here
	Fixed nd = dir.Dot(normal);//DotProductNormal(direction, normal);
	Fixed pn = point.Dot(normal);
	// if nd is positive, they're facing the same way. no collision
	if (nd >= 0) {
		return false;
	}
	Fixed hitDist = (planeDistance - pn) / nd;
	if (hitDist >= 0) {
		if (t != NULL) {
			*t = hitDist;
		}
		if (hitPos != NULL) {
			*hitPos = point + dir*hitDist;
			return true;
		}
	}
	return false;
}

bool RayCast::OnTriangle(const CollisionTriangle& triangle, BasicCollisionInfo* infoOut) {
	BasicCollisionInfo tmpInfoOut;
	if (infoOut == NULL) {
		infoOut = &tmpInfoOut;
	}
	Vec3 v;
	ShortToVec3(triangle.normal, infoOut->normal);
	ShortToVec3(triangle.verts[0], v);
	if (OnPlane(infoOut->normal, infoOut->normal.Dot(v), &infoOut->t, &infoOut->position)) {
		Vec3 barycentric = BarycentricCoords(triangle, infoOut->position);
		if (barycentric.x >= 0 && barycentric.x <= 4096
			&& barycentric.y >= 0 && barycentric.y <= 4096
			&& barycentric.z >= 0 && barycentric.z <= 4096) {
			return true;
		}
	}

	return false;
}

int CollisionBlock::GetQuadTreeCount() const {
	// potentially recode this to do the subdivided check before calling on it, to avoid extra function calls as a minor optimization
	int count = 0;
	if (subdivided) {
		for (int i = 0; i < 8; ++i) {
			count += blocks[i].GetQuadTreeCount();
		}
	}
	else {
		if (triCount > 0) {
			count = 1;
		}
	}
	return count;
}

int MeshCollider::GetQuadTreeCount() const {
	int count = 0;
	for (int i = 0; i < 8; ++i) {
		count += blocks[i].GetQuadTreeCount();
	}
	return count;
}

void RayCast::OnQuadTreeSub(const Vec3& AABBMin, const Vec3& AABBMax, const CollisionBlock* block, const CollisionBlock** hitBlocks, int* hitBlockPosition, int* triCount) {
	BasicCollisionInfo info;
	if (!block->subdivided) {
		if (block->triCount > 0) {
			Vec3 bMin, bMax;
			ShortToVec3(block->boundsMin, bMin);
			ShortToVec3(block->boundsMax, bMax);
			if (AABBCheck(AABBMin, AABBMax, bMin, bMax)) {
				bool hit = false;
				if (OnAABB(bMin, bMax, &info)) {
					if (info.t <= length) {
						hitBlocks[*hitBlockPosition] = block;
						*hitBlockPosition += 1;
						*triCount += block->triCount;
						hit = true;
					}
				}
				if (hit == false && (point.x >= AABBMin.x && point.x <= AABBMax.x &&
					point.y >= AABBMin.y && point.y <= AABBMax.y &&
					point.z >= AABBMin.z && point.z <= AABBMax.z)) {
					hitBlocks[*hitBlockPosition] = block;
					*hitBlockPosition += 1;
					*triCount += block->triCount;
				}
			}
		}
	}
	else {
		for (int i = 0; i < 8; ++i) {
			OnQuadTreeSub(AABBMin, AABBMax, &block->blocks[i], hitBlocks, hitBlockPosition, triCount);
		}
	}
}

void RayCast::OnQuadTree(const Vec3& AABBMin, const Vec3& AABBMax, const MeshCollider& mesh, CollisionBlock** hitBlocks, int* hitBlockPosition, int* triCount) {
	for (int i = 0; i < 8; ++i) {
		OnQuadTreeSub(AABBMin, AABBMax, &mesh.blocks[i], (const CollisionBlock**)hitBlocks, hitBlockPosition, triCount);
	}
}

int RayCast::OnMesh(const MeshCollider& mesh, const Vec3& meshOffset, const Vec3& meshScale, const Quaternion& meshRotation, BasicCollisionInfo* infoOut) {

	// attempt at optimization
	Fixed maxBounds = Max(Max(mesh.AABBBounds.x, mesh.AABBBounds.y), mesh.AABBBounds.z);
	Vec3 oldMeshMax = {
		(mesh.AABBPosition.x + maxBounds) * meshScale.x + meshOffset.x,
		(mesh.AABBPosition.y + maxBounds) * meshScale.y + meshOffset.y,
		(mesh.AABBPosition.z + maxBounds) * meshScale.z + meshOffset.z
	};

	Vec3 oldMeshMin = {
		(mesh.AABBPosition.x - maxBounds) * meshScale.x + meshOffset.x,
		(mesh.AABBPosition.y - maxBounds) * meshScale.y + meshOffset.y,
		(mesh.AABBPosition.z - maxBounds) * meshScale.z + meshOffset.z
	};

	Vec3 rayEnd = point + (dir * length);

	Vec3 rayMin = Vec3(
		Min(point.x, rayEnd.x),
		Min(point.y, rayEnd.y),
		Min(point.z, rayEnd.z)
	);

	Vec3 rayMax = Vec3(
		Max(point.x, rayEnd.x),
		Max(point.y, rayEnd.y),
		Max(point.z, rayEnd.z)
	);

	if (!AABBCheck(rayMin, rayMax, oldMeshMin, oldMeshMax)) {
		return false;
	}

	// transform the point and direction
	Vec3 newPoint, newDirection;
	Vec3 workVec;
	newPoint = point - meshOffset;
	Quaternion invQuat = meshRotation.Inverse();
	workVec = invQuat * newPoint;
	newPoint = workVec / meshScale;

	// direction now
	workVec = invQuat * dir;
	newDirection = workVec / meshScale;
	newDirection = newDirection.Normalize();

	// generate new length value
	Vec3 absDir = {
		abs(newDirection.x.value),
		abs(newDirection.y.value),
		abs(newDirection.z.value)
	};
	Vec3 absScale = {
		abs(meshScale.x.value),
		abs(meshScale.y.value),
		abs(meshScale.z.value)
	};
	Fixed lenDot = absDir.Dot(absScale);
	Fixed newLength = length / lenDot;

	Vec3 rayPlusDir = newPoint + newDirection * newLength;

	Vec3 rayAABBMin = {
		Min(newPoint.x, rayPlusDir.x),
		Min(newPoint.y, rayPlusDir.y),
		Min(newPoint.z, rayPlusDir.z)
	};
	Vec3 rayAABBMax = {
		Max(newPoint.x, rayPlusDir.x),
		Max(newPoint.y, rayPlusDir.y),
		Max(newPoint.z, rayPlusDir.z)
	};

	// check against the mesh AABB first
	BasicCollisionInfo tempInfo;
	Vec3 AABBMin, AABBMax;
	AABBMin = mesh.AABBPosition - mesh.AABBBounds;
	AABBMax = mesh.AABBPosition + mesh.AABBBounds;

	RayCast newRay(newPoint, newDirection, newLength);

	// simple AABB point check
	if (!(newPoint.x >= AABBMin.x && newPoint.x <= AABBMax.x &&
		newPoint.y >= AABBMin.y && newPoint.y <= AABBMax.y &&
		newPoint.z >= AABBMin.z && newPoint.z <= AABBMax.z)) {
		// not within AABB, check if we intersect
		if (newRay.OnAABB(AABBMin, AABBMax, &tempInfo)) {
			if (tempInfo.t > newLength) {
				return false;
			}
		}
		else {
			return false;
		}
	}

	// now raycast against the quadtrees
	int quadTreeCount = mesh.GetQuadTreeCount();

	CollisionBlock** hitBlocks = (CollisionBlock**)malloc(sizeof(CollisionBlock*) * quadTreeCount);
	int hitBlockPosition = 0;
	int triCount = 0;
	
	newRay.OnQuadTree(rayAABBMin, rayAABBMax, mesh, hitBlocks, &hitBlockPosition, &triCount);
	
	// TODO: potential optimization, sort by the quadtree positions so we have to sort fewer individual hits if applicable

	// all blocks hit, now create a list of triangles, omitting duplicates
	int* trisToCheck = (int*)malloc(sizeof(int) * triCount);
	int realTriCount = 0;
	Fixed closestHit = 2147483647;
	int closestTri;
	bool everHit = false;
	for (int i = 0; i < hitBlockPosition; ++i) {
		for (int j = 0; j < hitBlocks[i]->triCount; ++j) {
			// ensure no duplicates here
			bool duplicateTri = false;
			for (int k = 0; k < realTriCount; ++k) {
				if (trisToCheck[k] == hitBlocks[i]->triangleList[j]) {
					duplicateTri = true;
					break;
				}
			}
			if (duplicateTri) {
				continue;
			}
			trisToCheck[realTriCount] = hitBlocks[i]->triangleList[j];
			++realTriCount;

			// raycast the triangle now, starting with the AABB
			Vec3 newBoundsMin;
			Vec3 newBoundsMax;
			ShortToVec3(mesh.triangles[hitBlocks[i]->triangleList[j]].boundsMin, newBoundsMin);
			ShortToVec3(mesh.triangles[hitBlocks[i]->triangleList[j]].boundsMax, newBoundsMax);
			if (AABBCheck(rayAABBMin, rayAABBMax, newBoundsMin, newBoundsMax)) {
				//bool inAABB = (newPoint.x >= newBoundsMin->x && newPoint.x <= newBoundsMax->x &&
					//newPoint.y >= newBoundsMin->y && newPoint.y <= newBoundsMax->y &&
					//newPoint.z >= newBoundsMin->z && newPoint.z <= newBoundsMax->z);
				//if (inAABB || RayOnAABB(&newPoint, &newDirection, newBoundsMin, newBoundsMax, &tempt)) {
					//if (inAABB || tempt <= newLength) {
						if (newRay.OnTriangle(mesh.triangles[hitBlocks[i]->triangleList[j]], &tempInfo)) {
							if (tempInfo.t < closestHit) {
								closestHit = tempInfo.t;
								closestTri = hitBlocks[i]->triangleList[j];
								everHit = true;
							}
						}
					//}
				//}
			}
		}
	}
	free(trisToCheck);
	free(hitBlocks);
	if (everHit) {
		if (infoOut != NULL) {
			infoOut->t = closestHit * lenDot;
			infoOut->position = point + (dir * infoOut->t);
			ShortToVec3(mesh.triangles[closestTri].normal, infoOut->normal);
		}
		return closestTri;
	}
	return -1; // failed
}

void ClosestPointAABB(const Vec3& position, const Vec3& boxMin, const Vec3& boxMax, Vec3* out) {
	*out = position;

	out->x = (out->x < boxMin.x) ? boxMin.x : out->x;
	out->y = (out->y < boxMin.y) ? boxMin.y : out->y;
	out->z = (out->z < boxMin.z) ? boxMin.z : out->z;

	out->x = (out->x > boxMax.x) ? boxMax.x : out->x;
	out->y = (out->y > boxMax.y) ? boxMax.y : out->y;
	out->z = (out->z > boxMax.z) ? boxMax.z : out->z;
}

// essentially just sphere on AABB but applying inverse rotation to the sphere
bool SphereCollider::OnOBB(const BoxCollider& box, BasicCollisionInfo* infoOut) {
	if (infoOut == NULL) {
		return false;
	}

	Vec3 rotatedSpherePoint, workVec;
	workVec = *position - *box.position;
	Quaternion invQuat = box.rotation->Inverse();
	rotatedSpherePoint = invQuat * workVec;

	// center around 0
	Vec3 boxMin;
	Vec3 zeroVec = { 0, 0, 0 };
	boxMin = zeroVec - box.extents;

	Vec3 closestPoint;
	ClosestPointAABB(rotatedSpherePoint, boxMin, box.extents, &closestPoint);
	Vec3 closestRelativeToSphere = rotatedSpherePoint - closestPoint;
	Fixed sqrDist = closestRelativeToSphere.SqrMagnitude();

	if (sqrDist <= radius * radius) {
		infoOut->position = closestPoint;
		// global space it again...
		infoOut->position = infoOut->position + *box.position;
		if (sqrDist <= 1) {
			// we're INSIDE the cube, fix!
			infoOut->normal = rotatedSpherePoint.Normalize();

			// gross i know but nothing better came to me
			// 2867 = 0.7f
			const Fixed normalCheck = 2867;
			if (infoOut->normal.y.value >= normalCheck.value) {
				infoOut->normal.x = 0;
				infoOut->normal.y = 4096;
				infoOut->normal.z = 0;
			} else if (infoOut->normal.y.value <= -normalCheck.value) {
				infoOut->normal.x = 0;
				infoOut->normal.y = -4096;
				infoOut->normal.z = 0;
			} else if (infoOut->normal.x.value >= normalCheck.value) {
				infoOut->normal.x = 4096;
				infoOut->normal.y = 0;
				infoOut->normal.z = 0;
			} else if (infoOut->normal.x.value <= -normalCheck.value) {
				infoOut->normal.y = 0;
				infoOut->normal.x = -4096;
				infoOut->normal.z = 0;
			} else if (infoOut->normal.z.value >= normalCheck.value) {
				infoOut->normal.x = 0;
				infoOut->normal.z = 4096;
				infoOut->normal.y = 0;
			}
			else {
				infoOut->normal.x = 0;
				infoOut->normal.y = 0;
				infoOut->normal.z = -4096;
			}
			// this also sucks
			infoOut->penetration = radius + (infoOut->normal.Dot(box.extents)).fabs() - infoOut->normal.Dot(rotatedSpherePoint);
		}
		else {
			// we're outside of cube, return values
			infoOut->normal = closestRelativeToSphere.Normalize();
			infoOut->penetration = radius.value - sqrtf32f(sqrDist);
		}
		infoOut->normal = *box.rotation * infoOut->normal;
		return true;
	}
	return false;
}