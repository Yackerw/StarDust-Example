#include <stdio.h>
#include "sdcollision.h"
#include <nds.h>
#include "sdmath.h"
#include <stdint.h>
#define OCTREE_MAX_DEPTH 10
#define OCTREE_MAX_TRIS 40
// enable this to reserve DTCM to speed up collision code
#define COLLISION_DTCM
#define COLLISION_DTCM_SIZE 0x800 // reserve 0x800 shorts for collision DTCM data; or, in other words, 4 KB out of the 16 available

#define GJK_LENIENCY 16
// note: barely noticeable slowdown from this, and fixes a bug with large polygons
#define FLOATBARY
//#define BARY64

#ifdef COLLISION_DTCM
DTCM_DATA unsigned short staticCollisionAllocation[COLLISION_DTCM_SIZE];
#endif

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

ITCM_CODE void Vec3Subtractions(Vec3s* left, Vec3s* right, Vec3* out) {
	out->x = left->x - right->x;
	out->y = left->y - right->y;
	out->z = left->z - right->z;
}

ITCM_CODE Vec3 BarycentricCoords(CollisionTriangle *tri, Vec3 *point) {
	Vec3 v0, v1, v2;
	Vec3Subtractions(&tri->verts[1], &tri->verts[0], &v0);
	Vec3Subtractions(&tri->verts[2], &tri->verts[0], &v1);
	ShortToVec3(tri->verts[0], v2);
	Vec3Subtraction(point, &v2, &v2);
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
	f32 d00 = DotProduct(&v0, &v0);
	f32 d01 = DotProduct(&v0, &v1);
	f32 d11 = DotProduct(&v1, &v1);
	f32 d20 = DotProduct(&v2, &v0);
	f32 d21 = DotProduct(&v2, &v1);
	f32 denom = mulf32(d00, d11) - mulf32(d01, d01);
	Vec3 retValue;
	retValue.x = divf32(mulf32(d11, d20) - mulf32(d01, d21), denom);
	retValue.y = divf32(mulf32(d00, d21) - mulf32(d01, d20), denom);
	retValue.z = 4096 - retValue.x - retValue.y;
	
	return retValue;
#endif
	#endif
}

ITCM_CODE bool SphereOnPoint(CollisionSphere *sphere, f32 sphereSquareMagnitude, Vec3 *point, f32 *penetration) {
	Vec3 distance;
	Vec3Subtraction(sphere->position, point, &distance);
	f32 mag = SqrMagnitude(&distance);
	if (mag < sphereSquareMagnitude) {
		*penetration = sphere->radius - sqrtf32(mag);
		return true;
	}
	return false;
}

ITCM_CODE bool SphereOnLine(CollisionSphere *sphere, Vec3 *p1, Vec3 *p2, Vec3 *closestPoint) {
	Vec3 working;
	Vec3 working2;
	Vec3 working3;
	f32 t;
	f32 t2;
	Vec3Subtraction(p2, p1, &working3);
	Vec3Subtraction(p1, sphere->position, &working2);
	t = DotProduct(&working3, &working2);
	t2 = DotProduct(&working3, &working3);
	t = divf32(t, t2);
	
	working.x = mulf32(t, working3.x);
	working.y = mulf32(t, working3.y);
	working.z = mulf32(t, working3.z);
	closestPoint->x = p1->x - working.x;
	closestPoint->y = p1->y - working.y;
	closestPoint->z = p1->z - working.z;
	
	// if distance to closest point is greater than the spheres radius, no collision
	Vec3Subtraction(sphere->position, closestPoint, &working);
	if (SqrMagnitude(&working) >= mulf32(sphere->radius, sphere->radius)) {
		return false;
	}
	// okay, get magnitude between the points. if
	// the distance between the two points of the line and the closest point is equal to the distance between
	// the two points of the line, then it's on the line and we return true
	f32 magLine = SqrMagnitude(&working3);
	Vec3Subtraction(p1, closestPoint, &working);
	f32 magPoint1 = Magnitude(&working);
	Vec3Subtraction(p2, closestPoint, &working);
	f32 magPoint2 = Magnitude(&working);
	magPoint1 += magPoint2;
	// this saves us 1 (one) sqrt call for magLine. worth, I think.
	magPoint1 = mulf32(magPoint1, magPoint1);
	// add a little leniency for, uh...lack of precision
	if (magPoint1 <= magLine + 32 && magPoint1 >= magLine - 32) {
		return true;
	}
	
	return false;
}

ITCM_CODE bool SphereOnTriangleLine(CollisionSphere *sphere, CollisionTriangle *tri, Vec3 *normal, f32 *penetration) {
	// and finally, line collision
	for (int i = 0; i < 3; ++i) {
		Vec3 closestPoint;
		Vec3 v1;
		Vec3 v2;
		ShortToVec3(tri->verts[i], v1);
		ShortToVec3(tri->verts[(i + 1) % 3], v2);
		if (SphereOnLine(sphere, &v1, &v2, &closestPoint)) {
			// get penetration
			Vec3 lineDiff;
			Vec3Subtraction(sphere->position, &closestPoint, &lineDiff);
			*penetration = sphere->radius - Magnitude(&lineDiff);
			// and finally the normal
			Normalize(&lineDiff, normal);
			
			return true;
		}
	}
	return false;
}

ITCM_CODE bool SphereOnTriangleVertex(CollisionSphere *sphere, f32 sphereSquareMagnitude, CollisionTriangle *tri, Vec3 *normal, f32 *penetration) {
	for (int i = 0; i < 3; ++i) {
		f32 pointMag;
		Vec3 v;
		ShortToVec3(tri->verts[i], v);
		// TODO: cache sphere square radius so we aren't recalculating it every time. that 64 bit multiply isn't cheap!
		if (SphereOnPoint(sphere, sphereSquareMagnitude, &v, &pointMag)) {
			// get penetration and normal
			*penetration = pointMag;
			// normal; subtract the two vectors, then normalize
			Vec3Subtraction(sphere->position, &v, normal);
			Normalize(normal, normal);
			return true;
		}
	}
	return false;
}

ITCM_CODE bool SphereOnTrianglePlane(CollisionSphere *sphere, CollisionTriangle *tri, Vec3 *normal, f32 *penetration, bool *onPlane) {
	// adjust sphere so triangle is origin
	Vec3 newSpherePosition;
	newSpherePosition.x = sphere->position->x - tri->verts[0].x;
	newSpherePosition.y = sphere->position->y - tri->verts[0].y;
	newSpherePosition.z = sphere->position->z - tri->verts[0].z;
	// finally, just apply a dot product
	Vec3 n;
	ShortToVec3(tri->normal, n);
	f32 dot = DotProduct(&newSpherePosition, &n);
	// don't collide if we're beneath the polygon!
	if (dot >= sphere->radius || dot < 0) {
		*onPlane = false;
		return false;
	}
	*penetration = sphere->radius - dot;
	// get closest position on the plane
	Vec3 spotOnPlane;
	spotOnPlane.x = mulf32(dot, -tri->normal.x);
	spotOnPlane.y = mulf32(dot, -tri->normal.y);
	spotOnPlane.z = mulf32(dot, -tri->normal.z);
	spotOnPlane.x += sphere->position->x;
	spotOnPlane.y += sphere->position->y;
	spotOnPlane.z += sphere->position->z;
	// now calculate the barycentric coordinates on the triangle
	Vec3 barycentric = BarycentricCoords(tri, &spotOnPlane);
	*onPlane = true;
	// if any are < 0 or > 1, no collision here. if all arent, then yes collision. also add some leniency
	if (barycentric.x >= 0 && barycentric.x <= 4096
	&& barycentric.y >= 0 && barycentric.y <= 4096
	&& barycentric.z >= 0 && barycentric.z <= 4096) {
		normal->x = tri->normal.x;
		normal->y = tri->normal.y;
		normal->z = tri->normal.z;
		return true;
	}
	return false;
}

ITCM_CODE MeshCollider *LoadCollisionMesh(char *input) {
	FILE *f = fopen(input, "rb");
	fseek(f, 0, SEEK_END);
	int fileSize = ftell(f);
	fseek(f, 0, SEEK_SET);
	MeshCollider *mesh = malloc(fileSize);
	fread(mesh, fileSize, 1, f);
	fclose(f);
	mesh->triangles = (CollisionTriangle*)((uint32_t)mesh + (uint32_t)mesh->triangles);
	return mesh;
}

ITCM_CODE bool SphereOnSphere(CollisionSphere *sphere1, CollisionSphere *sphere2, f32 *penetration, Vec3 *normal) {
	// simple, just get distance
	Vec3 sub;
	Vec3Subtraction(sphere1->position, sphere2->position, &sub);
	int mag = Magnitude(&sub);
	if (mag < sphere1->radius + sphere2->radius) {
		Normalize(&sub, normal);
		*penetration = (sphere1->radius + sphere2->radius) - mag;
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

ITCM_CODE bool AABBCheck(Vec3 *minA, Vec3 *maxA, Vec3 *minB, Vec3 *maxB) {
	return (minA->x <= maxB->x &&
		maxA->x >= minB->x &&
		minA->y <= maxB->y &&
		maxA->y >= minB->y &&
		minA->z <= maxB->z &&
		maxA->z >= minB->z);
}

ITCM_CODE bool AABBCheckLeniency(Vec3* minA, Vec3* maxA, Vec3* minB, Vec3* maxB, f32 leniency) {
	return (minA->x <= maxB->x + leniency &&
		maxA->x >= minB->x - leniency &&
		minA->y <= maxB->y + leniency &&
		maxA->y >= minB->y - leniency &&
		minA->z <= maxB->z + leniency &&
		maxA->z >= minB->z - leniency);
}

ITCM_CODE bool AABBCheckLeniencyShort(Vec3s* minA, Vec3s* maxA, Vec3s* minB, Vec3s* maxB, f32 leniency) {
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
	retValue->AABBPosition.x = (input->boundsMin.x + input->boundsMax.x) / 2;
	retValue->AABBPosition.y = (input->boundsMin.y + input->boundsMax.y) / 2;
	retValue->AABBPosition.z = (input->boundsMin.z + input->boundsMax.z) / 2;
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
				NormalFromVertsFloat(&retValue->triangles[currTri].verts[0], &retValue->triangles[currTri].verts[1], &retValue->triangles[currTri].verts[2], &retValue->triangles[currTri].normal);
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
					NormalFromVertsFloat(&retValue->triangles[currTri].verts[0], &retValue->triangles[currTri].verts[1], &retValue->triangles[currTri].verts[2], &retValue->triangles[currTri].normal);
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
					NormalFromVertsFloat(&retValue->triangles[currTri].verts[0], &retValue->triangles[currTri].verts[1], &retValue->triangles[currTri].verts[2], &retValue->triangles[currTri].normal);
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
					NormalFromVertsFloat(&retValue->triangles[currTri].verts[0], &retValue->triangles[currTri].verts[1], &retValue->triangles[currTri].verts[2], &retValue->triangles[currTri].normal);
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
					NormalFromVertsFloat(&retValue->triangles[currTri].verts[0], &retValue->triangles[currTri].verts[1], &retValue->triangles[currTri].verts[2], &retValue->triangles[currTri].normal);
				}
				currTri += 1;
			}
		}
		currVertexGroup = (VertexHeader*)((uint32_t)(&(currVertexGroup->vertices)) + (uint32_t)(sizeof(Vertex) * (currVertexGroup->count)));
	}

	Vec3 AABBMin;
	Vec3Subtraction(&retValue->AABBPosition, &retValue->AABBBounds, &AABBMin);
	Vec3 AABBMax;
	Vec3Addition(&retValue->AABBPosition, &retValue->AABBBounds, &AABBMax);
	Vec3s AABBMins, AABBMaxs;
	Vec3ToShort(AABBMin, AABBMins);
	Vec3ToShort(AABBMax, AABBMaxs);
	GenerateBoundsForBlocks(&AABBMins, &AABBMaxs, retValue->blocks);
	for (int i = 0; i < 8; ++i) {
		GenerateOctree(&retValue->blocks[i], retValue, 0);
	}

	return retValue;
}

ITCM_CODE void FindTrianglesFromOctreeInternal(Vec3* min, Vec3* max, MeshCollider *mesh, CollisionBlock* block, unsigned short** retValue, int* maxSize, int* currSize) {
	if (block->subdivided) {
		for (int i = 0; i < 8; ++i) {
			Vec3 blockMin, blockMax;
			ShortToVec3(block->blocks[i].boundsMin, blockMin);
			ShortToVec3(block->blocks[i].boundsMax, blockMax);
			if (AABBCheck(min, max, &blockMin, &blockMax)) {
				FindTrianglesFromOctreeInternal(min, max, mesh, &block->blocks[i], retValue, maxSize, currSize);
			}
		}
	}
	else {
		// ensure we overlap the triangles
		for (int i = 0; i < block->triCount; ++i) {
			const CollisionTriangle* currTri = &mesh->triangles[block->triangleList[i]];
			Vec3 vMin, vMax;
			ShortToVec3(currTri->boundsMin, vMin);
			ShortToVec3(currTri->boundsMax, vMax);
			if (AABBCheck(min, max, &vMin, &vMax)) {
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
					retValue[0] = realloc(retValue[0], sizeof(unsigned short) * *maxSize);
#else
					if (*maxSize & 0x40000000) {
						*maxSize &= ~0x40000000;
						*maxSize += 512;
						retValue[0] = malloc(sizeof(unsigned short) * *maxSize);
						memcpy(retValue[0], staticCollisionAllocation, COLLISION_DTCM_SIZE);
					}
					else {
						*maxSize += 512;
						retValue[0] = realloc(retValue[0], sizeof(unsigned short) * *maxSize);
					}
#endif
				}
			}
		}
	}
}

// TODO: allow multiple calls within this to allocate DTCM simultaneously (perhaps just write our own simple DTCM allocator?)
ITCM_CODE unsigned short* FindTrianglesFromOctree(Vec3* min, Vec3* max, MeshCollider* meshCollider, int *totalTris) {
#ifndef COLLISION_DTCM
	unsigned short* retValue = (unsigned short*)malloc(sizeof(unsigned short) * 512);
	int maxSize = 512;
#else
	unsigned short* retValue = staticCollisionAllocation;
	int maxSize = COLLISION_DTCM_SIZE | 0x40000000;
#endif
	int currSize = 0;
	for (int i = 0; i < 8; ++i) {
		Vec3 blockMin, blockMax;
		ShortToVec3(meshCollider->blocks[i].boundsMin, blockMin);
		ShortToVec3(meshCollider->blocks[i].boundsMax, blockMax);
		if (AABBCheck(min, max, &blockMin, &blockMax)) {
			FindTrianglesFromOctreeInternal(min, max, meshCollider, &meshCollider->blocks[i], &retValue, &maxSize, &currSize);
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

void ReleaseTriangleOctreeAllocation(unsigned short *tris) {
#ifdef COLLISION_DTCM
	// no need to do anything...
	if (tris == staticCollisionAllocation) return;
#endif
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

void DestroyCollisionMesh(MeshCollider* meshCollider) {
	// destroy all the octrees...
	for (int i = 0; i < 8; ++i) {
		DestroyOctree(&meshCollider->blocks[i]);
	}
	free(meshCollider->triangles);
	free(meshCollider);
}

// doesn't return position by default since this should rarely be used by actual game code
ITCM_CODE bool RayOnAABB(Vec3* point, Vec3* direction, Vec3* boxMin, Vec3* boxMax, Vec3* normal, f32* t) {
	Vec3 workVec;
	Vec3 newDir = *direction;
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
	Vec3Subtraction(boxMin, point, &workVec);
	tMin1 = Int64Div(workVec.x, newDir.x);
	tMin2 = Int64Div(workVec.y, newDir.y);
	tMin3 = Int64Div(workVec.z, newDir.z);
	Vec3Subtraction(boxMax, point, &workVec);
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
	if (t != NULL) {
		*t = dist;
	}

	// return normal as well
	if (normal != NULL) {
		f32 tSeries[] = { tMin1, tMax1, tMin2, tMax2, tMin3, tMax3 };
		Vec3 normals[] = { {{{-4096, 0, 0 }}}, {{{4096, 0, 0}}},
			{{{0, -4096, 0}}}, {{{0, 4096, 0}}},
			{{{0, 0, -4096}}}, {{{0, 0, 4096}}} };
		for (int i = 0; i < 6; ++i) {
			if (dist == tSeries[i]) {
				*normal = normals[i];
				break;
			}
		}
	}

	return tNear <= tFar && tFar >= 0;
}

ITCM_CODE bool RayOnSphere(Vec3* point, Vec3* direction, CollisionSphere* sphere, f32* t, Vec3* hitPos) {
	// TODO: int64 version for DS?
	Vec3 dist;
	Vec3Subtraction(point, sphere->position, &dist);
	f32 b = DotProduct(&dist, direction);
	f32 c = DotProduct(&dist, &dist) - mulf32(sphere->radius, sphere->radius);
	// return if it's outside of the sphere and pointing the wrong way
	if (c > 0 && b > 0) return false;
	f32 discr = mulf32(b, b) - c;
	// negative discriminant means a miss
	if (discr < 0) return false;
	// ray found to intersect sphere, compute intersection
	f32 hitDist = -b - sqrtf32(discr);
	// if t is negative, started in sphere
	if (hitDist < 0) {
		hitDist = 0;
	}
	if (t != NULL) {
		*t = hitDist;
	}
	if (hitPos != NULL) {
		hitPos->x = point->x + mulf32(hitDist, direction->x);
		hitPos->y = point->y + mulf32(hitDist, direction->y);
		hitPos->z = point->z + mulf32(hitDist, direction->z);
	}
	return true;
}

ITCM_CODE bool RayOnPlane(Vec3* point, Vec3* direction, Vec3* normal, f32 planeDistance, f32* t, Vec3* hitPos) {
	// two normals, can safely not use 64 bit here
	f32 nd = DotProductNormal(direction, normal);
	f32 pn = DotProduct(point, normal);
	// if nd is positive, they're facing the same way. no collision
	if (nd >= 0) {
		return false;
	}
	f32 hitDist = divf32(planeDistance - pn, nd);
	if (hitDist >= 0) {
		if (t != NULL) {
			*t = hitDist;
		}
		if (hitPos != NULL) {
			hitPos->x = point->x + mulf32(direction->x, hitDist);
			hitPos->y = point->y + mulf32(direction->y, hitDist);
			hitPos->z = point->z + mulf32(direction->z, hitDist);
			return true;
		}
	}
	return false;
}

ITCM_CODE bool RayOnTriangle(Vec3* point, Vec3* direction, CollisionTriangle* triangle, f32* t, Vec3* hitPos) {
	Vec3 tmpHitPos;
	if (hitPos == NULL) {
		hitPos = &tmpHitPos;
	}
	Vec3 n, v;
	ShortToVec3(triangle->normal, n);
	ShortToVec3(triangle->verts[0], v);
	if (RayOnPlane(point, direction, &n, DotProduct(&n, &v), t, hitPos)) {
		Vec3 barycentric = BarycentricCoords(triangle, hitPos);
		if (barycentric.x >= 0 && barycentric.x <= 4096
			&& barycentric.y >= 0 && barycentric.y <= 4096
			&& barycentric.z >= 0 && barycentric.z <= 4096) {
			return true;
		}
	}

	return false;
}

ITCM_CODE int GetQuadTreeCountSub(CollisionBlock* block) {
	// potentially recode this to do the subdivided check before calling on it, to avoid extra function calls as a minor optimization
	int count = 0;
	if (block->subdivided) {
		for (int i = 0; i < 8; ++i) {
			count += GetQuadTreeCountSub(&block->blocks[i]);
		}
	}
	else {
		if (block->triCount > 0) {
			count = 1;
		}
	}
	return count;
}

ITCM_CODE int GetQuadTreeCount(MeshCollider* mesh) {
	int count = 0;
	for (int i = 0; i < 8; ++i) {
		count += GetQuadTreeCountSub(&mesh->blocks[i]);
	}
	return count;
}

ITCM_CODE void RaycastQuadTreeSub(Vec3* point, Vec3* direction, f32 length, Vec3* AABBMin, Vec3* AABBMax, CollisionBlock* block, CollisionBlock** hitBlocks, int* hitBlockPosition, int* triCount) {
	f32 t;
	if (!block->subdivided) {
		if (block->triCount > 0) {
			Vec3 bMin, bMax;
			ShortToVec3(block->boundsMin, bMin);
			ShortToVec3(block->boundsMax, bMax);
			if (AABBCheck(AABBMin, AABBMax, &bMin, &bMax)) {
				if (RayOnAABB(point, direction, &bMin, &bMax, NULL, &t)) {
					if (t <= length) {
						hitBlocks[*hitBlockPosition] = block;
						*hitBlockPosition += 1;
						*triCount += block->triCount;
					}
				}
			}
		}
	}
	else {
		for (int i = 0; i < 8; ++i) {
			RaycastQuadTreeSub(point, direction, length, AABBMin, AABBMax, &block->blocks[i], hitBlocks, hitBlockPosition, triCount);
		}
	}
}

ITCM_CODE void RaycastQuadTree(Vec3* point, Vec3* direction, f32 length, Vec3* AABBMin, Vec3* AABBMax, MeshCollider* mesh, CollisionBlock** hitBlocks, int* hitBlockPosition, int* triCount) {
	for (int i = 0; i < 8; ++i) {
		RaycastQuadTreeSub(point, direction, length, AABBMin, AABBMax, &mesh->blocks[i], hitBlocks, hitBlockPosition, triCount);
	}
}

ITCM_CODE bool RayOnMesh(Vec3* point, Vec3* direction, f32 length, Vec3* rayMin, Vec3* rayMax, MeshCollider* mesh, Vec3* meshOffset, Vec3* meshScale, Quaternion* meshRotation, f32* t, Vec3* hitPos, Vec3* normal, int* triId) {

	// attempt at optimization
	f32 maxBounds = Max(Max(mesh->AABBBounds.x, mesh->AABBBounds.y), mesh->AABBBounds.z);
	Vec3 oldMeshMax = { { {
		mulf32(mesh->AABBPosition.x + maxBounds, meshScale->x) + meshOffset->x,
		mulf32(mesh->AABBPosition.y + maxBounds, meshScale->y) + meshOffset->y,
		mulf32(mesh->AABBPosition.z + maxBounds, meshScale->z) + meshOffset->z
	} } };

	Vec3 oldMeshMin = { { {
		mulf32(mesh->AABBPosition.x - maxBounds, meshScale->x) + meshOffset->x,
		mulf32(mesh->AABBPosition.y - maxBounds, meshScale->y) + meshOffset->y,
		mulf32(mesh->AABBPosition.z - maxBounds, meshScale->z) + meshOffset->z
	} } };

	if (!AABBCheck(rayMin, rayMax, &oldMeshMin, &oldMeshMax)) {
		return false;
	}

	// transform the point and direction
	Vec3 newPoint, newDirection;
	Vec3 workVec;
	Vec3Subtraction(point, meshOffset, &newPoint);
	Quaternion invQuat;
	QuaternionInverse(meshRotation, &invQuat);
	QuatTimesVec3(&invQuat, &newPoint, &workVec);
	Vec3Division(&workVec, meshScale, &newPoint);

	// direction now
	QuatTimesVec3(&invQuat, direction, &workVec);
	Vec3Division(&workVec, meshScale, &newDirection);
	Normalize(&newDirection, &newDirection);

	// generate new length value
	Vec3 absDir = { { {
		f32abs(newDirection.x),
		f32abs(newDirection.y),
		f32abs(newDirection.z)
	} } };
	Vec3 absScale = { { {
		f32abs(meshScale->x),
		f32abs(meshScale->y),
		f32abs(meshScale->z)
	} } };
	f32 lenDot = DotProduct(&absDir, &absScale);
	f32 newLength = divf32(length, lenDot);

	Vec3 rayPlusDir = { { {
	newPoint.x + mulf32(newDirection.x, newLength),
	newPoint.y + mulf32(newDirection.y, newLength),
	newPoint.z + mulf32(newDirection.z, newLength)
	} } };

	Vec3 rayAABBMin = { { {
		Min(newPoint.x, rayPlusDir.x),
		Min(newPoint.y, rayPlusDir.y),
		Min(newPoint.z, rayPlusDir.z)
	} } };
	Vec3 rayAABBMax = { { {
		Max(newPoint.x, rayPlusDir.x),
		Max(newPoint.y, rayPlusDir.y),
		Max(newPoint.z, rayPlusDir.z)
	} } };

	// check against the mesh AABB first
	f32 tempt;
	Vec3 AABBMin, AABBMax;
	Vec3Subtraction(&mesh->AABBPosition, &mesh->AABBBounds, &AABBMin);
	Vec3Addition(&mesh->AABBPosition, &mesh->AABBBounds, &AABBMax);
	if (RayOnAABB(&newPoint, &newDirection, &AABBMin, &AABBMax, NULL, &tempt)) {
		if (tempt > newLength) {
			return false;
		}
	}
	else {
		// simple AABB point check
		if (!(newPoint.x >= AABBMin.x && newPoint.x <= AABBMax.x &&
			newPoint.y >= AABBMin.y && newPoint.y <= AABBMax.y &&
			newPoint.z >= AABBMin.z && newPoint.z <= AABBMax.z)) {
			return false;
		}
	}

	// now raycast against the quadtrees
	int quadTreeCount = GetQuadTreeCount(mesh);

	CollisionBlock** hitBlocks = (CollisionBlock**)malloc(sizeof(CollisionBlock*) * quadTreeCount);
	int hitBlockPosition = 0;
	int triCount = 0;
	
	RaycastQuadTree(&newPoint, &newDirection, newLength, &rayAABBMin, &rayAABBMax, mesh, hitBlocks, &hitBlockPosition, &triCount);
	
	// TODO: potential optimization, sort by the quadtree positions so we have to sort fewer individual hits if applicable

	// all blocks hit, now create a list of triangles, omitting duplicates
	int* trisToCheck = (int*)malloc(sizeof(int) * triCount);
	int realTriCount = 0;
	f32 closestHit = 2147483647;
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
			ShortToVec3(mesh->triangles[hitBlocks[i]->triangleList[j]].boundsMin, newBoundsMin);
			ShortToVec3(mesh->triangles[hitBlocks[i]->triangleList[j]].boundsMax, newBoundsMax);
			if (AABBCheck(&rayAABBMin, &rayAABBMax, &newBoundsMin, &newBoundsMax)) {
				//bool inAABB = (newPoint.x >= newBoundsMin->x && newPoint.x <= newBoundsMax->x &&
					//newPoint.y >= newBoundsMin->y && newPoint.y <= newBoundsMax->y &&
					//newPoint.z >= newBoundsMin->z && newPoint.z <= newBoundsMax->z);
				//if (inAABB || RayOnAABB(&newPoint, &newDirection, newBoundsMin, newBoundsMax, &tempt)) {
					//if (inAABB || tempt <= newLength) {
						if (RayOnTriangle(&newPoint, &newDirection, &mesh->triangles[hitBlocks[i]->triangleList[j]], &tempt, NULL)) {
							if (tempt < closestHit) {
								closestHit = tempt;
								closestTri = hitBlocks[i]->triangleList[j];
								everHit = true;
							}
						}
					//}
				//}
			}
		}
	}
	if (everHit) {
		if (t != NULL) {
			*t = mulf32(closestHit, lenDot);
		}
		if (hitPos != NULL) {
			f32 truLen;
			if (t != NULL) {
				truLen = *t;
			}
			else {
				truLen = divf32(closestHit, lenDot);
			}

			hitPos->x = point->x + mulf32(direction->x, truLen);
			hitPos->y = point->y + mulf32(direction->y, truLen);
			hitPos->z = point->z + mulf32(direction->z, truLen);
		}
		if (triId != NULL) {
			*triId = closestTri;
		}
		if (normal != NULL) {
			//*normal = mesh->triangles[closestTri].normal;
			ShortToVec3(mesh->triangles[closestTri].normal, *normal);
		}
	}
	free(trisToCheck);
	free(hitBlocks);
	return everHit;
}

ITCM_CODE void ClosestPointAABB(Vec3* position, Vec3* boxMin, Vec3* boxMax, Vec3* out) {
	*out = *position;

	out->x = (out->x < boxMin->x) ? boxMin->x : out->x;
	out->y = (out->y < boxMin->y) ? boxMin->y : out->y;
	out->z = (out->z < boxMin->z) ? boxMin->z : out->z;

	out->x = (out->x > boxMax->x) ? boxMax->x : out->x;
	out->y = (out->y > boxMax->y) ? boxMax->y : out->y;
	out->z = (out->z > boxMax->z) ? boxMax->z : out->z;
}

// essentially just sphere on AABB but applying inverse rotation to the sphere
ITCM_CODE bool SphereOnOBB(CollisionSphere* sphere, CollisionBox* box, Vec3* hitPos, Vec3* normal, f32* t) {
	Vec3 rotatedSpherePoint, workVec;
	Vec3Subtraction(sphere->position, box->position, &workVec);
	Quaternion invQuat;
	QuaternionInverse(box->rotation, &invQuat);
	QuatTimesVec3(&invQuat, &workVec, &rotatedSpherePoint);

	// center around 0
	Vec3 boxMin;
	Vec3 zeroVec = { { { 0, 0, 0 } } };
	Vec3Subtraction(&zeroVec, &box->extents, &boxMin);

	Vec3 closestPoint;
	ClosestPointAABB(&rotatedSpherePoint, &boxMin, &box->extents, &closestPoint);
	Vec3 closestRelativeToSphere;
	Vec3Subtraction(&rotatedSpherePoint, &closestPoint, &closestRelativeToSphere);
	f32 sqrDist = SqrMagnitude(&closestRelativeToSphere);

	if (sqrDist <= mulf32(sphere->radius, sphere->radius)) {
		*hitPos = closestPoint;
		// global space it again...
		Vec3Addition(hitPos, box->position, hitPos);
		if (sqrDist <= 1) {
			// we're INSIDE the cube, fix!
			Normalize(&rotatedSpherePoint, normal);

			// gross i know but nothing better came to me
			// 2867 = 0.7f
			const f32 normalCheck = 2867;
			if (normal->y >= normalCheck) {
				normal->x = 0;
				normal->y = 4096;
				normal->z = 0;
			} else if (normal->y <= -normalCheck) {
				normal->x = 0;
				normal->y = -4096;
				normal->z = 0;
			} else if (normal->x >= normalCheck) {
				normal->x = 4096;
				normal->y = 0;
				normal->z = 0;
			} else if (normal->x <= -normalCheck) {
				normal->y = 0;
				normal->x = -4096;
				normal->z = 0;
			} else if (normal->z >= normalCheck) {
				normal->x = 0;
				normal->z = 4096;
				normal->y = 0;
			}
			else {
				normal->x = 0;
				normal->y = 0;
				normal->z = -4096;
			}
			// this also sucks
			*t = sphere->radius + f32abs(DotProduct(normal, &box->extents)) - DotProduct(normal, &rotatedSpherePoint);
		}
		else {
			// we're outside of cube, return values
			Normalize(&closestRelativeToSphere, normal);
			*t = sphere->radius - sqrtf32(sqrDist);
		}
		Vec3 tmpNormal;
		QuatTimesVec3(box->rotation, normal, &tmpNormal);
		*normal = tmpNormal;
		return true;
	}
	return false;
}

void TripleCrossProduct(Vec3* a, Vec3* b, Vec3* c, Vec3* out) {
	Vec3 work;
	CrossProduct(a, b, &work);
	CrossProduct(&work, c, out);
}

f32 LineDistFromOrigin(Vec3 *d, Vec3 *a, Vec3 *b) {
	// a lot of magnitudes here...actually largely slower because of the 64 bit math rather than sqrts...
	f32 t = -DotProduct(a, d);
	t = divf32(t, Magnitude(d));

	if (t <= 0) {
		// off the line from a
		return Magnitude(a);
	}
	else if (t >= 4096) {
		// off the line from b
		return Magnitude(b);
	}
	else {
		Vec3 work;
		work.x = mulf32(d->x, t);
		work.y = mulf32(d->y, t);
		work.z = mulf32(d->z, t);
		Vec3Addition(&work, a, &work);
		return Magnitude(&work);
	}
}

f32 OriginDistTri(Vec3* a, Vec3* b, Vec3* c) {

	Vec3 d0, d1, d2;

	Vec3Subtraction(b, a, &d1);
	Vec3Subtraction(c, a, &d2);

	d0.x = -a->x;
	d0.y = -a->y;
	d0.z = -a->z;

	// specialized barycentric coordinates...
	f32 d11 = SqrMagnitude(&d1);
	f32 d22 = SqrMagnitude(&d2);
	f32 d01 = DotProduct(&d0, &d1);
	f32 d20 = DotProduct(&d2, &d0);
	f32 d21 = DotProduct(&d2, &d1);

	f32 denom = mulf32(d11, d22) - mulf32(d21, d21);
	f32 s, t;
	if (denom == 0) {
		// degenerate triangle error handler, just check the lines/points
		s = -1;
		t = -1;
	}
	else {
		s = divf32(mulf32(d20, d21) - mulf32(d22, d01), denom);
		t = divf32(mulf32(-s, d21) - d20, d22);
	}

	f32 dist;

	if (s >= 0
		&& t >= 0
		&& t + s <= 4096) {

		dist = mulf32(mulf32(s, s), d11);
		dist += mulf32(mulf32(t, t), d22);
		dist += mulf32(mulf32(s * 2, t), d21);
		dist += mulf32(s * 2, d01);
		dist += mulf32(t * 2, d20);
		dist += SqrMagnitude(a);
	}
	else {
		// find line that's closest to origin; just slightly pared down compared to sphere code tbh
		dist = LineDistFromOrigin(&d1, a, b);

		f32 dist2 = LineDistFromOrigin(&d2, a, c);
		if (dist2 < dist) {
			dist = dist2;
		}

		Vec3 d3;
		Vec3Subtraction(c, b, &d3);
		dist2 = LineDistFromOrigin(&d3, b, c);
		if (dist2 < dist) {
			dist = dist2;
		}
	}

	return dist;
}

f32 PointDistTri(Vec3* point, Vec3* a, Vec3* b, Vec3* c) {
	Vec3 at, bt, ct;
	Vec3Subtraction(a, point, &at);
	Vec3Subtraction(b, point, &bt);
	Vec3Subtraction(c, point, &ct);
	return OriginDistTri(&at, &bt, &ct);
}

int GJKProcessSimplex2(Simplex* simplex, Vec3* normal) {
	Vec3* a = &simplex->points[1];
	Vec3* b = &simplex->points[0];
	Vec3 ab, ao;
	Vec3Subtraction(b, a, &ab);
	ao.x = -a->x;
	ao.y = -a->y;
	ao.z = -a->z;

	f32 dot = DotProduct(&ab, &ao);

	Vec3 tmp;
	CrossProduct(&ab, &ao, &tmp);

	if (dot > 0 && f32abs(SqrMagnitude(&tmp)) <= GJK_LENIENCY) {
		return 1;
	}

	// get new direction
	if (dot <= 0) {
		// not crossing ab...
		simplex->count = 1;
		simplex->points[0] = *a;
		*normal = ao;
	}
	else {
		TripleCrossProduct(&ab, &ao, &ab, normal);
	}
	return 0;
}

int GJKProcessSimplex3(Simplex* simplex, Vec3* normal)
{
	Vec3* a = &simplex->points[2];
	Vec3* b = &simplex->points[1];
	Vec3* c = &simplex->points[0];

	// if the triangle is degenerate, then we can't expand the simplex, and therefor no intersection.
	if (VecEqual(a, b) || VecEqual(a, c)) {
		return 2;
	}

	// check if origin is touching; i wouldn't do this normally, but this saves us an iteration under many circumstances, so it's worth it to me
	//f32 dist = OriginDistTri(a, b, c);
	//if (f32abs(dist) < GJK_LENIENCY) {
	//	return 1;
	//}

	// relative to origin...
	Vec3 ao;
	ao.x = -a->x;
	ao.y = -a->y;
	ao.z = -a->z;

	// get the line segments and normal
	Vec3 ab, ac;
	Vec3Subtraction(b, a, &ab);
	Vec3Subtraction(c, a, &ac);
	Vec3 abc;
	CrossProduct(&ab, &ac, &abc);

	Vec3 tmp;
	CrossProduct(&abc, &ac, &tmp);
	f32 dot = DotProduct(&tmp, &ao);
	if (dot >= 0) {
		dot = DotProduct(&ac, &ao);
		if (dot >= 0) {
			simplex->points[1] = *a;
			simplex->count = 2;
			TripleCrossProduct(&ac, &ao, &ac, normal);
			return 0;
		}
		else {
			dot = DotProduct(&ab, &ao);
			if (dot >= 0) {
				// a is guaranteed to be safe here, even though it's a pointer into the simplex
				simplex->points[0] = *b;
				simplex->points[1] = *a;
				simplex->count = 2;
				TripleCrossProduct(&ab, &ao, &ab, normal);
				return 0;
			}
			else {
				// work with latest added point...
				simplex->points[0] = *a;
				simplex->count = 1;
				*normal = *a;
				return 0;
			}
		}
	}
	else {
		CrossProduct(&ab, &abc, &tmp);
		dot = DotProduct(&tmp, &ao);
		// copy + pasted from earlier, kinda bad tbh
		if (dot >= 0) {
			dot = DotProduct(&ab, &ao);
			if (dot >= 0) {
				// a is guaranteed to be safe here, even though it's a pointer into the simplex
				simplex->points[0] = *b;
				simplex->points[1] = *a;
				simplex->count = 2;
				TripleCrossProduct(&ab, &ao, &ab, normal);
				return 0;
			}
			else {
				// work with latest added point...
				simplex->points[0] = *a;
				simplex->count = 1;
				*normal = *a;
			}
		}
		else {
			dot = DotProduct(&abc, &ao);
			if (dot >= 0) {
				*normal = abc;
				return 0;
			}
			else {
				tmp = *c;
				// have to swap b and c...
				simplex->points[0] = *b;
				simplex->points[1] = tmp;
				// go find a point opposite the triangle we have
				normal->x = -abc.x;
				normal->y = -abc.y;
				normal->z = -abc.z;
				return 0;
			}
		}
	}
}

int GJKProcessSimplex4(Simplex* simplex, Vec3* normal) {

	Vec3* a = &simplex->points[3];
	Vec3* b = &simplex->points[2];
	Vec3* c = &simplex->points[1];
	Vec3* d = &simplex->points[0];

	// if the simplex is degenerate, it can't be a collision
	f32 dist = PointDistTri(a, b, c, d);
	if (f32abs(dist) < GJK_LENIENCY) {
		return 2;
	}

	// compute normals for each face, and lines that make them up

	Vec3 ab, ac, ad;
	Vec3Subtraction(b, a, &ab);
	Vec3Subtraction(c, a, &ac);
	Vec3Subtraction(d, a, &ad);
	Vec3 abc, acd, adb;
	CrossProduct(&ab, &ac, &abc);
	CrossProduct(&ac, &ad, &acd);
	CrossProduct(&ad, &ab, &adb);

	Vec3 ao;
	ao.x = -a->x;
	ao.y = -a->y;
	ao.z = -a->z;

	// figure out the direction relative to the face
	bool b_on_acd = DotProduct(&acd, &ab) >= 0;
	bool c_on_adb = DotProduct(&adb, &ac) >= 0;
	bool d_on_abc = DotProduct(&abc, &ad) >= 0;

	// now use the sign and do the same thing again
	bool ab_o = DotProduct(&acd, &ao) >= 0 == b_on_acd;
	bool ac_o = DotProduct(&adb, &ao) >= 0 == c_on_adb;
	bool ad_o = DotProduct(&abc, &ao) >= 0 == d_on_abc;

	if (ab_o && ac_o && ad_o) {
		// origin is in simplex! we're done!
		return 1;
	}
	else if (!ab_o) {
		// B is wrong, discard

		simplex->points[2] = *a;
	}
	else if (!ac_o) {
		// C is wrong, discard
		simplex->points[1] = *d;
		simplex->points[0] = *b;
		simplex->points[2] = *a;
	}
	else {
		// D is wrong, discard
		simplex->points[0] = *c;
		simplex->points[1] = *b;
		simplex->points[2] = *a;
	}

	simplex->count = 3;

	return GJKProcessSimplex3(simplex, normal);
}

int GJKProcessSimplex(Simplex* simplex, Vec3* normal) {
	switch (simplex->count) {
	case 2:
		return GJKProcessSimplex2(simplex, normal);
		break;
	case 3:
		return GJKProcessSimplex3(simplex, normal);
		break;
	default:
		return GJKProcessSimplex4(simplex, normal);
		break;
	}
	return 2; // ??
}

// normalBetweenShape1Shape2 expected in the form of shape2-shape1
bool BasicGJK(void *shape1, void *shape2, Vec3* normalBetweenShape1Shape2, Simplex* outputSimplex,
	Vec3(*findPointSupport1)(void* shape, Vec3 *normal), Vec3(*findPointSupport2)(void* shape, Vec3 *normal)) {
	Vec3 p1, p2;
	p1 = findPointSupport1(shape1, normalBetweenShape1Shape2);
	Vec3 normal;
	normal.x = -normalBetweenShape1Shape2->x;
	normal.y = -normalBetweenShape1Shape2->y;
	normal.z = -normalBetweenShape1Shape2->z;
	p2 = findPointSupport2(shape2, &normal);
	Vec3Subtraction(&p1, &p2, &outputSimplex->points[0]);
	// normalize this here so we can just target origin
	Normalize(&outputSimplex->points[0], &normal);
	outputSimplex->count = 1;
	normal.x = -normal.x;
	normal.y = -normal.y;
	normal.z = -normal.z;
	// should be enough to cover every use case...
	for (int i = 0; i < 16; ++i) {
		p1 = findPointSupport2(shape1, &normal);
		Vec3 normal2;
		normal2.x = -normal.x;
		normal2.y = -normal.y;
		normal2.z = -normal.z;
		p2 = findPointSupport1(shape2, &normal2);
		Vec3Subtraction(&p1, &p2, &outputSimplex->points[outputSimplex->count]);
		// early out: no point towards origin, therefor simplex can not contain origin
		if (DotProduct(&normal, &outputSimplex->points[outputSimplex->count]) < 0) {
			return false;
		}

		++outputSimplex->count;

		int simplexStatus = GJKProcessSimplex(outputSimplex, &normal);

		if (simplexStatus == 1) {
			return true;
		}
		else if (simplexStatus == 2) {
			// no intersection :(
			return false;
		}
		Normalize(&normal, &normal);
	}
	// TODO: add loop limit? idk, unlikely anyone will use this function directly tbh
	return false;
}