#include "sdobject.h"
#include "sdcollision.h"
#include "sddelta.h"
#include <stdio.h>
#include <nds.h>
#include <stdlib.h>
#include "sdsound.h"

Object *Object::first;

ObjectPtr* networkedObjects;
int networkedObjectsCount;

bool layerCollision[1024];

bool Object::RaycastWorld(RayCast& rayObject, unsigned int layerMask, CollisionHit* hitInfo) {
	Object* colObject = first;

	bool everHit = false;
	Fixed closestHit = rayObject.length;
	Object* closestObject = NULL;
	int closestHitType = 0;
	int closestTriHit;
	Vec3 closestNormal;

	Vec3 rayPlusDir = rayObject.point + rayObject.dir * rayObject.length;

	Vec3 rayAABBMin = {
		Min(rayObject.point.x, rayPlusDir.x),
		Min(rayObject.point.y, rayPlusDir.y),
		Min(rayObject.point.z, rayPlusDir.z)
	};
	Vec3 rayAABBMax = {
		Max(rayObject.point.x, rayPlusDir.x),
		Max(rayObject.point.y, rayPlusDir.y),
		Max(rayObject.point.z, rayPlusDir.z)
	};

	while (colObject != NULL) {
		// ensure object is solid and on the layer mask
		if (colObject->solid && ((1 << (colObject->layer - 1)) & layerMask) != 0) {
			BasicCollisionInfo tempInfo;
			int tempTri;
			if (colObject->sphereCol != NULL) {
				// oh boy, raycast against S P H E R E
				if (rayObject.OnSphere(*colObject->sphereCol, &tempInfo)) {
					if (tempInfo.t <= closestHit) {
						everHit = true;
						closestHit = tempInfo.t;
						closestTriHit = -1;
						closestNormal = tempInfo.normal;
						closestHitType = COLLIDER_SPHERE;
						closestObject = colObject;
					}
				}
			}
			if (colObject->meshCol != NULL) {
				if ((tempTri = rayObject.OnMesh(*colObject->meshCol, colObject->position, colObject->scale, colObject->rotation, &tempInfo)) != -1) {
					if (tempInfo.t <= closestHit) {
						everHit = true;
						closestTriHit = tempTri;
						closestNormal = tempInfo.normal;
						closestHit = tempInfo.t;
						closestHitType = COLLIDER_MESH;
						closestObject = colObject;
					}
				}
			}
			if (colObject->boxCol != NULL) {
				Vec3 boxMin = -colObject->boxCol->extents;
				// rotate point and direction
				Vec3 rotatedPoint, workVec;
				workVec = rayObject.point - *colObject->boxCol->position;
				Quaternion inverseRot = colObject->boxCol->rotation->Inverse();
				rotatedPoint = inverseRot * workVec;
				Vec3 rotatedDir = inverseRot * rayObject.dir;
				RayCast tempRay(rotatedPoint, rotatedDir, rayObject.length);
				// ray on AABB
				if (tempRay.OnAABB(boxMin, colObject->boxCol->extents, &tempInfo)) {
					if (tempInfo.t <= closestHit) {
						everHit = true;
						closestNormal = *colObject->boxCol->rotation * tempInfo.normal;
						closestHit = tempInfo.t;
						closestHitType = COLLIDER_BOX;
						closestObject = colObject;
						closestTriHit = -1;
					}
				}
			}
		}
		colObject = colObject->next;
	}

	if (everHit) {
		if (hitInfo != NULL) {
			hitInfo->position = rayObject.point + rayObject.dir * closestHit;
			hitInfo->normal = closestNormal;
			hitInfo->hitObject = closestObject;
			hitInfo->hitTri = closestTriHit;
			hitInfo->colliderType = closestHitType;
			hitInfo->t = closestHit;
		}
		return true;
	}
	return false;
}

int Object::SphereCollisionCheck(SphereCollider *sphere, unsigned int layerMask, CollisionHit* hitInfos, int maxHit) {
	// iterate over all objects and get mesh colliders
	Object *meshObject = first;
	// you MUST pass some collision storage to use function, sowwy
	if (maxHit <= 0) {
		return 0;
	}
	int objsFound = 0;
	while (meshObject != NULL) {
		if (meshObject->meshCol != NULL && meshObject->solid && ((1 << (meshObject->layer - 1)) & layerMask) != 0) {
			// apply blockmap
			Vec3 rotatedPosition;
			Quaternion inverseObjectRotation = meshObject->rotation.Inverse();
			rotatedPosition = inverseObjectRotation * *sphere->position;
			// adjust the position to inverse the transformation of the mesh object
			rotatedPosition = rotatedPosition - meshObject->position;
			// and scale
			rotatedPosition = rotatedPosition / meshObject->scale;
			// radius...
			Fixed newRadius = sphere->radius / meshObject->scale.x;
			// are we INSIDE the AABB?
			if ((rotatedPosition.x - meshObject->meshCol->AABBPosition.x).fabs() > meshObject->meshCol->AABBBounds.x + newRadius ||
			(rotatedPosition.y - meshObject->meshCol->AABBPosition.y).fabs() > meshObject->meshCol->AABBBounds.y + newRadius ||
			(rotatedPosition.z - meshObject->meshCol->AABBPosition.z).fabs() > meshObject->meshCol->AABBBounds.z + newRadius) {
				meshObject = meshObject->next;
				continue;
			}
			SphereCollider newSphere(&rotatedPosition, newRadius);
			int totalTris = 0;
			Vec3 min;
			Vec3 max;
			min.x = newSphere.position->x - newSphere.radius;
			min.y = newSphere.position->y - newSphere.radius;
			min.z = newSphere.position->z - newSphere.radius;
			max.x = newSphere.position->x + newSphere.radius;
			max.y = newSphere.position->y + newSphere.radius;
			max.z = newSphere.position->z + newSphere.radius;
			unsigned short* trisToCollideWith = meshObject->meshCol->FindTrianglesFromOctree(min, max, &totalTris);
			// now check them once more for whether they're on the plane
			for (int i = 0; i < totalTris; ++i) {
				bool onPlane;
				if ((onPlane = newSphere.OnTrianglePlane(meshObject->meshCol->triangles[trisToCollideWith[i]], &hitInfos[objsFound]))) {
					hitInfos[objsFound].hitTri = trisToCollideWith[i];
					hitInfos[objsFound].hitObject = meshObject;
					// scale pen by object scale
					hitInfos[objsFound].penetration = mulf32(hitInfos[objsFound].penetration, meshObject->scale.x);
					hitInfos[objsFound].colliderType = COLLIDER_MESH;
					++objsFound;
					if (objsFound >= maxHit) return objsFound;
					trisToCollideWith[i] = -1;
				}
				if (!onPlane) {
					trisToCollideWith[i] = -1;
				}
			}
			// lines...
			for (int i = 0; i < totalTris; ++i) {
				if (trisToCollideWith[i] != -1 && newSphere.OnTriangleLines(meshObject->meshCol->triangles[trisToCollideWith[i]], &hitInfos[objsFound])) {
					hitInfos[objsFound].hitTri = trisToCollideWith[i];
					hitInfos[objsFound].hitObject = meshObject;
					// scale pen by object scale
					hitInfos[objsFound].penetration = mulf32(hitInfos[objsFound].penetration, meshObject->scale.x);
					hitInfos[objsFound].colliderType = COLLIDER_MESH;
					++objsFound;
					if (objsFound >= maxHit) return objsFound;
					trisToCollideWith[i] = -1;
				}
			}
			// verts
			Fixed sphereRadiusSqr = mulf32(newSphere.radius, newSphere.radius);
			for (int i = 0; i < totalTris; ++i) {
				if (trisToCollideWith[i] != -1 && newSphere.OnTriangleVertices(meshObject->meshCol->triangles[trisToCollideWith[i]], &hitInfos[objsFound])) {
					hitInfos[objsFound].hitTri = trisToCollideWith[i];
					hitInfos[objsFound].hitObject = meshObject;
					// scale pen by object scale
					hitInfos[objsFound].penetration = mulf32(hitInfos[objsFound].penetration, meshObject->scale.x);
					hitInfos[objsFound].colliderType = COLLIDER_MESH;
					++objsFound;
					if (objsFound >= maxHit) return objsFound;
				}
			}
			MeshCollider::ReleaseTriangleOctreeAllocation(trisToCollideWith);
		}
		else if (meshObject->sphereCol != NULL) {
			if (sphere->OnSphere(*meshObject->sphereCol, &hitInfos[objsFound])) {
				hitInfos[objsFound].hitTri = -1;
				hitInfos[objsFound].hitObject = meshObject;
				hitInfos[objsFound].colliderType = COLLIDER_SPHERE;
				Fixed positionValue = sphere->radius - hitInfos[objsFound].penetration;
				++objsFound;
				if (objsFound >= maxHit) return objsFound;
			}
		}
		else if (meshObject->boxCol != NULL) {
			if (sphere->OnOBB(*meshObject->boxCol, &hitInfos[objsFound])) {
				hitInfos[objsFound].hitTri = -1;
				hitInfos[objsFound].hitObject = meshObject;
				hitInfos[objsFound].colliderType = COLLIDER_BOX;
				++objsFound;
				if (objsFound >= maxHit) return objsFound;
			}
		}
		meshObject = meshObject->next;
	}
	return objsFound;
}

void MoveObjectOut(Fixed penetration, Vec3 *normal, Fixed newPenetration, Vec3 *newNormal, SphereCollider *sphere, Object *meshObject, SphereCollider *newSphere) {
	// adjust our local space
	*newNormal = *newNormal * newPenetration;
	*newSphere->position = *newSphere->position + *newNormal;
	*normal = *normal * penetration;
	*sphere->position = *sphere->position + *normal;
}

void SphereObjOnMeshObj(SphereCollider *sphere, Object *meshObject, Object *sphereObject) {
	// apply blockmap
	Vec3 rotatedPosition;
	Vec3 tmpPos;
	// radius...
	Fixed newRadius = sphere->radius / meshObject->scale.x;
	// the quaternion operations here are actually rather expensive, so let's early out extra early. just be crazy lenient.
	// potential solution: bounding sphere instead of bounding box?
	Fixed AABBBounds = Max(meshObject->meshCol->AABBBounds.x, Max(meshObject->meshCol->AABBBounds.y, meshObject->meshCol->AABBBounds.z));
	Fixed AABBScaling = Max(meshObject->scale.x, Max(meshObject->scale.y, meshObject->scale.z));
	AABBBounds = mulf32(mulf32(AABBBounds, AABBScaling), 6144) + newRadius;
	if ((sphere->position->x - (meshObject->meshCol->AABBPosition.x + meshObject->position.x)).fabs() > AABBBounds ||
		(sphere->position->y - (meshObject->meshCol->AABBPosition.y + meshObject->position.y)).fabs() > AABBBounds ||
		(sphere->position->z - (meshObject->meshCol->AABBPosition.z + meshObject->position.z)).fabs() > AABBBounds) {
		return;
	}
	Quaternion inverseObjectRotation = meshObject->rotation.Inverse();
	tmpPos = *sphere->position - meshObject->position;
	rotatedPosition = inverseObjectRotation * tmpPos;
	// adjust the position to inverse the transformation of the mesh object
	// and scale
	rotatedPosition = rotatedPosition / meshObject->scale;
	// are we INSIDE the AABB?
	if ((rotatedPosition.x - meshObject->meshCol->AABBPosition.x).fabs() > meshObject->meshCol->AABBBounds.x + newRadius ||
	(rotatedPosition.y - meshObject->meshCol->AABBPosition.y).fabs() > meshObject->meshCol->AABBBounds.y + newRadius ||
	(rotatedPosition.z - meshObject->meshCol->AABBPosition.z).fabs() > meshObject->meshCol->AABBBounds.z + newRadius) {
		return;
	}
	SphereCollider newSphere(&rotatedPosition, newRadius);
	int totalTris = 0;
	Vec3 min;
	Vec3 max;
	min.x = newSphere.position->x - newSphere.radius;
	min.y = newSphere.position->y - newSphere.radius;
	min.z = newSphere.position->z - newSphere.radius;
	max.x = newSphere.position->x + newSphere.radius;
	max.y = newSphere.position->y + newSphere.radius;
	max.z = newSphere.position->z + newSphere.radius;
	unsigned short* trisToCollideWith = meshObject->meshCol->FindTrianglesFromOctree(min, max, &totalTris);
	CollisionHit hitInfo;
	hitInfo.colliderType = COLLIDER_MESH;
	hitInfo.hitObject = meshObject;
	// now check them once more for whether they're on the plane
	// we check them one at a time like this because otherwise you'll get caught up on lines and verts when you should
	// be on a flat surface
	for (int i = 0; i < totalTris; ++i) {
		SphereCollider::TriIntersectStatus onPlane;
		if ((onPlane = newSphere.OnTrianglePlane(meshObject->meshCol->triangles[trisToCollideWith[i]], &hitInfo)) == SphereCollider::TriIntersectStatus::YES_COLLIDE) {
			hitInfo.hitTri = trisToCollideWith[i];
			Fixed localPen = hitInfo.penetration;
			Vec3 localNormal = hitInfo.normal.Normalize();
			hitInfo.penetration = mulf32(meshObject->scale.x, hitInfo.penetration);
			// improve accuracy...
			hitInfo.normal = meshObject->rotation * localNormal;
			hitInfo.position = (-hitInfo.normal * (sphere->radius - hitInfo.penetration)) + *sphere->position;
			if (sphereObject->Collide(hitInfo))
				MoveObjectOut(hitInfo.penetration, &hitInfo.normal, localPen, &localNormal, sphere, meshObject, &newSphere);
			trisToCollideWith[i] = 0xFFFF;
		}
		if (onPlane == SphereCollider::TriIntersectStatus::NO_COLLIDE) {
			trisToCollideWith[i] = 0xFFFF;
		}
	}
	// lines...
	for (int i = 0; i < totalTris; ++i) {
		if (trisToCollideWith[i] != 0xFFFF && newSphere.OnTriangleLines(meshObject->meshCol->triangles[trisToCollideWith[i]], &hitInfo)) {
			hitInfo.hitTri = trisToCollideWith[i];
			Fixed localPen = hitInfo.penetration;
			Vec3 localNormal = hitInfo.normal.Normalize();
			hitInfo.penetration = mulf32(meshObject->scale.x, localPen);
			// improve accuracy...
			hitInfo.normal = meshObject->rotation * localNormal;
			hitInfo.position = (-hitInfo.normal * (sphere->radius - hitInfo.penetration)) + *sphere->position;
			if (sphereObject->Collide(hitInfo))
				MoveObjectOut(hitInfo.penetration, &hitInfo.normal, localPen, &localNormal, sphere, meshObject, &newSphere);
			trisToCollideWith[i] = 0xFFFF;
		}
	}
	// verts
	Fixed sphereRadiusSqr = mulf32(newSphere.radius, newSphere.radius);
	for (int i = 0; i < totalTris; ++i) {
		if (trisToCollideWith[i] != 0xFFFF && newSphere.OnTriangleVertices(meshObject->meshCol->triangles[trisToCollideWith[i]], &hitInfo)) {
			hitInfo.hitTri = trisToCollideWith[i];
			Fixed localPen = hitInfo.penetration;
			Vec3 localNormal = hitInfo.normal.Normalize();
			hitInfo.penetration = mulf32(meshObject->scale.x, localPen);
			// improve accuracy...
			hitInfo.normal = meshObject->rotation * localNormal;
			hitInfo.position = (-hitInfo.normal * (sphere->radius - hitInfo.penetration)) + *sphere->position;
			if (sphereObject->Collide(hitInfo))
				MoveObjectOut(hitInfo.penetration, &hitInfo.normal, localPen, &localNormal, sphere, meshObject, &newSphere);
		}
	}
	MeshCollider::ReleaseTriangleOctreeAllocation(trisToCollideWith);
}

void SphereObjOnSphereObj(Object *collider, Object* collidee) {
	CollisionHit tempHit;
	if (collider->sphereCol->OnSphere(*collidee->sphereCol, &tempHit)) {
		CollisionHit hitInfo;
		hitInfo.colliderType = COLLIDER_SPHERE;
		hitInfo.normal = tempHit.normal;
		hitInfo.penetration = tempHit.penetration;
		hitInfo.hitObject = collidee;
		hitInfo.position = tempHit.position;
		hitInfo.hitTri = -1;
		if (collider->Collide(hitInfo)) {
			collider->position = collider->position + hitInfo.normal * hitInfo.penetration;
		}
	}
}

void SphereObjOnBoxObj(Object* collider, Object* collidee) {
	// first, do AABB check
	Fixed maxExtents = mulf32(Max(collidee->boxCol->extents.x, Max(collidee->boxCol->extents.y, collidee->boxCol->extents.z)), 4096 + 2048);
	Fixed extentsPlusRadius = maxExtents + collider->sphereCol->radius;

	if ((collider->sphereCol->position->x - collidee->boxCol->position->x).fabs() > extentsPlusRadius ||
		(collider->sphereCol->position->y - collidee->boxCol->position->y).fabs() > extentsPlusRadius ||
		(collider->sphereCol->position->z - collidee->boxCol->position->z).fabs() > extentsPlusRadius) {
		return;
	}

	// okay great, perform actual collision check
	CollisionHit hitInfo;
	if (collider->sphereCol->OnOBB(*collidee->boxCol, &hitInfo)) {
		hitInfo.colliderType = COLLIDER_BOX;
		hitInfo.hitObject = collidee;
		if (collider->Collide(hitInfo)) {
			Vec3 tmpNormal = hitInfo.normal * hitInfo.penetration;
			// move out
			*collider->sphereCol->position = *collider->sphereCol->position + tmpNormal;
		}
	}
}

int Object::GetObjectsOfType(int type, Object **out, int maxObjects) {
	Object *currObject = first;
	int currIdx = 0;
	while (currObject != NULL) {
		if (currObject->GetObjectType() == type) {
			out[currIdx] = currObject;
			++currIdx;
			if (currIdx >= maxObjects) {
				return currIdx;
			}
		}
		currObject = currObject->next;
	}
	return currIdx;
}

ITCM_CODE void Object::ProcessObjects() {

	Object* currObject = first;
	while (currObject != NULL) {
		// TODO; functions
		if (currObject->active) {
			currObject->Update();
		}
		// handle physics
		// two step movement
		if (currObject->moves && currObject->active) {
			Vec3 stepVelocity;
			if (SDTime::deltaTimeEngine) {
				stepVelocity.x = mulf32(currObject->velocity.x.value / 2, SDTime::deltaTime);
				stepVelocity.y = mulf32(currObject->velocity.y.value / 2, SDTime::deltaTime);
				stepVelocity.z = mulf32(currObject->velocity.z.value / 2, SDTime::deltaTime);

			}
			else {
				stepVelocity.x = currObject->velocity.x.value / 2;
				stepVelocity.y = currObject->velocity.y.value / 2;
				stepVelocity.z = currObject->velocity.z.value / 2;
			}
			for (int i = 0; i < 2; ++i) {
				currObject->position = stepVelocity + currObject->position;
				if (currObject->sphereCol != NULL) {
					// iterate over all objects and get mesh colliders
					Object* colObject = first;
					while (colObject != NULL) {
						if (colObject != currObject) {
							if (colObject->solid && layerCollision[currObject->layer + (colObject->layer * 32)]) {
								if (colObject->meshCol != NULL) {
									SphereObjOnMeshObj(currObject->sphereCol, colObject, currObject);
								}
								if (colObject->sphereCol != NULL) {
									SphereObjOnSphereObj(currObject, colObject);
								}
								if (colObject->boxCol != NULL) {
									SphereObjOnBoxObj(currObject, colObject);
								}
							}
						}
						colObject = colObject->next;
					}

				}
			}
		}
		currObject = currObject->next;
	}

	//late update
	currObject = first;
	while (currObject != NULL) {
		// update animators before late update procs so late updaters can do what they want with it
		if (currObject->mesh != NULL && !currObject->culled) {
			if (currObject->mesh->skeletonCount != 0 && currObject->animator != NULL) {
				if (SDTime::deltaTimeEngine) {
					Fixed tmp = currObject->animator->speed;
					currObject->animator->speed = mulf32(tmp.value * 60, SDTime::deltaTime);
					UpdateAnimator(currObject->animator, currObject->mesh);
					currObject->animator->speed = tmp;
				}
				else {
					UpdateAnimator(currObject->animator, currObject->mesh);
				}
			}
		}
		if (currObject->active) {
			currObject->LateUpdate();
		}
		currObject = currObject->next;
	}

	// do rendering, now
	glClearPolyID(0x1F);
	glClearColor(0, 0, 0, 0x1F);
	short* const BG0OFFS = (short*)0x04000010;
	if (!multipassRendering) {
		*BG0OFFS = 0;
		// set up the camera
		SetupCameraMatrix();
		// disable capture
		REG_DISPCAPCNT = 0;
		RenderModelQueue(true);
		currObject = first;
		while (currObject != NULL) {
			if (currObject->mesh != NULL && !currObject->culled) {
				if (currObject->mesh->skeletonCount != 0 && currObject->animator != NULL) {
					RenderModelRigged(currObject->mesh, &currObject->position, &currObject->scale, &currObject->rotation, NULL, currObject->animator, currObject->renderPriority);
				}
				else {
					RenderModel(currObject->mesh, &currObject->position, &currObject->scale, &currObject->rotation, NULL, currObject->renderPriority);
				}
			}
			currObject = currObject->next;
		}
		RenderTransparentModels();
		FinalizeSprites();
		bgUpdate();
		glFlush(GL_TRANS_MANUALSORT);
		threadWaitForVBlank();
		// update music
		UpdateMusicBuffer();
	}
	else {
		// we have to start by rendering the left half, then the right half
		SetupCameraMatrixPartial(128, 0, 128, 192);
		int targetBank = 3;
		// okay, now we render twice regularly
		for (int i = 0; i < 2; ++i) {
			RenderModelQueue(i == 1);
			currObject = first;
			while (currObject != NULL) {
				if (currObject->mesh != NULL && !currObject->culled) {
					if (currObject->mesh->skeletonCount != 0 && currObject->animator != NULL) {
						RenderModelRigged(currObject->mesh, &currObject->position, &currObject->scale, &currObject->rotation, NULL, currObject->animator, currObject->renderPriority);
					}
					else {
						RenderModel(currObject->mesh, &currObject->position, &currObject->scale, &currObject->rotation, NULL, currObject->renderPriority);
					}
				}
				currObject = currObject->next;
			}
			RenderTransparentModels();
			glFlush(GL_TRANS_MANUALSORT);
			threadWaitForVBlank();
			if (i == 0) {
				REG_DISPCAPCNT = (targetBank << 16) | (3 << 20) | (1 << 24) | (1 << 31); // applies to next *rendered* frame; i.e. 2 glflush from now, the next glflush gets rendered. so we have to
				// backtrack in time and use the settings for the previous frame
				// change the display to display normally
				REG_DISPCNT = (REG_DISPCNT & ~((3 << 16) | (3 << 18) | 7)) | (1 << 16) | (targetBank << 18) | 5 | (1 << 8) | (1 << 11) | (1<<23);
				
				SetupCameraMatrixPartial(0, 0, 128, 192); // applies to next glflush
				
				unsigned short* writeBuffer = frameBuffer2;
				if (frameBufferToRead == 1) {
					writeBuffer = frameBuffer1;
				}
				for (int y = 0; y < 192; ++y) {
					dmaBusyWait(2);
					REG_DMAxSAD(2) = (unsigned int)&VRAM_D[(y*256)];
					REG_DMAxDAD(2) = (unsigned int)&writeBuffer[(y*256)]; // BG memory
					REG_DMAxCNT(2) = ((64)) | ((DMA_MODE_DST(DmaMode_Increment) | DMA_MODE_SRC(DmaMode_Increment) | DMA_UNIT_32 | DMA_START) << 16);
				}
				frameBufferToRead ^= 1;

				
				*BG0OFFS = -128;
			}
			if (i == 1) {
				// set up the new DISPCAPCNT to capture and render final!
				// store to VRAM bank x, capture 256x192 pixels, capture 3D output, enable mix, set mixA to 31, set mixB to 31, and enable capture
				//REG_DISPCAPCNT = (targetBank << 16) | (3 << 20) | (1 << 24) | (3 << 26) | (2 << 29) | (0x1F << 0) | (0x1F << 8) | (1 << 31);
				REG_DISPCAPCNT = (targetBank << 16) | (3 << 20) | (1 << 24) | (1 << 31);
				// change the display to display video, selecting VRAM target for capture mixing
				REG_DISPCNT = (REG_DISPCNT & ~((3 << 16) | (3 << 18) | 7)) | (1 << 16) | (targetBank << 18) | 5 | (1 << 8) | (1 << 11) | (1<<23);
				
				// applies to next *rendered* frame; i.e. frame we just set up to be rendered
				// set up bg to render over us
				unsigned short* writeBuffer = frameBuffer2;
				if (frameBufferToRead == 1) {
					writeBuffer = frameBuffer1;
				}
				for (int y = 0; y < 192; ++y) {
					dmaBusyWait(2);
					REG_DMAxSAD(2) = (unsigned int)&VRAM_D[(y*256)+128];
					REG_DMAxDAD(2) = (unsigned int)&writeBuffer[(y*256)+128]; // BG memory
					REG_DMAxCNT(2) = ((64)) | ((DMA_MODE_DST(DmaMode_Increment) | DMA_MODE_SRC(DmaMode_Increment) | DMA_UNIT_32 | DMA_START) << 16);
				}

				// sprites, done!
				FinalizeSprites();
				bgUpdate();
				*BG0OFFS = 128; // have to modify BG0OFFS to render on the opposite side to fix rendering artifacts in the middle
				// TODO: add a hardware/melonds check to disable this as it fails spectacularly on other emulators that aren't as accurate as melon
			}
			// update music
			UpdateMusicBuffer();
		}
	}

	// reset matrices so we can use it for math later
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glMatrixMode(GL_POSITION); // may need to change to GL_MODELVIEW if we ever want to make proper use of vec test; but for now, keeping it to pos only makes it faster.

	glClearDepth(GL_MAX_DEPTH); // this technically only needs to be initialized once, but whatever, idc

	currObject = first;
	while (currObject != NULL) {
		Object* tmpObj = currObject;
		currObject = currObject->next;
		// also destroy object if relevant
		if (tmpObj->destroy) {
			delete tmpObj;
		}
	}
}

ITCM_CODE void DestroyObject(Object *object) {
	object->destroy = true;
}

void AddCollisionBetweenLayers(int layer1, int layer2) {
	layerCollision[(layer1*32)+layer2] = true;
	layerCollision[layer1+(layer2*32)] = true;
}

void DestroyObjectImmediate(Object* object) {
	delete object;
}

ObjectPtr::ObjectPtr(Object* obj) {
	object = obj;
	prev = NULL;
	next = obj->references;
	obj->references = this;
}

void Object::CleanupObjPtr() {
	ObjectPtr* nextPtr = references;
	while (nextPtr != NULL) {
		ObjectPtr* tmp = nextPtr->next;
		nextPtr->object = NULL;
		nextPtr->prev = NULL;
		nextPtr->prev = NULL;
		nextPtr = tmp;
	}
}