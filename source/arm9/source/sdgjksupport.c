#include "sdmath.h"
#include "sdcollision.h"
#include <nds.h>

Vec3 BoxSupport(void* shape, Vec3* normal) {
	CollisionBox* box = shape;
	RestoreMatrixStack(box->matrixId);
	Vec3 tmpNormal = MultiplyVectorByMatrixStack(normal);
	Vec3 tmpRetValue;
	if (tmpNormal.x >= 0) {
		tmpRetValue.x = box->extents.x;
	}
	else {
		tmpRetValue.x = -box->extents.x;
	}
	if (tmpNormal.y >= 0) {
		tmpRetValue.y = box->extents.y;
	}
	else {
		tmpRetValue.y = -box->extents.y;
	}
	if (tmpNormal.z >= 0) {
		tmpRetValue.z = box->extents.z;
	}
	else {
		tmpRetValue.z = -box->extents.z;
	}

	RestoreMatrixStack(box->matrixId + 1);

	Vec3 retValue = MultiplyVectorByMatrixStack(&tmpRetValue);
	Vec3Addition(&retValue, box->position, &retValue);
	return retValue;
}