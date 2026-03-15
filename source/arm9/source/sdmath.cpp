#include <math.h>
#include <nds.h>
#include <stdio.h>
#include "sdmath.h"
#include <string.h>
#include <stdlib.h>

int sqrtfext64(long long value) {
	REG_SQRTCNT = SQRT_64;

	REG_SQRT_PARAM = value;

	while (REG_SQRTCNT & SQRT_BUSY);

	return REG_SQRT_RESULT;
}

__attribute__((target("arm")))
Fixed Fixed::operator*(const Fixed right) const {
	return Fixed(mulf32f(value, right.value));
}

Fixed Fixed::operator/(const Fixed right) const {
	return Fixed(divf32f(value, right.value));
}

__attribute__((target("arm")))
Fixed Fixed::operator*=(const Fixed right) {
	value = mulf32f(value, right.value);
	return *this;
}
Fixed Fixed::operator/=(const Fixed right) {
	value = divf32f(value, right.value);
	return *this;
}

#define ZeroMatrix(matrix) for(int i = 0; i < 16; ++i) {matrix->m[i] = 0;}

Mat4x4 Mat4x4::Translation(const Vec3& position) {
	Mat4x4 ret;
	ret.m[r1w] = position.x;
	ret.m[r2w] = position.y;
	ret.m[r3w] = position.z;
	return ret;
}
Mat4x4 Mat4x4::Scale(const Vec3& scale) {
	Mat4x4 ret;
	ret.m[r1x] = scale.x;
	ret.m[r2y] = scale.y;
	ret.m[r3z] = scale.z;
	return ret;
}

__attribute__((target("arm")))
Mat4x4 Mat4x4::Rotation(const Quaternion& rot) {
	int ww = mulf32fast(rot.w, rot.w);
	int xy = mulf32fast(rot.x, rot.y);
	int yz = mulf32fast(rot.y, rot.z);
	int xz = mulf32fast(rot.x, rot.z);
	int wx = mulf32fast(rot.w, rot.x);
	int wy = mulf32fast(rot.w, rot.y);
	int wz = mulf32fast(rot.w, rot.z);

	Mat4x4 ret;

	ret.m[r1x] = 2 * (ww + mulf32fast(rot.x, rot.x)) - 4096;
	ret.m[r1y] = 2 * (xy - wz);
	ret.m[r1z] = 2 * (xz + wy);
	
	ret.m[r2x] = 2 * (xy + wz);
	ret.m[r2y] = 2 * (ww + mulf32fast(rot.y, rot.y)) - 4096;
	ret.m[r2z] = 2 * (yz - wx);
	
	ret.m[r3x] = 2 * (xz - wy);
	ret.m[r3y] = 2 * (yz + wx);
	ret.m[r3z] = 2 * (ww + mulf32fast(rot.z, rot.z)) - 4096;

	return ret;
}

#define mulfnoshift(x,y) (int64_t)(x)*(int64_t)(y)

__attribute__((target("arm")))
Mat4x4 Mat4x4::Multiply3x3(const Mat4x4& right) const {
	Mat4x4 ret;
	ret.im[r1x] = (mulfnoshift(this->im[r1x], right.im[r1x]) + mulfnoshift(this->im[r1y], right.im[r2x]) + mulfnoshift(this->im[r1z], right.im[r3x])) >> 12;// + mulfnoshift(this->im[r1w], right.im[r4x]);
    ret.im[r1y] = (mulfnoshift(this->im[r1x], right.im[r1y]) + mulfnoshift(this->im[r1y], right.im[r2y]) + mulfnoshift(this->im[r1z], right.im[r3y])) >> 12;// + mulfnoshift(this->im[r1w], right.im[r4y]);
    ret.im[r1z] = (mulfnoshift(this->im[r1x], right.im[r1z]) + mulfnoshift(this->im[r1y], right.im[r2z]) + mulfnoshift(this->im[r1z], right.im[r3z])) >> 12;// + mulfnoshift(this->im[r1w], right.im[r4z]);

    ret.im[r2x] = (mulfnoshift(this->im[r2x], right.im[r1x]) + mulfnoshift(this->im[r2y], right.im[r2x]) + mulfnoshift(this->im[r2z], right.im[r3x])) >> 12;// + this->im[r2w], right.im[r4x];
    ret.im[r2y] = (mulfnoshift(this->im[r2x], right.im[r1y]) + mulfnoshift(this->im[r2y], right.im[r2y]) + mulfnoshift(this->im[r2z], right.im[r3y])) >> 12;// + this->im[r2w], right.im[r4y];
    ret.im[r2z] = (mulfnoshift(this->im[r2x], right.im[r1z]) + mulfnoshift(this->im[r2y], right.im[r2z]) + mulfnoshift(this->im[r2z], right.im[r3z])) >> 12;// + this->im[r2w], right.im[r4z];

    ret.im[r3x] = (mulfnoshift(this->im[r3x], right.im[r1x]) + mulfnoshift(this->im[r3y], right.im[r2x]) + mulfnoshift(this->im[r3z], right.im[r3x])) >> 12;// + this->im[r3w], right.im[r4x];
    ret.im[r3y] = (mulfnoshift(this->im[r3x], right.im[r1y]) + mulfnoshift(this->im[r3y], right.im[r2y]) + mulfnoshift(this->im[r3z], right.im[r3y])) >> 12;// + this->im[r3w], right.im[r4y];
    ret.im[r3z] = (mulfnoshift(this->im[r3x], right.im[r1z]) + mulfnoshift(this->im[r3y], right.im[r2z]) + mulfnoshift(this->im[r3z], right.im[r3z])) >> 12;// + this->im[r3w], right.im[r4z];

	return ret;
}

__attribute__((target("arm")))
Mat4x4 Mat4x4::Multiply4x3(const Mat4x4& right) const {
	Mat4x4 ret;

	ret.im[r1x] = (mulfnoshift(this->im[r1x], right.im[r1x]) + mulfnoshift(this->im[r1y], right.im[r2x]) + mulfnoshift(this->im[r1z], right.im[r3x])) >> 12;
    ret.im[r1y] = (mulfnoshift(this->im[r1x], right.im[r1y]) + mulfnoshift(this->im[r1y], right.im[r2y]) + mulfnoshift(this->im[r1z], right.im[r3y])) >> 12;
    ret.im[r1z] = (mulfnoshift(this->im[r1x], right.im[r1z]) + mulfnoshift(this->im[r1y], right.im[r2z]) + mulfnoshift(this->im[r1z], right.im[r3z])) >> 12;
    ret.im[r1w] = (mulfnoshift(this->im[r1x], right.im[r1w]) + mulfnoshift(this->im[r1y], right.im[r2w]) + mulfnoshift(this->im[r1z], right.im[r3w]) + this->im[r1w]) >> 12;

    ret.im[r2x] = (mulfnoshift(this->im[r2x], right.im[r1x]) + mulfnoshift(this->im[r2y], right.im[r2x]) + mulfnoshift(this->im[r2z], right.im[r3x])) >> 12;
    ret.im[r2y] = (mulfnoshift(this->im[r2x], right.im[r1y]) + mulfnoshift(this->im[r2y], right.im[r2y]) + mulfnoshift(this->im[r2z], right.im[r3y])) >> 12;
	ret.im[r2z] = (mulfnoshift(this->im[r2x], right.im[r1z]) + mulfnoshift(this->im[r2y], right.im[r2z]) + mulfnoshift(this->im[r2z], right.im[r3z])) >> 12;
    ret.im[r2w] = (mulfnoshift(this->im[r2x], right.im[r1w]) + mulfnoshift(this->im[r2y], right.im[r2w]) + mulfnoshift(this->im[r2z], right.im[r3w]) + this->im[r2w]) >> 12;

    ret.im[r3x] = (mulfnoshift(this->im[r3x], right.im[r1x]) + mulfnoshift(this->im[r3y], right.im[r2x]) + mulfnoshift(this->im[r3z], right.im[r3x])) >> 12;
    ret.im[r3y] = (mulfnoshift(this->im[r3x], right.im[r1y]) + mulfnoshift(this->im[r3y], right.im[r2y]) + mulfnoshift(this->im[r3z], right.im[r3y])) >> 12;
    ret.im[r3z] = (mulfnoshift(this->im[r3x], right.im[r1z]) + mulfnoshift(this->im[r3y], right.im[r2z]) + mulfnoshift(this->im[r3z], right.im[r3z])) >> 12;
    ret.im[r3w] = (mulfnoshift(this->im[r3x], right.im[r1w]) + mulfnoshift(this->im[r3y], right.im[r2w]) + mulfnoshift(this->im[r3z], right.im[r3w]) + this->im[r3w]) >> 12;

	return ret;
}

__attribute__((target("arm")))
Mat4x4 Mat4x4::operator*(const Mat4x4& right) const {
	Mat4x4 ret;
	ret.im[r1x] = (mulfnoshift(this->im[r1x], right.im[r1x]) + mulfnoshift(this->im[r1y], right.im[r2x]) + mulfnoshift(this->im[r1z], right.im[r3x]) + mulfnoshift(this->im[r1w], right.im[r4x])) >> 12;
	ret.im[r1y] = (mulfnoshift(this->im[r1x], right.im[r1y]) + mulfnoshift(this->im[r1y], right.im[r2y]) + mulfnoshift(this->im[r1z], right.im[r3y]) + mulfnoshift(this->im[r1w], right.im[r4y])) >> 12;
	ret.im[r1z] = (mulfnoshift(this->im[r1x], right.im[r1z]) + mulfnoshift(this->im[r1y], right.im[r2z]) + mulfnoshift(this->im[r1z], right.im[r3z]) + mulfnoshift(this->im[r1w], right.im[r4z])) >> 12;
	ret.im[r1w] = (mulfnoshift(this->im[r1x], right.im[r1w]) + mulfnoshift(this->im[r1y], right.im[r2w]) + mulfnoshift(this->im[r1z], right.im[r3w]) + mulfnoshift(this->im[r1w], right.im[r4w])) >> 12;

	ret.im[r2x] = (mulfnoshift(this->im[r2x], right.im[r1x]) + mulfnoshift(this->im[r2y], right.im[r2x]) + mulfnoshift(this->im[r2z], right.im[r3x]) + mulfnoshift(this->im[r2w], right.im[r4x])) >> 12;
	ret.im[r2y] = (mulfnoshift(this->im[r2x], right.im[r1y]) + mulfnoshift(this->im[r2y], right.im[r2y]) + mulfnoshift(this->im[r2z], right.im[r3y]) + mulfnoshift(this->im[r2w], right.im[r4y])) >> 12;
	ret.im[r2z] = (mulfnoshift(this->im[r2x], right.im[r1z]) + mulfnoshift(this->im[r2y], right.im[r2z]) + mulfnoshift(this->im[r2z], right.im[r3z]) + mulfnoshift(this->im[r2w], right.im[r4z])) >> 12;
	ret.im[r2w] = (mulfnoshift(this->im[r2x], right.im[r1w]) + mulfnoshift(this->im[r2y], right.im[r2w]) + mulfnoshift(this->im[r2z], right.im[r3w]) + mulfnoshift(this->im[r2w], right.im[r4w])) >> 12;

	ret.im[r3x] = (mulfnoshift(this->im[r3x], right.im[r1x]) + mulfnoshift(this->im[r3y], right.im[r2x]) + mulfnoshift(this->im[r3z], right.im[r3x]) + mulfnoshift(this->im[r3w], right.im[r4x])) >> 12;
	ret.im[r3y] = (mulfnoshift(this->im[r3x], right.im[r1y]) + mulfnoshift(this->im[r3y], right.im[r2y]) + mulfnoshift(this->im[r3z], right.im[r3y]) + mulfnoshift(this->im[r3w], right.im[r4y])) >> 12;
	ret.im[r3z] = (mulfnoshift(this->im[r3x], right.im[r1z]) + mulfnoshift(this->im[r3y], right.im[r2z]) + mulfnoshift(this->im[r3z], right.im[r3z]) + mulfnoshift(this->im[r3w], right.im[r4z])) >> 12;
	ret.im[r3w] = (mulfnoshift(this->im[r3x], right.im[r1w]) + mulfnoshift(this->im[r3y], right.im[r2w]) + mulfnoshift(this->im[r3z], right.im[r3w]) + mulfnoshift(this->im[r3w], right.im[r4w])) >> 12;

	ret.im[r4x] = (mulfnoshift(this->im[r4x], right.im[r1x]) + mulfnoshift(this->im[r4y], right.im[r2x]) + mulfnoshift(this->im[r4z], right.im[r3x]) + mulfnoshift(this->im[r4w], right.im[r4x])) >> 12;
	ret.im[r4y] = (mulfnoshift(this->im[r4x], right.im[r1y]) + mulfnoshift(this->im[r4y], right.im[r2y]) + mulfnoshift(this->im[r4z], right.im[r3y]) + mulfnoshift(this->im[r4w], right.im[r4y])) >> 12;
	ret.im[r4z] = (mulfnoshift(this->im[r4x], right.im[r1z]) + mulfnoshift(this->im[r4y], right.im[r2z]) + mulfnoshift(this->im[r4z], right.im[r3z]) + mulfnoshift(this->im[r4w], right.im[r4z])) >> 12;
	ret.im[r4w] = (mulfnoshift(this->im[r4x], right.im[r1w]) + mulfnoshift(this->im[r4y], right.im[r2w]) + mulfnoshift(this->im[r4z], right.im[r3w]) + mulfnoshift(this->im[r4w], right.im[r4w])) >> 12;
	return ret;
}

Mat4x4 Mat4x4::Transpose() const {
	Mat4x4 ret;
	ret.m[r1x] = m[r1x];
	ret.m[r2x] = m[r1y];
	ret.m[r3x] = m[r1z];
	ret.m[r4x] = m[r1w];
	ret.m[r1y] = m[r2x];
	ret.m[r2y] = m[r2y];
	ret.m[r3y] = m[r2z];
	ret.m[r4y] = m[r2w];
	ret.m[r1z] = m[r3x];
	ret.m[r2z] = m[r3y];
	ret.m[r3z] = m[r3z];
	ret.m[r4z] = m[r3w];
	ret.m[r1w] = m[r4x];
	ret.m[r2w] = m[r4y];
	ret.m[r3w] = m[r4z];
	ret.m[r4w] = m[r4w];
	return ret;
}

__attribute__((target("arm")))
Quaternion Quaternion::FromEuler(short x, short y, short z) {
	Quaternion q;
	int cy = cosLerp(z / 2);
	int sy = sinLerp(z / 2);
	int cp = cosLerp(y / 2);
	int sp = sinLerp(y / 2);
	int cr = cosLerp(x / 2);
	int sr = sinLerp(x / 2);
	
	q.w = (short)((mulfnoshift(mulf32(cr, cp), cy) + mulfnoshift(mulf32(sr, sp), sy)) >> 12);
	q.x = (short)((mulfnoshift(mulf32(sr, cp), cy) - mulfnoshift(mulf32(cr, sp), sy)) >> 12);
	q.y = (short)((mulfnoshift(mulf32(cr, sp), cy) + mulfnoshift(mulf32(sr, cp), sy)) >> 12);
	q.z = (short)((mulfnoshift(mulf32(cr, cp), sy) - mulfnoshift(mulf32(sr, sp), cy)) >> 12);
	return q;
}

#define mulf16noshift(x,y) ((int)(x))*((int)(y))

__attribute__((target("arm")))
Quaternion Quaternion::operator*(const Quaternion& right) const {
	Quaternion ret;

	ret.w = ((mulf16noshift(this->w, right.w) - mulf16noshift(this->x, right.x) - mulf16noshift(this->y, right.y) - mulf16noshift(this->z, right.z)) >> 12);
	ret.x = ((mulf16noshift(this->w, right.x) + mulf16noshift(this->x, right.w) + mulf16noshift(this->y, right.z) - mulf16noshift(this->z, right.y)) >> 12);
	ret.y = ((mulf16noshift(this->w, right.y) - mulf16noshift(this->x, right.z) + mulf16noshift(this->y, right.w) + mulf16noshift(this->z, right.x)) >> 12);
	ret.z = ((mulf16noshift(this->w, right.z) + mulf16noshift(this->x, right.y) - mulf16noshift(this->y, right.x) + mulf16noshift(this->z, right.w)) >> 12);
	return ret;
}

__attribute__((target("arm")))
Quaternion Quaternion::Normalize() const {
	Quaternion ret;
	int magnitude = mulf16noshift(x, x) + mulf16noshift(y,y) + mulf16noshift(z,z) + mulf16noshift(w,w);
	REG_SQRTCNT = SQRT_32;
	REG_SQRT_PARAM_L = magnitude;

	// while we wait on sqrt to do its thing, run some bitshifts n stuff to save time later
	REG_DIVCNT = 0;
	REG_DIV_NUMER_L = x << 12;
	int iy = y << 12;
	int iz = z << 12;
	int iw = w << 12;
	while (REG_SQRTCNT & SQRT_BUSY);
	magnitude = REG_SQRT_RESULT;
	REG_DIV_DENOM_L = magnitude;
	ret.y = iy / magnitude;
	ret.z = iz / magnitude;
	ret.w = iw / magnitude;
	while (REG_DIVCNT & DIV_BUSY);
	ret.x = REG_DIV_RESULT_L;
	return ret;
}

__attribute__((target("arm")))
Quaternion Quaternion::Slerp(const Quaternion& right, Fixed t) const {
	int cosOmega = (mulfnoshift(x, right.x) + mulfnoshift(y, right.y) + mulfnoshift(z, right.z) + mulfnoshift(w, right.w)) >> 12;
	Quaternion tempRight = right;
	// flip signs for shortest path
	if (cosOmega < 0) {
		cosOmega = -cosOmega;
		tempRight.x = -tempRight.x;
		tempRight.y = -tempRight.y;
		tempRight.z = -tempRight.z;
		tempRight.w = -tempRight.w;
	}

	if (cosOmega > 4096) {
		cosOmega = 4096;
	}
	int scaleFrom, scaleTo;
		// algorithm doesn't work well for extreme values, employ regular lerp
	if (cosOmega > 4095) {
		scaleFrom = 4096 - t;
		scaleTo = t;
	} else {
		// standard slerp
		int omega = acosLerp(cosOmega);
		// perform side-by-side division on DS
		REG_SQRTCNT = SQRT_32;
		// pre-shift by 12
		REG_SQRT_PARAM_L = 16777216 - cosOmega*cosOmega;
		omega = mulf32fast(omega, RotationToFixedRadians);
		REG_DIVCNT = 0;
		REG_DIV_NUMER_L = sinLerp(mulf32fast(t.value, omega)) << 12;
		scaleFrom = sinLerp(mulf32fast(4096 - t.value, omega));
		while (REG_SQRTCNT & SQRT_BUSY);
		int sinOmega = REG_SQRT_RESULT;
		REG_DIV_DENOM_L = sinOmega;
		// regrettably, one division isn't gonna cut it here...time to throw cycles...
		scaleFrom = divf32fast(scaleFrom, sinOmega);
		while (REG_DIVCNT & DIV_BUSY);
		scaleTo = REG_DIV_RESULT_L;
	}
	Quaternion ret;
	ret.x = (mulfnoshift(scaleFrom, this->x) + mulfnoshift(scaleTo, tempRight.x)) >> 12;
	ret.y = (mulfnoshift(scaleFrom, this->y) + mulfnoshift(scaleTo, tempRight.y)) >> 12;
	ret.z = (mulfnoshift(scaleFrom, this->z) + mulfnoshift(scaleTo, tempRight.z)) >> 12;
	ret.w = (mulfnoshift(scaleFrom, this->w) + mulfnoshift(scaleTo, tempRight.w)) >> 12;
	return ret.Normalize();
}

__attribute__((target("arm")))
Fixed Vec3::Dot(const Vec3& right) const {
	return {(mulfnoshift(x.value,right.x.value)+mulfnoshift(y.value,right.y.value)+mulfnoshift(z.value,right.z.value)) >> 12};
}

__attribute__((target("arm")))
Vec3 Vec3::Cross(const Vec3& right) const {
	Vec3 ret;
	ret.x = (mulfnoshift(y.value, right.z.value) - mulfnoshift(z.value, right.y.value)) >> 12;
	ret.y = -((mulfnoshift(x.value, right.z.value) - mulfnoshift(z.value, right.x.value)) >> 12);
	ret.z = (mulfnoshift(x.value, right.y.value) - mulfnoshift(y.value, right.x.value)) >> 12;
	return ret;
}

__attribute__((target("arm")))
Vec3 Quaternion::operator*(const Vec3& right) const {
	Vec3 u = Vec3(x,y,z);
	Fixed dot = u.Dot(right).value * 2;
	Vec3 temp = u * dot;
	
	Fixed s = mulf32fast(w, w);

	s -= u.Dot(u);
	temp = temp + (right * s);
	s = w * 2;
	Vec3 cross = u.Cross(right);
	return temp + (cross * s);
}

Quaternion Quaternion::Inverse() const {
	Quaternion ret(-x,-y,-z,w);
	return ret;
}

__attribute__((target("arm")))
Quaternion Quaternion::FromAngleAxis(short angle, Vec3 axis) {
	angle /= 2;
	Fixed s = sinLerp(angle);
	return Quaternion(mulf32f(s, axis.x), mulf32f(s, axis.y), mulf32f(s, axis.z), cosLerp(angle));
}

__attribute__((target("arm")))
Quaternion Quaternion::FromToRotation(Vec3 v1, Vec3 v2) {
	Fixed dot = v1.Dot(v2);
	Vec3 a;
	if (dot <= -4095) {
		Vec3 tmp(4096,0,0);
		a = tmp.Cross(v1);
		if (a.SqrMagnitude() == 0) {
			tmp.x = 0;
			tmp.y = 4096;
			a = tmp.Cross(v1);
		}
		a = a.Normalize();
		return Quaternion::FromAngleAxis(mulf32f(180*4096,FixedDegreesToRotation), a);
	}
	if (dot >= 4095) {
		// this is dumb
		return Quaternion(0,0,0,4096);
	}
	
	a = v1.Cross(v2);
	return Quaternion(a.x,a.y,a.z,sqrtf32(mulf32f(v1.SqrMagnitude(), v2.SqrMagnitude())) + dot).Normalize();
}

__attribute__((target("arm")))
Fixed Vec3::Magnitude() const {
	REG_SQRTCNT = SQRT_64;

	REG_SQRT_PARAM = mulfnoshift(x.value, x.value) + mulfnoshift(y.value, y.value) + mulfnoshift(z.value, z.value);

	while (REG_SQRTCNT & SQRT_BUSY);

	return REG_SQRT_RESULT;
}

__attribute__((target("arm")))
Fixed Vec3::SqrMagnitude() const {
	return (mulfnoshift(x.value, x.value) + mulfnoshift(y.value, y.value) + mulfnoshift(z.value, z.value)) >> 12;
}

__attribute__((target("arm")))
Vec3 Vec3::Normalize() const {
	REG_SQRTCNT = SQRT_64;

	REG_SQRT_PARAM = mulfnoshift(x.value, x.value) + mulfnoshift(y.value, y.value) + mulfnoshift(z.value, z.value);

	if (x.value == 0 && y.value == 0 && z.value == 0) {
		return Vec3(0,0,0);
	}

	REG_DIVCNT = DIV_64_32;

	REG_DIV_NUMER = ((long long)x.value) << 12;
	int iy = y.value << 12;
	int iz = z.value << 12;
	while (REG_SQRTCNT & SQRT_BUSY);

	int magnitude = REG_SQRT_RESULT;
	REG_DIV_DENOM_L = magnitude;
	iy = iy/magnitude;
	iz = iz/magnitude;
	while (REG_DIVCNT & DIV_BUSY);
	return Vec3(REG_DIV_RESULT, iy, iz);
}

Vec3 Vec3::operator+(const Vec3& right) const {
	return Vec3(x+right.x,y+right.y,z+right.z);
}

Vec3 Vec3::operator-(const Vec3& right) const {
	return Vec3(x - right.x, y - right.y, z - right.z);
}

__attribute__((target("arm")))
Vec3 Vec3::operator*(const Vec3& right) const {
	return Vec3(x*right.x,y*right.y,z*right.z);
}

__attribute__((target("arm")))
Vec3 Vec3::operator*(const Fixed right) const {
	return Vec3(x*right,y*right,z*right);
}

Vec3 Vec3::operator/(const Vec3& right) const {
	return Vec3(x/right.x,y/right.y,z/right.z);
}

Vec3 Vec3::operator/(const Fixed right) const {
	return Vec3(x/right,y/right,z/right);
}

__attribute__((target("arm")))
Fixed Lerp(Fixed left, Fixed right, Fixed t) {
	return left + t * (right-left);
}

__attribute__((target("arm")))
Vec3 Vec3::Reflect(const Vec3& direction, const Vec3& surface) const {
	Fixed dot = direction.Dot(surface);
	dot.value *= 2;
	return direction - (surface * dot);
}

__attribute__((target("arm")))
Fixed Atan2(Fixed yy, Fixed xx) {
	int y = yy.value;
	int x = xx.value;
	const int b = 2442;
	// arc tangent in first quadrant
	int bx_a = abs(mulf32f(b, mulf32f(x, y)));
	int num = bx_a + mulf32f(y, y);
	int atan_1q = divf32f(num, mulf32f(x, x) + bx_a + num);
	// multiply by half pi
	atan_1q = mulf32f(atan_1q,6434);
	// now set up the quadrant
	if (x < 0) {
		atan_1q = 6434 + (6434 - atan_1q);
	}
	if (y < 0) {
		atan_1q = -atan_1q;
	}
	return mulf32f(atan_1q, FixedRadiansToRotation);
}

Fixed Clamp(Fixed value, Fixed min, Fixed max) {
	if (value > max) {
		value = max;
	}
	if (value < min) {
		value = min;
	}
	return value;
}

Fixed Max(Fixed value, Fixed max) {
	if (max > value) {
		return max;
	}
	return value;
}

Fixed Min(Fixed value, Fixed min) {
	if (min < value) {
		return min;
	}
	return value;
}

int Max(int value, int max) {
	if (max > value) {
		return max;
	}
	return value;
}

int Min(int value, int min) {
	if (min < value) {
		return min;
	}
	return value;
}

short DeltaAngle(short dir1, short dir2)
{
	int a = dir2 - dir1;

	a += 32767/2;

	if (a < 0)
	{
		a += 32767;
	}

	if (a > 32767)
	{
		a -= 32767;
	}

	a -= 32767/2;

	return a;
}

Fixed Pow(Fixed value, Fixed toPow) {
	return powf(f32tofloat(value.value), f32tofloat(toPow));
}

__attribute__((target("arm")))
Vec3s Vec3s::NormalFromVerts(const Vec3s& vert1, const Vec3s& vert2, const Vec3s& vert3) {
	Vec3 U, V;
	U.x = vert2.x - vert1.x;
	U.y = vert2.y - vert1.y;
	U.z = vert2.z - vert1.z;
	V.x = vert3.x - vert1.x;
	V.y = vert3.y - vert1.y;
	V.z = vert3.z - vert1.z;
	Vec3s ret;
	ret.x = mulf32f(U.y, V.z) - mulf32f(U.z, V.y);
	ret.y = mulf32f(U.z, V.x) - mulf32f(U.x, V.z);
	ret.z = mulf32f(U.x, V.y) - mulf32f(U.y, V.x);
	int magnitude = sqrtfext64(ret.x * ret.x + ret.y * ret.y + ret.z * ret.z);
	ret.x = divf32f(ret.x, magnitude);
	ret.y = divf32f(ret.y, magnitude);
	ret.z = divf32f(ret.z, magnitude);
	return ret;
}

__attribute__((target("arm")))
Vec3 Vec3::NormalFromVerts(const Vec3& vert1, const Vec3& vert2, const Vec3& vert3) {
	Vec3 U, V;
	U = vert2-vert1;
	V = vert3-vert1;
	Vec3 ret((U.y*V.z)-(U.z*V.y),(U.z*V.x)-(U.x*V.z),(U.x*V.y)-(U.y*V.x));
	return ret.Normalize();
}

// above function is unreliable at low precision
__attribute__((target("arm")))
Vec3s Vec3s::NormalFromVertsFloat(const Vec3s& vert1, const Vec3s& vert2, const Vec3s& vert3) {
	Vec3f U, V;
	U.x = f32tofloat(vert2.x - vert1.x);
	U.y = f32tofloat(vert2.y - vert1.y);
	U.z = f32tofloat(vert2.z - vert1.z);
	V.x = f32tofloat(vert3.x - vert1.x);
	V.y = f32tofloat(vert3.y - vert1.y);
	V.z = f32tofloat(vert3.z - vert1.z);
	Vec3f computed;
	computed.x = (U.y * V.z) - (U.z * V.y);
	computed.y = (U.z * V.x) - (U.x * V.z);
	computed.z = (U.x * V.y) - (U.y * V.x);
	float magnitude = sqrtf(computed.x * computed.x + computed.y * computed.y + computed.z * computed.z);
	Vec3s ret;
	ret.x = floattof32(computed.x / magnitude);
	ret.y = floattof32(computed.y / magnitude);
	ret.z = floattof32(computed.z / magnitude);
	return ret;
}

Mat4x4 Mat4x4::FrustumToMatrix(Fixed xmin, Fixed xmax, Fixed ymin, Fixed ymax, Fixed near, Fixed far) {
	Mat4x4 ret;
	ret.m[r1x] = divf32f(2 * near, xmax - xmin);
	ret.m[r1y] = 0;
	ret.m[r1z] = divf32f(xmax + xmin, xmax - xmin);
	ret.m[r1w] = 0;
	ret.m[r2x] = 0;
	ret.m[r2y] = divf32f(2 * near, ymax - ymin);
	ret.m[r2z] = divf32f(ymax + ymin, ymax - ymin);
	ret.m[r2w] = 0;
	ret.m[r3x] = 0;
	ret.m[r3y] = 0;
	ret.m[r3z] = -divf32f(far + near, far - near);
	ret.m[r3w] = -divf32f(2 * mulf32f(far, near), far - near);
	ret.m[r4x] = 0;
	ret.m[r4y] = 0;
	ret.m[r4z] = -4096;
	ret.m[r4w] = 0;
	return ret;
}

__attribute__((target("arm")))
Mat4x4 Mat4x4::Perspective(const Fixed fov, const Fixed aspect, const Fixed near, const Fixed far) {
	Fixed xmin, xmax, ymin, ymax;

	ymax = mulf32f(near.value, tanLerp(fov / 2));

	ymin = -ymax;
	xmin = ymin * aspect;
	xmax = ymax * aspect;

	return FrustumToMatrix(xmin, xmax, ymin, ymax, near, far);
}

__attribute__((target("arm")))
Vec3 Mat4x4::operator*(const Vec3& right) const {
	Vec3 ret;
	ret.x = ((mulfnoshift(im[r1x], right.x.value) + mulfnoshift(im[r1y], right.y.value) + mulfnoshift(im[r1z], right.z.value)) >> 12) + im[r1w];
	ret.y = ((mulfnoshift(im[r2x], right.x.value) + mulfnoshift(im[r2y], right.y.value) + mulfnoshift(im[r2z], right.z.value)) >> 12) + im[r2w];
	ret.z = ((mulfnoshift(im[r3x], right.x.value) + mulfnoshift(im[r3y], right.y.value) + mulfnoshift(im[r3z], right.z.value)) >> 12) + im[r3w];
	return ret;
}

void Mat4x4::ExtractPlanesFromProj(
	Vec4* left, Vec4* right,
	Vec4* bottom, Vec4* top,
	Vec4* near, Vec4* far) const
{

	Mat4x4 transposed = this->Transpose();

	for (int i = 4; i--; ) ((Fixed*)&left)[i] = transposed.m[12 + i] + transposed.m[i];
	for (int i = 4; i--; ) ((Fixed*)&right)[i] = transposed.m[12 + i] - transposed.m[i];
	for (int i = 4; i--; ) ((Fixed*)&bottom)[i] = transposed.m[12 + i] + transposed.m[4 + i];
	for (int i = 4; i--; ) ((Fixed*)&top)[i] = transposed.m[12 + i] - transposed.m[4 + i];
	for (int i = 4; i--; ) ((Fixed*)&near)[i] = transposed.m[12 + i] + transposed.m[8 + i];
	for (int i = 4; i--; ) ((Fixed*)&far)[i] = transposed.m[12 + i] - transposed.m[8 + i];
}

__attribute__((target("arm")))
Mat4x4 Mat4x4::Invert() const
{
	Mat4x4 inv;
	Fixed det;
	int i;

	inv.m[r1x] = m[r2y] * m[r3z] * m[r4w] -
		m[r2y] * m[r3w] * m[r4z] -
		m[r3y] * m[r2z] * m[r4w] +
		m[r3y] * m[r2w] * m[r4z] +
		m[r4y] * m[r2z] * m[r3w] -
		m[r4y] * m[r2w] * m[r3z];

	inv.m[r2x] = -m[r2x] * m[r3z] * m[r4w] +
		m[r2x] * m[r3w] * m[r4z] +
		m[r3x] * m[r2z] * m[r4w] -
		m[r3x] * m[r2w] * m[r4z] -
		m[r4x] * m[r2z] * m[r3w] +
		m[r4x] * m[r2w] * m[r3z];

	inv.m[r3x] = m[r2x] * m[r3y] * m[r4w] -
		m[r2x] * m[r3w] * m[r4y] -
		m[r3x] * m[r2y] * m[r4w] +
		m[r3x] * m[r2w] * m[r4y] +
		m[r4x] * m[r2y] * m[r3w] -
		m[r4x] * m[r2w] * m[r3y];

	inv.m[r4x] = -m[r2x] * m[r3y] * m[r4z] +
		m[r2x] * m[r3z] * m[r4y] +
		m[r3x] * m[r2y] * m[r4z] -
		m[r3x] * m[r2z] * m[r4y] -
		m[r4x] * m[r2y] * m[r3z] +
		m[r4x] * m[r2z] * m[r3y];

	inv.m[r1y] = -m[r1y] * m[r3z] * m[r4w] +
		m[r1y] * m[r3w] * m[r4z] +
		m[r3y] * m[r1z] * m[r4w] -
		m[r3y] * m[r1w] * m[r4z] -
		m[r4y] * m[r1z] * m[r3w] +
		m[r4y] * m[r1w] * m[r3z];

	inv.m[r2y] = m[r1x] * m[r3z] * m[r4w] -
		m[r1x] * m[r3w] * m[r4z] -
		m[r3x] * m[r1z] * m[r4w] +
		m[r3x] * m[r1w] * m[r4z] +
		m[r4x] * m[r1z] * m[r3w] -
		m[r4x] * m[r1w] * m[r3z];

	inv.m[r3y] = -m[r1x] * m[r3y] * m[r4w] +
		m[r1x] * m[r3w] * m[r4y] +
		m[r3x] * m[r1y] * m[r4w] -
		m[r3x] * m[r1w] * m[r4y] -
		m[r4x] * m[r1y] * m[r3w] +
		m[r4x] * m[r1w] * m[r3y];

	inv.m[r4y] = m[r1x] * m[r3y] * m[r4z] -
		m[r1x] * m[r3z] * m[r4y] -
		m[r3x] * m[r1y] * m[r4z] +
		m[r3x] * m[r1z] * m[r4y] +
		m[r4x] * m[r1y] * m[r3z] -
		m[r4x] * m[r1z] * m[r3y];

	inv.m[r1z] = m[r1y] * m[r2z] * m[r4w] -
		m[r1y] * m[r2w] * m[r4z] -
		m[r2y] * m[r1z] * m[r4w] +
		m[r2y] * m[r1w] * m[r4z] +
		m[r4y] * m[r1z] * m[r2w] -
		m[r4y] * m[r1w] * m[r2z];

	inv.m[r2z] = -m[r1x] * m[r2z] * m[r4w] +
		m[r1x] * m[r2w] * m[r4z] +
		m[r2x] * m[r1z] * m[r4w] -
		m[r2x] * m[r1w] * m[r4z] -
		m[r4x] * m[r1z] * m[r2w] +
		m[r4x] * m[r1w] * m[r2z];

	inv.m[r3z] = m[r1x] * m[r2y] * m[r4w] -
		m[r1x] * m[r2w] * m[r4y] -
		m[r2x] * m[r1y] * m[r4w] +
		m[r2x] * m[r1w] * m[r4y] +
		m[r4x] * m[r1y] * m[r2w] -
		m[r4x] * m[r1w] * m[r2y];

	inv.m[r4z] = -m[r1x] * m[r2y] * m[r4z] +
		m[r1x] * m[r2z] * m[r4y] +
		m[r2x] * m[r1y] * m[r4z] -
		m[r2x] * m[r1z] * m[r4y] -
		m[r4x] * m[r1y] * m[r2z] +
		m[r4x] * m[r1z] * m[r2y];

	inv.m[r1w] = -m[r1y] * m[r2z] * m[r3w] +
		m[r1y] * m[r2w] * m[r3z] +
		m[r2y] * m[r1z] * m[r3w] -
		m[r2y] * m[r1w] * m[r3z] -
		m[r3y] * m[r1z] * m[r2w] +
		m[r3y] * m[r1w] * m[r2z];

	inv.m[r2w] = m[r1x] * m[r2z] * m[r3w] -
		m[r1x] * m[r2w] * m[r3z] -
		m[r2x] * m[r1z] * m[r3w] +
		m[r2x] * m[r1w] * m[r3z] +
		m[r3x] * m[r1z] * m[r2w] -
		m[r3x] * m[r1w] * m[r2z];

	inv.m[r3w] = -m[r1x] * m[r2y] * m[r3w] +
		m[r1x] * m[r2w] * m[r3y] +
		m[r2x] * m[r1y] * m[r3w] -
		m[r2x] * m[r1w] * m[r3y] -
		m[r3x] * m[r1y] * m[r2w] +
		m[r3x] * m[r1w] * m[r2y];

	inv.m[r4w] = m[r1x] * m[r2y] * m[r3z] -
		m[r1x] * m[r2z] * m[r3y] -
		m[r2x] * m[r1y] * m[r3z] +
		m[r2x] * m[r1z] * m[r3y] +
		m[r3x] * m[r1y] * m[r2z] -
		m[r3x] * m[r1z] * m[r2y];

	det = m[r1x] * inv.m[r1x] + m[r1y] * inv.m[r2x] + m[r1z] * inv.m[r3x] + m[r1w] * inv.m[r4x];

	if (det == 0)
		return Mat4x4();
	det = Fixed(4096) / det;

	Mat4x4 invOut;

	for (i = 0; i < 16; i++)
		invOut.m[i] = inv.m[i] * det;

	return invOut;
}

int sign(int value) {
	if (value < 0) { return -1; }
	else { return 1; }
}

__attribute__((target("arm")))
Vec3 Quaternion::ToEuler() const {
	// Roll (x-axis rotation)
	int sinr_cosp = mulf32f(2 * 4096, (mulf32f(w, x) + mulf32f(y, z)));
	int cosr_cosp = 4096 - mulf32f(2 * 4096, (mulf32f(x, x) + mulf32f(y, y)));
	Vec3 euler;
	euler.x = Atan2(sinr_cosp, cosr_cosp);

	// Pitch (y-axis rotation)
	int sinp = mulf32f(2 * 4096, (mulf32f(w, y) - mulf32f(z, x)));
	if (abs(sinp) >= 4096)
		euler.y = PI / 2 * sign(sinp); // use 90 degrees if out of range
	else
		euler.y = asinLerp(sinp);

	// Yaw (z-axis rotation)
	int siny_cosp = mulf32f(4096 * 2, (mulf32f(w, z) + mulf32f(x, y)));
	int cosy_cosp = 4096 - mulf32f(2 * 4096, (mulf32f(y, y) + mulf32f(z, z)));
	euler.z = Atan2(siny_cosp, cosy_cosp);
	return euler;
}

Fixed FixedRand(Fixed min, Fixed max) {
	// simple int rand
	return (rand() % (max - min)) + min;
}

long long Int64Div(long long left, long long right) {
	REG_DIVCNT = DIV_64_64;

	REG_DIV_NUMER = ((long long)left) << 12;
	REG_DIV_DENOM = right;

	while (REG_DIVCNT & DIV_BUSY);

	return REG_DIV_RESULT;
}

bool Vec3::operator==(const Vec3& right) const {
	return (x == right.x) && (y == right.y) && (z == right.z);
}

#ifdef _NOTDS
m4x4 matrices[32];
int matrixStackPos = 0;
#else
volatile unsigned int* GXSTAT = (volatile unsigned int*)0x04000600;
volatile unsigned int* POS_TEST = (volatile unsigned int*)0x040005C4;
Vec3* POS_RESULT = (Vec3*)0x04000620;
#endif

int PushMatrixStack(const Mat4x4& matrix) {
#ifdef _NOTDS
	matrices[matrixStackPos] = *matrix;
	return matrixStackPos++;
#else
	glPushMatrix();
	glLoadMatrix4x4((const m4x4*)&matrix);
	return ((*GXSTAT) >> 8) & 31;
#endif
}

void PopMatrixStack(int count) {
#ifdef _NOTDS
	matrixStackPos -= count;
	if (matrixStackPos < 0) {
		matrixStackPos = 0;
	}
#else
	glPopMatrix(count);
#endif
}

void RestoreMatrixStack(int stackPos) {
#ifdef _NOTDS
	matrices[matrixStackPos] = matrices[stackPos];
#else
	glRestoreMatrix(stackPos);
#endif
}

Vec3 MultiplyVectorByMatrixStack(const Vec3* v) {
#ifdef _NOTDS
	Vec3 retValue;
	MatrixTimesVec3(&matrices[matrixStackPos], v, &retValue);
	return retValue;
#else
	* POS_TEST = ((v->y.value) << 16) | (v->x.value & 0xFFFF);
	*POS_TEST = v->z.value;
	return *POS_RESULT;
#endif
}