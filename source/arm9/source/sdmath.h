#pragma once
#include <nds.h>

class Fixed {
public:
	int value;

	Fixed(int x) {
		value = x;
	}
	Fixed()=default;

	Fixed operator+(const Fixed right) const {
		return {value+right.value};
	}
	Fixed operator-(const Fixed right) const {
		return {value-right.value};
	}
	Fixed operator*(const Fixed right) const;
	Fixed operator/(const Fixed right) const;
	Fixed operator%(const Fixed right) const {
		return {value % right.value};
	}
	Fixed operator=(int right) {
		value = right;
		return *this;
	}
	Fixed operator-=(const Fixed right) {
		value -= right.value;
		return *this;
	}
	Fixed operator+=(const Fixed right) {
		value += right.value;
		return *this;
	}
	Fixed operator*=(const Fixed right);
	Fixed operator/=(const Fixed right);
	Fixed operator%=(const Fixed right) {
		value %= right.value;
		return *this;
	}
	Fixed operator-() const {
		return {-value};
	}
	bool operator==(const Fixed right) const {
		return value==right.value;
	}
	bool operator==(const int right) const {
		return value==right;
	}
	#define condop(x) bool operator x(const Fixed right) const {\
		return value x right.value;\
	}

	condop(<)
	condop(>)
	condop(<=)
	condop(>=)

	#undef condop
	#define condop(x) bool operator x(const int right) const {\
		return value x right;\
	}

	condop(<)
	condop(>)
	condop(<=)
	condop(>=)

	#undef condop

	operator int() {
		return value;
	}

	Fixed fabs() const {
		return {abs(value)};
	}
};

class Vec3;

class Quaternion {
public:
	short x;
	short y;
	short z;
	short w;
	Quaternion() {
		x = 0;
		y = 0;
		z = 0;
		w = 4096;
		return;
	}
	Quaternion(short x, short y, short z, short w) {
		this->x = x;
		this->y = y;
		this->z = z;
		this->w = w;
		return;
	}

	static Quaternion FromEuler(short x, short y, short z);
	Quaternion Normalize() const;
	Quaternion Slerp(const Quaternion& right, Fixed t) const;
	Quaternion Inverse() const;
	static Quaternion FromAngleAxis(short angle, Vec3 axis);
	static Quaternion FromToRotation(Vec3 v1, Vec3 v2);
	Quaternion operator*(const Quaternion& right) const;
	Vec3 operator*(const Vec3& right) const;
	Vec3 ToEuler() const;
};

class Vec4 {
public:
	Fixed x;
	Fixed y;
	Fixed z;
	Fixed w;

	Vec4() {
		x = 0;
		y = 0;
		z = 0;
		w = 0;
	}
	
	Vec4(Fixed x, Fixed y, Fixed z, Fixed w) {
		this->x = x;
		this->y = y;
		this->z = z;
		this->w = w;
	}
};

class Vec3 {
public:
	Fixed x;
	Fixed y;
	Fixed z;

	Vec3() {
		x = 0;
		y = 0;
		z = 0;
	}

	Vec3(Fixed x, Fixed y, Fixed z) {
		this->x = x;
		this->y = y;
		this->z = z;
	}

	Vec3(int x, int y, int z) {
		this->x.value = x;
		this->y.value = y;
		this->z.value = z;
	}

	Vec3 operator*(const Vec3& right) const;
	Vec3 operator*(const Fixed right) const;
	Vec3 operator/(const Vec3& right) const;
	Vec3 operator/(const Fixed right) const;
	Vec3 operator+(const Vec3& right) const;
	Vec3 operator-(const Vec3& right) const;
	bool operator==(const Vec3& right) const;
	Vec3 operator-() const {
		return Vec3(-x,-y,-z);
	}

	Fixed Dot(const Vec3& right) const;
	Vec3 Cross(const Vec3& right) const;
	Fixed SqrMagnitude() const;
	Fixed Magnitude() const;
	Vec3 Normalize() const;
	Vec3 Reflect(const Vec3& direction, const Vec3& surface) const;
	static Vec3 NormalFromVerts(const Vec3& vert1, const Vec3& vert2, const Vec3& vert3);
};

class Vec3s {
	public:
	union {
		struct {
			short x;
			short y;
			short z;
		};
		short coords[3];
	};

	static Vec3s NormalFromVerts(const Vec3s& vert1, const Vec3s& vert2, const Vec3s& vert3);
	static Vec3s NormalFromVertsFloat(const Vec3s& vert1, const Vec3s& vert2, const Vec3s& vert3);
};

struct Vec3f {
	union {
		struct {
			float x;
			float y;
			float z;
		};
		float coords[3];
	};
};

class Vec2 {
public:
	Fixed x;
	Fixed y;

};

struct ViewFrustum {
	Vec3 points[8];
	Vec4 planes[6];
};

class Mat4x4 {
public:
	union {
		Fixed m[16];
		Vec4 vm[4];
		Fixed sm[4][4];
		int im[16];
	};

	Mat4x4() {
		m[0] = 4096;
		m[1] = 0;
		m[2] = 0;
		m[3] = 0;
		m[4] = 0;
		m[5] = 4096;
		m[6] = 0;
		m[7] = 0;
		m[8] = 0;
		m[9] = 0;
		m[10] = 4096;
		m[11] = 0;
		m[12] = 0;
		m[13] = 0;
		m[14] = 0;
		m[15] = 4096;
	}

	Mat4x4(const Mat4x4& init) {
		vm[0] = init.vm[0];
		vm[1] = init.vm[1];
		vm[2] = init.vm[2];
		vm[3] = init.vm[3];
	}

	static Mat4x4 Translation(const Vec3& position);
	static Mat4x4 Scale(const Vec3& scale);
	static Mat4x4 Rotation(const Quaternion& rot);
	static Mat4x4 FrustumToMatrix(Fixed xmin, Fixed xmax, Fixed ymin, Fixed ymax, Fixed near, Fixed far);
	static Mat4x4 Perspective(const Fixed fov, const Fixed aspect, const Fixed near, const Fixed far);

	void ExtractPlanesFromProj(
	Vec4* left, Vec4* right,
	Vec4* bottom, Vec4* top,
	Vec4* near, Vec4* far) const;
	Mat4x4 Invert() const;

	Mat4x4 Multiply3x3(const Mat4x4& right) const;
	Mat4x4 Multiply4x3(const Mat4x4& right) const;
	Mat4x4 Transpose() const;
	Mat4x4 operator*(const Mat4x4& right) const;
	Vec3 operator*(const Vec3& right) const;
};

#define PI 12868
#define FixedDegreesToRotation 91
#define FixedRadiansToRotation 5215
#define RotationToFixedRadians 3217

// values for matrix computations...
#define r1x 0
#define r1y 4
#define r1z 8
#define r1w 12
#define r2x 1
#define r2y 5
#define r2z 9
#define r2w 13
#define r3x 2
#define r3y 6
#define r3z 10
#define r3w 14
#define r4x 3
#define r4y 7
#define r4z 11
#define r4w 15
/*#define r1x 0
#define r1y 1
#define r1z 2
#define r1w 3
#define r2x 4
#define r2y 5
#define r2z 6
#define r2w 7
#define r3x 8
#define r3y 9
#define r3z 10
#define r3w 11
#define r4x 12
#define r4y 13
#define r4z 14
#define r4w 15*/

// should only be used if you're certain values are < 65536
#define mulf32fast(a, b) (((int)(a) * (int)(b)) >> 12)
#define divf32fast(a, b) (((a) << 12) / (b))

Fixed Lerp(Fixed left, Fixed right, Fixed t);

Fixed Atan2(Fixed y, Fixed x);

Fixed Clamp(Fixed value, Fixed min, Fixed max);

Fixed Max(Fixed value, Fixed max);

Fixed Min(Fixed value, Fixed min);

int Max(int value, int max);

int Min(int value, int min);

short DeltaAngle(short dir1, short dir2);

Fixed Pow(Fixed value, Fixed toPow);

Fixed FixedRand(Fixed min, Fixed max);

long long Int64Div(long long left, long long right);

int PushMatrixStack(const Mat4x4& matrix);

void PopMatrixStack(int count);

void RestoreMatrixStack(int stackPos);

Vec3 MultiplyVectorByMatrixStack(const Vec3* v);

// introduced as libnds' built in divf32 is written incorrectly, causing it to actually take twice as long as it should!
static inline int divf32f(int left, int right) {
	REG_DIVCNT = DIV_64_32;

	REG_DIV_NUMER = ((long long)left) << 12;
	REG_DIV_DENOM_L = right;

	while (REG_DIVCNT & DIV_BUSY);

	return (REG_DIV_RESULT_L);
}

// see divf32f
static inline int sqrtf32f(int input)
{
	REG_SQRTCNT = SQRT_64;

	REG_SQRT_PARAM = ((long long)input) << 12;

	while (REG_SQRTCNT & SQRT_BUSY);

	return REG_SQRT_RESULT;
}

__attribute__((target("arm")))
static inline int mulf32f(int left, int right) {
	return (int)((((int64_t)left)*((int64_t)right))>>12);
}

#define divf32 divf32f
#define sqrtf32 sqrtf32f