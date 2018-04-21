#include "MainWindow.h"
#include "ui_MainWindow.h"

#include <QDebug>
#include <QMouseEvent>
#include <QPainter>

struct fpoint_t {
	double x, y;
	fpoint_t()
		: x(0)
		, y(0)
	{
	}
	fpoint_t(double x, double y)
		: x(x)
		, y(y)
	{
	}
	void operator += (fpoint_t pt)
	{
		x += pt.x;
		y += pt.y;
	}
	void operator -= (fpoint_t pt)
	{
		x -= pt.x;
		y -= pt.y;
	}
	void operator *= (double t)
	{
		x *= t;
		y *= t;
	}
	void operator /= (double t)
	{
		x /= t;
		y /= t;
	}
	fpoint_t operator + (fpoint_t pt) const
	{
		return fpoint_t(x + pt.x, y + pt.y);
	}
	fpoint_t operator - (fpoint_t pt) const
	{
		return fpoint_t(x - pt.x, y - pt.y);
	}
	fpoint_t operator * (double t) const
	{
		return fpoint_t(x * t, y * t);
	}
	fpoint_t operator / (double t) const
	{
		return fpoint_t(x / t, y / t);
	}
	double length() const
	{
		return sqrt(x * x + y * y);
	}
	bool operator == (fpoint_t const &r) const
	{
		return x == r.x && y == r.y;
	}
	bool operator != (fpoint_t const &r) const
	{
		return !operator == (r);
	}
};


// 二次元平面上の直線(x0,y0)→(x1,y1)と点(px,py)の最短座標を求める。
// 直線上の(x0,y0)を0、(x1,y1)を1とするとき、
// 最短座標における媒介変数の値tを得る。
// (x0,y0)と(x1,y1)が一致していた時、falseを返す。
bool NearestPoint(double x0, double y0, double x1, double y1, double px, double py, double *t)
{
	double dx = x1 - x0;
	double dy = y1 - y0;
	if (dx == 0) {
		if (dy == 0) {
			return false;
		}
		*t = (py - y0) / dy;
		return true;
	}
	if (dy == 0) {
		*t = (px - x0) / dx;
		return true;
	}
	double m1 = dy / dx;
	double m2 = 1.0 / m1;
	double b1 = y0 - (m1 * x0);
	double b2 = py + (m2 * px);
	double tx = (b2 - b1) / (m1 + m2);
	double ty = (b2 * m1 + b1 * m2) / (m1 + m2);
	if (fabs(dx) > fabs(dy)) { // （基本的にどちらに分岐しても同じ値が得られるはず）
		*t = (tx - x0) / dx;
	} else {
		*t = (ty - y0) / dy;
	}
	return true;
}

// 二つの直線の交点を求める
bool LineIntersection(fpoint_t a0, fpoint_t a1, fpoint_t b0, fpoint_t b1, double *t0, double *t1)
{
	double x0 = a0.x;
	double y0 = a0.y;
	double f0 = a1.x - a0.x;
	double g0 = a1.y - a0.y;
	double x1 = b0.x;
	double y1 = b0.y;
	double f1 = b1.x - b0.x;
	double g1 = b1.y - b0.y;

	double det = f1 * g0 - f0 * g1;
	if (det == 0) {
		return false; // 平行
	}

	double dx = x1 - x0;
	double dy = y1 - y0;
	*t0 = (f1 * dy - g1 * dx) / det;
	*t1 = (f0 * dy - g0 * dx) / det;

	return true; // 交差
}

// 3次ベジェ曲線の値を求める
double Bezier(double p0, double p1, double p2, double p3, double t)
{
	//double u = 1 - t;
	//return p0 * u * u * u + 3 * p1 * u * u * t + 3 * p2 * u * t * t + p3 * t * t * t;
	return 0
		+ p0
		- (p0 - p1) * t * 3
		+ (p0 - p1 * 2 + p2) * t * t * 3
		- (p0 - p1 * 3 + p2 * 3 - p3) * t * t * t
		;
}

fpoint_t Bezier(fpoint_t const &p0, fpoint_t const &p1, fpoint_t const &p2, fpoint_t const &p3, double t)
{
	return fpoint_t(Bezier(p0.x, p1.x, p2.x, p3.x, t), Bezier(p0.y, p1.y, p2.y, p3.y, t));
}

// 3次ベジェ曲線の傾きを求める
double BezierSlope(double p0, double p1, double p2, double p3, double t)
{
	return 0
		- p0
		+ p1
		+ (p0 - p1 * 2 + p2) * t * 2
		- (p0 - p1 * 3 + p2 * 3 - p3) * t * t
		;
}

fpoint_t BezierSlope(fpoint_t const &p0, fpoint_t const &p1, fpoint_t const &p2, fpoint_t const &p3, double t)
{
	fpoint_t pt;
	pt.x = BezierSlope(p0.x, p1.x, p2.x, p3.x, t);
	pt.y = BezierSlope(p0.y, p1.y, p2.y, p3.y, t);
	return pt;
}

// 3次ベジェ曲線を任意の位置で分割する
void SplitBezier(double p0, double p1, double p2, double p3, double t, double *a0, double *a1, double *a2, double *a3, double *b0, double *b1, double *b2, double *b3)
{
	double p = Bezier(p0, p1, p2, p3, t);
	double d = BezierSlope(p0, p1, p2, p3, t);
	*a0 = p0;
	*a1 = p0 + (p1 - p0) * t;
	*a2 = p - d * t;
	*a3 = p;
	*b0 = p;
	*b1 = p + d * (1 - t);
	*b2 = p3 + (p2 - p3) * (1 - t);
	*b3 = p3;
}

void SplitBezier(fpoint_t p0, fpoint_t p1, fpoint_t p2, fpoint_t p3, double t, fpoint_t *a0, fpoint_t *a1, fpoint_t *a2, fpoint_t *a3, fpoint_t *b0, fpoint_t *b1, fpoint_t *b2, fpoint_t *b3)
{
	SplitBezier(p0.x, p1.x, p2.x, p3.x, t, &a0->x, &a1->x, &a2->x, &a3->x, &b0->x, &b1->x, &b2->x, &b3->x);
	SplitBezier(p0.y, p1.y, p2.y, p3.y, t, &a0->y, &a1->y, &a2->y, &a3->y, &b0->y, &b1->y, &b2->y, &b3->y);
}

// 3次ベジェ曲線上の最短位置の媒介変数の値tを得る。
double BezierNearestPoint(fpoint_t p0, fpoint_t p1, fpoint_t p2, fpoint_t p3, fpoint_t pt)
{
	const double N = 16;
	double position = 0;
	double distance = 0;
	for (int i = 0; i < N; i++) {
		fpoint_t a;
		fpoint_t b;
		fpoint_t c;
		fpoint_t d;
		double t = i / N;
		double u = (i + 1) / N;
		a = Bezier(p0, p1, p2, p3, t);
		d = Bezier(p0, p1, p2, p3, u);
		b = a + BezierSlope(p0, p1, p2, p3, t) / N;
		c = d - BezierSlope(p0, p1, p2, p3, u) / N;
		if (NearestPoint(a.x, a.y, d.x, d.y, pt.x, pt.y, &t)) {
			fpoint_t z = Bezier(a, b, c, d, t);
			double d = (z - pt).length();
			if (i == 0 || d < distance) {
				position = (i + t) / N;
				distance = d;
			}
		}
	}
	if (position < 0) {
		position = 0;
	} else if (position > 1) {
		position = 1;
	}
	return position;
}

// 二つの3次ベジェ曲線の交点を求める
static void BezierIntersection_(fpoint_t a0, fpoint_t a1, fpoint_t a2, fpoint_t a3, fpoint_t b0, fpoint_t b1, fpoint_t b2, fpoint_t b3, double start, double range, std::vector<double> *result);
void BezierIntersection(fpoint_t a0, fpoint_t a1, fpoint_t a2, fpoint_t a3, fpoint_t b0, fpoint_t b1, fpoint_t b2, fpoint_t b3, std::vector<double> *result)
{
	result->clear();
	BezierIntersection_(a0, a1, a2, a3, b0, b1, b2, b3, 0, 1, result);
	std::sort(result->begin(), result->end());
}
static void Bounds_(fpoint_t pt0, fpoint_t pt1, fpoint_t pt2, fpoint_t pt3, double *lower_x, double *lower_y, double *upper_x, double *upper_y)
{
	fpoint_t *pp[] = {
		&pt1,
		&pt2,
		&pt3,
	};
	double lx;
	double ly;
	double ux;
	double uy;
	lx = ux = pt0.x;
	ly = uy = pt0.y;
	for (int i = 0; i < 3; i++) {
		double x = pp[i]->x;
		double y = pp[i]->y;
		if (lx > x) lx = x;
		if (ly > y) ly = y;
		if (ux < x) ux = x;
		if (uy < y) uy = y;
	}
	*lower_x = lx;
	*lower_y = ly;
	*upper_x = ux;
	*upper_y = uy;
}
static bool BoxIntersection_(double a_lower_x, double a_lower_y, double a_upper_x, double a_upper_y, double b_lower_x, double b_lower_y, double b_upper_x, double b_upper_y)
{
	if (a_upper_x <= b_lower_x) return false;
	if (a_upper_y <= b_lower_y) return false;
	if (b_upper_x <= a_lower_x) return false;
	if (b_upper_y <= a_lower_y) return false;
	return true;
}
static void BezierIntersection_(fpoint_t a0, fpoint_t a1, fpoint_t a2, fpoint_t a3, fpoint_t b0, fpoint_t b1, fpoint_t b2, fpoint_t b3, double start, double range, std::vector<double> *result)
{
	if (range * 64 > 1) {
		fpoint_t aa0, aa1, aa2, aa3;
		fpoint_t ab0, ab1, ab2, ab3;
		fpoint_t ba0, ba1, ba2, ba3;
		fpoint_t bb0, bb1, bb2, bb3;
		SplitBezier(a0, a1, a2, a3, 0.5, &aa0, &aa1, &aa2, &aa3, &ab0, &ab1, &ab2, &ab3);
		SplitBezier(b0, b1, b2, b3, 0.5, &ba0, &ba1, &ba2, &ba3, &bb0, &bb1, &bb2, &bb3);
		double aa_lower_x, aa_lower_y, aa_upper_x, aa_upper_y, ab_lower_x, ab_lower_y, ab_upper_x, ab_upper_y;
		double ba_lower_x, ba_lower_y, ba_upper_x, ba_upper_y, bb_lower_x, bb_lower_y, bb_upper_x, bb_upper_y;
		Bounds_(aa0, aa1, aa2, aa3, &aa_lower_x, &aa_lower_y, &aa_upper_x, &aa_upper_y);
		Bounds_(ab0, ab1, ab2, ab3, &ab_lower_x, &ab_lower_y, &ab_upper_x, &ab_upper_y);
		Bounds_(ba0, ba1, ba2, ba3, &ba_lower_x, &ba_lower_y, &ba_upper_x, &ba_upper_y);
		Bounds_(bb0, bb1, bb2, bb3, &bb_lower_x, &bb_lower_y, &bb_upper_x, &bb_upper_y);
		range /= 2;
		if (BoxIntersection_(aa_lower_x, aa_lower_y, aa_upper_x, aa_upper_y, ba_lower_x, ba_lower_y, ba_upper_x, ba_upper_y)) {
			BezierIntersection_(aa0, aa1, aa2, aa3, ba0, ba1, ba2, ba3, start, range, result);
		}
		if (BoxIntersection_(aa_lower_x, aa_lower_y, aa_upper_x, aa_upper_y, bb_lower_x, bb_lower_y, bb_upper_x, bb_upper_y)) {
			BezierIntersection_(aa0, aa1, aa2, aa3, bb0, bb1, bb2, bb3, start, range, result);
		}
		start += range;
		if (BoxIntersection_(ab_lower_x, ab_lower_y, ab_upper_x, ab_upper_y, ba_lower_x, ba_lower_y, ba_upper_x, ba_upper_y)) {
			BezierIntersection_(ab0, ab1, ab2, ab3, ba0, ba1, ba2, ba3, start, range, result);
		}
		if (BoxIntersection_(ab_lower_x, ab_lower_y, ab_upper_x, ab_upper_y, bb_lower_x, bb_lower_y, bb_upper_x, bb_upper_y)) {
			BezierIntersection_(ab0, ab1, ab2, ab3, bb0, bb1, bb2, bb3, start, range, result);
		}
	} else {
		double t0, t1;
		if (LineIntersection(a0, a3, b0, b3, &t0, &t1)) {
			if (t0 >= 0 && t0 < 1 && t1 >= 0 && t1 < 1) {
				result->push_back(start + t0 * range);
			}
		}
	}
}

// 3次ベジェ曲線と垂直線との交点を求める
void BezierIntersectionVLine_(double start, double range, fpoint_t pt0, fpoint_t pt1, fpoint_t pt2, fpoint_t pt3, double x, std::vector<double> *result);
void BezierIntersectionVLine(fpoint_t pt0, fpoint_t pt1, fpoint_t pt2, fpoint_t pt3, double x, std::vector<double> *result)
{
	result->clear();
	BezierIntersectionVLine_(0, 1, pt0, pt1, pt2, pt3, x, result);
	std::sort(result->begin(), result->end());
}

void BezierIntersectionVLine(fpoint_t *pts, int size, double x, std::vector<double> *result)
{
	result->clear();
	for (int i = 0; i + 3 < size; i += 3) {
		std::vector<double> v;
		BezierIntersectionVLine_(0, 1, pts[i + 0], pts[i + 1], pts[i + 2], pts[i + 3], x, &v);
		for (int j = 0; j < (int)v.size(); j++) {
			result->push_back(i / 3 + v[j]);
		}
	}
	std::sort(result->begin(), result->end());
}

void BezierIntersectionVLine_(double start, double range, fpoint_t pt0, fpoint_t pt1, fpoint_t pt2, fpoint_t pt3, double x, std::vector<double> *result)
{
	if (range * 64 > 1) {
		if (!((pt0.x < x && pt1.x < x && pt2.x < x && pt3.x < x) || (pt0.x > x && pt1.x > x && pt2.x > x && pt3.x > x))) {
			fpoint_t bez[8];
			SplitBezier(pt0, pt1, pt2, pt3, 0.5, bez + 0, bez + 1, bez + 2, bez + 3, bez + 4, bez + 5, bez + 6, bez + 7);
			range /= 2;
			BezierIntersectionVLine_(start, range, bez[0], bez[1], bez[2], bez[3], x, result);
			BezierIntersectionVLine_(start + range, range, bez[4], bez[5], bez[6], bez[7], x, result);
		}
	} else {
		if (pt0.x != pt3.x) {
			double t = (x - pt0.x) / (pt3.x - pt0.x);
			if (t >= 0 && t < 1) {
				result->push_back(start + t * range);
			}
		}
	}
}

// 3次ベジェ曲線と水平線との交点を求める
void BezierIntersectionHLine_(double start, double range, fpoint_t pt0, fpoint_t pt1, fpoint_t pt2, fpoint_t pt3, double y, std::vector<double> *result);
void BezierIntersectionHLine(fpoint_t pt0, fpoint_t pt1, fpoint_t pt2, fpoint_t pt3, double y, std::vector<double> *result)
{
	result->clear();
	BezierIntersectionHLine_(0, 1, pt0, pt1, pt2, pt3, y, result);
	std::sort(result->begin(), result->end());
}

void BezierIntersectionHLine(fpoint_t *pts, int size, double y, std::vector<double> *result)
{
	result->clear();
	for (int i = 0; i + 3 < size; i += 3) {
		std::vector<double> v;
		BezierIntersectionHLine_(0, 1, pts[i + 0], pts[i + 1], pts[i + 2], pts[i + 3], y, &v);
		for (int j = 0; j < (int)v.size(); j++) {
			result->push_back(i / 3 + v[j]);
		}
	}
	std::sort(result->begin(), result->end());
}

void BezierIntersectionHLine_(double start, double range, fpoint_t pt0, fpoint_t pt1, fpoint_t pt2, fpoint_t pt3, double y, std::vector<double> *result)
{
	if (range * 64 > 1) {
		if (!((pt0.y < y && pt1.y < y && pt2.y < y && pt3.y < y) || (pt0.y > y && pt1.y > y && pt2.y > y && pt3.y > y))) {
			fpoint_t bez[8];
			SplitBezier(pt0, pt1, pt2, pt3, 0.5, bez + 0, bez + 1, bez + 2, bez + 3, bez + 4, bez + 5, bez + 6, bez + 7);
			range /= 2;
			BezierIntersectionHLine_(start, range, bez[0], bez[1], bez[2], bez[3], y, result);
			BezierIntersectionHLine_(start + range, range, bez[4], bez[5], bez[6], bez[7], y, result);
		}
	} else {
		if (pt0.y != pt3.y) {
			double t = (y - pt0.y) / (pt3.y - pt0.y);
			if (t >= 0 && t < 1) {
				result->push_back(start + t * range);
			}
		}
	}
}

// 3次ベジェ曲線に外接する矩形を求める
struct bounds_t {
	double min_x;
	double min_y;
	double max_x;
	double max_y;
	bounds_t()
		: min_x(0)
		, min_y(0)
		, max_x(0)
		, max_y(0)
	{
	}
};
static void BezierBounds_(int recurse, fpoint_t a, fpoint_t b, fpoint_t c, fpoint_t d, bounds_t *result);
void BezierBounds(fpoint_t *pts, int size, double *min_x, double *min_y, double *max_x, double *max_y)
{
	bounds_t t;
	for (int i = 0; i + 3 < size; i += 3) {
		if (i == 0) {
			t.min_x = t.max_x = pts[0].x;
			t.min_y = t.max_y = pts[0].y;
		}
		BezierBounds_(10, pts[i + 0], pts[i + 1], pts[i + 2], pts[i + 3], &t);
	}
	*min_x = t.min_x;
	*min_y = t.min_y;
	*max_x = t.max_x;
	*max_y = t.max_y;
}
static bool UpdateBounds_(bounds_t *b, fpoint_t p)
{
	bool changed = false;
	if (b->min_x >= p.x) {
		b->min_x = p.x;
		changed = true;
	}
	if (b->min_y >= p.y) {
		b->min_y = p.y;
		changed = true;
	}
	if (b->max_x <= p.x) {
		b->max_x = p.x;
		changed = true;
	}
	if (b->max_y <= p.y) {
		b->max_y = p.y;
		changed = true;
	}
	return changed;
}
static void BezierBounds_(int recurse, fpoint_t a, fpoint_t b, fpoint_t c, fpoint_t d, bounds_t *result)
{
	bool changed = false;
	if (UpdateBounds_(result, a)) {
		changed = true;
	}
	if (UpdateBounds_(result, d)) {
		changed = true;
	}
	if (changed && recurse > 0) {
		fpoint_t bez[8];
		SplitBezier(a, b, c, d, 0.5, bez + 0, bez + 1, bez + 2, bez + 3, bez + 4, bez + 5, bez + 6, bez + 7);
		BezierBounds_(recurse - 1, bez[0], bez[1], bez[2], bez[3], result);
		BezierBounds_(recurse - 1, bez[4], bez[5], bez[6], bez[7], result);
	}
}




struct AffineMatrix {
	double m11, m12, m13;
	double m21, m22, m23;
	double m31, m32, m33;
	AffineMatrix()
		: m11(1), m12(0), m13(0)
		, m21(0), m22(1), m23(0)
		, m31(0), m32(0), m33(1)
	{
	}
	AffineMatrix operator * (AffineMatrix const &right) const;
	void offset(double x, double y);
	void rotate(double deg, double ox, double oy);
	void scale(double sx, double sy, double ox, double oy);
	void transform(double *x, double *y) const;
	bool invert(AffineMatrix *out) const;
};

AffineMatrix AffineMatrix::operator * (AffineMatrix const &right) const
{
	AffineMatrix ret;

	ret.m11 = m11 * right.m11 + m12 * right.m21 + m13 * right.m31;
	ret.m12 = m11 * right.m12 + m12 * right.m22 + m13 * right.m32;
	ret.m13 = m11 * right.m13 + m12 * right.m23 + m13 * right.m33;

	ret.m21 = m21 * right.m11 + m22 * right.m21 + m23 * right.m31;
	ret.m22 = m21 * right.m12 + m22 * right.m22 + m23 * right.m32;
	ret.m23 = m21 * right.m13 + m22 * right.m23 + m23 * right.m33;

	ret.m31 = m31 * right.m11 + m32 * right.m21 + m33 * right.m31;
	ret.m32 = m31 * right.m12 + m32 * right.m22 + m33 * right.m32;
	ret.m33 = m31 * right.m13 + m32 * right.m23 + m33 * right.m33;

	return ret;
}

void AffineMatrix::offset(double x, double y)
{
	m13 += x;
	m23 += y;
}

void AffineMatrix::rotate(double deg, double ox, double oy)
{
	double r = deg * 3.14159265358979 / 180;
	double s = sin(r);
	double c = cos(r);
	AffineMatrix t;
	t.m11 = c;
	t.m12 = -s;
	t.m21 = s;
	t.m22 = c;
	*this = *this * t;
	offset(-ox, -oy);
	t.transform(&m13, &m23);
	offset(ox, oy);
}

void AffineMatrix::scale(double sx, double sy, double ox, double oy)
{
	double dx = m13 - ox;
	double dy = m23 - oy;
	AffineMatrix t;
	t.m11 = sx;
	t.m22 = sy;
	*this = *this * t;
	t.transform(&dx, &dy);
	offset(dx, dy);
}

void AffineMatrix::transform(double *x, double *y) const
{
	double tx = *x;
	double ty = *y;
	*x = m11 * tx + m12 * ty + m13;
	*y = m21 * tx + m22 * ty + m23;
}

bool AffineMatrix::invert(AffineMatrix *out) const
{
	double det = m11 * m22 - m12 * m21;
	if (det == 0) return false;
	double rdet = 1 / det;
	double t = m13;
	out->m13 = (m12 * m23 - t * m22) * rdet;
	out->m23 = (t * m21 - m11 * m23) * rdet;
	out->m12 = -m12 * rdet;
	out->m21 = -m21 * rdet;
	out->m11 = m22 * rdet;
	out->m22 = m11 * rdet;
	return true;
}


class PerspectiveTransform {
private:
	static void LinePoint(fpoint_t a, fpoint_t b, double t, double *x, double *y)
	{
		*x = a.x + (b.x - a.x) * t;
		*y = a.y + (b.y - a.y) * t;
	}

	static double LineDistance(double x0, double y0, double x1, double y1, double tx, double ty)
	{
		double t;
		double x, y;
		if (NearestPoint(x0, y0, x1, y1, tx, ty, &t)) {
			LinePoint(fpoint_t(x0, y0), fpoint_t(x1, y1), t, &x, &y);
		} else {
			x = x0;
			y = y0;
		}
		double dx = x - tx;
		double dy = y - ty;
		return sqrt(dx * dx + dy * dy);
	}
public:

	// 投影座標から正方座標へ変換
	struct pr_to_sq_data_t {
		bool ok;
		bool horizon;
		fpoint_t horizon0, horizon1;
		AffineMatrix affine1;
		AffineMatrix affine2;
		double scale_x, scale_y;

		bool setup_(fpoint_t pt0, fpoint_t pt1, fpoint_t pt2, fpoint_t pt3)
		{
			// 二つの線分が交差しているときはfalse
			double t0, t1;
			bool f = LineIntersection(pt0, pt2, pt1, pt3, &t0, &t1);
			if (f) {
				if (t0 >= 0 && t0 <= 1) return false;
				if (t1 >= 0 && t1 <= 1) return false;
				LinePoint(pt0, pt2, t0, &horizon0.x, &horizon0.y);
			}
			bool g = LineIntersection(pt0, pt1, pt2, pt3, &t0, &t1);
			if (g) {
				if (t0 >= 0 && t0 <= 1) return false;
				if (t1 >= 0 && t1 <= 1) return false;
				LinePoint(pt0, pt1, t0, &horizon1.x, &horizon1.y);
			}

			if (f && g) {
				horizon = true;
			} else {
				if (f) {
					double dx = pt1.x - pt0.x + pt3.x - pt2.x;
					double dy = pt1.y - pt0.y + pt3.y - pt2.y;
					horizon1.x = horizon0.x + dx;
					horizon1.y = horizon0.y + dy;
					horizon = true;
				} else if (g) {
					double dx = pt2.x - pt0.x + pt3.x - pt1.x;
					double dy = pt2.y - pt0.y + pt3.y - pt1.y;
					horizon0.x = horizon1.x + dx;
					horizon0.y = horizon1.y + dy;
					horizon = true;
				} else {
					// 平行四辺形
					horizon = false;
				}
			}

			double x0, y0, x1, y1, x2, y2;

			affine1 = AffineMatrix();
			affine2 = AffineMatrix();
			scale_x = scale_y = 1;

			// 平行四辺形の左辺を垂直にする回転変換＋原点合わせ

			x0 = pt0.x;
			y0 = pt0.y;
			x1 = pt1.x;
			y1 = pt1.y;
			x2 = pt2.x;
			y2 = pt2.y;
			transform(&x0, &y0);
			transform(&x1, &y1);
			transform(&x2, &y2);
			affine1.offset(-x0, -y0);
			affine1.rotate(atan2(x2 - x0, y2 - y0) * 180 / 3.14159265358979324, 0, 0);

			// 平行四辺形から長方形への変換

			x0 = pt0.x;
			y0 = pt0.y;
			x1 = pt1.x;
			y1 = pt1.y;
			transform(&x0, &y0);
			transform(&x1, &y1);
			affine2.m21 = -(y1 - y0) / (x1 - x0);

			// 長方形から単位正方形への変換係数

			x0 = pt0.x;
			y0 = pt0.y;
			x1 = pt1.x;
			y1 = pt1.y;
			x2 = pt2.x;
			y2 = pt2.y;
			transform(&x0, &y0);
			transform(&x1, &y1);
			transform(&x2, &y2);
			scale_x = 1 / (x1 - x0);
			scale_y = 1 / (y2 - y0);

			return true;
		}
		void setup(fpoint_t pt0, fpoint_t pt1, fpoint_t pt2, fpoint_t pt3)
		{
			ok = setup_(pt0, pt1, pt2, pt3);
		}
		bool transform(double *px, double *py) const
		{
			if (!ok) return false;
			fpoint_t p(*px, *py);
			if (horizon) {
				// 水平線からの距離で割ると平行四辺形になる
				double d = LineDistance(horizon0.x, horizon0.y, horizon1.x, horizon1.y, p.x, p.y);
				p.x /= d;
				p.y /= d;
			}
			affine1.transform(&p.x, &p.y); // 原点あわせと回転
			affine2.transform(&p.x, &p.y); // 平行四辺形を長方形へ
			*px = p.x * scale_x; // 長方形を
			*py = p.y * scale_y; // 正方形へ
			return true;
		}
	} pr_to_sq;

	// 正方座標から投影座標へ変換
	struct sq_to_pr_data_t {
		bool ok;
		struct {
			fpoint_t pt0;
			fpoint_t pt1;
			fpoint_t pt2;
			fpoint_t pt3;
			double ox, oy;
			double z0, z1;
			bool ok;
			bool setup_()
			{
				double t0, t1;
				ok = LineIntersection(pt0, pt1, pt2, pt3, &t0, &t1);
				if (ok) {
					if ((t0 < 0 || t0 > 1) && (t1 < 0 || t1 > 1)) {
						LinePoint(pt0, pt1, t0, &ox, &oy);
						double vx0 = pt0.x - ox;
						double vy0 = pt0.y - oy;
						double vx1 = pt1.x - ox;
						double vy1 = pt1.y - oy;
						double vx2 = pt2.x - ox;
						double vy2 = pt2.y - oy;
						double vx3 = pt3.x - ox;
						double vy3 = pt3.y - oy;
						double z9;
						z0 = z9 = 0;
						if (vx0 != 0 && vx1 != 0) z0 = vx0 / vx1;
						if (vy0 != 0 && vy1 != 0) z9 = vy0 / vy1;
						if (fabs(z0) < fabs(z9)) z0 = z9;
						z1 = z9 = 0;
						if (vx2 != 0 && vx3 != 0) z1 = vx2 / vx3;
						if (vy2 != 0 && vy3 != 0) z9 = vy2 / vy3;
						if (fabs(z1) < fabs(z9)) z1 = z9;
						return true; // 通常true
					} else {
						return false; // 交差してたらfalse
					}
				}
				return true; // 並行の時はtrue
			}
			bool setup(fpoint_t pt0, fpoint_t pt1, fpoint_t pt2, fpoint_t pt3)
			{
				this->pt0 = pt0;
				this->pt1 = pt1;
				this->pt2 = pt2;
				this->pt3 = pt3;
				return setup_();
			}
			bool line(double t, double *x0, double *y0, double *x1, double *y1) const
			{
				if (ok) {
					if (z0 != 0 && z1 != 0) {
						double z;
						z = 1 + (z0 - 1) * t;
						*x0 = ox + (pt0.x - ox) / z;
						*y0 = oy + (pt0.y - oy) / z;
						z = 1 + (z1 - 1) * t;
						*x1 = ox + (pt2.x - ox) / z;
						*y1 = oy + (pt2.y - oy) / z;
					} else {
						return false;
					}
				} else {
					*x0 = pt0.x + (pt1.x - pt0.x) * t;
					*y0 = pt0.y + (pt1.y - pt0.y) * t;
					*x1 = pt2.x + (pt3.x - pt2.x) * t;
					*y1 = pt2.y + (pt3.y - pt2.y) * t;
				}
				return true;
			}
		} data_x, data_y;
		void setup(fpoint_t pt0, fpoint_t pt1, fpoint_t pt2, fpoint_t pt3)
		{
			ok = true;
			if (!data_x.setup(pt0, pt1, pt2, pt3)) ok = false;
			if (!data_y.setup(pt0, pt2, pt1, pt3)) ok = false;
		}
		bool line_x(double t, double *x0, double *y0, double *x1, double *y1) const
		{
			return data_x.line(t, x0, y0, x1, y1);
		}
		bool line_y(double t, double *x0, double *y0, double *x1, double *y1) const
		{
			return data_y.line(t, x0, y0, x1, y1);
		}
		bool transform(double *px, double *py) const
		{
			double tx = *px;
			double ty = *py;
			fpoint_t a0, a1, b0, b1;
			if (line_x(tx, &a0.x, &a0.y, &a1.x, &a1.y)) {
				if (line_y(ty, &b0.x, &b0.y, &b1.x, &b1.y)) {
					double t0, t1;
					if (LineIntersection(a0, a1, b0, b1, &t0, &t1)) {
						*px = a0.x + (a1.x - a0.x) * t0;
						*py = a0.y + (a1.y - a0.y) * t0;
						return true;
					}
				}
			}
			return false;
		}
	} sq_to_pr;

public:

	bool setup(fpoint_t pt0, fpoint_t pt1, fpoint_t pt2, fpoint_t pt3)
	{
		pr_to_sq.setup(pt0, pt1, pt2, pt3);
		sq_to_pr.setup(pt0, pt1, pt2, pt3);
		return pr_to_sq.ok && sq_to_pr.ok;
	}

	void transform_pr_to_sq(double *px, double *py)
	{
		pr_to_sq.transform(px, py);
	}

	bool transform_sq_to_pr(double *px, double *py)
	{
		return sq_to_pr.transform(px, py);
	}
};



void DrawRoundHandle(QPainter *p, double x, double y, double size, QColor inside, QColor outline)
{
	x -= size / 2;
	y -= size / 2;
	p->save();
	p->setRenderHint(QPainter::Antialiasing);
	p->setPen(outline);
	p->setBrush(inside);
	p->drawEllipse(x, y, size, size);
	p->restore();
}

struct MainWindow::Private {
	fpoint_t points[4];
	handle_id_t selected_handle;
	double drag_delta_x;
	double drag_delta_y;
	double mouse_tx0;
	double mouse_ty0;
};

MainWindow::MainWindow(QWidget *parent)
	: QMainWindow(parent)
	, ui(new Ui::MainWindow)
	, priv(new Private)
{
	ui->setupUi(this);

	priv->points[0] = fpoint_t(200, 200);
	priv->points[1] = fpoint_t(600, 200);
	priv->points[2] = fpoint_t(200, 400);
	priv->points[3] = fpoint_t(600, 400);
	priv->selected_handle = H_NONE;
}

MainWindow::~MainWindow()
{
	delete priv;
	delete ui;
}

void MainWindow::paintEvent(QPaintEvent *event)
{
	QPainter pr(this);
	pr.setRenderHint(QPainter::Antialiasing);

	pr.drawLine(priv->points[0].x, priv->points[0].y, priv->points[1].x, priv->points[1].y);
	pr.drawLine(priv->points[1].x, priv->points[1].y, priv->points[3].x, priv->points[3].y);
	pr.drawLine(priv->points[3].x, priv->points[3].y, priv->points[2].x, priv->points[2].y);
	pr.drawLine(priv->points[2].x, priv->points[2].y, priv->points[0].x, priv->points[0].y);

	PerspectiveTransform homography;

	bool ok = homography.setup(priv->points[0], priv->points[1], priv->points[2], priv->points[3]);

	if (ok) {
		int n = 16;

		for (int i = 1; i < n; i++) {
			double tx = (double)i / n;
			double x0, y0, x1, y1;
			if (homography.sq_to_pr.line_x(tx, &x0, &y0, &x1, &y1)) {
				pr.drawLine(x0, y0, x1, y1);
			}
		}

		for (int i = 1; i < n; i++) {
			double ty = (double)i / n;
			double x0, y0, x1, y1;
			if (homography.sq_to_pr.line_y(ty, &x0, &y0, &x1, &y1)) {
				pr.drawLine(x0, y0, x1, y1);
			}
		}

		for (int i = 0; i <= n; i++) {
			double ty = (double)i / n;
			for (int j = 0; j <= n; j++) {
				double tx = (double)j / n;
				double x = tx;
				double y = ty;
				if (homography.transform_sq_to_pr(&x, &y)) {
					DrawRoundHandle(&pr, x, y, 5, QColor(255, 0, 0), QColor(0, 0, 0));
				}
			}
		}
	}

	for (int i = 0; i < 4; i++) {
		bool hit = false;
		if (i == priv->selected_handle) {
			hit = true;
		}
		int x = (int)floor(priv->points[i].x + 0.5);
		int y = (int)floor(priv->points[i].y + 0.5);
		DrawRoundHandle(&pr, x, y, 7, hit ? QColor(0, 0, 0) : QColor(255, 255, 255), QColor(0, 0, 0));

	}

	if (ok) {

		double x0 = priv->points[0].x;
		double y0 = priv->points[0].y;
		double x1 = priv->points[1].x;
		double y1 = priv->points[1].y;
		double x2 = priv->points[2].x;
		double y2 = priv->points[2].y;
		double x3 = priv->points[3].x;
		double y3 = priv->points[3].y;

		homography.transform_pr_to_sq(&x0, &y0);
		homography.transform_pr_to_sq(&x1, &y1);
		homography.transform_pr_to_sq(&x2, &y2);
		homography.transform_pr_to_sq(&x3, &y3);

		AffineMatrix a;
		a.scale(50, 50, 0, 0);
		a.offset(width() / 2, height() / 2);
		a.transform(&x0, &y0);
		a.transform(&x1, &y1);
		a.transform(&x2, &y2);
		a.transform(&x3, &y3);

		if (0) {
			pr.drawLine(x0, y0, x1, y1);
			pr.drawLine(x1, y1, x3, y3);
			pr.drawLine(x3, y3, x2, y2);
			pr.drawLine(x2, y2, x0, y0);
			DrawRoundHandle(&pr, x0, y0, 5, QColor(255, 255, 0), QColor(0, 0, 0));
			DrawRoundHandle(&pr, x1, y1, 5, QColor(0, 255, 0), QColor(0, 0, 0));
			DrawRoundHandle(&pr, x2, y2, 5, QColor(255, 0, 0), QColor(0, 0, 0));
			DrawRoundHandle(&pr, x3, y3, 5, QColor(0, 0, 255), QColor(0, 0, 0));
		}
	}
}

handle_id_t MainWindow::HandleHitTest(QPoint pt) const
{
	handle_id_t ht = H_NONE;
	double distance = 50;
	for (int i = 0; i < 4; i++) {
		double dx = priv->points[i].x - pt.x();
		double dy = priv->points[i].y - pt.y();
		double d = dx * dx + dy * dy;
		if (distance > d) {
			distance = d;
			ht = (handle_id_t)i;
		}
	}
	return ht;
}


void MainWindow::mousePressEvent(QMouseEvent *event)
{
	QPoint point = event->pos();
	int i = (int)HandleHitTest(point);
	if (i >= 0 && i < 4) {
		priv->selected_handle = (handle_id_t)i;
		priv->drag_delta_x = priv->points[i].x - point.x();
		priv->drag_delta_y = priv->points[i].y - point.y();
	}

}

void MainWindow::mouseReleaseEvent(QMouseEvent *event)
{
	priv->selected_handle = (handle_id_t)H_NONE;
}

void MainWindow::mouseMoveEvent(QMouseEvent *event)
{
	QPoint point = event->pos();
	int i = (int)priv->selected_handle;
	if (i >= 0 && i < 4) {
		priv->points[i].x = point.x() + priv->drag_delta_x;
		priv->points[i].y = point.y() + priv->drag_delta_y;
		update();
	}
}



