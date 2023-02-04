#pragma once

// -----------------------------------------------------------
// scene.h
// Simple test scene for ray tracing experiments. Goals:
// - Super-fast scene intersection
// - Easy interface: scene.FindNearest / IsOccluded
// - With normals and albedo: GetNormal / GetAlbedo
// - Area light source (animated), for light transport
// - Primitives can be hit from inside - for dielectrics
// - Can be extended with other primitives and/or a BVH
// - Optionally animated - for temporal experiments
// - Not everything is axis aligned - for cache experiments
// - Can be evaluated at arbitrary time - for motion blur
// - Has some high-frequency details - for filtering
// Some speed tricks that severely affect maintainability
// are enclosed in #ifdef SPEEDTRIX / #endif. Mind these
// if you plan to alter the scene in any way.
// -----------------------------------------------------------

#define SPEEDTRIX

#define PLANE_X(o,i) {if((t=-(ray.O.x+o)*ray.rD.x)<ray.t)ray.t=t,ray.objIdx=i;}
#define PLANE_Y(o,i) {if((t=-(ray.O.y+o)*ray.rD.y)<ray.t)ray.t=t,ray.objIdx=i;}
#define PLANE_Z(o,i) {if((t=-(ray.O.z+o)*ray.rD.z)<ray.t)ray.t=t,ray.objIdx=i;}

namespace Tmpl8 {

	struct BVHNode {
		int c1, c2, c3, c4;	//index of children
		int n, index;		//n children in leaf node, index is idex of the first shape                     
		aabb aabb;			//aabb
	};

	struct Bin {
		int n;
		aabb aabb;
	};

	struct Photon {
		float3 origin;
		float3 direction;
		float3 power;
	};

	struct KDNode {
		int left, right;
		int index, axis;
	};

	struct Compare {
		bool operator() (const std::pair<double, int>& a, const std::pair<double, int>& b) const {
			return a.first > b.first;
		}
	};

	const float BRIGHTNESS = 2.0f * 3.1415926f;
	const float INF = 1e30f;

	static long long raycount = 0;
	static long double alldepth = 0;

	static long long bvhcount = 0;
	static long double totaltime = 0;

	enum MatType {
		Air,
		Basic,
		Diffuse,
		Glass,
		Glossy,
		Light,
		Mirror,
		Water
	};

	enum Media {
		AirToGlass,
		GlassToAir,
		AirToWater,
		WaterToAir,
		WaterToGlass,
		GlassToWater
	};

	inline map<Media, float> refractive =
	{
		{AirToGlass, 0.6666666667},
		{GlassToAir, 1.5},
		{AirToWater, 0.75},
		{WaterToAir, 1.3333333333},
		{WaterToGlass, 0.8888888888},
		{GlassToWater, 1.125}
	};

	class Material {
	public:
		Material() = default;
		Material(float3 albedo, MatType type) : 
		albedo(albedo), type(type) {}

		float3 albedo;
		MatType type;
	};

	inline float mht(float3 a, float3 b) {
		return abs(a.x - b.x) + abs(a.y - b.y) + abs(a.z - b.z);
	}

	inline float3 f3sqrt(float3 vector) {
		return float3(sqrt(vector.x), sqrt(vector.y), sqrt(vector.z));
	}

	inline float3 Absorb(float3 color, float distance, float3 absorption)
	{
		float3 output = color;
		output.x *= exp(absorption.x * distance);
		output.y *= exp(absorption.y * distance);
		output.z *= exp(absorption.z * distance);
		return output;
	}

	inline float3 mix(float3 x, float3 y, float a) {
		return (1 - a) * x + a * y;
	}

	inline float3 random_in_hemisphere(float3 N) {
		while (true) {
			float3 result;
			float x = rand() * (2.0 / RAND_MAX) - 1.0;
			float y = rand() * (2.0 / RAND_MAX) - 1.0;
			float z = rand() * (2.0 / RAND_MAX) - 1.0;
			result = float3(x, y, z);
			if (length(result) <= 1.0) {
				if (dot(result, N) < 0) return normalize(-result);
				return normalize(result);
			}
		}
	}

	inline float3 CosineWeightedDistribution(float3 N) {
		float r0 = rand() * (1.0 / RAND_MAX);
		float r1 = rand() * (1.0 / RAND_MAX);
		float r = sqrt(1 - r0);
		float theta = 2 * PI * r1;
		float x = r * cosf(theta);
		float y = r * sinf(theta);
		return normalize(N + float3(x, y, sqrt(1 - r0)));
	}

	__declspec(align(64)) class Ray
	{
	public:
		Ray() = default;
		Ray(float3 origin, float3 direction, float distance = 1e34f, 
			MatType media = Air, float3 energy = float3(1.0)) :
			media(media), energy(energy)
		{
			O = origin, D = direction, t = distance;
			// calculate reciprocal ray direction for triangles and AABBs
			rD = float3(1 / D.x, 1 / D.y, 1 / D.z);
#ifdef SPEEDTRIX
			d0 = d1 = d2 = 0;
#endif
		}
		float3 IntersectionPoint() { return O + t * D; }
		// ray data
#ifndef SPEEDTRIX
		float3 O, D, rD;
#else
		union { struct { float3 O; float d0; }; __m128 O4; };
		union { struct { float3 D; float d1; }; __m128 D4; };
		union { struct { float3 rD; float d2; }; __m128 rD4; };
#endif
		MatType media;
		float t = 1e34f;
		int objIdx = -1;
		bool inside = false; // true when in medium
		float3 energy;
	};

	class Shape
	{
	public:
		virtual void Intersect(Ray& ray) const = 0;
		virtual float3 GetNormal(const float3 I) const = 0;
		virtual float3 GetCenter() const = 0;
		virtual aabb GetAABB() const = 0;
		virtual Photon GetPhoton() const = 0;
		
		int objIdx = -1;
		float area;
		Material material;
		float3 center;
		mat4 T, invT;
	};

	// -----------------------------------------------------------
	// Sphere Shape
	// Basic sphere, with explicit support for rays that start
	// inside it. Good candidate for a dielectric material.
	// -----------------------------------------------------------
	class Sphere : public Shape
	{
	public:
		Sphere() = default;
		Sphere(int objIdx, float3 p, float r, Material material, mat4 transform = mat4::Identity()) : r(r), invr(1 / r)
		{
			this->objIdx = objIdx;
			this->area = 4 * PI * r * r;
			this->material = material;
			this->center = p;
			this->T = transform, this->invT = transform.FastInvertedTransformNoScale();
		}
		void Intersect(Ray& ray) const
		{
			float3 oc = TransformPosition(ray.O, T) - this->GetCenter();
			float b = dot(oc, TransformVector(ray.D, T));
			float c = dot(oc, oc) - this->r * this->r;
			float t, d = b * b - c;
			if (d <= 0) return;
			d = sqrtf(d), t = -b - d;
			if (t < ray.t && t > 0)
			{
				ray.t = t, ray.objIdx = this->objIdx;
				return;
			}
			t = d - b;
			if (t < ray.t && t > 0)
			{
				ray.t = t, ray.objIdx = this->objIdx;
				return;
			}
		}
		float3 GetNormal(const float3 I) const
		{
			return (I - this->GetCenter()) * invr;
		}
		float3 GetCenter() const
		{
			return TransformPosition(center, T);
		}
		aabb GetAABB() const
		{
			return aabb(this->GetCenter() - r, this->GetCenter() + r);
		}
		Photon GetPhoton() const {
			Photon result;
			float3 dir;
			float x = rand() * (2.0 / RAND_MAX) - 1.0;
			float y = rand() * (2.0 / RAND_MAX) - 1.0;
			float z = rand() * (2.0 / RAND_MAX) - 1.0;
			dir = float3(x, y, z);
		
			result.origin = this->GetCenter() + dir * r;
			result.direction = CosineWeightedDistribution(dir);
			result.power = float3(1);
			return result;
		}
		float r, invr;
	};

	// -----------------------------------------------------------
	// Cube Shape
	// Oriented cube. Unsure if this will also work for rays that
	// start inside it; maybe not the best candidate for testing
	// dielectrics.
	// -----------------------------------------------------------
	class Cube : public Shape
	{
	public:
		Cube() = default;
		Cube(int objIdx, float3 pos, float3 size, Material material, mat4 transform = mat4::Identity())
		{
			this->objIdx = objIdx;
			this->area = 6 * size.x * size.y * size.z;
			this->material = material;
			this->center = pos;
			b[0] = pos - 0.5f * size, b[1] = pos + 0.5f * size;
			this->T = transform, this->invT = transform.FastInvertedTransformNoScale();
		}
		void Intersect(Ray& ray) const
		{
			// 'rotate' the cube by transforming the ray into object space
			// using the inverse of the cube transform.
			float3 O = TransformPosition(ray.O, invT);
			float3 D = TransformVector(ray.D, invT);
			float rDx = 1 / D.x, rDy = 1 / D.y, rDz = 1 / D.z;
			int signx = D.x < 0, signy = D.y < 0, signz = D.z < 0;
			float tmin = (b[signx].x - O.x) * rDx;
			float tmax = (b[1 - signx].x - O.x) * rDx;
			float tymin = (b[signy].y - O.y) * rDy;
			float tymax = (b[1 - signy].y - O.y) * rDy;
			if (tmin > tymax || tymin > tmax) return;
			tmin = max(tmin, tymin), tmax = min(tmax, tymax);
			float tzmin = (b[signz].z - O.z) * rDz;
			float tzmax = (b[1 - signz].z - O.z) * rDz;
			if (tmin > tzmax || tzmin > tmax) return;
			tmin = max(tmin, tzmin), tmax = min(tmax, tzmax);
			if (tmin > 0)
			{
				if (tmin < ray.t) ray.t = tmin, ray.objIdx = this->objIdx;
			}
			else if (tmax > 0)
			{
				if (tmax < ray.t) ray.t = tmax, ray.objIdx = this->objIdx;
			}
		}
		float3 GetNormal(const float3 I) const
		{
			// transform intersection point to object space
			float3 objI = TransformPosition(I, invT);
			// determine normal in object space
			float3 N = float3(-1, 0, 0);
			float d0 = fabs(objI.x - b[0].x), d1 = fabs(objI.x - b[1].x);
			float d2 = fabs(objI.y - b[0].y), d3 = fabs(objI.y - b[1].y);
			float d4 = fabs(objI.z - b[0].z), d5 = fabs(objI.z - b[1].z);
			float minDist = d0;
			if (d1 < minDist) minDist = d1, N.x = 1;
			if (d2 < minDist) minDist = d2, N = float3(0, -1, 0);
			if (d3 < minDist) minDist = d3, N = float3(0, 1, 0);
			if (d4 < minDist) minDist = d4, N = float3(0, 0, -1);
			if (d5 < minDist) minDist = d5, N = float3(0, 0, 1);
			// return normal in world space
			return TransformVector(N, T);
		}
		float3 GetCenter() const
		{
			return TransformPosition(center, T);
		}
		aabb GetAABB() const
		{
			float3 b0 = TransformPosition(b[0], T);
			float3 b1 = TransformPosition(b[1], T);
			return aabb(fminf(b0, b1), fmaxf(b0, b1));
		}

		Photon GetPhoton() const {
			Photon result;
			
			return result;
		}
		float3 b[2];
	};

	// -----------------------------------------------------------
	// Plane Shape
	// Oriented plane.
	// -----------------------------------------------------------
	class Plane : public Shape
	{
	public:
		Plane() = default;
		Plane(int objIdx, float3 pos, float s, Material material, mat4 transform = mat4::Identity())
		{
			this->center = pos;
			this->area = s * s;
			this->objIdx = objIdx;
			this->material = material;
			this->T = transform, this->invT = transform.FastInvertedTransformNoScale();
			size = 0.5 * s;
		}
		void Intersect(Ray& ray) const
		{
			const float3 O = TransformPosition(ray.O, invT);
			const float3 D = TransformVector(ray.D, invT);
			const float t = O.y / -D.y;
			if (t < ray.t && t > 0)
			{
				float3 I = O + t * D;
				if (I.x > -size && I.x < size && I.z > -size && I.z < size)
					ray.t = t, ray.objIdx = this->objIdx;
			}
		}
		float3 GetNormal(const float3 I) const
		{
			return normalize(float3(-T.cell[1], -T.cell[5], -T.cell[9]));
		}
		float3 GetCenter() const
		{
			return TransformPosition(center, T);
		}
		aabb GetAABB() const
		{
			float3 corner1 = TransformPosition(center + float3(- size, 0, -size), T);
			float3 corner2 = TransformPosition(center + float3(size, 0, size), T);
			return aabb(fminf(corner1, corner2), fmaxf(corner1, corner2));
		}

		Photon GetPhoton() const {
			Photon result;
			float x = rand() * (2.0 / RAND_MAX) - 1.0;
			float y = rand() * (2.0 / RAND_MAX) - 1.0;
			result.direction = CosineWeightedDistribution(this->GetNormal(float3(0)));
			result.origin = TransformPosition(center + float3(size * x, 0, size * y), T);
			result.power = float(1);
			return result;
		}
		float size;
	};

	class Triangle : public Shape
	{
	public:
	public:
		Triangle() = default;
		Triangle(int objIdx, float3 v0, float3 v1, float3 v2, Material material, mat4 transform = mat4::Identity()) :
			v0(v0), v1(v1), v2(v2)
		{
			N = normalize(cross(v1-v0, v2-v0));
			this->objIdx = objIdx;
			this->area = abs(length(cross(v1 - v2, v0 - v2))) / 2;
			this->material = material;
			this->T = transform, this->invT = transform.FastInvertedTransformNoScale();
			this->center = (v0 + v1 + v2) * 0.3333333333;
		}
		void Intersect(Ray& ray) const
		{
			float3 O = TransformPosition(ray.O, invT);
			float3 D = TransformVector(ray.D, invT);

			const float3 edge1 = v1 - v0;
			const float3 edge2 = v2 - v0;
			const float3 h = cross(ray.D, edge2);
			const float a = dot(edge1, h);
			if (a > -0.0001f && a < 0.0001f) return; // ray parallel to triangle
			const float f = 1 / a;
			const float3 s = O - v0;
			const float u = f * dot(s, h);
			if (u < 0 || u > 1) return;
			const float3 q = cross(s, edge1);
			const float v = f * dot(D, q);
			if (v < 0 || u + v > 1) return;
			const float t = f * dot(edge2, q);
			if (t > 0.0001f) {
				ray.t = min(ray.t, t);
				ray.objIdx = this->objIdx;
			}
		}
		float3 GetNormal(const float3 I) const
		{
			return TransformVector(N, T);
		}
		float3 GetCenter() const
		{
			return TransformPosition(center, invT);
		}
		aabb GetAABB() const
		{   
			return aabb(fminf(fminf(v0, v1), v2), fmaxf(fmaxf(v0, v1), v2));
		}

		Photon GetPhoton() const {
			Photon result;

			return result;
		}
		float3 v0, v1, v2, N;
	};

	class TriangleMesh
	{
	public:
		TriangleMesh() = default;
		TriangleMesh(int mesh, const string& file_name) {
			// Open the file
			ifstream file(file_name);
			if (!file.is_open()) {
				cerr << "Error: Unable to open file " << file_name << endl;
				return;
			}

			int faceIdx = 0;

			material = Material(float3(1, 0.5, 0), Diffuse);

			// Read the file line by line
			string line;
			while (getline(file, line)) {
				stringstream ss(line);

				// Split the line into tokens
				string token;
				ss >> token;

				// Parse the tokens
				if (token == "v") {
					// Parse vertex coordinates
					float3 vertex;
					ss >> vertex.x >> vertex.y >> vertex.z;
					vertices.push_back(vertex);
				}
				else if (token == "f") {
					int v0, v1, v2;
					ss >> v0 >> v1 >> v2;
					v0--;
					v1--;
					v2--;
					triangles.push_back(Triangle(-1, vertices[v0]*10, vertices[v1]*10, vertices[v2]*10, material, mat4::Identity()));
					faceIdx++;
				}
			}
		}
		
		vector<float3> vertices;
		vector<Triangle> triangles;
		Material material;
	};

	inline bool cmpx(const Shape* s1, const Shape* s2) {
		return s1->GetAABB().Center(0) < s2->GetAABB().Center(0);
	}
	inline bool cmpy(const Shape* s1, const Shape* s2) {
		return s1->GetAABB().Center(1) < s2->GetAABB().Center(1);
	}
	inline bool cmpz(const Shape* s1, const Shape* s2) {
		return s1->GetAABB().Center(2) < s2->GetAABB().Center(2);
	}

	inline bool cmppx(Photon p1, Photon p2) {
		return  p1.origin.x < p2.origin.x;
	}
	inline bool cmppy(Photon p1, Photon p2) {
		return  p1.origin.y < p2.origin.y;
	}
	inline bool cmppz(Photon p1, Photon p2) {
		return  p1.origin.z < p2.origin.z;
	}

	inline float IntersectAABB(const Ray & ray, const aabb box)
	{
		float3 bmin = box.bmin3;
		float3 bmax = box.bmax3;
		float tx1 = (bmin.x - ray.O.x) * ray.rD.x, tx2 = (bmax.x - ray.O.x) * ray.rD.x;
		float tmin = min(tx1, tx2), tmax = max(tx1, tx2);
		float ty1 = (bmin.y - ray.O.y) * ray.rD.y, ty2 = (bmax.y - ray.O.y) * ray.rD.y;
		tmin = max(tmin, min(ty1, ty2)), tmax = min(tmax, max(ty1, ty2));
		float tz1 = (bmin.z - ray.O.z) * ray.rD.z, tz2 = (bmax.z - ray.O.z) * ray.rD.z;
		tmin = max(tmin, min(tz1, tz2)), tmax = min(tmax, max(tz1, tz2));
		if (tmax >= tmin && tmin < ray.t && tmax > 0) return tmin; else return INF;
	}

	// -----------------------------------------------------------
	// Scene class
	// We intersect this. The query is internally forwarded to the
	// list of primitives, so that the nearest hit can be returned.
	// For this hit (distance, obj id), we can query the normal and
	// albedo.
	// -----------------------------------------------------------
	class Scene
	{
	public:
		Scene()
		{
			// we store all primitives in one continuous buffer
			Material red = Material(float3(0.8, 0, 0), Diffuse);
			Material green = Material(float3(0, 1, 0), Diffuse);
			Material blue = Material(float3(0.1, 0.3, 1), Diffuse);
			Material purple = Material(float3(0.9, 0.2, 0.9), Glass);
			Material yellow = Material(float3(1, 0.8, 0), Mirror);
			Material white = Material(float3(1, 1, 1), Diffuse);
			Material light = Material(float3(1, 1, 1), Light);
			Material mirror = Material(float3(1, 1, 1), Mirror);

			mat4 leftT = mat4::Translate(float3(-7, 3, 0)) * mat4::RotateZ(PI / 2);
			mat4 rightT = mat4::Translate(float3(7, 3, 0)) * mat4::RotateZ(-PI / 2);
			mat4 topT = mat4::Translate(float3(0, 6, 0));
			mat4 botT = mat4::Translate(float3(0, -1, 0)) * mat4::RotateZ(PI);
			mat4 frontT = mat4::Translate(float3(0, 3, -7)) * mat4::RotateX(PI / 2);
			mat4 backT = mat4::Translate(float3(0, 3, 7)) * mat4::RotateX(-PI / 2);;

			plane[0] = Plane(0, float3(0), 4, light, mat4::Translate(0, 6, 0));	            // light source
			plane[1] = Plane(1, float3(0), 14, red, leftT);									// left wall
			plane[2] = Plane(2, float3(0), 14, blue, rightT);								// right wall
			plane[3] = Plane(3, float3(0), 14, white, botT);								// floor
			plane[4] = Plane(4, float3(0), 14, light, topT);								// ceiling
			plane[5] = Plane(5, float3(0), 14, green, frontT);								// front wall
			plane[6] = Plane(6, float3(0), 14, white, backT);								// back wall
			sphere = Sphere(7, float3(-2, 0, 1.5), 0.75f, yellow, mat4::Identity());		// yellow mirror sphere
			cube = Cube(8, float3(2, 0, 1.5), float3(1.15f), purple, mat4::Identity());		// purple glass cube

			for (int i = 0; i < 7; i++) {
				shapes.push_back(&plane[i]);
			}

			shapes.push_back(&sphere);
			shapes.push_back(&cube);

			mesh = TriangleMesh(0, "assets/bunny.obj");

			for (int i = 0; i < mesh.triangles.size(); i++) {
				shapes.push_back(&mesh.triangles[i]);
			}

			lights.push_back(&plane[0]);

			SetTime(0);

		}

		void SetTime(float t)
		{
			// default time for the scene is simply 0. Updating/ the time per frame 
			// enables animation. Updating it per ray can be used for motion blur.
			animTime = t;

			//// light source animation: swing
			//mat4 M1base = mat4::Translate(float3(0, 5.0f, 2));
			//mat4 M1 = M1base * mat4::RotateZ(sinf(animTime * 0.6f) * 0.1f) * mat4::Translate(float3(0, -0.9, 0));
			//plane[0].T = M1, plane[0].invT = M1.FastInvertedTransformNoScale();

			//// cube animation: spin
			//mat4 M2base = mat4::RotateX(PI / 4) * mat4::RotateZ(PI / 4);
			//mat4 M2 = mat4::Translate(float3(2.0f, 0, 2)) * mat4::RotateY(animTime * 0.5f) * M2base;
			//cube.T = M2, cube.invT = M2.FastInvertedTransformNoScale();

			//// sphere animation: bounce
			//float tm = 1 - sqrf(fmodf(animTime, 2.0f) - 1);
			//sphere.center = float3(-2.0f, -0.5f + tm, 2);

			//mat4 M2 = mat4::Translate(float3(2.0f, 0, 2));
			//cube.T = M2, cube.invT = M2.FastInvertedTransformNoScale();

			vector<BVHNode>().swap(nodes);
			vector<Photon>().swap(photons);
			vector<KDNode>().swap(photonMap);

			int temp = BuildQBVH(0, shapes.size() - 1, 2);

			for (int i = 0; i < shapes.size(); i++) {
				shapes[i]->objIdx = i;
			}

			GeneratePhoton(100000);
		}

		inline int FindSplit(int lLocal, int rLocal, int& Axis) {
			if (rLocal > shapes.size() - 1) rLocal = shapes.size() - 1;
			if (lLocal > shapes.size() - 1 || lLocal >= rLocal) return -1;

			//Get the least cost, skip the computation involving the current node
			float Cost = INF;
			//Get the optimal split position
			int Split;

			vector<float> leftS, rightS;
			aabb aabbLeft, aabbRight;
			int leftCount, rightCount;

			//For each axis, find the optimal position
			for (int axis = 0; axis < 3; axis++) {
				if (axis == 0) sort(&shapes[0] + lLocal, &shapes[0] + rLocal + 1, cmpx);
				if (axis == 1) sort(&shapes[0] + lLocal, &shapes[0] + rLocal + 1, cmpy);
				if (axis == 2) sort(&shapes[0] + lLocal, &shapes[0] + rLocal + 1, cmpz);

				//Get the optimal split position along this axis
				int split;
				//Get the least cost along this axis
				float cost = INF;

				//Compute the left node surface area when the split position is different
				vector<float>().swap(leftS);
				aabbLeft = aabb(float3(INF), float3(-INF));
				for (int i = lLocal; i < rLocal; i++) {
					aabbLeft = aabbLeft.Union(this->shapes[i]->GetAABB());
					leftS.push_back(aabbLeft.Area());
				}

				//Compute the right node surface area when the split position is different
				vector<float>().swap(rightS);
				aabbRight = aabb(float3(INF), float3(-INF));
				for (int i = rLocal; i > lLocal; i--) {
					aabbRight = aabbRight.Union(this->shapes[i]->GetAABB());
					rightS.push_back(aabbRight.Area());
				}

				leftCount = 0;
				rightCount = rLocal - lLocal + 1;

				//Compute the total cost when the split position is different
				float totalCost;
				for (int i = 0; i < rLocal - lLocal; i++) {
					leftCount += 1;
					rightCount -= 1;

					float leftCost = leftCount * leftS[i];
					float rightCost = rightCount * rightS[rLocal - lLocal - i - 1];

					totalCost = leftCost + rightCost;
					if (totalCost < cost) {
						cost = totalCost;
						split = lLocal + i;
					}
				}

				//If the cost when split along the current axis at the local optimal split
				//position is better, update the information
				if (cost < Cost) {
					Axis = axis;
					Cost = cost;
					Split = split;
				}
			}

			return Split;
		}

		//int BuildBVH(int l, int r, int n) {
		//	//no nodes 
		//	if (l > r) return -1;
		//
		//	//generate newest node
		//	int idx = nodes.size();
		//	nodes.push_back(BVHNode());
		//	nodes[idx].c1 = -1;
		//	nodes[idx].c2 = -1;
		//	nodes[idx].c3 = -1;
		//	nodes[idx].c4 = -1;
		//	nodes[idx].n = -1;
		//	nodes[idx].index = -1;
		//
		//	nodes[idx].aabb = aabb(float3(INF), float3(-INF));
		//
		//	//update node boundary
		//	for (int i = l; i <= r; i++) {
		//		nodes[idx].aabb = nodes[idx].aabb.Union(this->shapes[i]->GetAABB());
		//	}
		//
		//	//if less than n shapes in this node, then this is a leaf node
		//	if ((r - l + 1) <= n) {
		//		nodes[idx].n = r - l + 1;
		//		nodes[idx].index = l;
		//		return idx;
		//	}
		//
		//	int Axis = nodes[idx].aabb.LongestAxis();	//axis of the current split method
		//	int Split = (l + r) / 2;
		//
		//	if (Axis == 0) sort(&shapes[0] + l, &shapes[0] + r + 1, cmpx);
		//	if (Axis == 1) sort(&shapes[0] + l, &shapes[0] + r + 1, cmpy);
		//	if (Axis == 2) sort(&shapes[0] + l, &shapes[0] + r + 1, cmpz);
		//
		//	nodes[idx].c1 = BuildBVH(l, Split, n);
		//	nodes[idx].c2 = BuildBVH(Split + 1, r, n);
		//
		//	return idx;
		//}
		//
		//int BuildBVH_SAH(int l, int r, int n) {
		//	//no nodes 
		//	if (l > r) return -1;
		//
		//	//generate newest node
		//	int idx = nodes.size();
		//	nodes.push_back(BVHNode());
		//	nodes[idx].c1 = -1;
		//	nodes[idx].c2 = -1;
		//	nodes[idx].c3 = -1;
		//	nodes[idx].c4 = -1;
		//	nodes[idx].n = -1;
		//	nodes[idx].index = -1;
		//	nodes[idx].aabb = aabb(float3(INF), float3(-INF));
		//
		//	//update node boundary
		//	for (int i = l; i <= r; i++) {
		//		nodes[idx].aabb = nodes[idx].aabb.Union(this->shapes[i]->GetAABB());
		//	}
		//
		//	//if less than n shapes in this node, then this is a leaf node
		//	if ((r - l + 1) <= n) {
		//		nodes[idx].n = r - l + 1;
		//		nodes[idx].index = l;
		//		return idx;
		//	}
		//
		//	//Get the optimal split axis
		//	int Axis;
		//	
		//	int Split = FindSplit(l, r, Axis);
		//
		//	if (Axis == 0) sort(&shapes[0] + l, &shapes[0] + r + 1, cmpx);
		//	if (Axis == 1) sort(&shapes[0] + l, &shapes[0] + r + 1, cmpy);
		//	if (Axis == 2) sort(&shapes[0] + l, &shapes[0] + r + 1, cmpz);
		//
		//	nodes[idx].c1 = BuildBVH_SAH(l, Split, n);
		//	nodes[idx].c2 = BuildBVH_SAH(Split + 1, r, n);
		//
		//	return idx;
		//}

		int BuildQBVH(int l, int r, int n) {
			//no nodes 
			if (l > r) return -1;

			//generate newest node
			int idx = nodes.size();
			nodes.push_back(BVHNode());
			nodes[idx].c1 = -1;
			nodes[idx].c2 = -1;
			nodes[idx].c3 = -1;
			nodes[idx].c4 = -1;
			nodes[idx].n = -1;
			nodes[idx].index = -1;
			nodes[idx].aabb = aabb(float3(INF), float3(-INF));

			//update node boundary
#	pragma omp parallel for schedule(dynamic)
			for (int i = l; i <= r; i++) {
				nodes[idx].aabb = nodes[idx].aabb.Union(shapes[i]->GetAABB());
			}

			//if less than n shapes in this node, then this is a leaf node
			if ((r - l + 1) <= n) {
				nodes[idx].n = r - l + 1;
				nodes[idx].index = l;
				return idx;
			}

			//axis of the current split method
			int Axis;
			int Middle, MidLeft, MidRight;

			Middle = FindSplit(l, r, Axis);

			if (Axis == 0) sort(&shapes[0] + l, &shapes[0] + r + 1, cmpx);
			if (Axis == 1) sort(&shapes[0] + l, &shapes[0] + r + 1, cmpy);
			if (Axis == 2) sort(&shapes[0] + l, &shapes[0] + r + 1, cmpz);

			MidLeft = FindSplit(l, Middle, Axis);
			MidLeft = MidLeft == -1 ? l : MidLeft;

			if (Axis == 0) sort(&shapes[0] + l, &shapes[0] + Middle + 1, cmpx);
			if (Axis == 1) sort(&shapes[0] + l, &shapes[0] + Middle + 1, cmpy);
			if (Axis == 2) sort(&shapes[0] + l, &shapes[0] + Middle + 1, cmpz);

			MidRight = FindSplit(Middle + 1, r, Axis);
			MidRight = MidRight == -1 ? r - 1 : MidRight;

			if (Axis == 0) sort(&shapes[0] + Middle + 1, &shapes[0] + r + 1, cmpx);
			if (Axis == 1) sort(&shapes[0] + Middle + 1, &shapes[0] + r + 1, cmpy);
			if (Axis == 2) sort(&shapes[0] + Middle + 1, &shapes[0] + r + 1, cmpz);

			nodes[idx].c1 = BuildQBVH(l, MidLeft, n);
			nodes[idx].c2 = BuildQBVH(MidLeft + 1, Middle, n);
			nodes[idx].c3 = BuildQBVH(Middle + 1, MidRight, n);
			nodes[idx].c4 = BuildQBVH(MidRight + 1, r, n);

			return idx;
		}

		//inline int FindBinSplit(int lLocal, int rLocal, BVHNode& node, int& Axis, int bin) {
		//	if (rLocal > shapes.size() - 1) rLocal = shapes.size() - 1;
		//	if (lLocal > shapes.size() - 1 || lLocal >= rLocal) return -1;
		//
		//	//Get the least cost, skip the computation involving the current node
		//	float Cost = INF;
		//	//Get the optimal split position
		//	int Split = -1;
		//
		//	vector<float> leftS, rightS;
		//	aabb aabbLeft, aabbRight;
		//	int leftCount, rightCount;
		//	float boundsMin, boundsMax;
		//	Bin b;
		//	b.aabb = aabb(float3(INF), float3(-INF));
		//	b.n = 0;
		//	vector<Bin> bins;
		//
		//	//For each axis, find the optimal position
		//	for (int axis = 0; axis < 3; axis++) {
		//		int split;					//Get the optimal split position along this axis
		//		float cost = INF;			//Get the least cost along this axis
		//		vector<float>().swap(leftS);
		//		vector<float>().swap(rightS);
		//		boundsMin = INF;
		//		boundsMax = -INF;
		//		aabbLeft = aabb(float3(INF), float3(-INF));
		//		aabbRight = aabb(float3(INF), float3(-INF));
		//		vector<Bin>(bin, b).swap(bins);
		//		
		//		for (int i = lLocal; i <= rLocal; i++)
		//		{
		//			float a = shapes[i]->GetCenter()[axis];
		//			boundsMax = fmaxf(boundsMax, a);
		//			boundsMin = fminf(boundsMin, a);
		//		}
		//		
		//		float scale = bin / (boundsMax - boundsMin);
		//
		//		for (int i = lLocal; i <= rLocal; i++)
		//		{
		//			int binIdx = min(bin - 1,
		//				(int)((shapes[i]->GetCenter()[axis] - boundsMin) * scale));
		//			bins[binIdx].n++;
		//			bins[binIdx].aabb = bins[binIdx].aabb.Union(shapes[i]->GetAABB());
		//		}
		//
		//		//for (int i = 0; i < bin; i++) {
		//		//	printf("bin: %d, bin.n: %d, binS: %f\n", i, bins[i].n, bins[i].aabb.Area());
		//		//}
		//
		//		leftCount = 0;
		//		for (int i = 0; i < bin - 1; i++) {
		//			aabbLeft = aabbLeft.Union(bins[i].aabb);
		//			leftCount += bins[i].n;
		//			if (leftCount > 0) leftS.push_back(aabbLeft.Area());
		//			else leftS.push_back(INF);
		//		}
		//
		//		rightCount = rLocal - lLocal + 1;
		//		for (int i = bin - 1; i > 0; i--) {
		//			aabbRight = aabbRight.Union(bins[i].aabb);
		//			rightCount -= bins[i].n;
		//			if (rightCount > 0) rightS.push_back(aabbRight.Area());
		//			else rightS.push_back(INF);
		//		}
		//
		//		leftCount = 0;
		//		rightCount = rLocal - lLocal + 1;
		//
		//		//Compute the total cost when the split position is different
		//		float totalCost;
		//		for (int i = 0; i < bin - 1; i++) {
		//			if (bins[i].n == 0) continue;
		//			leftCount += bins[i].n;
		//			rightCount -= bins[i].n;
		//			float leftCost = leftCount * leftS[i];
		//			float rightCost = rightCount * rightS[bin - i - 2];
		//
		//			totalCost = leftCost + rightCost;
		//			if (totalCost < cost) {
		//				cost = totalCost;
		//				split = lLocal + leftCount - 1;
		//			}
		//		}
		//
		//		//If the cost when split along the current axis at the local optimal split
		//		//position is better, update the information
		//		if (cost < Cost) {
		//			Axis = axis;
		//			Cost = cost;
		//			Split = split;
		//		}
		//	}
		//
		//	return Split;
		//}
		//
		//int BuildBVH_BIN(int l, int r, int n, int bin) {
		//	//no nodes 
		//	if (l > r) return -1;
		//
		//	//generate newest node
		//	int idx = nodes.size();
		//	nodes.push_back(BVHNode());
		//	nodes[idx].c1 = -1;
		//	nodes[idx].c2 = -1;
		//	nodes[idx].c3 = -1;
		//	nodes[idx].c4 = -1;
		//	nodes[idx].n = -1;
		//	nodes[idx].index = -1;
		//	nodes[idx].aabb = aabb(float3(INF), float3(-INF));
		//
		//	//update node boundary
		//	for (int i = l; i <= r; i++) {
		//		nodes[idx].aabb = nodes[idx].aabb.Union(shapes[i]->GetAABB());
		//	}
		//
		//	//if less than n shapes in this node, then this is a leaf node
		//	if ((r - l + 1) <= n) {
		//		nodes[idx].n = r - l + 1;
		//		nodes[idx].index = l;
		//		return idx;
		//	}
		//
		//	//Get the optimal split axis
		//	int Axis;
		//	int Split;
		//
		//	Split = FindSplit(l, r, Axis);
		//
		//	if (Axis == 0) sort(&shapes[0] + l, &shapes[0] + r + 1, cmpx);
		//	if (Axis == 1) sort(&shapes[0] + l, &shapes[0] + r + 1, cmpy);
		//	if (Axis == 2) sort(&shapes[0] + l, &shapes[0] + r + 1, cmpz);
		//
		//	nodes[idx].c1 = BuildBVH_BIN(l, Split, n, bin);
		//	nodes[idx].c2 = BuildBVH_BIN(Split + 1, r, n, bin);
		//
		//	return idx;
		//}
		//
		//int BuildQBVH_BIN(int l, int r, int n, int bin) {
		//	//no nodes 
		//	if (l > r) return -1;
		//
		//	//generate newest node
		//	int idx = nodes.size();
		//	nodes.push_back(BVHNode());
		//	nodes[idx].c1 = -1;
		//	nodes[idx].c2 = -1;
		//	nodes[idx].c3 = -1;
		//	nodes[idx].c4 = -1;
		//	nodes[idx].n = -1;
		//	nodes[idx].index = -1;
		//	nodes[idx].aabb = aabb(float3(INF), float3(-INF));
		//
		//	//update node boundary
		//	for (int i = l; i <= r; i++) {
		//		nodes[idx].aabb = nodes[idx].aabb.Union(shapes[i]->GetAABB());
		//	}
		//
		//	//if less than n shapes in this node, then this is a leaf node
		//	if ((r - l + 1) <= n) {
		//		nodes[idx].n = r - l + 1;
		//		nodes[idx].index = l;
		//		return idx;
		//	}
		//
		//	//axis of the current split method
		//	int Axis;
		//	int Middle, MidLeft, MidRight;
		//
		//	Middle = FindBinSplit(l, r, nodes[idx], Axis, bin);
		//
		//	if (Axis == 0) sort(&shapes[0] + l, &shapes[0] + r + 1, cmpx);
		//	if (Axis == 1) sort(&shapes[0] + l, &shapes[0] + r + 1, cmpy);
		//	if (Axis == 2) sort(&shapes[0] + l, &shapes[0] + r + 1, cmpz);
		//
		//	if (Middle - l > bin) {
		//		MidLeft = FindBinSplit(l, Middle, nodes[idx], Axis, bin);
		//	}
		//	else {
		//		MidLeft = FindSplit(l, Middle, Axis);
		//	}
		//	MidLeft = MidLeft == -1 ? l : MidLeft;
		//
		//	if (Axis == 0) sort(&shapes[0] + l, &shapes[0] + Middle + 1, cmpx);
		//	if (Axis == 1) sort(&shapes[0] + l, &shapes[0] + Middle + 1, cmpy);
		//	if (Axis == 2) sort(&shapes[0] + l, &shapes[0] + Middle + 1, cmpz);
		//
		//	if (Middle - l > bin) {
		//		MidRight = FindBinSplit(Middle + 1, r, nodes[idx], Axis, bin);
		//	}
		//	else {
		//		MidRight = FindSplit(Middle + 1, r, Axis);
		//	}
		//	MidRight = MidRight == -1 ? r - 1 : MidRight;
		//
		//	if (Axis == 0) sort(&shapes[0] + Middle + 1, &shapes[0] + r + 1, cmpx);
		//	if (Axis == 1) sort(&shapes[0] + Middle + 1, &shapes[0] + r + 1, cmpy);
		//	if (Axis == 2) sort(&shapes[0] + Middle + 1, &shapes[0] + r + 1, cmpz);
		//
		//	nodes[idx].c1 = BuildQBVH(l, MidLeft, n);
		//	nodes[idx].c2 = BuildQBVH(MidLeft + 1, Middle, n);
		//	nodes[idx].c3 = BuildQBVH(Middle + 1, MidRight, n);
		//	nodes[idx].c4 = BuildQBVH(MidRight + 1, r, n);
		//
		//	return idx;
		//}

		void IntersectBVH(Ray& ray, int nodeIdx) const
		{
			if (nodeIdx < 0) return;
			BVHNode node = nodes[nodeIdx];
			if (IntersectAABB(ray, node.aabb) == INF) return;
			if (node.n > 0)
			{
				for (int i = 0; i < node.n; i++)
					shapes[node.index + i]->Intersect(ray);
			}
			else
			{
				IntersectBVH(ray, node.c1);
				IntersectBVH(ray, node.c2);
				IntersectBVH(ray, node.c3);
				IntersectBVH(ray, node.c4);
			}
		}

		void FindNearest(Ray& ray) const
		{
			IntersectBVH(ray, 0);
		}

		bool IsOccluded(Ray& ray) const
		{
			float rayLength = ray.t;
			IntersectBVH(ray, 0);
			return ray.t < rayLength&& GetObjMatType(ray.objIdx) != Light;
		}

		inline void PhotonTrace(Ray& ray, int iter = 0) {
			FindNearest(ray);
			float3 I = ray.O + ray.t * ray.D;
			float3 N = GetNormal(ray.objIdx, I, ray.D);
			float cos1 = dot(N, -ray.D);
			float3 energy;
			
			if (iter > 5) return;

			if (ray.objIdx == -1) return;
			MatType mat = GetObjMatType(ray.objIdx);

			if (mat == Light) return;
			
			if (mat == Diffuse) {
				float3 BRDF = INVPI * GetAlbedo(ray.objIdx, I);
				float3 randomRayDir = random_in_hemisphere(N);
				energy = 1.25 * ray.energy * BRDF * dot(N, randomRayDir);
				float r = rand() * (1.0 / RAND_MAX);
				float P = 0.8;
				if (r < P) {
					Ray randomRay = Ray(I + randomRayDir * 0.001, randomRayDir, INF, Air, energy);
					PhotonTrace(randomRay, iter + 1);
				}
				else {
					Photon result;
					result.origin = I;
					result.power = energy;
					result.direction = randomRayDir;
					photons.push_back(result);
					return;
				}
			}
			if (mat == Mirror) {
				energy = cos1 * ray.energy * GetAlbedo(ray.objIdx, I);
				float3 reflectRayDir = normalize(reflect(ray.D, N));
				Ray mirrorRay = Ray(I + reflectRayDir * 0.001, reflectRayDir, INF, ray.media, energy);
				PhotonTrace(mirrorRay, iter + 1);
			}
			if (mat == Glass)
			{
				float k;
				float cos2;
				if (ray.media == Air)
				{
					cos2 = sqrt(1 - pow(refractive[AirToGlass] * sqrt(1 - pow(cos1, 2)), 2));
					k = 1 - pow(refractive[AirToGlass], 2) * (1 - pow(cos1, 2));

					float Fr = 0.5 * ((pow((cos1 - refractive[GlassToAir] * cos2) / (cos1 + refractive[GlassToAir] * cos2), 2)) + (pow((cos2 - refractive[GlassToAir] * cos1) / (cos2 + refractive[GlassToAir] * cos1), 2)));

					float p = rand() * (1.0 / RAND_MAX);

					if (p > Fr) {
						float3 refractRayDir = normalize(-cos2 * N + refractive[AirToGlass] * (ray.D + cos1 * N));
						Ray refractRay = Ray(I + refractRayDir * 0.001, refractRayDir, INF, Glass, ray.energy);
						PhotonTrace(refractRay, iter + 1);
					}
					else {
						float3 reflectRayDir = normalize(reflect(ray.D, N));
						Ray reflectRay = Ray(I + reflectRayDir * 0.001, reflectRayDir, INF, Glass, ray.energy);
						PhotonTrace(reflectRay, iter + 1);
					}
				}
				if (ray.media == Glass) {
					energy = Absorb(GetAlbedo(ray.objIdx, I), ray.t, ray.energy * 0.1);
					k = 1 - pow(refractive[GlassToAir], 2) * (1 - pow(cos1, 2));
					if (k < 0) {
						float3 reflectRayDir = normalize(reflect(ray.D, N));
						Ray reflectRay = Ray(I + reflectRayDir * 0.001, reflectRayDir, INF, Glass, energy);
						PhotonTrace(reflectRay, iter + 1);
					}
					else {
						cos2 = sqrt(1 - pow(refractive[GlassToAir] * sqrt(1 - pow(cos1, 2)), 2));

						float Fr = 0.5 * ((pow((refractive[GlassToAir] * cos1 - cos2) / (refractive[GlassToAir] * cos1 + cos2), 2)) + (pow((refractive[GlassToAir] * cos2 - cos1) / (refractive[GlassToAir] * cos2 + cos1), 2)));

						float p = rand() * (1.0 / RAND_MAX);

						if (p > Fr) {
							float3 refractRayDir = normalize(-cos2 * N + refractive[GlassToAir] * (ray.D + cos1 * N));
							Ray refractRay = Ray(I + refractRayDir * 0.001, refractRayDir, INF, Air, energy);
							PhotonTrace(refractRay, iter + 1);
						}
						else {
							float3 reflectRayDir = normalize(reflect(ray.D, N));
							Ray reflectRay = Ray(I + reflectRayDir * 0.001, reflectRayDir, INF, Glass, energy);
							PhotonTrace(reflectRay, iter + 1);
						}
					}
				}
			}
		}

		inline int BuildKDTree(int l, int r) {
			//no PhotonTree 
			if (l > r) return -1;

			//generate newest node
			int idx = photonMap.size();
			photonMap.push_back(KDNode());
			photonMap[idx].left = -1;
			photonMap[idx].right = -1;
			photonMap[idx].index = -1;
			photonMap[idx].axis = -1;

			if (l == r) {
				photonMap[idx].index = l;
				return idx;
			}

			aabb box = aabb(float3(INF), float3(-INF));

			//update node boundary
			for (int i = l; i <= r; i++) {
				box.Grow(photons[i].origin);
			}

			photonMap[idx].axis = box.LongestAxis();	//axis of the current split method
			int Split = (l + r) / 2;
			photonMap[idx].index = Split;

			if (photonMap[idx].axis == 0) sort(&photons[0] + l, &photons[0] + r + 1, cmppx);
			if (photonMap[idx].axis == 1) sort(&photons[0] + l, &photons[0] + r + 1, cmppy);
			if (photonMap[idx].axis == 2) sort(&photons[0] + l, &photons[0] + r + 1, cmppz);

			photonMap[idx].left = BuildKDTree(l, Split);
			photonMap[idx].right = BuildKDTree(Split + 1, r);

			return idx;
		}

		inline void GeneratePhoton(int n) {
			int num = n / lights.size();
			for (int i = 0; i < lights.size(); i++) {
				Shape* light = lights[i];
				for (int j = 0; j < num; j++) {
					Photon p = light->GetPhoton();
					Ray photonRay = Ray(p.origin + 0.001 * p.direction, p.direction);
					PhotonTrace(photonRay);
				}
			}
			BuildKDTree(0, photons.size() - 1);
		}

		inline void FindWithin(float3& I, float& r, int idx, std::set<std::pair<double, int>, Compare>& result) {
			if (idx == -1) return;

			int photon_idx = photonMap[idx].index;
			float3 origin = photons[photon_idx].origin;

			float dist2 = dot(I - origin, I - origin);
			float r2 = r * r;

			if (dist2 < r2) {
				result.insert({ dist2, photon_idx });
			}

			float dist = I[photonMap[idx].axis] - origin[photonMap[idx].axis];
			if (dist < r) {
				FindWithin(I, r, photonMap[idx].left, result);
			}
			if (dist > -r) {
				FindWithin(I, r, photonMap[idx].right, result);
			}
		}

		inline float3 GetRadiance(float3 I, float3 N, float3 D, float r, int n) {
			std::set<std::pair<double, int>, Compare> result;
			std::set<std::pair<double, int>, Compare>::iterator it;

			float3 power = float3(0);

			FindWithin(I, r, 0, result);
			it = result.begin();

			int l = min(int(result.size()), n);

			for (int i = 0; i < l; i++) {
				if (dot(photons[(*it).second].direction, D) < 0) power += photons[(*it).second].power * dot(photons[(*it).second].direction, -D);
				it++;
			}

			float invarea = INVPI / (r * r);

			return  invarea * power;
		}

		float3 GetLightPos() const
		{
			// light point position is the middle of the swinging quad
			return lights[0]->GetCenter() - float3(0, 0.01f, 0);
		}

		float3 GetLightColor() const
		{
			return float3(1);
		}

		MatType GetObjMatType(int objIdx) const
		{
			if (objIdx == -1) return Basic;
			return shapes[objIdx]->material.type;
		}

		float3 GetNormal(int objIdx, float3 I, float3 wo) const
		{
			if (objIdx == -1) return float3(0);
			float3 N;
			N = shapes[objIdx]->GetNormal(I);
			if (dot(N, wo) > 0) N = -N; // hit backside / inside
			return N;
		}

		float3 GetAlbedo(int objIdx, float3 I) const
		{
			if (objIdx == -1) return float3(0);
			return shapes[objIdx]->material.albedo;
		}

		float GetReflectivity(int objIdx, float3 I) const
		{
			if (objIdx == 1 /* ball */) return 1;
			if (objIdx == 6 /* floor */) return 0.3f;
			return 0;
		}

		float GetRefractivity(int objIdx, float3 I) const
		{
			return objIdx == 3 ? 1.0f : 0.0f;
		}

		float GetLightArea() const {
			return lights[0]->area;
		}

		float3 GetLightNormal(float3 I) const {
			return lights[0]->GetNormal(I);
		}

		float3 GetLightPoint() const {
			return lights[0]->GetPhoton().origin;
		}

		__declspec(align(64)) // start a new cacheline here
			float animTime = 0;
		Plane plane[7];
		Sphere sphere;
		Cube cube;
		TriangleMesh mesh;
		vector<Shape*> shapes;
		vector<Shape*> lights;
		vector<BVHNode> nodes;
		
		vector<Photon> photons;
		vector<KDNode> photonMap;

		int root;
	};
}