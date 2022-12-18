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

	const float BRIGHTNESS = 2.0f * 3.1415926f;
	const float INF = 1e34f;

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
		Material(float3 color, float3 albedo, MatType type) : 
		color(color), albedo(albedo), type(type) {}

		float3 color;
		float3 albedo;
		MatType type;
	};

	inline float3 fsqrt(float3 vector) {
		return float3(sqrt(vector.x), sqrt(vector.y), sqrt(vector.z));
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
				if (dot(result, N) < 0) return -result;
				return result;
			}
		}
	}

	__declspec(align(64)) class Ray
	{
	public:
		Ray() = default;
		Ray(float3 origin, float3 direction, float distance = 1e34f, 
			MatType media = Air) :
			media(media)
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
	};

	class Shape
	{
	public:
		virtual void Intersect(Ray& ray) const = 0;
		virtual float3 GetNormal(const float3 I) const = 0;
		virtual float3 GetCenter() const = 0;
		virtual vector<float3> GetAABB() const = 0;
		
		int objIdx = -1;
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
		Sphere(int objIdx, float3 p, float r, Material material) : r(r), invr(1 / r)
		{
			this->objIdx = objIdx;
			this->material = material;
			this->center = p;
		}
		void Intersect(Ray& ray) const
		{
			float3 oc = ray.O - this->center;
			float b = dot(oc, ray.D);
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
			return (I - this->center) * invr;
		}
		float3 GetCenter() const
		{
			return this->center;
		}
		vector<float3> GetAABB() const
		{
			vector<float3> aabb;
			aabb.push_back(this->center - r);
			aabb.push_back(this->center + r);
			return aabb;
		}
		float r, invr;
	};

	// -----------------------------------------------------------
	// Plane Shape
	// Basic infinite plane, defined by a normal and a distance
	// from the origin (in the direction of the normal).
	// -----------------------------------------------------------
	//class Plane : public Shape
	//{
	//public:
	//	Plane() = default;
	//	Plane(int objIdx, float3 normal, float dist, Material material) : N(normal), d(dist) 
	//	{
	//		this->objIdx = objIdx;
	//		this->material = material;
	//	}
	//	void Intersect(Ray& ray) const
	//	{
	//		float t = -(dot(ray.O, this->N) + this->d) / (dot(ray.D, this->N));
	//		if (t < ray.t && t > 0) ray.t = t, ray.objIdx = this->objIdx;
	//	}
	//	float3 GetNormal(const float3 I) const
	//	{
	//		return N;
	//	}
	//	BoundingBox GetAABB() const
	//	{
	//		return this->aabb;
	//	}
	//	float3 N;
	//	float d;
	//};

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
			return TransformPosition(center, invT);
		}
		vector<float3> GetAABB() const
		{
			vector<float3> aabb;
			float3 b0 = TransformPosition(b[0], invT);
			float3 b1 = TransformPosition(b[1], invT);
			aabb.push_back(fminf(b0, b1));
			aabb.push_back(fmaxf(b0, b1));
			return aabb;
		}
		float3 b[2];
	};

	// -----------------------------------------------------------
	// Quad Shape
	// Oriented quad, intended to be used as a light source.
	// -----------------------------------------------------------
	class Quad : public Shape
	{
	public:
		Quad() = default;
		Quad(int objIdx, float s, Material material, mat4 transform = mat4::Identity())
		{
			this->objIdx = objIdx;
			this->material = material;
			size = s * 0.5f;
			this->T = transform, this->invT = transform.FastInvertedTransformNoScale();
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
			// TransformVector( float3( 0, -1, 0 ), T ) 
			return float3(-T.cell[1], -T.cell[5], -T.cell[9]);
		}
		float3 GetCenter() const
		{
			float3 corner1 = TransformPosition(float3(-0.5f, 0, -0.5f), T);
			float3 corner2 = TransformPosition(float3(0.5f, 0, 0.5f), T);
			return (corner1 + corner2) * 0.5f;
		}
		vector<float3> GetAABB() const
		{
			vector<float3> aabb;
			float3 corner1 = TransformPosition(float3(-0.5f, 0, -0.5f), T);
			float3 corner2 = TransformPosition(float3(0.5f, 0, 0.5f), T);
			aabb.push_back(fminf(corner1, corner2));
			aabb.push_back(fmaxf(corner1, corner2));
			return aabb;
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
		vector<float3> GetAABB() const
		{
			vector<float3> aabb;
			aabb.push_back(fminf(fminf(v0, v1), v2));
			aabb.push_back(fmaxf(fmaxf(v0, v1), v2));
			return aabb;
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

			material = Material(float3(1, 0.5, 0), float3(0), Diffuse);

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
		return s1->GetCenter().x < s2->GetCenter().x;
	}
	inline bool cmpy(const Shape* s1, const Shape* s2) {
		return s1->GetCenter().y < s2->GetCenter().y;
	}
	inline bool cmpz(const Shape* s1, const Shape* s2) {
		return s1->GetCenter().z < s2->GetCenter().z;
	}

	struct BVHNode {
		int left, right;
		int n, index;                          
		float3 aabbMin, aabbMax;
	};

	inline bool IntersectAABB(const Ray & ray, const float3 bmin, const float3 bmax)
	{
		float tx1 = (bmin.x - ray.O.x) / ray.D.x, tx2 = (bmax.x - ray.O.x) / ray.D.x;
		float tmin = min(tx1, tx2), tmax = max(tx1, tx2);
		float ty1 = (bmin.y - ray.O.y) / ray.D.y, ty2 = (bmax.y - ray.O.y) / ray.D.y;
		tmin = max(tmin, min(ty1, ty2)), tmax = min(tmax, max(ty1, ty2));
		float tz1 = (bmin.z - ray.O.z) / ray.D.z, tz2 = (bmax.z - ray.O.z) / ray.D.z;
		tmin = max(tmin, min(tz1, tz2)), tmax = min(tmax, max(tz1, tz2));
		return tmax >= tmin && tmin < ray.t && tmax > 0;
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
			Material red = Material(float3(0.8, 0, 0), float3(0), Diffuse);
			Material green = Material(float3(0, 1, 0), float3(0), Diffuse);
			Material blue = Material(float3(0.1, 0.3, 1), float3(0), Diffuse);
			Material purple = Material(float3(0.9, 0.2, 0.9), float3(0), Glass);
			Material yellow = Material(float3(1, 0.8, 0), float3(0), Mirror);
			Material white = Material(float3(1, 1, 1), float3(0), Diffuse);
			Material light = Material(float3(1, 1, 1), float3(0), Light);
			Material mirror = Material(float3(1, 1, 1), float3(0), Mirror);

			quad = Quad(0, 1, light, mat4::Identity());						// 0: light source
			sphere = Sphere(1, float3(0), 0.5f, yellow);				// 1: bouncing ball
			sphere2 = Sphere(2, float3(0, 2.5f, -3.07f), 8, white);	// 2: rounded corners
			//cube = Cube(3, float3(0), float3(1.15f), purple, mat4::Identity());									// 3: cube
			
			//plane[0] = Plane(4, float3(1, 0, 0), 3, red);									// 4: left wall
			//plane[1] = Plane(5, float3(-1, 0, 0), 2.99f, blue);								// 5: right wall
			//plane[2] = Plane(6, float3(0, 1, 0), 1, white);									// 6: floor
			//plane[3] = Plane(7, float3(0, -1, 0), 2, white);									// 7: ceiling
			//plane[4] = Plane(8, float3(0, 0, 1), 3, green);									// 8: front wall
			//plane[5] = Plane(9, float3(0, 0, -1), 3.99f, white);								// 9: back wall
			
			shapes.push_back(&quad);
			shapes.push_back(&sphere);
			shapes.push_back(&sphere2);
			shapes.push_back(&cube);
			//shapes.push_back(&plane[0]);
			//shapes.push_back(&plane[1]);
			//shapes.push_back(&plane[2]);
			//shapes.push_back(&plane[3]);
			//shapes.push_back(&plane[4]);
			//shapes.push_back(&plane[5]);

			//for (int i = 0; i < shapes.size(); i++) {
			//	printf("index: %d, min: %f, %f, %f, max: %f, %f, %f\n",
			//		shapes[i]->objIdx,
			//		shapes[i]->GetAABB()[0].x,
			//		shapes[i]->GetAABB()[0].y,
			//		shapes[i]->GetAABB()[0].z,
			//		shapes[i]->GetAABB()[1].x,
			//		shapes[i]->GetAABB()[1].y,
			//		shapes[i]->GetAABB()[1].z
			//	);
			//}

			mesh = TriangleMesh(0, "assets/bunny.obj");

			//printf("vertices: %d, triangles: %d\n", 
			//	mesh.vertices.size(),
			//	mesh.triangles.size()
			//);

			for (int i = 0; i < mesh.triangles.size(); i++) {
				//printf("normal: %f, %f, %f\n", 
				//	mesh.triangles[i].GetNormal(float3(0)).x,
				//	mesh.triangles[i].GetNormal(float3(0)).y,
				//	mesh.triangles[i].GetNormal(float3(0)).z
				//);
				shapes.push_back(&mesh.triangles[i]);
				//printf("min: %f, %f, %f, max: %f, %f, %f\n", 
				//	mesh.triangles[i].GetAABB()[0].x,
				//	mesh.triangles[i].GetAABB()[0].y,
				//	mesh.triangles[i].GetAABB()[0].z,
				//	mesh.triangles[i].GetAABB()[1].x,
				//	mesh.triangles[i].GetAABB()[1].y,
				//	mesh.triangles[i].GetAABB()[1].z
				//);
				//printf("index: %d, min: %f, %f, %f, max: %f, %f, %f\n",
				//	shapes[4 + i]->objIdx,
				//	shapes[4 + i]->GetAABB()[0].x,
				//	shapes[4 + i]->GetAABB()[0].y,
				//	shapes[4 + i]->GetAABB()[0].z,
				//	shapes[4 + i]->GetAABB()[1].x,
				//	shapes[4 + i]->GetAABB()[1].y,
				//	shapes[4 + i]->GetAABB()[1].z
				//);
			}

			//for (int i = 0; i < shapes.size(); i++) {
			//	vector<float3> aabb = shapes[i]->GetAABB();
			//}

			SetTime(0);

		 	root = BuildBVH_SAH(0, shapes.size()-1, 2);

			for (int i = 0; i < shapes.size(); i++) {
				shapes[i]->objIdx = i;
			}
		}

		int BuildBVH_SAH(int l, int r, int n) {
			if (l > r) return 0;

			int idx = nodes.size();
			nodes.push_back(BVHNode());
			nodes[idx].left = nodes[idx].right = nodes[idx].n = nodes[idx].index = 0;

			nodes[idx].aabbMin = float3(INF);
			nodes[idx].aabbMax = float3(-INF);

			for (int i = l; i <= r; i++) {
				vector<float3> aabb = this->shapes[i]->GetAABB();
				nodes[idx].aabbMin = fminf(nodes[idx].aabbMin, aabb[0]);
				nodes[idx].aabbMax = fmaxf(nodes[idx].aabbMax, aabb[1]);
			}

			if ((r - l + 1) <= n) {
				nodes[idx].n = r - l + 1;
				nodes[idx].index = l;
				return idx;
			}

			float Cost = INF;
			int Axis = 0;
			int Split = (l + r) / 2;
			for (int axis = 0; axis < 3; axis++) {
				if (axis == 0) sort(&shapes[0] + l, &shapes[0] + r + 1, cmpx);
				if (axis == 1) sort(&shapes[0] + l, &shapes[0] + r + 1, cmpy);
				if (axis == 2) sort(&shapes[0] + l, &shapes[0] + r + 1, cmpz);

				vector<float3> leftMax(r - l + 1, float3(-INF));
				vector<float3> leftMin(r - l + 1, float3(INF));
				for (int i = l; i <= r; i++) {
					vector<float3> aabb = shapes[i]->GetAABB();
					int bias = (i == l) ? 0 : 1;

					leftMin[i - l] = fminf(leftMin[i - l], aabb[0]);
					leftMax[i - l] = fmaxf(leftMax[i - l], aabb[1]);
				}

				vector<float3> rightMax(r - l + 1, float3(-INF));
				vector<float3> rightMin(r - l + 1, float3(INF));
				for (int i = r; i >= l; i--) {
					vector<float3> aabb = shapes[i]->GetAABB();
					int bias = (i == r) ? 0 : 1;

					rightMin[i - l] = fminf(rightMin[i - l], aabb[0]);
					rightMax[i - l] = fmaxf(rightMax[i - l], aabb[1]);
				}

				float cost = INF;
				int split = l;
				for (int i = l; i <= r - 1; i++) {
					float lenx, leny, lenz;
					float3 leftaabbMin = leftMin[i - l];
					float3 leftaabbMax = leftMax[i - l];
					lenx = leftaabbMax.x - leftaabbMin.x;
					leny = leftaabbMax.y - leftaabbMin.y;
					lenz = leftaabbMax.z - leftaabbMin.z;
					float leftS = 2.0 * ((lenx * leny) + (lenx * lenz) + (leny * lenz));
					float leftCost = leftS * (i - l + 1);

					float3 rightaabbMin = rightMin[i + 1 - l];
					float3 rightaabbMax = rightMax[i + 1 - l];
					lenx = rightaabbMax.x - rightaabbMin.x;
					leny = rightaabbMax.y - rightaabbMin.y;
					lenz = rightaabbMax.z - rightaabbMin.z;
					float rightS = 2.0 * ((lenx * leny) + (lenx * lenz) + (leny * lenz));
					float rightCost = rightS * (r - i);

					float totalCost = leftCost + rightCost;
					if (totalCost < cost) {
						cost = totalCost;
						split = i;
					}
				}

				if (cost < Cost) {
					Cost = cost;
					Axis = axis;
					Split = split;
				}
			}

			if (Axis == 0) sort(&shapes[0] + l, &shapes[0] + r + 1, cmpx);
			if (Axis == 1) sort(&shapes[0] + l, &shapes[0] + r + 1, cmpy);
			if (Axis == 2) sort(&shapes[0] + l, &shapes[0] + r + 1, cmpz);

			int left = BuildBVH_SAH(l, Split, n);
			int right = BuildBVH_SAH(Split + 1, r, n);

			nodes[idx].left = left;
			nodes[idx].right = right;

			return idx;
		}
		
		void IntersectBVH(Ray& ray, int nodeIdx) const
		{
			BVHNode node = nodes[nodeIdx];
			if (!IntersectAABB(ray, node.aabbMin, node.aabbMax)) return;
			if (node.n > 0)
			{
				for (uint i = 0; i < node.n; i++)
					shapes[node.index + i]->Intersect(ray);
			}
			else
			{
				IntersectBVH(ray, node.left);
				IntersectBVH(ray, node.right);
			}
		}

		void SetTime(float t)
		{
			// default time for the scene is simply 0. Updating/ the time per frame 
			// enables animation. Updating it per ray can be used for motion blur.
			animTime = t;
			// light source animation: swing
			
			//mat4 M1base = mat4::Translate(float3(0, 14.5f, 2));
			//mat4 M1 = M1base * mat4::RotateZ(sinf(animTime * 0.6f) * 0.1f) * mat4::Translate(float3(0, -0.9, 0));
			//quad.T = M1, quad.invT = M1.FastInvertedTransformNoScale();

			//// cube animation: spin
			//mat4 M2base = mat4::RotateX(PI / 4) * mat4::RotateZ(PI / 4);
			//mat4 M2 = mat4::Translate(float3(10.0f, 0, 2)) * mat4::RotateY(animTime * 0.5f) * M2base;
			//cube.T = M2, cube.invT = M2.FastInvertedTransformNoScale();

			//// sphere animation: bounce
			//float tm = 1 - sqrf(fmodf(animTime, 2.0f) - 1);
			//sphere.center = float3(-10.0f, -0.5f + tm, 2);

			mat4 M1base = mat4::Translate(float3(0, 5.0f, 0));
			mat4 M1 = M1base * mat4::Translate(float3(0, -0.9, 0));
			
			quad.T = M1, quad.invT = M1.FastInvertedTransformNoScale();

			mat4 M2 = mat4::Translate(float3(2.4f, 0, 2));
			//cube.T = M2, cube.invT = M2.FastInvertedTransformNoScale();
	
			sphere.center = float3(-2.4f, 0.2, 2);
		
		}
		float3 GetLightPos() const
		{
			// light point position is the middle of the swinging quad
			return quad.GetCenter() - float3(0, 0.01f, 0);
		}
		float3 GetLightColor(int objIdx) const
		{
			if (objIdx == -1) return float3(0); // or perhaps we should just crash
			return shapes[objIdx]->material.color;
		}
		MatType GetObjMatType(int objIdx) const
		{
			if (objIdx == -1) return Basic; // or perhaps we should just crash
			return shapes[objIdx]->material.type;
		}
		void FindNearest(Ray& ray) const
		{
			IntersectBVH(ray, root);
		}
		bool IsOccluded(Ray& ray) const
		{
			float rayLength = ray.t;
			IntersectBVH(ray, root);
			return ray.t < rayLength;
			// technically this is wasteful: 
			// - we potentially search beyond rayLength
			// - we store objIdx and t when we just need a yes/no
			// - we don't 'early out' after the first occlusion
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
			// once we have triangle support, we should pass objIdx and the bary-
			// centric coordinates of the hit, instead of the intersection location.
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

		__declspec(align(64)) // start a new cacheline here
			float animTime = 0;
		Quad quad;
		Sphere sphere;
		Sphere sphere2;
		Cube cube;
		//Plane plane[6];
		TriangleMesh mesh;

		vector<Shape*> shapes;
		vector<BVHNode> nodes;
		int root;
	};
}