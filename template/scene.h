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

	const double BRIGHTNESS = 2.0f * 3.1415926f;

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
		Triangle(int objIdx, int v0, int v1, int v2, vector<float3>& vertices, Material material, mat4 transform = mat4::Identity()) :
			v0(v0), v1(v1), v2(v2), vertices(vertices)
		{
			this->objIdx = objIdx;
			this->material = material;
			this->T = transform, this->invT = transform.FastInvertedTransformNoScale();
			float3 p0 = vertices[v0];
			float3 p1 =	vertices[v1];
			float3 p2 = vertices[v2];
			this->center = (p0 + p1 + p2) / 3;
			N = normalize(cross(p1-p0, p2-p0));
		}
		void Intersect(Ray& ray) const
		{
			float3 p0 = vertices[v0];
			float3 p1 = vertices[v1];
			float3 p2 = vertices[v2];
			
			float3 O = TransformPosition(ray.O, invT);
			float3 D = TransformVector(ray.D, invT);

			float DP = dot(N, v0);

			// parralel check
			float parallel = dot(N, D);
			if (parallel == 0) return;
			float t = -(dot(N, O) + DP) / dot(N, D);
			if( t < ray.t && t > 0 ) {	
				float3 I = O + D * ray.t;

				// edge 0
				float3 edge0 = v1 - v0;
				float3 vp0 = I - v0;
				float3 C = cross(edge0, vp0);
				if (dot(N,C) < 0) return;

				// edge 1
				float3 edge1 = v2 - v1;
				float3 vp1 = I - v1;
				C = cross(edge1, vp1);
				if (dot(N, C) < 0)  return;

				// edge 2
				float3 edge2 = v0 - v2;
				float3 vp2 = I - v2;
				C = cross(edge2,vp2);
				if (dot(N,C) < 0) return;

				ray.t = t;
				ray.objIdx = objIdx;
			
			}
		}
		float3 GetNormal(const float3 I) const
		{
			return N;
		}
		float3 GetCenter() const
		{
			return center;
		}
		vector<float3> GetAABB() const
		{
			vector<float3> aabb;
			float3 p0 = TransformPosition(vertices[v0], invT);
			float3 p1 = TransformPosition(vertices[v1], invT);
			float3 p2 = TransformPosition(vertices[v2], invT);
			aabb.push_back(fminf(fminf(p0, p1), p2));
			aabb.push_back(fmaxf(fmaxf(p0, p1), p2));
			return aabb;
		}
		vector<float3>& vertices;
		int v0, v1, v2;
		float3 N;
	};

	void read_from_obj_file(
		const string& file_name,
		vector<float3>& vertices,
		vector<Triangle>& triangles) {
		// Open the file
		ifstream file(file_name);
		if (!file.is_open()) {
			cerr << "Error: Unable to open file " << file_name << endl;
			return;
		}

		Material mat_tri = Material(float3(1, 1, 1), float3(0), Diffuse);

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
				// Parse triangle vertices
				int v0, v1, v2;
				ss >> v0 >> v1 >> v2;

				// OBJ indices are 1-based, so we need to subtract 1 to get 0-based indices
				v0--;
				v1--;
				v2--;
				triangles.push_back(Triangle(-1, v0, v1, v2, vertices, mat_tri));
			}
		}
	}

	struct BVHNode
	{
	public:
		float3 aabbMin, aabbMax;
		BVHNode* left;
		BVHNode* right;
		bool isLeaf;
		int root, objIdx;
	};

	BVHNode* buildBVH() {
		BVHNode* node = new BVHNode();
		return node;
	}
	
	bool IntersectAABB(const Ray & ray, const float3 bmin, const float3 bmax)
	{
		float tx1 = (bmin.x - ray.O.x) / ray.D.x, tx2 = (bmax.x - ray.O.x) / ray.D.x;
		float tmin = min(tx1, tx2), tmax = max(tx1, tx2);
		float ty1 = (bmin.y - ray.O.y) / ray.D.y, ty2 = (bmax.y - ray.O.y) / ray.D.y;
		tmin = max(tmin, min(ty1, ty2)), tmax = min(tmax, max(ty1, ty2));
		float tz1 = (bmin.z - ray.O.z) / ray.D.z, tz2 = (bmax.z - ray.O.z) / ray.D.z;
		tmin = max(tmin, min(tz1, tz2)), tmax = min(tmax, max(tz1, tz2));
		return tmax >= tmin && tmin < ray.t&& tmax > 0;
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

			quad = Quad(0, 2, light, mat4::Identity());						// 0: light source
			sphere = Sphere(1, float3(0), 0.5f, yellow);				// 1: bouncing ball
			sphere2 = Sphere(2, float3(0, 2.5f, -3.07f), 8, white);	// 2: rounded corners
			cube = Cube(3, float3(0), float3(1.15f), purple, mat4::Identity());									// 3: cube
			
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

			read_from_obj_file("assets/bunny.obj", vertices, triangles);

			for (int i = 0; i < triangles.size(); i++) {
				triangles[i].objIdx = 4 + i;
				shapes.push_back(&triangles[i]);
			}
			
			SetTime(0);
			// Note: once we have triangle support we should get rid of the class
			// hierarchy: virtuals reduce performance somewhat.
		}
		void SetTime(float t)
		{
			// default time for the scene is simply 0. Updating/ the time per frame 
			// enables animation. Updating it per ray can be used for motion blur.
			animTime = t;
			// light source animation: swing
			
			mat4 M1base = mat4::Translate(float3(0, 2.6f, 2));
			mat4 M1 = M1base * mat4::RotateZ(sinf(animTime * 0.6f) * 0.1f) * mat4::Translate(float3(0, -0.9, 0));
			quad.T = M1, quad.invT = M1.FastInvertedTransformNoScale();

			// cube animation: spin
			mat4 M2base = mat4::RotateX(PI / 4) * mat4::RotateZ(PI / 4);
			mat4 M2 = mat4::Translate(float3(1.4f, 0, 2)) * mat4::RotateY(animTime * 0.5f) * M2base;
			cube.T = M2, cube.invT = M2.FastInvertedTransformNoScale();

			// sphere animation: bounce
			float tm = 1 - sqrf(fmodf(animTime, 2.0f) - 1);
			sphere.center = float3(-1.4f, -0.5f + tm, 2);

			//mat4 M1base = mat4::Translate(float3(0, 2.6f, 2));
			//mat4 M1 = M1base * mat4::Translate(float3(0, -0.9, 0));
			//
			//quad.T = M1, quad.invT = M1.FastInvertedTransformNoScale();

			//mat4 M2 = mat4::Translate(float3(1.4f, 0, 2));
			//cube.T = M2, cube.invT = M2.FastInvertedTransformNoScale();

			////mat4 M3 = mat4::RotateY(animTime * 0.5f);
			////triangle.M = M3, triangle.invM = M3.FastInvertedTransformNoScale();
			//
			//sphere.center = float3(-1.4f, 0.2, 2);
		
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
			quad.Intersect(ray);
			sphere.Intersect(ray);
			sphere2.Intersect(ray);
			cube.Intersect(ray);
		}
		bool IsOccluded(Ray& ray) const
		{
			float rayLength = ray.t;
			// skip planes: it is not possible for the walls to occlude anything
			quad.Intersect(ray);
			sphere.Intersect(ray);
			sphere2.Intersect(ray);
			cube.Intersect(ray);
			return ray.t < rayLength;
			// technically this is wasteful: 
			// - we potentially search beyond rayLength
			// - we store objIdx and t when we just need a yes/no
			// - we don't 'early out' after the first occlusion
		}
		float3 GetNormal(int objIdx, float3 I, float3 wo) const
		{
			// we get the normal after finding the nearest intersection:
			// this way we prevent calculating it multiple times.
			if (objIdx == -1) return float3(0); // or perhaps we should just crash
			float3 N;
			N = shapes[objIdx]->GetNormal(I);
			if (dot(N, wo) > 0) N = -N; // hit backside / inside
			return N;
		}
		float3 GetAlbedo(int objIdx, float3 I) const
		{
			if (objIdx == -1) return float3(0); // or perhaps we should just crash
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
		vector<Triangle> triangles;
		vector<float3> vertices;
		vector<Shape*> shapes;
		vector<BVHNode> nodes;
	};
}