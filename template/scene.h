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
		virtual aabb GetAABB() const = 0;
		
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
		aabb GetAABB() const
		{
			return aabb(this->center - r, this->center + r);
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
		aabb GetAABB() const
		{
			float3 b0 = TransformPosition(b[0], T);
			float3 b1 = TransformPosition(b[1], T);
			return aabb(fminf(b0, b1), fmaxf(b0, b1));
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
			return float3(-T.cell[1], -T.cell[5], -T.cell[9]);
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
		aabb GetAABB() const
		{   
			return aabb(fminf(fminf(v0, v1), v2), fmaxf(fmaxf(v0, v1), v2));
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
		return s1->GetAABB().Center(0) < s2->GetAABB().Center(0);
	}
	inline bool cmpy(const Shape* s1, const Shape* s2) {
		return s1->GetAABB().Center(1) < s2->GetAABB().Center(1);
	}
	inline bool cmpz(const Shape* s1, const Shape* s2) {
		return s1->GetAABB().Center(2) < s2->GetAABB().Center(2);
	}

	struct BVHNode {
		int c1, c2, c3, c4;	//index of children
		int n, index;		//n children in leaf node, index is idex of the first shape                     
		aabb aabb;			//aabb
	};

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
		if (tmax >= tmin && tmin < ray.t && tmax > 0) return tmin; else return 1e30f;
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

			mat4 leftT = mat4::Translate(float3(-4, 3, 0)) * mat4::RotateZ(PI / 2);
			mat4 rightT = mat4::Translate(float3(4, 3, 0)) * mat4::RotateZ(-PI / 2);
			mat4 topT = mat4::Translate(float3(0, 7, 0));
			mat4 botT = mat4::Translate(float3(0, -1, 0)) * mat4::RotateZ(PI);
			mat4 frontT = mat4::Translate(float3(0, 3, -4)) * mat4::RotateX(PI / 2);
			mat4 backT = mat4::Translate(float3(0, 3, 4)) * mat4::RotateX(-PI / 2);;

			plane[0] = Plane(0, float3(0), 2, light, mat4::Identity());						// light source
			plane[1] = Plane(1, float3(0), 10, red, leftT);									// left wall
			plane[2] = Plane(2, float3(0), 10, blue, rightT);								// right wall
			plane[3] = Plane(3, float3(0), 10, white, botT);								// floor
			plane[4] = Plane(4, float3(0), 10, white, topT);								// ceiling
			plane[5] = Plane(5, float3(0), 10, green, frontT);								// front wall
			plane[6] = Plane(6, float3(0), 10, white, backT);								// back wall
			sphere = Sphere(6, float3(0), 0.5f, yellow);									// yellow mirror sphere
			cube = Cube(8, float3(0), float3(1.15f), purple, mat4::Identity());				// purple glass cube
			
			for (int i = 0; i < 7; i++) {
				shapes.push_back(&plane[i]);
			}

			shapes.push_back(&sphere);
			shapes.push_back(&cube);

			mesh = TriangleMesh(0, "assets/bunny.obj");

			for (int i = 0; i < mesh.triangles.size(); i++) {
				shapes.push_back(&mesh.triangles[i]);
			}

			SetTime(0);

		 	root = BuildQBVH(0, shapes.size()-1, 2);

			for (int i = 0; i < shapes.size(); i++) {
				shapes[i]->objIdx = i;
			}

			//for (int i = 0; i < nodes.size(); i++) {
			//	printf("node: %d, c3: %d, c4: %d\n", i, nodes[i].c3, nodes[i].c4);
			//}
		}

		int BuildBVH(int l, int r, int n) {
			//no nodes 
			if (l > r) return 0;

			//generate newest node
			int idx = nodes.size();
			nodes.push_back(BVHNode());
			nodes[idx].c1 = -1;
			nodes[idx].c2 = -1;
			nodes[idx].n = -1;
			nodes[idx].index = -1;

			nodes[idx].aabb = aabb(float3(INF), float3(-INF));

			//update node boundary
			for (int i = l; i <= r; i++) {
				nodes[idx].aabb = nodes[idx].aabb.Union(this->shapes[i]->GetAABB());
			}

			//if less than n shapes in this node, then this is a leaf node
			if ((r - l + 1) <= n) {
				nodes[idx].n = r - l + 1;
				nodes[idx].index = l;
				return idx;
			}

			int Axis = nodes[idx].aabb.LongestAxis();	//axis of the current split method
			int Split = (l + r) / 2;

			if (Axis == 0) sort(&shapes[0] + l, &shapes[0] + r + 1, cmpx);
			if (Axis == 1) sort(&shapes[0] + l, &shapes[0] + r + 1, cmpy);
			if (Axis == 2) sort(&shapes[0] + l, &shapes[0] + r + 1, cmpz);

			nodes[idx].c1 = BuildBVH(l, Split, n);
			nodes[idx].c2 = BuildBVH(Split + 1, r, n);

			return idx;
		}

		int BuildBVH_SAH(int l, int r, int n) {
			//no nodes 
			if (l > r) return 0;

			//generate newest node
			int idx = nodes.size();
			nodes.push_back(BVHNode());
			nodes[idx].c1 = -1;
			nodes[idx].c2 = -1;
			nodes[idx].n = -1;
			nodes[idx].index = -1;
			nodes[idx].aabb = aabb(float3(INF), float3(-INF));

			//update node boundary
			for (int i = l; i <= r; i++) {
				nodes[idx].aabb = nodes[idx].aabb.Union(this->shapes[i]->GetAABB());
			}

			//if less than n shapes in this node, then this is a leaf node
			if ((r - l + 1) <= n) {
				nodes[idx].n = r - l + 1;
				nodes[idx].index = l;
				return idx;
			}

			//Get the optimal split axis
			int Axis;
			//Get the least cost, skip the computation involving the current node
			float Cost = INF;
			//Get the optimal split position
			int Split;

			//For each axis, find the optimal position
			for (int axis = 0; axis < 3; axis++) {
				if (axis == 0) sort(&shapes[0] + l, &shapes[0] + r + 1, cmpx);
				if (axis == 1) sort(&shapes[0] + l, &shapes[0] + r + 1, cmpy);
				if (axis == 2) sort(&shapes[0] + l, &shapes[0] + r + 1, cmpz);

				//Get the optimal split position along this axis
				int split;
				//Get the least cost along this axis
				float cost = INF;

				//Compute the left node surface area when the split position is different
				vector<float> leftS;
				aabb aabbLeft;
				for (int i = l; i < r; i++) {
					aabbLeft = aabbLeft.Union(this->shapes[i]->GetAABB());
					leftS.push_back(aabbLeft.Area());
				}

				//Compute the right node surface area when the split position is different
				vector<float> rightS;
				aabb aabbRight;
				for (int i = r; i > l; i--) {
					aabbRight = aabbRight.Union(this->shapes[i]->GetAABB());
					rightS.push_back(aabbRight.Area());
				}
				
				//Compute the total cost when the split position is different
				float totalCost;
				for (int i = 0; i < r - l; i++) {
					int leftCount = i + 1;
					int rightCount = r - l - leftCount;

					float leftCost = leftCount * leftS[i];
					float rightCost = rightCount * rightS[r - l - i - 1];

					totalCost = leftCost + rightCost;
					if (totalCost < cost) {
						cost = totalCost;
						split = i + l;
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

			if (Axis == 0) sort(&shapes[0] + l, &shapes[0] + r + 1, cmpx);
			if (Axis == 1) sort(&shapes[0] + l, &shapes[0] + r + 1, cmpy);
			if (Axis == 2) sort(&shapes[0] + l, &shapes[0] + r + 1, cmpz);

			nodes[idx].c1 = BuildBVH_SAH(l, Split, n);
			nodes[idx].c2 = BuildBVH_SAH(Split + 1, r, n);

			return idx;
		}

		int BuildQBVH(int l, int r, int n) {
			//no nodes 
			if (l > r) return 0;

			//generate newest node
			int idx = nodes.size();
			nodes.push_back(BVHNode());
			nodes[idx].aabb = aabb(float3(INF), float3(-INF));

			//update node boundary
			for (int i = l; i <= r; i++) {
				nodes[idx].aabb = nodes[idx].aabb.Union(this->shapes[i]->GetAABB());
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
			float Cost = INF;

			int lLocal = l;
			int rLocal = r;

			for (int axis = 0; axis < 3; axis++) {
				if (axis == 0) sort(&shapes[0] + lLocal, &shapes[0] + rLocal + 1, cmpx);
				if (axis == 1) sort(&shapes[0] + lLocal, &shapes[0] + rLocal + 1, cmpy);
				if (axis == 2) sort(&shapes[0] + lLocal, &shapes[0] + rLocal + 1, cmpz);

				//Get the optimal split position along this axis
				int middle;
				//Get the least cost along this axis
				float cost = INF;

				//Compute the left node surface area when the split position is different
				vector<float> leftS;
				aabb aabbLeft;
				for (int i = lLocal; i < rLocal; i++) {
					aabbLeft = aabbLeft.Union(this->shapes[i]->GetAABB());
					leftS.push_back(aabbLeft.Area());
				}

				//Compute the right node surface area when the split position is different
				vector<float> rightS;
				aabb aabbRight;
				for (int i = rLocal; i > lLocal; i--) {
					aabbRight = aabbRight.Union(this->shapes[i]->GetAABB());
					rightS.push_back(aabbRight.Area());
				}

				//Compute the total cost when the split position is different
				float totalCost;
				for (int i = 0; i < rLocal - lLocal; i++) {
					int leftCount = i + 1;
					int rightCount = rLocal - lLocal - leftCount;

					float leftCost = leftCount * leftS[i];
					float rightCost = rightCount * rightS[rLocal - lLocal - i - 1];

					totalCost = leftCost + rightCost;
					if (totalCost < cost) {
						cost = totalCost;
						middle = i + lLocal;
					}
				}

				//If the cost when split along the current axis at the local optimal split
				//position is better, update the information
				if (cost < Cost) {
					Axis = axis;
					Cost = cost;
					Middle = middle;
				}
			}

			if (Axis == 0) sort(&shapes[0] + lLocal, &shapes[0] + rLocal + 1, cmpx);
			if (Axis == 1) sort(&shapes[0] + lLocal, &shapes[0] + rLocal + 1, cmpy);
			if (Axis == 2) sort(&shapes[0] + lLocal, &shapes[0] + rLocal + 1, cmpz);

			lLocal = l;
			rLocal = Middle;
			Cost = INF;

			for (int axis = 0; axis < 3; axis++) {
				if (axis == 0) sort(&shapes[0] + lLocal, &shapes[0] + rLocal + 1, cmpx);
				if (axis == 1) sort(&shapes[0] + lLocal, &shapes[0] + rLocal + 1, cmpy);
				if (axis == 2) sort(&shapes[0] + lLocal, &shapes[0] + rLocal + 1, cmpz);

				//Get the optimal split position along this axis
				int middle;
				//Get the least cost along this axis
				float cost = INF;

				//Compute the left node surface area when the split position is different
				vector<float> leftS;
				aabb aabbLeft;
				for (int i = lLocal; i < rLocal; i++) {
					aabbLeft = aabbLeft.Union(this->shapes[i]->GetAABB());
					leftS.push_back(aabbLeft.Area());
				}

				//Compute the right node surface area when the split position is different
				vector<float> rightS;
				aabb aabbRight;
				for (int i = rLocal; i > lLocal; i--) {
					aabbRight = aabbRight.Union(this->shapes[i]->GetAABB());
					rightS.push_back(aabbRight.Area());
				}

				//Compute the total cost when the split position is different
				float totalCost;
				for (int i = 0; i < rLocal - lLocal; i++) {
					int leftCount = i + 1;
					int rightCount = rLocal - lLocal - leftCount;

					float leftCost = leftCount * leftS[i];
					float rightCost = rightCount * rightS[rLocal - lLocal - i - 1];

					totalCost = leftCost + rightCost;
					if (totalCost < cost) {
						cost = totalCost;
						middle = i + lLocal;
					}
				}

				//If the cost when split along the current axis at the local optimal split
				//position is better, update the information
				if (cost < Cost) {
					Axis = axis;
					Cost = cost;
					MidLeft = middle;
				}
			}

			if (Axis == 0) sort(&shapes[0] + lLocal, &shapes[0] + rLocal + 1, cmpx);
			if (Axis == 1) sort(&shapes[0] + lLocal, &shapes[0] + rLocal + 1, cmpy);
			if (Axis == 2) sort(&shapes[0] + lLocal, &shapes[0] + rLocal + 1, cmpz);

			nodes[idx].c1 = BuildBVH_SAH(lLocal, MidLeft, n);
			nodes[idx].c2 = BuildBVH_SAH(MidLeft + 1, rLocal, n);

			lLocal = Middle + 1;
			rLocal = r;
			Cost = INF;

			if (rLocal > lLocal) return idx;

			for (int axis = 0; axis < 3; axis++) {
				if (axis == 0) sort(&shapes[0] + lLocal, &shapes[0] + rLocal + 1, cmpx);
				if (axis == 1) sort(&shapes[0] + lLocal, &shapes[0] + rLocal + 1, cmpy);
				if (axis == 2) sort(&shapes[0] + lLocal, &shapes[0] + rLocal + 1, cmpz);

				//Get the optimal split position along this axis
				int middle;
				//Get the least cost along this axis
				float cost = INF;

				//Compute the left node surface area when the split position is different
				vector<float> leftS;
				aabb aabbLeft;
				for (int i = lLocal; i < rLocal; i++) {
					aabbLeft = aabbLeft.Union(this->shapes[i]->GetAABB());
					leftS.push_back(aabbLeft.Area());
				}

				//Compute the right node surface area when the split position is different
				vector<float> rightS;
				aabb aabbRight;
				for (int i = rLocal; i > lLocal; i--) {
					aabbRight = aabbRight.Union(this->shapes[i]->GetAABB());
					rightS.push_back(aabbRight.Area());
				}

				//Compute the total cost when the split position is different
				float totalCost;
				for (int i = 0; i < rLocal - lLocal; i++) {
					int leftCount = i + 1;
					int rightCount = rLocal - lLocal - leftCount;

					float leftCost = leftCount * leftS[i];
					float rightCost = rightCount * rightS[rLocal - lLocal - i - 1];

					totalCost = leftCost + rightCost;
					if (totalCost < cost) {
						cost = totalCost;
						middle = i + lLocal;
					}
				}

				//If the cost when split along the current axis at the local optimal split
				//position is better, update the information
				if (cost < Cost) {
					Axis = axis;
					Cost = cost;
					MidRight = middle;
				}
			}

			if (Axis == 0) sort(&shapes[0] + lLocal, &shapes[0] + rLocal + 1, cmpx);
			if (Axis == 1) sort(&shapes[0] + lLocal, &shapes[0] + rLocal + 1, cmpy);
			if (Axis == 2) sort(&shapes[0] + lLocal, &shapes[0] + rLocal + 1, cmpz);

			nodes[idx].c3 = BuildBVH_SAH(lLocal, MidRight, n);
			nodes[idx].c4 = BuildBVH_SAH(MidRight + 1, rLocal, n);

			return idx;
		}

		void IntersectBVH(Ray& ray, int nodeIdx) const
		{
			BVHNode node = nodes[nodeIdx];
			if (IntersectAABB(ray, node.aabb) == 1e30f) return;
			if (node.n > 0)
			{
				for (uint i = 0; i < node.n; i++)
					shapes[node.index + i]->Intersect(ray);
			}
			else
			{
				IntersectBVH(ray, node.c1);
				IntersectBVH(ray, node.c2);
				//IntersectBVH(ray, node.c3);
				//IntersectBVH(ray, node.c4);
			}
		}

		//void IsoccludedBVH(Ray& ray, int nodeIdx) const
		//{
		//	BVHNode node = nodes[nodeIdx];
		//	if (!IntersectAABB(ray, node.aabbMin, node.aabbMax)) return;
		//	if (node.n > 0)
		//	{
		//		for (uint i = 0; i < node.n; i++)
		//			shapes[node.index + i]->Intersect(ray);
		//	}
		//	else
		//	{
		//		IntersectBVH(ray, node.left);
		//		IntersectBVH(ray, node.right);
		//	}
		//}

		void SetTime(float t)
		{
			// default time for the scene is simply 0. Updating/ the time per frame 
			// enables animation. Updating it per ray can be used for motion blur.
			animTime = t;
			// light source animation: swing
			
			//mat4 M1base = mat4::Translate(float3(0, 14.5f, 2));
			//mat4 M1 = M1base * mat4::RotateZ(sinf(animTime * 0.6f) * 0.1f) * mat4::Translate(float3(0, -0.9, 0));
			//plane[0].T = M1, plane[0].invT = M1.FastInvertedTransformNoScale();

			//// cube animation: spin
			//mat4 M2base = mat4::RotateX(PI / 4) * mat4::RotateZ(PI / 4);
			//mat4 M2 = mat4::Translate(float3(10.0f, 0, 2)) * mat4::RotateY(animTime * 0.5f) * M2base;
			//cube.T = M2, cube.invT = M2.FastInvertedTransformNoScale();

			//// sphere animation: bounce
			//float tm = 1 - sqrf(fmodf(animTime, 2.0f) - 1);
			//sphere.center = float3(-10.0f, -0.5f + tm, 2);

			mat4 M1base = mat4::Translate(float3(0, 4.0f, 0));
			mat4 M1 = M1base * mat4::Translate(float3(0, -0.9, 0));
			
			plane[0].T = M1, plane[0].invT = M1.FastInvertedTransformNoScale();

			mat4 M2 = mat4::Translate(float3(2.4f, 0, 2));
			cube.T = M2, cube.invT = M2.FastInvertedTransformNoScale();
	
			sphere.center = float3(-2.4f, 0.2, 2);
		
		}
		float3 GetLightPos() const
		{
			// light point position is the middle of the swinging quad
			return plane[0].GetCenter() - float3(0, 0.01f, 0);
		}
		float3 GetLightColor(int objIdx) const
		{
			if (objIdx == -1) return float3(0);
			return shapes[objIdx]->material.color;
		}
		MatType GetObjMatType(int objIdx) const
		{
			if (objIdx == -1) return Basic;
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
		Plane plane[7];
		Sphere sphere;
		Cube cube;
		TriangleMesh mesh;

		vector<Shape*> shapes;
		vector<BVHNode> nodes;
		int root;
	};
}