#include "precomp.h"

// -----------------------------------------------------------
// Initialize the renderer
// -----------------------------------------------------------
void Renderer::Init()
{
	// create fp32 rgb pixel buffer to render to
	accumulator = (float4*)MALLOC64(SCRWIDTH * SCRHEIGHT * 16);
	memset(accumulator, 0, SCRWIDTH * SCRHEIGHT * 16);
	srand(time(0));
	//This is the counter for real time sampling for path tracing
	sample = 1;

	pathTrace = true;

	Kernel::InitCL();

	Kernel kernel = Kernel("BVH.cl", "IntersectBVHcl");
	
}

// -----------------------------------------------------------
// Evaluate light transport
// -----------------------------------------------------------
float3 Renderer::Trace(Ray& ray, int iter = 0)
{
	scene.FindNearest(ray);
	if (ray.objIdx == -1 || iter > 5) return 0; // or a fancy sky color
	
	float3 I = ray.O + ray.t * ray.D;
	float3 N = scene.GetNormal(ray.objIdx, I, ray.D);
	float cos1 = dot(N, -ray.D);

	float3 shadowRayDir = normalize(scene.GetLightPos() - I);
	float distance = length(scene.GetLightPos() - I);
	Ray shadowray = Ray(I + shadowRayDir * 0.001, shadowRayDir, distance, ray.media);
	float3 albedo = scene.GetAlbedo(ray.objIdx, I);

	MatType mat = scene.GetObjMatType(ray.objIdx);
	float3 color = scene.GetLightColor(ray.objIdx);
	if (scene.IsOccluded(shadowray)) {
		if (mat == Mirror) {
			float3 reflectRayDir = normalize(reflect(ray.D, N));
			Ray mirrorRay = Ray(I + reflectRayDir * 0.001, reflectRayDir);
			return color * Trace(mirrorRay, iter + 1);
		}
		if (mat == Glass) {
			
		}
		else return 0.0f;
	}

	if (mat == Diffuse) {
		return color / (0.5 * distance);
	}
	if (mat == Mirror) {
		float3 reflectRayDir = normalize(reflect(ray.D, N));
		Ray mirrorRay = Ray(I + reflectRayDir * 0.001, reflectRayDir, 10000, Air);
		return color * Trace(mirrorRay, iter + 1);
	}	
	if (mat == Glass) 
	{	
		float3 reflectRayDir = normalize(reflect(ray.D, N));
		Ray reflectRay = Ray(I + reflectRayDir * 0.001, reflectRayDir, 10000, ray.media);
		
		float k;
		float cos2;
		if (ray.media == Air)
		{
			cos2 = sqrt(1 - pow(refractive[AirToGlass] * sqrt(1 - pow(cos1, 2)), 2));
			k = 1 - pow(refractive[AirToGlass], 2) * (1 - pow(cos1, 2));
			
			float3 refractRayDir = normalize(-cos2 * N + refractive[AirToGlass] * (ray.D + cos1 * N));
			Ray refractRay = Ray(I + refractRayDir * 0.001, refractRayDir, 10000, Glass);
			
			float Fr = 0.5 * ((pow((cos1 - refractive[GlassToAir] * cos2) / (cos1 + refractive[GlassToAir] * cos2), 2)) + (pow((cos2 - refractive[GlassToAir] * cos1) / (cos2 + refractive[GlassToAir] * cos1), 2)));
			float Ft = 1 - Fr;

			return Absorb(Trace(refractRay, iter + 1) * Ft, ray.t, color * 0.1) + Trace(reflectRay, iter + 1) * Fr;
		}
		if (ray.media == Glass) {
			k = 1 - pow(refractive[GlassToAir], 2) * (1 - pow(cos1, 2));
			if (k < 0) return Trace(reflectRay, iter + 1);
			else {
				cos2 = sqrt(1 - pow(refractive[GlassToAir] * sqrt(1 - pow(cos1, 2)), 2));

				float3 refractRayDir = normalize(-cos2 * N + refractive[GlassToAir] * (ray.D + cos1 * N));
				Ray refractRay = Ray(I + refractRayDir * 0.001, refractRayDir, 10000, Air);

				float Fr = 0.5 * ((pow((refractive[GlassToAir] * cos1 - cos2) / (refractive[GlassToAir] * cos1 + cos2), 2)) + (pow((refractive[GlassToAir] * cos2 - cos1) / (refractive[GlassToAir] * cos2 + cos1), 2)));
				float Ft = 1 - Fr;

				return Trace(refractRay, iter + 1) * Ft + Trace(reflectRay, iter + 1) * Fr;
			}
		}
	}
	return color;
}

float3 Renderer::PathTrace(Ray& ray, float iter = 0) {
	scene.FindNearest(ray);
	MatType mat = scene.GetObjMatType(ray.objIdx);
	float3 color = scene.GetLightColor(ray.objIdx);

	if (mat == Light) return color;
	if (ray.objIdx == -1 || iter > 8) return 0; // or a fancy sky color

	//In order to reduce too many recursion, we use this method to randomly decide whether one ray
	//should stop bouncing between objects. Here the probability of keeping the ray is P = 0.8
	double r = rand() * (1.0 / RAND_MAX);
	float P = 0.8;
	if (r > P) return float3(0);

	float3 I = ray.O + ray.t * ray.D;
	float3 N = scene.GetNormal(ray.objIdx, I, ray.D);
	float cos1 = dot(N, -ray.D);
	float3 color_accum = float3(0);

	//Choose the random ray that bounce between objects to implement the environment lighting
	float3 randomRayDir = normalize(random_in_hemisphere(N));
	float bounceCos = -dot(ray.D, randomRayDir);
	Ray randomRay = Ray(I + randomRayDir * 0.001, randomRayDir, 10000, ray.media);
	
	//float3 albedo = scene.GetAlbedo(ray.objIdx, I);

	//Basic material is for testing, will still be referred to in future development
	if (mat == Basic) return cos1 * 1.25 * color;

	//To make the expectation of the color even, we need to divide the result by P. And to avoid
	//division, we multiply the color by reciprocal of P
	if (mat == Diffuse) {
		return cos1 * 1.25 * color * PathTrace(randomRay, iter + 1);
	}

	if (mat == Mirror) {
		float3 reflectRayDir = normalize(reflect(ray.D, N));
		Ray mirrorRay = Ray(I + reflectRayDir * 0.001, reflectRayDir);
		return cos1 * 1.25 * color * PathTrace(mirrorRay, iter+1);
	}
	if (mat == Glass)
	{
		float3 reflectRayDir = normalize(reflect(ray.D, N));
		Ray reflectRay = Ray(I + reflectRayDir * 0.001, reflectRayDir, 10000, ray.media);

		float k;
		float cos2;
		if (ray.media == Air)
		{
			cos2 = sqrt(1 - pow(refractive[AirToGlass] * sqrt(1 - pow(cos1, 2)), 2));
			k = 1 - pow(refractive[AirToGlass], 2) * (1 - pow(cos1, 2));

			float3 refractRayDir = normalize(-cos2 * N + refractive[AirToGlass] * (ray.D + cos1 * N));
			Ray refractRay = Ray(I + refractRayDir * 0.001, refractRayDir, 10000, Glass);

			float Fr = 0.5 * ((pow((cos1 - refractive[GlassToAir] * cos2) / (cos1 + refractive[GlassToAir] * cos2), 2)) + (pow((cos2 - refractive[GlassToAir] * cos1) / (cos2 + refractive[GlassToAir] * cos1), 2)));
			float Ft = 1 - Fr;

			return cos1 * 1.25 * (Absorb(PathTrace(refractRay, iter + 1) * Ft, ray.t, color * 0.1) + PathTrace(reflectRay, iter + 1) * Fr);
		}
		if (ray.media == Glass) {
			k = 1 - pow(refractive[GlassToAir], 2) * (1 - pow(cos1, 2));
			if (k < 0) return 1.25 * cos1 * PathTrace(reflectRay, iter + 1);
			else {
				cos2 = sqrt(1 - pow(refractive[GlassToAir] * sqrt(1 - pow(cos1, 2)), 2));

				float3 refractRayDir = normalize(-cos2 * N + refractive[GlassToAir] * (ray.D + cos1 * N));
				Ray refractRay = Ray(I + refractRayDir * 0.001, refractRayDir, 10000, Air);

				float Fr = 0.5 * ((pow((refractive[GlassToAir] * cos1 - cos2) / (refractive[GlassToAir] * cos1 + cos2), 2)) + (pow((refractive[GlassToAir] * cos2 - cos1) / (refractive[GlassToAir] * cos2 + cos1), 2)));
				float Ft = 1 - Fr;

				return 1.25 * cos1 * (PathTrace(refractRay, iter + 1) * Ft + PathTrace(reflectRay, iter + 1) * Fr);
			}
		}
	}
}

float3 Renderer::PathTraceNew(Ray& ray, float iter = 0) {
	scene.FindNearest(ray);
	MatType mat = scene.GetObjMatType(ray.objIdx);
	float3 color = scene.GetLightColor(ray.objIdx);

	if (mat == Light) return color;
	if (ray.objIdx == -1 || iter > maxPathLength) return 0; // or a fancy sky color

	if (ray.media == Glass) {
		Absorb(color, ray.t, scene.GetLightColor(ray.objIdx));
	}

	//In order to reduce too many recursion, we use this method to randomly decide whether one ray
	//should stop bouncing between objects. Here the probability of keeping the ray is P = 0.8
	//double r = rand() * (1.0 / RAND_MAX);
	//float P = 0.8;
	//if (r > P) return float3(0);

	float3 I = ray.O + ray.t * ray.D;
	float3 N = scene.GetNormal(ray.objIdx, I, ray.D);
	float cos1 = dot(N, -ray.D);
	float3 color_accum = float3(0);

	if (mat == Diffuse) {

		//Choose the random ray that bounce between objects to implement the environment lighting
		float3 randomRayDir = normalize(random_in_hemisphere(N));
		float bounceCos = -dot(ray.D, randomRayDir);
		Ray rayToHemisphere = Ray(I + randomRayDir * 0.001, randomRayDir, 10000, ray.media);
		scene.FindNearest(rayToHemisphere);

		if (scene.GetObjMatType(rayToHemisphere.objIdx) == Light) {
			float3 BRDF = color * INVPI;
			float cos_i = dot(randomRayDir, N);
			// the render equation
			return 2.0f * PI * BRDF * scene.GetLightColor(rayToHemisphere.objIdx) * cos_i;
		}
		return float3(0);
	}

	if (mat == Mirror) {
		float3 reflectRayDir = normalize(reflect(ray.D, N));
		Ray mirrorRay = Ray(I + reflectRayDir * 0.001, reflectRayDir);
		return PathTrace(mirrorRay, iter++);
	}

	if (mat == Glass) {

		double r = rand() * (1.0 / RAND_MAX);
		float Fr = 0.5f;

		if (ray.media == Air) {

			float cos2 = sqrt(1 - pow(refractive[AirToGlass] * sqrt(1 - pow(cos1, 2)), 2));
			Fr = 0.5 * ((pow((cos1 - refractive[GlassToAir] * cos2) / (cos1 + refractive[GlassToAir] * cos2), 2)) + (pow((cos2 - refractive[GlassToAir] * cos1) / (cos2 + refractive[GlassToAir] * cos1), 2)));
		}
		if (ray.media == Glass) {

			float cos2 = sqrt(1 - pow(refractive[GlassToAir] * sqrt(1 - pow(cos1, 2)), 2));
			Fr = 0.5 * ((pow((refractive[GlassToAir] * cos1 - cos2) / (refractive[GlassToAir] * cos1 + cos2), 2)) + (pow((refractive[GlassToAir] * cos2 - cos1) / (refractive[GlassToAir] * cos2 + cos1), 2)));
		}

		if (r < Fr) {

			//reflect
			float3 reflectRayDir = normalize(reflect(ray.D, N));
			Ray mirrorRay = Ray(I + reflectRayDir * 0.001, reflectRayDir);
			return PathTrace(mirrorRay, iter++);
		}
		else
		{
			//transmit
			if (ray.media == Air)
			{
				float cos2 = sqrt(1 - pow(refractive[AirToGlass] * sqrt(1 - pow(cos1, 2)), 2));
				float k = 1 - pow(refractive[AirToGlass], 2) * (1 - pow(cos1, 2));

				float3 refractRayDir = normalize(-cos2 * N + refractive[AirToGlass] * (ray.D + cos1 * N));
				Ray refractRay = Ray(I + refractRayDir * 0.001, refractRayDir, Glass);

				return PathTrace(refractRay, iter++);
			}
			if (ray.media == Glass) {

				float cos2 = sqrt(1 - pow(refractive[GlassToAir] * sqrt(1 - pow(cos1, 2)), 2));

				float3 refractRayDir = normalize(-cos2 * N + refractive[GlassToAir] * (ray.D + cos1 * N));
				Ray refractRay = Ray(I + refractRayDir * 0.001, refractRayDir, 10000, Air);

				return PathTrace(refractRay, iter++);
			}
		}
	}
}

float3 Renderer::Absorb(float3 color, float distance, float3 absorption)
{
	float3 output = color;
	output.x *= exp(absorption.x * distance);
	output.y *= exp(absorption.y * distance);
	output.z *= exp(absorption.z * distance);
	return output;
}

// -----------------------------------------------------------
// Main application tick function - Executed once per frame
// -----------------------------------------------------------
void Renderer::Tick(float deltaTime)
{
	// animation
	static float animTime = 0;
	scene.SetTime(animTime += deltaTime * 0.002f);
	// pixel loop
	Timer t;

	//1. Whitted-style ray-tracing, uncomment this and comment 2. 3. to render this way
	// lines are executed as OpenMP parallel tasks (disabled in DEBUG)
#	pragma omp parallel for schedule(dynamic)
	for (int y = 0; y < SCRHEIGHT; y++)
	{
		// trace a primary ray for each pixel on the line
		for (int x = 0; x < SCRWIDTH; x++)
		{
			if (pathTrace) {
				float3 color = PathTraceNew(camera.GetPrimaryRay(x, y));

				accumulator[x + y * SCRWIDTH] *= (sample - 1) / sample;
				accumulator[x + y * SCRWIDTH] += float4(color * (BRIGHTNESS / sample), 0);
			}
			else {
				float3 color = float3(0);
				color = Trace(camera.GetPrimaryRay(x, y));

				////anti-aliasing
				//color = color + Trace(camera.GetPrimaryRay(x + 0.25, y + 0.1));
				//color = color + Trace(camera.GetPrimaryRay(x - 0.25, y - 0.1));
				//color = color + Trace(camera.GetPrimaryRay(x + 0.1, y - 0.25));
				//color = color + Trace(camera.GetPrimaryRay(x - 0.1, y + 0.25));
				//color = 0.25 * color;

				accumulator[x + y * SCRWIDTH] =
					float4(color, 0);
			}
		}

		// translate accumulator contents to rgb32 pixels
		for (int dest = y * SCRWIDTH, x = 0; x < SCRWIDTH; x++)
			screen->pixels[dest + x] =
			RGBF32_to_RGB8(&accumulator[x + y * SCRWIDTH]);
	}

	if (pathTrace) sample++;

////2. Fixed sampling for path tracing, uncomment this and comment 1. 3. to render this way
//	int i, fsample = 4;
//
//	// lines are executed as OpenMP parallel tasks (disabled in DEBUG)
//#	pragma omp parallel for schedule(dynamic)
//	for (int y = 0; y < SCRHEIGHT; y++)
//	{
//		// trace a primary ray for each pixel on the line
//		for (int x = 0; x < SCRWIDTH; x++)
//		{
//			float3 color = float3(0);
//			for (i = 0; i < fsample; i++) {
//				color += PathTrace(camera.GetPrimaryRay(x, y));
//			}
//
//			color *= BRIGHTNESS / (float)fsample;
//
//			accumulator[x + y * SCRWIDTH] =
//				float4(color, 0);
//		}
//			
//		// translate accumulator contents to rgb32 pixels
//		for (int dest = y * SCRWIDTH, x = 0; x < SCRWIDTH; x++)
//			screen->pixels[dest + x] =
//			RGBF32_to_RGB8(&accumulator[x + y * SCRWIDTH]);
//	}

////3. Real-time sampling for path tracing, uncomment this and comment 1. 2. to render this way
////(in game control is hardly usable here).
//	printf("sample: %f\n", sample);
//	// lines are executed as OpenMP parallel tasks (disabled in DEBUG)
//#	pragma omp parallel for schedule(dynamic)
//	for (int y = 0; y < SCRHEIGHT; y++)
//	{
//		// trace a primary ray for each pixel on the line
//		for (int x = 0; x < SCRWIDTH; x++)
//		{
//			float3 color = PathTrace(camera.GetPrimaryRay(x, y));
//
//			accumulator[x + y * SCRWIDTH] *= (sample - 1) / sample;
//			accumulator[x + y * SCRWIDTH] += float4(color * (BRIGHTNESS / sample), 0);
//		}
//
//		// translate accumulator contents to rgb32 pixels
//		for (int dest = y * SCRWIDTH, x = 0; x < SCRWIDTH; x++)
//			screen->pixels[dest + x] =
//			RGBF32_to_RGB8(&accumulator[x + y * SCRWIDTH]);
//	}
//	sample++;
	
	// in game control
	//Move left
	if (GetKeyState('A') < 0) camera.Translate(float3(0.1, 0, 0)); 
	//Move right
	if (GetKeyState('D') < 0) camera.Translate(float3(-0.1, 0, 0));
	//Move backward
	if (GetKeyState('S') < 0) camera.Translate(float3(0, 0, 0.1));
	//Move forward
	if (GetKeyState('W') < 0) camera.Translate(float3(0, 0, -0.1));
	//Move up
	if (GetKeyState('Q') < 0) camera.Translate(float3(0, -0.1, 0));
	//Move down
	if (GetKeyState('E') < 0) camera.Translate(float3(0, 0.1, 0));
	//Left key to rotate left
	if (GetKeyState(37) < 0) camera.Rotate(0, 0.05);
	//Up key to rotate up
	if (GetKeyState(38) < 0) camera.Rotate(0.05, 0);
	//Right key to rotate right
	if (GetKeyState(39) < 0) camera.Rotate(0, -0.05);
	//Down key to rotate down
	if (GetKeyState(40) < 0) camera.Rotate(-0.05, 0);
	
	// performance report - running average - ms, MRays/s
	static float avg = 10, alpha = 1;
	avg = (1 - alpha) * avg + alpha * t.elapsed() * 1000;
	if (alpha > 0.05f) alpha *= 0.5f;
	float fps = 1000 / avg, rps = (SCRWIDTH * SCRHEIGHT) * fps;

	printf("%5.2fms (%.1fps) - %.1fMrays/s\n", avg, fps, rps / 1000000);
}