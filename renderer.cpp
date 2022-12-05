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
	float3 shadowRayDir = normalize(scene.GetLightPos() - I);
	float distance = length(scene.GetLightPos() - I);
	Ray shadowray = Ray(I + shadowRayDir * 0.001, shadowRayDir, distance, ray.media);
	float3 albedo = scene.GetAlbedo(ray.objIdx, I);
	float cos1 = dot(N, -ray.D);

	MatType mat = scene.GetObjMat(ray.objIdx);
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
		return color / distance;
	}
	if (mat == Mirror) {
		float3 reflectRayDir = normalize(reflect(ray.D, N));
		Ray mirrorRay = Ray(I + reflectRayDir * 0.001, reflectRayDir, 10000, Air);
		return color * Trace(mirrorRay, iter + 1);
	}	
	if (mat == Glass) 
	{	
		float k;
		if (ray.media == Air) k = 1 - pow(refractive[AirToGlass], 2) * (1 - pow(cos1, 2));
		if (ray.media == Glass) k = 1 - pow(refractive[GlassToAir], 2) * (1 - pow(cos1, 2));
		
		if (k < 0) {
			float3 reflDirection = normalize(reflect(ray.D, N));

			Ray reflectRay = Ray(I + reflDirection * 0.001, reflDirection, 10000, ray.media);
			return Trace(reflectRay, iter + 1);
		}

		else {
			float cos2 = sqrt(1 - pow(refractive[AirToGlass] * sqrt(1 - pow(cos1, 2)), 2));

			if (ray.media == Glass) {
				float3 refractRayDir = normalize(-cos2 * N + refractive[GlassToAir] * (ray.D + cos1 * N));
				Ray refractRay = Ray(I + refractRayDir * 0.001, refractRayDir, 10000, Air);

				float3 reflectRayDir = normalize(reflect(ray.D, N));
				Ray reflectRay = Ray(I + reflectRayDir * 0.001, reflectRayDir, 10000, Glass);

				float cos2 = sqrt(1 - pow(refractive[GlassToAir] * sqrt(1 - pow(cos1, 2)), 2));

				float Fr = 0.5 * ((pow((refractive[GlassToAir] * cos1 - cos2) / (refractive[GlassToAir] * cos1 + cos2), 2)) + (pow((refractive[GlassToAir] * cos2 - cos1) / (refractive[GlassToAir] * cos2 + cos1), 2)));
				float Ft = 1 - Fr;

				return Trace(refractRay, iter + 1) * Ft + Trace(reflectRay, iter + 1) * Fr;
			}
			if (ray.media == Air) {
				float3 refractRayDir = normalize(-cos2 * N + refractive[AirToGlass] * (ray.D + cos1 * N));
				Ray refractRay = Ray(I + refractRayDir * 0.001, refractRayDir, 10000, Glass);

				float3 reflectRayDir = normalize(reflect(ray.D, N));
				Ray reflectRay = Ray(I + reflectRayDir * 0.001, reflectRayDir, 10000, Air);

				float Fr = 0.5 * ((pow((cos1 - refractive[GlassToAir] * cos2) / (cos1 + refractive[GlassToAir] * cos2), 2)) + (pow((cos2 - refractive[GlassToAir] *cos1) / (cos2 + refractive[GlassToAir] * cos1), 2)));
				float Ft = 1 - Fr;

				return Absorb(Trace(refractRay, iter + 1) * Ft, ray.t, color * 0.1) + Trace(reflectRay, iter + 1) * Fr;
			}
		}
	}
	return color;
}

float3 Renderer::PathTrace(Ray& ray, int iter = 0, int n = 100) {
	scene.FindNearest(ray);
	MatType mat = scene.GetObjMat(ray.objIdx);
	float3 color = scene.GetLightColor(ray.objIdx);

	if (mat == Light) return color;
	if (ray.objIdx == -1 || iter > 1) return 0; // or a fancy sky color

	//float r = rand() / RAND_MAX;
	//float p = 0.8;
	//if (r > p) return 0;

	float3 I = ray.O + ray.t * ray.D;
	float3 N = scene.GetNormal(ray.objIdx, I, ray.D);
	//float3 albedo = scene.GetAlbedo(ray.objIdx, I);
	float cos1 = dot(N, -ray.D);
	
	int i;
	float3 color_accum = float3(0);
	for (i = 0; i < n; i++) {
		float3 bounceRayDir = normalize(N + random_in_uint_sphere());
		float bounceCos = -dot(ray.D, bounceRayDir);
		Ray bounceRay = Ray(I + bounceRayDir * 0.001, bounceRayDir, 10000, ray.media);

		float3 bounceColor = PathTrace(bounceRay, iter + 1, n);
		color_accum += bounceColor;
	}
	color_accum *= 1.0 / n;
	return color * fsqrt(color_accum);
}

float Renderer::DirectIllumination(float3& I) {
	float3 shadowRayDirection = scene.GetLightPos() - I;
	float distance = length(shadowRayDirection);
	Ray shadowray = Ray(I + shadowRayDirection * 0.001, normalize(shadowRayDirection), distance);
	if (scene.IsOccluded(shadowray)) return 0.0f;
	else return 1.0f / distance;
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

	// lines are executed as OpenMP parallel tasks (disabled in DEBUG)
#	pragma omp parallel for schedule(dynamic)
	for (int y = 0; y < SCRHEIGHT; y++)
	{
		// trace a primary ray for each pixel on the line
		for (int x = 0; x < SCRWIDTH; x++)
		{
			//float3 color = float3(0);
			////color = Trace(camera.GetPrimaryRay(x, y));

			////anti-aliasing
			//color = color + Trace(camera.GetPrimaryRay(x + 0.25, y + 0.1));
			//color = color + Trace(camera.GetPrimaryRay(x - 0.25, y - 0.1));
			//color = color + Trace(camera.GetPrimaryRay(x + 0.1, y - 0.25));
			//color = color + Trace(camera.GetPrimaryRay(x - 0.1, y + 0.25));
			//color = 0.25 * color;
			//accumulator[x + y * SCRWIDTH] =
			//float4(color, 0);

			accumulator[x + y * SCRWIDTH] =
				float4(PathTrace(camera.GetPrimaryRay(x, y), 0, 5), 0);
		}
			
		// translate accumulator contents to rgb32 pixels
		for (int dest = y * SCRWIDTH, x = 0; x < SCRWIDTH; x++)
			screen->pixels[dest + x] =
			RGBF32_to_RGB8(&accumulator[x + y * SCRWIDTH]);
	}
	
	// in game control
	if (GetKeyState('A') < 0) camera.Translate(float3(0.1, 0, 0));
	if (GetKeyState('D') < 0) camera.Translate(float3(-0.1, 0, 0));
	if (GetKeyState('S') < 0) camera.Translate(float3(0, 0, 0.1));
	if (GetKeyState('W') < 0) camera.Translate(float3(0, 0, -0.1));
	if (GetKeyState('Q') < 0) camera.Translate(float3(0, -0.1, 0));
	if (GetKeyState('E') < 0) camera.Translate(float3(0, 0.1, 0));
	if (GetKeyState(37) < 0) camera.Rotate(0, 0.05);
	if (GetKeyState(38) < 0) camera.Rotate(0.05, 0);
	if (GetKeyState(39) < 0) camera.Rotate(0, -0.05);
	if (GetKeyState(40) < 0) camera.Rotate(-0.05, 0);
	
	// performance report - running average - ms, MRays/s
	static float avg = 10, alpha = 1;
	avg = (1 - alpha) * avg + alpha * t.elapsed() * 1000;
	if (alpha > 0.05f) alpha *= 0.5f;
	float fps = 1000 / avg, rps = (SCRWIDTH * SCRHEIGHT) * fps;
	
	
	//printf("%5.2fms (%.1fps) - %.1fMrays/s\n", avg, fps, rps / 1000000);
}