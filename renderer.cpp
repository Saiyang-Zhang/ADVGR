#include "precomp.h"

// -----------------------------------------------------------
// Initialize the renderer
// -----------------------------------------------------------
void Renderer::Init()
{
	// create fp32 rgb pixel buffer to render to
	accumulator = (float4*)MALLOC64(SCRWIDTH * SCRHEIGHT * 16);
	memset(accumulator, 0, SCRWIDTH * SCRHEIGHT * 16);
}

// -----------------------------------------------------------
// Evaluate light transport
// -----------------------------------------------------------
float3 Renderer::Trace(Ray& ray, int iter = 0)
{
	if (iter > 2) return 0;
	float3 black = float3(0, 0, 0);
	scene.FindNearest(ray);
	if (ray.objIdx == -1) return 0; // or a fancy sky color
	float3 I = ray.O + ray.t * ray.D;
	float3 N = scene.GetNormal(ray.objIdx, I, ray.D);
	float3 albedo = scene.GetAlbedo(ray.objIdx, I);

	MatType type = scene.GetObjMat(ray.objIdx);
	float3 color = scene.GetLightColor(ray.objIdx);
	float3 shadowRayDirection = scene.GetLightPos() - I;
	float distance = length(shadowRayDirection);
	Ray shadowray = Ray(I + shadowRayDirection * 0.001, normalize(shadowRayDirection), distance);
	if (scene.IsOccluded(shadowray)) return black;
	if (type == Mirror) {
		float3 mirrorRayDirection = ray.D - 2 * dot(ray.D, N) * N;
		Ray mirrorRay = Ray(I + shadowRayDirection * 0.001, normalize(mirrorRayDirection));
		return Trace(mirrorRay, iter + 1);
	}
	if (type == Distance) {
		return color / distance;
	}
	return color;
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
#pragma omp parallel for schedule(dynamic)
	for (int y = 0; y < SCRHEIGHT; y++)
	{
		// trace a primary ray for each pixel on the line
		for (int x = 0; x < SCRWIDTH; x++)
			accumulator[x + y * SCRWIDTH] =
			float4(Trace(camera.GetPrimaryRay(x, y)), 0);
		// translate accumulator contents to rgb32 pixels
		for (int dest = y * SCRWIDTH, x = 0; x < SCRWIDTH; x++)
			screen->pixels[dest + x] =
			RGBF32_to_RGB8(&accumulator[x + y * SCRWIDTH]);
	}
	// performance report - running average - ms, MRays/s
	static float avg = 10, alpha = 1;
	avg = (1 - alpha) * avg + alpha * t.elapsed() * 1000;
	if (alpha > 0.05f) alpha *= 0.5f;
	float fps = 1000 / avg, rps = (SCRWIDTH * SCRHEIGHT) * fps;
	if (_kbhit()) {
		KeyDown(_getch());
	}
	//printf("%5.2fms (%.1fps) - %.1fMrays/s\n", avg, fps, rps / 1000000);
}