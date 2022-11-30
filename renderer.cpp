#include "precomp.h"

// -----------------------------------------------------------
// Initialize the renderer
// -----------------------------------------------------------
void Renderer::Init()
{
	// create fp32 rgb pixel buffer to render to
	accumulator = (float4*)MALLOC64(SCRWIDTH * SCRHEIGHT * 16);
	memset(accumulator, 0, SCRWIDTH * SCRHEIGHT * 16);
	indexAir = 1;
	indexGlass = 1.33;
	airToGlass = indexAir / indexGlass;
	glassToAir = indexGlass / indexAir;
}

// -----------------------------------------------------------
// Evaluate light transport
// -----------------------------------------------------------
float3 Renderer::Trace(Ray& ray, int iter = 0)
{
	float3 black = float3(0, 0, 0);
	scene.FindNearest(ray);
	if (ray.objIdx == -1) return 0; // or a fancy sky color
	float3 I = ray.O + ray.t * ray.D;
	float3 N = scene.GetNormal(ray.objIdx, I, ray.D);
	float3 albedo = scene.GetAlbedo(ray.objIdx, I);
	float cosO = dot(N, -ray.D);

	MatType matType = scene.GetObjMat(ray.objIdx);
	float3 color = scene.GetLightColor(ray.objIdx);

	if (matType == diffuse || iter > 3) {
		return color * DirectIllumination(I);
	}
	if (matType == Mirror) {
		float3 reflDirection = reflect(ray.D, N);
		Ray mirrorRay = Ray(I + reflDirection * 0.001, normalize(reflDirection), Air);
		return Trace(mirrorRay, iter + 1);
	}	
	if (matType == Glass) {

		float k;
		if (ray.material == Air) k = 1 - pow(airToGlass, 2) * (1 - pow(cosO, 2)); 
		if (ray.material == Glass) k = 1 - pow(glassToAir, 2) * (1 - pow(cosO, 2));
		else return float3(0, 0, 0);
		
		if (k < 0) {
			/// TIR
			float3 reflDirection = normalize(reflect(ray.D, N));
			Ray reflectRay = Ray(I + reflDirection * 0.001, reflDirection, Air);
			return Trace(reflectRay, iter + 1);
		}
		else {
			Ray T, R;

			// for later use
			float angleRad = acos(cosO);
			float cosOt = sqrt(1 - pow(airToGlass * sin(angleRad), 2));

			// Air to Glass
			if (ray.material == Air) {
				float3 Tdir = normalize(airToGlass * ray.D + N * (airToGlass * cosO - sqrt(k)));
				T = Ray(I + Tdir * 0.001, Tdir, Glass);

				float3 reflDirection = normalize(reflect(ray.D, N));
				R = Ray(I + reflDirection * 0.001, reflDirection, Air);

				float Fr = 0.5 * ((pow((indexAir * cosO - indexGlass * cosOt) / (indexAir * cosO + indexGlass * cosOt), 2)) + (pow((indexAir * cosOt - indexGlass * cosO) / (indexAir * cosOt + indexGlass * cosO), 2)));
				float Ft = 1 - Fr;

				return Trace(T, iter + 1) * Ft + Trace(R, iter + 1) * Fr;
			}

			// Glass to Air
			if (ray.material == Glass) {
				float3 Tdir = normalize(glassToAir * ray.D + N * (glassToAir * cosO - sqrt(k)));
				Ray T = Ray(I + Tdir * 0.001, Tdir, Air);

				float3 reflDirection = normalize(reflect(ray.D, N));
				R = Ray(I + reflDirection * 0.001, reflDirection, Glass);

				float Fr = 0.5 * ((pow((indexGlass * cosO - indexAir * cosOt) / (indexGlass * cosO + indexAir * cosOt), 2)) + (pow((indexGlass * cosOt - indexAir * cosO) / (indexGlass * cosOt + indexAir * cosO), 2)));
				float Ft = 1 - Fr;

				return Absorption(Trace(T, iter + 1) * Ft, ray.t, color) + Trace(R, iter + 1) * Fr;
			}	
		}
	}
	return color;
}

float Renderer::DirectIllumination(float3& I) {
	float3 shadowRayDirection = scene.GetLightPos() - I;
	float distance = length(shadowRayDirection);
	Ray shadowray = Ray(I + shadowRayDirection * 0.001, normalize(shadowRayDirection), distance, Air);
	if (scene.IsOccluded(shadowray)) return 0.0f;
	else return 1.0f / distance;
}

float3 Renderer::Absorption(float3 color, float d, float3 absorpMat)
{
	float3 output = color;
	output.x *= exp(absorpMat.x * d);
	output.y *= exp(absorpMat.y * d);
	output.z *= exp(absorpMat.z * d);
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