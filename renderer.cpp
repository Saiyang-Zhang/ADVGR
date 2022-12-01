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
	scene.FindNearest(ray);
	if (ray.objIdx == -1 || iter > 5) return 0; // or a fancy sky color
	
	float3 I = ray.O + ray.t * ray.D;
	float3 N = scene.GetNormal(ray.objIdx, I, ray.D);
	float3 shadowRayDirection = scene.GetLightPos() - I;
	float distance = length(shadowRayDirection);
	Ray shadowray = Ray(I + shadowRayDirection * 0.001, normalize(shadowRayDirection), distance, ray.color, ray.media);
	float3 albedo = scene.GetAlbedo(ray.objIdx, I);
	float cosO = dot(N, -ray.D);

	MatType mat = scene.GetObjMat(ray.objIdx);
	float3 color = scene.GetLightColor(ray.objIdx);
	if (scene.IsOccluded(shadowray)) {
		if (mat == Mirror) {
			float3 reflDirection = reflect(ray.D, N);
			Ray mirrorRay = Ray(I + reflDirection * 0.001, normalize(reflDirection));
			return Trace(mirrorRay, iter + 1);
		}
		if (mat == Glass)
		{
			if (ray.media == Glass) {
				float3 refracRayDir = refract(ray.D, -N, GlassToAir);
				Ray refracRay = Ray(I + refracRayDir * 0.001, refracRayDir, 10000, 1, Air);
				return Trace(refracRay, iter+1);
			}
			if (ray.media == Air) {
				float3 refracRayDir = refract(ray.D, N, AirToGlass);
				Ray refracRay = Ray(refracRayDir * 0.001, refracRayDir, 10000, 1, Glass);
				return Trace(refracRay, iter+1);
			}
		}
		else return 0.0f;
	}

	if (mat == Diffuse) {
		return color / distance;
	}
	if (mat == Mirror) {
		float3 reflDirection = reflect(ray.D, N);
		Ray mirrorRay = Ray(I + reflDirection * 0.001, normalize(reflDirection));
		return Trace(mirrorRay, iter + 1);
	}	
	if (mat == Glass) 
	{	
		if (ray.media == Glass) {
			float3 refracRayDir = refract(ray.D, -N, GlassToAir);
			Ray refracRay = Ray(I + refracRayDir*0.001, refracRayDir, 10000, 1, Air);
			return Trace(refracRay, iter+1);
		}
		if (ray.media == Air) {
			float3 refracRayDir = refract(ray.D, N, AirToGlass);
			Ray refracRay = Ray(refracRayDir * 0.001, refracRayDir, 10000, 1, Glass);
			return Trace(refracRay, iter+1);
		}
		//float k;
		//if (ray.media == Lucht) k = 1 - pow(airToGlass, 2) * (1 - pow(cosO, 2)); 
		//if (ray.media == Glas) k = 1 - pow(glassToAir, 2) * (1 - pow(cosO, 2));
		//else return float3(0, 0, 0);
		//
		//if (k < 0) {
		//	/// TIR
		//	float3 reflDirection = normalize(reflect(ray.D, N));
		//	Ray reflectRay = Ray(I + reflDirection * 0.001, reflDirection, Lucht);
		//	return Trace(reflectRay, iter + 1);
		//}
		//else {
		//	Ray T, R;



		//	// Glass to Air
		//	if (ray.media == Glass) {
		//		float3 Tdir = normalize(glassToAir * ray.D + N * (glassToAir * cosO - sqrt(k)));
		//		Ray T = Ray(I + Tdir * 0.001, Tdir, Lucht);

		//		float3 reflDirection = normalize(reflect(ray.D, N));
		//		R = Ray(I + reflDirection * 0.001, reflDirection, Glass);

		//		float Fr = 0.5 * ((pow((indexGlass * cosO - indexAir * cosOt) / (indexGlass * cosO + indexAir * cosOt), 2)) + (pow((indexGlass * cosOt - indexAir * cosO) / (indexGlass * cosOt + indexAir * cosO), 2)));
		//		float Ft = 1 - Fr;

		//		return Absorption(Trace(T, iter + 1) * Ft, ray.t, color) + Trace(R, iter + 1) * Fr;
		//	}	
		//}
	}
	return color;
}

float Renderer::DirectIllumination(float3& I) {
	float3 shadowRayDirection = scene.GetLightPos() - I;
	float distance = length(shadowRayDirection);
	Ray shadowray = Ray(I + shadowRayDirection * 0.001, normalize(shadowRayDirection), distance);
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