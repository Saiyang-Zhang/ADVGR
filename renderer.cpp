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
	
	float3 L = scene.GetLightPos() - I;
	float3 shadowRayDir = normalize(L);
	float distance = length(L);
	Ray shadowray = Ray(I + shadowRayDir * 0.001, shadowRayDir, distance, ray.media);
	float3 albedo = scene.GetRadiance(I, N, ray.D, 0.1, 10);

	//MatType mat = scene.GetObjMatType(ray.objIdx);
	//float3 lightColor = scene.GetLightColor();

	//if (scene.IsOccluded(shadowray)) {
	//	if (mat == Mirror) {
	//		float3 reflectRayDir = normalize(reflect(ray.D, N));
	//		Ray mirrorRay = Ray(I + reflectRayDir * 0.001, reflectRayDir);
	//		return albedo * Trace(mirrorRay, iter + 1);
	//	}
	//	if (mat == Glass) {
	//		
	//	}
	//	else return 0.0f;
	//}

	//if (mat == Diffuse) {
	//	return albedo;
	//}
	//if (mat == Mirror) {
	//	float3 reflectRayDir = normalize(reflect(ray.D, N));
	//	Ray mirrorRay = Ray(I + reflectRayDir * 0.001, reflectRayDir, INF, Air);
	//	return albedo * Trace(mirrorRay, iter + 1);
	//}	
	//if (mat == Glass) 
	//{	
	//	float3 reflectRayDir = normalize(reflect(ray.D, N));
	//	Ray reflectRay = Ray(I + reflectRayDir * 0.001, reflectRayDir, INF, ray.media);
	//	
	//	float k;
	//	float cos2;
	//	if (ray.media == Air)
	//	{
	//		cos2 = sqrt(1 - pow(refractive[AirToGlass] * sqrt(1 - pow(cos1, 2)), 2));
	//		k = 1 - pow(refractive[AirToGlass], 2) * (1 - pow(cos1, 2));
	//		
	//		float3 refractRayDir = normalize(-cos2 * N + refractive[AirToGlass] * (ray.D + cos1 * N));
	//		Ray refractRay = Ray(I + refractRayDir * 0.001, refractRayDir, INF, Glass);
	//		
	//		float Fr = 0.5 * ((pow((cos1 - refractive[GlassToAir] * cos2) / (cos1 + refractive[GlassToAir] * cos2), 2)) + (pow((cos2 - refractive[GlassToAir] * cos1) / (cos2 + refractive[GlassToAir] * cos1), 2)));
	//		float Ft = 1 - Fr;

	//		return Absorb(Trace(refractRay, iter + 1) * Ft, ray.t, albedo * 0.1) + Trace(reflectRay, iter + 1) * Fr;
	//	}
	//	if (ray.media == Glass) {
	//		k = 1 - pow(refractive[GlassToAir], 2) * (1 - pow(cos1, 2));
	//		if (k < 0) return Trace(reflectRay, iter + 1);
	//		else {
	//			cos2 = sqrt(1 - pow(refractive[GlassToAir] * sqrt(1 - pow(cos1, 2)), 2));

	//			float3 refractRayDir = normalize(-cos2 * N + refractive[GlassToAir] * (ray.D + cos1 * N));
	//			Ray refractRay = Ray(I + refractRayDir * 0.001, refractRayDir, INF, Air);

	//			float Fr = 0.5 * ((pow((refractive[GlassToAir] * cos1 - cos2) / (refractive[GlassToAir] * cos1 + cos2), 2)) + (pow((refractive[GlassToAir] * cos2 - cos1) / (refractive[GlassToAir] * cos2 + cos1), 2)));
	//			float Ft = 1 - Fr;

	//			return Trace(refractRay, iter + 1) * Ft + Trace(reflectRay, iter + 1) * Fr;
	//		}
	//	}
	//}
	return albedo;
}

float3 Renderer::PathTrace(Ray& ray, float iter = 0) {
	if (iter > 8) return 0;
	scene.FindNearest(ray);
	MatType mat = scene.GetObjMatType(ray.objIdx);
	float3 result;

	if (mat == Light) return scene.GetAlbedo(ray.objIdx, 0);
	if (ray.objIdx == -1) return 0; // or a fancy sky color

	//In order to reduce too many recursion, we use this method to randomly decide whether one ray
	//should stop bouncing between objects. Here the probability of keeping the ray is P = 0.8
	double r = rand() * (1.0 / RAND_MAX);
	float P = 0.8;
	if (r > P) return float3(0);

	float3 I = ray.O + ray.t * ray.D;
	float3 N = scene.GetNormal(ray.objIdx, I, ray.D);
	float cos1 = dot(N, -ray.D);
	float3 albedo = scene.GetAlbedo(ray.objIdx, I);

	//Choose the random ray that bounce between objects to implement the environment lighting
	float3 randomRayDir = normalize(random_in_hemisphere(N));
	float bounceCos = -dot(ray.D, randomRayDir);
	Ray randomRay = Ray(I + randomRayDir * 0.001, randomRayDir, INF, ray.media);
	
	//float3 albedo = scene.GetAlbedo(ray.objIdx, I);

	//Basic material is for testing, will still be referred to in future development
	if (mat == Basic) result = cos1 * 1.25 * albedo;

	//To make the expectation of the color even, we need to divide the result by P. And to avoid
	//division, we multiply the color by reciprocal of P
	if (mat == Diffuse) {
		result = cos1 * 1.25 * albedo * PathTrace(randomRay, iter + 1);
	}

	if (mat == Mirror) {
		float3 reflectRayDir = normalize(reflect(ray.D, N));
		Ray mirrorRay = Ray(I + reflectRayDir * 0.001, reflectRayDir);
		result = cos1 * 1.25 * albedo * PathTrace(mirrorRay, iter+1);
	}
	if (mat == Glass)
	{
		float3 reflectRayDir = normalize(reflect(ray.D, N));
		Ray reflectRay = Ray(I + reflectRayDir * 0.001, reflectRayDir, INF, ray.media);

		float k;
		float cos2;
		if (ray.media == Air)
		{
			cos2 = sqrt(1 - pow(refractive[AirToGlass] * sqrt(1 - pow(cos1, 2)), 2));
			k = 1 - pow(refractive[AirToGlass], 2) * (1 - pow(cos1, 2));

			float3 refractRayDir = normalize(-cos2 * N + refractive[AirToGlass] * (ray.D + cos1 * N));
			Ray refractRay = Ray(I + refractRayDir * 0.001, refractRayDir, INF, Glass);

			float Fr = 0.5 * ((pow((cos1 - refractive[GlassToAir] * cos2) / (cos1 + refractive[GlassToAir] * cos2), 2)) + (pow((cos2 - refractive[GlassToAir] * cos1) / (cos2 + refractive[GlassToAir] * cos1), 2)));
			double r = rand() * (1.0 / RAND_MAX);
			if (r < Fr) {
				result = cos1 * 1.25 * PathTrace(reflectRay, iter + 1);
			}
			else {
				result = cos1 * 1.25 * Absorb(PathTrace(refractRay, iter + 1), ray.t, albedo * 0.1);
			}
		}
		if (ray.media == Glass) {
			k = 1 - pow(refractive[GlassToAir], 2) * (1 - pow(cos1, 2));
			if (k < 0) result = 1.25 * cos1 * PathTrace(reflectRay, iter + 1);
			else {
				cos2 = sqrt(1 - pow(refractive[GlassToAir] * sqrt(1 - pow(cos1, 2)), 2));

				float3 refractRayDir = normalize(-cos2 * N + refractive[GlassToAir] * (ray.D + cos1 * N));
				Ray refractRay = Ray(I + refractRayDir * 0.001, refractRayDir, INF, Air);

				float Fr = 0.5 * ((pow((refractive[GlassToAir] * cos1 - cos2) / (refractive[GlassToAir] * cos1 + cos2), 2)) + (pow((refractive[GlassToAir] * cos2 - cos1) / (refractive[GlassToAir] * cos2 + cos1), 2)));
				double r = rand() * (1.0 / RAND_MAX);
				if (r < Fr) {
					result = 1.25 * cos1 * PathTrace(reflectRay, iter + 1);
				}
				else {
					result = 1.25 * cos1 * PathTrace(refractRay, iter + 1);
				}
			}
		}
	}
	return result;
}

float3 Renderer::PathTraceNew(Ray& ray, float iter = 0) {
	scene.FindNearest(ray);
	MatType mat = scene.GetObjMatType(ray.objIdx);
	float3 albedo = scene.GetAlbedo(ray.objIdx, 0);

	if (mat == Light) return scene.GetLightColor();;
	if (ray.objIdx == -1 || iter > maxPathLength) return 0; // or a fancy sky color

	if (ray.media == Glass) {
		Absorb(albedo, ray.t, albedo);
	}

	//In order to reduce too many recursion, we use this method to decide whether one ray
	//should stop bouncing between objects using russian roulette.
	double r = rand() * (1.0 / RAND_MAX);
	float m = max(albedo.x, albedo.y);
	float P = max(min(max(m, albedo.z), 0.9f), 0.1f);
	if (r > P) return float3(0);

	float3 I = ray.O + ray.t * ray.D;
	float3 N = scene.GetNormal(ray.objIdx, I, ray.D);
	float cos1 = dot(N, -ray.D);
	float3 color_accum = float3(0);

	if (mat == Diffuse) {

		//Choose the random ray that bounce between objects to implement the environment lighting
		float3 randomRayDir = normalize(random_in_hemisphere(N));
		//float3 randomRayDir = normalize(N + normalize(CosineWeightedDiffuseReflection()));
		float bounceCos = -dot(ray.D, randomRayDir);
		Ray rayToHemisphere = Ray(I + randomRayDir * 0.001, randomRayDir, INF, ray.media);
		scene.FindNearest(rayToHemisphere);

		if (scene.GetObjMatType(rayToHemisphere.objIdx) == Light) {
			float3 BRDF = albedo * INVPI;
			float cos_i = dot(randomRayDir, N);
			float PDF = 1 / BRIGHTNESS;
			//float PDF = cos_i / PI;
			float3 Ei = scene.GetAlbedo(rayToHemisphere.objIdx, 0) * cos_i / PDF;
			return BRDF * Ei;
		}
		return float3(0);
	}

	if (mat == Mirror) {
		float3 reflectRayDir = normalize(reflect(ray.D, N));
		Ray mirrorRay = Ray(I + reflectRayDir * 0.001, reflectRayDir);
		return PathTraceNew(mirrorRay, iter++);
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
			return PathTraceNew(mirrorRay, iter++);
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

				return PathTraceNew(refractRay, iter++);
			}
			if (ray.media == Glass) {

				float cos2 = sqrt(1 - pow(refractive[GlassToAir] * sqrt(1 - pow(cos1, 2)), 2));

				float3 refractRayDir = normalize(-cos2 * N + refractive[GlassToAir] * (ray.D + cos1 * N));
				Ray refractRay = Ray(I + refractRayDir * 0.001, refractRayDir, INF, Air);

				return PathTraceNew(refractRay, iter++);
			}
		}
	}
}

float3 Renderer::BDPT(Ray& ray, float iter = 0) {
	if (iter > 8) return 0;

	double r = rand() * (1.0 / RAND_MAX);
	float P = 0.8;
	if (r > P) return 0;

	scene.FindNearest(ray);
	MatType mat = scene.GetObjMatType(ray.objIdx);
	if (ray.objIdx == -1) return 0;
	if (mat == Light) {
		if (ray.media == Mirror) return 0;
		return 0;
	}
	
	
	float3 I = ray.O + ray.t * ray.D;
	float3 N = scene.GetNormal(ray.objIdx, I, ray.D);
	float A = scene.GetLightArea();
	float3 Ld = 0;
	float3 Ei;
	float3 BRDF = scene.GetAlbedo(ray.objIdx, 0) * INVPI;

	float3 pointOnLight = scene.GetLightPoint();
	float3 N1 = scene.GetLightNormal(pointOnLight);
	if (mat == Diffuse) {
		/*float3 shadowRayDir = normalize(pointOnLight - I);
		if (dot(N, shadowRayDir) > 0 && -dot(N1, shadowRayDir) > 0) {
			float dist = length(pointOnLight - I);
			Ray shadowRay = Ray(I + shadowRayDir * 0.001, shadowRayDir, dist - 0.001);
			if (!scene.IsOccluded(shadowRay)) {
				float solidAngle = (-dot(N1, shadowRayDir) * A) / (dist * dist);
				Ld = scene.GetLightColor() * solidAngle * BRDF * dot(N, shadowRayDir);
			}
		}*/
		Ld = scene.GetRadiance(I, N, ray.D, 0.1, 10) / (ray.t * ray.t);
		float3 randomRayDir = normalize(random_in_hemisphere(N));
		Ray randomRay = Ray(I + randomRayDir * 0.001, randomRayDir, INF, ray.media); 
		Ei = BDPT(randomRay, iter + 1) * dot(N, randomRayDir) * INVPI;
	}

	if (mat == Mirror) {
		float3 reflectRayDir = normalize(reflect(ray.D, N));
		Ray mirrorRay = Ray(I + reflectRayDir * 0.001, reflectRayDir, INF, Mirror);
		Ei = BDPT(mirrorRay, iter + 1);
	}

	if (mat == Glass)
	{
		float cos1 = dot(N, -ray.D);
		float k;
		float cos2;
		if (ray.media == Air)
		{
			cos2 = sqrt(1 - pow(refractive[AirToGlass] * sqrt(1 - pow(cos1, 2)), 2));
			k = 1 - pow(refractive[AirToGlass], 2) * (1 - pow(cos1, 2));
			float Fr = 0.5 * ((pow((cos1 - refractive[GlassToAir] * cos2) / (cos1 + refractive[GlassToAir] * cos2), 2)) + (pow((cos2 - refractive[GlassToAir] * cos1) / (cos2 + refractive[GlassToAir] * cos1), 2)));

			double r = rand() * (1.0 / RAND_MAX);
			if (r < Fr) {
				float3 reflectRayDir = normalize(reflect(ray.D, N));
				Ray reflectRay = Ray(I + reflectRayDir * 0.001, reflectRayDir, INF, Mirror);
				Ei = BDPT(reflectRay, iter + 1);
			}
			else {
				float3 refractRayDir = normalize(-cos2 * N + refractive[AirToGlass] * (ray.D + cos1 * N));
				Ray refractRay = Ray(I + refractRayDir * 0.001, refractRayDir, INF, Glass);
				Ei = Absorb(BDPT(refractRay, iter + 1), ray.t, 0.1 * scene.GetRadiance(I, N, ray.D, 0.1, 10));
			}
		}
		if (ray.media == Glass) {
			k = 1 - pow(refractive[GlassToAir], 2) * (1 - pow(cos1, 2));
			if (k < 0) {
				float3 reflectRayDir = normalize(reflect(ray.D, N));
				Ray reflectRay = Ray(I + reflectRayDir * 0.001, reflectRayDir, INF, ray.media);
				Ei = BDPT(reflectRay, iter + 1);
			}
			else {
				cos2 = sqrt(1 - pow(refractive[GlassToAir] * sqrt(1 - pow(cos1, 2)), 2));
				float Fr = 0.5 * ((pow((refractive[GlassToAir] * cos1 - cos2) / (refractive[GlassToAir] * cos1 + cos2), 2)) + (pow((refractive[GlassToAir] * cos2 - cos1) / (refractive[GlassToAir] * cos2 + cos1), 2)));

				double r = rand() * (1.0 / RAND_MAX);
				if (r < Fr) {
					float3 reflectRayDir = normalize(reflect(ray.D, N));
					Ray reflectRay = Ray(I + reflectRayDir * 0.001, reflectRayDir, INF, ray.media);
					Ei = BDPT(reflectRay, iter + 1);
				}
				else {
					float3 refractRayDir = normalize(-cos2 * N + refractive[GlassToAir] * (ray.D + cos1 * N));
					Ray refractRay = Ray(I + refractRayDir * 0.001, refractRayDir, INF, Air);
					Ei = BDPT(refractRay, iter + 1);
				}
			}
		}
	}
	return 1.25 * PI * 2.0f * BRDF * Ei + Ld;
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
//
//1. Whitted-style ray-tracing, uncomment this and comment 2. 3. to render this way
// lines are executed as OpenMP parallel tasks (disabled in DEBUG)
//#	pragma omp parallel for schedule(dynamic)
//	for (int y = 0; y < SCRHEIGHT; y++)
//	{
//		for (int x = 0; x < SCRWIDTH; x++)
//		{
//			float3 color = float3(0);
//			color = Trace(camera.GetPrimaryRay(x, y));
//
//			accumulator[x + y * SCRWIDTH] =
//				float4(color, 0);
//		}
//
//		for (int dest = y * SCRWIDTH, x = 0; x < SCRWIDTH; x++)
//			screen->pixels[dest + x] =
//			RGBF32_to_RGB8(&accumulator[x + y * SCRWIDTH]);
//	}
//
////2. Real-time sampling for path tracing, uncomment this and comment 1. 2. to render this way
////(in game control is hardly usable here).
//	// lines are executed as OpenMP parallel tasks (disabled in DEBUG)
//#	pragma omp parallel for schedule(dynamic)
//	for (int y = 0; y < SCRHEIGHT; y++)
//	{
//		// trace a primary ray for each pixel on the line
//		for (int x = 0; x < SCRWIDTH; x++)
//		{
//			//float3 color = float3(0);
//			//for (int i = 0; i < 10; i++) {
//			//	color += PathTrace(camera.GetPrimaryRay(x, y));
//			//}
//
//			//color *= BRIGHTNESS / (float)10;
//			float3 color = BDPT(camera.GetPrimaryRay(x, y));
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
//
////4. Real-time sampling for path tracing, uncomment this and comment 1. 2. to render this way
////(in game control is hardly usable here).
//	//printf("sample: %f\n", sample);
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
////
////4. Real-time sampling for path tracing, uncomment this and comment 1. 2. to render this way
////(in game control is hardly usable here).
//	printf("sample: %f\n", sample);
//	// lines are executed as OpenMP parallel tasks (disabled in DEBUG)
//#	pragma omp parallel for schedule(dynamic)
//	for (int y = 0; y < SCRHEIGHT; y++)
//	{
//		// trace a primary ray for each pixel on the line
//		for (int x = 0; x < SCRWIDTH; x++)
//		{
//			float3 color = PathTraceNew(camera.GetPrimaryRay(x, y));
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
//5. Real-time sampling for BDPT, uncomment this and comment 1. 2. to render this way
//(in game control is hardly usable here).
	//printf("sample: %f\n", sample);
	// lines are executed as OpenMP parallel tasks (disabled in DEBUG)
#	pragma omp parallel for schedule(dynamic)
	for (int y = 0; y < SCRHEIGHT; y++)
	{
		// trace a primary ray for each pixel on the line
		for (int x = 0; x < SCRWIDTH; x++)
		{
			float3 color = BDPT(camera.GetPrimaryRay(x, y));

			accumulator[x + y * SCRWIDTH] *= (sample - 1) / sample;
			accumulator[x + y * SCRWIDTH] += float4(color * (BRIGHTNESS / sample), 0);
		}

		// translate accumulator contents to rgb32 pixels
		for (int dest = y * SCRWIDTH, x = 0; x < SCRWIDTH; x++)
			screen->pixels[dest + x] =
			RGBF32_to_RGB8(&accumulator[x + y * SCRWIDTH]);
	}
	sample++;
	
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

	//printf("%5.2fms (%.1fps) - %.1fMrays/s\n", avg, fps, rps / 1000000);
	//printf("ray count: %d, avg depth: %f\n", raycount, alldepth / raycount);
	//printf("%5.2fms nodes: %d\n", totaltime/bvhcount * 1000, scene.nodes.size());
}