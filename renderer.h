#pragma once

namespace Tmpl8
{

	class Renderer : public TheApp
	{
	public:
		// game flow methods
		void Init();
		float3 Trace(Ray& ray, int iter);
		float3 PathTrace(Ray& ray, float iter);
		float3 PathTraceNew(Ray& ray, float iter);
		float3 BDPT(Ray& ray, float iter);
		void Tick(float deltaTime);
		void Shutdown() { /* implement if you want to do something on exit */ }
		// input handling
		void MouseUp(int button) { /* implement if you want to detect mouse button presses */ }
		void MouseDown(int button) { /* implement if you want to detect mouse button presses */ }
		void MouseMove(int x, int y) { 
			mousePos.x = x, mousePos.y = y; 
		}
		void MouseWheel(float y) { /* implement if you want to handle the mouse wheel */ }
		void KeyUp(int key) { /* implement if you want to handle keys */ }
		void KeyDown(int key) { };

		// data members
		int2 mousePos;
		float4* accumulator;
		Scene scene;
		Camera camera;
		//This is the counter for real time sampling for path tracing
		float sample;
		bool pathTrace;
		int maxPathLength = 16;
	};

} // namespace Tmpl8