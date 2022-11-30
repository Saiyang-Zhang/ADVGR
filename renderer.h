#pragma once

namespace Tmpl8
{

	class Renderer : public TheApp
	{
	public:
		// game flow methods
		void Init();
		float3 Trace(Ray& ray, int iter);
		float DirectIllumination(float3& I);
		float3 Absorption(float3 color, float d, float3 absorpMat);
		void Tick(float deltaTime);
		void Shutdown() { /* implement if you want to do something on exit */ }
		// input handling
		void MouseUp(int button) { /* implement if you want to detect mouse button presses */ }
		void MouseDown(int button) { /* implement if you want to detect mouse button presses */ }
		void MouseMove(int x, int y) { mousePos.x = x, mousePos.y = y; }
		void MouseWheel(float y) { /* implement if you want to handle the mouse wheel */ }
		void KeyUp(int key) { /* implement if you want to handle keys */ }
		void KeyDown(int key) {
			if (key == 65 /*right*/) {
				cout << "A" << endl;
				camera.Translate(float3(-0.1, 0, 0));
			}
			if (key == 68 /*left*/) {
				cout << "D" << endl;
				camera.Translate(float3(0.1, 0, 0));
			}
			if (key == 83 /*down*/) {
				cout << "S" << endl;
				camera.Translate(float3(0, -0.1, 0));
			}
			if (key == 87 /*up*/) {
				cout << "W" << endl;
				camera.Translate(float3(0, 0.1, 0));
			}
			//if (key == 97 /*right*/) {
			//	cout << "a" << endl;
			//}
			//if (key == 100 /*left*/) {
			//	cout << "d" << endl;
			//}
			//if (key == 115 /*down*/) {
			//	cout << "s" << endl;
			//}
			//if (key == 119 /*up*/) {
			//	cout << "w" << endl;
			//}
			if (key == 262 /*right*/) {
				cout << "right" << endl;
			}
			if (key == 263 /*left*/) {
				cout << "left" << endl;
			}
			if (key == 264 /*down*/) {
				cout << "down" << endl;
			}
			if (key == 265 /*up*/) {
				cout << "up" << endl;
			}
			cout << key << endl;
		}
		// data members
		int2 mousePos;
		float4* accumulator;
		Scene scene;
		Camera camera;
		float indexAir, indexGlass, airToGlass, glassToAir;
	};

} // namespace Tmpl8