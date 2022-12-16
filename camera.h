#pragma once

// default screen resolution
#define SCRWIDTH	1280
#define SCRHEIGHT	640
// #define FULLSCREEN
// #define DOUBLESIZE

namespace Tmpl8 {

	class Camera
	{
	public:
		Camera()
		{
			// setup a basic view frustum
			camPos = float3(0, 0, -29);
			topLeft = float3(-aspect, 1, 2);
			topRight = float3(aspect, 1, 2);
			bottomLeft = float3(-aspect, -1, 2);

			M = mat4::Identity();
			invM = M.FastInvertedTransformNoScale();
			R = mat4::Identity();
			invR = M.FastInvertedTransformNoScale();
		}
		Ray GetPrimaryRay(const float x, const float y)
		{
			// calculate pixel position on virtual screen plane
			const float u = (float)x * (1.0f / SCRWIDTH);
			const float v = (float)y * (1.0f / SCRHEIGHT);
			float3 tL = TransformVector(topLeft, invR);
			float3 tR = TransformVector(topRight, invR); 
			float3 bL = TransformVector(bottomLeft, invR);
			const float3 P = tL + u * (tR - tL) + v * (bL - tL);
			return Ray(TransformPosition(camPos, invM), normalize(P));
		}
		void Translate(float3 vector) {
			float3 trueVector = TransformVector(vector, invR);
			M = M * mat4::Translate(trueVector);
			invM = M.FastInvertedTransformNoScale();
		}
		void Rotate(float x, float y) {
			R = mat4::RotateX(x) * mat4::RotateY(y) * R;
			invR = R.FastInvertedTransformNoScale();
		}
		float aspect = (float)SCRWIDTH / (float)SCRHEIGHT;
		float3 camPos;
		float3 topLeft, topRight, bottomLeft;
		mat4 M, invM, R, invR;
	};
}