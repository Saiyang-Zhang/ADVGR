#pragma once

// default screen resolution
#define SCRWIDTH	1280
#define SCRHEIGHT	720
// #define FULLSCREEN
// #define DOUBLESIZE

namespace Tmpl8 {

	class Camera
	{
	public:
		Camera()
		{
			// setup a basic view frustum
			camPos = float3(0, 0, -2);
			topLeft = float3(-aspect, 1, 2);
			topRight = float3(aspect, 1, 2);
			bottomLeft = float3(-aspect, -1, 2);
			M = mat4::Identity();
			invM = M.FastInvertedTransformNoScale();
		}
		Ray GetPrimaryRay(const int x, const int y)
		{
			// calculate pixel position on virtual screen plane
			const float u = (float)x * (1.0f / SCRWIDTH);
			const float v = (float)y * (1.0f / SCRHEIGHT);
			const float3 P = topLeft + u * (topRight - topLeft) + v * (bottomLeft - topLeft);
			return Ray(TransformPosition(camPos, invM), normalize(P));
		}
		void Translate(float3 vector) {
			float3 trueVector = TransformVector(vector, invR);
			M = M * mat4::Translate(vector);
			invM = M.FastInvertedTransformNoScale();
		}
		void Rotate(float x, float y) {
			Rotation = Rotation * mat4::RotateX(x) * mat4::RotateY(y);
			invR = Rotation.FastInvertedTransformNoScale();
			topLeft = TransformVector(topLeft, invR);
			topRight = TransformVector(topRight, invR);
			bottomLeft = TransformVector(bottomLeft, invR);
		}
		float aspect = (float)SCRWIDTH / (float)SCRHEIGHT;
		float3 camPos;
		float3 topLeft, topRight, bottomLeft;
		float sinXZ, sinYZ;
		mat4 M, invM, Rotation, invR;
	};
}