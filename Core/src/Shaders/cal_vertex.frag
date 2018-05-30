#version 330 core

in vec2 texcoord;

uniform sampler2D gSampler;
uniform vec4 cam; //cx, cy, 1/fx, 1/fy
uniform float cols;
uniform float rows;
uniform float maxD;

layout(location = 0) out vec3 vertex;

#include "geometry.glsl"

void main()
{
    float x = texcoord.x * cols;
    float y = texcoord.y * rows;

    vertex = getVertex(texcoord.xy, x, y, cam, gSampler);

	if (vertex.z <= 0 || vertex.z > maxD)
	{
		vertex = vec3(0, 0, 0);
	}

	//vertex.x = vertex.x * 1000;
	//vertex.y = vertex.y * 1000;
	//vertex.z = vertex.z * 1000;
}
