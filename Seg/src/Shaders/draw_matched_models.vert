#version 330 core

layout (location = 0) in vec4 position;

uniform mat4 MVP;
uniform int modelNum;

out vec4 vColor;

void main()
{
	vColor = vec4(1.0f - 1.0f / modelNum * position.w, 1.0f / modelNum * position.w, 1.0f / modelNum * position.w, 1.0f);

	gl_Position = MVP * vec4(position.xyz, 1.0);
}
