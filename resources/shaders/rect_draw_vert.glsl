// Remember: macOS max supported version is 4.1
#version 410 core

layout (location = 0) in vec3 position;
layout (location = 1) in vec4 penColor;
layout (location = 2) in vec4 brushColor;
layout (location = 3) in float isFilled;

uniform mat4 projView;

flat out vec4 fsPenColor;
flat out vec4 fsBrushColor;
flat out int fsIsFilled;

void main() {
    fsPenColor = penColor;
    fsBrushColor = brushColor;
    fsIsFilled = isFilled != 0.0f ? 1: 0;
    gl_Position = projView * vec4(position, 1.0f);
}