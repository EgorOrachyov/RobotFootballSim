// Remember: macOS max supported version is 4.1
#version 410 core

layout (location = 0) in vec3 position;
layout (location = 1) in vec2 texCoords;
layout (location = 2) in vec4 color;
layout (location = 3) in vec4 transparentColor;
layout (location = 4) in float isSRGB;

uniform mat4 projView;

out vec2 fsTexCoords;
out vec4 fsColor;
flat out vec4 fsTransparentColor;
flat out int fsIsSRGB;

void main() {
    fsTexCoords = vec2(texCoords.x, 1.0 - texCoords.y);
    fsColor = color;
    fsTransparentColor = transparentColor;
    fsIsSRGB = isSRGB != 0.0f? 1: 0;
    gl_Position = projView * vec4(position, 1.0f);
}