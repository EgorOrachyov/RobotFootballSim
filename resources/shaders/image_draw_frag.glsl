// Remember: macOS max supported version is 4.1
#version 410 core

layout (location = 0) out vec4 outColor;

uniform sampler2D imageTexture;

in vec2 fsTexCoords;
in vec4 fsColor;
flat in vec4 fsTransparentColor;
flat in int fsIsSRGB;

#define COLOR_GAMMA 2.2

vec4 convertSRBGtoLinear(in vec4 color) {
    float alpha = color.a;
    vec3 converted = pow(color.rgb,vec3(COLOR_GAMMA));
    return vec4(converted,alpha);
}

vec4 convertLinearToSRGB(in vec4 color) {
    float alpha = color.a;
    vec3 converted = pow(color.rgb,vec3(1.0f / COLOR_GAMMA));
    return vec4(converted,alpha);
}

void main() {
    vec4 color = texture(imageTexture, fsTexCoords).rgba;

    if (fsTransparentColor.rgb == color.rgb) {
        discard;
    }

    if (fsIsSRGB == 1) {
        color = convertSRBGtoLinear(color);
    }

    vec4 multiplied = fsColor * color;

    outColor = convertLinearToSRGB(multiplied);
    outColor.a = clamp(outColor.a, 0.0f, 1.0f);
}