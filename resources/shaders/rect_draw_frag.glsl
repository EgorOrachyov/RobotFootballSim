// Remember: macOS max supported version is 4.1
#version 410 core

layout (location = 0) out vec4 outColor;

flat in vec4 fsPenColor;
flat in vec4 fsBrushColor;
flat in int fsIsFilled;

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
    vec4 color;

    if (fsIsFilled != 0)
        color = fsBrushColor;
    else
        color = fsPenColor;

    outColor = convertLinearToSRGB(color);
    outColor.a = clamp(outColor.a, 0.0f, 1.0f);
}