#version 300 es
precision highp float;

// 3 uniform variables provided by the display:
// the variables and coordinate positioning are equal to the convention of shadertoy
uniform float u_time;         // seconds since Initialize() or last Reset()
uniform vec2  u_resolution;   // vec2(12.0, 9.0) * vec2(shader_scale)
uniform int   u_frame;        // frames rendered since Initialize() / Reset()

out vec4 fragColor;

void main()
{
    // Center-origin coords, normalized to [-0.5, 0.5] on the short axis
    vec2 uv = (gl_FragCoord.xy - u_resolution * 0.5) / min(u_resolution.x, u_resolution.y);

    float angle  = atan(uv.y, uv.x);           // -PI .. PI
    float radius = length(uv)*.2;

    const float PI = 3.14159265;
    float t      = fract(u_time * 0.4 + 0.35); // 0.35 = desired start phase [0,1)
    float spiral = fract((angle / (2.0 * PI)) - radius * 4.0 + t);

    vec3 col1    = vec3(0.5882, 0.1098, 0.6549);
    vec3 col2 = vec3(0.1686, 0.0235, 0.1804);

    fragColor = vec4(mix(col1, col2, spiral), 1.0);
}