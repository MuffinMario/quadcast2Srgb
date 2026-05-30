#version 300 es
precision mediump float;
// optional — set by CGLSLDisplay when present in the linked program:
uniform float iTime;         // seconds since Initialize() or last Reset()
uniform vec2  iResolution;   // always vec2(12.0, 9.0)
uniform int   iFrame;        // frames rendered since Initialize() / Reset()
out vec4 fragColor;
void main() { 
    fragColor = vec4(1.0,0,0,1.0); 
}