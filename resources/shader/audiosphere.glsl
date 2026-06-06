#version 300 es
precision highp float;

/**
* Small shader script to demonstrate the audio volume functionality
* Displays a small purple circle at the center of the frame, lighting up and 
* expanding with increasing audio volume captured from the audio device
* IMPORTANT: Make sure you have --capture-audio enabled!! `qc2srgb ... --capture-audio` or `capture-audio = true` in the config
**/

// Uniforms provided by the display engine
uniform float u_time;          // seconds since display started
uniform vec2  u_resolution;    // render resolution (for displays: 12*scale x 9*scale)
uniform int   u_frame;         // frame counter
uniform float u_audioVolume;   // Max band volume [0, 1] from the audio processor (if enabled)

out vec4 fragColor;

void main()
{
    // Center-origin UV, normalized so the shorter axis spans [-0.5, 0.5]
    vec2 uv = (gl_FragCoord.xy - u_resolution * 0.5) 
                / min(u_resolution.x, u_resolution.y);
    uv *= vec2(0.2,1.0); 
    // Distance from center
    float dist = length(uv);

    float vol = u_audioVolume+.0;
    float innerRadius  = 0.05 + vol * 0.025;
    float outerRadius = 0.1 + vol * 0.4; 
    
    vec3 circleColor = vec3(0.2392, 0.0627, 0.2941);
    vec3 outerColor = vec3(0.);//vec3(0.2275, 0.0392, 0.2863);

    // it does not have to look perfect in a shader preview, 
    // the values are heavily averaged given we are using supersampling, else we will experience aliasing
    float radstep=  smoothstep(innerRadius,outerRadius,dist);
    vec3 color = vec3(mix(circleColor,outerColor,radstep));

    fragColor = vec4(color, 1.0);
}