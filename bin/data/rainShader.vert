
#ifdef GL_ES
precision mediump float;
#endif

uniform mat4 modelViewProjectionMatrix;
attribute vec4 position;
attribute vec2 texcoord;
varying vec2 u_texcoord;
varying vec3 vPos;

void main() {
    gl_Position = modelViewProjectionMatrix * position;
    vPos = position.xyz;
    u_texcoord = vec2(texcoord.x,texcoord.y);
}
