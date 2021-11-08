#ifdef GL_ES
precision mediump float;
#endif

uniform mat4 modelViewProjectionMatrix;
attribute vec3 normal;
attribute vec3 direction;
attribute vec4 position;
attribute float curvature;
varying vec3 vPos;
varying vec3 vNor;
varying vec3 vDir;
varying float vCur;

attribute vec2 texcoord;
varying vec2 u_texcoord;
void main() {
    gl_Position = modelViewProjectionMatrix * position;
    vPos = position.xyz;
    vNor = normal;
    vDir = direction;
    vCur = curvature;
    u_texcoord = texcoord;
}
