#ifdef GL_ES
precision mediump float;
#endif

uniform mat4 modelViewProjectionMatrix;
attribute vec4 position;
varying vec3 vPos;

void main() {
    gl_Position = modelViewProjectionMatrix * position;
    vPos = position.xyz;
}
