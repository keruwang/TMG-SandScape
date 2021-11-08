#ifdef GL_ES
precision mediump float;
#endif
uniform float u_time;
uniform vec2 u_resolution;
uniform vec2 u_mouse;
uniform float u_mode;

varying vec3 vPos;

const vec3 white = vec3(1.,1.,1.);
const vec3 black = vec3(0.,0.,0.);


float noise(vec3 v) {
   vec4 r[2];
   const mat4 E = mat4(0.,0.,0.,0., 0.,.5,.5,0., .5,0.,.5,0., .5,.5,0.,0.);
   for (int j = 0 ; j < 2 ; j++)
   for (int i = 0 ; i < 4 ; i++) {
      vec3 p = .60*v + E[i].xyz, C = floor(p), P = p - C-.5, A = abs(P), D;
      C += mod(C.x+C.y+C.z+float(j),2.) * step(max(A.yzx,A.zxy),A)*sign(P);
      D  = 314.1*sin(59.2*float(i+4*j) + 65.3*C + 58.9*C.yzx + 79.3*C.zxy);
      r[j][i] = dot(P=p-C-.5,fract(D)-.5) * pow(max(0.,1.-2.*dot(P,P)),4.);
   }
   return 6.50 * (r[0].x+r[0].y+r[0].z+r[0].w+r[1].x+r[1].y+r[1].z+r[1].w);
}

void main() {
    if(sin(u_time) > 0.) gl_FragColor = vec4(sin(u_time) * vec3(1.3,0.8,0.1),1.);
}
