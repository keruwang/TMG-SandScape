#ifdef GL_ES
precision mediump float;
#endif
uniform float u_time;
uniform vec2 u_resolution;
uniform vec2 u_mouse;
uniform float u_mode;
uniform float u_cloud;

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

float turbulence(vec3 p) {
    float t = 0., f = 1.;
    for (int i = 0 ; i < 10 ; i++) {
        t += abs(noise(f * p)) / f;
        f *= 2.;
    }
    return t;
}

vec3 stripes(float x) {
    float t = pow(sin(x) * .5 + .5, .1);
    return vec3(t, t*t, t*t*t);
   }

vec3 clouds(float y) {
    vec3 sky = vec3(.0);
    float s = mix(.6,1., clamp(3.*y-2., 0.,1.));
    return mix(sky, vec3(s), clamp(.5*y,0.,1.));
}



void main() {
    if(u_cloud < 4.99) {
        vec3 p = 8. * vPos/50. + vec3(0., 0., .5* u_time);
        vec3 color = clouds(1. - u_cloud * abs((vPos.x)/(25. + 3. *abs(sin(0.2 * u_time)))) - u_cloud * abs((vPos.y)/(9. + 3. *abs(sin(0.2 * u_time)))) +3.*turbulence(p/3.) );
        gl_FragColor = vec4(.5 * sqrt(color), length(color) - (abs(vPos.y)/25.5 + abs(vPos.x)/25.5));
    }
}
