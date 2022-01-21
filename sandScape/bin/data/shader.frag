#ifdef GL_ES
precision mediump float;
#endif
uniform sampler2DRect waterTexture;
uniform sampler2DRect particleTexture;
uniform float u_time;
uniform vec2 u_resolution;
uniform vec2 u_mouse;
uniform float u_mode;
uniform float u_width;
uniform float u_length;
uniform float u_height;
uniform int u_season;
uniform int u_waterMode;
uniform float u_snowHeight;
uniform float u_cloud;
uniform float u_specular;

varying vec3 vPos;
varying vec3 vNor;
varying vec3 vDir;
varying float vCur;
varying vec2 u_texcoord;

const vec3 white = vec3(1.,1.,1.);
const vec3 black = vec3(0.,0.,0.);
// color palette for the colorful mist
const vec3 blue = vec3(0.684,0.8297,0.98);
const vec3 yellow = vec3(1.,0.99,0.56);
const vec3 purple = vec3(0.97,0.329,0.9);
const vec3 pink = vec3(0.93,0.418,0.55);
const vec3 green = vec3(0.59,0.96,0.826);
const vec3 nude = vec3(0.96,0.792,0.624);
// color palette for winter
const vec3 w_blue0 = vec3(0.4,0.5,0.7);
const vec3 w_blue1 = vec3(0.4,0.6,1);
const vec3 w_blue2 = vec3(0.8,1,1);
const vec3 w_green0 = vec3(0.7,0.6,0.5);
const vec3 w_green1 = vec3(0.7,0.9,0.8);
const vec3 w_green2 = vec3(0.8,1,0.9);
// color palette for spring
const vec3 s_blue0 = vec3(0.,0.3,0.5);
const vec3 s_blue1 = vec3(0.2,0.7,0.9);
const vec3 s_blue2 = vec3(0.6,0.9,0.95);
const vec3 s_green0 = vec3(0.3,0.4,0.1);
const vec3 s_green1 = vec3(0.6,0.9,0.6);
const vec3 s_green2 = vec3(0.9,1,0.8);
// color palette for summer
const vec3 su_blue0 = vec3(0.,0.3,0.5);
const vec3 su_blue1 = vec3(0.2,0.7,0.9);
const vec3 su_green3 = vec3(0.6,0.9,0.1);
const vec3 su_green0 = vec3(0.2,0.5,0.1);
const vec3 su_green1 = vec3(0.6,0.7,0.2);
const vec3 su_green2 = vec3(0.5,0.8,0.3);
// color palette for fall
const vec3 f_yellow0 = 0.6 * vec3(0.7,0.5,0.1);
const vec3 f_yellow1 = vec3(0.8,0.4,0.1);
const vec3 f_yellow2 = vec3(.9,0.6,0.2);
const vec3 f_green0 = vec3(0.5,0.4,0.2);
const vec3 f_green1 = vec3(.5,0.9,0.3);
const vec3 f_green2 = vec3(1.,1.2,0.1);

// lighting initials
vec3 lightDir = vec3(0.,0.,0.);
vec3 lightColor = vec3(1.,0.94,0.9);

// phong shading algo for the shadow rendering
vec3 phong(vec3 Ldir, vec3 Lrgb, vec3 normal, vec3 diffuse, vec3 specular, float p) {
    vec3 color = vec3(0., 0., 0.);
    float d = dot(Ldir, normal);
    if (d > 0.)
        color += diffuse * d * Lrgb;
    
    vec3 R = 2. * normal * dot(Ldir, normal) - Ldir;
    float s = pow(dot(R, normal),1.);
    if (s > 0.)
        color += specular * pow(s, p) * Lrgb;
    
    return color;
}

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
    // blur boundary
    float alpha = 1.;
    
    if(u_mode == 0.) { // elevation
        vec3 color = vec3(253., 128., 46.)/255.;
        // different color for different height
        color = mix(vec3(253.,198.,104.)/255., color, clamp(pow((vPos.z/u_height + 0.1),10.), 0.,1.));
        color = mix(vec3(211.,231.,125.)/255., color, clamp(pow((vPos.z/u_height + 0.2),10.), 0.,1.));
        color = mix(vec3(168.,233.,106.)/255., color, clamp(pow((vPos.z/u_height + 0.4),10.), 0.,1.));
        color = mix(vec3(171.,233.,235.)/255., color, clamp(pow((vPos.z/u_height + 0.5),10.), 0.,1.));
        color = mix(vec3(55.,231.,238.)/255., color, clamp(pow((vPos.z/u_height + 0.6),10.), 0.,1.));
        color = mix(vec3(75.,220.,253.)/255., color, clamp(pow((vPos.z/u_height + 0.7),10.), 0.,1.));
        color = mix(vec3(75.,194.,253.)/255., color, clamp(pow((vPos.z/u_height + 0.85),10.), 0.,1.));
        color = mix(vec3(6.,182.,243.)/255., color, clamp(pow((vPos.z/u_height + 0.8),10.), 0.,1.));
        gl_FragColor = vec4(color.rgb,1.);
        
    } else if(u_mode == 1.){ // rain <-> elevation transition
        vec3 color = vec3(253., 128., 46.)/255.;
        // different color for different height
        color = mix(vec3(253.,198.,104.)/255., color, clamp(pow((vPos.z/u_height + 0.1),10.), 0.,1.));
        color = mix(vec3(211.,231.,125.)/255., color, clamp(pow((vPos.z/u_height + 0.2),10.), 0.,1.));
        color = mix(vec3(168.,233.,106.)/255., color, clamp(pow((vPos.z/u_height + 0.4),10.), 0.,1.));
        color = mix(vec3(171.,233.,235.)/255., color, clamp(pow((vPos.z/u_height + 0.5),10.), 0.,1.));
        color = mix(vec3(55.,231.,238.)/255., color, clamp(pow((vPos.z/u_height + 0.6),10.), 0.,1.));
        color = mix(vec3(75.,220.,253.)/255., color, clamp(pow((vPos.z/u_height + 0.7),10.), 0.,1.));
        color = mix(vec3(75.,194.,253.)/255., color, clamp(pow((vPos.z/u_height + 0.85),10.), 0.,1.));
        color = mix(vec3(6.,182.,243.)/255., color, clamp(pow((vPos.z/u_height + 0.8),10.), 0.,1.));
        // init water drainage texture
        vec4 water_tex = texture(waterTexture, vec2(u_texcoord.x, u_texcoord.y) * 10.);
        // contour line
        float slope = 1. - abs(dot(normalize(vNor), vec3(0.,0.,1.)));
        float strokeWeight = .3 * slope;
        float countourLineNum = 15.;
        float unit = u_height / countourLineNum;
      
        if(u_waterMode == 2 && length(water_tex.rgb) > 0.8  && water_tex.b <= water_tex.r) {
            water_tex.rgb = color;
        } else if(u_waterMode == 3 && length(water_tex.rgb) > 0.8){
            if(water_tex.b > water_tex.r) water_tex.rgb = color;
        } else if(u_waterMode == 5 && length(water_tex.rgb) > 0.8){
            water_tex.rgb = white;
        }
        
        vec3 avg = vec3((color.r + color.g + color.b)/3.);
        float normCloud = u_cloud/5.;
        float sc = max(normCloud,0.3);
        color = vec3(avg) + sc * (color - avg);
        color *= max(normCloud,0.3);
        gl_FragColor = vec4(color,1.);
        if(vPos.x > - u_width/2 + 1. && vPos.x < u_width/2 - 2. && vPos.y > -u_length/2 + 1. && vPos.y < u_length/2 - 2.) {
            for (int i = 1; i < int(countourLineNum); i ++) {
                float scale = 1.;
                if(vPos.z > i * unit - scale * strokeWeight && vPos.z < i * unit + scale * strokeWeight) {
                    gl_FragColor = vec4(1.5 * white * clamp(normCloud,0.3,1.),1.);
                }
            }
            if(length(water_tex.rgb) > 0.8 && normCloud < 0.5){
                color = mix(water_tex.rgb,color,2. * normCloud);
                gl_FragColor = vec4(color,1.);
            }
        }
        
       
    } else if(u_mode == 2.){ // season
        vec2 coord = gl_FragCoord.xy;
        // winter mesh color
        vec3 low_w_color = mix(w_green0,w_blue0,0.5 + 0.5 * (vPos.x + vPos.y + 50. * noise(0.01 * vPos))/u_length);
        vec3 high_w_color = mix(w_green1,w_blue2,clamp(0.,1.,0.5 + 2 * (vPos.z)/u_length));
        high_w_color = mix(high_w_color,yellow,(noise(20. + 0.05 * vPos.xyz)));
        float ratio = smoothstep(0.,1.,0.2 * noise(0.1 * vPos.xyz) + (vPos.z - 8.)/u_height);
        high_w_color = mix(high_w_color, vec3(1.,1.,1.), ratio);
        vec3 w_color = mix(low_w_color,high_w_color,0.1 + vPos.z/u_height);
        if(vPos.z > u_height - u_snowHeight + noise(20. * vPos)) w_color = min(w_color * 1.2, white);
        // spring mesh color
        vec3 low_s_color = mix(s_green0,s_green1,noise(0.01*vPos) + 0.5 * (vPos.x + vPos.z)/u_length);
        vec3 high_s_color = mix(s_blue2,s_green2,0.5 + 2 * (vPos.y+4.)/u_length);
        low_s_color = mix(low_s_color,pink, clamp(0.,1.,(noise(10. + 0.05 * vPos.xyz))));
        vec3 s_color = mix(low_s_color,high_s_color,-0.2 + vPos.z/u_height);
        vec4 particle_tex = texture(particleTexture, vec2(u_texcoord.x, u_texcoord.y) * 10.);
        if(particle_tex.r > 0.9) s_color = particle_tex.rgb;
        if(vPos.z > u_height - u_snowHeight + noise(20. * vPos)) s_color = min(mix(w_color,s_color,.5 )*1.1, white);
        // summer mesh color
        vec3 low_su_color = mix(su_green0,su_green1,0.1 + (vPos.x)/u_length);
        vec3 high_su_color = mix(su_green2,su_green3,0.5 + 2 * (vPos.y+4.)/u_length);
        low_su_color = mix(low_su_color,green, clamp(0.,1.,(noise(3. + 0.05 * vPos.xyz))));
        vec3 su_color = mix(low_su_color,high_su_color,-0.2 + vPos.z/u_height);
        // fall mesh color
        vec3 low_f_color = mix(f_yellow0,f_green0,0.5 + 0.5*(vPos.y + vPos.x + noise(0.01*vPos))/u_length);
        vec3 high_f_color = mix(f_green1,f_yellow2,0.5 + 2 * (vPos.y+4.)/u_length);
        high_f_color = mix(high_f_color,f_yellow1,0.8 + 0.9 * noise(0.1 * vPos.xyz));
        vec3 f_color = mix(low_f_color,high_f_color,-0.2 + vPos.z/u_height);
        if(particle_tex.r > 0.9) f_color = 1. * vec3(particle_tex.r * 1.3,particle_tex.g,particle_tex.b * 0.5);
        // season auto transition
        //        float n = 0.5 * (-0.4 + .2 * noise(0.06 * vPos));
        //        temp = 0.625 * vPos.x/u_width;
        //        if(temp < 0.01 + n) {
        //            alpha = mix(alpha,0.,10. * (0.01 + n - temp));
        //        } else if (vPos.x/u_width > 0.6 + n ) {
        //            alpha = mix(alpha,0.,10. * ( vPos.x/u_width - 0.6 - n));
        //        }
        //
        //        temp = 0.625 * vPos.y/u_length;
        //        if (temp < 0.01 + n ) {
        //            alpha = mix(alpha,0.,10. * (0.01 + n - temp));
        //        } else if (vPos.y/u_length > 0.6 + n ){
        //            alpha = mix(alpha,0.,10. * (vPos.y/u_length - 0.6 - n));
        //        }
        //        vec3 color = mix(f_color,w_color, 0.5 + 0.5 * sin(0.5 * u_time));
        //        color = mix(color, s_color, 0.5 - 0.5 * cos(0.5 * u_time));
        
        vec3 color = white;
        if(u_season == 0) color = s_color;
        else if(u_season == 1) color = f_color;
        else color = w_color;
        // add noise to the color to make it looks better
        float c = noise(600.0 * vPos);
        color += clamp(c,.0,.05);
        gl_FragColor = vec4(color,alpha);
        // add particles from the OF to the mesh
        vec4 water_tex = texture(waterTexture, vec2(u_texcoord.x, u_texcoord.y) * 10.);
        // snow on the top
        if(u_season == 0 && water_tex.b > 0.7 && vPos.z < min(u_height - u_snowHeight , u_height - 1.)) gl_FragColor = vec4(clamp(color + water_tex.rgb * (u_snowHeight/5.), color, white),alpha);
        
        
    } else if(u_mode == 3.) { // shadow
        float baseColor = 0.5 + u_specular/100.;
        gl_FragColor = vec4(baseColor);
        float s = sin(u_time);
        float c = cos(u_time);
        lightDir.xz = vec2(c,s);
        vec3 N = normalize(vNor);
        vec3 R = 2. * dot(lightDir, N) * N - lightDir;
        vec3 color = phong(lightDir, lightColor, N, gl_FragColor.xyz, 0.1 * gl_FragColor.xyz, 1. );
        gl_FragColor = vec4(color,alpha);
        
    }
}
