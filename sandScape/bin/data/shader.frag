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
uniform float u_shadow;
uniform float u_lineDarkness;
uniform float u_mAlpha;
uniform float u_wAlpha;
uniform float u_mBrightness;
uniform float u_mSaturation;

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
//const vec3 f_green1 = vec3(0.6,0.7,0.1);
const vec3 f_green1 = vec3(.5,0.9,0.3);
const vec3 f_green2 = vec3(1.,1.2,0.1);

// lighting initials
vec3 lightDir = vec3(0.,0.,0.);
vec3 lightColor = vec3(1.,0.94,0.9);

// arrow
const float PI = 3.1415927;
const float ARROW_TILE_SIZE = 2;
// How sharp should the arrow head be Used
const float ARROW_HEAD_ANGLE = 40.0 * PI / 180.0;
const float ARROW_HEAD_LENGTH = 1.5;
const float ARROW_SHAFT_THICKNESS = .5;

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

// Computes the center pixel of the tile containing pixel pos
vec2 arrowTileCenterCoord(vec2 pos) {
    return (floor(pos / ARROW_TILE_SIZE) + 0.5) * ARROW_TILE_SIZE;
}

// v = field sampled at tileCenterCoord(p), scaled by the length
// desired in pixels for arrows
// Returns 1.0 where there is an arrow pixel.
float arrow(vec2 p, vec2 v) {
    // Make everything relative to the center, which may be fractional
    //    p -= arrowTileCenterCoord(p);
    //    vec2 offset = ARROW_TILE_SIZE * v / length(v) * ((0.5 * u_time - floor(0.5 * u_time)));
    //    vec2 offset = v * (u_time - floor(u_time));
    v = normalize(v);
    v.x = abs(v.x);
    v.y = abs(v.y);
    if (v.x <= .2 || v.x >= .8) v.x = 0.;
    else v.x = .5;
    if (v.y <= .2 || v.y >= .8) v.y = 0.;
    else v.y = .5;
    vec2 offset = vec2(0.,0.);
    p -= arrowTileCenterCoord(p - offset) + offset;
    //    v -= offset;
    float mag_v = length(v), mag_p = length(p);
    
    if (mag_v >= 0.0 ) {
        // Non-zero velocity case
        vec2 dir_p = p / mag_p, dir_v = v / mag_v;
        
        // We can't draw arrows larger than the tile radius, so clamp magnitude.
        // Enforce a minimum length to help see direction
        mag_v = max(50. * mag_v, 3.6);
        
        // Arrow tip location
        v = dir_v * mag_v;
        
        // Define a 2D implicit surface so that the arrow is antialiased.
        // In each line, the left expression defines a shape and the right controls
        // how quickly it fades in or out.
        
        float dist;
        p += offset;
        v -= offset;
        
        dist = max(
                   // Shaft
                   ARROW_SHAFT_THICKNESS / 4.0 -
                   max(abs(dot(p, vec2(dir_v.y, - dir_v.x))), // Width
                       abs(dot(p, dir_v)) - mag_v + ARROW_HEAD_LENGTH / 2.0), // Length
                   
                   // Arrow head
                   min(0.0,dot(v - p, dir_v) - cos(ARROW_HEAD_ANGLE / 2.0) * length(v - p)) * 2.0 + // Front sides
                   min(0.0, dot(p, dir_v) + ARROW_HEAD_LENGTH - mag_v)); // Back
        
        return clamp(1. + dist, 0., 1.);
    } else {
        // Center of the pixel is always on the arrow
        return max(0.0, 1.2 - mag_p);
    }
}

void main() {
    // blur boundary
    float alpha = 1.;
    
    if(u_mode == 0.) { // posotion correction
        vec3 color = vec3(253., 128., 46.)/255.;
        color = mix(vec3(253.,198.,104.)/255., color, clamp(pow((vPos.z/u_height + 0.1),10.), 0.,1.));
        color = mix(vec3(211.,231.,125.)/255., color, clamp(pow((vPos.z/u_height + 0.2),10.), 0.,1.));
        color = mix(vec3(168.,233.,106.)/255., color, clamp(pow((vPos.z/u_height + 0.4),10.), 0.,1.));
        color = mix(vec3(171.,233.,235.)/255., color, clamp(pow((vPos.z/u_height + 0.5),10.), 0.,1.));
        color = mix(vec3(55.,231.,238.)/255., color, clamp(pow((vPos.z/u_height + 0.6),10.), 0.,1.));
        color = mix(vec3(75.,220.,253.)/255., color, clamp(pow((vPos.z/u_height + 0.7),10.), 0.,1.));
        color = mix(vec3(75.,194.,253.)/255., color, clamp(pow((vPos.z/u_height + 0.85),10.), 0.,1.));
        color = mix(vec3(6.,182.,243.)/255., color, clamp(pow((vPos.z/u_height + 0.8),10.), 0.,1.));
        gl_FragColor = vec4(color.rgb,1.);
        
    } else if(u_mode == 1.){ // geometry
        vec3 color = vec3(253., 128., 46.)/255.;
        
        color = mix(vec3(253.,198.,104.)/255., color, clamp(pow((vPos.z/u_height + 0.1),10.), 0.,1.));
        color = mix(vec3(211.,231.,125.)/255., color, clamp(pow((vPos.z/u_height + 0.2),10.), 0.,1.));
        color = mix(vec3(168.,233.,106.)/255., color, clamp(pow((vPos.z/u_height + 0.4),10.), 0.,1.));
        color = mix(vec3(171.,233.,235.)/255., color, clamp(pow((vPos.z/u_height + 0.5),10.), 0.,1.));
        color = mix(vec3(55.,231.,238.)/255., color, clamp(pow((vPos.z/u_height + 0.6),10.), 0.,1.));
        color = mix(vec3(75.,220.,253.)/255., color, clamp(pow((vPos.z/u_height + 0.7),10.), 0.,1.));
        color = mix(vec3(75.,194.,253.)/255., color, clamp(pow((vPos.z/u_height + 0.85),10.), 0.,1.));
        color = mix(vec3(6.,182.,243.)/255., color, clamp(pow((vPos.z/u_height + 0.8),10.), 0.,1.));
        //        if(u_waterMode == 1){
        //            //            if(arrow(vPos.xy, vDir.xy) == 1.){
        //            //                color = vec3(0.0,0.0,0.);
        //            //                color += vec3(0.4,1.,1.0) * (1. - abs(sin(PI * (0.1 * u_time - floor(0.1 * u_time)) + 0.5 * PI * vPos.z/u_height)));
        //            //            }
        //            lightDir.xz = vec2(1,1);
        //            vec3 N = normalize(vNor);
        //            vec3 R = 2. * dot(lightDir, N) * N - lightDir;
        //            vec3 shadowColor = min(phong(lightDir, lightColor, N, 0.5 * color, 0.1 * color, u_specular),white);
        //            color = mix(color,shadowColor, u_shadow);
        //
        //            gl_FragColor = vec4(color,1.);
        //
        //        }else
        vec4 water_tex = texture(waterTexture, vec2(u_texcoord.x, u_texcoord.y) * 10.);
        // contour line
        float stiffness = 1. - abs(dot(normalize(vNor), vec3(0.,0.,1.)));
        //        float stiffness = .9;
        float strokeWeight = .3 * stiffness;
        float unit = u_height / 15.;
      
        if(u_waterMode == 2 && length(water_tex.rgb) > 0.8  && water_tex.b <= water_tex.r) {
            water_tex.rgb = color;
        } else if(u_waterMode == 3 && length(water_tex.rgb) > 0.8){
            if(water_tex.b > water_tex.r) water_tex.rgb = color;
        } else if(u_waterMode == 5 && length(water_tex.rgb) > 0.8){
            water_tex.rgb = white;
        }
        
        vec3 avg = vec3((color.r + color.g + color.b)/3.);
        float normCloud = u_cloud/5.;
        //            float sc = u_mSaturation;
        float sc = max(normCloud,0.3);
        color = vec3(avg) + sc * (color - avg);
        //            color *= u_mBrightness;
        color *= max(normCloud,0.3);
        gl_FragColor = vec4(color,u_mAlpha);
        if(vPos.x > - u_width/2 + 1. && vPos.x < u_width/2 - 2. && vPos.y > -u_length/2 + 1. && vPos.y < u_length/2 - 2.) {
            for (int i = 1; i < 15; i ++) {
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
        
        vec3 low_w_color = mix(w_green0,w_blue0,0.5 + 0.5 * (vPos.x + vPos.y + 50. * noise(0.01 * vPos))/u_length);
        vec3 high_w_color = mix(w_green1,w_blue2,clamp(0.,1.,0.5 + 2 * (vPos.z)/u_length));
        high_w_color = mix(high_w_color,yellow,(noise(20. + 0.05 * vPos.xyz)));
        float ratio = smoothstep(0.,1.,0.2 * noise(0.1 * vPos.xyz) + (vPos.z - 8.)/u_height);
        high_w_color = mix(high_w_color, vec3(1.,1.,1.), ratio);
        vec3 w_color = mix(low_w_color,high_w_color,0.1 + vPos.z/u_height);
        if(vPos.z > u_height - u_snowHeight + noise(20. * vPos)) w_color = min(w_color * 1.2, white);
        
        vec3 low_s_color = mix(s_green0,s_green1,noise(0.01*vPos) + 0.5 * (vPos.x + vPos.z)/u_length);
        vec3 high_s_color = mix(s_blue2,s_green2,0.5 + 2 * (vPos.y+4.)/u_length);
        low_s_color = mix(low_s_color,pink, clamp(0.,1.,(noise(10. + 0.05 * vPos.xyz))));
        vec3 s_color = mix(low_s_color,high_s_color,-0.2 + vPos.z/u_height);
        vec4 particle_tex = texture(particleTexture, vec2(u_texcoord.x, u_texcoord.y) * 10.);
        if(particle_tex.r > 0.9) s_color = particle_tex.rgb;
        if(vPos.z > u_height - u_snowHeight + noise(20. * vPos)) s_color = min(mix(w_color,s_color,.5 )*1.1, white);
        
        vec3 low_su_color = mix(su_green0,su_green1,0.1 + (vPos.x)/u_length);
        vec3 high_su_color = mix(su_green2,su_green3,0.5 + 2 * (vPos.y+4.)/u_length);
        low_su_color = mix(low_su_color,green, clamp(0.,1.,(noise(3. + 0.05 * vPos.xyz))));
        vec3 su_color = mix(low_su_color,high_su_color,-0.2 + vPos.z/u_height);
        
        vec3 low_f_color = mix(f_yellow0,f_green0,0.5 + 0.5*(vPos.y + vPos.x + noise(0.01*vPos))/u_length);
        vec3 high_f_color = mix(f_green1,f_yellow2,0.5 + 2 * (vPos.y+4.)/u_length);
        high_f_color = mix(high_f_color,f_yellow1,0.8 + 0.9 * noise(0.1 * vPos.xyz));
        vec3 f_color = mix(low_f_color,high_f_color,-0.2 + vPos.z/u_height);
        if(particle_tex.r > 0.9) f_color = 1. * vec3(particle_tex.r * 1.3,particle_tex.g,particle_tex.b * 0.5);
        
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
        
        float c = noise(600.0 * vPos);
        color += clamp(c,.0,.05);
        gl_FragColor = vec4(color,alpha);
        vec4 water_tex = texture(waterTexture, vec2(u_texcoord.x, u_texcoord.y) * 10.);
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
