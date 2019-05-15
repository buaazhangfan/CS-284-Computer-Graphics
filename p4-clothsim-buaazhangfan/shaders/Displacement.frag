#version 330

uniform vec3 u_cam_pos;
uniform vec3 u_light_pos;
uniform vec3 u_light_intensity;

uniform vec4 u_color;

uniform sampler2D u_texture_3;
uniform vec2 u_texture_3_size;

uniform float u_normal_scaling;
uniform float u_height_scaling;

in vec4 v_position;
in vec4 v_normal;
in vec4 v_tangent;
in vec2 v_uv;

out vec4 out_color;

float h(vec2 uv) {
  // You may want to use this helper function...
  return texture(u_texture_3, uv)[0];
}

void main() {
  // YOUR CODE HERE
  
  // (Placeholder code. You will want to replace it.)
  vec3 n = vec3(v_normal);
  vec3 t = vec3(v_tangent);
  vec3 b = cross(n, t);
  mat3 TBD = mat3(t, b, n);

  vec2 uv_w = vec2(v_uv[0] + 1 / u_texture_3_size[0], v_uv[1]);
  vec2 uv_h = vec2(v_uv[0], v_uv[1] + 1 / u_texture_3_size[1]);

  float du = (h(uv_w) - h(v_uv)) * u_normal_scaling * u_height_scaling;
  float dv = (h(uv_h) - h(v_uv)) * u_normal_scaling * u_height_scaling;

  vec3 n0 = vec3(-du, -dv, 1);
  vec3 nd = TBD * n0;


  float ka = 0.1;
  vec3 Ia = vec3(1.0, 1.0, 1.0);
  float kd = 0.5;
  float ks = 1.0;
  float p = 64.0;

  // Diffuse
  float r = distance(u_light_pos, vec3(v_position));
  vec3 diffuse = kd * u_light_intensity / (r * r) * max(0, dot(vec3(nd), normalize(u_light_pos - vec3(v_position))));

  // specular
  vec3 v = normalize(u_cam_pos - vec3(v_position));
  vec3 l = normalize(u_light_pos - vec3(v_position));
  vec3 h = (v + l) / length(v + l);
  vec3 specular = ks * u_light_intensity / (r * r) * pow(max(0, dot(vec3(nd), h)), p);

  vec3 phong = ka * Ia + diffuse + specular;
  out_color = vec4(phong, 1);


}

