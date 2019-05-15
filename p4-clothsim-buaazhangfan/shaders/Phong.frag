#version 330

uniform vec4 u_color;
uniform vec3 u_cam_pos;
uniform vec3 u_light_pos;
uniform vec3 u_light_intensity;

in vec4 v_position;
in vec4 v_normal;
in vec2 v_uv;

out vec4 out_color;

void main() {
  // YOUR CODE HERE
  // (Placeholder code. You will want to replace it.)
  float ka = 0.1;
  vec3 Ia = vec3(1.0, 1.0, 1.0);
  float kd = 0.5;
  float ks = 1.0;
  float p = 64.0;

  // Diffuse
  float r = distance(u_light_pos, vec3(v_position));
  vec3 diffuse = kd * u_light_intensity / (r * r) * max(0, dot(vec3(v_normal), normalize(u_light_pos - vec3(v_position))));

  // specular
  vec3 v = normalize(u_cam_pos - vec3(v_position));
  vec3 l = normalize(u_light_pos - vec3(v_position));
  vec3 h = (v + l) / length(v + l);
  vec3 specular = ks * u_light_intensity / (r * r) * pow(max(0, dot(vec3(v_normal), h)), p);
  vec3 phong = ka * Ia + diffuse + specular;
  out_color = vec4(phong, 1);
}

