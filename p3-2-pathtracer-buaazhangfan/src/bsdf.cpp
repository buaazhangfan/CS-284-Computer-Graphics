#include "bsdf.h"

#include <iostream>
#include <algorithm>
#include <utility>

using std::min;
using std::max;
using std::swap;

namespace CGL {

void make_coord_space(Matrix3x3& o2w, const Vector3D& n) {
  Vector3D z = Vector3D(n.x, n.y, n.z);
  Vector3D h = z;
  if (fabs(h.x) <= fabs(h.y) && fabs(h.x) <= fabs(h.z)) h.x = 1.0;
  else if (fabs(h.y) <= fabs(h.x) && fabs(h.y) <= fabs(h.z)) h.y = 1.0;
  else h.z = 1.0;

  z.normalize();
  Vector3D y = cross(h, z);
  y.normalize();
  Vector3D x = cross(z, y);
  x.normalize();

  o2w[0] = x;
  o2w[1] = y;
  o2w[2] = z;
}

// Mirror BSDF //

Spectrum MirrorBSDF::f(const Vector3D& wo, const Vector3D& wi) {
  return Spectrum();
}

Spectrum MirrorBSDF::sample_f(const Vector3D& wo, Vector3D* wi, float* pdf) {
  // TODO: 1.2
  // Using BSDF::reflect(), implement sample_f for a mirror surface
  *pdf = 1.0;
  reflect(wo, wi);

  return reflectance / abs_cos_theta(*wi);
  // return Spectrum();
}

// Microfacet BSDF //

double MicrofacetBSDF::G(const Vector3D& wo, const Vector3D& wi) {
    return 1.0 / (1.0 + Lambda(wi) + Lambda(wo));
}

double MicrofacetBSDF::D(const Vector3D& h) {
  // TODO: 2.2
  // Compute Beckmann normal distribution function (NDF) here.
  // You will need the roughness alpha.
  Vector3D n(0, 0, 1);
  double tan_theta, numerator, denominator;
  tan_theta = sin_theta(h) / cos_theta(h);
  numerator = exp((-1.0 * pow(tan_theta, 2)) / pow(alpha, 2));
  denominator = M_PI * pow(alpha, 2) * pow(cos_theta(h), 4);
  return numerator / denominator;
  return std::pow(cos_theta(h), 100.0);;
}

Spectrum MicrofacetBSDF::F(const Vector3D& wi) {
  // TODO: 2.3
  // Compute Fresnel term for reflection on dielectric-conductor interface.
  // You will need both eta and etaK, both of which are Spectrum.
  double cos_i = cos_theta(wi);
  Spectrum Rs, Rp;
  Spectrum eta_k = eta * eta + k * k;
  Rs = (eta_k - 2 * eta * cos_i + pow(cos_i, 2)) / (eta_k + 2 * eta * cos_i + pow(cos_i, 2));
  Rp = (eta_k * pow(cos_i, 2) - 2 * eta * cos_i + 1) / (eta_k * pow(cos_i, 2) + 2 * eta * cos_i + 1);

  return (Rs + Rp) / 2.0;
  return Spectrum();
}

Spectrum MicrofacetBSDF::f(const Vector3D& wo, const Vector3D& wi) {
  // TODO: 2.1
  // Implement microfacet model here

  Vector3D n(0.0, 0.0, 1.0);
  if ((dot(n, wo) > 0) && (dot(n, wi) > 0))
  {
      Vector3D h = wi + wo;
      h.normalize();
      return (F(wi) * G(wo, wi) * D(h)) / (4.0 * dot(n, wo) * dot(n, wi));
  }
  else
    return Spectrum();

}

Spectrum MicrofacetBSDF::sample_f(const Vector3D& wo, Vector3D* wi, float* pdf) {
  // TODO: 2.4
  // *Importance* sample Beckmann normal distribution function (NDF) here.
  // Note: You should fill in the sampled direction *wi and the corresponding *pdf,
  //       and return the sampled BRDF value.

  //cosine hemisphere sampling
  /*
  *wi = cosineHemisphereSampler.get_sample(pdf); //placeholder
  return MicrofacetBSDF::f(wo, *wi);
  */

  // importance sampling


  Vector2D r = sampler.get_sample();
  double r1 = r.x;
  double r2 = r.y;
  double theta = atan(sqrt(-1.0 * alpha * alpha * log(1.0 - r1)));
  double phi = 2 * M_PI * r2;
  double h_x = sin(theta) * cos(phi);
  double h_y = sin(theta) * sin(phi);
  double h_z = cos(theta);

  Vector3D h = Vector3D(h_x, h_y, h_z);
  if (h.z <= 0 || dot(wo, h) <= 0)
  {
    *pdf = float(0);
    return Spectrum();
  }

  *wi = -wo + 2 * dot(wo, h) * h;
  if (wi->z <= 0 || dot(*wi, h) <= 0 )
  {
    *pdf = float(0);
    return Spectrum();
  }

  double prob_theta = ((2 * sin(theta)) / (alpha * alpha * pow(cos(theta), 3))) * exp((-1.0 * tan(theta) * tan(theta)) / (alpha * alpha));
  double prob_phi = 1.0 / (2.0 * M_PI);
  double prob_h = (prob_theta * prob_phi) / sin(theta);
  *pdf = prob_h / (4.0 * dot(*wi, h));

  if (*pdf < 0)
    {
      *pdf = float(0);
      return Spectrum();
    }


  return MicrofacetBSDF::f(wo, *wi);

}

// Refraction BSDF //

Spectrum RefractionBSDF::f(const Vector3D& wo, const Vector3D& wi) {
  return Spectrum();
}

Spectrum RefractionBSDF::sample_f(const Vector3D& wo, Vector3D* wi, float* pdf) {
  return Spectrum();
}

// Glass BSDF //

Spectrum GlassBSDF::f(const Vector3D& wo, const Vector3D& wi) {
  return Spectrum();
}

Spectrum GlassBSDF::sample_f(const Vector3D& wo, Vector3D* wi, float* pdf) {

  // TODO: 1.4
  // Compute Fresnel coefficient and either reflect or refract based on it.
  if (refract(wo, wi, ior) == false)
  {
      reflect(wo, wi);
      *pdf = 1;
      return reflectance / abs_cos_theta(*wi);
  }

  else
  {
      double ni = 1.0;
      double no = ior;
      double R = pow((ni - no) / (ni + no), 2);

      if (coin_flip(R))
      {
          reflect(wo, wi);
          *pdf = R;
          return R * reflectance / abs_cos_theta(*wi);
      }
      else
      {
          refract(wo, wi, ior);
          *pdf = 1 - R;
          double eta = 1.0;

          eta = wo.z >= 0 ? 1.0 / ior : ior;

          return (1 - R) * transmittance * pow(eta, 2) / abs_cos_theta(*wi);
      }
  }

}

void BSDF::reflect(const Vector3D& wo, Vector3D* wi) {

  // TODO: 1.1
  // Implement reflection of wo about normal (0,0,1) and store result in wi.
  wi -> x = - wo.x;
  wi -> y = - wo.y;
  wi -> z = wo.z;

}

bool BSDF::refract(const Vector3D& wo, Vector3D* wi, float ior) {

  // TODO: 1.3
  // Use Snell's Law to refract wo surface and store result ray in wi.
  // Return false if refraction does not occur due to total internal reflection
  // and true otherwise. When dot(wo,n) is positive, then wo corresponds to a
  // ray entering the surface through vacuum.
  double ita = 0.0;
  if (wo.z >= 0)
  {
      // entering
      ita = 1.0 / ior;
  }

  else
  {
      // exiting
      ita = ior;
  }

  double cos_theta_phi_square = 1 - pow(ita, 2) * (1 - cos_theta(wo) * cos_theta(wo));
  if (cos_theta_phi_square < 0)
      return false;

  wi -> x = - wo.x * ita;
  wi -> y = - wo.y * ita;

  if (wo.z >= 0)
      wi -> z = - sqrt(cos_theta_phi_square);
  else
      wi -> z = sqrt(cos_theta_phi_square);

  wi->normalize();

  return true;
}

// Emission BSDF //

Spectrum EmissionBSDF::f(const Vector3D& wo, const Vector3D& wi) {
  return Spectrum();
}

Spectrum EmissionBSDF::sample_f(const Vector3D& wo, Vector3D* wi, float* pdf) {
  *pdf = 1.0 / PI;
  *wi  = sampler.get_sample(pdf);
  return Spectrum();
}

} // namespace CGL
