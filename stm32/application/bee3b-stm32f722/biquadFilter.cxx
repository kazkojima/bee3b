#include "biquadFilter.h"
#include <cmath>

#if 0
extern "C" float Sinf(float x);
extern "C" float Cosf(float x);
#define sinf Sinf
#define cosf Cosf
#endif

static const float float_pi = 3.14159265359f;

// Computes a biquad_t filter on a sample
float biquadFilter::Update (float input)
{
  const float result = m_b0*input + m_d1;
  m_d1 = m_b1*input - m_a1*result + m_d2;
  m_d2 = m_b2*input - m_a2*result;
  return result;
}

float biquadFilter::Reset (float value)
{
  m_d1 = value - (value*m_b0);
  m_d2 = (m_b2 - m_a2)*value;
  return value;
}

biquadLPF::biquadLPF (float filtf, float q, float sampf)
{
  // setup variables
  const float omega = 2*float_pi*filtf/sampf;
  const float sn = sinf (omega);
  const float cs = cosf (omega);
  const float alpha = sn/(2*q);
  float b0, b1, b2, a0, a1, a2;
  // low pass filter
  b0 = (1 - cs)/2;
  b1 = 1 - cs;
  b2 = (1 - cs)/2;
  a0 =  1 + alpha;
  a1 = -2 * cs;
  a2 =  1 - alpha;

  // precompute the coefficients
  m_b0 = b0/a0;
  m_b1 = b1/a0;
  m_b2 = b2/a0;
  m_a1 = a1/a0;
  m_a2 = a2/a0;
  
  // zero initial samples
  m_d1 = m_d2 = 0;
}

biquadNOTCHF::biquadNOTCHF (float filtf, float q, float sampf)
{
  // setup variables
  const float omega = 2*float_pi*filtf/sampf;
  const float sn = sinf (omega);
  const float cs = cosf (omega);
  const float alpha = sn/(2*q);
  float b0, b1, b2, a0, a1, a2;
  // notch filter
  b0 =  1;
  b1 = -2 * cs;
  b2 =  1;
  a0 =  1 + alpha;
  a1 = -2 * cs;
  a2 =  1 - alpha;

  // precompute the coefficients
  m_b0 = b0/a0;
  m_b1 = b1/a0;
  m_b2 = b2/a0;
  m_a1 = a1/a0;
  m_a2 = a2/a0;
  
  // zero initial samples
  m_d1 = m_d2 = 0;
}
