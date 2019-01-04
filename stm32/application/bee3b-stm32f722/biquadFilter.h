class biquadFilter
{
public:
  float Update (float input);
  float Reset (float value);
protected:
  // precompute the coefficients
  float m_b0 = 1.0f;
  float m_b1 = 0.0f;
  float m_b2 = 0.0f;
  float m_a1 = 0.0f;
  float m_a2 = 0.0f;
  float m_d1 = 0.0f;
  float m_d2 = 0.0f;
};

class biquadLPF
  :public biquadFilter
{
public:
  biquadLPF (float filtf, float q, float sampf);
private:
  // None ATM
};

class biquadNOTCHF
  :public biquadFilter
{
public:
  biquadNOTCHF (float filtf, float q, float sampf);
private:
  // None ATM
};
