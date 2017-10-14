struct PIDState {
  int position;
  int integrator;

  double pGain;
  double iGain;
  double dGain;
};
