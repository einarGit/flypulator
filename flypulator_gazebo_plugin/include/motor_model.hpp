namespace flypulator
{

class MotorModel
{
  public:
    MotorModel():omega_(0)
    {

    }

    double update(double u, double dt)
    {
        double omega_dot = (-omega_ + k_rotor*u) / T_rotor;
        omega_ = omega_ + omega_dot * dt;
        return omega_;
    }

    double getOmega()
    {
        return omega_;
    }

    double getK()
    {
        return k_rotor;
    }

        double getT()
    {
        return T_rotor;
    }

    void reset()
    {
        omega_ = 0;
    }

  private:
    double km = 1.0;
    double Ra = 0.05;
    double b = 1.0;
    double omega_H = 1.0;
    double Im = 1.0;
    double k_rotor = 1; //km / (km * km + 2 * Ra * b * omega_H); // ~0.9
    double T_rotor = 0.1; //Im * Ra / (km * km + 2 * Ra * b * omega_H); // ~0.045
    double omega_ = 0;
};

} // namespace flypulator