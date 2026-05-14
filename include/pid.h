#ifndef PID_H
#define PID_H

class PID {
public:
    PID() = default;
    PID(double p_term, double i_term, double d_term);

    double calculate(double new_error);
    void reset();

private:
    double P = 0.0;
    double I = 0.0;
    double D = 0.0;

    double last_error = 0.0;
    double last_iterm = 0.0;
};

#endif
