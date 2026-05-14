#include "pid.h"
#include "config.h"

PID::PID(double p_term, double i_term, double d_term) {
    P = p_term;
    I = i_term;
    D = d_term;
}

double PID::calculate(double new_error) {
    const double pterm = P * new_error;

    // Trapezoidal integration
    last_iterm += I * 0.5 * (new_error + last_error) * DT;

    if (last_iterm > 0.1) {
        last_iterm = 0.1;
    } else if (last_iterm < -0.1) {
        last_iterm = -0.1;
    }

    const double dterm_raw = D * (new_error - last_error) / DT;
    double dterm = dterm_raw;

    if (dterm > 0.12) {
        dterm = 0.12;
    } else if (dterm < -0.12) {
        dterm = -0.12;
    }

    last_error = new_error;

    double pid_out = pterm + dterm + last_iterm;

    if (pid_out > 0.4) {
        pid_out = 0.4;
    } else if (pid_out < -0.4) {
        pid_out = -0.4;
    }

    return pid_out;
}

void PID::reset() {
    last_error = 0.0;
    last_iterm = 0.0;
}
