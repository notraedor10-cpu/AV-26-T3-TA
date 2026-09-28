#pragma once
// Your model of the actuator, reconstructed from the decoded CSVs. This is
// the Part B deliverable, alongside your written notes.
//
// Implement step(): given a commanded velocity and a timestep, return the
// measured output angle. The placeholder below is a bare integrator with
// gain 1 -- NOT the real actuator. Replace it with what the data shows
// (dynamics, gain, any nonlinearity, any lag), or the harness proves nothing.

#include <cmath>

// add whatever state your model needs (velocity, motor-side angle, ...)
    /* whats needed (based on info plugged into claude); K (gradient?), velocity, tao (lag time), and delay (with delay buffer)
    
    parameters for when the delay ends in t,u_commanded, y_commanded: 0.195,15,0.1
    parameters for when the slope ends (step occurs, u_commanded hits 0):3.005,0,54.8
    when the curve begins to plateau (tau, lag time constant):3.245,0,56.1

    given values: 
    u        = 15                       // commanded rate, deg/s
    t_move   = 0.195   y_move = 0.1     // first row where the output moves
    t_stop   = 3.005   y_stop = 54.8    // first row where u = 0
    t_plat   = 3.245   y_plat = 56.1    // output stops changing
    T_hold   = 3.0                      // command held ~0 to 3.0 s (check last u=15 row)

    calculated values (taken from .log file):
    K = y_stop - y_move / (t_stop - t_move)
    K = (54.8 - 0.1) / (3.005 - 0.195) = 19.47 deg/s -->
    K = slope / u = 19.47 / 15 = 1.30

    tau_a ==> after the step occurs:
    after the command stops, velocity decays exponentially and the
    extra distance travelled = slope * tau
    coast  = y_plat - y_stop = 56.1 - 54.8 = 1.3 deg
    tau_A  = coast / slope = 1.3 / 19.47 = 0.067 s

    // cross-check: the sensor stops changing once the remaining coast < 0.05 deg
    // (half the 0.1 deg resolution)
    tau_B  = (t_plat - 3.0) / ln( (slope * 0.07) / 0.05 )
    tau_A  = 0.245 / ln(27) = 0.074 s
    tau therefore can be estimated to be around 0.07 s

    
    */
struct Plant {
    // ---- parameters (tune these against Check 1) ----
    double K   = 1.30;   // gain: actual speed / commanded speed
    double tau = 0.07;   // lag time constant, s
    double gap = 2.3;    // backlash width, deg (confirm with deadband_test)

    // ---- state ----
    double v     = 0.0;          // lagged velocity, deg/s
    double drive = -gap / 2.0;   // motor-side angle; starts on the negative contact edge
    double angle = 0.0;          // true output angle, deg

    // u_cmd : commanded velocity, deg/s
    // dt    : timestep, seconds
    // return: measured output angle, deg
    double step(double u_cmd, double dt) {
        // 1. first-order lag on velocity (exact discretisation, stable for any dt)
        double alpha = 1.0 - std::exp(-dt / tau);
        v += alpha * (K * u_cmd - v);

        // 2. integrate velocity into the motor-side angle
        drive += v * dt;

        // 3. backlash: the output only moves once the drive has crossed the gap
        double half = gap / 2.0;
        if (drive - angle > half)        angle = drive - half;   // pushing forward
        else if (drive - angle < -half)  angle = drive + half;   // pushing backward
        // otherwise: inside the gap, output stays put

        // 4. sensor reads to 0.1 deg
        return std::round(angle / 0.1) * 0.1;
    }

    void reset() {
        v     = 0.0;
        drive = -gap / 2.0;
        angle = 0.0;
    }
};
