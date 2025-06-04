#include <stdio.h>
#include <stdbool.h>
#include <math.h>

// PID Constants
float Kp = 2.0, Ki = 0.5, Kd = 1.0;

// Control variables
float output = 0.0, prev_error = 0.0, integral = 0.0;

// Limits
float max_output = 100.0, min_output = 0.0;
float max_change_per_cycle = 10.0; // For rate limiting

// Deadband
float deadband = 0.5;

// Modes
bool isManual = false;
bool isRemote = false;
bool isDirect = true;

// Setpoints and manual mode
float manual_output = 0.0;
float local_setpoint = 50.0;
float remote_setpoint = 60.0;

// Clamp helper
float clamp(float val, float min, float max) {
    if (val > max) return max;
    if (val < min) return min;
    return val;
}

// Reset PID state
void reset_pid() {
    integral = 0.0;
    prev_error = 0.0;
    output = 0.0;
}

// PID function with deadband and rate limiting
float PID(float input) {
    float sp = isRemote ? remote_setpoint : local_setpoint;
    float error = (isDirect ? 1 : -1) * (sp - input);

    // Apply deadband
    if (fabs(error) < deadband) error = 0.0;

    // Anti-windup
    if ((output < max_output && output > min_output) || (error * integral < 0))
        integral += error;

    float derivative = error - prev_error;
    prev_error = error;

    float raw_output = Kp * error + Ki * integral + Kd * derivative;

    // Rate limiting
    float delta = raw_output - output;
    if (fabs(delta) > max_change_per_cycle) {
        delta = (delta > 0 ? 1 : -1) * max_change_per_cycle;
        raw_output = output + delta;
    }

    output = clamp(raw_output, min_output, max_output);
    return output;
}

// Controller wrapper
float controller(float input) {
    return isManual ? manual_output : PID(input);
}

// Toggles
void auto_manual_toggle(bool toManual) {
    if (toManual && !isManual) manual_output = output;
    if (!toManual) reset_pid();
    isManual = toManual;
}

// Setters
void set_local_setpoint(float sp) { local_setpoint = sp; }
void set_remote_setpoint(float sp) { remote_setpoint = sp; }
void set_manual_output(float mo) { manual_output = clamp(mo, min_output, max_output); }
void set_PID_constants(float p, float i, float d) { Kp = p; Ki = i; Kd = d; }
void set_output_limits(float min, float max) { min_output = min; max_output = max; }
void set_mode_remote(bool remote) { isRemote = remote; }
void set_control_direction(bool direct) { isDirect = direct; }
void set_deadband(float db) { deadband = db; }
void set_rate_limit(float rate) { max_change_per_cycle = rate; }

// Simulate process
float simulate_process(float pv, float control_signal) {
    float gain = 0.1;
    return pv + gain * (control_signal - pv);
}

// Simulation
void run_simulation() {
    float process_var = 40.0;

    printf("Simulation with deadband=%.2f, rate_limit=%.2f\n", deadband, max_change_per_cycle);

    for (int i = 0; i < 20; i++) {
        float ctrl = controller(process_var);
        process_var = simulate_process(process_var, ctrl);

        printf("Time %2d | PV: %.2f | OUT: %.2f | SP: %.2f\n",
               i, process_var, ctrl, isRemote ? remote_setpoint : local_setpoint);
    }
}

// Main demo
int main() {
    set_output_limits(0, 100);
    set_local_setpoint(55);
    set_deadband(0.5);
    set_rate_limit(5.0);  // Slow changes
    set_PID_constants(2.0, 0.6, 1.2);
    set_control_direction(true);
    set_mode_remote(false);

    auto_manual_toggle(false);  // Auto mode
    run_simulation();

    return 0;
}
