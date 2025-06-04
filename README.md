                                                                                    PID CONTROLLER DESIGN

$IMPLEMENTATION::
gcc pid_controller.c -o pid_controller
./pid_controller




$Explanation

1.Controls a system using a PID algorithm.

2.Modes: Manual/Auto, Local/Remote setpoint, Direct/Reverse action.

3.Adds features: deadband, rate limiting, and output clamping.

4.Saves/loads settings (like Kp, Ki, setpoint) using a config file (simulates EEPROM).

5.Simulates process behavior for testing the controller.

6.Runs a loop to show how the system responds over time.
 
