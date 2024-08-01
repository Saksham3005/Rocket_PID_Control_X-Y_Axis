# Rocket_PID_Control_X-Y_Axis

This is a basic code depicting 2 axis PID control of a rocket with vertical and horizontal thrusters.  
The PID Gains are tuned using the Zeigler-Nichols Method:  

Ku = 0.4  
Tu = 35 ms  
KP = 0.6Ku  
KI = 1.2*Ku/Tu  
KD = 3*Ku*Tu/40  
