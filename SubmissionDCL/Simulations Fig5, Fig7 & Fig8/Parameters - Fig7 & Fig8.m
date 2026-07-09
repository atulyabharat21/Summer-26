% 
% Ra = 0.7; 
% La = 505.5e-6; 
% Kt = 0.031;          
% Kb_rpm = 0.0167;      
% Kb = Kb_rpm * 60/(2*pi);  
% Bf = 4.7e-6;
% Jm = 3.975e-4;

Ra = 0.7; 
La = 505.5e-6; 
Kb_rpm = 0.0167;
Kb = Kb_rpm * 60/(2*pi);   % V*s/rad
Kt = Kb;                    
Bf = 4.7e-6;
Jm = 3.975e-4;

A  = [-Ra/La, -Kb/La; 
       Kt/Jm,  -Bf/Jm];
B  = [1/La; 0];
Bd = [0; 1/Jm];
C  = [1 0; 0 1];      

Aa = [ A,         Bd; 
       0,  0,     0  ]; 

Ba = [ B; 
       0 ];

Ca = [ C, zeros(2,1) ]; 
Da = zeros(2,1);        


w0 = -6920;
poles_obs = [w0, 1.02*w0, 1.04*w0];   
L = place(Aa', Ca', poles_obs)';


Ao = Aa - L*Ca;
Bo = [Ba, L];    
Co = [0 0 1];  
Do =[0 0 0 ]; 


