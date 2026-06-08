import numpy as np
import scipy.signal as signal
import matplotlib.pyplot as plt
import control as ct

#Input

omega_amp = 187 #initialization 
Tload = 0 

#simulation mode
startTime = 0
numberSamples = 1000
h = 0.0001 #step size
endTime = numberSamples*h
timeVector = np.linspace(startTime,endTime,numberSamples)

s = ct.TransferFunction.s

#Model of plant - estimated using graphs - ref: EE380
G = (306.27) / (1+ 0.1267*s)

#Model of the controller desgined
Gc = (0.327) *(1+ 0.1267*s) / (1 + 0.06*s)

L = G * Gc

Trf = L / (1+ L)

#convert to LTI
num = np.array(Trf.num[0][0], dtype=float)
den = np.array(Trf.den[0][0], dtype=float) 

trf = ct.TransferFunction(num,den)


omega_ref = omega_amp*np.ones((numberSamples,1)) #unit input

#calulate the complete 
t_out, omega =  ct.step_response(trf, T=timeVector)

plt.plot(t_out, omega * omega_amp)
plt.plot(t_out, omega_ref)
plt.grid(True)
plt.xlabel('Time (s)')
plt.ylabel('Speed (rad/s)')
plt.title('Closed Loop Step Response')
plt.show()
