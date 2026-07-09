import pandas as pd
import matplotlib.pyplot as plt


file_path = 'motor_data_normal.csv'
df = pd.read_csv(file_path)


start_time = df['t_us'].iloc[0]
df['t_s'] = (df['t_us'] - start_time) / 1e6


fig, axs = plt.subplots(4, 1, figsize=(10, 12), sharex=True)
fig.suptitle('Motor System Response Over Time', fontsize=16)


axs[0].plot(df['t_s'], df['current_A'], color='tab:blue', linewidth=1.5)
axs[0].plot(df['t_s'], df['filtered_A'], color='tab:red', linewidth=1.5)
axs[0].set_ylabel('Current (A)', fontweight='bold')
axs[0].grid(True, linestyle='--', alpha=0.7)
axs[0].legend(['Raw', 'Filtered'])


axs[1].plot(df['t_s'], df['pwmCmd'], color='tab:orange', linewidth=1.5)
axs[1].set_ylabel('PWM Command', fontweight='bold')
axs[1].grid(True, linestyle='--', alpha=0.7)


axs[2].plot(df['t_s'], df['encCount'], color='tab:green', linewidth=1.5)
axs[2].set_ylabel('Encoder Count (rads)', fontweight='bold')
axs[2].grid(True, linestyle='--', alpha=0.7)


axs[3].plot(df['t_s'], df['rpm'], color='tab:red', linewidth=1.5)
axs[3].set_ylabel('RPM', fontweight='bold')
axs[3].set_xlabel('Elapsed Time (Seconds)', fontweight='bold')
axs[3].grid(True, linestyle='--', alpha=0.7)


plt.tight_layout()
plt.subplots_adjust(top=0.95) 

plt.show()