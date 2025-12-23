#%%

import numpy as np 
import matplotlib.pyplot as plt
TEMP_RAMP_INTERVAL=20000
X=np.linspace(0,20000,20)

sinusoidalProgress = 0.5 * (1.0 - np.cos(X/TEMP_RAMP_INTERVAL*np.pi))

sinusoidalProgress = 0.5 * (1.0 - np.cos(X/TEMP_RAMP_INTERVAL*np.pi))
plt.plot(X,sinusoidalProgress)
# %%
