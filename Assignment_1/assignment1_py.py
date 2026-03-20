import numpy as np
import matplotlib.pyplot as plt
from scipy.io import loadmat
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from scipy.interpolate import interp1d
import time

# Carica dati ROBUSTO
print("Caricamento pos_data.mat...")
data = loadmat('pos_data.mat')
pos = data['pos']
if len(pos.shape) == 2 and pos.shape[1] == 1:
    pos = pos[0][0]
    
X = np.squeeze(pos['X']).flatten()
Y = np.squeeze(pos['Y']).flatten()
time_vect = np.squeeze(pos['time_vect']).flatten()
theta = np.squeeze(pos['theta']).flatten()
vx = np.squeeze(pos['vx']).flatten()

print(f"Dati caricati: {len(X)} punti")

e = 1e-5
theta_unwrapped = np.unwrap(theta)

vx_num = np.diff(np.concatenate([[0], X])) / np.diff(np.concatenate([[0], time_vect]))
vy_num = np.diff(np.concatenate([[0], Y])) / np.diff(np.concatenate([[0], time_vect]))
v_tot = np.sqrt(vx_num**2 + vy_num**2)
v_tot[np.abs(v_tot) < e] = 0

dtheta = np.diff(np.concatenate([[0], theta_unwrapped])) / np.diff(np.concatenate([[0], time_vect]))
for ii in range(len(dtheta)-2):
    if np.abs(dtheta[ii] - dtheta[ii+1]) > 2:
        dtheta[ii+1] = 0.5 * (dtheta[ii] + dtheta[ii+2])
dtheta[np.abs(dtheta) < e] = 0

# NO slippage
v_comp = v_tot.copy()
dtheta_smooth = dtheta.copy()

# INTERPOLAZIONE 10Hz
time_10hz = time_vect[::5]
v_int_func = interp1d(time_vect, v_comp, kind='linear', bounds_error=False, fill_value=0)
dtheta_int_func = interp1d(time_vect, dtheta_smooth, kind='linear', bounds_error=False, fill_value=0)
v_int = v_int_func(time_10hz)
dtheta_int = dtheta_int_func(time_10hz)

print(f"Comandi 10Hz interpolati: {len(v_int)} punti (~{len(time_10hz)*0.1/60:.1f} min)")

# Plot
plt.figure(figsize=(12,10))
plt.subplot(2,3,1); plt.plot(X,Y,'k-',lw=3); plt.title('Trajectory'); plt.grid(True); plt.axis('equal')
plt.subplot(2,3,2); plt.plot(time_vect, v_tot,'b-', time_vect, vx,'m-'); plt.title('Speed'); plt.legend(['v_tot','vx']); plt.grid(True)
plt.subplot(2,3,3); plt.plot(time_vect, dtheta,'g-'); plt.title('dtheta'); plt.grid(True)
plt.subplot(2,3,4); plt.plot(time_vect, v_comp,'b-', alpha=0.7, label='original'); plt.plot(time_10hz, v_int,'r-', lw=2, label='interpolated'); plt.title('Velocity'); plt.legend(); plt.grid(True)
plt.subplot(2,3,5); plt.plot(time_vect, dtheta_smooth,'g-', alpha=0.7, label='original'); plt.plot(time_10hz, dtheta_int,'r-', lw=2, label='interpolated'); plt.title('Angular'); plt.legend(); plt.grid(True)
plt.subplot(2,3,6); plt.plot(time_10hz, v_int,'r-', time_10hz, dtheta_int,'g-'); plt.title('10Hz Commands'); plt.legend(['v','w']); plt.grid(True)
plt.tight_layout()
plt.show(block=False)


class VelocityPublisher(Node):
    def __init__(self, v_int, dtheta_int):
        super().__init__('matlab_tb3')
        self.publisher_ = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.v_int = v_int
        self.dtheta_int = dtheta_int

def replay_trajectory(v_int, dtheta_int):
    rclpy.init()
    node = VelocityPublisher(v_int, dtheta_int)
    
    print("Inizio replay INTERPOLATO (~2.6 min)...")
    for i in range(len(v_int)):
        msg = TwistStamped()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.twist.linear.x = float(v_int[i])
        msg.twist.angular.z = float(dtheta_int[i])
        node.publisher_.publish(msg)
        
        progress = (i+1)/len(v_int)*100
        if i % 50 == 0:
            print(f"Progress: {progress:.1f}% (v={v_int[i]:.3f}, w={dtheta_int[i]:.3f})")
        
        time.sleep(0.1)  
    
    # Stop
    msg = TwistStamped()
    msg.twist.linear.x = 0.0
    msg.twist.angular.z = 0.0
    node.publisher_.publish(msg)
    print("Replay INTERPOLATO COMPLETATO!")
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    input("Premi INVIO per ROS2 replay INTERPOLATO...")
    replay_trajectory(v_int, dtheta_int)

