import numpy as np

L1, L2, L3 = 80.0, 188.0, 211.0
q1_lim = [-94.0, 105.0]
q2_lim = [-122.0, 117.0]
q3_lim = [-157.0, 154.0]
step_deg = 5.0

def generate_range_with_zero_and_limits(lim, step_deg):
    neg_part = np.arange(0, lim[0], -step_deg)[::-1]
    pos_part = np.arange(0, lim[1], step_deg)
    return np.radians(np.unique(np.concatenate([[lim[0]], neg_part, pos_part, [lim[1]]])))

q1_range = generate_range_with_zero_and_limits(q1_lim, step_deg)
q2_range = generate_range_with_zero_and_limits(q2_lim, step_deg)
q3_range = generate_range_with_zero_and_limits(q3_lim, step_deg)

x_pts, y_pts, z_pts = [], [], []

for q1 in q1_range:
    for q2 in q2_range:
        for q3 in q3_range:
            r = L1 + L2 * np.cos(q2) + L3 * np.cos(q2 + q3)
            x = r * np.cos(q1)
            y = r * np.sin(q1)
            z = L2 * np.sin(q2) + L3 * np.sin(q2 + q3)
            
            x_pts.append(x)
            y_pts.append(y)
            z_pts.append(z)

x_pts = np.array(x_pts)
y_pts = np.array(y_pts)
z_pts = np.array(z_pts)

print(f"Puntos generados: {len(x_pts)}")
print(f"Max X: {np.max(x_pts):.4f} mm")
print(f"Min X: {np.min(x_pts):.4f} mm")
print(f"Max Y: {np.max(y_pts):.4f} mm")
print(f"Min Y: {np.min(y_pts):.4f} mm")
print(f"Max Z: {np.max(z_pts):.4f} mm")
print(f"Min Z: {np.min(z_pts):.4f} mm")
