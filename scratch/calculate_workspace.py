import numpy as np

# Robot link lengths (mm)
L1 = 80.0
L2 = 188.0
L3 = 211.0

# Joint limits in degrees
q1_lim = [-94.0, 105.0]
q2_lim = [-122.0, 117.0]
q3_lim = [-157.0, 154.0]

# Convert limits to radians
q1_rad = np.radians(q1_lim)
q2_rad = np.radians(q2_lim)
q3_rad = np.radians(q3_lim)

# Discretize joint ranges
N = 150
q1_vals = np.linspace(q1_rad[0], q1_rad[1], N)
q2_vals = np.linspace(q2_rad[0], q2_rad[1], N)
q3_vals = np.linspace(q3_rad[0], q3_rad[1], N)

# Sample a fine grid or generate points to calculate bounding box and volume
# Using a grid in joint space:
Q1, Q2, Q3 = np.meshgrid(q1_vals, q2_vals, q3_vals, indexing='ij')

# Forward Kinematics
R = L1 + L2 * np.cos(Q2) + L3 * np.cos(Q2 + Q3)
X = R * np.cos(Q1)
Y = R * np.sin(Q1)
Z = L2 * np.sin(Q2) + L3 * np.sin(Q2 + Q3)

# Flatten
x_pts = X.flatten()
y_pts = Y.flatten()
z_pts = Z.flatten()

# Bounding box
x_min, x_max = np.min(x_pts), np.max(x_pts)
y_min, y_max = np.min(y_pts), np.max(y_pts)
z_min, z_max = np.min(z_pts), np.max(z_pts)

print(f"X range: [{x_min:.2f}, {x_max:.2f}] mm")
print(f"Y range: [{y_min:.2f}, {y_max:.2f}] mm")
print(f"Z range: [{z_min:.2f}, {z_max:.2f}] mm")

# Let's calculate the volume of the 3D workspace.
# We can do this by using a voxel grid.
# Define a grid in Cartesian space:
grid_res = 5.0 # 5 mm voxels
x_grid = np.arange(x_min - 5, x_max + 5, grid_res)
y_grid = np.arange(y_min - 5, y_max + 5, grid_res)
z_grid = np.arange(z_min - 5, z_max + 5, grid_res)

# To calculate the exact volume, we can integrate over the workspace.
# The volume of a solid of revolution (or sector of revolution):
# Since q1 is decoupled, the workspace is a sector of rotation.
# The angle delta_q1 is q1_max - q1_min.
# The local workspace in (r, z) plane can be computed.
# Let's find the area of the local workspace in (r, z) plane:
r_min, r_max = -400.0, 600.0
z_min_local, z_max_local = -400.0, 400.0
N_rz = 500
r_grid = np.linspace(0, 500, N_rz)
z_grid_rz = np.linspace(-350, 350, N_rz)
R_grid, Z_grid = np.meshgrid(r_grid, z_grid_rz)

# For each (r, z), check if it is reachable
# We can use the check_reachability function
def check_reachability(X_loc, Z_loc, L1, L2, L3, q2_min, q2_max, q3_min, q3_max):
    s_femur = X_loc - L1
    d = Z_loc
    D2 = s_femur**2 + d**2
    D = np.sqrt(D2)
    
    if D > (L2 + L3) or D < np.abs(L2 - L3):
        return False
        
    cos_q3 = (D2 - L2**2 - L3**2) / (2.0 * L2 * L3)
    if cos_q3 < -1.0 or cos_q3 > 1.0:
        return False
    q3_val = np.arccos(cos_q3)
    
    for q3 in [-q3_val, q3_val]:
        if q3 < q3_min or q3 > q3_max:
            continue
            
        alpha1 = np.arctan2(d, s_femur)
        cos_alpha2 = (L2**2 + D2 - L3**2) / (2.0 * L2 * D)
        if cos_alpha2 < -1.0 or cos_alpha2 > 1.0:
            continue
        alpha2 = np.arccos(cos_alpha2)
        
        if q3 < 0:
            q2 = alpha1 + alpha2
        else:
            q2 = alpha1 - alpha2
            
        if q2_min <= q2 <= q2_max:
            return True
    return False

reachable_rz = np.zeros_like(R_grid, dtype=bool)
for i in range(R_grid.shape[0]):
    for j in range(R_grid.shape[1]):
        reachable_rz[i, j] = check_reachability(
            R_grid[i, j], Z_grid[i, j],
            L1, L2, L3,
            q2_rad[0], q2_rad[1], q3_rad[0], q3_rad[1]
        )

# Compute area in r-z plane:
dr = r_grid[1] - r_grid[0]
dz = z_grid_rz[1] - z_grid_rz[0]
area_rz = np.sum(reachable_rz) * dr * dz

# Volume by Pappus's Centroid Theorem or direct integration:
# V = \int \int r * d\theta * dr * dz = \Delta \theta * \int \int r * dr * dz
delta_q1 = q1_rad[1] - q1_rad[0]
volume = delta_q1 * np.sum(R_grid[reachable_rz]) * dr * dz # in mm^3
volume_liters = volume / 1e6 # 1 liter = 1e6 mm^3
print(f"Area local en plano R-Z: {area_rz:.2f} mm^2")
print(f"Volumen del espacio de trabajo 3D: {volume:.2f} mm^3 ({volume_liters:.3f} litros)")
