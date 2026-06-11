import numpy as np
import matplotlib.pyplot as plt

# Estilo académico premium (Times New Roman / DejaVu Serif)
plt.rcParams.update({
    'font.family': 'serif',
    'font.serif': ['Times New Roman', 'DejaVu Serif'],
    'font.size': 10,
    'axes.labelsize': 10,
    'axes.titlesize': 11,
    'xtick.labelsize': 9,
    'ytick.labelsize': 9,
    'figure.titlesize': 12,
    'legend.fontsize': 9,
    'savefig.dpi': 300
})

# Parámetros mecánicos del robot (de Leg.h)
L1 = 80.0    # Coxa (mm)
L2 = 188.0   # Fémur (mm)
L3 = 211.0   # Tibia (mm)
R_HIP = 150.0 # Radio del chasis a las caderas (mm)

# Límites articulares de la pata (de Leg.cpp/Leg.h)
q1_min_deg, q1_max_deg = -90.0, 90.0
q2_min_deg, q2_max_deg = -100.0, 100.0
q3_min_deg, q3_max_deg = -150.0, 30.0 # para knee_up = True

# Ángulos de montaje de las caderas respecto al chasis
thetas = np.array([np.pi/4.0, -np.pi/4.0, -3.0*np.pi/4.0, 3.0*np.pi/4.0]) # Patas 1, 2, 3, 4

# Pose nominal de diseño de cada pata para calcular apoyos nominales
q1_nom = 0.0
q2_nom = np.radians(20.0)
q3_nom = np.radians(-100.0)

# Posición del pie en el sistema local de cada pata en pose nominal
r_local_nom = L1 + L2 * np.cos(q2_nom) + L3 * np.cos(q2_nom + q3_nom)
z_local_nom = L2 * np.sin(q2_nom) + L3 * np.sin(q2_nom + q3_nom)

# Posiciones globales fijas de los pies (apoyos nominales en el mundo con chasis en (0,0,0))
feet_world = []
for th in thetas:
    x_foot = (R_HIP + r_local_nom) * np.cos(th)
    y_foot = (R_HIP + r_local_nom) * np.sin(th)
    z_foot = z_local_nom
    feet_world.append([x_foot, y_foot, z_foot])
feet_world = np.array(feet_world) # shape: (4, 3)

def check_leg_ik(x_l, y_l, z_l):
    """
    Verifica la cinemática inversa local de una pata y devuelve si es alcanzable
    junto con su manipulabilidad de Yoshikawa.
    """
    # q1 (yaw)
    q1 = np.arctan2(y_l, x_l)
    q1_deg = np.degrees(q1)
    if q1_deg < q1_min_deg or q1_deg > q1_max_deg:
        return False, np.nan
        
    s = np.sqrt(x_l**2 + y_l**2)
    s_femur = s - L1
    d = z_l
    
    D2 = s_femur**2 + d**2
    D = np.sqrt(D2)
    
    # Alcance básico
    if D > (L2 + L3) or D < np.abs(L2 - L3):
        return False, np.nan
        
    # q3
    cos_q3 = (D2 - L2**2 - L3**2) / (2.0 * L2 * L3)
    if cos_q3 < -1.0 or cos_q3 > 1.0:
        return False, np.nan
    q3_val = np.arccos(cos_q3)
    q3 = -q3_val # knee_up = True
    q3_deg = np.degrees(q3)
    if q3_deg < q3_min_deg or q3_deg > q3_max_deg:
        return False, np.nan
        
    # q2
    alpha1 = np.arctan2(d, s_femur)
    cos_alpha2 = (L2**2 + D2 - L3**2) / (2.0 * L2 * D)
    if cos_alpha2 < -1.0 or cos_alpha2 > 1.0:
        return False, np.nan
    alpha2 = np.arccos(cos_alpha2)
    q2 = alpha1 + alpha2 # knee_up = True
    q2_deg = np.degrees(q2)
    if q2_deg < q2_min_deg or q2_deg > q2_max_deg:
        return False, np.nan
        
    # Manipulabilidad de Yoshikawa
    w = np.sin(q3_val)
    return True, w

def evaluate_chassis_pose(x_b, y_b, z_b):
    """
    Verifica si una pose (x_b, y_b, z_b) del chasis es alcanzable por las 4 patas
    y devuelve la manipulabilidad global (mínima entre las 4 patas).
    """
    w_legs = []
    for i in range(4):
        th = thetas[i]
        # Posición del pie respecto al chasis
        dx = feet_world[i, 0] - x_b
        dy = feet_world[i, 1] - y_b
        dz = feet_world[i, 2] - z_b
        
        # Rotar al sistema local de la pata i
        x_l = dx * np.cos(th) + dy * np.sin(th)
        y_l = -dx * np.sin(th) + dy * np.cos(th)
        z_l = dz
        
        reachable, w = check_leg_ik(x_l, y_l, z_l)
        if not reachable:
            return 0, np.nan
        w_legs.append(w)
        
    # Manipulabilidad global del chasis = la mínima de las 4 patas (peor caso)
    return 1, np.min(w_legs)

def apply_styling(ax):
    ax.spines['top'].set_visible(False)
    ax.spines['right'].set_visible(False)
    ax.spines['left'].set_color('#cbd5e1')
    ax.spines['bottom'].set_color('#cbd5e1')
    ax.grid(True, which='both', linestyle=':', linewidth=0.5, color='#cbd5e1')

def main():
    # 1. Malla para el plano Horizontal X-Y (con Z_b = 0)
    grid_size = 200
    xy_range = np.linspace(-250, 250, grid_size)
    X_xy, Y_xy = np.meshgrid(xy_range, xy_range)
    
    reachable_xy = np.zeros_like(X_xy)
    manip_xy = np.full_like(X_xy, np.nan)
    
    for i in range(grid_size):
        for j in range(grid_size):
            reach, w = evaluate_chassis_pose(X_xy[i, j], Y_xy[i, j], 0.0)
            reachable_xy[i, j] = reach
            if reach:
                manip_xy[i, j] = w
                
    # 2. Malla para el plano Sagital X-Z (con Y_b = 0)
    xz_range_x = np.linspace(-250, 250, grid_size)
    xz_range_z = np.linspace(-200, 200, grid_size)
    X_xz, Z_xz = np.meshgrid(xz_range_x, xz_range_z)
    
    reachable_xz = np.zeros_like(X_xz)
    manip_xz = np.full_like(X_xz, np.nan)
    
    for i in range(grid_size):
        for j in range(grid_size):
            reach, w = evaluate_chassis_pose(X_xz[i, j], 0.0, Z_xz[i, j])
            reachable_xz[i, j] = reach
            if reach:
                manip_xz[i, j] = w

    # --- Crear Figura de 2 Subplots ---
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6))
    fig.patch.set_facecolor('white')
    
    # ------------------ SUBPLOT 1: PLANO X-Y ------------------
    ax1.set_facecolor('white')
    apply_styling(ax1)
    
    # Rellenar con manipulabilidad
    cnt1 = ax1.contourf(X_xy, Y_xy, manip_xy, levels=np.linspace(0, 1.0, 11), cmap='viridis', alpha=0.85, zorder=1)
    # Contorno exterior del espacio de trabajo
    ax1.contour(X_xy, Y_xy, reachable_xy, levels=[0.5], colors=['#0f172a'], linewidths=1.5, zorder=2)
    
    # Dibujar apoyos fijos
    ax1.scatter(feet_world[:, 0], feet_world[:, 1], color='#ef4444', marker='s', s=45, label='Apoyos de Pies Fijos', zorder=5, edgecolors='black')
    
    # Dibujar contorno de chasis nominal (círculo)
    theta_circle = np.linspace(0, 2*np.pi, 100)
    ax1.plot(R_HIP * np.cos(theta_circle), R_HIP * np.sin(theta_circle), color='#64748b', linestyle='--', lw=1.2, label='Chasis Nominal (Cuerpo)', zorder=3)
    
    # Dibujar líneas que conectan el origen con las caderas
    for th in thetas:
        ax1.plot([0, R_HIP * np.cos(th)], [0, R_HIP * np.sin(th)], color='#94a3b8', linestyle=':', lw=1.0, zorder=3)
        
    ax1.set_xlabel('Desplazamiento Horizontal $X_b$ (mm)', labelpad=6)
    ax1.set_ylabel('Desplazamiento Lateral $Y_b$ (mm)', labelpad=6)
    ax1.set_title('(a) Espacio de Trabajo Horizontal (Plano $X_b$-$Y_b$ a $Z_b = 0$)', fontweight='bold', pad=12)
    ax1.legend(loc='lower left', frameon=True, facecolor='white', edgecolor='#cbd5e1')
    ax1.axis('equal')
    ax1.set_xlim(-240, 240)
    ax1.set_ylim(-240, 240)
    
    # ------------------ SUBPLOT 2: PLANO X-Z ------------------
    ax2.set_facecolor('white')
    apply_styling(ax2)
    
    # Rellenar con manipulabilidad
    cnt2 = ax2.contourf(X_xz, Z_xz, manip_xz, levels=np.linspace(0, 1.0, 11), cmap='viridis', alpha=0.85, zorder=1)
    # Contorno exterior
    ax2.contour(X_xz, Z_xz, reachable_xz, levels=[0.5], colors=['#0f172a'], linewidths=1.5, zorder=2)
    
    # Proyectar pies nominales en X-Z
    ax2.scatter(feet_world[:, 0], feet_world[:, 2], color='#ef4444', marker='s', s=45, label='Apoyos de Pies Fijos', zorder=5, edgecolors='black')
    
    # Chasis proyectado
    ax2.plot([-R_HIP, R_HIP], [0, 0], color='#64748b', linestyle='--', lw=2.0, label='Chasis Nominal', zorder=3)
    
    ax2.set_xlabel('Desplazamiento Horizontal $X_b$ (mm)', labelpad=6)
    ax2.set_ylabel('Desplazamiento Vertical $Z_b$ (mm)', labelpad=6)
    ax2.set_title('(b) Espacio de Trabajo Sagital (Plano $X_b$-$Z_b$ a $Y_b = 0$)', fontweight='bold', pad=12)
    ax2.legend(loc='lower left', frameon=True, facecolor='white', edgecolor='#cbd5e1')
    ax2.axis('equal')
    ax2.set_xlim(-240, 240)
    ax2.set_ylim(-180, 180)
    
    # --- Añadir Barra de Colores y Texto Informativo ---
    cbar = fig.colorbar(cnt1, ax=[ax1, ax2], pad=0.03, aspect=30, location='right')
    cbar.set_label(r'Manipulabilidad Mínima del Chasis $W_{\min} = \min(w_n)$', rotation=90, labelpad=12, fontsize=10)
    cbar.ax.tick_params(labelsize=9)
    cbar.outline.set_linewidth(0.8)
    cbar.outline.set_color('#cbd5e1')
    
    # Cuadro informativo de parámetros
    info_text = (
        "Parámetros del Análisis:\n" +
        r"$R_{\mathrm{hip}} = 150\text{ mm}$" + "\n" +
        "Apoyos Simétricos: 443.3 mm\n" +
        "Altura Nominal: 143.5 mm\n" +
        r"$W_{\min}$ representa la destreza del" + "\n" +
        "peor caso para evitar singularidades."
    )
    fig.text(0.78, 0.22, info_text, fontsize=9, va='top', ha='left',
             bbox=dict(boxstyle="round,pad=0.5", facecolor='white', edgecolor='#cbd5e1', alpha=0.95, lw=0.8))
    
    plt.savefig('espacio_trabajo_chasis.pdf', format='pdf', bbox_inches='tight')
    plt.savefig('espacio_trabajo_chasis.png', format='png', bbox_inches='tight')
    plt.close(fig)
    print("Gráfica del espacio de trabajo del chasis generada con éxito (PDF y PNG).")

if __name__ == '__main__':
    main()
