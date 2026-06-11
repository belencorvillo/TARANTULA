import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import warnings

# Silenciar advertencias de rebanadas vacías en np.nanmin/max
warnings.filterwarnings("ignore", category=RuntimeWarning)

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


# Parámetros mecánicos de Tarantula
L1 = 80.0     # Coxa (mm)
L2 = 188.0    # Fémur (mm)
L3 = 211.0    # Tibia (mm)
R_HIP = 150.0  # Radio del chasis a las caderas (mm)

# Límites articulares reales de la pata (de Leg.cpp)
q1_min_deg, q1_max_deg = -90.0, 90.0
q2_min_deg, q2_max_deg = -100.0, 100.0
q3_min_deg, q3_max_deg = -150.0, 30.0 # knee_up = True

# Ángulos de montaje de las caderas (caderas de patas 1, 2, 3, 4)
thetas = np.array([np.pi/4.0, -np.pi/4.0, -3.0*np.pi/4.0, 3.0*np.pi/4.0])

# Postura nominal de diseño para cálculo de apoyos nominales
q1_nom = 0.0
q2_nom = np.radians(20.0)
q3_nom = np.radians(-100.0)

# Posición del pie en el sistema local de la pata en pose nominal
r_local_nom = L1 + L2 * np.cos(q2_nom) + L3 * np.cos(q2_nom + q3_nom)
z_local_nom = L2 * np.sin(q2_nom) + L3 * np.sin(q2_nom + q3_nom)

# Posiciones globales fijas de los pies en el mundo (chasis centrado en (0,0,0))
feet_world = []
for th in thetas:
    x_foot = (R_HIP + r_local_nom) * np.cos(th)
    y_foot = (R_HIP + r_local_nom) * np.sin(th)
    z_foot = z_local_nom
    feet_world.append([x_foot, y_foot, z_foot])
feet_world = np.array(feet_world)

def check_leg_ik_vectorized(x_l, y_l, z_l):
    """
    Verifica la cinemática inversa local de una pata para arreglos de puntos.
    Devuelve un arreglo booleano de alcanzabilidad.
    """
    reachable = np.ones(x_l.shape, dtype=bool)
    
    # 1. Verificar límites de q1 (yaw)
    q1 = np.arctan2(y_l, x_l)
    q1_deg = np.degrees(q1)
    reachable &= (q1_deg >= q1_min_deg) & (q1_deg <= q1_max_deg)
    
    # Proyección en plano sagital local
    s = np.sqrt(x_l**2 + y_l**2)
    s_femur = s - L1
    d = z_l
    
    D2 = s_femur**2 + d**2
    D = np.sqrt(D2)
    
    # 2. Alcance mecánico básico
    reachable &= (D <= (L2 + L3)) & (D >= np.abs(L2 - L3))
    
    # 3. Verificar límites de q3 (tibia)
    cos_q3 = (D2 - L2**2 - L3**2) / (2.0 * L2 * L3)
    # Evitar errores numéricos de precisión de punto flotante
    cos_q3_clamped = np.clip(cos_q3, -1.0, 1.0)
    q3_val = np.arccos(cos_q3_clamped)
    q3_deg = np.degrees(-q3_val) # knee_up = True
    
    reachable &= (cos_q3 >= -1.0001) & (cos_q3 <= 1.0001)
    reachable &= (q3_deg >= q3_min_deg) & (q3_deg <= q3_max_deg)
    
    # 4. Verificar límites de q2 (fémur)
    alpha1 = np.arctan2(d, s_femur)
    cos_alpha2 = (L2**2 + D2 - L3**2) / (2.0 * L2 * np.maximum(D, 1e-6))
    cos_alpha2_clamped = np.clip(cos_alpha2, -1.0, 1.0)
    alpha2 = np.arccos(cos_alpha2_clamped)
    q2_deg = np.degrees(alpha1 + alpha2) # knee_up = True
    
    reachable &= (cos_alpha2 >= -1.0001) & (cos_alpha2 <= 1.0001)
    reachable &= (q2_deg >= q2_min_deg) & (q2_deg <= q2_max_deg)
    
    return reachable

def check_chassis_pose_vectorized(x_b, y_b, z_b):
    """
    Verifica si una pose (x_b, y_b, z_b) del chasis es alcanzable por las 4 patas.
    Devuelve un arreglo booleano.
    """
    reach_all = np.ones(x_b.shape, dtype=bool)
    for i in range(4):
        th = thetas[i]
        # Transformación local para cada pata i (desplazamientos p_local relativos al origen J1 de la pata)
        # p_local_i = R_z(-theta_i) * (p_foot_world_i - T_body - [R_HIP * cos(theta_i), R_HIP * sin(theta_i), 0]^T)
        # Esto simplifica a:
        x_l = r_local_nom - (x_b * np.cos(th) + y_b * np.sin(th))
        y_l = x_b * np.sin(th) - y_b * np.cos(th)
        z_l = z_local_nom - z_b
        
        reach_all &= check_leg_ik_vectorized(x_l, y_l, z_l)
    return reach_all

def apply_styling(ax):
    ax.spines['top'].set_visible(False)
    ax.spines['right'].set_visible(False)
    ax.spines['left'].set_color('#cbd5e1')
    ax.spines['bottom'].set_color('#cbd5e1')
    ax.grid(True, which='both', linestyle=':', linewidth=0.5, color='#cbd5e1')

def main():
    # 1. Definir la malla de muestreo 3D para el chasis
    grid_size = 100
    x_range = np.linspace(-220, 220, grid_size)
    y_range = np.linspace(-220, 220, grid_size)
    z_range = np.linspace(-180, 180, grid_size)
    
    X_3d, Y_3d, Z_3d = np.meshgrid(x_range, y_range, z_range, indexing='ij')
    
    print("Calculando espacio de trabajo del chasis...")
    reach_3d = check_chassis_pose_vectorized(X_3d, Y_3d, Z_3d)
    
    # 2. Obtener las superficies superior e inferior para el gráfico 3D
    Z_masked = np.where(reach_3d, Z_3d, np.nan)
    Z_min = np.nanmin(Z_masked, axis=2)
    Z_max = np.nanmax(Z_masked, axis=2)
    
    # 3. Proyecciones 2D (OR lógico en la dimensión colapsada)
    proj_xy = np.any(reach_3d, axis=2)
    proj_xz = np.any(reach_3d, axis=1)
    proj_yz = np.any(reach_3d, axis=0)
    
    # --- Configurar Figura ---
    fig = plt.figure(figsize=(14, 11))
    fig.patch.set_facecolor('white')
    
    # 1. SUBPLOT 3D: Volumen del Espacio de Trabajo
    ax_3d = fig.add_subplot(221, projection='3d')
    ax_3d.set_facecolor('white')
    
    # Generar la rejilla 2D para graficar la superficie
    X_2d, Y_2d = np.meshgrid(x_range, y_range, indexing='ij')
    
    # Graficar la tapa superior y la tapa inferior
    surf_max = ax_3d.plot_surface(X_2d, Y_2d, Z_max, cmap='viridis', alpha=0.75, linewidth=0, antialiased=True, label='Límite Superior')
    surf_min = ax_3d.plot_surface(X_2d, Y_2d, Z_min, cmap='plasma', alpha=0.6, linewidth=0, antialiased=True, label='Límite Inferior')
    
    ax_3d.set_xlabel('Desplazamiento X (mm)', labelpad=6)
    ax_3d.set_ylabel('Desplazamiento Y (mm)', labelpad=6)
    ax_3d.set_zlabel('Desplazamiento Z (mm)', labelpad=6)
    ax_3d.set_title('(a) Volumen 3D del Espacio de Trabajo', fontweight='bold', pad=10)
    ax_3d.view_init(elev=25, azim=-45)
    
    # 2. SUBPLOT XY (Plano Horizontal)
    ax_xy = fig.add_subplot(222)
    ax_xy.set_facecolor('white')
    apply_styling(ax_xy)
    
    # Rellenar contorno de proyección
    ax_xy.contourf(X_2d, Y_2d, proj_xy, levels=[0.5, 1.5], colors=['#e2e8f0'], alpha=0.9)
    ax_xy.contour(X_2d, Y_2d, proj_xy, levels=[0.5], colors=['#0f172a'], linewidths=1.5)
    
    # Apoyos fijos en XY
    ax_xy.scatter(feet_world[:, 0], feet_world[:, 1], color='#ef4444', marker='s', s=45, label='Apoyos Pies Fijos', zorder=5, edgecolors='black')
    
    # Contorno de chasis nominal (círculo)
    theta_circle = np.linspace(0, 2*np.pi, 100)
    ax_xy.plot(R_HIP * np.cos(theta_circle), R_HIP * np.sin(theta_circle), color='#64748b', linestyle='--', lw=1.2, label='Chasis Nominal (Cuerpo)')
    for th in thetas:
        ax_xy.plot([0, R_HIP * np.cos(th)], [0, R_HIP * np.sin(th)], color='#94a3b8', linestyle=':', lw=1.0)
        
    ax_xy.set_xlabel('Desplazamiento Horizontal $X_b$ (mm)')
    ax_xy.set_ylabel('Desplazamiento Lateral $Y_b$ (mm)')
    ax_xy.set_title('(b) Proyección en el Plano Horizontal $X_b$-$Y_b$', fontweight='bold', pad=10)
    ax_xy.legend(loc='lower left', frameon=True, facecolor='white', edgecolor='#cbd5e1')
    ax_xy.axis('equal')
    ax_xy.set_xlim(-360, 360)
    ax_xy.set_ylim(-360, 360)
    
    # 3. SUBPLOT XZ (Plano Sagital)
    ax_xz = fig.add_subplot(223)
    ax_xz.set_facecolor('white')
    apply_styling(ax_xz)
    
    X_xz, Z_xz = np.meshgrid(x_range, z_range, indexing='ij')
    ax_xz.contourf(X_xz, Z_xz, proj_xz, levels=[0.5, 1.5], colors=['#e2e8f0'], alpha=0.9)
    ax_xz.contour(X_xz, Z_xz, proj_xz, levels=[0.5], colors=['#0f172a'], linewidths=1.5)
    
    # Apoyos fijos en XZ (proyectados)
    ax_xz.scatter(feet_world[:, 0], feet_world[:, 2], color='#ef4444', marker='s', s=45, label='Apoyos Pies Fijos', zorder=5, edgecolors='black')
    
    # Cuerpo nominal proyectado
    ax_xz.plot([-R_HIP, R_HIP], [0, 0], color='#64748b', linestyle='--', lw=2.0, label='Chasis Nominal')
    
    ax_xz.set_xlabel('Desplazamiento Horizontal $X_b$ (mm)')
    ax_xz.set_ylabel('Desplazamiento Vertical $Z_b$ (mm)')
    ax_xz.set_title('(c) Proyección en el Plano Sagital $X_b$-$Z_b$', fontweight='bold', pad=10)
    ax_xz.legend(loc='lower left', frameon=True, facecolor='white', edgecolor='#cbd5e1')
    ax_xz.axis('equal')
    ax_xz.set_xlim(-360, 360)
    ax_xz.set_ylim(-200, 200)
    
    # 4. SUBPLOT YZ (Plano Frontal)
    ax_yz = fig.add_subplot(224)
    ax_yz.set_facecolor('white')
    apply_styling(ax_yz)
    
    Y_yz, Z_yz = np.meshgrid(y_range, z_range, indexing='ij')
    ax_yz.contourf(Y_yz, Z_yz, proj_yz, levels=[0.5, 1.5], colors=['#e2e8f0'], alpha=0.9)
    ax_yz.contour(Y_yz, Z_yz, proj_yz, levels=[0.5], colors=['#0f172a'], linewidths=1.5)
    
    # Apoyos fijos en YZ (proyectados)
    ax_yz.scatter(feet_world[:, 1], feet_world[:, 2], color='#ef4444', marker='s', s=45, label='Apoyos Pies Fijos', zorder=5, edgecolors='black')
    
    # Cuerpo nominal proyectado
    ax_yz.plot([-R_HIP, R_HIP], [0, 0], color='#64748b', linestyle='--', lw=2.0, label='Chasis Nominal')
    
    ax_yz.set_xlabel('Desplazamiento Lateral $Y_b$ (mm)')
    ax_yz.set_ylabel('Desplazamiento Vertical $Z_b$ (mm)')
    ax_yz.set_title('(d) Proyección en el Plano Frontal $Y_b$-$Z_b$', fontweight='bold', pad=10)
    ax_yz.legend(loc='lower left', frameon=True, facecolor='white', edgecolor='#cbd5e1')
    ax_yz.axis('equal')
    ax_yz.set_xlim(-360, 360)
    ax_yz.set_ylim(-200, 200)
    
    plt.suptitle('Espacio de Trabajo del Chasis de Tarantula (Traslación Pura, Ángulos de Euler = 0°)', fontsize=13, fontweight='bold', y=0.96)
    
    # Cuadro informativo de parámetros
    info_text = (
        "Parámetros del Chasis:\n" +
        r"$R_{\mathrm{hip}} = 150\text{ mm}$" + "\n" +
        r"$L_1 = 80\text{ mm}$" + "\n" +
        r"$L_2 = 188\text{ mm}$" + "\n" +
        r"$L_3 = 211\text{ mm}$" + "\n" +
        "Altura Nominal: 143.5 mm\n\n" +
        "Límites Articulares:\n" +
        r"$q_1 \in [-90^\circ, 90^\circ]$" + "\n" +
        r"$q_2 \in [-100^\circ, 100^\circ]$" + "\n" +
        r"$q_3 \in [-150^\circ, 30^\circ]$"
    )
    fig.text(0.44, 0.49, info_text, fontsize=9.5, va='center', ha='left',
             bbox=dict(boxstyle="round,pad=0.6", facecolor='white', edgecolor='#cbd5e1', alpha=0.95, lw=0.8))
    
    plt.savefig('espacio_trabajo_traslacion.pdf', format='pdf', bbox_inches='tight')
    plt.savefig('espacio_trabajo_traslacion.png', format='png', bbox_inches='tight')
    plt.close(fig)
    print("Documento 1 (Traslación Pura) generado con éxito en PDF y PNG.")

if __name__ == '__main__':
    main()
