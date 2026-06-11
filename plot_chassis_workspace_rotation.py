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
h_nominal = -z_local_nom  # ~143.5 mm (Altura del chasis sobre el plano de los apoyos)

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

def check_pose_pitch_vectorized(z_b, pitch_rad):
    """
    Evalúa si la pose de chasis (X=0, Y=0, Z_b, Pitch) es alcanzable por las 4 patas.
    """
    reach_all = np.ones(z_b.shape, dtype=bool)
    c_p = np.cos(pitch_rad)
    s_p = np.sin(pitch_rad)
    
    for i in range(4):
        th = thetas[i]
        x_f = feet_world[i, 0]
        y_f = feet_world[i, 1]
        z_f = feet_world[i, 2]
        
        # Rotar apoyos en plano Y (Pitch)
        p_body_x = x_f * c_p - (z_f - z_b) * s_p
        p_body_y = y_f
        p_body_z = x_f * s_p + (z_f - z_b) * c_p
        
        # Transformar a local de pata
        dx = p_body_x - R_HIP * np.cos(th)
        dy = p_body_y - R_HIP * np.sin(th)
        dz = p_body_z
        
        x_l = dx * np.cos(th) + dy * np.sin(th)
        y_l = -dx * np.sin(th) + dy * np.cos(th)
        z_l = dz
        
        reach_all &= check_leg_ik_vectorized(x_l, y_l, z_l)
    return reach_all

def check_pose_roll_vectorized(z_b, roll_rad):
    """
    Evalúa si la pose de chasis (X=0, Y=0, Z_b, Roll) es alcanzable por las 4 patas.
    """
    reach_all = np.ones(z_b.shape, dtype=bool)
    c_r = np.cos(roll_rad)
    s_r = np.sin(roll_rad)
    
    for i in range(4):
        th = thetas[i]
        x_f = feet_world[i, 0]
        y_f = feet_world[i, 1]
        z_f = feet_world[i, 2]
        
        # Rotar apoyos en plano X (Roll)
        p_body_x = x_f
        p_body_y = y_f * c_r + (z_f - z_b) * s_r
        p_body_z = -y_f * s_r + (z_f - z_b) * c_r
        
        # Transformar a local de pata
        dx = p_body_x - R_HIP * np.cos(th)
        dy = p_body_y - R_HIP * np.sin(th)
        dz = p_body_z
        
        x_l = dx * np.cos(th) + dy * np.sin(th)
        y_l = -dx * np.sin(th) + dy * np.cos(th)
        z_l = dz
        
        reach_all &= check_leg_ik_vectorized(x_l, y_l, z_l)
    return reach_all

def apply_styling(ax):
    ax.spines['top'].set_visible(False)
    ax.spines['right'].set_visible(False)
    ax.spines['left'].set_color('#cbd5e1')
    ax.spines['bottom'].set_color('#cbd5e1')
    ax.grid(True, which='both', linestyle=':', linewidth=0.5, color='#cbd5e1')

def main():
    # 1. Definir los rangos de barrido
    angle_max_deg = 60.0
    grid_size = 400
    
    angles_deg = np.linspace(-angle_max_deg, angle_max_deg, grid_size)
    z_b_range = np.linspace(-150, 150, grid_size)
    
    # Rejillas para Sagital (Pitch) y Frontal (Roll)
    PITCH_DEG, Z_B_PITCH = np.meshgrid(angles_deg, z_b_range, indexing='ij')
    ROLL_DEG, Z_B_ROLL = np.meshgrid(angles_deg, z_b_range, indexing='ij')
    
    # Convertir ángulos a radianes
    pitch_rad = np.radians(PITCH_DEG)
    roll_rad = np.radians(ROLL_DEG)
    
    # 2. Calcular alcanzabilidad
    print("Calculando espacio de trabajo de Rotación vs. Altura...")
    reach_sagittal = check_pose_pitch_vectorized(Z_B_PITCH, pitch_rad)
    reach_frontal = check_pose_roll_vectorized(Z_B_ROLL, roll_rad)
    
    # Convertir Z_b (desplazamiento) a Altura Absoluta Z del chasis (Z = h_nominal + Z_b)
    Z_ABS_PITCH = h_nominal + Z_B_PITCH
    Z_ABS_ROLL = h_nominal + Z_B_ROLL
    
    # --- Configurar Figura ---
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6.5))
    fig.patch.set_facecolor('white')
    
    # ------------------ SUBPLOT 1: CORTE SAGITAL (PITCH) ------------------
    ax1.set_facecolor('white')
    apply_styling(ax1)
    
    # Rellenar la zona de alcanzabilidad
    ax1.contourf(PITCH_DEG, Z_ABS_PITCH, reach_sagittal, levels=[0.5, 1.5], colors=['#e2e8f0'], alpha=0.9, zorder=1)
    # Contorno nítido de la frontera
    ax1.contour(PITCH_DEG, Z_ABS_PITCH, reach_sagittal, levels=[0.5], colors=['#0f4c81'], linewidths=1.8, zorder=2)
    
    # Encontrar máximos límites para anotaciones
    valid_indices = np.where(reach_sagittal)
    if len(valid_indices[0]) > 0:
        max_pitch = PITCH_DEG[valid_indices].max()
        min_pitch = PITCH_DEG[valid_indices].min()
        max_z = Z_ABS_PITCH[valid_indices].max()
        min_z = Z_ABS_PITCH[valid_indices].min()
        
        # Puntos de cabeceo máximo en altura nominal
        idx_nom_h = np.argmin(np.abs(z_b_range)) # Z_b = 0
        reach_at_nom_h = reach_sagittal[:, idx_nom_h]
        valid_angles_nom_h = angles_deg[reach_at_nom_h]
        if len(valid_angles_nom_h) > 0:
            max_p_nom_h = valid_angles_nom_h.max()
            ax1.axvline(max_p_nom_h, color='#f58220', linestyle=':', lw=1.2, zorder=3)
            ax1.axvline(-max_p_nom_h, color='#f58220', linestyle=':', lw=1.2, zorder=3)
            ax1.scatter([max_p_nom_h, -max_p_nom_h], [h_nominal, h_nominal], color='#f58220', marker='o', s=35, zorder=4)
            ax1.text(max_p_nom_h + 1.5, h_nominal, f'{max_p_nom_h:.1f}°', color='#f58220', fontsize=9, va='center', ha='left', fontweight='bold')
            ax1.text(-max_p_nom_h - 1.5, h_nominal, f'-{max_p_nom_h:.1f}°', color='#f58220', fontsize=9, va='center', ha='right', fontweight='bold')
            
        # Altura nominal línea de referencia
        ax1.axhline(h_nominal, color='#64748b', linestyle='--', lw=1.0, zorder=2)
        ax1.text(52, h_nominal + 4, 'Altura Nominal', color='#64748b', fontsize=8.5, ha='right')

    ax1.set_xlabel('Ángulo de Cabeceo / Pitch (grados)', labelpad=6)
    ax1.set_ylabel('Altura del Chasis $Z$ (mm)', labelpad=6)
    ax1.set_title('(a) Corte Sagital (Altura $Z$ vs. Pitch)', fontweight='bold', pad=12)
    ax1.set_xlim(-60, 60)
    ax1.set_ylim(20, 260)
    
    # ------------------ SUBPLOT 2: CORTE FRONTAL (ROLL) ------------------
    ax2.set_facecolor('white')
    apply_styling(ax2)
    
    # Rellenar la zona de alcanzabilidad
    ax2.contourf(ROLL_DEG, Z_ABS_ROLL, reach_frontal, levels=[0.5, 1.5], colors=['#e2e8f0'], alpha=0.9, zorder=1)
    # Contorno nítido de la frontera
    ax2.contour(ROLL_DEG, Z_ABS_ROLL, reach_frontal, levels=[0.5], colors=['#0f4c81'], linewidths=1.8, zorder=2)
    
    # Encontrar máximos límites para anotaciones en Roll
    valid_indices_r = np.where(reach_frontal)
    if len(valid_indices_r[0]) > 0:
        # Puntos de balanceo máximo en altura nominal
        reach_at_nom_h_r = reach_frontal[:, idx_nom_h]
        valid_angles_nom_h_r = angles_deg[reach_at_nom_h_r]
        if len(valid_angles_nom_h_r) > 0:
            max_r_nom_h = valid_angles_nom_h_r.max()
            ax2.axvline(max_r_nom_h, color='#f58220', linestyle=':', lw=1.2, zorder=3)
            ax2.axvline(-max_r_nom_h, color='#f58220', linestyle=':', lw=1.2, zorder=3)
            ax2.scatter([max_r_nom_h, -max_r_nom_h], [h_nominal, h_nominal], color='#f58220', marker='o', s=35, zorder=4)
            ax2.text(max_r_nom_h + 1.5, h_nominal, f'{max_r_nom_h:.1f}°', color='#f58220', fontsize=9, va='center', ha='left', fontweight='bold')
            ax2.text(-max_r_nom_h - 1.5, h_nominal, f'-{max_r_nom_h:.1f}°', color='#f58220', fontsize=9, va='center', ha='right', fontweight='bold')
            
        # Altura nominal línea de referencia
        ax2.axhline(h_nominal, color='#64748b', linestyle='--', lw=1.0, zorder=2)
        ax2.text(52, h_nominal + 4, 'Altura Nominal', color='#64748b', fontsize=8.5, ha='right')

    ax2.set_xlabel('Ángulo de Balanceo / Roll (grados)', labelpad=6)
    ax2.set_ylabel('Altura del Chasis $Z$ (mm)', labelpad=6)
    ax2.set_title('(b) Corte Frontal (Altura $Z$ vs. Roll)', fontweight='bold', pad=12)
    ax2.set_xlim(-60, 60)
    ax2.set_ylim(20, 260)
    
    plt.suptitle('Espacio de Trabajo del Chasis de Tarantula (Rotación vs. Altura)', fontsize=13, fontweight='bold', y=0.96)
    
    # Cuadro informativo de parámetros
    info_text = (
        "Condiciones del Ensayo:\n" +
        "Pies Fijos: Anclados en Stand\n" +
        "Altura Nominal: 143.5 mm\n\n" +
        "Límites de Orientación (a Altura Nom.):\n" +
        f"Pitch Máx: ±{max_p_nom_h:.1f}°\n" +
        f"Roll Máx: ±{max_r_nom_h:.1f}°\n\n" +
        "Área Gris = Región Operativa Alcanzable\n" +
        "Línea Azul = Límite Físico del Chasis"
    )
    fig.text(0.44, 0.22, info_text, fontsize=9, va='bottom', ha='left',
             bbox=dict(boxstyle="round,pad=0.5", facecolor='white', edgecolor='#cbd5e1', alpha=0.95, lw=0.8))
    
    plt.savefig('espacio_trabajo_rotacion_altura.pdf', format='pdf', bbox_inches='tight')
    plt.savefig('espacio_trabajo_rotacion_altura.png', format='png', bbox_inches='tight')
    plt.close(fig)
    print("Documento 2 (Rotación vs Altura) generado con éxito en PDF y PNG.")

if __name__ == '__main__':
    main()
