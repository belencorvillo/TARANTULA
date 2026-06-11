import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.patches import Arc

# Estilo académico limpio
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

def check_reachability(X, Z, L1, L2, L3, q2_min, q2_max, q3_min, q3_max):
    # Proyección desde la cadera (X puede ser negativo, indicando detrás de J1)
    s_femur = X - L1
    d = Z
    D2 = s_femur**2 + d**2
    D = np.sqrt(D2)
    
    # 1. Alcance mecánico básico
    if D > (L2 + L3) or D < np.abs(L2 - L3):
        return 0, np.nan
        
    # 2. Calcular q3
    cos_q3 = (D2 - L2**2 - L3**2) / (2.0 * L2 * L3)
    if cos_q3 < -1.0 or cos_q3 > 1.0:
        return 0, np.nan
    q3_val = np.acos(cos_q3)
    
    # Probar ambas soluciones (codo arriba y codo abajo)
    for q3 in [-q3_val, q3_val]:
        if q3 < q3_min or q3 > q3_max:
            continue
            
        # Calcular q2
        alpha1 = np.arctan2(d, s_femur)
        cos_alpha2 = (L2**2 + D2 - L3**2) / (2.0 * L2 * D)
        if cos_alpha2 < -1.0 or cos_alpha2 > 1.0:
            continue
        alpha2 = np.acos(cos_alpha2)
        
        # q2 correspondiente a la solución q3
        if q3 < 0:
            q2 = alpha1 + alpha2
        else:
            q2 = alpha1 - alpha2
            
        if q2_min <= q2 <= q2_max:
            w_norm = np.sin(q3_val)
            return 1, w_norm
            
    return 0, np.nan

def draw_angle_arc(ax, center, angle_start_deg, angle_end_deg, radius, label="", text_dist_factor=1.4, color='#e11d48'):
    t1 = angle_start_deg
    t2 = angle_end_deg
    if t2 < t1:
        t1, t2 = t2, t1
        
    arc = Arc(center, 2*radius, 2*radius, angle=0, theta1=t1, theta2=t2, color=color, lw=1.2, ls='-', zorder=10)
    ax.add_patch(arc)
    
    mid_angle = np.radians((t1 + t2) / 2.0)
    label_pos = np.array(center) + radius * text_dist_factor * np.array([np.cos(mid_angle), np.sin(mid_angle)])
    ax.text(label_pos[0], label_pos[1], label, color=color, fontsize=9, ha='center', va='center', zorder=12,
            bbox=dict(boxstyle="round,pad=0.1", fc="white", ec="none", alpha=0.7))

def main():
    # Longitudes de eslabones en mm
    L1 = 80.0
    L2 = 188.0
    L3 = 211.0
    
    # Limites articulares en radianes
    q1_range = np.radians([-94.0, 105.0])
    q2_range = np.radians([-122.0, 117.0])
    q3_range = np.radians([-157.0, 154.0])
    
    # --- 1. Muestras para la vista 3D ---
    num_samples = 18000
    np.random.seed(42)
    q1_s = np.random.uniform(q1_range[0], q1_range[1], num_samples)
    q2_s = np.random.uniform(q2_range[0], q2_range[1], num_samples)
    q3_s = np.random.uniform(q3_range[0], q3_range[1], num_samples)
    
    r_s = L1 + L2 * np.cos(q2_s) + L3 * np.cos(q2_s + q3_s)
    x_s = r_s * np.cos(q1_s)
    y_s = r_s * np.sin(q1_s)
    z_s = L2 * np.sin(q2_s) + L3 * np.sin(q2_s + q3_s)
    
    # --- 2. Malla para el perfil sagital 2D (plano X-Z local, q1 = 0) ---
    x_grid = np.linspace(-300, 500, 300)
    z_grid = np.linspace(-350, 350, 300)
    X_m, Z_m = np.meshgrid(x_grid, z_grid)
    
    reachable = np.zeros_like(X_m)
    manipulability = np.full_like(X_m, np.nan)
    for i in range(X_m.shape[0]):
        for j in range(X_m.shape[1]):
            reach, w = check_reachability(
                X_m[i, j], Z_m[i, j], 
                L1, L2, L3, 
                q2_range[0], q2_range[1], q3_range[0], q3_range[1]
            )
            reachable[i, j] = reach
            if reach:
                manipulability[i, j] = w
            
    # --- 3. Crear figura ---
    fig = plt.figure(figsize=(14, 6.5))
    fig.patch.set_facecolor('white')
    
    # Subplot 1: Vista 3D limpia
    ax_3d = fig.add_subplot(121, projection='3d')
    ax_3d.set_facecolor('white')
    
    # Pintar nube de puntos en 3D
    sc_3d = ax_3d.scatter(x_s, y_s, z_s, c=z_s, cmap='coolwarm', s=0.2, alpha=0.15, depthshade=True)
    
    # Origen (Cuerpo/J1)
    ax_3d.plot([0], [0], [0], 'ko', ms=5, label='Origen J1')
    
    ax_3d.set_xlabel('Eje X (mm)', labelpad=6)
    ax_3d.set_ylabel('Eje Y (mm)', labelpad=6)
    ax_3d.set_zlabel('Eje Z (mm)', labelpad=6)
    ax_3d.set_title('(a) Espacio de Trabajo Tridimensional (3D)', fontweight='bold', pad=10)
    ax_3d.view_init(elev=20, azim=-55)
    
    # Subplot 2: Perfil 2D con Diseño Académico Premium
    ax_2d = fig.add_subplot(122)
    ax_2d.set_facecolor('white')
    
    # Sombreado de la zona de trabajo según manipulabilidad
    cnt_2d = ax_2d.contourf(X_m, Z_m, manipulability, levels=np.linspace(0, 1.0, 11), cmap='viridis', alpha=0.85, zorder=1)
    ax_2d.contour(X_m, Z_m, reachable, levels=[0.5], colors=['#1e293b'], linewidths=1.5, zorder=2)
    
    # Generar curvas de contorno de límites articulares para referencia académica
    q2_sweep = np.linspace(q2_range[0], q2_range[1], 200)
    q3_sweep = np.linspace(q3_range[0], q3_range[1], 200)
    
    x_q3min = L1 + L2 * np.cos(q2_sweep) + L3 * np.cos(q2_sweep + q3_range[0])
    z_q3min = L2 * np.sin(q2_sweep) + L3 * np.sin(q2_sweep + q3_range[0])
    
    x_q3max = L1 + L2 * np.cos(q2_sweep) + L3 * np.cos(q2_sweep + q3_range[1])
    z_q3max = L2 * np.sin(q2_sweep) + L3 * np.sin(q2_sweep + q3_range[1])
    
    x_q2min = L1 + L2 * np.cos(q2_range[0]) + L3 * np.cos(q2_range[0] + q3_sweep)
    z_q2min = L2 * np.sin(q2_range[0]) + L3 * np.sin(q2_range[0] + q3_sweep)
    
    x_q2max = L1 + L2 * np.cos(q2_range[1]) + L3 * np.cos(q2_range[1] + q3_sweep)
    z_q2max = L2 * np.sin(q2_range[1]) + L3 * np.sin(q2_range[1] + q3_sweep)
    
    x_max_reach = L1 + (L2 + L3) * np.cos(q2_sweep)
    z_max_reach = (L2 + L3) * np.sin(q2_sweep)

    ax_2d.plot(x_q3min, z_q3min, color='#ef4444', linestyle='--', linewidth=0.9, alpha=0.7, zorder=2)
    ax_2d.plot(x_q3max, z_q3max, color='#ef4444', linestyle='-.', linewidth=0.9, alpha=0.7, zorder=2)
    ax_2d.plot(x_q2min, z_q2min, color='#3b82f6', linestyle='--', linewidth=0.9, alpha=0.7, zorder=2)
    ax_2d.plot(x_q2max, z_q2max, color='#3b82f6', linestyle='-.', linewidth=0.9, alpha=0.7, zorder=2)
    ax_2d.plot(x_max_reach, z_max_reach, color='#0f172a', linestyle=':', linewidth=1.1, alpha=0.8, zorder=2)
    
    # Pose nominal de ejemplo (q2 = 25°, q3 = -95°)
    q2_nom = np.radians(25.0)
    q3_nom = np.radians(-95.0)
    
    x0, y0 = 0.0, 0.0
    x1, y1 = L1, 0.0
    x2, y2 = L1 + L2 * np.cos(q2_nom), L2 * np.sin(q2_nom)
    x3, y3 = x2 + L3 * np.cos(q2_nom + q3_nom), y2 + L3 * np.sin(q2_nom + q3_nom)
    
    # Sistemas de coordenadas base en J1
    arrow_len = 60.0
    ax_2d.annotate('', xy=(arrow_len, 0), xytext=(0, 0),
                  arrowprops=dict(arrowstyle="-|>", color='#0f172a', lw=1.2, mutation_scale=10, shrinkA=0, shrinkB=0), zorder=9)
    ax_2d.text(arrow_len + 6, 0, r'$X_0$', fontsize=9, fontweight='bold', va='center', zorder=10)
    
    ax_2d.annotate('', xy=(0, arrow_len), xytext=(0, 0),
                  arrowprops=dict(arrowstyle="-|>", color='#0f172a', lw=1.2, mutation_scale=10, shrinkA=0, shrinkB=0), zorder=9)
    ax_2d.text(0, arrow_len + 6, r'$Z_0$', fontsize=9, fontweight='bold', ha='center', va='bottom', zorder=10)

    # Línea horizontal de referencia en J2 para q2
    ax_2d.plot([L1, L1 + 70], [0, 0], color='#475569', linestyle='--', linewidth=0.8, alpha=0.8, zorder=3)
    
    # Prolongación de Fémur (L2) para referenciar q3 en J3
    dx_femur = x2 - x1
    dy_femur = y2 - y1
    len_femur = np.sqrt(dx_femur**2 + dy_femur**2)
    ext_len = 60.0
    x3_ref_ext = x2 + (dx_femur / len_femur) * ext_len
    y3_ref_ext = y2 + (dy_femur / len_femur) * ext_len
    ax_2d.plot([x2, x3_ref_ext], [y2, y3_ref_ext], color='#475569', linestyle='--', linewidth=0.8, alpha=0.8, zorder=3)

    # Arcos para los ángulos q2 y q3
    draw_angle_arc(ax_2d, (x1, y1), 0.0, 25.0, radius=38.0, label=r'$q_2$', color='#e11d48')
    draw_angle_arc(ax_2d, (x2, y2), 25.0 - 95.0, 25.0, radius=32.0, label=r'$q_3$', color='#e11d48')

    # Eslabones con efecto outline
    ax_2d.plot([x0, x1], [y0, y1], color='#0f172a', lw=7.0, solid_capstyle='round', zorder=4)
    ax_2d.plot([x0, x1], [y0, y1], color='#475569', lw=4.0, solid_capstyle='round', label='Eslabón Coxa ($L_1$)', zorder=5)
    
    ax_2d.plot([x1, x2], [y1, y2], color='#0f172a', lw=7.0, solid_capstyle='round', zorder=4)
    ax_2d.plot([x1, x2], [y1, y2], color='#ea580c', lw=4.0, solid_capstyle='round', label='Eslabón Fémur ($L_2$)', zorder=5)
    
    ax_2d.plot([x2, x3], [y2, y3], color='#0f172a', lw=7.0, solid_capstyle='round', zorder=4)
    ax_2d.plot([x2, x3], [y2, y3], color='#0d9488', lw=4.0, solid_capstyle='round', label='Eslabón Tibia ($L_3$)', zorder=5)
    
    # Articulaciones
    ax_2d.plot(x0, y0, 'o', color='white', markeredgecolor='#0f172a', markeredgewidth=1.5, ms=7.5, zorder=6)
    ax_2d.plot(x1, y1, 'o', color='white', markeredgecolor='#0f172a', markeredgewidth=1.5, ms=7.5, zorder=6)
    ax_2d.plot(x2, y2, 'o', color='white', markeredgecolor='#0f172a', markeredgewidth=1.5, ms=7.5, zorder=6)
    ax_2d.plot(x3, y3, 'o', color='#ef4444', markeredgecolor='#0f172a', markeredgewidth=1.5, ms=7.5, label='Efector Final (Pie)', zorder=7)
    
    # Rotulación de articulaciones
    ax_2d.text(x0, y0 - 15, r'$J_1$', fontsize=9, fontweight='bold', ha='center', va='top', zorder=10)
    ax_2d.text(x1, y1 + 12, r'$J_2$', fontsize=9, fontweight='bold', ha='center', va='bottom', zorder=10)
    ax_2d.text(x2 - 5, y2 + 12, r'$J_3$', fontsize=9, fontweight='bold', ha='right', va='bottom', zorder=10)
    ax_2d.text(x3 + 8, y3 - 8, r'$P_{\mathrm{pie}}$', fontsize=9, fontweight='bold', ha='left', va='top', zorder=10)
    
    # Detalles visuales
    ax_2d.spines['top'].set_visible(False)
    ax_2d.spines['right'].set_visible(False)
    ax_2d.spines['left'].set_color('#cbd5e1')
    ax_2d.spines['bottom'].set_color('#cbd5e1')
    ax_2d.grid(True, which='both', linestyle=':', linewidth=0.5, color='#cbd5e1')
    
    ax_2d.set_xlabel('Alcance Horizontal X (mm)', labelpad=6, fontsize=10)
    ax_2d.set_ylabel('Altura Z (mm)', labelpad=6, fontsize=10)
    ax_2d.set_title('(b) Envolvente de Trabajo Sagital (Plano X-Z)', fontweight='bold', pad=10)
    
    # Caja de parámetros
    info_text = (
        r"$\mathbf{Par\acute{a}metros\ Cinem\acute{a}ticos}$" + "\n" +
        r"$L_1 = 80.0\text{ mm}$" + "\n" +
        r"$L_2 = 188.0\text{ mm}$" + "\n" +
        r"$L_3 = 211.0\text{ mm}$" + "\n" +
        r"$q_2 \in [-122^\circ, 117^\circ]$" + "\n" +
        r"$q_3 \in [-157^\circ, 154^\circ]$" + "\n" +
        r"$w_{\mathrm{rel}} = |\sin(q_3)|$"
    )
    ax_2d.text(480, 310, info_text, fontsize=8.5, va='top', ha='right',
              bbox=dict(boxstyle="round,pad=0.4", facecolor='white', edgecolor='#cbd5e1', alpha=0.95, lw=0.8), zorder=8)
    
    # Ficticios para la leyenda de curvas límite
    ax_2d.plot([], [], color='#ef4444', linestyle='--', label=r'Límites de $q_3$')
    ax_2d.plot([], [], color='#3b82f6', linestyle='--', label=r'Límites de $q_2$')
    ax_2d.plot([], [], color='#0f172a', linestyle=':', label=r'Alcance Máx Teórico')
    
    ax_2d.legend(loc='lower left', frameon=True, facecolor='white', edgecolor='#cbd5e1')
    ax_2d.axis('equal')
    ax_2d.set_xlim(-280, 520)
    ax_2d.set_ylim(-340, 340)
    
    # Barra de colores
    cbar = fig.colorbar(cnt_2d, ax=ax_2d, pad=0.03, aspect=25)
    cbar.set_label(r'Manipulabilidad Relativa $w_n = |\sin(q_3)|$', rotation=90, labelpad=12, fontsize=10)
    cbar.ax.tick_params(labelsize=8)
    cbar.outline.set_linewidth(0.8)
    cbar.outline.set_color('#cbd5e1')
    
    plt.tight_layout()
    fig.savefig('espacio_trabajo_pata_3d.pdf', format='pdf', bbox_inches='tight')
    fig.savefig('espacio_trabajo_pata_3d.png', format='png', bbox_inches='tight')
    plt.close(fig)
    print("Gráfica 3D/2D del espacio de trabajo generada con éxito.")

if __name__ == '__main__':
    main()
