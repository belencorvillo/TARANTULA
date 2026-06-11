import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Arc

# Estilo académico y tipografía serif para publicaciones
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
    # Proyección desde la cadera (X puede ser negativo, indicando detrás del cuerpo)
    s_femur = X - L1
    d = Z
    D2 = s_femur**2 + d**2
    D = np.sqrt(D2)
    
    # 1. Comprobar alcance mecánico básico
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
            q2 = alpha1 + alpha2 # codo arriba
        else:
            q2 = alpha1 - alpha2 # codo abajo
            
        if q2_min <= q2 <= q2_max:
            # Es alcanzable y respeta límites, retornamos manipulabilidad normalizada
            w_norm = np.sin(q3_val) # sin(q3_val) es positivo ya que q3_val está en [0, pi]
            return 1, w_norm
            
    return 0, np.nan

def draw_angle_arc(ax, center, angle_start_deg, angle_end_deg, radius, label="", text_dist_factor=1.4, color='#e11d48'):
    t1 = angle_start_deg
    t2 = angle_end_deg
    if t2 < t1:
        t1, t2 = t2, t1
        
    # Dibujar arco en matplotlib
    arc = Arc(center, 2*radius, 2*radius, angle=0, theta1=t1, theta2=t2, color=color, lw=1.2, ls='-', zorder=10)
    ax.add_patch(arc)
    
    # Calcular posición del texto de etiqueta en la mitad del arco
    mid_angle = np.radians((t1 + t2) / 2.0)
    label_pos = np.array(center) + radius * text_dist_factor * np.array([np.cos(mid_angle), np.sin(mid_angle)])
    ax.text(label_pos[0], label_pos[1], label, color=color, fontsize=10, ha='center', va='center', zorder=12,
            bbox=dict(boxstyle="round,pad=0.1", fc="white", ec="none", alpha=0.7))

def main():
    # Longitudes de eslabones en mm
    L1 = 80.0
    L2 = 188.0
    L3 = 211.0
    
    # Limites articulares en radianes
    q2_min, q2_max = np.radians(-122.0), np.radians(117.0)
    q3_min, q3_max = np.radians(-157.0), np.radians(154.0)
    
    # Crear malla X-Z (permitiendo X negativo para la zona trasera de J1)
    x_grid = np.linspace(-300, 500, 400)
    z_grid = np.linspace(-350, 350, 400)
    X, Z = np.meshgrid(x_grid, z_grid)
    
    # Evaluar alcanzabilidad y manipulabilidad de Yoshikawa en la malla
    reachable = np.zeros_like(X)
    manipulability = np.full_like(X, np.nan)
    for i in range(X.shape[0]):
        for j in range(X.shape[1]):
            reach, w = check_reachability(
                X[i, j], Z[i, j], 
                L1, L2, L3, 
                q2_min, q2_max, q3_min, q3_max
            )
            reachable[i, j] = reach
            if reach:
                manipulability[i, j] = w
            
    # Crear figura
    fig, ax = plt.subplots(figsize=(8.5, 6.5))
    fig.patch.set_facecolor('white')
    ax.set_facecolor('white')
    
    # 1. Dibujar el área del espacio de trabajo sombreada según manipulabilidad
    cnt = ax.contourf(X, Z, manipulability, levels=np.linspace(0, 1.0, 11), cmap='viridis', alpha=0.85, zorder=1)
    
    # Contorno nítido de la frontera del espacio de trabajo
    ax.contour(X, Z, reachable, levels=[0.5], colors=['#1e293b'], linewidths=1.5, zorder=2)
    
    # 2. Generar curvas de contorno de límites articulares para referencia académica
    q2_sweep = np.linspace(q2_min, q2_max, 200)
    q3_sweep = np.linspace(q3_min, q3_max, 200)
    
    # Curva q3 = q3_min (variando q2)
    x_q3min = L1 + L2 * np.cos(q2_sweep) + L3 * np.cos(q2_sweep + q3_min)
    z_q3min = L2 * np.sin(q2_sweep) + L3 * np.sin(q2_sweep + q3_min)
    
    # Curva q3 = q3_max (variando q2)
    x_q3max = L1 + L2 * np.cos(q2_sweep) + L3 * np.cos(q2_sweep + q3_max)
    z_q3max = L2 * np.sin(q2_sweep) + L3 * np.sin(q2_sweep + q3_max)
    
    # Curva q2 = q2_min (variando q3)
    x_q2min = L1 + L2 * np.cos(q2_min) + L3 * np.cos(q2_min + q3_sweep)
    z_q2min = L2 * np.sin(q2_min) + L3 * np.sin(q2_min + q3_sweep)
    
    # Curva q2 = q2_max (variando q3)
    x_q2max = L1 + L2 * np.cos(q2_max) + L3 * np.cos(q2_max + q3_sweep)
    z_q2max = L2 * np.sin(q2_max) + L3 * np.sin(q2_max + q3_sweep)
    
    # Alcance máximo teórico (cuando q3 = 0, variando q2)
    x_max_reach = L1 + (L2 + L3) * np.cos(q2_sweep)
    z_max_reach = (L2 + L3) * np.sin(q2_sweep)

    # Graficar las curvas límite
    ax.plot(x_q3min, z_q3min, color='#ef4444', linestyle='--', linewidth=0.9, alpha=0.7, zorder=2)
    ax.plot(x_q3max, z_q3max, color='#ef4444', linestyle='-.', linewidth=0.9, alpha=0.7, zorder=2)
    ax.plot(x_q2min, z_q2min, color='#3b82f6', linestyle='--', linewidth=0.9, alpha=0.7, zorder=2)
    ax.plot(x_q2max, z_q2max, color='#3b82f6', linestyle='-.', linewidth=0.9, alpha=0.7, zorder=2)
    ax.plot(x_max_reach, z_max_reach, color='#0f172a', linestyle=':', linewidth=1.1, alpha=0.8, zorder=2)

    # 3. Dibujar la estructura de la pata en una pose nominal (ej. q2=25°, q3=-95°)
    q2_nom = np.radians(25.0)
    q3_nom = np.radians(-95.0)
    
    # Coordenadas de las articulaciones
    x0, y0 = 0.0, 0.0 # J1 (Base coxa)
    x1, y1 = L1, 0.0  # J2 (Fémur)
    x2, y2 = L1 + L2 * np.cos(q2_nom), L2 * np.sin(q2_nom) # J3 (Tibia)
    x3, y3 = x2 + L3 * np.cos(q2_nom + q3_nom), y2 + L3 * np.sin(q2_nom + q3_nom) # Pie
    
    # Líneas y flechas de referencia
    # Sistema de Coordenadas Base {0} en J1
    arrow_len = 65.0
    ax.annotate('', xy=(arrow_len, 0), xytext=(0, 0),
                arrowprops=dict(arrowstyle="-|>", color='#0f172a', lw=1.5, mutation_scale=12, shrinkA=0, shrinkB=0), zorder=9)
    ax.text(arrow_len + 8, 0, r'$X_0$', fontsize=10, fontweight='bold', va='center', zorder=10)
    
    ax.annotate('', xy=(0, arrow_len), xytext=(0, 0),
                arrowprops=dict(arrowstyle="-|>", color='#0f172a', lw=1.5, mutation_scale=12, shrinkA=0, shrinkB=0), zorder=9)
    ax.text(0, arrow_len + 8, r'$Z_0$', fontsize=10, fontweight='bold', ha='center', va='bottom', zorder=10)

    # Línea horizontal de referencia en J2 para q2
    ax.plot([L1, L1 + 75], [0, 0], color='#475569', linestyle='--', linewidth=0.8, alpha=0.8, zorder=3)
    
    # Prolongación de Fémur (L2) para referenciar q3 en J3
    dx_femur = x2 - x1
    dy_femur = y2 - y1
    len_femur = np.sqrt(dx_femur**2 + dy_femur**2)
    ext_len = 65.0
    x3_ref_ext = x2 + (dx_femur / len_femur) * ext_len
    y3_ref_ext = y2 + (dy_femur / len_femur) * ext_len
    ax.plot([x2, x3_ref_ext], [y2, y3_ref_ext], color='#475569', linestyle='--', linewidth=0.8, alpha=0.8, zorder=3)

    # Dibujar arcos para los ángulos articulares q2 y q3
    draw_angle_arc(ax, (x1, y1), 0.0, 25.0, radius=40.0, label=r'$q_2$', color='#e11d48')
    draw_angle_arc(ax, (x2, y2), 25.0 - 95.0, 25.0, radius=35.0, label=r'$q_3$', color='#e11d48')

    # Dibujar eslabones físicos con efecto contorno (outline)
    ax.plot([x0, x1], [y0, y1], color='#0f172a', lw=7.5, solid_capstyle='round', zorder=4)
    ax.plot([x0, x1], [y0, y1], color='#475569', lw=4.5, solid_capstyle='round', label='Eslabón Coxa ($L_1$)', zorder=5)
    
    ax.plot([x1, x2], [y1, y2], color='#0f172a', lw=7.5, solid_capstyle='round', zorder=4)
    ax.plot([x1, x2], [y1, y2], color='#ea580c', lw=4.5, solid_capstyle='round', label='Eslabón Fémur ($L_2$)', zorder=5)
    
    ax.plot([x2, x3], [y2, y3], color='#0f172a', lw=7.5, solid_capstyle='round', zorder=4)
    ax.plot([x2, x3], [y2, y3], color='#0d9488', lw=4.5, solid_capstyle='round', label='Eslabón Tibia ($L_3$)', zorder=5)
    
    # Dibujar articulaciones (puntos blancos con borde)
    ax.plot(x0, y0, 'o', color='white', markeredgecolor='#0f172a', markeredgewidth=1.8, ms=8.5, zorder=6)
    ax.plot(x1, y1, 'o', color='white', markeredgecolor='#0f172a', markeredgewidth=1.8, ms=8.5, zorder=6)
    ax.plot(x2, y2, 'o', color='white', markeredgecolor='#0f172a', markeredgewidth=1.8, ms=8.5, zorder=6)
    # Pie / Efector final
    ax.plot(x3, y3, 'o', color='#ef4444', markeredgecolor='#0f172a', markeredgewidth=1.8, ms=8.5, label='Efector Final (Pie)', zorder=7)
    
    # Etiquetas de articulaciones
    ax.text(x0, y0 - 15, r'$J_1$', fontsize=10, fontweight='bold', ha='center', va='top', zorder=10)
    ax.text(x1, y1 + 12, r'$J_2$', fontsize=10, fontweight='bold', ha='center', va='bottom', zorder=10)
    ax.text(x2 - 6, y2 + 12, r'$J_3$', fontsize=10, fontweight='bold', ha='right', va='bottom', zorder=10)
    ax.text(x3 + 8, y3 - 8, r'$P_{\mathrm{pie}}$', fontsize=10, fontweight='bold', ha='left', va='top', zorder=10)
    
    # Líneas y textos informativos
    ax.spines['top'].set_visible(False)
    ax.spines['right'].set_visible(False)
    ax.spines['left'].set_color('#cbd5e1')
    ax.spines['bottom'].set_color('#cbd5e1')
    ax.grid(True, which='both', linestyle=':', linewidth=0.5, color='#cbd5e1')
    
    ax.set_xlabel('Alcance Horizontal X (mm)', labelpad=8, fontsize=10)
    ax.set_ylabel('Altura Z (mm)', labelpad=8, fontsize=10)
   # ax.set_title('Espacio de Trabajo Sagital de la Extremidad (Plano X-Z)\nÍndice de Manipulabilidad de Yoshikawa', fontweight='bold', pad=15, fontsize=11)
    
    # Cuadro de parámetros cinemáticos
    info_text = (
        r"$\mathbf{Par\acute{a}metros\ Cinem\acute{a}ticos}$" + "\n" +
        r"$L_1 = 80.0\text{ mm}$" + "\n" +
        r"$L_2 = 188.0\text{ mm}$" + "\n" +
        r"$L_3 = 211.0\text{ mm}$" + "\n" +
        r"$q_2 \in [-122^\circ, 117^\circ]$" + "\n" +
        r"$q_3 \in [-157^\circ, 154^\circ]$" + "\n" +
        r"$w_{\mathrm{rel}} = |\sin(q_3)|$"
    )
    ax.text(480, 310, info_text, fontsize=9.5, va='top', ha='right',
            bbox=dict(boxstyle="round,pad=0.5", facecolor='white', edgecolor='#cbd5e1', alpha=0.95, lw=0.8), zorder=8)
    
    # Elementos de la leyenda agrupados académicamente
    # Creamos las líneas ficticias para la leyenda de las curvas límite
    ax.plot([], [], color='#ef4444', linestyle='--', label=r'Límites de $q_3$ ($\pm 157^\circ, 154^\circ$)')
    ax.plot([], [], color='#3b82f6', linestyle='--', label=r'Límites de $q_2$ ($\pm 122^\circ, 117^\circ$)')
    ax.plot([], [], color='#0f172a', linestyle=':', label=r'Alcance Singular Máx ($q_3=0^\circ$)')
    
    ax.legend(loc='lower left', frameon=True, facecolor='white', edgecolor='#cbd5e1')
    ax.axis('equal')
    ax.set_xlim(-280, 520)
    ax.set_ylim(-340, 340)
    
    # Añadir barra de colores para la manipulabilidad
    cbar = fig.colorbar(cnt, ax=ax, pad=0.03, aspect=25)
    cbar.set_label(r'Manipulabilidad Relativa $w_n = |\sin(q_3)|$', rotation=90, labelpad=12, fontsize=10)
    cbar.ax.tick_params(labelsize=9)
    cbar.outline.set_linewidth(0.8)
    cbar.outline.set_color('#cbd5e1')
    
    plt.tight_layout()
    fig.savefig('espacio_trabajo_pata_limpio.pdf', format='pdf', bbox_inches='tight')
    fig.savefig('espacio_trabajo_pata_limpio.png', format='png', bbox_inches='tight')
    plt.close(fig)
    print("Gráfica académica del espacio de trabajo generada con éxito (PDF y PNG).")

if __name__ == '__main__':
    main()
