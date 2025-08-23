import pandas as pd
import numpy as np
import plotly.express as px
import plotly.graph_objects as go

def poza_u_matricu(p):
    # p = [x_mm, y_mm, z_mm, rx_rad, ry_rad, rz_rad]
    x, y, z, rx, ry, rz = p
    T = np.identity(4)
    kut = np.sqrt(rx**2 + ry**2 + rz**2)
    if kut > 1e-8:
        kx, ky, kz = rx / kut, ry / kut, rz / kut
        ca, sa, vaca = np.cos(kut), np.sin(kut), 1 - np.cos(kut)
        R = np.array([
            [kx*kx*vaca + ca,      kx*ky*vaca - kz*sa, kx*kz*vaca + ky*sa],
            [ky*kx*vaca + kz*sa,   ky*ky*vaca + ca,    ky*kz*vaca - kx*sa],
            [kz*kx*vaca - ky*sa,   kz*ky*vaca + kx*sa, kz*kz*vaca + ca]
        ])
        T[:3, :3] = R
    # POZOR: radimo sve u milimetrima, a Kamera.csv je već u mm
    T[:3, 3] = [x, y, z]
    return T

# --- 1) UČITAJ POZE IZ Frames.csv---
put_frames = "Frames.csv"
try:
    df_frames = pd.read_csv(put_frames, delimiter=';')
    # Riješi moguće praznine: odbaci okvire bez poze
    df_frames = df_frames.dropna(subset=['x_mm','y_mm','z_mm','rx_rad','ry_rad','rz_rad'])
    poze = df_frames[['x_mm', 'y_mm', 'z_mm', 'rx_rad', 'ry_rad', 'rz_rad']].values
    # (Ako ti treba vrijeme framea za kasniju analizu)
    # t_frame_ns = df_frames['t_frame_ns'].values
except FileNotFoundError:
    print(f"Greška: Datoteka '{put_frames}' nije pronađena.")
    raise SystemExit
except Exception as e:
    print(f"Greška pri učitavanju Frames.csv: {e}")
    raise SystemExit

# --- 2) UČITAJ OKVIRE IZ Kamera.csv---
put_tocke = "Kamera.csv"
okviri = []
try:
    with open(put_tocke, 'r') as f:
        for linija in f:
            linija = linija.strip()
            if not linija:
                continue
            dijelovi = [d.strip() for d in linija.split(';')]
            tocke = [list(map(float, d.split(','))) for d in dijelovi if d]
            okviri.append(pd.DataFrame(tocke, columns=['x', 'y', 'z', 'i']))
except FileNotFoundError:
    print(f"Greška: Datoteka '{put_tocke}' nije pronađena.")
    raise SystemExit
except Exception as e:
    print(f"Greška pri učitavanju točaka: {e}")
    raise SystemExit

# --- 3) Uskladi broj poza i okvira ---
br_okvira = min(len(poze), len(okviri))
if br_okvira == 0:
    print("Nema podudarnih okvira i poza za vizualizaciju.")
    raise SystemExit

# --- 4) Transformiraj i spajaj točke ---
sve_tocke = []
for i in range(br_okvira):
    poza = poze[i]                 # [x_mm, y_mm, z_mm, rx, ry, rz]
    df = okviri[i]                 # stupci: x,y,z,i (sve u mm)

    T = poza_u_matricu(poza)
    homog = np.hstack([df[['x', 'y', 'z']].values, np.ones((len(df), 1))])
    tform = homog @ T.T            # točke -> bazni sustav (mm)

    df_t = pd.DataFrame(tform[:, :3], columns=['x_t', 'y_t', 'z_t'])
    df_t['i'] = df['i']

    filtrirano = df_t[df_t['z_t'] <= (poza[2] - 200)]
    sve_tocke.append(filtrirano)

glavni_df = pd.concat(sve_tocke, ignore_index=True)

# --- 5) Plotly 3D scatter ---
fig = px.scatter_3d(
    glavni_df,
    x='x_t',
    y='y_t',
    z='z_t',
    color='z_t',
    title='3D prikaz obojen prema Z koordinati',
    color_continuous_scale='viridis',
    labels={'x_t': 'X [mm]', 'y_t': 'Y [mm]', 'z_t': 'Z [mm]'}
)

# --- 6) Filtriraj samo točke iznad -15 mm za bounding box ---
df_filter = glavni_df[glavni_df['z_t'] > -20]

if not df_filter.empty:
    xmin, xmax = df_filter['x_t'].min(), df_filter['x_t'].max()
    ymin, ymax = df_filter['y_t'].min(), df_filter['y_t'].max()
    zmin, zmax = df_filter['z_t'].min(), df_filter['z_t'].max()

    # Definiraj 8 vrhova kocke
    vrhovi = np.array([
        [xmin, ymin, zmin],
        [xmin, ymin, zmax],
        [xmin, ymax, zmin],
        [xmin, ymax, zmax],
        [xmax, ymin, zmin],
        [xmax, ymin, zmax],
        [xmax, ymax, zmin],
        [xmax, ymax, zmax]
    ])

    # Definiraj bridove (indeksi parova vrhova)
    bridovi = [
        (0,1),(0,2),(0,4),
        (1,3),(1,5),
        (2,3),(2,6),
        (3,7),
        (4,5),(4,6),
        (5,7),
        (6,7)
    ]

    # Dodaj bounding box kao line trace
    for (i1, i2) in bridovi:
        fig.add_trace(go.Scatter3d(
            x=[vrhovi[i1,0], vrhovi[i2,0]],
            y=[vrhovi[i1,1], vrhovi[i2,1]],
            z=[vrhovi[i1,2], vrhovi[i2,2]],
            mode='lines',
            line=dict(color='red', width=3),
            name='Bounding Box'
        ))

    # Ispis dimenzija kutije
    print(f"Bounding Box (za točke iznad -15 mm):")
    print(f"X: {xmin:.2f} – {xmax:.2f}  (širina: {xmax-xmin:.4f} mm)")
    print(f"Y: {ymin:.2f} – {ymax:.2f}  (dubina: {ymax-ymin:.4f} mm)")
    print(f"Z: {zmin:.2f} – {zmax:.2f}  (visina: {zmax-zmin:.2f} mm)")
else:
    print("Nema točaka iznad -15 mm, bounding box nije napravljen.")

# --- 7) Prosjek svih točaka u rasponu -23 <= z_t <= -21 ---
df_avg = glavni_df[(glavni_df['z_t'] >= -23) & (glavni_df['z_t'] <= -21)]
if not df_avg.empty:
    avg_x, avg_y, avg_z = df_avg[['x_t','y_t','z_t']].mean()
    print(f"Prosječna točka u rasponu -23 <= z_t <= -21 mm:")
    print(f"X = {avg_x:.2f} mm, Y = {avg_y:.2f} mm, Z = {avg_z:.2f} mm")

    # Dodaj marker prosječne točke u graf
    fig.add_trace(go.Scatter3d(
        x=[avg_x], y=[avg_y], z=[avg_z],
        mode='markers+text',
        marker=dict(size=6, color='blue', symbol='diamond'),
        text=['Avg -23:-21'],
        textposition='top center',
        name='Prosjek (-23:-21)'
    ))
else:
    print("Nema točaka u rasponu -23 <= z_t <= -21 mm.")






fig.update_traces(marker=dict(size=2))
fig.update_layout(
    scene=dict(
        xaxis=dict(title='X [mm]'),
        yaxis=dict(title='Y [mm]'),
        zaxis=dict(title='Z [mm]'),
        aspectmode='data'
    )
)

fig.add_trace(go.Scatter3d(
    x=[0], y=[0], z=[0],
    mode='markers+text',
    marker=dict(size=6, color='red'),
    text=['Baza (0,0,0)'],
    textposition='top center',
    name='Baza robota'
))

fig.show()
