import pandas as pd
import numpy as np
import plotly.express as px
import plotly.graph_objects as go

def poza_u_matricu(p):
    x, y, z, rx, ry, rz = p
    T = np.identity(4)
    kut = np.sqrt(rx**2 + ry**2 + rz**2)
    
    if kut > 1e-8:
        kx, ky, kz = rx / kut, ry / kut, rz / kut
        ca, sa, vaca = np.cos(kut), np.sin(kut), 1 - np.cos(kut)
        R = np.array([
            [kx*kx*vaca + ca,      kx*ky*vaca - kz*sa, kx*kz*vaca + ky*sa],
            [ky*kx*vaca + kz*sa,   ky*ky*vaca + ca,      ky*kz*vaca - kx*sa],
            [kz*kx*vaca - ky*sa,   kz*ky*vaca + kx*sa, kz*kz*vaca + ca]
        ])
        T[:3, :3] = R
        
    T[:3, 3] = [x, y, z]
    return T

put_poze = "TCP.csv"
try:
    df_poze = pd.read_csv(put_poze, delimiter=';')
    poze = df_poze[['x_mm', 'y_mm', 'z_mm', 'rx_rad', 'ry_rad', 'rz_rad']].values
except FileNotFoundError:
    print(f"Greška: Datoteka '{put_poze}' nije pronađena.")
    exit()
except Exception as e:
    print(f"Greška pri učitavanju poza: {e}")
    exit()

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
    exit()
except Exception as e:
    print(f"Greška pri učitavanju točaka: {e}")
    exit()

br_okvira = min(len(poze), len(okviri))


sve_tocke = []
for i in range(br_okvira):
    poza = poze[i]
    df = okviri[i]

    T = poza_u_matricu(poza)
    homog = np.hstack([df[['x', 'y', 'z']].values, np.ones((len(df), 1))])
    tform = homog @ T.T

    df_t = pd.DataFrame(tform[:, :3], columns=['x_t', 'y_t', 'z_t'])
    df_t['i'] = df['i']

    filtrirano = df_t[df_t['z_t'] <= (poza[2] - 200)]
    sve_tocke.append(filtrirano)

glavni_df = pd.concat(sve_tocke, ignore_index=True)

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
