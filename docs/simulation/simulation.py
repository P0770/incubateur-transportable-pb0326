"""
Simulation du comportement thermique et hygrométrique de l'incubateur PB-0326.

ATTENTION : il s'agit d'un MODÈLE NUMÉRIQUE, pas d'une mesure du prototype.

Modèle à 3 nœuds thermiques :
  - résistance chauffante  C_h · dT_h/dt = u·P − (T_h − T_a)/R_ha
  - air + capteur SHT35    C_a · dT_a/dt = (T_h − T_a)/R_ha − (T_a − T_m)/R_am
  - masse (eau éponge+PETG) C_m · dT_m/dt = (T_a − T_m)/R_am − (T_m − T_amb)/R_ext
Commande identique au firmware v4.0.1 : u = 1 si T_mesurée < 37,1 °C, lecture toutes les 2 s.
Humidité : la vapeur tend vers la saturation à la température de l'éponge (τ = 3 min),
HR mesurée = pression de vapeur / pression de saturation à la température de l'air (formule de Magnus).

Les capacités et résistances thermiques sont des ESTIMATIONS (voir commentaires).
Usage : python3 simulation.py   (nécessite numpy et matplotlib)
"""
from pathlib import Path
import numpy as np
# ---- Paramètres ----
U, Rh = 12.0, 8.2; P = U**2/Rh            # 17.56 W
Tamb = 21.0
C_h, C_a, C_m = 27.0, 25.0, 1290.0         # J/K : résistance, air+capteur, masse (eau+PETG)
R_ha, R_am, R_ext = 1.5, 0.25, 3.5          # K/W
tau_s = 6.0                                # s, constante de temps SHT35 (air brassé)
Ts, dt, tend = 2.0, 0.05, 70*60
seuil = 37.1
tau_e = 180.0                              # s, saturation éponge
def esat(T): return 610.94*np.exp(17.625*T/(T+243.04))   # Pa (Magnus)
rng = np.random.default_rng(3)
Th=Ta=Tm=Ts_=Tamb; e=0.45*esat(Tamb); u=0; Tmeas=Tamb
out=[]; nk=int(Ts/dt); k=0
for i in range(int(tend/dt)+1):
    t=i*dt
    if i%nk==0:
        Tmeas=round(Ts_+rng.normal(0,0.03),2)
        u = 1 if Tmeas<seuil else 0
        RH=min(100.0, e/esat(Ts_)*100 + rng.normal(0,0.15))
        out.append((t,Tmeas,RH,u,Tm))
    qh=(Th-Ta)/R_ha; qa=(Ta-Tm)/R_am; qe=(Tm-Tamb)/R_ext
    Th+=dt*(u*P-qh)/C_h; Ta+=dt*(qh-qa)/C_a; Tm+=dt*(qa-qe)/C_m
    Ts_+=dt*(Ta-Ts_)/tau_s
    e+=dt*(esat(Tm)-e)/tau_e
out=np.array(out)
t=out[:,0]/60; T=out[:,1]; RH=out[:,2]; u=out[:,3]
i365=np.argmax(T>=36.5); print('P=%.2f W'%P, 't(36.5)=%.1f min'%t[i365])
st=t>=t[i365]+5
print('apres stabilisation: min %.2f max %.2f moy %.2f sd %.3f'%(T[st].min(),T[st].max(),T[st].mean(),T[st].std()))
i85=np.argmax(RH>85); print('t HR>85 %.1f min'%t[i85] if RH[i85]>85 else 'jamais>85');print('Tmax global %.2f'%T.max(),' duty %.0f%%'%(100*u[st].mean()), 'HR fin %.1f min %.1f'%(RH[-1],RH[st].min()), 't HR>85: %.1f min'%t[np.argmax(RH>85)])

# ─── Tracé ───
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager as fm
for f in fm.findSystemFonts():
    if 'Inter' in f: fm.fontManager.addfont(f)
plt.rcParams.update({'font.family':'Inter','font.size':13,'axes.edgecolor':'#d1d5db','axes.labelcolor':'#374151',
 'xtick.color':'#6b7280','ytick.color':'#6b7280','axes.spines.top':False,'axes.spines.right':False})
d=out; t=d[:,0]/60; T=d[:,1]; RH=d[:,2]; u=d[:,3]
INK='#111111'; MUT='#6b7280'; BLUE='#2563eb'; TEAL='#0f766e'
fig=plt.figure(figsize=(12,7.9),dpi=140,facecolor='white')
gs=fig.add_gridspec(3,1,height_ratios=[3.2,0.35,2.2],hspace=0.18,left=0.08,right=0.975,top=0.86,bottom=0.125)
a1=fig.add_subplot(gs[0]); a0=fig.add_subplot(gs[1],sharex=a1); a2=fig.add_subplot(gs[2],sharex=a1)
# Température
a1.axhspan(36.5,37.5,color=BLUE,alpha=0.07,lw=0)
a1.axhline(37.0,color=MUT,lw=1,ls=(0,(4,3)))
a1.plot(t,T,color=BLUE,lw=1.6)
a1.set_ylim(20,39.8); a1.set_ylabel('Température (°C)')
a1.text(0.6,38.0,'consigne 37,0 °C (tirets) · bande de tolérance 36,5 – 37,5 °C',color=MUT,fontsize=11)
ax=a1.inset_axes([0.42,0.08,0.55,0.48])
m=t>=25; ax.axhspan(36.5,37.5,color=BLUE,alpha=0.07,lw=0); ax.axhline(37,color=MUT,lw=1,ls=(0,(4,3)))
ax.plot(t[m],T[m],color=BLUE,lw=1.2); ax.set_xlim(25,70); ax.set_ylim(36.3,37.7)
ax.set_facecolor('white'); ax.tick_params(labelsize=10)
for s in ax.spines.values(): s.set_color('#d1d5db')
ax.set_title('zoom 25 – 70 min',fontsize=10.5,color=MUT,loc='left',pad=3)
a1.grid(axis='y',color='#f0f0f2'); plt.setp(a1.get_xticklabels(),visible=False)
# Etat résistance
a0.fill_between(t,0,u,step='post',color='#9ca3af',lw=0)
a0.set_yticks([]); a0.set_ylim(0,1); a0.set_ylabel('Résist.',rotation=0,ha='right',va='center',fontsize=11)
for s in ['left','bottom']: a0.spines[s].set_visible(False)
plt.setp(a0.get_xticklabels(),visible=False); a0.tick_params(axis='x',length=0)
# Humidité
a2.axhline(85,color=MUT,lw=1,ls=(0,(4,3)))
a2.text(0.6,86.5,"seuil d'alerte du firmware : 85 %",color=MUT,fontsize=11)
a2.plot(t,RH,color=TEAL,lw=1.6)
a2.set_ylim(40,100); a2.set_ylabel('Humidité relative (%)'); a2.set_xlabel('Temps depuis la mise en marche (min)')
a2.set_xlim(0,70); a2.grid(axis='y',color='#f0f0f2')
fig.text(0.08,0.955,'Incubateur PB-0326 — comportement attendu en marche',fontsize=17,weight='semibold',color=INK)
fig.text(0.08,0.915,'Ambiance 21 °C · résistance 17,6 W en tout-ou-rien (seuil 37,1 °C, lecture toutes les 2 s) · éponge passive',fontsize=11.5,color=MUT)
fig.text(0.975,0.955,'SIMULATION',ha='right',fontsize=12,weight='bold',color='#b45309',
         bbox=dict(boxstyle='round,pad=0.35',fc='#fffbeb',ec='#f59e0b',lw=1))
fig.text(0.08,0.018,'Modèle numérique à 3 nœuds thermiques (résistance, air, masse eau + PETG) — ce graphique n\'est pas une mesure du prototype.',fontsize=10.5,color='#b45309')
fig.savefig(Path(__file__).with_name('simulation-temperature-humidite.png'),facecolor='white')
