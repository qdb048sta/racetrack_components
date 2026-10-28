"""Plot previously calculated arrays. This module does not fit a controller."""
from __future__ import annotations
from pathlib import Path
import numpy as np
from .interpolation import LABELS
from .road import PeriodicTrack

def plot_results(data: str|Path,output: str|Path) -> list[Path]:
    """Each chart is a separate figure; no fixed colors or styles are selected."""
    import matplotlib.pyplot as plt
    out=Path(output);d=np.load(out/'plot_data.npz');a=d['source'];u=d['u'];ids=d['training_indices']
    track=PeriodicTrack(data);s=np.linspace(0,track.length,10000);q=track.sample(s)
    files=[]
    fig,ax=plt.subplots(figsize=(9,7))
    ax.plot(q['boundary_left'][:,0],q['boundary_left'][:,1],label='Left boundary (reconstructed)',lw=1)
    ax.plot(q['boundary_right'][:,0],q['boundary_right'][:,1],label='Right boundary (reconstructed)',lw=1)
    ax.plot(q['center'][:,0],q['center'][:,1],label='Full-data periodic cubic',lw=1)
    ax.scatter(a[::10,0],a[::10,1],s=8,label='Source points (every 10th)')
    ax.set(aspect='equal',xlabel='Local x [m]',ylabel='Local y [m]',title='Norisring: online source geometry and reconstructed road')
    ax.legend(fontsize=8);ax.grid(True,alpha=.25);fig.tight_layout()
    p=out/'track_overview.png';fig.savefig(p,dpi=180);plt.close(fig);files.append(p)
    fig,ax=plt.subplots(figsize=(9,6))
    region=(u>1580)&(u<1740)
    for m in ('linear','pchip_cyclic','akima_cyclic','periodic_cubic','periodic_quintic'):
        xy=d[m];ax.plot(xy[region,0],xy[region,1],label=LABELS[m],lw=1.4)
    inside=(d['source_u']>1580)&(d['source_u']<1740)
    ax.scatter(a[inside,0],a[inside,1],marker='x',s=25,label='All source samples')
    train=ids[(d['source_u'][ids]>1580)&(d['source_u'][ids]<1740)]
    ax.scatter(a[train,0],a[train,1],s=60,marker='+',label='Retained training points')
    ax.set(aspect='equal',xlabel='Local x [m]',ylabel='Local y [m]',title='Tight bend: reconstructing from approximately 20 m spacing')
    ax.legend(fontsize=8);ax.grid(True,alpha=.25);fig.tight_layout()
    p=out/'bend_comparison.png';fig.savefig(p,dpi=180);plt.close(fig);files.append(p)
    fig,ax=plt.subplots(figsize=(10,5))
    for m in ('pchip_cyclic','akima_cyclic','periodic_cubic','periodic_quintic','periodic_smoothing_cubic'):
        ax.plot(u[region],d[m+'_curvature'][region],label=LABELS[m],lw=1.3)
    ax.set(xlabel='Source chord-distance parameter u [m]',ylabel='Signed curvature [1/m]',
           title='Similar-looking geometry can produce different curvature')
    ax.legend(fontsize=8);ax.grid(True,alpha=.25);fig.tight_layout()
    p=out/'curvature_comparison.png';fig.savefig(p,dpi=180);plt.close(fig);files.append(p)
    return files
