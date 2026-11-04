"""Liu tire laboratory: modal vibration, force relaxation, rigid ring and PCM."""
import argparse
import json
from dataclasses import replace
from pathlib import Path
import numpy as np
from tracklab.paper_models.liu2023 import ModalRing,RigidRing,LegendrePCM,relaxation_step
from tracklab.paper_models.common import write_run


def run(output="outputs/papers/liu2023"):
    ring=ModalRing();times=np.linspace(0,.25,501)
    frequency,mode=ring.modes(2);initial=np.r_[mode[:,0]*.001,np.zeros(2)]
    response=ring.integrate(2,times,initial)
    force=np.array([relaxation_step(0.,1000.,10.,.1,t) for t in times])
    rigid=RigidRing();state=np.zeros(10);state[1]=-1000/rigid.p.kz
    # Prescribed force pulse: no claim to reproduce the thesis contact solver.
    forcing=lambda t:{"fz":-1000.-100.*np.exp(-((t-.05)/.008)**2),"axle_load":1000.}
    rigid_response=rigid.integrate(times,state,forcing)
    nodes=np.linspace(-1,1,15)[:,None]
    def frequencies(points):
        return np.array([ModalRing(replace(ring.p,radial_foundation=ring.p.radial_foundation*(1+.1*x))).modes(2)[0][0] for x in np.asarray(points).ravel()])
    pcm=LegendrePCM(1,3).fit(nodes,frequencies(nodes));validation=np.linspace(-.97,.97,44)[:,None]
    error=float(np.max(np.abs(pcm.predict(validation)-frequencies(validation))))
    mean,var=pcm.moments()
    summary={"experiment":"Liu 2023 tire submodel laboratory","mode_2_frequencies_hz":frequency.tolist(),
             "frequency_mean_hz":float(mean),"frequency_variance_hz2":float(var),"pcm_max_validation_error_hz":error,
             "mpc_present_in_source":False,"scope":"Nonrotating in-plane modes, prescribed-force rigid ring, relaxation, uniform-parameter PCM; no contact solver or vehicle"}
    rows=np.column_stack([times,response,force,rigid_response[:,1],rigid_response[:,3]])
    write_run(output,["time_s","radial_mode_m","circumferential_mode_m","radial_mode_rate_mps","circumferential_mode_rate_mps",
                      "relaxed_fx_n","ring_z_m","rim_z_m"],rows,summary)
    out=Path(output)
    np.savetxt(out/"modes.csv",np.array([[n,*ring.modes(n)[0]] for n in range(11)]),delimiter=",",header="mode,low_hz,high_hz",comments="")
    return summary


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__);p.add_argument("--output",default="outputs/papers/liu2023");a=p.parse_args()
    print(json.dumps(run(a.output),indent=2))
