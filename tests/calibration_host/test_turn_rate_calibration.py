"""Synthetic full-sequence captures for Python and MATLAB calibration checks."""
import sys
sys.dont_write_bytecode = True
from pathlib import Path
import tempfile

import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[2]/"src/scripts"))
from analyze_turn_rate_calibration import load_capture, analyze


def make_capture(scale=2.4, yaw_axis=0, body_stationary=False):
    t=np.arange(40001)*.001
    t[1::2] += .0001  # actual timestamps differ from nominal control ticks
    schedule=[(0,4),(.2,4),(0,2),(-.2,4),(0,2),(.4,4),(0,2),
              (-.4,4),(0,2),(.6,4),(0,2),(-.6,4),(0,2)]
    reference=np.zeros(len(t)); phase=np.zeros(len(t)); edge=0
    boundaries=[0]
    for i,(rate,duration) in enumerate(schedule):
        mask=(t>=edge)&(t<edge+duration)
        reference[mask]=rate; phase[mask]=10+i
        edge+=duration; boundaries.append(edge)
    phase[-1]=22
    offsets=np.array([3,-7,10])*(1000/32768)
    raw=np.tile(offsets,(len(t),1))
    if not body_stationary:
        raw[:,yaw_axis] += np.rad2deg(-reference/scale)
    counts=np.rint(-raw*32768/1000)
    reported=-counts*(1000/32768)-offsets
    calibrated=t>=1
    gyro=reported.copy(); gyro[~calibrated] += offsets
    rates=np.deg2rad(reported)*calibrated[:,None]
    integral=np.vstack([np.zeros(3), np.cumsum(
        np.diff(t)[:,None]*(rates[1:]+rates[:-1])/2,axis=0)])
    gyro_count=np.cumsum(calibrated)
    ref_angle=np.r_[0,np.cumsum(np.diff(t)*(reference[1:]+reference[:-1])/2)]
    metres_per_count=2*np.pi*.0315/9.6/48
    encoder=np.column_stack([np.rint(-.07*ref_angle/metres_per_count),
                             np.rint(.07*ref_angle/metres_per_count)])
    wheel_angle=(encoder[:,1]-encoder[:,0])*metres_per_count/.14
    d=np.zeros((len(t),62));d[:,0]=t;d[:,1]=6
    d[:,2]=3+8192+np.where(calibrated,2048,0)+np.where(t>=2,4096,0)
    d[:,4]=-.07*reference*9.6/.0315;d[:,5]=-d[:,4];d[:,6:8]=d[:,4:6]
    d[:,8]=np.where(reference!=0,-.4*np.sign(reference),0);d[:,9]=-d[:,8]
    d[:,12:15]=gyro;d[:,15]= -1 if body_stationary else 1
    d[:,26]=np.nan;d[:,28]=-rates[:,0]
    d[:,29]=np.arctan2(np.sin(-integral[:,0]),np.cos(-integral[:,0]))
    d[:,40]=phase;d[:,42]=1000
    d[:,43:45]=(encoder+32768)%65536-32768
    d[:,45:48]=reference[:,None];d[:,48]=-rates[:,0];d[:,50]=reference
    d[:,51]=np.where(calibrated,(t+1)*1e6,0)
    d[:,52:54]=((t+1)*1e6)[:,None];d[:,54]=16
    d[:,55:58]=counts;d[:,58:61]=integral;d[:,61]=gyro_count
    phases=[]
    for i,(rate,duration) in enumerate(schedule):
        start=np.searchsorted(t,boundaries[i]);end=np.searchsorted(t,boundaries[i+1])
        phases.append([10+i,rate,t[start],t[end],*(integral[end]-integral[start]),
                       wheel_angle[end]-wheel_angle[start],gyro_count[end]-gyro_count[start],0])
    data=d[::19]
    return data,np.asarray(phases),offsets


def write_capture(path, scale=2.4, yaw_axis=0, body_stationary=False):
    data,phases,offsets=make_capture(scale,yaw_axis,body_stationary)
    with open(path,'w') as f:
        f.write('% CAL turn-rate-calibration\n% Title: Turn-rate calibration sequence\n% Result: complete\n')
        f.write('% Sample time 1000 us, log divider 19, gear 9.6, wheel radius 0.0315 m, wheel base 0.14 m, PPR 48\n')
        f.write('% Yaw mapping axis,sign,scale 0 -1 1\n')
        f.write('% Gyro offsets (library deg/s) '+' '.join(map(str,offsets))+'\n')
        f.write('% Stationary gravity (library g) '+('-1' if body_stationary else '1')+' 0 0\n')
        f.write('% Stationary calibration duration 4 gyro samples 1000 accel samples 2000 complete 1\n')
        f.write('% Gyro readback at start: readable 1 valid 1 register 16 dlpf 2 divider 0\n')
        for phase in phases:
            f.write('% turn-phase '+' '.join('%.9g'%x for x in phase)+'\n')
        np.savetxt(f,data,fmt='%.9g')
    return data,phases


def main():
    with tempfile.TemporaryDirectory() as tmp:
        path=Path(tmp)/'turn.log'
        data,phases=write_capture(path)
        loaded,loaded_phases,text=load_capture(path)
        assert loaded.shape[1]==62 and len(loaded_phases)==13
        result=analyze(loaded,loaded_phases,text)
        assert result['dominant_axis']==0 and len(result['turns'])==6
        assert result['conversion_error_dps']<1e-5
        assert abs(result['mapped_slope']-1/2.4)<.002
        assert abs(result['mapped_bias'])<.001
        assert all(abs(row['wheel_over_gyro']-2.4)<.02 for row in result['turns'])
        assert all(999<row['gyro_hz']<1001 for row in result['turns'])
        # Select the last complete block in a monitor file with repeated runs.
        old=path.read_text();write_capture(path,scale=1.7)
        new=path.read_text();path.write_text(old+new)
        d,p,text=load_capture(path)
        assert abs(analyze(d,p,text)['mapped_slope']-1/1.7)<.002
        write_capture(path,yaw_axis=2)
        d,p,text=load_capture(path)
        assert analyze(d,p,text)['dominant_axis']==2
        write_capture(path,body_stationary=True)
        d,p,text=load_capture(path)
        assert abs(analyze(d,p,text)['mapped_slope'])<1e-8
        # Stored integrals, not decimated rates, drive the per-turn ratios.
        d[:,48]=0
        assert analyze(d,p,text)['turns'][0]['gyro_deg']==0
    print('PASS turn-rate analysis: full sequence, fresh integrals, 2.4x scale, axes, repeated captures, upside-down')


if __name__=='__main__': main()
