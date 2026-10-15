"""Offline check of the drafted Flight.cs fixes (docs/handoff-flight-fixes.md).

Compares CURRENT, FIX 1 (inertia-scaled gains) and FIX 1+2 (plus yaw-lowest-priority
mixing) for five frames: hover after a 0.5 rad/s roll kick, and 2 s of full yaw stick.
Same physics as flight_loop_replica.py (coupled model). Run:
    python tools/flight-audit/fix_check.py
"""
import math
kf=9.13e-6; km=5.0e-7; Fmax=kf*1256.6**2
Kp,Kd,Kpy=10.0,0.06,0.55; dt=0.02; g=9.81; clamp=3.0
roll=[-1,1,1,-1]; pitch=[-1,-1,1,1]; yaw=[-1,1,-1,1]
REF=(0.003,0.006,0.003)
def sim(m,Ixx,Iyy,Izz,arm,fix1=False,fix2=False,T=8.0,yaw_cmd=0.0,yaw_kick=0.001,roll_kick=0.0):
    a=arm*math.sqrt(2)/2
    sp,sy,sr=(Ixx/REF[0],Iyy/REF[1],Izz/REF[2]) if fix1 else (1,1,1)
    thp=thr=0.0; wp,wy,wr=0.0,yaw_kick,roll_kick; z=vz=0.0; tgt=0.0; zmax=0; ymax=0
    for k in range(int(T/dt)):
        ycmd = yaw_cmd if k*dt<2.0 else 0.0
        mag=math.sqrt(wp*wp+wy*wy+wr*wr)
        if mag>clamp: s=clamp/mag; wp,wy,wr=wp*s,wy*s,wr*s
        hover=m*g
        if ycmd==0: thrust=hover*(1+max(-1.8,min(1.8,0.5*(tgt-z)-0.8*vz)))
        else: thrust=hover; 
        tf=math.cos(thr)*math.cos(thp)
        if tf>0.01: thrust*=min(1/tf,1.15)
        des_roll=-ycmd*math.radians(15)*0.5   # banking at MaxSpeed 0.5
        tp=(Kp*(0-thp)-Kd*wp)*sp; tr=(Kp*(des_roll-thr)-Kd*wr)*sr; ty=Kpy*(ycmd*math.radians(200)-wy)*sy
        rp=[thrust/4+roll[i]*tr/(4*a)+pitch[i]*tp/(4*a) for i in range(4)]
        yw=[yaw[i]*ty*kf/(4*km) for i in range(4)]
        kk=1.0
        if fix2:
            for i in range(4):
                if yw[i]<0 and rp[i]+yw[i]<0: kk=min(kk,max(0,rp[i])/-yw[i])
                if yw[i]>0 and rp[i]+yw[i]>Fmax: kk=min(kk,max(0,Fmax-rp[i])/yw[i])
        F=[rp[i]+kk*yw[i] for i in range(4)]
        lo,hi=min(F),max(F)
        if lo<0: F=[f-lo for f in F]; hi-=lo
        if hi>Fmax: F=[f-(hi-Fmax) for f in F]
        F=[min(max(f,0),Fmax) for f in F]
        Tp=sum(pitch[i]*F[i]*a for i in range(4)); Tr=sum(roll[i]*F[i]*a for i in range(4)); Ty=sum(yaw[i]*km/kf*F[i] for i in range(4))
        wp+=dt*(Tp-0.005*wp)/Ixx; wr+=dt*(Tr-0.005*wr)/Izz; wy+=dt*(Ty-0.005*wy)/Iyy
        d=1/(1+dt*0.02); wp*=d; wr*=d; wy*=d
        mag=math.sqrt(wp*wp+wy*wy+wr*wr)
        if mag>6: s=6/mag; wp,wy,wr=wp*s,wy*s,wr*s
        thp+=dt*wp; thr+=dt*wr
        vz+=dt*(sum(F)*math.cos(thr)*math.cos(thp)-m*g-0.12*vz)/m; z+=dt*vz
        if ycmd: tgt=z
        zmax=max(zmax,abs(z)); ymax=max(ymax,abs(wy))
    return z,zmax,wy,ymax
frames=[("stock",2.0,0.003,0.006,0.003,0.16),("cube quad",1.7,0.006167,0.010667,0.006167,0.2121),
        ("1.2 kg yaw-light",1.2,0.002167,0.002667,0.002167,0.0707),("0.28 kg small",0.28,0.000679,0.001191,0.000679,0.1131),
        ("big 3 kg",3.0,0.03,0.05,0.03,0.35)]
for label,f1,f2 in (("CURRENT",False,False),("FIX 1",True,False),("FIX 1+2",True,True)):
    print(label)
    for n,m,a,b,c,arm in frames:
        z,zmax,wy,_=sim(m,a,b,c,arm,f1,f2,roll_kick=0.5)
        z2,zmax2,wy2,ymax2=sim(m,a,b,c,arm,f1,f2,yaw_cmd=1.0,yaw_kick=0)
        print("  %-17s hover+kick: max|dz|=%8.3f m end yawRate=%+.3f | full yaw stick 2s: max yawRate=%.2f rad/s, max|dz|=%.3f m"%(n,zmax,wy,ymax2,zmax2))
