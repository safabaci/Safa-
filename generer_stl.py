#!/usr/bin/env python3
import struct, math, os

OUTPUT = os.path.expanduser("~/dmg_mori_ws/src/dmg_mori_5axis/meshes/")
os.makedirs(OUTPUT, exist_ok=True)

def normale(v0, v1, v2):
    ax=v1[0]-v0[0]; ay=v1[1]-v0[1]; az=v1[2]-v0[2]
    bx=v2[0]-v0[0]; by=v2[1]-v0[1]; bz=v2[2]-v0[2]
    nx=ay*bz-az*by; ny=az*bx-ax*bz; nz=ax*by-ay*bx
    lg=math.sqrt(nx*nx+ny*ny+nz*nz) or 1
    return nx/lg, ny/lg, nz/lg

def ecrire_stl(nom, tris):
    with open(OUTPUT+nom, "wb") as f:
        f.write(b"\x00"*80)
        f.write(struct.pack("<I", len(tris)))
        for t in tris:
            v0,v1,v2=t
            n=normale(v0,v1,v2)
            f.write(struct.pack("<fff",*n))
            for v in (v0,v1,v2):
                f.write(struct.pack("<fff",*v))
            f.write(b"\x00\x00")
    print("OK "+nom)

def quad(a,b,c,d):
    return [(a,b,c),(a,c,d)]

def cercle(cx,cy,z,r,n):
    return [(cx+r*math.cos(2*math.pi*i/n), cy+r*math.sin(2*math.pi*i/n), z) for i in range(n)]

def gen_bloc():
    t=[]
    x=y=z=0.040
    t+=quad((-x,-y,0),(x,-y,0),(x,y,0),(-x,y,0))
    t+=quad((-x,-y,z),(x,y,z),(x,-y,z),(-x,y,z))
    t+=quad((-x,-y,0),(-x,-y,z),(x,-y,z),(x,-y,0))
    t+=quad((-x,y,0),(x,y,0),(x,y,z),(-x,y,z))
    t+=quad((-x,-y,0),(-x,y,0),(-x,y,z),(-x,-y,z))
    t+=quad((x,-y,0),(x,-y,z),(x,y,z),(x,y,0))
    return t

def gen_ebauche():
    t=[]; r=0.038; h=0.036; n=8
    b=cercle(0,0,0,r,n); tp=cercle(0,0,h,r,n)
    for i in range(n):
        j=(i+1)%n
        t+=quad(b[i],b[j],tp[j],tp[i])
        t.append(((0,0,0),b[j],b[i]))
        t.append(((0,0,h),tp[i],tp[j]))
    return t

def gen_semi():
    t=[]; r1=0.035; r2=0.025; h1=0.030; h2=0.020; n=32
    b1=cercle(0,0,0,r1,n); t1=cercle(0,0,h1,r1,n)
    b2=cercle(0,0,h1,r2,n); t2=cercle(0,0,h1+h2,r2,n)
    for i in range(n):
        j=(i+1)%n
        t+=quad(b1[i],b1[j],t1[j],t1[i])
        t.append(((0,0,0),b1[j],b1[i]))
        t.append(((0,0,h1),t1[i],t1[j]))
        t+=quad(b2[i],t1[i],t1[j],b2[j])
        t+=quad(b2[i],b2[j],t2[j],t2[i])
        t.append(((0,0,h1),b2[j],b2[i]))
        t.append(((0,0,h1+h2),t2[i],t2[j]))
    return t

def gen_finie():
    t=[]; r=0.030; h=0.025; rp=0.012; hp=0.010; n=48
    ext=cercle(0,0,0,r,n); top=cercle(0,0,h,r,n)
    ri=cercle(0,0,h,rp,n); ro=cercle(0,0,h,r,n)
    pb=cercle(0,0,h-hp,rp,n); ph=cercle(0,0,h,rp,n)
    for i in range(n):
        j=(i+1)%n
        t+=quad(ext[i],ext[j],top[j],top[i])
        t.append(((0,0,0),ext[j],ext[i]))
        t+=quad(ri[i],ro[i],ro[j],ri[j])
        t+=quad(pb[i],pb[j],ph[j],ph[i])
        t.append(((0,0,h-hp),pb[j],pb[i]))
    return t

print("Generation STL...")
ecrire_stl("piece_brute.stl", gen_bloc())
ecrire_stl("piece_ebauche.stl", gen_ebauche())
ecrire_stl("piece_semifinie.stl", gen_semi())
ecrire_stl("piece_finie.stl", gen_finie())
print("Termine! Fichiers dans: "+OUTPUT)
