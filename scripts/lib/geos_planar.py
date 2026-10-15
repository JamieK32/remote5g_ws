"""Minimal GEOS C API adapter for exact planar partitions (GEOS >= 3.10)."""
import ctypes as C
from ctypes.util import find_library

class Planar:
    def __init__(self):
        library = find_library('geos_c')
        if not library:
            raise RuntimeError('GEOS C library required: install libgeos-c1v5 in WSL.')
        self.lib = C.CDLL(library)
        p, u, d = C.c_void_p, C.c_uint, C.c_double
        signatures = {
            'GEOS_init_r': (p, []), 'GEOS_finish_r': (None, [p]),
            'GEOSWKTReader_create_r': (p, [p]),
            'GEOSWKTReader_read_r': (p, [p,p,C.c_char_p]),
            'GEOSWKTReader_destroy_r': (None, [p,p]),
            'GEOSGeom_destroy_r': (None, [p,p]),
            'GEOSGeom_setPrecision_r': (p,[p,p,d,C.c_int]),
            'GEOSUnion_r': (p,[p,p,p]), 'GEOSDifference_r': (p,[p,p,p]),
            'GEOSIntersection_r': (p,[p,p,p]),
            'GEOSDelaunayTriangulation_r': (p,[p,p,d,C.c_int]),
            'GEOSGetNumInteriorRings_r': (C.c_int,[p,p]),
            'GEOSGeomTypeId_r': (C.c_int,[p,p]),
            'GEOSGetNumGeometries_r': (C.c_int,[p,p]),
            'GEOSGetGeometryN_r': (p,[p,p,C.c_int]),
            'GEOSGetExteriorRing_r': (p,[p,p]),
            'GEOSGeom_getCoordSeq_r': (p,[p,p]),
            'GEOSCoordSeq_getSize_r': (C.c_int,[p,p,C.POINTER(u)]),
            'GEOSCoordSeq_getX_r': (C.c_int,[p,p,u,C.POINTER(d)]),
            'GEOSCoordSeq_getY_r': (C.c_int,[p,p,u,C.POINTER(d)]),
            'GEOSArea_r': (C.c_int,[p,p,C.POINTER(d)]),
            'GEOSisValid_r': (C.c_char,[p,p]),
        }
        for name,(result,args) in signatures.items():
            fn=getattr(self.lib,name);fn.restype=result;fn.argtypes=args
        self.ctx=self.lib.GEOS_init_r()
        self.reader=self.lib.GEOSWKTReader_create_r(self.ctx)
        self.owned=[]
        import sys
        handler_type=C.CFUNCTYPE(None,C.c_char_p,C.c_void_p)
        self.error_handler=handler_type(lambda message,_: print('GEOS:',message.decode(),file=sys.stderr))
        fn=self.lib.GEOSContext_setErrorMessageHandler_r
        fn.restype=C.c_void_p;fn.argtypes=[C.c_void_p,handler_type,C.c_void_p]
        fn(self.ctx,self.error_handler,None)
    def keep(self,geometry):
        if not geometry: raise RuntimeError('GEOS geometry operation failed')
        self.owned.append(geometry)
        return geometry
    def polygon(self,points):
        points=list(points)
        wkt='POLYGON (('+','.join(f'{x:.9f} {y:.9f}' for x,y in points+[points[0]])+'))'
        geometry=self.keep(self.lib.GEOSWKTReader_read_r(self.ctx,self.reader,wkt.encode()))
        return self.keep(self.lib.GEOSGeom_setPrecision_r(self.ctx,geometry,1e-7,0))
    def rect(self,x0,x1,y0,y1):
        return self.polygon([(x0,y0),(x1,y0),(x1,y1),(x0,y1)])
    def union(self,a,b): return self.keep(self.lib.GEOSUnion_r(self.ctx,a,b))
    def difference(self,a,b): return self.keep(self.lib.GEOSDifference_r(self.ctx,a,b))
    def intersection(self,a,b): return self.keep(self.lib.GEOSIntersection_r(self.ctx,a,b))
    def area(self,g):
        value=C.c_double()
        if not self.lib.GEOSArea_r(self.ctx,g,C.byref(value)): raise RuntimeError('GEOS area failed')
        return value.value
    def points(self,g):
        ring=self.lib.GEOSGetExteriorRing_r(self.ctx,g)
        seq=self.lib.GEOSGeom_getCoordSeq_r(self.ctx,ring)
        size=C.c_uint(); self.lib.GEOSCoordSeq_getSize_r(self.ctx,seq,C.byref(size))
        points=[]
        for j in range(size.value-1):
            x,y=C.c_double(),C.c_double()
            self.lib.GEOSCoordSeq_getX_r(self.ctx,seq,j,C.byref(x))
            self.lib.GEOSCoordSeq_getY_r(self.ctx,seq,j,C.byref(y))
            points.append((x.value,y.value))
        return points
    def simple_triangles(self,g):
        if self.area(g)<1e-12: return
        if self.lib.GEOSGeomTypeId_r(self.ctx,g) in (6,7):
            for i in range(self.lib.GEOSGetNumGeometries_r(self.ctx,g)):
                yield from self.simple_triangles(self.lib.GEOSGetGeometryN_r(self.ctx,g,i))
            return
        if self.lib.GEOSGetNumInteriorRings_r(self.ctx,g):
            raise RuntimeError('Unexpected hole in clipped Delaunay cell')
        points=self.points(g)
        def cross(a,b,c):
            return (b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0])
        signed=sum(a[0]*b[1]-b[0]*a[1] for a,b in zip(points,points[1:]+points[:1]))
        if signed<0: points.reverse()
        while len(points)>3:
            for i,b in enumerate(points):
                a,c=points[i-1],points[(i+1)%len(points)]
                if abs(cross(a,b,c))<1e-12:
                    points.pop(i); break
                if cross(a,b,c)<=0: continue
                other=[q for j,q in enumerate(points) if j not in ((i-1)%len(points),i,(i+1)%len(points))]
                if any(cross(a,b,q)>=-1e-12 and cross(b,c,q)>=-1e-12 and cross(c,a,q)>=-1e-12 for q in other): continue
                yield [a,b,c];points.pop(i);break
            else: raise RuntimeError('Cannot triangulate clipped cell')
        if len(points)==3 and cross(*points)>1e-12: yield points
    def triangles(self,g):
        if self.area(g)<1e-12: return
        if self.lib.GEOSGeomTypeId_r(self.ctx,g) in (6,7):
            for i in range(self.lib.GEOSGetNumGeometries_r(self.ctx,g)):
                yield from self.triangles(self.lib.GEOSGetGeometryN_r(self.ctx,g,i))
            return
        if self.lib.GEOSGetNumInteriorRings_r(self.ctx,g)==0:
            yield from self.simple_triangles(g)
            return
        # GEOS 3.10 CDT has failures on very narrow interconnected paint regions.
        # Delaunay plus exact polygon clipping covers every region without overlap.
        collection=self.keep(self.lib.GEOSDelaunayTriangulation_r(self.ctx,g,0.0,0))
        for i in range(self.lib.GEOSGetNumGeometries_r(self.ctx,collection)):
            triangle=self.lib.GEOSGetGeometryN_r(self.ctx,collection,i)
            yield from self.simple_triangles(self.intersection(triangle,g))
    def close(self):
        for g in reversed(self.owned): self.lib.GEOSGeom_destroy_r(self.ctx,g)
        self.lib.GEOSWKTReader_destroy_r(self.ctx,self.reader)
        self.lib.GEOS_finish_r(self.ctx)
