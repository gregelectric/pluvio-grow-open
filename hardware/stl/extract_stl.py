#!/usr/bin/env python3
"""Regenerate the printed STLs from the authoritative STEP assembly.

The STEP file (../step/pluvio_m5_assembly.STEP) is the single source of truth. This
reads the named printed solids out of it, meshes them, and writes binary STLs
with builder-friendly names next to this script. Requires OpenCASCADE's Python
bindings (OCP), which ship with cadquery:  pip install cadquery

Run from the hardware/stl/ folder:  python extract_stl.py
"""
import os, sys, time

from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.TDocStd import TDocStd_Document
from OCP.TCollection import TCollection_ExtendedString
from OCP.XCAFDoc import XCAFDoc_DocumentTool, XCAFDoc_ShapeTool
from OCP.TDF import TDF_LabelSequence
from OCP.TDataStd import TDataStd_Name
from OCP.TopoDS import TopoDS_Shape
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.StlAPI import StlAPI_Writer
from OCP.Bnd import Bnd_Box
from OCP.BRepBndLib import BRepBndLib

# Map each printed solid's name in the SolidWorks assembly to a
# builder-friendly output filename.
PARTS = {
    "duct":               "carousel_disk",
    "duct_bott":          "housing_body",
    "duct_top":           "housing_shroud",
    "duct_base":          "base_plate",
    "duct_tank_platform": "tank_platform",
    "duct_aquaduct":      "dosing_spout",
    "duct_mount":         "motor_mount",
    "duct_shim":          "leveling_shim",
}

HERE = os.path.dirname(os.path.abspath(__file__))
STEP = os.path.join(HERE, "..", "step", "pluvio_m5_assembly.STEP")
OUT = HERE


def dims(shape):
    b = Bnd_Box(); BRepBndLib.Add_s(shape, b)
    xmin, ymin, zmin, xmax, ymax, zmax = b.Get()
    return xmax - xmin, ymax - ymin, zmax - zmin


def main(step=STEP, out=OUT):
    t = time.time()
    doc = TDocStd_Document(TCollection_ExtendedString("doc"))
    r = STEPCAFControl_Reader(); r.SetNameMode(True)
    r.ReadFile(step)
    r.Transfer(doc)
    st = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
    labs = TDF_LabelSequence(); st.GetShapes(labs)

    done = set()
    for i in range(1, labs.Length() + 1):
        lab = labs.Value(i); nm = TDataStd_Name()
        if not lab.FindAttribute(TDataStd_Name.GetID_s(), nm):
            continue
        name = nm.Get().ToExtString()
        if name not in PARTS or name in done:
            continue
        shape = TopoDS_Shape()
        if not XCAFDoc_ShapeTool.GetShape_s(lab, shape):
            continue
        BRepMesh_IncrementalMesh(shape, 0.3, False, 0.5, True)
        out_name = PARTS[name]
        path = os.path.join(out, out_name + ".stl")
        w = StlAPI_Writer()
        try:
            w.ASCIIMode = False  # binary STL
        except Exception:
            pass
        ok = w.Write(shape, path)
        d = dims(shape); sz = os.path.getsize(path) if os.path.exists(path) else 0
        print(f"  {'OK ' if ok else 'ERR'} {out_name:16s} (cad: {name:18s}) "
              f"{d[0]:5.0f}x{d[1]:5.0f}x{d[2]:5.0f}mm  {sz // 1024}KB")
        done.add(name)
    missing = [p for p in PARTS if p not in done]
    if missing:
        print("missing:", missing)
    print("done in %.1fs" % (time.time() - t))


if __name__ == "__main__":
    args = sys.argv[1:]
    main(*(args or [STEP]))
