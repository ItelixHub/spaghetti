
import os
from OCC.Extend.DataExchange import write_stl_file
from OCC.Core.STEPControl import STEPControl_Writer, STEPControl_AsIs
from OCC.Core.Interface import Interface_Static_SetCVal
from OCC.Core.BRep import BRep_Builder
from OCC.Core.TopoDS import TopoDS_Compound
from OCC.Display.SimpleGui import init_display
from OCC.Core.STEPControl import STEPControl_Writer, STEPControl_AsIs
from OCC.Core.Interface import Interface_Static_SetCVal
from OCC.Core.BRep import BRep_Builder
from OCC.Core.TopoDS import TopoDS_Compound

tree_geo = TopoDS_Compound()
res = []
  
def write_step(display, shp, res, infile, TREE_ONLY, MERGE):
    aBuilder = BRep_Builder()
    aBuilder.MakeCompound(tree_geo)
    
    display.EraseAll()
    step_writer = STEPControl_Writer()
    Interface_Static_SetCVal("write.step.schema", "AP203")
    Interface_Static_SetCVal("write.step.assembly", "0")
    # transfer shapes and write file
    if TREE_ONLY == False:
        step_writer.Transfer(shp, STEPControl_AsIs)
        display.DisplayShape(shp, update=True)
    if MERGE == True:
        i = 0
        for r in res:
            tree_name = 'tree_' + str(i)
            Interface_Static_SetCVal('write.step.product.name', tree_name)
            step_writer.Transfer(r, STEPControl_AsIs)
            display.DisplayShape(r, update=True)
            i += 1
    else:
        step_writer.Transfer(tree_geo, STEPControl_AsIs)
        display.DisplayShape(tree_geo, update=True)

    stp_name = str(os.path.splitext(infile)[0]) + "_tree.step" 
    status = step_writer.Write(stp_name)
    
    display.FitAll()
    return status




def write_stl(display, shp, infile, TREE_ONLY, LIN_DEF, ANG_DEF):
    stl_name = str(os.path.splitext(infile)[0]) + "_tree.stl"
    stl_ex = TopoDS_Compound()
    stlBuilder = BRep_Builder()
    stlBuilder.MakeCompound(stl_ex)
    stlBuilder.Add(stl_ex, tree_geo)
    if TREE_ONLY == False:
        stlBuilder.Add(stl_ex, shp)
    # stl_ex = BRepAlgoAPI_Fuse(tree_geo, shp).Shape()
    write_stl_file(
    stl_ex,
    stl_name,
    mode="binary",
    linear_deflection=LIN_DEF,
    angular_deflection=ANG_DEF,
    )
    print(f"STL file {stl_name} saved.")