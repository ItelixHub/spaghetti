import trimesh
import trimesh.creation
import trimesh.repair
import trimesh.ray.ray_triangle as ray
import trimesh.exchange.load
import numpy as np
from sklearn.neighbors import KDTree
from sklearn.cluster import MeanShift
import time
import wx
import sys
import os
import logging
import math

from config_manager import FDIALOG, ANIMATION, RAD_ALGO, MERGE, TREE_ONLY, AUTO_STEP, OVERHANG_ANGLE, MAX_BRANCH_LENGTH, GROUND, GRID_STEP, POINT_OVERHANG_TOLERANCE, K, RADIUS, RADIUS_FACTOR, MIN_AREA, MESH_SIZE_MAX, MESH_SIZE_MIN, CROWN_RAD, PT_SORT, OVERHANG_ANGLE_EDGE, LIN_DEF, ANG_DEF, CALC_ADD_PTS, CSV_IMP, SHIFT_CONUS 
from csv_manager import manage_csv_import

from file_exporter import write_step, write_stl

from menu_manager import MenuManager

from OCC.Core.gp import gp_Pnt, gp_Ax2, gp_Dir, gp_Circ #, gp_Vec
from OCC.Core.BRepBuilderAPI import (
BRepBuilderAPI_MakeEdge,       
BRepBuilderAPI_MakeWire,
BRepBuilderAPI_MakeFace,
)
from OCC.Extend.DataExchange import write_stl_file
from OCC.Core.BRepOffsetAPI import BRepOffsetAPI_MakePipe
from OCC.Core.BRepPrimAPI import (BRepPrimAPI_MakeSphere, BRepPrimAPI_MakeBox, BRepPrimAPI_MakeCone)
from OCC.Display.SimpleGui import init_display
from OCC.Core.STEPControl import STEPControl_Writer, STEPControl_AsIs   # TODO Harmonize the STEP Reader / Writer (STEPContol or DataExchange)
from OCC.Extend.DataExchange import read_step_file
from OCC.Core.Interface import Interface_Static_SetCVal
from OCC.Core.BRep import BRep_Builder
from OCC.Core.TopoDS import TopoDS_Compound
from OCC.Display.OCCViewer import rgb_color
from OCC.Core.BRepAlgoAPI import BRepAlgoAPI_Fuse
from OCC.Extend.ShapeFactory import get_aligned_boundingbox
from OCC.Core.Quantity import (Quantity_NOC_RED, Quantity_NOC_ORANGE, Quantity_NOC_TOMATO, Quantity_NOC_STEELBLUE, Quantity_NOC_NAVYBLUE, Quantity_NOC_SPRINGGREEN4, Quantity_NOC_TURQUOISE3,)
from general_utilities import farbe

app = wx.App(False)
csv_data, csv_file_path, CSV_IMP = manage_csv_import(app, CSV_IMP)

VEC_Z = np.array([0,0,+1])
supp_angle_rad = np.radians(OVERHANG_ANGLE)
sin_alpha = np.sin(np.radians(90 - OVERHANG_ANGLE))
sin_beta = np.sin(supp_angle_rad)
A1 = np.radians(88)
A2 = np.radians(0)
L = logging.getLogger('trimesh')
L.setLevel(logging.CRITICAL)
gmshargs = [("General.Verbosity",1),
            ("General.Terminal",0),
            ("Geometry.OCCFixDegenerated",1),
            ("Mesh.MeshSizeMin", MESH_SIZE_MIN),
            ("Mesh.MeshSizeMax", MESH_SIZE_MAX)] #,("Mesh.Algorithm",1)]


def exit(event=None):
    sys.exit()

def OverhangEdges(geo):
    theta_face = []
    edgepts = []
    adjacentFaceAngles = np.rad2deg(geo.face_adjacency_angles) # Values for adjacent faces
    sharp = abs(adjacentFaceAngles) > OVERHANG_ANGLE # Boolean filter for adjacent faces
    sharp_edges = geo.face_adjacency_edges[sharp] # Vertex indices which correspond to face_adjacency
    sharp_faces = geo.face_adjacency[sharp] # Get indicies of faces from sharp edges
    sharp_normals = geo.face_normals[sharp_faces]
    
    z_vectors = np.full((len(sharp_normals),2,3),[[0., 0., 1.0],[0., 0., 1.0]])
    # Calculate the 2 angles between face normals and Z axis
    theta_face = np.empty([len(sharp_normals),2])
    theta_face[:,0] = np.arccos(np.sum(
        geo.face_normals[sharp_faces][:,0] * z_vectors[:,0], axis=1) /
                                (np.linalg.norm(sharp_normals[:,0], axis=1) * np.linalg.norm(z_vectors[:,0], axis=1)))
    theta_face[:,1] = np.arccos(np.sum(geo.face_normals[sharp_faces][:,1] * z_vectors[:,0], axis=1) /
                                (np.linalg.norm(sharp_normals[:,1], axis=1) * np.linalg.norm(z_vectors[:,0], axis=1)))
    theta_face = np.degrees(theta_face)

    cont_edges = geo.vertices[sharp_edges]

    normals = geo.face_normals[sharp_faces[:, 0]]
    origins = geo.vertices[sharp_edges[:, 0]]
    unshared_sharp = geo.face_adjacency_unshared[sharp]
    vid_other = unshared_sharp[:, 1]
    vector_other = geo.vertices[vid_other] - origins
    a = np.asanyarray(vector_other)
    dots = np.dot(a * normals, [1.0] * a.shape[1])
    
    """calculate angle between edge and Z axis"""
    v1 = cont_edges[:,0] - cont_edges[:,1]
    z_axis = np.array([0,0,1])
    dot_product = np.dot(v1, z_axis)
    
    cos_angles = dot_product / (np.linalg.norm(v1, axis=1) * np.linalg.norm(z_axis))
    theta_edge = np.rad2deg(np.arccos(cos_angles))
    theta_edge = np.where(theta_edge > 90, (180 - theta_edge), theta_edge)

    n = 0

    for e in cont_edges:
        if all([theta_edge[n] > OVERHANG_ANGLE_EDGE, 
                np.all((np.round(theta_face[n], 3) >= 90)), # and angle of both face normals greater 90°
                dots[n] < (POINT_OVERHANG_TOLERANCE / 100)  # and edge is convex (negativ or zero projection)
                ]):
            nbPts = round((np.linalg.norm(e[0]- e[1])) / GRID_STEP)
            eq_pts = np.column_stack((
                np.linspace(e[0][0], e[1][0], nbPts+1),
                np.linspace(e[0][1], e[1][1], nbPts+1),
                np.linspace(e[0][2], e[1][2], nbPts+1)))
            # mask = (eq_pts[:,2]>(GROUND + POINT_OVERHANG_TOLERANCE))
            mask = (eq_pts[:,2] > GROUND)
            eq_pts = eq_pts[mask, :].tolist()
            edgepts.extend(eq_pts)
        n += 1

    return edgepts

def findOverhangPoints(part):
    meshVerts = part.vertices
    vAdjacency = part.vertex_neighbors

    pointOverhangs = []
    for i in range(len(vAdjacency)):
        # Find the edge deltas between the points
        v = meshVerts[i]
        neighborVerts = meshVerts[vAdjacency[i], :]
        delta = neighborVerts - v
  
        if np.all(delta[:, 2] > POINT_OVERHANG_TOLERANCE):

            if part.vertex_normals[i][2] < 0.0 and v[2] > GROUND:
                pointOverhangs.append([v[0],v[1],v[2]])
    return pointOverhangs

def Support_Grid(mesh):

    x_supp = np.arange(np.min(mesh.vertices[:,0] + 1), np.max(mesh.vertices[:,0] + 1), GRID_STEP)
    y_supp = np.arange(np.min(mesh.vertices[:,1] + 1), np.max(mesh.vertices[:,1] + 1), GRID_STEP)
    nb_coords = x_supp.shape[0] * y_supp.shape[0]
    # set the start points for the raytraycing to Z -2
    z_supp = np.full((nb_coords,1),-2)
    # create coordinates for each x and y position
    xv, yv = np.meshgrid(x_supp, y_supp)
    xv = xv.reshape(nb_coords,1)
    yv = yv.reshape(nb_coords,1)
    # attach the z value to the xy coordinates
    xyz = np.hstack((xv,yv,z_supp))
    # create z vector for each xy coordinate
    dv = np.full((nb_coords,3),[0,0,1])
    ray_grid = [xyz, dv]
    return ray_grid

def angle_between_z(p0, p1):
    v1_u = (p1 - p0) / np.linalg.norm((p1 - p0))
    return np.degrees(np.arccos(np.clip(np.dot(v1_u, -VEC_Z), -1.0, 1.0)))
    
def branch_loop(locations, k, radius):
    global tree_geo, args, t_args, branches 
    temp_pts = []
    branch_tmp_pt = []
    nb_locations = len(locations)
    single_pts = []
    
    if nb_locations < k:
        k = nb_locations
  
    match PT_SORT:
            case 0:
                #Z,Y,X
                locations = locations[np.lexsort((locations[:,0],
                                    locations[:,1],
                                    -locations[:,2]))]
            case 1:
                #X,Y,Z
                locations = locations[np.lexsort((locations[:,2],
                                    locations[:,1],
                                    locations[:,0]))]
            case 2:
                #Y,X,Z
                locations = locations[np.lexsort((locations[:,0],
                                    locations[:,2],
                                    locations[:,1]))] 
            case 3:
                sort_idx = np.argsort(np.einsum('ij,ij->i', locations - locations[0], locations - locations[0]))
                locations = locations[sort_idx]
    
    while nb_locations > 0:
        if nb_locations > 1:
            # search with KDTree for k closest points
            tree = KDTree(locations, leaf_size=8)
            distances, indices = tree.query(locations, k)
            # check the distance between points for one branch_point in the first level
            if radius == RADIUS and np.any(distances[0,0:k] > GRID_STEP * 5):
                print(farbe.YELLOW + "Abstand zu groß" + farbe.END)
                far_pt = np.argmax(distances[0,0:k] > GRID_STEP * 5) #- 1
                poly = locations[indices[0,0:far_pt]]
            else:
                # get coordinates of first k points
                poly = locations[indices[0,0:k]]
            
            #  get center point of polygon
            branch_point = np.mean(poly, 0)
            # set Z coordinate to lowest point -1 
            branch_point[2] = np.min(poly[:,2]) - 1
            double_s = False
            branch_color = None
        else:
            # 1 or 0 points
            poly = np.array([locations[0]])
            indices = np.array([[0]])
            # create sphere on both ends of branch
            double_s = True
            branch_color = Quantity_NOC_TOMATO
            # temp_pts is empty
            if len(temp_pts) == 0:
                branch_point = np.reshape(poly[[0],:], 3) # use "advanced indexing" to create a COPY of poly
                branch_point[2] = GROUND
            else:
                single_pts.append((poly[0,0],poly[0,1], poly[0,2]))
                locations = np.delete(locations, indices[0], 0)
                nb_locations = len(locations)
                continue
                           
        # calculate angles between branch_point / polygon points and Z axis
        temp_Z = [VEC_Z] * len(poly)
        b_vec = poly - ([branch_point] * len(poly))
        b_angles =np.arccos(np.sum(b_vec * temp_Z, axis=1) 
                            / (np.linalg.norm(b_vec, axis=1) 
                            * np.linalg.norm(temp_Z, axis=1)))
        
        # if support angle is exceeded, move branch_point down
        if np.max(b_angles) > supp_angle_rad:
            # get distances between branch_point and polygon points
            branch_length = np.linalg.norm(poly-branch_point, axis=1)
            # Search farthest point
            b = np.max(branch_length)
            # Calculate the offset in Z with sine theorem
            z_offset = (b * sin_alpha) / sin_beta
            # branch_point[2] = branch_point[2] - z_offset      
            branch_point = np.array([branch_point[0], branch_point[1], branch_point[2] - z_offset])
        branch_tmp_pt = branch_point
        
        """ Check for intersection with part to detect branch_points inside the part"""
        if not np.array_equal(np.reshape(branch_point, (1,3)), poly):   # check if branch_point = poly
            _branch_points, branch_idx = hull_itersec(branch_point, poly)
        else: 
            _branch_points = []
            branch_idx = []
            
        """Generate geometry"""    
        _branches = TopoDS_Compound()
        aBuilder.MakeCompound(_branches)
        
        cancel_b_pt = 0
        b_counter = 0
        for end_pt in poly:
            use_trunk = False  
            # Check if for the point in poly a intersection with the part exists          
            if b_counter in branch_idx[:]:
                # set branch_tmp_pt to the intersection
                branch_tmp_pt = _branch_points[branch_idx.index(b_counter)]
                double_s = True
                branch_color = Quantity_NOC_TURQUOISE3

             # Check if branch_point is below GROUND
            if branch_point[2] <= (GROUND):
                # create a vertical branch / trunk to GROND
                branch_tmp_pt = (end_pt[0], end_pt[1], GROUND)
                branch_color = Quantity_NOC_SPRINGGREEN4
                double_s = False
                use_trunk = True
            
            b_length = np.linalg.norm(branch_tmp_pt - end_pt)
            # Increase radius for long branches
            if b_length > MAX_BRANCH_LENGTH:
                radius = radius * 1.25
                print(f"Radius increased {radius:.2f}")  
            if b_length <= 1:
                print(f"Short branch skiped {b_length}")
                continue
                
            if end_pt[2] < b_length:
                branch_tmp_pt = (end_pt[0], end_pt[1], GROUND)
                # branch_point = [0,0,-1]
                branch_color = Quantity_NOC_RED
                double_s = False
                cancel_b_pt +=1
                use_trunk = True
            
            if use_trunk:
                branch = trunk_gen(np.reshape(end_pt, (1,3)), radius)
            else:
                branch = branch_gen(gp_Pnt(branch_tmp_pt[0],branch_tmp_pt[1], branch_tmp_pt[2]),
                                    gp_Pnt(end_pt[0], end_pt[1], end_pt[2]),
                                    radius,
                                    double_s,
                                    branch_color)  
            if MERGE == True:
                    if _branches is None:
                        _branches = branch
                    _branches = BRepAlgoAPI_Fuse(_branches,branch).Shape()
            b_counter += 1
        
        # save branch_points for  next branch_loop
        # ignore points below GROUND or all branches intersects the part
        if branch_point[2] >= (GROUND) and len(branch_idx) != k and cancel_b_pt != b_counter:
            temp_pts.append(tuple((branch_point[0], branch_point[1], branch_point[2])))
      
        # remove used points from locations
        locations = np.delete(locations, indices[0], 0)
        nb_locations = len(locations)
        
        if nb_locations < k:
            k = nb_locations
    if len(single_pts) > 0:
        temp_pts.extend(single_pts)    
    return np.array(temp_pts)


def branch_gen(point1, point2, radius, double_s=False, col=None, root_radius=None, trunk_length=None):
    dir = gp_Dir(point2.X() - point1.X(),
                 point2.Y() - point1.Y(),
                 point2.Z() - point1.Z())
    axis = gp_Ax2(point2,-dir)
    # test if point2 is in a_points ( overhang edges / points)
    m_pts_test = []
    if CALC_ADD_PTS:
        m_pts_test = np.where((a_points == point2.Coord()).all(axis=1))[0]
    if len(m_pts_test) == 0:
        end_shape = BRepPrimAPI_MakeSphere(axis, radius, -A1, A2 ).Shape()
    # create a cone instead of a sphere on edge points
    else:
        pt2 = (point2.X(), point2.Y(),point2.Z())
        vec = dir.Coord()
        unit_vec = vec / np.linalg.norm(vec)
        t_pt = pt2 - (radius-SHIFT_CONUS) * unit_vec
        disp_pt = gp_Pnt(t_pt[0], t_pt[1], t_pt[2])
        axis = gp_Ax2(disp_pt, dir)
        end_shape = BRepPrimAPI_MakeCone(axis, radius, 0, radius).Shape()
        point2 = disp_pt
        
        global cc
        cc += 1
    # create cylindrical branch
    if root_radius is None:
        makeWire = BRepBuilderAPI_MakeWire()
        edge = BRepBuilderAPI_MakeEdge(point2, point1).Edge()
        makeWire.Add(edge)
        makeWire.Build()
        wire = makeWire.Wire()
        circle = gp_Circ(axis, radius)
        profile_edge = BRepBuilderAPI_MakeEdge(circle).Edge()
        profile_wire = BRepBuilderAPI_MakeWire(profile_edge).Wire()
        profile_face = BRepBuilderAPI_MakeFace(profile_wire).Face()
        p = BRepOffsetAPI_MakePipe(wire, profile_face).Shape()
        p = BRepAlgoAPI_Fuse(end_shape,p).Shape()
    # create conical branch / trunk
    else:
        col = Quantity_NOC_ORANGE
        pt2 = (point2.X(), point2.Y(),point2.Z())
        vec = dir.Coord()
        unit_vec = vec / np.linalg.norm(vec)
        t_pt = pt2 - radius * unit_vec
        p = BRepPrimAPI_MakeCone(axis, radius, root_radius, trunk_length).Shape()
        p = BRepAlgoAPI_Fuse(end_shape,p).Shape()
        # point2 = disp_pt
    
    if double_s == True and point1.Coord()[2] > GROUND:
        axis2 = gp_Ax2(point1,dir)
        if root_radius is None:
            s2 = BRepPrimAPI_MakeSphere(axis2, radius, -A1, A2 ).Shape()
        else:
            s2 = BRepPrimAPI_MakeSphere(axis2, root_radius, -A1, A2 ).Shape()
        p = BRepAlgoAPI_Fuse(s2,p).Shape()
         
    display.DisplayShape(p, update=ANIMATION, color=col)
    if MERGE == False:
        aBuilder.Add(tree_geo, p)
    return p

def trunk_gen(temp_pts, radius):
    root_radius = None
    # Calculate intersection with part
    trunk_pt, trunk_idx = hull_itersec((temp_pts[0,0], temp_pts[0,1], GROUND), temp_pts[[0]])
    if len(trunk_pt) == 0:
        trunk_length = 0
    else:    
        trunk_length = np.linalg.norm(temp_pts[0] - trunk_pt)
    # Increase radius for long trunks
    if MAX_BRANCH_LENGTH < trunk_length:# < MAX_BRANCH_LENGTH * 1.5:
        # radius *= 1.25
        root_radius = radius * 1.25
        print(farbe.CYAN + f"Long trunk, radius increased {radius:.2f}" + farbe.END)
    if trunk_length > MAX_BRANCH_LENGTH * 1.5:
        if radius < 1.5:
            root_radius = radius * 1.5
        else:
            root_radius = radius * 2.5
        print(farbe.RED + f"Extra long trunk, radius increased {radius:.2f}" + farbe.END)
        
    if len(trunk_idx) != 0:
        if np.isclose(trunk_pt[0][2],GROUND):
            # If trunk ends close to GROUND don't create a sphere at the end
            trunk = branch_gen(gp_Pnt(trunk_pt[0][0], trunk_pt[0][1], trunk_pt[0][2]),
                        gp_Pnt(temp_pts[0,0], temp_pts[0,1], temp_pts[0,2]),
                        radius,
                        False,
                        Quantity_NOC_NAVYBLUE,
                        root_radius,
                        trunk_length)
        else:
            trunk = branch_gen(gp_Pnt(trunk_pt[0][0], trunk_pt[0][1], trunk_pt[0][2]),
                            gp_Pnt(temp_pts[0,0], temp_pts[0,1], temp_pts[0,2]),
                            radius,
                            True,
                            Quantity_NOC_NAVYBLUE,
                            root_radius,
                            trunk_length)
    else:
        print(farbe.PURPLE + "keine Projetion" + farbe.END)    
        trunk = branch_gen(gp_Pnt(temp_pts[0,0], temp_pts[0,1], GROUND),
                        gp_Pnt(temp_pts[0,0], temp_pts[0,1], temp_pts[0,2]),
                        radius,
                        False,
                        'CYAN1')
    aBuilder.Add(tree_geo, trunk)
    return trunk

def crown_gen(p_locations, k_):
    radius = RADIUS
    while len(p_locations) > 1:
        if len(p_locations) < k_:
            k_ = len(p_locations)
        print(f"Branch radius: {radius:.2f}") 
        temp_pts = branch_loop(p_locations, k_, radius)
        if len(temp_pts) < (1):
            p_locations = []
        else:
            p_locations = np.array(temp_pts[(temp_pts[:,2] > GROUND)])
            # increase the radius
            radius = inc_radius(radius)
          
    return p_locations, radius

def line_pt_pt(pt1, pt2, col=None):
    my_line = BRepBuilderAPI_MakeEdge(pt1, pt2).Edge()
    display.DisplayShape(my_line, update=ANIMATION, color=col)
    return my_line

def sphere(centre, radius, col=None):
    sphere = BRepPrimAPI_MakeSphere (centre, radius ).Shape()
    display.DisplayShape(sphere, update=ANIMATION, color=col)
      
def tag_edge(_edge, msg, _color=(0, 0, 1)):
    """tag an edge"""
    center_pt = get_aligned_boundingbox(_edge)[0]
    display.DisplayMessage(center_pt, msg, 20, _color, False)

def tag_faces(_shape, _color, shape_name):
    """tag the faces of a shape

    in this example, this easy to see which faces of the 2 shapes we need to glue together
    so by reading the tagged faces, its easy to find the correct index for the
    list `facesA` (5) and `facesB` (4)

    :param _shape: the shape to tag
    :param color: a string, to colors the faces the `_shape` we're exploring
    :param shape_name: name to tag the faces the `_shape`
    """
    for n, f in enumerate(_shape):
        # centroid of the face
        center_pt = get_aligned_boundingbox(f)[0]
        # displays the face in the viewer
        display.DisplayShape(f, color=_color, transparency=0.9)
        # tag the face in the viewer
        display.DisplayMessage(center_pt, "{0}_nr_{1}".format(shape_name, n))

def select_file():
    frame = wx.Frame(None, -1)
    frame.SetSize(0,0,200,50)
    # Create open file dialog
    openFileDialog = wx.FileDialog(frame, "Select STEP file", "", "", "STEP files (*.stp;*.step)|*.step;*.stp", wx.FD_OPEN | wx.FD_FILE_MUST_EXIST)
    openFileDialog.ShowModal()
    file = openFileDialog.GetPath()
    openFileDialog.Destroy()
    return file


# status = write_step(display, shp, res, infile, TREE_ONLY, MERGE)
# write_stl(display, shp, infile, TREE_ONLY, LIN_DEF, ANG_DEF)
#TODO: Muss eigentlich weiter nach unten, stört dann aber bei der Baumkonstruktion


def support_mesh(imp_part):
    """Calculate angle between face normal and Z axis"""
    v1 = imp_part.face_normals
    v0 = np.array([[0., 0., -1.0]])
    theta = np.arccos(np.clip(np.dot(v0, v1.T), -1.0, 1.0))
    theta = np.degrees(theta).flatten()
    supportFaceIds = np.argwhere(theta < 90 - OVERHANG_ANGLE).flatten()
    supported_mesh = trimesh.Trimesh(vertices=imp_part.vertices,
                                    faces=imp_part.faces[supportFaceIds])
    return supported_mesh.split(only_watertight=False)

def inc_radius(radius):
    """Set the algorithm and increase the radius"""
    match RAD_ALGO:
            case 0:
                radius = (radius + radius) * RADIUS_FACTOR
            case 1:
                radius = np.sqrt((np.pi * radius**2 * K) / np.pi) * RADIUS_FACTOR
    return radius

def line_intersection(line_point_1, line_point_2):
    """Intersection from line point_1 to point_2 with xy plane"""
    x1, y1, z1 = line_point_1
    x2, y2, z2 = line_point_2
    t = -z1 / (z2 - z1)
    return [x1 + t * (x2 - x1), y1 + t * (y2 - y1), 0]

def hull_itersec(branch_pt, points):
    """calculates the possible intersections between part and branch / trunk"""
    if points.size > 3:
        ray_dv = branch_pt - points   
    else:
        # add dummy point, intersects_id needs at least 2 points
        ray_dv = np.vstack((branch_pt - points, branch_pt - points))
        
    # move start points for raytracing into direction to branch point to avoid false positive at start point
    _poly = points + (0.0001 * ray_dv)
    # get the coordinates of first intersections and the index of the ray
    idx_tri, idx_ray, inters_loc = hull.intersects_id(ray_origins=_poly,
                                                    ray_directions=ray_dv,
                                                    return_locations=True,
                                                    multiple_hits=False)
    intersec_pts = []
    intersec_idx =[]
    i = 0
    for pt in inters_loc: 
        if is_point_on_line(branch_pt, points[idx_ray[i]], pt):
            intersec_pts.append(pt)
            intersec_idx.append(idx_ray[i]) 
        if points.size == 3:
            break       
        i += 1

    return intersec_pts, intersec_idx

def clustering(points):
    # Create trunk if only 1 point is available
    if len(points) == 1:
        radius = RADIUS
        print(f"Trunk radius: {radius:.2f}")
        trunk = trunk_gen(points, radius)    
        return trunk
    
    """create clusters of points with CROWN_RAD"""
    # TODO Adjust bin seeding if less than 20 points available to avoid warning
    ms = MeanShift(bandwidth=CROWN_RAD, bin_seeding=True)
    ms.fit(points)
    labels_unique = np.unique(ms.labels_)

    for lab_u in labels_unique:
        label_filter = ms.labels_ == lab_u
        p_locations_cluster = points[label_filter]
        k_ = K
        radius = RADIUS
        while len(p_locations_cluster) >= 1:
            if len(p_locations_cluster) < k_:
                k_ = len(points)
            print(f"Branch radius: {radius:.2f}")
            temp_pts = branch_loop(p_locations_cluster, k_, radius)
            if len(temp_pts) < 1:
                p_locations_cluster = []
            else:
                p_locations_cluster = np.array(temp_pts[(temp_pts[:,2] > GROUND)])
                # increase the radius
                radius = inc_radius(radius)
                
        if (temp_pts.shape != (0,)) and (temp_pts[0,2] > GROUND):
            print(f"Trunk radius: {radius:.2f}")
            trunk = trunk_gen(temp_pts, radius)    
    return

def is_point_on_line(p1, p2, p3):
    x1, y1, z1 = p1
    x2, y2, z2 = p2
    x3, y3, z3 = p3
    # Check if point is within bounding box of other two points
    if min(x1, x2) <= x3 <= max(x1, x2) and min(y1, y2) <= y3 <= max(y1, y2) and min(z1, z2) <= z3 <= max(z1, z2):
        return True
    
    return False
###################################################################################################################################
### Start
###################################################################################################################################
tic = time.time()   # track execution time
tree_geo = TopoDS_Compound()
aBuilder = BRep_Builder()
aBuilder.MakeCompound(tree_geo)


fuse = BRepAlgoAPI_Fuse()
b_shape = BRepAlgoAPI_Fuse()
args = []
t_args = []
res = []
_a_points = []
temp_pts = []
cc = 0
skip_mesh = 0
work_mesh = []


if FDIALOG:
    infile = select_file()  # Ich habe diese Zeile auskommentiert, da Sie sie anscheinend auskommentiert haben
    """read STEP, IGES or BREP file and convert it into a mesh"""
    # TODO Check file exist and replace gmsh load with OCC load and meshing
    stp_part = trimesh.Trimesh(**trimesh.interfaces.gmsh.load_gmsh(infile, gmsh_args=gmshargs))
    trimesh.repair.fix_inversion(stp_part, multibody=True)
    trimesh.repair.fix_normals(stp_part, multibody=True)
    trimesh.repair.fix_winding(stp_part)
    stp_part.fix_normals()
    



""" Definition of build platform
calculate extends of part in XY to defien platform, from trimesh bounding box """
bb = stp_part.bounding_box_oriented.vertices
Box = BRepPrimAPI_MakeBox(gp_Pnt((np.min(bb[:,0]) - 10), (np.min(bb[:,1]) - 10), GROUND), ((np.max(bb[:,0]) - np.min(bb[:,0])) + 20), ((np.max(bb[:,1]) - np.min(bb[:,1])) + 20), (GROUND -100)).Shape()

mbounds = [((np.min(bb[:,0]) - 20), (np.min(bb[:,1]) - 20), GROUND - 10), ((np.max(bb[:,0]) + 20), (np.max(bb[:,1]) + 20), GROUND)]
mbox = trimesh.creation.box(extents=None, transform=None, bounds=mbounds)
_hull = trimesh.util.concatenate(stp_part, mbox)
hull = ray.RayMeshIntersector(_hull)
# mbox.show()
# _hull.show()

# Create support areas
supp_mesh = support_mesh(stp_part)
shp = read_step_file(infile)

# merge part and buildplate box
Cutter = BRepAlgoAPI_Fuse(shp, Box).Shape()

display, start_display, add_menu, add_function_to_menu = init_display(size=wx.DisplaySize())
display.DisplayShape(shp, color=Quantity_NOC_STEELBLUE, transparency=0.8, update=ANIMATION)


if CALC_ADD_PTS:
# add points from overhang edges and process simulation
    print("Calculating overhang points ...")
    OverhangPt = findOverhangPoints(stp_part)
    if len(OverhangPt) > 0:
        _a_points.extend(OverhangPt)

    print("Calculating overhang edges ...")
    OverhangEdgePt = OverhangEdges(stp_part)
    if len(OverhangEdgePt) > 0:
        _a_points.extend(OverhangEdgePt)
        


# import points from process simulation
max_dev_csv = []
if CSV_IMP:
    print("Importing points ...")
    csv_data = np.genfromtxt(csv_file_path, delimiter=',',usecols=(0, 1, 2))
    
# array with additional point
a_points = np.array(_a_points)
if len(csv_data) > 0 and CALC_ADD_PTS:
    a_points = np.concatenate((a_points, csv_data))
    
for m in supp_mesh:
    # TODO Länge und Breite der Fläche checken
    if m.area < (MIN_AREA):
        skip_mesh += 1
        continue
    # Ignore meshes on or below GROUND
    if np.all(m.vertices[:,2] < (GROUND + POINT_OVERHANG_TOLERANCE)):
        skip_mesh += 1
        continue
    work_mesh.append(m)
    

if len(a_points) > 0 and len(work_mesh) > 0:
    """Calculate distance between additional points and overhang mesh"""
    distance_pt = np.zeros(shape=((np.shape(work_mesh)[0]),(len(a_points)))) #Create array filled with 0 
    i = 0
    for m in work_mesh:
        distance_pt[i] = trimesh.proximity.signed_distance(m, a_points)
        i += 1
    # find the nearest mesh for each point
    idx = np.abs(distance_pt).argmin(axis=0)

mesh_count = 0  
if len(work_mesh) > 0:
    i = 0
    for m in work_mesh:
        m_pts = []
        if len(a_points) > 0:
            pt_filter = np.where(idx == i)
            m_pts = np.squeeze(np.take(a_points,pt_filter,0),0)           
        print(f"Working on area {i} with {len(m_pts)} additionel points")
        
        # If any support surface exists, get the interface points on the support surfaces with raytracing
        p_locations = []
        xyz, dv = Support_Grid(m)
        points = ray.RayMeshIntersector(m)
        i += 1
        p_loc, index_ray, index_tri = points.intersects_location(ray_origins=xyz,
                                                                ray_directions=dv,
                                                                multiple_hit=True)
        #Continue if no points found
        if len(p_loc) < 1 and len(m_pts) < 1:
            continue
            
        if len(m_pts) > 0 and len(p_loc) > 0:
            p_loc = np.append(p_loc, m_pts, axis=0)
        
        if len(p_loc) < 1:
            p_loc = m_pts

        #Filter points out with Z below 0
        p_locations = p_loc[((p_loc[:,2] > GROUND))]
        if len(p_locations) < 1:
            continue

        branches = TopoDS_Compound() 
        aBuilder.MakeCompound(branches)
        # Cluster points and generate geometry
        trunk = clustering(p_locations)                       
        if MERGE == True and trunk:
            res.append(BRepAlgoAPI_Fuse(branches,trunk).Shape())

        mesh_count += 1

    

# Create tree if no support surface exists
if mesh_count == 0:
    branches = TopoDS_Compound()
    aBuilder.MakeCompound(branches)
    trunk = clustering(a_points)
    if MERGE == True:
        if (temp_pts[0,2] < GROUND):
            res.append(branches)
        else:
            res.append(BRepAlgoAPI_Fuse(branches,trunk).Shape())

seconds, minutes = math.modf((time.time() - tic) / 60)       
print(f"Execution time: {minutes} min. {seconds * 60:.1f} sec.")
print(f"a_points: {len(a_points)} cc: {cc} ==> {cc - len(a_points)}")
print(f"skiped meshes {skip_mesh}")

if AUTO_STEP == True:
    stp_status = write_step()



def new_func(exit, display, start_display, add_menu, add_function_to_menu):
    add_menu("BEST Tree")
    add_function_to_menu("BEST Tree", write_step)
    add_function_to_menu("BEST Tree", write_stl)
    add_function_to_menu("BEST Tree", exit)
    display.FitAll()
    start_display()
    exit()

new_func(exit, display, start_display, add_menu, add_function_to_menu)