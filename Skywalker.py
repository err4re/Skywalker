#Author- Alexander Wagner
#Description- Script creates Skywalker-like flying wing

import adsk.core, adsk.fusion, adsk.cam, traceback
import math
import os

#Design constants

#sweep in radians
SWEEP = (30/360)*2*math.pi 
#root chord length in cm
ROOT_CHORD_LENGTH = 20
#tip chord length in cm
TIP_CHORD_LENGTH = 14
#span length in cm
SPAN = 55

#aileron length in cm
AIL_LENGTH = 3.2
#aileron width in cm
AIL_WIDTH = 25
#aileron spacing in cm
AIL_SPACE = 0.1
#aileron slit in cm
AIL_SLIT = 0.5
#calculate trailing edge sweep
TRAIL_SWEEP = math.asin( (TIP_CHORD_LENGTH + (SPAN/2)*math.tan(SWEEP) - ROOT_CHORD_LENGTH) / (SPAN/2))

#distance between spars in cm
SPAR_DIST = 4
#spar grid angle in radians
SPAR_ANGLE = (45/360)*2*math.pi
#spar width in cm, maybe not needed?
SPAR_WIDTH = 0.01

#make the slicing work...
#offset between outer perimeter and inner structures in cm
OFFSET = 0.07

#profile path
this_dir = os.path.dirname(__file__)
PROFILE_PATH = os.path.join(this_dir, "MH45.txt")


#functions

#read in aerodynamic profile
def read_profile(path):
    """Reads in text file with points defining the profile and returns points.
    
    Parameters
    ----------
    path : string, mandatory
        The path where the text file describing the profile is located.
        
    Returns
    -------
    profile : list, tuple of floats (x, y)
        All coordinates of points defining the profile.
    
    """
    
    #read in profile text file
    f = open(PROFILE_PATH, "r")
    lines = f.readlines()
    x_profile = []
    y_profile = []
    
    for line in lines[1:]:
        
        x,y = line.split(" ")
        x_profile.append(float(x))
        y_profile.append(float(y))
        
    #use list, zip is emptied upon iteration    
    profile = list(zip(x_profile, y_profile)) 
    return profile
    
    
#sketch chord
def sketch_chord(sketch, length, profile, x, y):
    """Draws a chord with tip at the position (x,y), scaled by length.
    
    Parameters 
    ----------
    sketch : sketches
        sketch to draw onto (defines plane for chord)
        
    length : float
        length desired
    
    profile : list, tuple of floats (x,y)
        profile for this chord
        
    x, y : floats
        position of the tip for this chord
        
    Returns
    -------
    chord : adsk.fusion.SketchFittedSpline
        chord drawn
    
    sketch_profile : adsk.fusion.Profile
        profile corresponding to chord (required for loft)
    """
    
    #create object collection for the chord
    chord_points = adsk.core.ObjectCollection.create()
    
    for p_x, p_y in profile:
        x_chord = p_x*(length) + x
        y_chord = p_y*(length) + y
        
        chord_points.add(adsk.core.Point3D.create(x_chord, y_chord, 0))
        
    
    #Create chord
    chord = sketch.sketchCurves.sketchFittedSplines.add(chord_points)
    
    #get profile, select most recent profile contained in sketch
    sketch_profile = sketch.profiles.item(sketch.profiles.count - 1)
    
    return chord, sketch_profile


#draw line between two points
def draw_line(sketch, pnt1, pnt2):
    """Draws a line between two points, usually 3D points.

    Parameters
    ----------
    sketch : adsk.fusion.Sketch
        sketch that provides addByTwoPoints, 
        use xy_root sketch to avoid coordinate flipping

    pnt1, pnt2 : SketchPoint or Point3D

    Returns
    -------
    line : adsk.fusion.SketchLine
        line drawn between pnt1 and pnt2 using sketch
    """

    line = sketch.sketchCurves.sketchLines.addByTwoPoints(pnt1, pnt2)

    return line


#create loft
def create_loft(rootComp, sections, solid=True, rails=None):
    """Creates a loft based on collection of profiles and rails.

    Parameters
    ----------
    rootComp : adsk.fusion.Component 
        root component of active design

    sections : adsk.core.ObjectCollection
        collection of sections/profiles to loft through

    solid : boolean
        solid loft or just a surface

    rails : adsk.core.ObjectCollection
        collection of guide rails for loft

    Returns
    -------
    loft : 
        loft created
    """

    #create loft input, get sections object, get rails object
    loftFeats = rootComp.features.loftFeatures
    loftInput = loftFeats.createInput(adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    loftSectionsObj = loftInput.loftSections
    loftCenterLineOrRails = loftInput.centerLineOrRails

    #add all sections from collection
    for section in sections:
        loftSectionsObj.add(section)
    
    #add rails
    if rails != None:
        for rail in rails:    
            loftCenterLineOrRails.addRail(rail)
        
    #set solid, create loft
    loftInput.isSolid = solid
    loft = loftFeats.add(loftInput)

    return loft


#create offset face
def create_offset_face(rootComp, faces, distance):
    """Creates offset faces, offset by distance.

    Parameters
    ----------
    rootComp : adsk.fusion.Component 
        root component of active design

    faces : adsk.fusion.BRepFaces
        faces that will be offset

    distance : float
        distance by which to offset

    Returns
    -------
    offset : 
    """

    #create value input
    input_distance = adsk.core.ValueInput.createByReal(distance)
        
    #get offset features object, create object collection
    offFeats = rootComp.features.offsetFeatures
    entities = adsk.core.ObjectCollection.create()
    
    #add all faces to collection
    for face in faces:
        entities.add(face)
    
    #create offset
    offInput = offFeats.createInput(entities, input_distance, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    offset = offFeats.add(offInput)

    return offset

#create offset spline
def create_offset_chord(sketch, spline, offset):
    """Create offset spline.

    Parameters
    ----------
    sketch : adsk.fusion.sketch
        sketche for spline to be offset

    spline : 
        spline to be offset

    offset : float
        distance for offset

    Returns
    -------
    offset_spline : 
        offset spline
    """

    #create object collection to offset
    splines = adsk.core.ObjectCollection.create()
    splines.add(spline)

    #create direction point for offset
    dir_point = adsk.core.Point3D.create(-1, 0, 0)

    #create offset
    offset_splines = sketch.offset(splines, dir_point, offset)    

    #return spline and profile from object collection
    offset_spline = offset_splines.item(0)
    offset_profile = sketch.profiles.item(sketch.profiles.count - 1)

    return offset_spline, offset_profile


#create spars
def create_spars(sketch, angle, distance):
    """Create quadratic grid of spars, reinforcing wing.

    Parameters
    ----------
    sketch : adsk.fusion.Sketch
        sketch to draw grid onto

    angle : float
        angle relative to root chord

    distance : float
        distance between spars in quadratic grid

    Returns
    -------
    grid : 
        object collection of spar lines
    """

    #create object collections for points and lines
    start_points = adsk.core.ObjectCollection.create()
    end_points = adsk.core.ObjectCollection.create()
    grid = adsk.core.ObjectCollection.create()

    #lines from front to back
    for x in range(-2*ROOT_CHORD_LENGTH, 2*ROOT_CHORD_LENGTH, distance):
        start_point = adsk.core.Point3D.create(x, 0, 0)
        end_point = adsk.core.Point3D.create(x + (SPAN/2)*math.tan(angle), -SPAN/2, 0)
        grid.add(draw_line(sketch, start_point, end_point))

    #lines from back to front
    for x in range(-2*ROOT_CHORD_LENGTH, 2*ROOT_CHORD_LENGTH, distance):
        start_point = adsk.core.Point3D.create(x, -SPAN/2, 0)
        end_point = adsk.core.Point3D.create(x + (SPAN/2)*math.tan(angle), 0, 0)
        grid.add(draw_line(sketch, start_point, end_point))

    #return collection of lines
    return grid



### main function

def run(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui  = app.userInterface
        design = app.activeProduct        
        
        ui.messageBox('Running Skywalker script...')
        
        #my own code
        #casting can be used to help intellisense...
        
        #read in profile text file
        profile = read_profile(PROFILE_PATH)
                    
        
        #get the root component of the active design
        rootComp = design.rootComponent

        #create basic sketch in xy plane (to avoid interference in xy root sketch)
        sketch_xy = rootComp.sketches.add(rootComp.xYConstructionPlane)
        sketch_xy.name = "Basic Utility Sketch"
        
        #create a new sketch on the xy plane, for root chord
        sketch_xy_root = rootComp.sketches.add(rootComp.xYConstructionPlane)
        sketch_xy_root.name = "Root Chord Sketch"
        
        #create sketch for tip chord
        planes = rootComp.constructionPlanes
        planeInput = planes.createInput()
        offsetValue = adsk.core.ValueInput.createByReal(SPAN/2)
        planeInput.setByOffset(rootComp.xYConstructionPlane, offsetValue)
        tip_plane = planes.add(planeInput)
        sketch_xy_tip = rootComp.sketches.add(tip_plane)
        sketch_xy_tip.name = "Tip Chord Sketch"
        
        #sketch root chord
        root_chord, root_profile = sketch_chord(sketch_xy_root, ROOT_CHORD_LENGTH, profile, 0, 0)

        
        #sketch tip chord
        tip_chord, tip_profile = sketch_chord(sketch_xy_tip, TIP_CHORD_LENGTH, profile, (SPAN/2)*math.tan(SWEEP), 0)


        #create loft, inner wing

        #rails for loft
        #create a new sketch on the xy plane, for rails chord
        sketch_xy_rails = rootComp.sketches.add(rootComp.xYConstructionPlane)
        sketch_xy_rails.name = "Rails Sketch"
        lead_point1 = root_chord.fitPoints.item(31).worldGeometry
        lead_point2 = tip_chord.fitPoints.item(31).worldGeometry
        leading_edge = draw_line(sketch_xy_rails, lead_point1, lead_point2)
       
        
        trail_point1 = root_chord.fitPoints.item(0).worldGeometry
        trail_point2 = tip_chord.fitPoints.item(0).worldGeometry
        trailing_edge = draw_line(sketch_xy_rails, trail_point1, trail_point2)
        

        #create inner wing loft
        ui.messageBox("Creating wing loft...")

        #collect sections and rails
        inner_wing_sections = adsk.core.ObjectCollection.create()
        inner_wing_sections.add(root_profile)
        inner_wing_sections.add(tip_profile)

        inner_wing_rails = adsk.core.ObjectCollection.create()
        inner_wing_rails.add(leading_edge)
        inner_wing_rails.add(trailing_edge)

        #solid loft for inner wing, later used to intersect with ribs
        inner_wing = create_loft(rootComp, inner_wing_sections, solid=True, rails=inner_wing_rails)
        inner_wing.name = "inner wing"



        
        #create outer wing, offset face
        #use offset profiles and rails for clean outerwing, better than offset surface

        #create offset chords
        off_root_chord, off_root_profile = create_offset_chord(sketch_xy_root, root_chord, OFFSET)
        off_tip_chord, off_tip_profile = create_offset_chord(sketch_xy_tip, tip_chord, OFFSET)

        #create offset edges
        leading_vector = adsk.core.Vector3D.create(-OFFSET,0,0)
        leading_transform = adsk.core.Matrix3D.create()
        leading_transform.translation = leading_vector 
        off_leading_edge = draw_line(sketch_xy_rails, lead_point1, lead_point2)
        offset_leading = adsk.core.ObjectCollection.create()
        offset_leading.add(off_leading_edge)
        sketch_xy_rails.move(offset_leading, leading_transform)

        trailing_vector = adsk.core.Vector3D.create(OFFSET,0,0)
        trailing_transform = adsk.core.Matrix3D.create()
        trailing_transform.translation = trailing_vector 
        off_trailing_edge = draw_line(sketch_xy_rails, trail_point1, trail_point2)
        offset_trailing = adsk.core.ObjectCollection.create()
        offset_trailing.add(off_trailing_edge)
        sketch_xy_rails.move(offset_trailing, trailing_transform)

        #collect sections and rails
        outer_wing_sections = adsk.core.ObjectCollection.create()
        outer_wing_sections.add(off_root_profile)
        outer_wing_sections.add(off_tip_profile)

        outer_wing_rails = adsk.core.ObjectCollection.create()
        outer_wing_rails.add(off_leading_edge)
        outer_wing_rails.add(off_trailing_edge)

        #loft outer wing
        outer_wing = create_loft(rootComp, outer_wing_sections, solid=False, rails=outer_wing_rails)
        outer_wing.name = "outer wing"
        



        ###################
        ### clean above ###
        ###################




        #create aileron
        ui.messageBox("Creating ailerons...")

        #aileron points from tip to root and front to back:
        # lead1  -   lead2  -   slit2
        #            slit1  -   slit3
        ail_point_lead1 = adsk.core.Point3D.create(TIP_CHORD_LENGTH + (SPAN/2)*math.tan(SWEEP) - AIL_LENGTH, -(SPAN/2), 0)
        ail_point_lead2 = adsk.core.Point3D.create((ROOT_CHORD_LENGTH - AIL_LENGTH) + (SPAN/2 - AIL_WIDTH)*math.tan(TRAIL_SWEEP), -(SPAN/2 - AIL_WIDTH), 0)
        ail_point_slit1 = adsk.core.Point3D.create((ROOT_CHORD_LENGTH - AIL_LENGTH) + (SPAN/2 - AIL_WIDTH)*math.tan(TRAIL_SWEEP) + AIL_LENGTH*2, -(SPAN/2 - AIL_WIDTH), 0)
        ail_point_slit2 = adsk.core.Point3D.create((ROOT_CHORD_LENGTH - AIL_LENGTH) + (SPAN/2 - AIL_WIDTH - AIL_SPACE)*math.tan(TRAIL_SWEEP), -(SPAN/2 - AIL_WIDTH - AIL_SPACE), 0)
        ail_point_slit3 = adsk.core.Point3D.create((ROOT_CHORD_LENGTH - AIL_LENGTH) + (SPAN/2 - AIL_WIDTH - AIL_SPACE)*math.tan(TRAIL_SWEEP) + AIL_LENGTH*2, -(SPAN/2 - AIL_WIDTH - AIL_SPACE), 0)

        sketch_xz_ail = rootComp.sketches.add(rootComp.xZConstructionPlane)
        sketch_xz_ail.name = "Aileron Edges Sketch"
        aileron_leading_edge = sketch_xz_ail.sketchCurves.sketchLines.addByTwoPoints(ail_point_lead1, ail_point_lead2)
        aileron_side = sketch_xz_ail.sketchCurves.sketchLines.addByTwoPoints(ail_point_lead2, ail_point_slit1)

        #small spacing between wing and aileron
        aileron_space_top = draw_line(sketch_xz_ail, ail_point_lead2, ail_point_slit2)
        aileron_space_right = draw_line(sketch_xz_ail,ail_point_slit2, ail_point_slit3)
        aileron_space_bottom = draw_line(sketch_xz_ail, ail_point_slit1, ail_point_slit3)
        ail_space = sketch_xz_ail.profiles.item(sketch_xz_ail.profiles.count - 1)


        #cut space from outer wing (later also spar grid)
        #left of aileron
        '''
        # Create the extrusion cut
        extrudes = rootComp.features.extrudeFeatures
        cut_ail_space_input = extrudes.createInput(ail_space, adsk.fusion.FeatureOperations.CutFeatureOperation)
        extent_distance = adsk.fusion.DistanceExtentDefinition.create(adsk.core.ValueInput.createByString("20cm"))
        cut_ail_space_input.setTwoSidesExtent(extent_distance, extent_distance)
        #cut only outer wing
        cut_ail_space_input.participantBodies = [outer_wing.bodies.item(0)]
        cut_ail_space = extrudes.add(cut_ail_space_input)
        '''

        #cut space from outer wing (later also spar grid)
        #leading edge of aileron
        sketch_xy_tip_ail = rootComp.sketches.add(tip_plane)
        sketch_xy_tip_ail.name = "Aileron Tip Sketch"
        planeInput = planes.createInput()
        offsetValue = adsk.core.ValueInput.createByReal(SPAN/2 - AIL_WIDTH)
        planeInput.setByOffset(rootComp.xYConstructionPlane, offsetValue)
        plane_root_ail = planes.add(planeInput)
        sketch_xy_root_ail = rootComp.sketches.add(plane_root_ail)
        sketch_xy_root_ail.name = "Aileron Root Sketch"

        #get intersection with outer wing into aileron sketches
        intersection = sketch_xy_root_ail.intersectWithSketchPlane([outer_wing.bodies.item(0)])
        intersection_curve = sketch_xy_root_ail.sketchCurves.item(0)

        #get intersection points in aileron root plane
        inter1 = adsk.core.Point3D.create((ROOT_CHORD_LENGTH - AIL_LENGTH) + (SPAN/2 - AIL_WIDTH)*math.tan(TRAIL_SWEEP), SPAN/4, 0)
        inter2 = adsk.core.Point3D.create((ROOT_CHORD_LENGTH - AIL_LENGTH) + (SPAN/2 - AIL_WIDTH)*math.tan(TRAIL_SWEEP), -SPAN/4, 0)
        line = sketch_xy_root_ail.sketchCurves.sketchLines.addByTwoPoints(inter1, inter2)
        line.isConstruction = True


        #worldGeometry returns line3D object
        points = line.worldGeometry.intersectWithCurve(sketch_xy_root_ail.sketchCurves.item(0).worldGeometry)

        #line_ail = draw_line(sketch_xy, points.item(0), points.item(1))
        pointA = points.item(0)
        pointB = points.item(1)

        if points.item(0).y < points.item(1).y:
            pointA = points.item(1)
            pointB = points.item(0)
        else:
            pointA = points.item(0)
            pointB = points.item(1)

        vec = adsk.core.Vector3D.create(0, AIL_SLIT/10, 0)
        vec2 = adsk.core.Vector3D.create(0, -AIL_SLIT/2, 0)
        pointA.translateBy(vec)
        pointB.translateBy(vec2)

        line_ail = draw_line(sketch_xy, pointA, pointB)

        pointC = adsk.core.Point3D.create(pointB.x + AIL_SLIT, pointB.y, pointB.z)
        line_ail2 = draw_line(sketch_xy, pointB, pointC)
        line_ail3 = draw_line(sketch_xy, pointC, pointA)



        #create spar grid
        #ui.messageBox("Creating spars...")
        #sketch_xz_grid = rootComp.sketches.add(rootComp.xZConstructionPlane)
        #sketch_xz_grid.name = "Spar Grid Sketch"
        #wing_grid = create_spars(sketch_xz_grid, SPAR_ANGLE, SPAR_DIST)        

        
        '''
        #create a new sketch on the xz plane
        sketch_xz = rootComp.sketches.add(rootComp.xZConstructionPlane)

        #create ailerons
        
        
        aileron_leading_edge = sketch_xz.sketchCurves.sketchLines.addByTwoPoints(adsk.core.Point3D.create((ROOT_CHORD_LENGTH - AIL_LENGTH) + (SPAN/2 - AIL_WIDTH)*math.tan(TRAIL_SWEEP), -(SPAN/2 - AIL_WIDTH), 0), adsk.core.Point3D.create(TIP_CHORD_LENGTH + (SPAN/2)*math.tan(SWEEP) - AIL_LENGTH, -(SPAN/2), 0))
        aileron_side = sketch_xz.sketchCurves.sketchLines.addByTwoPoints(adsk.core.Point3D.create((ROOT_CHORD_LENGTH - AIL_LENGTH) + (SPAN/2 - AIL_WIDTH)*math.tan(TRAIL_SWEEP), -(SPAN/2 - AIL_WIDTH), 0), adsk.core.Point3D.create((ROOT_CHORD_LENGTH - AIL_LENGTH) + (SPAN/2 - AIL_WIDTH)*math.tan(TRAIL_SWEEP) + AIL_LENGTH*2, -(SPAN/2 - AIL_WIDTH), 0))

        #small spacing between wing and aileron
        aileron_space_z = sketch_xz.sketchCurves.sketchLines.addByTwoPoints(adsk.core.Point3D.create((ROOT_CHORD_LENGTH - AIL_LENGTH) + (SPAN/2 - AIL_WIDTH)*math.tan(TRAIL_SWEEP), -(SPAN/2 - AIL_WIDTH), 0), adsk.core.Point3D.create((ROOT_CHORD_LENGTH - AIL_LENGTH) + (SPAN/2 - AIL_WIDTH - AIL_SPACE)*math.tan(TRAIL_SWEEP), -(SPAN/2 - AIL_WIDTH - AIL_SPACE), 0))
        aileron_space_x = sketch_xz.sketchCurves.sketchLines.addByTwoPoints(adsk.core.Point3D.create((ROOT_CHORD_LENGTH - AIL_LENGTH) + (SPAN/2 - AIL_WIDTH - AIL_SPACE)*math.tan(TRAIL_SWEEP), -(SPAN/2 - AIL_WIDTH - AIL_SPACE), 0), adsk.core.Point3D.create((ROOT_CHORD_LENGTH - AIL_LENGTH) + (SPAN/2 - AIL_WIDTH - AIL_SPACE)*math.tan(TRAIL_SWEEP) + AIL_LENGTH*2, -(SPAN/2 - AIL_WIDTH - AIL_SPACE), 0))


        #create infill
        ui.messageBox("Creating infill...")

        box_points = [adsk.core.Point3D.create(ROOT_CHORD_LENGTH, SPAN, 0), adsk.core.Point3D.create( - ROOT_CHORD_LENGTH, - SPAN/4, 0), adsk.core.Point3D.create(ROOT_CHORD_LENGTH, - SPAN*1.5, 0), adsk.core.Point3D.create(ROOT_CHORD_LENGTH*3, - SPAN/4, 0)]

        box_lines = [sketch_xz.sketchCurves.sketchLines.addByTwoPoints(box_points[i], box_points[(i+1)%len(box_points)]) for i,point in enumerate(box_points)]

        #line = sketch_xz.sketchCurves.sketchLines.addByTwoPoints(box_points[0], box_points[1])
        #box_1 = sketch_xz.sketchCurves.sketchLines.addByTwoPoints(adsk.core.Point3D.create(ROOT_CHORD_LENGTH, SPAN/2, 0), adsk.core.Point3D.create( - ROOT_CHORD_LENGTH, - SPAN/4, 0))
        #box_2 = sketch_xz.sketchCurves.sketchLines.addByTwoPoints(adsk.core.Point3D.create(ROOT_CHORD_LENGTH, SPAN/2, 0), adsk.core.Point3D.create(ROOT_CHORD_LENGTH*3, - SPAN/4, 0))
        #box_3 = sketch_xz.sketchCurves.sketchLines.addByTwoPoints(adsk.core.Point3D.create(ROOT_CHORD_LENGTH*3, - SPAN/4, 0), adsk.core.Point3D.create(ROOT_CHORD_LENGTH, - SPAN, 0))
        '''

    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))
