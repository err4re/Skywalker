#Author- Alexander Wagner
#Description- Script creates Skywalker-like flying wing

import adsk.core, adsk.fusion, adsk.cam, traceback
import math

#Design constants

#sweep in radians
SWEEP = (30/360)*2*math.pi 
#root chord length in cm
ROOT_CHORD_LENGTH = 20
#tip chord length in cm
TIP_CHORD_LENGTH = 14
#span length in cm
SPAN = 55

#make the slicing work...
#offset between outer perimeter and inner structures in cm
OFFSET = 0.08


#profile path
PROFILE_PATH = r"C:\Users\Alexander\Documents\Nurflügel\nur12d\Profile\MH45.txt"


def run(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui  = app.userInterface
        design = app.activeProduct        
        
        ui.messageBox('Running Skywalker script...')
        
        #my own code
        
        #read in profile text file
        f = open(PROFILE_PATH, "r")
        lines = f.readlines()
        x_profile = []
        y_profile = []
        
        for line in lines[1:]:
            
            x,y = line.split(" ")
            x_profile.append(float(x))
            y_profile.append(float(y))
            
        
        #get the rrot component of the active design
        rootComp = design.rootComponent
        
        #create a new sketch on the xy plane
        sketch_xy = rootComp.sketches.add(rootComp.xYConstructionPlane)
        
        planes = rootComp.constructionPlanes
        planeInput = planes.createInput()
        offsetValue = adsk.core.ValueInput.createByReal(SPAN/2)
        planeInput.setByOffset(rootComp.xYConstructionPlane, offsetValue)
        tip_plane = planes.add(planeInput)
        sketch_xy_tip = rootComp.sketches.add(tip_plane)
        
        #create object collection for the root chord
        root_chord_points = adsk.core.ObjectCollection.create()
        
        for x,y in zip(x_profile,y_profile):
            x_root = x*(ROOT_CHORD_LENGTH)
            y_root = y*(ROOT_CHORD_LENGTH)
            
            root_chord_points.add(adsk.core.Point3D.create(x_root, y_root, 0))
        
        # Create root chord
        root_chord = sketch_xy.sketchCurves.sketchFittedSplines.add(root_chord_points)
        
        #get profile
        root_profile = sketch_xy.profiles.item(0)
                    
        #create object collection for the tip chord
        tip_chord_points = adsk.core.ObjectCollection.create()
        
        for x,y in zip(x_profile, y_profile):
            x_tip = x*(TIP_CHORD_LENGTH) + (SPAN/2)*math.tan(SWEEP)
            y_tip = y*(TIP_CHORD_LENGTH)
            #z_tip = (SPAN/2)
            
            tip_chord_points.add(adsk.core.Point3D.create(x_tip, y_tip, 0))
            
        # Create tip chord
        tip_chord = sketch_xy_tip.sketchCurves.sketchFittedSplines.add(tip_chord_points)
        
        #get profile
        tip_profile = sketch_xy_tip.profiles.item(0)
        
        ui.messageBox("Drawing offset chords...")        
        
        #create offset splines for root and tip chord
        tip = adsk.core.ObjectCollection.create()
        tip.add(tip_chord)
        
        root = adsk.core.ObjectCollection.create()
        root.add(root_chord)
        
        dir_point_root = adsk.core.Point3D.create(-1, 0, 0)
        dir_point_tip = adsk.core.Point3D.create(-1, 0, 0)
        
        off_root_chord = sketch_xy.offset(root, dir_point_root, -OFFSET)
        off_root_profile = sketch_xy.profiles.item(1)        
        
        off_tip_chord = sketch_xy_tip.offset(tip, dir_point_tip, -OFFSET)
        off_tip_profile = sketch_xy_tip.profiles.item(1)
        
        ui.messageBox("Creating wing loft...")
        
        #create wing loft
        
        # Create loft feature input
        loftFeats = rootComp.features.loftFeatures
        loftInput = loftFeats.createInput(adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        loftSectionsObj = loftInput.loftSections
        
        loftSectionsObj.add(root_profile)
        loftSectionsObj.add(tip_profile)
        
        #add rails for outer loft
        
        #create a new sketch on the xy plane
        sketch_xz = rootComp.sketches.add(rootComp.xZConstructionPlane)
        leading_edge = sketch_xz.sketchCurves.sketchLines.addByTwoPoints(adsk.core.Point3D.create(0,0,0), adsk.core.Point3D.create((SPAN/2)*math.tan(SWEEP), -(SPAN/2), 0))
        trailing_edge = sketch_xz.sketchCurves.sketchLines.addByTwoPoints(adsk.core.Point3D.create(ROOT_CHORD_LENGTH, 0, 0), adsk.core.Point3D.create(TIP_CHORD_LENGTH + (SPAN/2)*math.tan(SWEEP), -(SPAN/2), 0))
        
        loftCenterLineOrRails = loftInput.centerLineOrRails
        loftCenterLineOrRails.addRail(leading_edge)
        loftCenterLineOrRails.addRail(trailing_edge)        
        
        loftInput.isSolid = False
        outer_wing = loftFeats.add(loftInput)
        
        #create offset wing loft
        
        #create feature input
        
        loftInput = loftFeats.createInput(adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        loftSectionsObj = loftInput.loftSections
        
        loftSectionsObj.add(off_root_profile)
        loftSectionsObj.add(off_tip_profile)
        
        #add rails for outer loft
        
        off_leading_edge = sketch_xz.sketchCurves.sketchLines.addByTwoPoints(adsk.core.Point3D.create(OFFSET,0,0), adsk.core.Point3D.create(OFFSET+(SPAN/2)*math.tan(SWEEP), -(SPAN/2), 0))
        
        loftCenterLineOrRails = loftInput.centerLineOrRails
        loftCenterLineOrRails.addRail(off_leading_edge)
        
        loftInput.isSolid = True
        off_outer_wing = loftFeats.add(loftInput)
        
        

    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))
