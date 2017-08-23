from abaqus import *
from abaqusConstants import *
backwardCompatibility.setValues(includeDeprecated=True, reportDeprecated=False)
import sketch
import __main__
from caeModules import *
import part
import sys
import time
import os
import string
from optparse import OptionParser

''' geometric parameters '''
airfoil		= 'SC1095.txt' 		        # SELIG, tracing arbitrary continuous loop
blade		= 'UH60bladeNOTWIST.txt' 	# x, y, z, chord (ft), twist (rad) at c/4
scriptDir	= 'C:/cygwin64/home/dipalm/abaquscodes/abq-accordion'
workDir		= 'D:/abq/accordion'
segLocs		= [.25, .6]		        # fractions of chord where D-spar and conformable sections end
rotRate		= 27				# radians per second
seedsize	= .1				# mesh global seed size

''' don't edit below this line '''
os.chdir(scriptDir)
sparLoc=segLocs[0]
print '\n'*50, 'Initialized.\n'

def sign(x): return 1 if x >= 0 else -1 # sign function

def GetBlockPosition(modelName, blockPrefix): # for editing attribute keywords
	if blockPrefix == '':
		return len(mdb.models[modelName].keywordBlock.sieBlocks)-1
	pos = 0
	for block in mdb.models[modelName].keywordBlock.sieBlocks:
		if string.lower(block[0:len(blockPrefix)])==string.lower(blockPrefix):
			return pos
		pos=pos+1
	return -1

def rotate(ffffX,ffffY,chord,twist): # rotate coordinates by twist
	ffX = []
	ffY = []
	for i in range(0,len(ffffX)):
		ffX.append(ffffX[i]*chord*cos(twist)-ffffY[i]*chord*sin(twist))
		ffY.append(ffffX[i]*chord*sin(twist)+ffffY[i]*chord*cos(twist))
	return ffX,ffY;

def airwire(ffX,ffY,xcoord,ycoord,zcoord,chord,twist,datumplane): # creates airfoil wireframe at Z-coord
	airPart1.DatumPlaneByPrincipalPlane(principalPlane=XYPLANE, offset=(-zcoord)) # LE section
	fX, fY = rotate(ffX,ffY,chord,-twist)
	t = airPart1.MakeSketchTransform(sketchPlane=airPart1.datums[datumplane], sketchUpEdge=uedgea, sketchPlaneSide=SIDE1, sketchOrientation=RIGHT, origin=(-xcoord, ycoord, -zcoord))
	s = myModel.ConstrainedSketch(name='__profile__',sheetSize=200.0, transform=t)
	b=()
	b+=((fX[0]*sign(xcoord),fY[0]*sign(xcoord)),)
	for i in range(0,(len(fX)-1)):
		if (ffX[i]<=segLocs[0] and ffY[i]>0):
			b+=((fX[i]*sign(xcoord),fY[i]*sign(xcoord)),)
			firsti1=i
	s.Spline(points=b)
	b=()
	iff=1
	for i in range(0,(len(fX))):
		if (ffX[i]<=segLocs[0] and ffY[i]<0):
			b+=((fX[i]*sign(xcoord),fY[i]*sign(xcoord)),)
			if iff:
				firsti2=i
				iff=0
	b+=((fX[0]*sign(xcoord),fY[0]*sign(xcoord)),)
	s.Spline(points=b)
	s.Line(point1=(fX[firsti1]*sign(xcoord),fY[firsti1]*sign(xcoord)), point2=(fX[firsti2]*sign(xcoord),fY[firsti2]*sign(xcoord)))
	airPart1.Wire(sketchPlane=airPart1.datums[datumplane], sketchUpEdge=uedgea, sketchPlaneSide=SIDE1, sketchOrientation=RIGHT, sketch=s)
	del myModel.sketches['__profile__']
				
	airPart3.DatumPlaneByPrincipalPlane(principalPlane=XYPLANE, offset=(-zcoord)) # TE section
	t = airPart3.MakeSketchTransform(sketchPlane=airPart3.datums[datumplane], sketchUpEdge=uedgec, sketchPlaneSide=SIDE1, sketchOrientation=RIGHT, origin=(-xcoord, ycoord, -zcoord))
	s = myModel.ConstrainedSketch(name='__profile__',sheetSize=200.0, transform=t)
	b=()
	iff=1
	for i in range(0,(len(fX))):
		if (ffX[i]>=segLocs[1]):
			b+=((fX[i]*sign(xcoord),fY[i]*sign(xcoord)),)
			if iff:
				lasti1=i
				iff=0
			lasti2=i
	s.Spline(points=b)
	s.Line(point1=(fX[lasti1]*sign(xcoord),fY[lasti1]*sign(xcoord)), point2=(fX[lasti2]*sign(xcoord),fY[lasti2]*sign(xcoord)))
	airPart3.Wire(sketchPlane=airPart3.datums[datumplane], sketchUpEdge=uedgec, sketchPlaneSide=SIDE1, sketchOrientation=RIGHT, sketch=s)
	del myModel.sketches['__profile__']		
		
	airPart2.DatumPlaneByPrincipalPlane(principalPlane=XYPLANE, offset=(-zcoord)) # conformable section
	t = airPart2.MakeSketchTransform(sketchPlane=airPart2.datums[datumplane], sketchUpEdge=uedgeb, sketchPlaneSide=SIDE1, sketchOrientation=RIGHT, origin=(-xcoord, ycoord, -zcoord))
	s = myModel.ConstrainedSketch(name='__profile__',sheetSize=200.0, transform=t)
	s.Line(point1=(fX[firsti1]*sign(xcoord),fY[firsti1]*sign(xcoord)), point2=(fX[firsti2]*sign(xcoord),fY[firsti2]*sign(xcoord)))
        for i in range(0,lasti1-firsti1):
            if not i%2:
                s.Line(point1=(fX[firsti1+i]*sign(xcoord),fY[firsti1+i]*sign(xcoord)), point2=(fX[firsti1+i+1]*sign(xcoord),fY[firsti1+i+1]*sign(xcoord)))
                s.Line(point1=(fX[firsti1+i+1]*sign(xcoord),fY[firsti1+i+1]*sign(xcoord)), point2=(fX[firsti2-i-1]*sign(xcoord),fY[firsti2-i-1]*sign(xcoord)))
            else:
                s.Line(point1=(fX[firsti2-i]*sign(xcoord),fY[firsti2-i]*sign(xcoord)), point2=(fX[firsti2-i-1]*sign(xcoord),fY[firsti2-i-1]*sign(xcoord)))
                s.Line(point1=(fX[firsti1+i+1]*sign(xcoord),fY[firsti1+i+1]*sign(xcoord)), point2=(fX[firsti2-i-1]*sign(xcoord),fY[firsti2-i-1]*sign(xcoord)))
	airPart2.Wire(sketchPlane=airPart2.datums[datumplane], sketchUpEdge=uedgeb, sketchPlaneSide=SIDE1, sketchOrientation=RIGHT, sketch=s)
	del myModel.sketches['__profile__']
	return;
	
''' import airfoil coordinates '''
foilX = []
foilY = []
f = open(airfoil,'r')
for line in f.readlines():
	l = line.strip().split()
	foilX.append(float(l[0]))
	foilY.append(float(l[1]))
f.close()
bladeX = []
bladeY = []
bladeZ = []
chord = []
twist = []
f = open(blade,'r')
for line in f.readlines():
	l = line.strip().split()
	bladeX.append(float(l[0])*.3048)
	bladeY.append(float(l[1])*.3048)
	bladeZ.append(float(l[2])*.3048)
	chord.append(float(l[3])*.3048)
	twist.append(float(l[4]))
f.close()
os.chdir(workDir)
print 'Airfoil coordinates and blade properties have been parsed.\n'

''' create shell '''
start = time.time()
name='AIRSHELL' # can parametrize this name with independent variables
myModel = mdb.Model(name=name)
airPart1 = myModel.Part(name='airshell1', dimensionality=THREE_D, type=DEFORMABLE_BODY)
airPart1.DatumAxisByPrincipalAxis(principalAxis=YAXIS)
uedgea = airPart1.datums[1];
airPart2 = myModel.Part(name='airshell2', dimensionality=THREE_D, type=DEFORMABLE_BODY)
airPart2.DatumAxisByPrincipalAxis(principalAxis=YAXIS)
uedgeb = airPart2.datums[1];
airPart3 = myModel.Part(name='airshell3', dimensionality=THREE_D, type=DEFORMABLE_BODY)
airPart3.DatumAxisByPrincipalAxis(principalAxis=YAXIS)
uedgec = airPart3.datums[1];
dplane = 2
for j in range(0, len(bladeX)):
	airwire(foilX,foilY,bladeX[j],bladeY[j],bladeZ[j],chord[j],twist[j],dplane)
	dplane = dplane + 2
for j in range(1, len(bladeX)):
	oldWire = airPart1.edges.getByBoundingBox(-1e6,-1e6,-bladeZ[j-1]-.00001,1e6,1e6,-bladeZ[j-1]+.00001)
	newWire = airPart1.edges.getByBoundingBox(-1e6,-1e6,-bladeZ[j]-.00001,1e6,1e6,-bladeZ[j]+.00001)
	airPart1.ShellLoft(loftsections=(oldWire, newWire), startCondition=NONE, endCondition=NONE)
for j in range(1, len(bladeX)):
	oldWire = airPart2.edges.getByBoundingBox(-1e6,-1e6,-bladeZ[j-1]-.00001,1e6,1e6,-bladeZ[j-1]+.00001)
	newWire = airPart2.edges.getByBoundingBox(-1e6,-1e6,-bladeZ[j]-.00001,1e6,1e6,-bladeZ[j]+.00001)
	airPart2.ShellLoft(loftsections=(oldWire, newWire), startCondition=NONE, endCondition=NONE)
for j in range(1, len(bladeX)):
	oldWire = airPart3.edges.getByBoundingBox(-1e6,-1e6,-bladeZ[j-1]-.00001,1e6,1e6,-bladeZ[j-1]+.00001)
	newWire = airPart3.edges.getByBoundingBox(-1e6,-1e6,-bladeZ[j]-.00001,1e6,1e6,-bladeZ[j]+.00001)
	airPart3.ShellLoft(loftsections=(oldWire, newWire), startCondition=NONE, endCondition=NONE)
end=time.time()
print 'Blade shell has been lofted in ', (end - start), ' seconds.\n'

''' assign material properties '''
# start = time.time()
# ins = open("D:/tw/Optim/Analysis/abqprop.txt")
# properties=[line.split() for line in ins]
# myModel.Material(name='Fiber')
# myModel.materials['Fiber'].Elastic(type=LAMINA, table=((83000000000.0, 8500000000.0, 0.35, 4200000000.0, 4200000000.0, 4200000000.0), ))
# myModel.materials['Fiber'].Density(table=((1452.0, ), ))
# nn=0
# nn=airprops(bladeZ,airPart,properties[0::4],nn,1)
# nn=airprops(bladeZ,airPart2,properties[1::4],nn,1)
# nn=airprops(bladeZ,airPart3,properties[1::4],nn,-1)
# airPart.MaterialOrientation(region=regionToolset.Region(faces=airPart.faces.getByBoundingBox(-1e6,-1e6,-1e6,1e6,1e6,1e6)), 
	# orientationType=GLOBAL, axis=AXIS_2, additionalRotationType=ROTATION_NONE, 
	# localCsys=None, fieldName='')
# airPart2.MaterialOrientation(region=regionToolset.Region(faces=airPart2.faces.getByBoundingBox(-1e6,-1e6,-1e6,1e6,1e6,1e6)), 
	# orientationType=GLOBAL, axis=AXIS_2, additionalRotationType=ROTATION_NONE, 
	# localCsys=None, fieldName='')	
# airPart3.MaterialOrientation(region=regionToolset.Region(faces=airPart3.faces.getByBoundingBox(-1e6,-1e6,-1e6,1e6,1e6,1e6)), 
	# orientationType=GLOBAL, axis=AXIS_2, additionalRotationType=ROTATION_NONE, 
	# localCsys=None, fieldName='')
# end=time.time()
# print 'Composite properties applied in ', (end - start), ' seconds.\n'

''' set up load case '''
start = time.time()
a1 = myModel.rootAssembly
a1.DatumCsysByDefault(CARTESIAN)
a1.Instance(name='airshell1-1', part=airPart, dependent=ON)
a1.Instance(name='airshell2-1', part=airPart2, dependent=ON)
a1.Instance(name='airshell3-1', part=airPart3, dependent=ON)
faces1=myModel.rootAssembly.instances['airshell1-1'].faces.getByBoundingBox(-1e6,-1e6,-1e6,1e6,1e6,1e6)
faces2=myModel.rootAssembly.instances['airshell2-1'].faces.getByBoundingBox(-1e6,-1e6,-1e6,1e6,1e6,1e6)
faces3=myModel.rootAssembly.instances['airshell3-1'].faces.getByBoundingBox(-1e6,-1e6,-1e6,1e6,1e6,1e6)
myModel.StaticStep(name='ACTUATION', previous='Initial')
myModel.rootAssembly.Surface(side2Faces= faces1, name='CP-airshell1-1')
region1=myModel.rootAssembly.surfaces['CP-airshell1-1']
myModel.rootAssembly.Surface(side2Faces= faces2, name='CP-airshell2-1')
region2=myModel.rootAssembly.surfaces['CP-airshell2-1']
myModel.rootAssembly.Surface(side2Faces= faces3, name='CP-airshell3-1')
region3=myModel.rootAssembly.surfaces['CP-airshell3-1']
myModel.Tie(name='CP-1', master=region1, slave=region2, positionToleranceMethod=SPECIFIED, positionTolerance=1e-06, adjust=ON, thickness=OFF, constraintEnforcement=SURFACE_TO_SURFACE)
myModel.Tie(name='CP-2', master=region2, slave=region3, positionToleranceMethod=SPECIFIED, positionTolerance=1e-06, adjust=ON, thickness=OFF, constraintEnforcement=SURFACE_TO_SURFACE)
e1 = myModel.rootAssembly.instances['airshell1-1'].vertices.getByBoundingBox(-1e6,-1e6,-bladeZ[0]-.00001,1e6,1e6,1e6)
regione1 = myModel.rootAssembly.Set(vertices=e1, name='fixed-set1')
# ########### can incorporate other parts here to fix more than the Dspar
myModel.DisplacementBC(name='ROOT1', createStepName='Initial', region=regione1, u1=SET, u2=SET, u3=SET, ur1=SET, ur2=SET, ur3=SET,  amplitude=UNSET, distributionType=UNIFORM, fieldName='', localCsys=None)
myModel.FieldOutputRequest(name='COORDS', createStepName='ACTUATION', variables=('COORD', ))

end=time.time()
print 'Load case established in ', (end - start), ' seconds.\n'

# ''' set up mesh '''
# start = time.time()
# elemType1 = mesh.ElemType(elemCode=S8R, elemLibrary=STANDARD)
# elemType2 = mesh.ElemType(elemCode=STRI65, elemLibrary=STANDARD)
# airPart.setElementType(regions=(airPart.faces.getByBoundingBox(-1e6,-1e6,-1e6,1e6,1e6,1e6), ), elemTypes=(elemType1, elemType2))
# airPart.seedPart(size=seedsize, deviationFactor=0.5, minSizeFactor=0.5)
# airPart.generateMesh()
# airPart2.setElementType(regions=(airPart2.faces.getByBoundingBox(-1e6,-1e6,-1e6,1e6,1e6,1e6), ), elemTypes=(elemType1, elemType2))
# airPart2.seedPart(size=seedsize, deviationFactor=0.5, minSizeFactor=0.5)
# airPart2.generateMesh()
# airPart3.setElementType(regions=(airPart3.faces.getByBoundingBox(-1e6,-1e6,-1e6,1e6,1e6,1e6), ), elemTypes=(elemType1, elemType2))
# airPart3.seedPart(size=seedsize, deviationFactor=0.5, minSizeFactor=0.5)
# airPart3.generateMesh()
# myModel.keywordBlock.synchVersions()
# #LE=myModel.rootAssembly.instances['airshell1-1'].nodes.getByBoundingBox(rotate([0],[0],chord[0],twist[0])[0][0]-bladeX[0]-1e-6,-rotate([0],[0],chord[0],twist[0])[1][0]-1e-6,-bladeZ[0]-1e-6,rotate([0],[0],chord[0],twist[0])[0][0]-bladeX[0]+1e-6,-rotate([0],[0],chord[0],twist[0])[1][0]+1e-6,-bladeZ[0]+1e-6)
# #TE=myModel.rootAssembly.instances['airshell4-1'].nodes.getByBoundingBox(rotate([1.000000],[-0.001729],chord[0],twist[0])[0][0]-bladeX[0]-1e-6,-rotate([1.000000],[-0.001729],chord[0],twist[0])[1][0]-1e-6,-bladeZ[0]-1e-6,rotate([1.000000],[-0.001729],chord[0],twist[0])[0][0]-bladeX[0]+1e-6,-rotate([1.000000],[-0.001729],chord[0],twist[0])[1][0]+1e-6,-bladeZ[0]+1e-6)
# #for j in range(1, len(bladeZ))
# #	LE+=myModel.rootAssembly.instances['airshell1-1'].nodes.getByBoundingBox(rotate([0],[0],chord[j],twist[j])[0][0]-bladeX[j]-1e-6,-rotate([0],[0],chord[j],twist[j])[1][0]-1e-6,-bladeZ[j]-1e-6,rotate([0],[0],chord[j],twist[j])[0][0]-bladeX[j]+1e-6,-rotate([0],[0],chord[j],twist[j])[1][0]+1e-6,-bladeZ[j]+1e-6)
# #	TE+=myModel.rootAssembly.instances['airshell4-1'].nodes.getByBoundingBox(rotate([1.000000],[-0.001729],chord[j],twist[j])[0][0]-bladeX[j]-1e-6,-rotate([1.000000],[-0.001729],chord[j],twist[j])[1][0]-1e-6,-bladeZ[j]-1e-6,rotate([1.000000],[-0.001729],chord[j],twist[j])[0][0]-bladeX[j]+1e-6,-rotate([1.000000],[-0.001729],chord[j],twist[j])[1][0]+1e-6,-bladeZ[j]+1e-6)
# LE=myModel.rootAssembly.instances['airshell1-1'].nodes.getByBoundingBox(rotate([0],[0],chord[-1],twist[-1])[0][0]-bladeX[-1]-1e-6,-rotate([0],[0],chord[-1],twist[-1])[1][0]-1e-6,-bladeZ[-1]-1e-6,rotate([0],[0],chord[-1],twist[-1])[0][0]-bladeX[-1]+1e-6,-rotate([0],[0],chord[-1],twist[-1])[1][0]+1e-6,-bladeZ[-1]+1e-6)
# TE=myModel.rootAssembly.instances['airshell4-1'].nodes.getByBoundingBox(rotate([1.000000],[-0.001729],chord[-1],twist[-1])[0][0]-bladeX[-1]-1e-6,-rotate([1.000000],[-0.001729],chord[-1],twist[-1])[1][0]-1e-6,-bladeZ[-1]-1e-6,rotate([1.000000],[-0.001729],chord[-1],twist[-1])[0][0]-bladeX[-1]+1e-6,-rotate([1.000000],[-0.001729],chord[-1],twist[-1])[1][0]+1e-6,-bladeZ[-1]+1e-6)
# myModel.rootAssembly.Set(name='TE', nodes=TE)
# myModel.rootAssembly.Set(name='LE', nodes=LE)

# end=time.time()
# print 'Structure meshed in ', (end - start), ' seconds.\n'
# start = time.time()
# mdb.Job(name='Job-1', model='AIRSHELL', description='', type=ANALYSIS, 
    # atTime=None, waitMinutes=0, waitHours=0, queue=None, memory=90, 
    # memoryUnits=PERCENTAGE, getMemoryFromAnalysis=True, 
    # explicitPrecision=SINGLE, nodalOutputPrecision=SINGLE, echoPrint=OFF, 
    # modelPrint=OFF, contactPrint=OFF, historyPrint=OFF, userSubroutine='', 
    # scratch='', multiprocessingMode=DEFAULT, numCpus=1, numGPUs=0)

# # mdb.jobs['Job-1'].submit(consistencyChecking=OFF)
# # mdb.jobs['Job-1'].waitForCompletion()
# # w=open('Job-1'+'.log', 'rb')
# # lines=w.readlines()
# # if (lines[-1]=='Abaqus JOB '+'Job-1'+' COMPLETED\r\n'):	
	# # from odbAccess import *
	# # odb = openOdb ('Job-1'+'.odb')
	# # step1 = odb.steps['SPIN']
	# # step1frame0 = step1.frames[0]
	# # step1frame1 = step1.frames[-1]
	# # disp1 = step1frame0.fieldOutputs['COORD']
	# # disp2 = step1frame1.fieldOutputs['U']
	# # disp3 = step1frame1.fieldOutputs['E']

	# # LEx1=disp1.getSubset(region=odb.rootAssembly.nodeSets["LE"]).values[0].data[0]
	# # TEx1=disp1.getSubset(region=odb.rootAssembly.nodeSets["TE"]).values[0].data[0]
	# # LEx2=disp1.getSubset(region=odb.rootAssembly.nodeSets["LE"]).values[0].data[1]
	# # TEx2=disp1.getSubset(region=odb.rootAssembly.nodeSets["TE"]).values[0].data[1]
	# # LEu1=disp2.getSubset(region=odb.rootAssembly.nodeSets["LE"]).values[0].data[0]
	# # TEu1=disp2.getSubset(region=odb.rootAssembly.nodeSets["TE"]).values[0].data[0]
	# # LEu2=disp2.getSubset(region=odb.rootAssembly.nodeSets["LE"]).values[0].data[1]
	# # TEu2=disp2.getSubset(region=odb.rootAssembly.nodeSets["TE"]).values[0].data[1]
	
	# # inittwist=degrees(atan((LEx2-TEx2)/(LEx1-TEx1)))
	# # finaltwist=degrees(atan((LEx2+LEu2-TEx2-TEu2)/(LEx1+LEu1-TEx1-TEu1)))
		
	# # session.viewports['Viewport: 1'].setValues(displayedObject=odb)
	# # session.viewports['Viewport: 1'].odbDisplay.setFrame(step=0, frame=1)
	
	# # session.writeFieldReport(fileName='abaqus.rpt', append=OFF, 
		# # sortItem='Element Label', odb=odb, step=0, frame=1, 
		# # outputPosition=ELEMENT_CENTROID, variable=(('E', INTEGRATION_POINT, ((
		# # COMPONENT, 'E11'), (COMPONENT, 'E22'), (COMPONENT, 'E12'), )), ))
	
	# # myoutfile = open('TIP-TWIST.txt','w')
	# # print "hi ",inittwist," ",finaltwist
	# # myoutfile.write(str(-inittwist) + '\t' + str(-finaltwist))
	# # myoutfile.close()
	# # odb.close()
	# # w.close()
# end=time.time()
# print "Analyzed and post-processed in ", (end - start), ' seconds.\n'
# mdb.saveAs(pathName=workDir+'/LOFTEDBODY')
