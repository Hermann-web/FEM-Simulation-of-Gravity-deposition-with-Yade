# gravity deposition in box, showing how to plot and save history of data,
# and how to control the simulation while it is running by calling
# python functions from within the simulation loop

# import yade modules that we will use below
from yade import pack, plot
import math 
import numpy as np


# create rectangular box from facets

#O.bodies.append(geom.facetBox((.5,.5,.5),(.5,.5,.5),wallMask=31))
#facetCylinder(center=(0.5,0.5,0.5), radius=0.5, height=1, orientation=Quaternion((1, 0, 0), 0), segmentsNumber=10, wallMask=6, angleRange=None, closeGap=False, radiusTopInner=-1, radiusBottomInner=-1, **kw

O.bodies.append(geom.facetCylinder(center=(0.5,0.5,0.5), radius=(0.5)*math.sqrt(2), height=2, orientation=Quaternion((1, 0, 0), 0), segmentsNumber=30, wallMask=6))
#on augmente le rayon pour ne pas que les spheres sortent
#on augmente le nb de segmenyts pour etre proche du cercle


O.materials.append(FrictMat(young=70e5,poisson=.4,
                            frictionAngle=.4,label="verre"))


# create empty sphere packing
# sphere packing is not equivalent to particles in simulation, it contains only the pure geometry
sp=pack.SpherePack()
# generate randomly spheres with uniform radius distribution
sp.makeCloud((0,0,0),(1,1,2),rMean=0.05,rRelFuzz=.5)
#sp.makeCloud((0,0,0),(1,1,2),rMean=0.05,rRelFuzz=.5,num=320)
#on change le max des z
# add the sphere pack to the simulation
sp.toSimulation()

#ajouter les murs
O.bodies.append(utils.wall((0,0,2.2),2))
piston = O.bodies[-1]
piston.state.vel = (0,0,-0.8)

#je change la loi de contact ; je passse a hertz donc je change la physique
O.engines=[
	ForceResetter(),
	InsertionSortCollider([Bo1_Sphere_Aabb(),Bo1_Facet_Aabb(),Bo1_Wall_Aabb()]),
	InteractionLoop(
		# handle sphere+sphere and facet+sphere collisions
		[Ig2_Sphere_Sphere_ScGeom(),Ig2_Facet_Sphere_ScGeom(),Ig2_Wall_Sphere_ScGeom()],
		[Ip2_FrictMat_FrictMat_MindlinPhys()],
		#found on https://yade-dem.org/doc/yade.wrapper.html
		[ Law2_ScGeom_MindlinPhys_Mindlin()] 
		#found on https://yade-dem.org/wiki/ConstitutiveLaws
	),
	NewtonIntegrator(gravity=(0,0,-9.81),damping=0.4),
	# call the checkUnbalanced function (defined below) every 2 seconds
	PyRunner(command='checkUnbalanced()',realPeriod=2),
	# call the addPlotData function every 200 steps
	PyRunner(command='addPlotData()',iterPeriod=500)
	#PyRunner(command='print_energy()',iterPeriod=100)
]




O.dt=.5*PWaveTimeStep()

# enable energy tracking; any simulation parts supporting it
# can create and update arbitrary energy types, which can be
# accessed as O.energy['energyName'] subsequently
O.trackEnergy=True

# if the unbalanced forces goes below .05, the packing
# is considered stabilized, therefore we stop collected
# data history and stop
def checkUnbalanced():
	if unbalancedForce()<.05:
		#print(unbalancedForce())
		#print(piston.state.pos[2])
		O.pause()
		plot.saveDataTxt('bbb.txt.bz2')
		# plot.saveGnuplot('bbb') is also possible

		#
		L1 = []
		L2 = []
		for i in O.interactions:
			if i.isReal:
				fn = i.phys.normalForce
				fs = i.phys.shearForce
			L1.append(fn)
			L2.append(fs)
		#print(L1)
		#piston = O.bodies[-1]
		#piston.state.pos[2] = 0.1


# collect history of data which will be plotted
def addPlotData1():
	# each item is given a names, by which it can be the unsed in plot.plots
	# the **O.energy converts dictionary-like O.energy to plot.addData arguments
	plot.addData(i=O.iter,unbalanced=unbalancedForce(),**O.energy)

# define how to plot data: 'i' (step number) on the x-axis, unbalanced force
# on the left y-axis, all energies on the right y-axis
# (O.energy.keys is function which will be called to get all defined energies)
# None separates left and right y-axis
#plot.plots={'i':('unbalanced',None,O.energy.keys)}

# show the plot on the screen, and update while the simulation runs
#plot.plot()

def addPlotData():
	if not isinstance(O.bodies[-1].shape,Wall):
		plot.addData(); return
	#Fz=O.forces.f(piston.id)[2]
	Fz=0
	Amp = 0
	fz=0
	rz=0
	V=0
	Sigma_z = 0
	#INTER = [i for i in O.interactions if (isinstance(i.id1.shape,Sphere) and isinstance(i.id2.shape,Sphere))]
	SPHERES = [b for b in O.bodies if isinstance(b.shape, Sphere)]
	for i in O.interactions:
		if i.isReal:
			#force normale
			fn = i.phys.normalForce
			#Amp_inter = math.sqrt(fn[0]**2 + fn[1]**2 + fn[2]**2)
			Amp_inter = fn.norm() 
			Amp+=Amp_inter
			#calcul de fz
			fs = i.phys.shearForce
			fz = fn[2] + fs[2]

			#calcul de rz
			#rz = i.id1.state.pos[2] - i.id2.state.pos[2]
			
			for sphr2 in SPHERES:
				for sphr1 in SPHERES:
					if i.id1 == sphr1.id and i.id2 == sphr2.id: rz = sphr1.state.pos[2] - sphr2.state.pos[2];break
				if rz!=0: break
			#L_rz = [[ sphr1.state.pos[2] - sphr2.state.pos[2] for sphr1 in SPHERES if i.id1 == sphr1.id and i.id2 == sphr2.id] for sphr2 in SPHERES]
			
			#on parcoure les spheres pour trouver #on parcoure les spheres une seconde fois #si l'interaction i est entre ces deux spheres, alors

			"""rz = np.array(L_rz).sum()
			print("----rz :",rz)
			try: 
				rz = rz[0]
			except:
				rz = 0"""
			#print("----rz :",rz)

			#print( "constraint = ",fz*rz)
			Sigma_z += fz*rz



	for sphr1 in SPHERES:
		#calcul du volume de la sphere
		r = sphr1.shape.radius
		V += (4/3)* math.pi * r**3
	#print("V=",V)
	Sigma_z = -Sigma_z/V

	#L1 = tuple(L1)
	#L1 = math.sqrt(L1[0]**2 + L1[1]**2 + L1[2]**2)
	energy = 0
	for p in SPHERES : energy += p.state.mass*p.state.vel.norm()**2
	energy = 0.5*energy
	#print(piston.state.pos[2])
	
	print("Sigma_z= ",Sigma_z)

	#calcul de sigma_zz
	#for i in O.interactions: print i.id1,i.id2;=O.forces.f(i.id1)[2]

	#plot.addData(Fz=Fz,unbalanced=unbalancedForce(),i=O.iter,j=O.iter,Amplitude_EffortsNormal=Amp,Ec=energy,**O.energy,F=fz)
	plot.addData(Fz_inter=fz,Fz=Fz,unbalanced=unbalancedForce(),i=O.iter,j=O.iter,k=O.iter,Amplitude_EffortsNormal=Amp,Ec=energy,Sigma_z=Sigma_z,positionPiston=piston.state.pos[2],**O.energy)




# besides unbalanced force evolution, also plot the displacement-force diagram
plot.plots={'i':('unbalanced','Amplitude_EffortsNormal','Fz_inter','Fz'),'j':('Ec',O.energy.keys),'k':('Sigma_z'),'positionPiston':('Sigma_z')}
#plot.plots={'i':('unbalanced'),'j':('Ec',O.energy.keys)}

plot.plot()

"""
		L1 = []
		L2 = []
		for i in O.interactions:
			if i.isReal:
				fn = i.phys.normalForce
				fs = i.phys.shearForce
			L1.append(fn)
			L2.append(fs)
		print(L1)"""


O.saveTmp()


#pour find la rigidite, on a besoin du rayon (unique i guess) et le module de young
