# gravity deposition in box, showing how to plot and save history of data,
# and how to control the simulation while it is running by calling
# python functions from within the simulation loop

# import yade modules that we will use below
from yade import pack, plot
import numpy as np
import math

# create rectangular box from facets

O.bodies.append(geom.facetBunker(center=(0,0,1.5), dBunker=1+2*math.sqrt(2), dOutput=1, hBunker=3, hOutput=0.5, hPipe=1.5))

O.bodies.append(geom.facetCylinder(center=(0,0,0), radius=20, height=0.1, orientation=Quaternion((1, 0, 0), 0), segmentsNumber=30, wallMask=6))

O.materials.append(FrictMat(young=70e5,poisson=.4,frictionAngle=.4,label="verre"))

# create empty sphere packing
# sphere packing is not equivalent to particles in simulation, it contains only the pure geometry
sp=pack.SpherePack()
# generate randomly spheres with uniform radius distribution
sp.makeCloud((-1,-1,5-1),(1,1,8-1),rMean=.05,rRelFuzz=0)
# add the sphere pack to the simulation
sp.toSimulation()


O.engines=[
	ForceResetter(),
	InsertionSortCollider([Bo1_Sphere_Aabb(),Bo1_Facet_Aabb(), 
                        Bo1_Wall_Aabb()]),
	InteractionLoop(
		# handle sphere+sphere and facet+sphere collisions
		[Ig2_Sphere_Sphere_ScGeom(),Ig2_Facet_Sphere_ScGeom(),
         Ig2_Wall_Sphere_ScGeom()],
		[Ip2_FrictMat_FrictMat_MindlinPhys(), ],
		[Law2_ScGeom_MindlinPhys_Mindlin()]
	),
	NewtonIntegrator(gravity=(0,0,-9.81),damping=0.4),
	# call the checkUnbalanced function (defined below) every 2 seconds
	PyRunner(command='checkUnbalanced()',realPeriod=2),
	# call the addPlotData function every 200 steps
	PyRunner(command='addPlotData()',iterPeriod=100)
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
		O.pause()
		#plot.saveDataTxt('bbb.txt.bz2')
		# plot.saveGnuplot('bbb') is also possible

# collect history of data which will be plotted
def addPlotData():
	# each item is given a names, by which it can be the unsed in plot.plots
	#the **O.energy converts dictionary-like O.energy to plot.addData arguments	
	
	#calcul du rayon
	
	l  = [ b for b in O.bodies if (isinstance(b.shape,Sphere))];print("len(l) = ",len(l))
	liste_rayons = [ b.shape.radius for b in l ];print("len(liste_rayons) = ",len(liste_rayons))
	#rayon max
	r_max = max(liste_rayons)
	alpha=0
	try:
		liste_sphere_sol = [ b for b in l if b.state.pos[2]<=r_max];print("len(liste_sphere_sol) = ",len(liste_sphere_sol))


		liste_rayons_detallemnt = [ math.sqrt( b.state.pos[0]**2 + b.state.pos[1]**2) for b in liste_sphere_sol]
		liste_rayons_detallemnt.sort()
		nb = len(liste_rayons_detallemnt)*0.9;print("nb = ",nb)
		rayon_d_etallement = liste_rayons_detallemnt[int(nb)];print("int(nb) = ",int(nb))
		
		print("rayon_d_etallement = ",rayon_d_etallement)
		
		#calcul de la hauteur
		hauteur = max([1+b.state.pos[2] for b in l])
		#angle de talus
		alpha = (180/math.pi)*math.atan(hauteur/rayon_d_etallement)
		print("alpha =  ",alpha)
	except: 
		pass
		print("down here")
	plot.addData(i=O.iter,j=O.iter,unbalanced=unbalancedForce(),alpha=alpha,**O.energy)
    


# define how to plot data: 'i' (step number) on the x-axis, unbalanced force
# on the left y-axis, all energies on the right y-axis
# (O.energy.keys is function which will be called to get all defined energies)
# None separates left and right y-axis
plot.plots={'i':('unbalanced',None,O.energy.keys),'j':('alpha')}

# show the plot on the screen, and update while the simulation runs
plot.plot()

O.saveTmp()


print(alpha)
