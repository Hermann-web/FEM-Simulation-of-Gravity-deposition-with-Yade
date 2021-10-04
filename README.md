# Thalus-Angle-Mesurement-by-Simulating-Gavity-deposition-with-Yade

# Introduction
Dans le cadre du TP sur les matériaux granulaires, nous sommes amenés à utiliser sur la distribution Ubuntu de linux, le logiciel Yade afin de réaliser la simulation des comportements des matériaux granulaires. L’intérêt de ce logiciel découle du fait qu’il contient des codes déjà optimisés qui permettant cette simulation. De plus, les programmes à écrire pour décrire les matériaux voulus se font avec le language Python, ce qui permet de faire la construction, le contrôle et le traitement des simulations.

# Etape1 : Installation
Nous installons le logiciel Yade à partie des dépots linux. 

# Etape2 : Documentation
Afin de prendre en main le logiciel Yade et effectuer les simulations, il est nécessaire de consulter la documentation accessible en ligne à l’adresse : yade-dem.org

# Etape3 : Ecriture du script de simulation
Le script suit le paragdime de la POO. Nous travailerons alors avec les grains, des objects POO ayant des attributs tels que la masse, la position, la vitesse ainsi que des méthodes tels que les calculs de contacts
- O.bodies : Un grain est un object (POO) (masse, position, vitesse ; calculs de contact,.)
- O.forces
- O.interactions : Il s’agit des lois d’interactions entre les differents corps (au moins deux bodies). Il sont de trois types : 
  - Géométrique : modes de calculs (par exemple, une comparaison des positions pour savoir si il y a contact)
  - Physique : Comment 
  - Comportement : Il s’agit des lois de comportements que régissent la dynamique des structures 
- O.engines : Il s’agit de machines, fonctions pour detecter les contacts, les interactions, utiliser des schémas intégrateurs,
 

# Etape4 : Description de simulation
Voici dans le cas général, l’ensemble des étapes à suivre pour effectuer une simulation.
- On définit la boite : ce sera soit parallélogramme ouvert ou soit un cylindre, quitte à s’assurer que les grains ne soient pas répartis à l’extérieur de ce dernier 
- On crée les grains : ce seront des sphères. Pour que les sphères tombent dans la boite cylindrique, on s’assure que la base soit inscrite dans un cercle de diamètre
- On ajoute les spheres au O.bodies
- On definit la géométrie, interactions, loi de comportement à utiliser

# Etape5 : Simulation
### Remplissage d’un cylindre par des particules soumises à leur poids
<pre>
#création du conclour cylindrique
  #on augmente le rayon pour ne pas que les spheres sortent #on augmente le nb de segments pour etre proche du cercle
O.bodies.append(geom.facetCylinder(center=(0.5,0.5,0.5), radius=(0.5)*math.sqrt(2), height=2, orientation=Quaternion((1, 0, 0), 0), segmentsNumber=30, wallMask=6))


# create empty sphere packing # sphere packing is not equivalent to particles in simulation, it contains only the pure geometry
sp=pack.SpherePack()

# generate randomly spheres with uniform radius distribution
sp.makeCloud((0,0,0),(1,1,2),rMean=0.05,rRelFuzz=.5)

# add the sphere pack to the simulation
sp.toSimulation()
</pre>
![image](https://user-images.githubusercontent.com/69398651/135937871-2292c59c-51eb-4012-afaf-bbc6115b4578.png)

### Changer le type de contact
On change la loi du contact pour passer à la contrainte calculée  
<pre>
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
</pre>

### Changer le type de matériau des grains
En effet, si on ne change pas le type de matériaux, c’est le matériaux par défaut qui est considéré
<pre>O.materials.append(FrictMat(young=70e5,poisson=.4,frictionAngle=.4,label="verre"))</pre>


A partir de la simulation 1, comprimer ces particules en utilisant un bouchon qui se déplace verticalement.
Pour réaliser celà, on ajoute un mur auquel on assigne une vitesse. 
- Position du mur : Le mur doit être au dessus des grains
- Vitesse du mur : on lui assigne unr vitesse à la fois
  - Assez Grande : car si c’est trop faible, les grains sont à l’équilibre avant que le piston ne descende, par conséquent, il faut relancer plusieurs fois la simulation 
  - Assez Faible : car sinon, la contrainte imposée aux grains est si forte que les grains sont éclaboussés vers l’extérieur de la boite

## ajouter les murs
<pre>
O.bodies.append(utils.wall((0,0,2.2),2))
piston = O.bodies[-1]
piston.state.vel = (0,0,-0.8)
</pre>




### Mouvements des grains
![image](https://user-images.githubusercontent.com/69398651/135937776-5a01d281-f071-43ba-a60c-52ab0848b97e.png)![image](https://user-images.githubusercontent.com/69398651/135937801-d5d7594c-8401-4ae3-a2ae-952647816cf3.png)![image](https://user-images.githubusercontent.com/69398651/135937989-651990c2-091a-4747-bae3-2ab19495d151.png)![image](https://user-images.githubusercontent.com/69398651/135937998-4b534469-3c21-4fa6-8a7a-f7bed0ad48be.png)

![image](https://user-images.githubusercontent.com/69398651/135937829-934b7344-5f18-4bd2-83fc-e7b970468f60.png)



  
### Evolution du réseau de force
Le réseau de force est ajouté à la figure. L’affichage de cette option est disponible sur le simulateur

### Evolution de distribution d’amplitude des forces normales
On remarque que les forces normales 
	Evolution de la contrainte globale σ_zz
	Evolution de la contrainte σ_zz (z)

### Interprétation des résultats
Energie Cinétique
- Pendant la descente sous le poids : l’energie cinétique croit puis décroit à l’équilibre pendant la descente du piston. Ce qui est normal puisqu’au départ, on laisse les grains tomber sous leur poids, donc leur vitesse augmente due à l’accélération constante. Ensuite, les boules tombent dans le fond de la boite et leur vitesse moyenne sera alors très faible
- Pendant l’appui : l’énergie cinétique augmente puis décroit. Ceci s’explique aussi par le fait que : Au début de la compression, les grains se mettent en mouvement puis ils sont totalement comprimé
- Remarque supplémentaire : Quand on continue la simulation à l’aide du bouton « play » de l’interface, les boulent bondissent, sortent de la boite puis retombent

Evolution de la contrainte globale sigma_zz
- Pendant la descente sous le poids : La contrainte est constamment nulle puis prend un pic avant de varier en suivant des formes triangulaires (min=-1200, max=1300 sur un exemple)
- Pendant l’appui : La contrainte (en valeur absolue elle est négative pour une compression) monte rapidement vers un nouveau maximum (6,48 e4)

# Conclusion
La prédiction des comportements des matériaux granulaires est faisable à l’aide de Yade disponible sur Linux. Si on veut de meilleures visualisations, on peut utiliser paraview. Cela n’a pas été utile dans notre TP.


