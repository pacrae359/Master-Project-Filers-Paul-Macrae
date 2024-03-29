from __future__ import division
import time # for timing
from math import sqrt, tan, sin, cos, pi, ceil, floor, acos, atan, asin, degrees, radians, log, atan2, acos, asin
from random import *
from numpy import *
from pymclevel import alphaMaterials, MCSchematic, MCLevel, BoundingBox
from mcplatform import *

import utilityFunctions as utilityFunctions

listOfHouses = []
listOfWorkplaces = []
endFound = False
tooHigh = False
paths = []
budget = 0
widthG = 0
depthG = 0
bottomWidth = 0
bottomDepth = 0

class house:
	def __init__(self, x, y, z, number, workplaceNumber):
		self.x = x
		self.y = y
		self.z = z
		self.number = number
		self.workplaceNumber = workplaceNumber

# Class storing some properties of a workplace in the game
class workplace:
	def __init__(self, x, y, z, number):
		self.x = x
		self.y = y
		self.z = z
		self.number = number

# Class storing some properties of a tile in the game
class tile:
	def __init__(self, x, y, z, material):
		self.x = x
		self.y = y
		self.z = z
		self.material = material

#class for storing node information
class node:
	def __init__(self, x, y, z, fvalue, gcost, parent):
		self.x = x
		self.y = y
		self.z = z
		self.fvalue = fvalue
		self.gcost = gcost
		self.parent = parent

#class for storing details of each path and their destination.
class pathcosthouse:
	def __init__(self, path, cost, house):
		self.path = path
		self.cost = cost
		self.house = house

class gennode:
	def __init__(self, x, y, z, material):
		self.x = x
		self.y = y
		self.z = z
		self.material = material

class individual:
	def __init__(self,nodelist,fitness):
		self.nodelist = nodelist
		self.fitness = fitness

def perform(level, box, options):
	global paths
	print("----------------------------------------------------------------------------")
	tileMap = getTileMap(level, box, 255, 0)
	findWorkplaces(level, box)
	findHouses(level, box)
	fixHouses()
	GeneticAlgorithm(level,box,tileMap)

def GeneticAlgorithm(level,box,tileMap):
	global paths
	population = []
	population = generateInitialPop(population,level,box,tileMap)
	#place the blocks from each individual, then evaluate using A* for a fitness value.
	for individual in population:
		seed(time.clock())
		totalpathcost = 0
		changeBlocks(individual,level,box,tileMap)
		for house in listOfHouses:
			Astar(level, box, house, tileMap)
		for pathcosts in paths:
			totalpathcost = totalpathcost + pathcosts.cost
		totalpathcost = totalpathcost / len(paths)
		individual.fitness = totalpathcost
		resetBlocks(individual,level,box,tileMap)
		print("Individual Checked")
	notDone = True
	counta = 1
	#How will notDone be defined here, maybe when the average fitness value has barely changed over the last two generations?
	#Idk figure this one out last I guess or you'll just piss about not doing anything :P
	#while notDone:
	for x in range(0,1):
		counta = counta + 1
		print('iteration number: ',counta)
		notDone = False
		population = generateNewPop(population,level,box,tileMap)
		for individual in population:
			seed(time.clock())
			totalpathcost = 0
			changeBlocks(individual,level,box,tileMap)
			for house in listOfHouses:
				Astar(level, box, house, tileMap)
			for pathcosts in paths:
				totalpathcost = totalpathcost + pathcosts.cost
			totalpathcost = totalpathcost / len(paths)
			individual.fitness = totalpathcost
			resetBlocks(individual,level,box,tileMap)

def mutation(nodeList):
	global widthG
	global depthG
	global bottomWidth
	global bottomDepth
	for node in nodeList:
		checkMut = random.randint(1,100)
		if checkMut <=5:
			notChanged = True
			while notChanged:
				XValue = 0
				ZValue = 0
				XValue = randint(0,widthG)
				ZValue = randint(0,depthG)
				XValue = bottomWidth + XValue
				ZValue = bottomDepth + ZValue
				newNode = gennode(Xvalue,0,ZValue,0)
				newNode.y = getY(tileMap,newNode)
				for nodeU in nodeList:
					if nodeU.x == newNode.x and nodeU.y == newNode.y and nodeU.z == newNode.z:
						unique = False
						break
				block = level.blockAt(newNode.x, newNode.y, newNode.z)
				data = level.blockDataAt(newNode.x, newNode.y, newNode.z)
				newNode.material = (block,data)
				if unique:
					if block == 35 and data == 4:
						continue
					if block == 35 and data == 11:
						continue
					if block == 9 and data == 0:
						continue
					if block == 11 and data == 0:
						continue
					if block == 5 and data == 1:
						continue
					if block == 1 and data == 4:
						continue
					node.x = XValue
					node.z = ZValue
					node.y = newNode.y
					node.material = newNode.material
					notChanged = False
	return nodeList

def crossover(parent1,parent2):
	newList = []
	crossoverPoint = random.randint(1,len(parent1.nodelist))
	for i in range(0,len(parent1.nodelist)):
		if i < crossoverPoint:
			newList.append(parent1.nodelist[i])
		else:
			newList.append(parent2.nodelist[i])
	return newList

def changeBlocks(individual,level,box,tileMap):
	for nodess in individual.nodelist:
		utilityFunctions.setBlock(level, (4, 0), nodess.x, nodess.y, nodess.z)
	print("Blocks Changed")

def resetBlocks(individual,level,box,tileMap):
	for tiles in individual.nodelist:
		utilityFunctions.setBlock(level, tiles.material, tiles.x, tiles.y, tiles.z)
	print("Blocks Reset")
	return

def getParents(population,parent1,parent2):
	for indi in population:
		if indi.fitness > parent1.fitness:
			parent2.nodelist = parent1.nodelist
			parent2.fitness = parent1.fitness
			parent1.nodelist = indi.nodelist
			parent1.fitness = indi.fitness
	print("P1F: ",parent1.fitness,"P2F: ",parent2.fitness)

def generateNewPop(population,level,box,tileMap):
	pop = []
	parent1 = individual([],0)
	parent2 = individual([],0)
	getParents(population,parent1,parent2)
	for x in range(0,5):
		indiv = individual([],1000)
		indiv.nodelist = genNewIndividual(population,parent1,parent2,level,box,tileMap)
		pop.append(indiv)
	return pop

def genNewIndividual(population,parent1,parent2,level,box,tileMap):
	nodeList = []
	del nodeList[:]
	global budget
	global widthG
	global depthG
	global bottomWidth
	global bottomDepth
	budget = getBudget(level, box, tileMap)
	nodeList = crossover(parent1,parent2)
	nodeList = mutation(nodeList)
	unique = True
	return nodeList

def generateInitialPop(population,level,box,tileMap):
	#Create 5 individuals for the initial population
	for x in range(0,5):
		indiv = individual([],1000)
		indiv.nodelist = genInitialIndividual(indiv,level,box,tileMap)
		population.append(indiv)
	return population

def genInitialIndividual(indiv,level,box,tileMap):
	nodeList = []
	del nodeList[:]
	global budget
	global widthG
	global depthG
	global bottomWidth
	global bottomDepth
	budget = getBudget(level, box, tileMap)
	while budget > 0:
		XValue = 0
		ZValue = 0
		XValue = randint(0,widthG)
		ZValue = randint(0,depthG)
		XValue = bottomWidth + XValue
		ZValue = bottomDepth + ZValue
		newNode = gennode(XValue,0,ZValue,(0,0))
		newNode.y = getY(tileMap,newNode)
		block = level.blockAt(newNode.x, newNode.y, newNode.z)
		data = level.blockDataAt(newNode.x, newNode.y, newNode.z)
		newNode.material = (block,data)
		unique = True
		for nodeU in indiv.nodelist:
			if nodeU.x == newNode.x and nodeU.y == newNode.y and nodeU.z == newNode.z:
				unique = False
				break
		if unique:
			if block == 35 and data == 4:
				continue
			if block == 35 and data == 11:
				continue
			if block == 9 and data == 0:
				continue
			if block == 11 and data == 0:
				continue
			if block == 5 and data == 1:
				continue
			if block == 1 and data == 4:
				continue
			#print(newNode.x,newNode.z)
			nodeList.append(newNode)
			budget = budget - 1
	return nodeList

def Astar(level, box, house, tileMap):
	global tooHigh
	openList = []
	closedList = []
	del openList[:]
	del closedList[:]
	currentNode = 0
	gcost = 0
	global endFound
	endFound = False
	startNode = node(house.x,house.y,house.z,0,0,0)
	openList.append(startNode)
	for work in listOfWorkplaces:
		if work.number == house.workplaceNumber:
			endNode = node(work.x,work.y,work.z,0,0,0)
	while len(openList) > 0:
		#if you have reached the end node, stop searching and get the path taken to the start
		if endFound:
			pathToEnd = getPath(startNode,currentNode)
			savePath(house,currentNode,pathToEnd,tileMap,level,box)
			return
		#have an f value so high it will definitely get replaced every iteration
		lowestF = 10000
		#check open list for the lowest F value in it to be explored next
		for option in openList:
			if option.fvalue<lowestF:
				lowestF = option.fvalue
				currentNode = option
		#add the current node to the closed list
		openList.remove(currentNode)
		#go in the four cardinal directions to get children of the currentNode
		for direction in range(1,5):
			tooHigh = False
			#to the side of the current node
			if direction == 1:
				childNode = node(currentNode.x+1,currentNode.y,currentNode.z,0,0,currentNode)
			#to the other side of the current node
			if direction == 2:
				childNode = node(currentNode.x-1,currentNode.y,currentNode.z,0,0,currentNode)
			#to the 'top' of the current node
			if direction == 3:
				childNode = node(currentNode.x,currentNode.y,currentNode.z+1,0,0,currentNode)
			#to the 'bottom' of the current node relatively speaking
			if direction == 4:
				childNode = node(currentNode.x,currentNode.y,currentNode.z-1,0,0,currentNode)
			childNode.y = getY(tileMap,childNode)
			getTooHigh(currentNode.y,childNode.y)
			if tooHigh:
				continue
			elif checkMat(childNode.x,childNode.y,childNode.z, level, box):
				gcost = getCost(childNode.x,childNode.y,childNode.z,level,box,currentNode.y,getY(tileMap,childNode))
				# cost to get to current square plus cost to child node
				gcost = currentNode.gcost + gcost
				#now set the coordinates of the child node
				hcost = getHeuristic(childNode,endNode)
				childNode.gcost = gcost
				fvalue = gcost + hcost
				childNode.fvalue = fvalue
				if checkOpenList(childNode, openList):
					continue
				else:
					checkClosedList(childNode, closedList, openList)

		closedList.append(currentNode)

def getTileMap (level, box, maxHeight, minHeight):
	#This getTileMap() procedure is not written by me this is from the CoGTutorialPillar.py filter by Rodrigo Canaan
	xmin = box.minx
	xmax = box.maxx
	zmin = box.minz
	zmax = box.maxz

	width = xmax-xmin+1
	depth = zmax-zmin+1

	global widthG
	global depthG

	global bottomWidth
	global bottomDepth

	widthG = xmax-xmin
	depthG = zmax-zmin

	widthG = widthG - 1
	depthG = depthG - 1

	bottomWidth = xmin
	bottomDepth = zmin

	tileMap = empty( (width,depth) ,dtype=object)

	for x in range(xmin-1,xmax+1):
		for z in range(zmin-1,zmax+1):
			for y in range(maxHeight, minHeight-1,-1):
				material = level.blockAt(x,y,z)
				if (material != 0):
					tileMap[x-xmin,z-zmin] = tile(x,y,z,material)
					#print"Found block of type {} at position({},{},{})".format(material,x,y,z)
					break

	return tileMap

def getY(tileMap, tileXZ):
	ymax = 0
	for tile in tileMap.flatten():
		if (tile.x == tileXZ.x and tile.z == tileXZ.z):
			ymax = tile.y
	return ymax

def getBudget(level, box, tileMap):
	global widthG
	global depthG
	totalspaces = widthG * depthG
	totalspaces = totalspaces / 10
	budg = round(totalspaces)
	return budg

def findHouses(level, box):
	for x in xrange(box.minx, box.maxx):
		for z in xrange(box.minz, box.maxz):
			for y in xrange(box.miny, box.maxy):
				block = level.blockAt(x, y, z)
				data = level.blockDataAt(x, y, z)
				#this checks if the block is Yellow Wool, which is the block type being used to denote a house's front door
				if block == 35 and data == 4:
					newhouse = house(x,y,z,(len(listOfHouses)+1),randint(1,len(listOfWorkplaces)))
					listOfHouses.append(newhouse)

def getPath(startNode,currentNode):
	#this procedure returns an array of the nodes taken to reach the end goal node
	pathTaken = []
	nextNode = 0
	nextNode = currentNode
	while nextNode != startNode:
		pathTaken.append(nextNode)
		nextNode = nextNode.parent
	return pathTaken

def getCost(x,y,z,level, box, thisY, thatY):
	cost = blockCosts(x,y,z,level, box) + (abs((abs(thisY)-abs(thatY)))*3)
	return cost

def checkOpenList(childNode,openList):
	for listNode in openList:
		if listNode.x == childNode.x and listNode.y == childNode.y and listNode.z == childNode.z:
			if listNode.gcost <= childNode.gcost:
				return True
	return False

def checkClosedList(childNode,closedList,openList):
	for listNode in closedList:
		if listNode.x == childNode.x and listNode.y == childNode.y and listNode.z == childNode.z:
			if listNode.gcost > childNode.gcost:
				closedList.remove(listNode)
				openList.append(listNode)
	openList.append(childNode)
	return

def getTooHigh(thisY, thatY):
	global tooHigh
	difference = abs((abs(thisY)-abs(thatY)))
	if difference >= 2:
		tooHigh = True
	else:
		tooHigh = False
	return

def getHeuristic(thisPoint, end):
	xThis = thisPoint.x
	zThis = thisPoint.z
	xEnd = end.x
	zEnd = end.z
	return abs(abs(abs(xThis) - abs(xEnd)) + abs(abs(zThis) - abs(zEnd)))

def savePath(house,currentNode,pathTaken,tileMap,level,box):
	global paths
	thisPath = pathcosthouse(pathTaken,currentNode.gcost,house)
	paths.append(thisPath)
	return

def blockCosts(x,y,z,level,box):
	block = level.blockAt(x, y, z)
	data = level.blockDataAt(x, y, z)
	#cobblestone is slightly better than dirt
	if block == 4 and data == 0:
		return 0.8
	#sand is worse than dirt
	if block == 12 and data == 0:
		return 1.2
	#quartz is the most efficient road type
	if block == 155 and data == 0:
		return 0.5
	#dirt blocks
	if block == 2 and data == 0:
		return 1
	#grass blocks
	if block == 3 and data == 0:
		return 1

def checkMat(x,y,z,level,box):
	global endFound
	#this procedure checks if the block is deemed as passable terrain so buildings arent deleted making the Road
	block = level.blockAt(x, y, z)
	data = level.blockDataAt(x, y, z)
	if block == 35 and data == 4:
		return False
	if block == 35 and data == 11:
		endFound = True
		return False
	if block == 9 and data == 0:
		return False
	if block == 11 and data == 0:
		return False
	if block == 5 and data == 1:
		return False
	if block == 1 and data == 4:
		return False
	return True

def fixHouses():
	currentWorkplaces = []
	for h in listOfHouses:
		currentWorkplaces.append(h.workplaceNumber)
	for work in listOfWorkplaces:
		if work.number not in currentWorkplaces:
			dupes = 0
			for h in listOfHouses:
				for i in range(len(listOfWorkplaces)+1):
					if i == h.workplaceNumber:
						dupes += 1
						if dupes > 1:
							h.workplaceNumber = work.number
							return

def findWorkplaces(level, box):
	for x in xrange(box.minx, box.maxx):
		for z in xrange(box.minz, box.maxz):
			for y in xrange(box.miny, box.maxy):
				block = level.blockAt(x, y, z)
				data = level.blockDataAt(x, y, z)
				#this checks if the block is Blue Wool, which is the block type being used to denote a workplace's front door
				if block == 35 and data == 11:
					newworkplace = workplace(x,y,z,(len(listOfWorkplaces)+1))
					listOfWorkplaces.append(newworkplace)
