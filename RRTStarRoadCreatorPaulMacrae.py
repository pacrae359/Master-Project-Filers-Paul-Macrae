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
widthG = 0
depthG = 0
tooHigh = False
newNodePathCost = 0
potentialPath = []

inputs = (
	("RRT* Road Generator", "label"),
	("Material", alphaMaterials.BlockofQuartz),
	("Creator: Paul Macrae", "label"),
	)
#keeping these here as a reference of which blocks I am interested in.
#alphaMaterials.BlockofQuartz The Road will be constructed out of Quartz.
#alphaMaterials.BlockofRedstone The faster road type will be made of Redstone Blocks.
#alphaMaterials.BlueWool Blue Wool Denotes the doorway to a workplace.
#alphaMaterials.YellowWool Yellow Wool denotes the doorway to the house.

# Class storing some properties of a house in the game
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

class node:
	def __init__(self, x, y, z, nodecost, startpathcost, parent, parentpath):
		self.x = x
		self.y = y
		self.z = z
		self.nodecost = nodecost
		self.startpathcost = startpathcost
		self.parent = parent
		self.parentpath = parentpath

def perform(level, box, options):
	seed()
	tileMap = getTileMap(level, box, 255, 0)
	findWorkplaces(level, box)
	findHouses(level, box)
	fixHouses()
	for house in listOfHouses:
		if len(listOfHouses) < 1:
			print("Please select a house")
		print(house.number, house.x, house.y, house.z, house.workplaceNumber)
	for workplace in listOfWorkplaces:
		if len(listOfWorkplaces) < 1:
			print("Please pick a workplace")
		print(workplace.number, workplace.x, workplace.y, workplace.z)
	for house in listOfHouses:
		RRTstar(level, box, house, tileMap)

def RRTstar(level, box, house, tileMap):
	nodeList = []
	global tooHigh
	currentNode = 0
	global endFound
	global potentialPath
	endFound = False
	startNode = node(house.x,house.y,house.z,0,0,0,0)
	nodeList.append(startNode)
	for work in listOfWorkplaces:
		if work.number == house.workplaceNumber:
			endNode = node(work.x,work.y,work.z,0,0,0,0)
	while endFound == False:
		skip = False
		tooHigh = False
		dontAdd = False
		#randomize a location for the next point within the selected area
		newPositionx = random.randint(0,widthG)
		newPositionz = random.randint(0,depthG)
		#this sets up the maximum distance away from an existing node a new node can be
		distanceX = 3
		distanceZ = 3
		#use the randomized values to pick the next node randomly
		#nodeX = box.minx+newPositionx
		#nodeZ = box.minz+newPositionz
		nodeY = 0
		nextNode = node(nodeX, nodeY, nodeZ, 0,0,0,0)
		nodeY = getY(tileMap, nextNode)
		nextNode.y = nodeY
		print("New Node: ", nextNode.x, nextNode.y, nextNode.z)
		for potentialParent in nodeList:
			if abs(nextNode.x - potentialParent.x) == 0 and abs(nextNode.z - potentialParent.z) == 0:
				skip = True
		# check if the random node is within the range of the maximum placement radius for all possible parent nodes, if its not get a new one
		if not skip:
			for potentialParent in nodeList:
				if abs(nextNode.x - potentialParent.x) <= distanceX and abs(nextNode.z - potentialParent.z) <= distanceZ:
					# check if path to chosen node is not obstructed. If it isnt then connect it to the node that would give it the fastest path back to the start
					if getInterPath(potentialParent, nextNode, level, box, tileMap,0):
						if nextNode.startpathcost == 0 or (potentialParent.startpathcost + newNodePathCost) < nextNode.startpathcost:
							nextNode.nodecost = newNodePathCost
							nextNode.startpathcost = potentialParent.startpathcost + newNodePathCost
							nextNode.parent = potentialParent
							nextNode.parentpath = potentialPath
						if checkEnd(endNode,nextNode,distanceX,distanceZ):
							#if you find the end, set the endnode as the next Node and set the current node (if it can reach the end actually) as the parent of it.

							

							pathToEnd = getPath(startNode,nextNode)
							placePath(pathToEnd,tileMap,level,box, tileMap)
			if nextNode.parent != 0:
				nodeList.append(nextNode)
		#ADDITIONALLY NEED TO CHECK IF ANY NEARBY NODES CAN BE ADJUSTED TO CONNECT TO THE CURRENT ONE OVER THEIR CURRENT PARENT



		pathToEnd = getPath(startNode,nextNode)
		placePath(pathToEnd,startNode,nextNode,tileMap,level,box)

		endFound = True

def checkEnd(endNode,nextNode, distanceX, distanceZ):
	if abs(endNode.x - nextNode.x) <= distanceX and abs(endNode.z - nextNode.z) <= distanceZ:
		return True
	else:
		return False

def getPath(startNode,currentNode):
	#this procedure returns an array of the blocks used to reach the end goal node
	pathTaken = []
	nextNode = 0
	nextNode = currentNode
	while nextNode.parent != 0:
		for nodes in nextNode.parentpath:
			pathTaken.append(nodes)
		nextNode = nextNode.parent
	return pathTaken

def placePath(pathTaken,startNode,nextNode,tileMap,level,box):
	for node in pathTaken:
			utilityFunctions.setBlock(level, (4, 0), node.x, node.y, node.z)

def getY(tileMap, tileXZ):
	ymax = 0
	for tile in tileMap.flatten():
		if (tile.x == tileXZ.x and tile.z == tileXZ.z):
			ymax = tile.y
	return ymax

def getCost(thisY, thatY):
	cost = 1 + (abs((abs(thisY)-abs(thatY)))*3)
	return cost

def getTooHigh(thisY, thatY):
	global tooHigh
	difference = abs((abs(thisY)-abs(thatY)))
	if difference >= 2:
		tooHigh = True
	else:
		tooHigh = False
	return

def getInterPath(potentialParent, nextNode, level, box, tileMap, XorZ):
	global potentialPath
	potentialPath = []
	global tooHigh
	tooHigh = False
	global newNodePathCost
	newNodePathCost = 0
	potentialPath.append(nextNode)
	thisNode = nextNode
	changeMade = False
	newXValue = 0
	newZValue = 0
	newYValue = 0
	counter = 0
	xdiff = abs(potentialParent.x - thisNode.x)
	zdiff =  abs(potentialParent.z - thisNode.z)
	while thisNode.x != potentialParent.x or thisNode.z != potentialParent.z:
		if checkMat(thisNode.x,thisNode.y,thisNode.z, level, box):
			xabove = False
			zabove = False
			xbelow = False
			zbelow = False
			counter = counter + 1
			print("count: ",counter)
			print(thisNode.x, thisNode.z)
			print(potentialParent.x, potentialParent.z)
			changeMade = False
			newXValue = thisNode.x
			newZValue = thisNode.z
	#I have to know if the new nodes X and Z values are above or below the potential parent node so I can construct a path back to the parent
			if thisNode.x > potentialParent.x:
				xabove = True
			if thisNode.z > potentialParent.z:
				zabove = True
			if thisNode.x < potentialParent.x:
				xbelow = True
			if thisNode.z < potentialParent.z:
				zbelow = True
			if XorZ == 0:
				if xabove and not changeMade:
					newXValue = thisNode.x-1
					changeMade = True
				if xbelow and not changeMade:
					newXValue = thisNode.x+1
					changeMade = True
				if zabove and not changeMade:
					newZValue = thisNode.z-1
					changeMade = True
				if zbelow and not changeMade:
					newZValue = thisNode.z+1
					changeMade = True
			else:
				if zabove and not changeMade:
					newZValue = thisNode.z-1
					changeMade = True
				if zbelow and not changeMade:
					newZValue = thisNode.z+1
					changeMade = True
				if xabove and not changeMade:
					newXValue = thisNode.x-1
					changeMade = True
				if xbelow and not changeMade:
					newXValue = thisNode.x+1
					changeMade = True
			oldYValue = thisNode.y
	#must check if each block on the way back is too high
			thisNode = node(newXValue,0,newZValue,0,0,0,0)
			newYValue = getY(tileMap,thisNode)
			getTooHigh(oldYValue,newYValue)
			if tooHigh:
				if XorZ == 0:
					print("RECALL")
					if getInterPath(potentialParent, nextNode, level, box, tileMap, 1):
						print(potentialPath)
						return True
				return False
			else:
				thisNode.y = newYValue
				#tally up the total cost required to get back to the parent node
				newNodePathCost = newNodePathCost + getCost(oldYValue,newYValue)
				#add the block to the path between the nodes
				print("Added Node")
				if(thisNode.x == potentialParent.x and thisNode.z == potentialParent.z):
					return True
				else:
					potentialPath.append(thisNode)
		else:
			if XorZ == 0:
				print("RECALL")
				if getInterPath(potentialParent, nextNode, level, box, tileMap, 1):
					print(potentialPath)
					return True
			return False
	print(len(potentialPath))
	print("clearly they are equal")
	return True

def checkMat(x,y,z,level,box):
	global endFound
	#this procedure checks if the block is deemed as passable terrain so buildings arent deleted making the Road
	block = level.blockAt(x, y, z)
	data = level.blockDataAt(x, y, z)
	if block == 35 and data == 4:
		return False
	if block == 35 and data == 11:
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

	widthG = xmax-xmin+1
	depthG = zmax-zmin+1

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

#The number of houses and workplaces must be found and allocated to each other so paths can then be created between them.
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
