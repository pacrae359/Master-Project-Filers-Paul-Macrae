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
heatMap = []
budget = 0
periteration = 0
widthG = 0
depthG = 0

inputs = (
	("Desire Paths Street Network (Iterative)", "label"),
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

class heatmaptile:
	def __init__(self, x, y, z, crossed, material):
		self.x = x
		self.y = y
		self.z = z
		self.crossed = crossed
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

def perform(level, box, options):
	global heatMap
	global budget
	global paths
	global periteration
	placed = 0
	periteration = 0
	keepDPValues = []
	desirePathValues = []
	continueIterations = True
	seed()
	tileMap = getTileMap(level, box, 255, 0)
	findWorkplaces(level, box)
	findHouses(level, box)
	fixHouses()
	totalpathcost = 0
	for house in listOfHouses:
		if len(listOfHouses) < 1:
			print("Please select a house")
		print(house.number, house.x, house.y, house.z, house.workplaceNumber)
	for workplace in listOfWorkplaces:
		if len(listOfWorkplaces) < 1:
			print("Please pick a workplace")
		print(workplace.number, workplace.x, workplace.y, workplace.z)
	budget = getBudget(level, box, tileMap)
	while continueIterations:
	#this line is only here to showcase the iterative nature of this implementation in the final development video.
	#for x in range(0,3):
		del keepDPValues[:]
		del desirePathValues[:]
		del heatMap[:]
		del paths[:]
		for house in listOfHouses:
			Astar(level, box, house, tileMap)
		findHeatMap(heatMap, paths, level, box, house, tileMap)
		desirePathValues = findDesirePaths(heatMap)
		if placed >= len(desirePathValues):
			continueIterations = False
		placed = placed + placeDesirePaths(keepDPValues,periteration,heatMap,desirePathValues,level,box,tileMap)
		if not continueIterations:
			print("huh: ",budget)
			placeQuartzDesirePaths(keepDPValues,periteration,heatMap,desirePathValues,level,box,tileMap)
			break

def Astar(level, box, house, tileMap):
	global budget
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

def findHeatMap(heatMap, paths, level, box, house, tileMap):
	for pathcosts in paths:
		for nodeInPath in pathcosts.path:
			updateValue = 0
			inHeatMap = False
			mat = level.blockAt(nodeInPath.x, nodeInPath.y, nodeInPath.z)
			for mapValue in heatMap:
				if nodeInPath.x == mapValue.x and nodeInPath.z == mapValue.z:
					inHeatMap = True
					mapValue.crossed = mapValue.crossed + 1
					break
			if not inHeatMap:
				hmt = heatmaptile(nodeInPath.x, nodeInPath.y, nodeInPath.z,1,mat)
				heatMap.append(hmt)
	return

def getBudget(level, box, tileMap):
	global widthG
	global depthG
	totalspaces = widthG * depthG
	totalspaces = totalspaces / 10
	budg = round(totalspaces)
	return budg

def placeDesirePaths(keepDPValues,periteration,heatMap,desirePathValues,level,box,tileMap):
	global budget
	placedthisiteration = 0
	#so for each block of cobble placed take 1 away from the budget, for each quartz placed take away 3. Quartz is only to be placed where most effective.
	#places cobblestone on locations of desire paths to begin with, while within the budget.
	while placedthisiteration != periteration and len(desirePathValues) > 0:
		currentCheck = desirePathValues[0]
		keepDPValues.append(desirePathValues.pop(0))
		for HMTile in heatMap:
			if HMTile.crossed == currentCheck:
				if level.blockAt(HMTile.x, HMTile.y, HMTile.z)!=4 and level.blockAt(HMTile.x, HMTile.y, HMTile.z)!=155:
					utilityFunctions.setBlock(level, (4, 0), HMTile.x, HMTile.y, HMTile.z)
					placedthisiteration = placedthisiteration + 1
					budget = budget - 1
					print(budget)
					break
	return placedthisiteration

def placeQuartzDesirePaths(keepDPValues,periteration,heatMap,desirePaths,level,box,tileMap):
	global budget
	while budget > 2:
		checkForQuartz = keepDPValues.pop(0)
		for HMTile in heatMap:
			if HMTile.crossed == checkForQuartz:
				 if level.blockAt(HMTile.x, HMTile.y, HMTile.z)==4:
					utilityFunctions.setBlock(level, (155, 0), HMTile.x, HMTile.y, HMTile.z)
					#this is -2 because the cost for the cobblestone path in place is removed first before adding the quartz cost
					budget = budget - 2
					break

def findDesirePaths(heatMap):
	global periteration
	heatmaptemp = []
	for value in heatMap:
		heatmaptemp.append(value)
	desirePaths = []
	mostCrossed = 0
	while len(heatmaptemp) > 0:
		for value in heatmaptemp:
			desirePaths.append(value.crossed)
			heatmaptemp.remove(value)
	desirePaths.sort(reverse=True)
	if periteration == 0:
		periteration = round(len(desirePaths) / 10)
	return desirePaths

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

def getPath(startNode,currentNode):
	#this procedure returns an array of the nodes taken to reach the end goal node
	pathTaken = []
	nextNode = 0
	nextNode = currentNode
	while nextNode != startNode:
		pathTaken.append(nextNode)
		nextNode = nextNode.parent
	return pathTaken

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
	#quartz is the fastest road type
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

	widthG = xmax-xmin
	depthG = zmax-zmin

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

#this returns the highest Y value at the current x,z coordinate which is the surface tile
def getY(tileMap, tileXZ):
	ymax = 0
	for tile in tileMap.flatten():
		if (tile.x == tileXZ.x and tile.z == tileXZ.z):
			ymax = tile.y
	return ymax

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

def getCost(x,y,z,level, box, thisY, thatY):
	cost = blockCosts(x,y,z,level, box) + (abs((abs(thisY)-abs(thatY)))*3)
	return cost

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
