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

inputs = (
	("A* Road Generator", "label"),
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
	def __init__(self, x, y, z, fvalue, gcost, parent):
		self.x = x
		self.y = y
		self.z = z
		self.fvalue = fvalue
		self.gcost = gcost
		self.parent = parent

def perform(level, box, options):
	seed()
	tileMap = getTileMap(level, box, 255, 0)
	findWorkplaces(level, box)
	findHouses(level, box)
	fixHouses()
	for house in listOfHouses:
		Astar(level, box, house, tileMap)
	for house in listOfHouses:
		if len(listOfHouses) < 1:
			print("Please select a house")
		print(house.number, house.x, house.y, house.z, house.workplaceNumber)
	for workplace in listOfWorkplaces:
		if len(listOfWorkplaces) < 1:
			print("Please pick a workplace")
		print(workplace.number, workplace.x, workplace.y, workplace.z)

def Astar(level, box, house, tileMap):
	global tooHigh
	openList = []
	closedList = []
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
			placePath(pathToEnd,tileMap,level,box)
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
			getTooHigh(tileMap,childNode)
			if tooHigh:
				continue
			elif checkMat(childNode.x,childNode.y,childNode.z, level, box):
				gcost = getCost(currentNode.y,getY(tileMap,childNode))
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

def checkOpenList(childNode,openList):
	for listNode in openList:
		if listNode.x == childNode.x and listNode.y == childNode.y and listNode.z == childNode.z:
			if listNode.gcost <= childNode.gcost:
				return True
	return False

def checkClosedList(childNode,closedList,openList):
	for listNode in closedList:
		if listNode.x == childNode.x and listNode.y == childNode.y and listNode.z == childNode.z:
			if listNode.gcost <= childNode.gcost:
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

def placePath(pathTaken,tileMap,level,box):
	for node in pathTaken:
		utilityFunctions.setBlock(level, (4, 0), node.x, node.y, node.z)

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

	widthG = xmax-xmin+1
	depthG = xmax-xmin+1

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

def getHeuristic(thisPoint, end):
	xThis = thisPoint.x
	zThis = thisPoint.z
	xEnd = end.x
	zEnd = end.z
	return (abs(abs(xThis) - abs(xEnd)) + abs(abs(zThis) - abs(zEnd)))
