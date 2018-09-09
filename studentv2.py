# Diogo Daniel Soares Ferreira
# Luís Davide Leira
# Student Agent
import heapq
from snake import Snake
from constants import *
from game import SnakeGame
from functools import reduce

class Snake2:
	def __init__(self):
		self.body = []
		self.head = []
		self.points = 0
		self.tail = []
		self.last_body = []
		self.last_head_positions = []


class StudentPlayer2(Snake):
	def __init__(self, game, body=[(0,0)], direction=(1,0)):
		super().__init__(body,direction,name="Salta Pocinhas")
		self.game = game
		self.jumps = []
		self.jps = None

	# Adds two points in the map
	def add(self,a,b):
		added_d = (a[0]+b[0])%(self.mapsize[0]+1),(a[1]+b[1])%(self.mapsize[1]+1)
		return added_d

	# Updates game data
	def update(self,points=None, mapsize=None, count=None, agent_time=None):
		self.mapsize=mapsize
		self.count=count
		self.agent_time = agent_time

	def updateDirection(self,maze):

		self.jps = JPS(self, self.body[0], maze.foodpos, maze)
		self.path = self.jps.run("JPS")

		self.game.paint([e.pos for e in self.path], (128,50,50))

		self.direction=[1,0]

	def get_path(self, posd1, posd2, maze):
		# No Way
		if posd1.pos==posd2.pos:
			return None
		path = []
		p = []
		dic = {}


		validdir=[self.add(dir,posd1.pos) for dir in directions if not (self.add(posd1.pos,dir) in maze.obstacles or self.add(posd1.pos,dir) in self.body)]

		if self.add(self.jps.get_dir(posd2.pos, posd1.pos),posd1.pos) in validdir:
			return [self.add(self.jps.get_dir(posd2.pos, posd1.pos),posd1.pos)]
		elif self.jps.get_dir(posd2.pos, posd1.pos)==[1,1]:
			if self.add(posd1.pos, [1,0]) not in maze.obstacles:
				return [self.add(posd1.pos, [1,0]), self.add(posd1.pos, [1,1])]
			if self.add(posd1.pos, [0,1]) not in maze.obstacles:
				return [self.add(posd1.pos, [0,1]), self.add(posd1.pos, [1,1])]
		elif self.jps.get_dir(posd2.pos, posd1.pos)==[-1,-1]:
			if self.add(posd1.pos, [-1,0]) not in maze.obstacles:
				return [self.add(posd1.pos, [-1,0]), self.add(posd1.pos, [-1,-1])]
			if self.add(posd1.pos, [0,-1]) not in maze.obstacles:
				return [self.add(posd1.pos, [0,-1]), self.add(posd1.pos, [-1,-1])]
		elif self.jps.get_dir(posd2.pos, posd1.pos)==[1,-1]:
			if self.add(posd1.pos, [1,0]) not in maze.obstacles:
				return [self.add(posd1.pos, [1,0]), self.add(posd1.pos, [1,-1])]
			if self.add(posd1.pos, [0,-1]) not in maze.obstacles:
				return [self.add(posd1.pos, [0,-1]), self.add(posd1.pos, [1,-1])]
		elif self.jps.get_dir(posd2.pos, posd1.pos)==[-1,1]:
			if self.add(posd1.pos, [-1,0]) not in maze.obstacles:
				return [self.add(posd1.pos, [-1,0]), self.add(posd1.pos, [-1,1])]
			if self.add(posd1.pos, [0,1]) not in maze.obstacles:
				return [self.add(posd1.pos, [0,1]), self.add(posd1.pos, [-1,1])]

		return None

	# If food is too far away, reach next square
	def get_goal(self, maze, pos, food):
		n=30
		# Get n reference points
		hor = [i*self.mapsize[0]/n for i in range(n)]
		vert = [i*self.mapsize[1]/n for i in range(n)]

		goals = [ (h, v) for h in hor for v in vert if (h,v) not in maze.obstacles+[maze.foodpos]+self.body ]

		if self.heuristic(pos, food)<self.mapsize[0]/n+self.mapsize[1]/n:
			return food

		heur = [(self.heuristic(pos, goal),goal) for goal in goals if self.heuristic(pos, goal)<self.heuristic(pos, food) and self.heuristic(goal, food)<self.heuristic(pos, food)]

		if len(heur)==0:
			return food
		self.game.paint([min(heur, key=lambda x: x[0])[1]],(128,0,0))

		return min(heur, key=lambda x: x[0])[1]
		

class JPS:
	def __init__(self, snake, start, goal, maze):
		self.snake = snake
		self.start = start
		self.goal = goal
		self.maze = maze
		self.all_list = {} # PosDir to best travel-cost
		self.queue = []    # Priority queue on traveled + estimate
		self.visited = []

		for dx in [-1, 0, 1]:
			for dy in [-1, 0, 1]:
				if dx != 0 or dy != 0:
					self.add_node(self.start[0], self.start[1], (dx, dy), 0)


	# Manhattan Distance, as read here: http://theory.stanford.edu/~amitp/GameProgramming/Heuristics.html
	# Also read for breaking ties (here with 'p' variable)
	def heuristic(self, pos, dir):

		if dir!=None:
			pos = self.snake.add(pos, dir)
		best_x_move = min(abs(pos[0]-self.goal[0]), pos[0]+self.snake.mapsize[0]+1-self.goal[0], self.goal[0]+self.snake.mapsize[0]+1-pos[0])
		best_y_move = min(abs(pos[1]-self.goal[1]), pos[1]+self.snake.mapsize[1]+1-self.goal[1], self.goal[1]+self.snake.mapsize[1]+1-pos[1])

		# Used for breaking ties
		# p <(minimum cost of taking one step)/(expected maximum path length)
		p=1/(self.snake.mapsize[0]+self.snake.mapsize[1]+3)

		return best_x_move+best_y_move+p

	def get_dir(self, a, b):
		
		dir = [1,0]
		dir[0] = (a[0]-b[0])
		if dir[0]<-1:
			dir[0] = 1
		elif dir[0]>1:
			dir[0] = -1
		dir[1] = (a[1]-b[1])
		if dir[1]<-1:
			dir[1] = 1
		elif dir[1]>1:
			dir[1] = -1
		return dir


	def add_node(self, x, y, dir, dist):
		
		pd = PosDir((x, y), dir)
		current = self.all_list.get(pd)
		if current is None or current > dist:
			total = dist + self.heuristic(pd.pos, dir)
			self.all_list[pd] = dist
			self.add_open(total, pd, dist)

		return pd

	def get_closed_node(self, x, y, dir, dist):
		
		pd = PosDir((x, y), dir)
		current = self.all_list.get(pd)
		if current is not None and current <= dist:
			return pd

		self.all_list[pd] = dist
		return pd

	def add_open(self, total, pd, dist):
		heapq.heappush(self.queue, (total, pd, dist))

	def get_open(self):
		while True:
			if len(self.queue) == 0:
				return None, None, None

			total, pd, dist = heapq.heappop(self.queue)
			current = self.all_list.get(pd)
			if dist == current:
				return total, pd, dist

	def step(self, dist, elm):
		
		if elm.pos == self.goal:
			return elm

		hor_dir, vert_dir = elm.dir

		if hor_dir != 0 and vert_dir != 0:
			nodes = self.search_diagonal(elm.pos, hor_dir, vert_dir, dist)
		elif hor_dir != 0:
			nodes = self.search_hor(elm.pos, hor_dir, dist)
		else:
			nodes = self.search_vert(elm.pos, vert_dir, dist)
		for nd in nodes:
			nd.set_parent(elm)

		return None

	def search_hor(self, pos, hor_dir, dist):
		x0, y0 = pos

		while True:
			#print("H")
			
			x1 = x0 + hor_dir
			#x1 = self.snake.add(pos,[hor_dir,0])[0]

			# CHANGE THIS IF RULE
			if x1<0 or y0<0 or x1>self.snake.mapsize[0] or y0>self.snake.mapsize[0]:
				return []

			if (x1, y0) in self.maze.obstacles+self.snake.body:
				return []

			if (x1, y0) == self.goal:
				return [self.add_node(x1, y0, None, dist + 1)]

			dist = dist + 1
			x2 = x1 + hor_dir
			#x2 = self.snake.add([x1,pos[1]],[hor_dir,0])[0]

			nodes = []
			#if self.snake.add([x1,y0],[0,-1]) in self.maze.obstacles and not self.snake.add([x2,y0],[0,-1]) in self.maze.obstacles:
			if (x1, y0 - 1) in self.maze.obstacles+self.snake.body and not (x2, y0 - 1) in self.maze.obstacles+self.snake.body:
				nodes.append(self.add_node(x1, y0, (hor_dir, -1), dist))

			#if self.snake.add([x1,y0],[0,1]) in self.maze.obstacles and not self.snake.add([x2,y0],[0,1]) in self.maze.obstacles:
			if (x1, y0 + 1) in self.maze.obstacles+self.snake.body and not (x2, y0 + 1) in self.maze.obstacles+self.snake.body:
				nodes.append(self.add_node(x1, y0, (hor_dir, 1), dist))

			if len(nodes) > 0:
				nodes.append(self.add_node(x1, y0, (hor_dir, 0), dist))
				return nodes

			x0 = x1

	def search_vert(self, pos, vert_dir, dist):
		x0, y0 = pos

		while True:
			#print("V")
			y1 = y0 + vert_dir
			#y1 = self.snake.add(pos,[0,vert_dir])[1]
			
			# CHANGE THIS IF RULE
			if x0<0 or y1<0 or x0>self.snake.mapsize[0] or y1>self.snake.mapsize[0]:
				
				return []
			
			if (x0, y1) in self.maze.obstacles+self.snake.body:
				return []

			if (x0, y1) == self.goal:
				return [self.add_node(x0, y1, None, dist + 5)]

			dist = dist + 1
			# y2 = y1 + vert_dir
			y2 = self.snake.add([pos[0],y1],[0,vert_dir])[1]

			nodes = []
			#if self.snake.add([x0,y1],[-1,0]) in self.maze.obstacles and not self.snake.add([x0,y2],[-1,0]) in self.maze.obstacles:
			if (x0 - 1, y1) in self.maze.obstacles+self.snake.body and not (x0 - 1, y2) in self.maze.obstacles+self.snake.body:
				nodes.append(self.add_node(x0, y1, (-1, vert_dir), dist))

			#if self.snake.add([x0,y1],[1,0]) in self.maze.obstacles and not self.snake.add([x0,y2],[1,0]) in self.maze.obstacles:
			if (x0 + 1, y1) in self.maze.obstacles+self.snake.body and not (x0 + 1, y2) in self.maze.obstacles+self.snake.body:
				nodes.append(self.add_node(x0, y1, (1, vert_dir), dist))

			if len(nodes) > 0:
				nodes.append(self.add_node(x0, y1, (0, vert_dir), dist))
				return nodes

			y0 = y1

	def search_diagonal(self, pos, hor_dir, vert_dir, dist):

		x0, y0 = pos
		while True:
			#print("D")
			x1, y1 = x0 + hor_dir, y0 + vert_dir
			#x1, y1 = self.snake.add(pos, [hor_dir, vert_dir])

			# CHANGE THIS IF
			if x1<0 or y1<0 or x1>self.snake.mapsize[0] or y1>self.snake.mapsize[0]:
				return []

			if (x1, y1) in self.maze.obstacles+self.snake.body:
				return []

			if (x1, y1) == self.goal:
				return [self.add_node(x1, y1, None, dist + 1)]

			# Open space at (x1, y1)
			dist = dist + 2
			x2, y2 = x1 + hor_dir, y1 + vert_dir
			#x2, y2 = self.snake.add([x1, y1], [hor_dir, vert_dir])

			nodes = []
			if (x0, y1) in self.maze.obstacles+self.snake.body and not (x0, y2) in self.maze.obstacles+self.snake.body:
				nodes.append(self.add_node(x1, y1, (-hor_dir, vert_dir), dist))

			if (x1, y0) in self.maze.obstacles+self.snake.body and not (x2, y0) in self.maze.obstacles+self.snake.body:
				nodes.append(self.add_node(x1, y1, (hor_dir, -vert_dir), dist))
			
			hor_done, vert_done = False, False
			if len(nodes) == 0:
				sub_nodes = self.search_hor((x1, y1), hor_dir, dist)
				hor_done = True

				if len(sub_nodes) > 0:
					# Horizontal search ended with a spawn point.
					pd = self.get_closed_node(x1, y1, (hor_dir, 0), dist)
					for sub in sub_nodes:
						sub.set_parent(pd)

					nodes.append(pd)
			
			if len(nodes) == 0:
				sub_nodes = self.search_vert((x1, y1), vert_dir, dist)
				vert_done = True

				if len(sub_nodes) > 0:
					# Vertical search ended with a spawn point.
					pd = self.get_closed_node(x1, y1, (0, vert_dir), dist)
					for sub in sub_nodes:
						sub.set_parent(pd)

					nodes.append(pd)
			
			if len(nodes) > 0:
				if not hor_done:
					nodes.append(self.add_node(x1, y1, (hor_dir, 0), dist))

				if not vert_done:
					nodes.append(self.add_node(x1, y1, (0, vert_dir), dist))

				nodes.append(self.add_node(x1, y1, (hor_dir, vert_dir), dist))


				return nodes
			
			# Tile done, move to next tile.
			x0, y0 = x1, y1

	def run(self, alg_name):

		while True:
			total, pd, dist = self.get_open()
			if total is None:
				break

			pd = self.step(dist, pd)

			if pd is not None:
				break

		while True:
			total, pd, dist = self.get_open()
			if total is None:
				break

		jumps = self.getJumps(self.goal)


		#self.snake.game.paint([e.pos for e in self.getJPSPath(self.getJumps(self.goal))],(128,100,100))
		return self.getJumps(self.goal)

	def getJumps(self, pos):
		e1 = [elem.parent for elem in self.all_list if elem.pos==pos and elem.parent!=None]

		if len(e1)==0:
			return []

		elem2 = reduce(lambda r,h: h if self.all_list[h]<self.all_list[r] else r, e1)
		
		if elem2==None or elem2.pos==self.start:
			return [elem2]


		return [elem2]+list(set(self.getJumps(elem2.pos)))
	

class PosDir:
	
	def __init__(self, pos, dir):
		self.pos = pos
		self.dir = dir
		self.parent = None

	def set_parent(self, parent):
		assert self.parent is None
		self.parent = parent

	def __eq__(self, other):
		if not isinstance(other, PosDir):
			return False
		return self.pos == other.pos and self.dir == other.dir

	def __lt__(self, other):
		if not isinstance(other, PosDir):
			return False
		if self.pos != other.pos:
			return self.pos < other.pos
		return self.dir < other.dir

	def __hash__(self):
		return hash(self.pos) + hash(self.dir) * 737

	def __repr__(self):
		if self.parent!=None:
			return "PosDir({}, {}, {})".format(self.pos, self.dir, self.parent.pos)
		else:
			return "PosDir({}, {}, None)".format(self.pos, self.dir)