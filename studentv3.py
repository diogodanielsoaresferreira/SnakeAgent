import heapq
from snake import Snake
from constants import *
from game import SnakeGame

'''

LEFT:
-> Verificar se é viável ir para um certo "ponto" i.e. se não leva a nenhum beco sem saída
-> Falta ver porque é que a snake morre tantas vezes.
-> Ver tempo que snake demora e melhorar
-> Se duas comidas, apostamos na comida mais perto de nós relativamente à outra cobra.
-> Fazer certas coisas SÓ UMA VEZ

FEATURES DONE:
-> A* without other snake (except on next move) (with condition to avoid loops)
-> Chicken condition if our points<=other snake points (with condition to avoid loops)
-> We know the tail and head of other snake
-> Quando fica sem saída, tenta acertar com corpo de outra cobra,
ou contra nosso corpo, ou contra obstáculo, pos esta ordem.
-> Entalar outra cobra contra obstaculo.
-> Cabeça com Cabeça se ganhar
-> Dá para ativar pruning

'''



class StudentPlayer3(Snake):
	def __init__(self, game, body=[(0,0)], direction=(1,0)):
		super().__init__(body,direction,name="Zé da banana")
		# Last maze
		self.last_maze = []
		# Points on last round
		self.last_points = []

		self.game = game

		self.goal = [0,0]

	# Adds two points in the map
	def add(self,a,b):
		added_d = (a[0]+b[0])%(self.mapsize[0]+1),(a[1]+b[1])%(self.mapsize[1]+1)
		return added_d

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

	# Updates game data
	def update(self,points=None, mapsize=None, count=None, agent_time=None):
		if(self.last_points != self.points):
			self.cc_data = {'number':0, 'heuristic':1000}
			self.snake_collision = 0
		self.mapsize=mapsize
		self.count=count
		self.last_points = self.points
		self.agent_time = agent_time

	def updateDirection(self,maze):

		print(self.body[0])
		self.goal = self.get_goal(maze, self.body[0], maze.foodpos)

		# Get best path
		dir = [1,0]
		paths, costs = self.a_star_search(maze)

		# If finds a way to food
		if(maze.foodpos in paths):

			path = self.get_path(paths, self.body[0], maze.foodpos)+[maze.foodpos]


			if(len(path)>1):
				dir = self.get_dir(path[1], self.body[0])
				self.direction=dir
				return

		self.direction=dir

	def get_path(self, paths, start, goal):
		return [] if goal==start else self.get_path(paths, start, paths[goal])+[paths[goal]]

	# Return valid neighbours for current position
	def get_neighbours(self, maze, position, last_pos, count_with_other_snake=True):
		olddir = [0,0]

		# If last position is none, count with actual direction
		if last_pos==None:
			olddir = list(self.direction)
		# We don't have information about it
		elif last_pos == -1:
			olddir = [0,0]
		else:
			olddir = self.get_dir(position, last_pos)

		complement=[(up,down),(down,up),(right,left),(left,right)]

		invaliddir=[x for (x,y) in complement if list(y)==olddir]
		validdir=[dir for dir in directions if not ( dir in invaliddir )]

		
		validdir=[self.add(dir,position) for dir in validdir if not (self.add(position,dir) in maze.obstacles or self.add(position,dir) in self.body)]

		return validdir

	# Manhattan Distance, as read here: http://theory.stanford.edu/~amitp/GameProgramming/Heuristics.html
	# Also read for breaking ties (here with 'p' variable)
	def heuristic(self, a, b):

		best_x_move = min(abs(a[0]-b[0]), a[0]+self.mapsize[0]+1-b[0], b[0]+self.mapsize[0]+1-a[0])
		best_y_move = min(abs(a[1]-b[1]), a[1]+self.mapsize[1]+1-b[1], b[1]+self.mapsize[1]+1-a[1])

		# Used for breaking ties
		# p <(minimum cost of taking one step)/(expected maximum path length)
		p=1/(self.mapsize[0]+self.mapsize[1]+3)

		return best_x_move+best_y_move+p


	# Update other snake position
	def update_snakes(self, maze):
		
		# Get food position
		self.last_maze = maze

	# A star algorithm for search
	def a_star_search(self, maze):
		# Queue sorted by cost
		open_list = []
		open_list.append(Node(self.body[0], 0.0, self.heuristic(self.body[0], maze.foodpos)))
		paths = {}
		costs = {}
		paths[self.body[0]] = None
		costs[self.body[0]] = 0
		self.cc = False

		while open_list!=[]:
			position = open_list[0]
			open_list[0:1] = []
			

			if position.name==self.goal:
				return paths, costs
			
			best_h = 1000

			# Count with other snake only on next move or 
			# if the other snake already collided with us five or more time on the search for the same food

			valid_dirs = self.get_neighbours(maze, position.name, paths[position.name], (position.name==self.body[0] or self.snake_collision<3))
			for neighbour in valid_dirs:
				new_cost = costs[position.name] + 1.0
				if (neighbour not in costs or new_cost < costs[neighbour]):
					costs[neighbour] = new_cost
					act_cost = new_cost + self.heuristic(neighbour, maze.foodpos)
					open_list.append(Node(neighbour, new_cost, self.heuristic(neighbour, maze.foodpos), position.prof+1))
					paths[neighbour] = position.name

			open_list.sort(key=lambda node: node.cost+node.heur)

		return paths, costs

	# If food is too far away, reach next square
	def get_goal(self, maze, pos, food):
		n=30
		# Get n reference points
		hor = [i*self.mapsize[0]/n for i in range(n)]
		vert = [i*self.mapsize[1]/n for i in range(n)]
		#return food
		# Get possible goals
		goals = [ (h, v) for h in hor for v in vert if (h,v) not in maze.obstacles+[maze.foodpos]+self.body ]

		if self.heuristic(pos, food)<self.mapsize[0]/n+self.mapsize[1]/n:
			return food

		heur = [(self.heuristic(pos, goal),goal) for goal in goals if self.heuristic(pos, goal)<self.heuristic(pos, food) and self.heuristic(goal, food)<self.heuristic(pos, food)]

		if len(heur)==0:
			return food
		self.game.paint([min(heur, key=lambda x: x[0])[1]],(128,0,0))

		return min(heur, key=lambda x: x[0])[1]

class Node:
	def __init__(self, name, cost, heur, prof=0):
		self.name = name
		self.cost = cost
		self.heur = heur
		self.prof = prof
