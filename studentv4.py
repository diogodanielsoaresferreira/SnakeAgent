# Diogo Daniel Soares Ferreira
# Luís Davide Leira
# Student Agent

import heapq
import pygame
from snake import Snake
from constants import *
from functools import reduce
import time

'''

LEFT:

-> Resolver loops infinitos
-> Dividir mapa em tamanhos diferentes para mapas pequenos e consoante cenas
-> Se passar de um certo limite, ignorar tempo máximo e calcular path
-> Aproveitar path calculado até à data

FEATURES:
-> Mapa só calculado quando é necessário
-> We know the tail and head of other snake
-> Entalar outra cobra contra obstaculo.
-> Cabeça com Cabeça se ganhar
-> Quando fica sem saída, tenta acertar com corpo de outra cobra,
ou contra nosso corpo, ou contra obstáculo, pos esta ordem.
-> Chicken Condition
-> Mapa dividido em partes para calcular next path
-> Se JPS demorar demasiado tempo, ir pela heurística
-> jps sempre que comida muda de tile
-> Verificar se next step não leva a um beco sem saída
'''

class Snake2:
	def __init__(self):
		self.body = []
		self.head = []
		self.points = 0
		self.tail = []
		self.last_body = []
		self.last_head_positions = []


class StudentPlayer4(Snake):
	def __init__(self, game, body=[(0,0)], direction=(1,0), name="Make Snake Great Again"):
		super().__init__(body,direction,name)
		self.game = game
		# Jumps to be made
		self.jumps = []
		# Jump Point Search instance
		self.jps = None
		# (Position of Food when it was calculed JPS
		self.last_food = None
		# Last maze
		self.last_maze = []
		# Points on last round
		self.last_points = []
		# Other Snake
		self.snake2 = Snake2()
		# Flag to calculate JPS
		self.calc_jps = False
		# Center position
		self.center = None
		# Save parts of map
		self.map_places = []
		# Number of tiles to divide the map
		self.n = 16
		# Tile of food
		self.food_tile = None
		# Last Positions of snake
		self.positions = []

	# Adds two points in the map
	def add(self,a,b):
		added_d = (a[0]+b[0])%(self.mapsize[0]),(a[1]+b[1])%(self.mapsize[1])
		return added_d
	
	# Retorna o número do mapa onde estamos com n posições, e as posições adjacentes.
	# Dividir o mapa em 1
	# [0]
	# Dividir o mapa em 4.
	# [0 1]
	# [2 3]
	# Dividir o mapa em 8
	# [0 1 2 3]
	# [4 5 6 7]
	# Dividir o mapa em 16
	# [0 1 2 3]
	# [4 5 6 7]
	# [8 9 10 11]
	# [12 13 14 15]
	def get_part_map(self, position):

		if self.n==1:
			return (0,[])

		if self.n==4:
			if position[0]<=self.center[0]:
				if position[1]<=self.center[1]:
					return (0,[1,2])
				else:
					return (2,[0,3])
			else:
				if position[1]<=self.center[1]:
					return (1,[0,3])
				else:
					return (3,[1,2])
		if self.n==8:
			if position[0]<=self.center[0]/2:
				if position[1]<=self.center[1]:
					return (0,[1,4,3])
				else:
					return (4,[0,5,7])
			elif position[0]<=self.center[0]:
				if position[1]<=self.center[1]:
					return (1,[0,2,5])
				else:
					return (5,[1,4,6])
			elif position[0]<=self.center[0]*(3/2):
				if position[1]<=self.center[1]:
					return (2,[1,3,6])
				else:
					return (6,[2,5,7])
			else:
				if position[1]<=self.center[1]:
					return (3,[0,2,7])
				else:
					return (7,[3,6,4])
		else:
			if position[0]<=self.center[0]/2:
				if position[1]<=self.center[1]/2:
					return (0,[1,3,4,12])
				elif position[1]<=self.center[1]:
					return (4,[0,8,5,7])
				elif position[1]<=self.center[1]*(3/2):
					return (8,[4,12,9,11])
				else:
					return (12,[8,13,0,15])

			elif position[0]<=self.center[0]:
				if position[1]<=self.center[1]/2:
					return (1,[2,13,5,0])
				elif position[1]<=self.center[1]:
					return (5,[1,4,9,6])
				elif position[1]<=self.center[1]*(3/2):
					return (9,[5,8,10,13])
				else:
					return (13,[9,12,14,1])

			elif position[0]<=self.center[0]*(3/2):
				if position[1]<=self.center[1]/2:
					return (2,[1,3,6,14])
				elif position[1]<=self.center[1]:
					return (6,[2,5,7,10])
				elif position[1]<=self.center[1]*(3/2):
					return (10,[6,9,11,14])
				else:
					return (14,[10,13,15,2])
			else:
				if position[1]<=self.center[1]/2:
					return (3,[2,7,0,15])
				elif position[1]<=self.center[1]:
					return (7,[3,6,11,4])
				elif position[1]<=self.center[1]*(3/2):
					return (11,[10,7,15,8])
				else:
					return (15,[12,14,11,3])

	def get_center_tile(self, tile):
		if self.n==1:
			return self.center
		if self.n==4:
			if tile==0:
				return (self.mapsize[0]/4, self.mapsize[1]/4)
			if tile==1:
				return (3*self.mapsize[0]/4, self.mapsize[1]/4)
			if tile==2:
				return (self.mapsize[0]/4, 3*self.mapsize[1]/4)
			else:
				return (3*self.mapsize[0]/4, 3*self.mapsize[1]/4)

		if self.n==8:
			if tile==0:
				return (self.mapsize[0]/8, self.mapsize[1]/4)
			if tile==1:
				return (3*self.mapsize[0]/8, self.mapsize[1]/4)
			if tile==2:
				return (5*self.mapsize[0]/8, self.mapsize[1]/4)
			if tile==3:
				return (7*self.mapsize[0]/8, self.mapsize[1]/4)
			if tile==4:
				return (self.mapsize[0]/8, 3*self.mapsize[1]/4)
			if tile==5:
				return (3*self.mapsize[0]/8, 3*self.mapsize[1]/4)
			if tile==6:
				return (5*self.mapsize[0]/8, 3*self.mapsize[1]/4)
			else:
				return (7*self.mapsize[0]/8, 3*self.mapsize[1]/4)
				
		if self.n==16:
			if tile==0:
				return (self.mapsize[0]/8, self.mapsize[1]/8)
			if tile==1:
				return (3*self.mapsize[0]/8, self.mapsize[1]/8)
			if tile==2:
				return (5*self.mapsize[0]/8, self.mapsize[1]/8)
			if tile==3:
				return (7*self.mapsize[0]/8, self.mapsize[1]/8)
			if tile==4:
				return (self.mapsize[0]/8, 3*self.mapsize[1]/8)
			if tile==5:
				return (3*self.mapsize[0]/8, 3*self.mapsize[1]/8)
			if tile==6:
				return (5*self.mapsize[0]/8, 3*self.mapsize[1]/8)
			if tile==7:
				return (7*self.mapsize[0]/8, 3*self.mapsize[1]/8)
			if tile==8:
				return (self.mapsize[0]/8, 5*self.mapsize[1]/8)
			if tile==9:
				return (3*self.mapsize[0]/8, 5*self.mapsize[1]/8)
			if tile==10:
				return (5*self.mapsize[0]/8, 5*self.mapsize[1]/8)
			if tile==11:
				return (7*self.mapsize[0]/8, 5*self.mapsize[1]/8)
			if tile==12:
				return (self.mapsize[0]/8, 7*self.mapsize[1]/8)
			if tile==13:
				return (3*self.mapsize[0]/8, 7*self.mapsize[1]/8)
			if tile==14:
				return (5*self.mapsize[0]/8, 7*self.mapsize[1]/8)
			else:
				return (7*self.mapsize[0]/8, 7*self.mapsize[1]/4)


	# Updates game data
	def update(self,points=None, mapsize=None, count=None, agent_time=None):
		if(self.last_points != self.points):
			self.calc_jps=True
			self.map_places = []
			self.positions = []
		if self.center is None:
			self.center = (mapsize[0]/2, mapsize[1]/2)
		self.mapsize=mapsize
		self.count=count
		self.agent_time = agent_time
		self.last_points=self.points

	def updateDirection(self,maze):
		start_time = pygame.time.get_ticks()

		# Detects loop and says to calculate JPS again
		if self.positions.count(self.body[0])>3:
			self.calc_jps = True

		# Snake changed tile. Calculate JPS and save new tile
		if len(self.map_places)==0 or self.map_places[-1]!=self.get_part_map(self.body[0])[0]:
			self.map_places.append(self.get_part_map(self.body[0])[0])
			self.calc_jps=True

		# Update snakes positions
		self.update_snakes(maze)

		# Everytime food changes tile, recalculate jps and clean last tiles
		if self.food_tile==None or self.food_tile!=self.get_part_map(maze.foodpos)[0]:
			self.food_tile = self.get_part_map(maze.foodpos)[0]
			self.map_places = []
			self.map_places.append(self.get_part_map(self.body[0])[0])
			self.calc_jps = True
			#self.positions = []

		# Append actual position of snake
		self.positions.append(self.body[0])

		# Tries to collide heads
		if self.collide_head(maze)!=None:
			self.snake2.last_body = self.snake2.body
			self.direction=self.collide_head(maze)
			return

		# Tries to trap other snake
		if self.trap_snake(maze) != None:
			self.snake2.last_body = self.snake2.body
			self.direction=self.trap_snake(maze)
			return

		# Delete actual position from jumps
		if len(self.jumps)>0 and self.body[0]==self.jumps[0][0].pos:
			self.jumps[0:1]=[]

		# Calculate JPS
		if self.calc_jps==True:
			self.jps = JPS(self, self.body[0], maze.foodpos, maze, self.agent_time, start_time)
			self.jumps = self.jps.run()[1:]
			self.last_food = maze.foodpos
			self.calc_jps = False if self.jumps!=[] else True

		# Go catch food
		if self.jumps==[]:
			self.direction = self.get_next_dir(maze.foodpos, maze)
			return

		self.direction=self.get_next_dir(self.jumps[0][0].pos, maze)

		self.snake2.last_body = self.snake2.body


		# Debugging
		#self.game.paint([e[0].pos for e in self.jumps], (128,50,50))
		#self.game.paint([self.jumps[0][0].pos], (0,50,50))
		

	# Get next best dir
	def get_next_dir(self, jump, maze):
		validdir = [(self.jps.get_dir(n,self.body[0]),self.jps.heuristic(n,jump)) for n in self.get_neighbours(maze, self.body[0], None, True)]
		dir = [0,0]
		runaway = False
		while validdir!=[]:
			next_pos = min(validdir, key=lambda x: x[1])
			# Get best next move of other snake
			# First, we need to get last position of head of other snake
			last_head = -1
			if(self.count>1):
				if(len(self.head_last_positions)==1):
					last_head = [pos for pos in self.snake2.last_body if pos != self.head_last_positions[0]][0]
				elif len(self.head_last_positions)>1:
					last_head = self.head_last_positions[-2][0]
				elif len(self.snake2.last_body)>0:
					last_head = self.snake2.last_body[0]

			best_head_next_moves = []
			if len(self.snake2.head)>0 and self.best_next_moves(maze, self.snake2.head[0], last_head)!=None:
				best_head_next_moves = self.best_next_moves(maze, self.snake2.head[0], last_head)
			
			# Next position calculated
			newpos = self.add(next_pos[0], self.body[0])
			

			# Check Chicken Condition of if we get trapped
			if (newpos not in best_head_next_moves or self.points>self.snake2.points) and self.check_exists_path(maze, newpos, next_pos[0], 0):
				return next_pos[0]
			
			runaway = True
			del validdir[validdir.index(next_pos)]
		
		# If did chicken condition or we get trapped, but there is no direction to go, ignore last check and go there
		if runaway==True:
			validdir = [(self.jps.get_dir(n,self.body[0]),self.jps.heuristic(n,jump)) for n in self.get_neighbours(maze, self.body[0], None, True)]
			return min(validdir, key=lambda x: x[1])[0]

		# If no valid dir, tries to chock with other head
		complement=[(up,down),(down,up),(right,left),(left,right)]

		invaliddir=[x for (x,y) in complement if y==self.direction]
		validdir=[dir for dir in directions if not ( dir in invaliddir )]

		next_m = [self.add(dir, self.body[0]) for dir in validdir if self.add(dir, self.body[0]) in self.snake2.body]
		# Better hit other snake than suicide
		if len(next_m)>0:
			dir = self.jps.get_dir(next_m[0], self.body[0])
		
		# Better suicide than hit against obstacle
		else:
			next_m = [self.add(dir, self.body[0]) for dir in validdir if self.add(dir, self.body[0]) in self.body]
			if len(next_m)>0:
				dir = self.jps.get_dir(next_m[0], self.body[0])

		return dir

	# Check if path has an exit
	def check_exists_path(self, maze, pos, direct, n=0):
		# Stop Condition
		if n>=max(self.mapsize[0], self.mapsize[1]):
			return True

		# Next possible directions
		complement=[(up,down),(down,up),(right,left),(left,right)]
		invaliddir=[x for (x,y) in complement if list(y)==list(direct)]
		possible_next_dirs=[dir for dir in directions if not ( dir in invaliddir )]

		other_snake_without_tail = [pos for pos in self.snake2.body if pos!=self.snake2.tail[0]]
		
		# Other snake can eat on next move. We must count with his tail being in the same place.
		if len(self.snake2.head)>0 and self.jps.heuristic(self.snake2.head[0],maze.foodpos)<=1.5:
			other_snake_without_tail.append(self.snake2.tail[0])

		pos = [(self.add(pos, direction) in maze.obstacles+self.body+other_snake_without_tail, self.add(pos, direction), direction) for direction in possible_next_dirs]
		
		if all([p[0] for p in pos]):
			return False
		if not any([p[0] for p in pos]):
			return True
		else:
			next_p = pos[0][1]
			next_dir = pos[0][2]
			for p in pos:
				if p[0]==False:
					return self.check_exists_path(maze, p[1], p[2], n+1)


	# Tries to collide heads of snake
	def collide_head(self, maze):

		if self.points<=self.snake2.points:
			return None

		# Get valid head moves
		valid_pos = self.get_neighbours(maze, self.body[0], None)

		# Get best next move of other snake
		# First, we need to get last position of head of other snake
		last_head = -1
		if(self.count>1):
			if(len(self.head_last_positions)==1):
				last_head = [pos for pos in self.snake2.last_body if pos != self.head_last_positions[0]][0]
			elif len(self.head_last_positions)>1:
				last_head = self.head_last_positions[-2][0]
			elif len(self.snake2.last_body)>0:
				last_head = self.snake2.last_body[0]
				
		best_head_next_moves = []
		if len(self.snake2.head)>0 and self.best_next_moves(maze, self.snake2.head[0], last_head)!=None:
			best_head_next_moves = self.best_next_moves(maze, self.snake2.head[0], last_head)
		else:
			return None
		pos = [ our_pos for our_pos in valid_pos for other_pos in best_head_next_moves if other_pos==our_pos ]

		if len(pos)==0:
			return None

		return self.jps.get_dir(pos[0], self.body[0])

	# Try to kill other snake by trapping him
	def trap_snake(self, maze):
		# Verify if the other snake head is next to any part of our body except head
		dirs = [up, down, left, right]

		if len(self.body)==1 or len(self.snake2.head)==0:
			return None

		around = [(pos, dir) for dir in dirs for pos in self.body[1:] if self.add(pos, dir)==self.snake2.head[0] ]

		if len(around)<1:
			return None

		# For that position, verify if is next to an obstacle
		if self.add(self.add(around[0][0],around[0][1]),around[0][1]) not in maze.obstacles:
			return None
		
		# Get valid head moves
		valid_pos = self.get_neighbours(maze, self.body[0], None)

		# Check if head can go in the direction of other snake and
		# if there is an obstacle on body[1] two times the direction
		if self.add(around[0][1],self.body[0]) in valid_pos and self.add(around[0][1], self.add(around[0][1], self.body[1])) in maze.obstacles:
			dir = around[0][1]
			# Get best next move of other snake
			# First, we need to get last position of head of other snake
			last_head = -1
			if(self.count>1):
				if(len(self.head_last_positions)==1):
					last_head = [pos for pos in self.snake2.last_body if pos != self.head_last_positions[0]][0]
				elif len(self.head_last_positions)>1:
					last_head = self.head_last_positions[-2][0]
				elif len(self.snake2.last_body)>0:
					last_head = self.snake2.last_body[0]
					
			best_head_next_moves = []
			if len(self.snake2.head)>0 and self.best_next_moves(maze, self.snake2.head[0], last_head)!=None:
				best_head_next_moves = self.best_next_moves(maze, self.snake2.head[0], last_head)

			if self.add(self.body[0], dir) not in best_head_next_moves or self.points>self.snake2.points:
				return dir
			else:
				return None

		else:
			return None

	# Tries to guess next best move of other snake
	# With greedy algorithm
	def best_next_moves(self, maze, position, direction):
		valid_dir = self.get_neighbours(maze, position, direction)
		heur = self.mapsize[0]+self.mapsize[1]
		best_dir=[]
		if len(valid_dir)==0:
			return []
			
		for dirs in valid_dir:
			if (self.jps.heuristic(dirs, maze.foodpos)<heur):
					best_dir = [dirs]
					heur = self.jps.heuristic(dirs, maze.foodpos)
			elif (self.jps.heuristic(dirs, maze.foodpos) == heur):
					best_dir.append(dirs)
		return best_dir


	# Update other snake position
	def update_snakes(self, maze):

		# Update other snakes position
		self.snake2.body = [head for head in maze.playerpos if head not in self.body]

		# If is beginning of the game, get snake's heads
		if self.count==1:
			self.snake2.head = self.snake2.body
			self.head_last_positions = []

		# Update other snake's head
		else:
			new_head = []
			head = self.snake2.head
			dirs = [up, down, left, right]

			# Find valid new head
			valid_new_head = [ position for position in self.snake2.body if position not in self.snake2.last_body ]
			

			if len(valid_new_head)==1:
				new_head.append(valid_new_head[0])

			# If there is no valid head, but length of snake is !=0,
			# Head is where tail were last time
			elif len(self.snake2.body):
				new_head.append(self.snake2.tail[0])
			self.snake2.head = new_head


		if(len(self.snake2.body)!=len(self.snake2.last_body) and self.count!=1 and len(self.snake2.body)!=0):
			self.snake2.points+=10
			self.map_places = []
			self.positions = []
			self.calc_jps = True

		# Update Snake tail

		# If length of other tail is one, tail is head
		if len(self.snake2.body)==1:
			self.snake2.tail = self.snake2.body

		# If other snake did not grown, gets the first position of head positions queue
		elif len(self.snake2.body) == len(self.snake2.last_body) and len(self.snake2.last_head_positions)!=0:
			self.snake2.tail = self.snake2.last_head_positions.pop(0)

		if len(self.snake2.body)>1:
			self.snake2.last_head_positions.append(self.snake2.head)

		# Get food position
		self.last_maze = maze

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
			olddir = self.jps.get_dir(position, last_pos)

		complement=[(up,down),(down,up),(right,left),(left,right)]

		invaliddir=[x for (x,y) in complement if list(y)==olddir]
		validdir=[dir for dir in directions if not ( dir in invaliddir )]

		if count_with_other_snake:
			# If not in same position, if not 360 degree turn, if not obstacle and if not snakes position without tails
			validdir=[self.add(dir,position) for dir in validdir if not (self.add(position,dir) in maze.obstacles+self.snake2.body+self.body)]
		else:
			validdir=[self.add(dir,position) for dir in validdir if not (self.add(position,dir) in maze.obstacles+self.body)]

		return validdir


# Jump Point Search algorithm
# As read in https://harablog.wordpress.com/2011/09/07/jump-point-search/
class JPS:
	def __init__(self, snake, start, goal, maze, agent_time, time, time_limit=True):
		self.snake = snake
		self.start = start
		self.goal = goal
		self.maze = maze
		self.all_list = {} # PosDir to best travel-cost
		self.queue = []    # Priority queue on traveled + estimate
		self.visited = []
		self.final_p = []
		self.time = time
		self.agent_time = agent_time
		self.max_time = 3.2
		self.time_limit = True


		for dx in [-1, 0, 1]:
			for dy in [-1, 0, 1]:
				if dx != 0 or dy != 0:
					self.add_node(self.start[0], self.start[1], (dx, dy), 0)


	def get_goal(self, pos):

		goal_tile, goal_adjacent = self.snake.get_part_map(self.goal)
		snake_tile, snake_adjacent = self.snake.get_part_map(self.snake.body[0])

		# If on same tile, search for snake.
		if goal_tile==snake_tile:
			if pos==self.goal:
				return True
			else:
				return False

		# If on other tile, search for adjacent tile with better heuristic to food.
		else:
			pos_tile, pos_adjacent = self.snake.get_part_map(pos)

			# If i am on an adjacent tile, only returns True when finds a path to the desired tile
			if snake_tile in goal_adjacent:
				return pos_tile==goal_tile

			# If i am not on an adjacent tile, go to tile with better heuristic to the goal
			else:
				heur = [ (tile_adj, self.heuristic(self.snake.get_center_tile(tile_adj),self.goal)) for tile_adj in snake_adjacent]
				best_tile, adj = min(heur, key=lambda x:x[1])

				if pos_tile==best_tile:
					return True
				else:
					return False


	# Manhattan Distance, as read here: http://theory.stanford.edu/~amitp/GameProgramming/Heuristics.html
	# Also read for breaking ties (here with 'p' variable)
	def heuristic(self, pos, goal):

		# Used for breaking ties
		# p <(minimum cost of taking one step)/(expected maximum path length)
		p=1/(self.snake.mapsize[0]+self.snake.mapsize[1]+1)

		best_x_move = min(abs(pos[0]-goal[0]), pos[0]+self.snake.mapsize[0]-goal[0], goal[0]+self.snake.mapsize[0]-pos[0])
		best_y_move = min(abs(pos[1]-goal[1]), pos[1]+self.snake.mapsize[1]-goal[1], goal[1]+self.snake.mapsize[1]-pos[1])

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
			heur = 0
			if dir is None:
				heur = self.heuristic(pd.pos, self.goal)
			else:
				heur = self.heuristic(self.snake.add(pd.pos, dir), self.goal)
			total = dist + heur
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
		
		if self.get_goal(elm.pos):
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
		i=0
		while True:

			if self.time_limit and self.agent_time-(pygame.time.get_ticks()-self.time)<self.max_time:
				return []

			#self.snake.game.paint([(x0,y0)], (0,50,50))
			i+=1
			x1 = self.snake.add([x0,y0],[hor_dir,0])[0]
			
			if i>=self.snake.mapsize[0]:
				return []

			if (x1, y0) in self.maze.obstacles+self.snake.body:
				return []

			if self.get_goal((x1, y0)):
				self.final_p.append((x1,y0))
				return [self.add_node(x1, y0, None, dist + 1)]


			# Condition to avoid exploring other parts of map
			if self.snake.get_part_map((x1,y0))!=self.snake.get_part_map(self.snake.body[0]):
				return []

			dist = dist + 1

			x2 = self.snake.add([x1,pos[1]],[hor_dir,0])[0]

			nodes = []
			if self.snake.add([x1,y0],[0,-1]) in self.maze.obstacles and not self.snake.add([x2,y0],[0,-1]) in self.maze.obstacles:
				nodes.append(self.add_node(x1, y0, (hor_dir, -1), dist))

			if self.snake.add([x1,y0],[0,1]) in self.maze.obstacles and not self.snake.add([x2,y0],[0,1]) in self.maze.obstacles:
				nodes.append(self.add_node(x1, y0, (hor_dir, 1), dist))

			if len(nodes) > 0:
				nodes.append(self.add_node(x1, y0, (hor_dir, 0), dist))
				return nodes

			x0 = x1

	def search_vert(self, pos, vert_dir, dist):
		x0, y0 = pos
		i=0
		while True:

			if self.time_limit and self.agent_time-(pygame.time.get_ticks()-self.time)<self.max_time:
				return []

			#self.snake.game.paint([(x0,y0)], (0,50,50))
			i+=1
			y1 = self.snake.add([x0,y0],[0,vert_dir])[1]
			
			if i>=self.snake.mapsize[1]:
				return []
			
			if (x0, y1) in self.maze.obstacles+self.snake.body:
				return []

			if self.get_goal((x0,y1)):
				self.final_p.append((x0,y1))
				return [self.add_node(x0, y1, None, dist + 1)]

			# Condition to avoid exploring other parts of map
			if self.snake.get_part_map((x0,y1))!=self.snake.get_part_map(self.snake.body[0]):
				return []

			dist = dist + 1

			y2 = self.snake.add([pos[0],y1],[0,vert_dir])[1]

			nodes = []
			if self.snake.add([x0,y1],[-1,0]) in self.maze.obstacles and not self.snake.add([x0,y2],[-1,0]) in self.maze.obstacles:
				nodes.append(self.add_node(x0, y1, (-1, vert_dir), dist))

			if self.snake.add([x0,y1],[1,0]) in self.maze.obstacles and not self.snake.add([x0,y2],[1,0]) in self.maze.obstacles:
				nodes.append(self.add_node(x0, y1, (1, vert_dir), dist))

			if len(nodes) > 0:
				nodes.append(self.add_node(x0, y1, (0, vert_dir), dist))
				return nodes

			y0 = y1

	def search_diagonal(self, pos, hor_dir, vert_dir, dist):

		x0, y0 = pos
		while True:

			if self.time_limit and self.agent_time-(pygame.time.get_ticks()-self.time)<self.max_time:
				return []

			#self.snake.game.paint([(x0,y0)], (0,50,50))
			x1, y1 = self.snake.add([x0, y0], [hor_dir, vert_dir])

			if self.snake.add([x1,y1],[hor_dir,vert_dir])[1]==pos[1] or self.snake.add([x1,y1],[hor_dir,vert_dir])[0]==pos[0]:
				return []

			if (x1, y1) in self.maze.obstacles+self.snake.body:
				return []

			if self.get_goal((x1, y1)):
				self.final_p.append((x1,y1))
				return [self.add_node(x1, y1, None, dist + 2)]

			# Condition to avoid exploring other parts of map
			if self.snake.get_part_map((x1,y1))!=self.snake.get_part_map(self.snake.body[0]):
				return []

			if (x0, y1) in self.maze.obstacles+self.snake.body and (x1, y0) in self.maze.obstacles+self.snake.body:
				return [] 

			# Open space at (x1, y1)
			dist = dist + 2

			x2, y2 = self.snake.add([x1, y1], [hor_dir, vert_dir])

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

	def run(self):
		while True:
			total, pd, dist = self.get_open()
			if total is None:
				break

			pd = self.step(dist, pd)

			if pd is not None:
				break
			if self.time_limit and self.agent_time-(pygame.time.get_ticks()-self.time)<self.max_time:
				return []


		while True:
			total, pd, dist = self.get_open()
			if total is None:
				break
			if self.time_limit and self.agent_time-(pygame.time.get_ticks()-self.time)<self.max_time:
				return []

		# Return Jumps sorted
		if self.final_p!=[]:
			next_hop = min(self.final_p, key=lambda x:self.heuristic(self.snake.body[0], self.goal))
			l = sorted(self.getJumps(next_hop), key=lambda x: x[1], reverse=True)
			return l+[(PosDir(next_hop,(0,0)), l[-1][1]+1)]
		else:
			return []

	# Get jumps of path searched
	def getJumps(self, pos, n=1):
		e1 = [elem.parent for elem in self.all_list if elem.pos==pos and elem.parent!=None]

		if len(e1)==0:
			return []

		elem2 = reduce(lambda r,h: h if self.all_list[h]<self.all_list[r] else r, e1)
		
		if elem2==None or elem2.pos==self.start:
			return [(elem2, n)]


		return [(elem2,n)]+list(set(self.getJumps(elem2.pos, n+1)))


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
