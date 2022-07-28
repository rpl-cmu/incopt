#! /usr/bin/env python3
# coding: utf-8

# generator-1.py, a simple python dungeon generator by
# James Spencer <jamessp [at] gmail.com>.
# http://www.roguebasin.com/index.php?title=A_Simple_Dungeon_Generator_for_Python_2_or_3

# To the extent possible under law, the person who associated CC0 with
# pathfinder.py has waived all copyright and related or neighboring rights
# to pathfinder.py.

# You should have received a copy of the CC0 legalcode along with this
# work. If not, see <http://creativecommons.org/publicdomain/zero/1.0/>.

# The room list is in [x1, y1, width, height] format
# The corridor list is in [(x1, y1), (x2, y2), ?(x3, y3)] format where (x3, y3) only exist if a hallway has an 'L' bend. 

# Run and then move "rooms.txt" to the swarmulator environment

from __future__ import print_function
import random
import numpy as np
from operator import itemgetter
import itertools
import matplotlib.pyplot as plt
import cv2
import argparse

import os
BASE_PATH = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../.."))

parser = argparse.ArgumentParser(description='Make random environments')
parser.add_argument('num_envs', type=int, help="(int) Num envs to be generated", default=100)
args = parser.parse_args()

CHARACTER_TILES = {'stone': 2,'floor': 1,'wall': 0}

# Python3 program to Find groups  
# of strictly increasing numbers within  
from itertools import groupby, cycle 
  
def groupSequence(l): 
    temp_list = cycle(l) 
  
    next(temp_list) 
    groups = groupby(l, key = lambda j: j + 1 == next(temp_list)) 
    for k, v in groups: 
        if k: 
            yield tuple(v) + (next((next(groups)[1])), ) 
	

class Generator():
	def __init__(self, width=64, height=64, max_rooms=15, min_room_xy=2,
					max_room_xy=15, rooms_overlap=False, random_connections=1,
					random_spurs=3, tiles=CHARACTER_TILES, imsize = 100):

		self.width = width
		self.height = height
		self.max_rooms = max_rooms
		self.min_room_xy = min_room_xy
		self.max_room_xy = max_room_xy
		self.rooms_overlap = rooms_overlap
		self.random_connections = random_connections
		self.random_spurs = random_spurs
		self.tiles = CHARACTER_TILES
		self.level = []
		self.room_list = []
		self.corridor_list = []
		self.tiles_level = []
		self.wall_coordinates = []
		self.im_size = imsize
	def gen_room(self):
		x, y, w, h = 0, 0, 0, 0
		w = random.randint(self.min_room_xy, self.max_room_xy)
		h = random.randint(self.min_room_xy, self.max_room_xy)
		x = random.randint(1, (self.width - w - 1))
		y = random.randint(1, (self.height - h - 1))
		return [x, y, w, h]

	def room_overlapping(self, room, room_list):
		x = room[0]
		y = room[1]
		w = room[2]
		h = room[3]
	
		for current_room in room_list:
	
			# The rectangles don't overlap if
			# one rectangle's minimum in some dimension
			# is greater than the other's maximum in
			# that dimension.
	
			if (x < (current_room[0] + current_room[2]) and
				current_room[0] < (x + w) and
				y < (current_room[1] + current_room[3]) and
				current_room[1] < (y + h)):
	
				return True

		return False
	
	
	def corridor_between_points(self, x1, y1, x2, y2, join_type='either'):
		if x1 == x2 and y1 == y2 or x1 == x2 or y1 == y2:
			return [(x1, y1), (x2, y2)]
		else:
			# 2 Corridors
			# NOTE: Never randomly choose a join that will go out of bounds
			# when the walls are added.
			join = None
			if join_type == 'either' and set([0, 1]).intersection(
					set([x1, x2, y1, y2])):
	
				join = 'bottom'
			elif join_type == 'either' and set([self.width - 1,
					self.width - 2]).intersection(set([x1, x2])) or set(
					[self.height - 1, self.height - 2]).intersection(
					set([y1, y2])):
	
				join = 'top'
			elif join_type == 'either':
				join = random.choice(['top', 'bottom'])
			else:
				join = join_type
	
			if join == 'top':
				return [(x1, y1), (x1, y2), (x2, y2)]
			elif join =='bottom':
				return [(x1, y1), (x2, y1), (x2, y2)]
	
	def join_rooms(self, room_1, room_2, join_type='either'):
		# sort by the value of x
		sorted_room = [room_1, room_2]
		sorted_room.sort(key=lambda x_y: x_y[0])
	
		x1 = sorted_room[0][0]
		y1 = sorted_room[0][1]
		w1 = sorted_room[0][2]
		h1 = sorted_room[0][3]
		x1_2 = x1 + w1 - 1
		y1_2 = y1 + h1 - 1
	
		x2 = sorted_room[1][0]
		y2 = sorted_room[1][1]
		w2 = sorted_room[1][2]
		h2 = sorted_room[1][3]
		x2_2 = x2 + w2 - 1
		y2_2 = y2 + h2 - 1
	
		# overlapping on x
		if x1 < (x2 + w2) and x2 < (x1 + w1):
			jx1 = random.randint(x2, x1_2)
			jx2 = jx1
			tmp_y = [y1, y2, y1_2, y2_2]
			tmp_y.sort()
			jy1 = tmp_y[1] + 5
			jy2 = tmp_y[2] - 5
	
			corridors = self.corridor_between_points(jx1, jy1, jx2, jy2)
			self.corridor_list.append(corridors)
	
		# overlapping on y
		elif y1 < (y2 + h2) and y2 < (y1 + h1):
			if y2 > y1:
				jy1 = random.randint(y2, y1_2)
				jy2 = jy1
			else:
				jy1 = random.randint(y1, y2_2)
				jy2 = jy1
			tmp_x = [x1, x2, x1_2, x2_2]
			tmp_x.sort()
			jx1 = tmp_x[1] + 1
			jx2 = tmp_x[2] - 1
	
			corridors = self.corridor_between_points(jx1, jy1, jx2, jy2)
			self.corridor_list.append(corridors)
	
		# no overlap
		else:
			join = None
			if join_type == 'either':
				join = random.choice(['top', 'bottom'])
			else:
				join = join_type
	
			if join == 'top':
				if y2 > y1:
					jx1 = x1_2 + 1
					jy1 = random.randint(y1, y1_2)
					jx2 = random.randint(x2, x2_2)
					jy2 = y2 - 1
					corridors = self.corridor_between_points(
						jx1, jy1, jx2, jy2, 'bottom')
					self.corridor_list.append(corridors)
				else:
					jx1 = random.randint(x1, x1_2)
					jy1 = y1 - 1
					jx2 = x2 - 1
					jy2 = random.randint(y2, y2_2)
					corridors = self.corridor_between_points(
						jx1, jy1, jx2, jy2, 'top')
					self.corridor_list.append(corridors)
	
			elif join == 'bottom':
				if y2 > y1:
					jx1 = random.randint(x1, x1_2)
					jy1 = y1_2 + 1
					jx2 = x2 - 1
					jy2 = random.randint(y2, y2_2)
					corridors = self.corridor_between_points(
						jx1, jy1, jx2, jy2, 'top')
					self.corridor_list.append(corridors)
				else:
					jx1 = x1_2 + 1
					jy1 = random.randint(y1, y1_2)
					jx2 = random.randint(x2, x2_2)
					jy2 = y2_2 + 1
					corridors = self.corridor_between_points(
						jx1, jy1, jx2, jy2, 'bottom')
					self.corridor_list.append(corridors)
	
	
	def gen_level(self):
	
		# build an empty dungeon, blank the room and corridor lists
		for i in range(self.height):
			self.level.append(['stone'] * self.width)
		self.room_list = []
		self.corridor_list = []
	
		max_iters = self.max_rooms * 5
	
		for a in range(max_iters):
			tmp_room = self.gen_room()
	
			if self.rooms_overlap or not self.room_list:
				self.room_list.append(tmp_room)
			else:
				tmp_room = self.gen_room()
				tmp_room_list = self.room_list[:]
	
				if self.room_overlapping(tmp_room, tmp_room_list) == False:
					self.room_list.append(tmp_room)
	
			if len(self.room_list) >= self.max_rooms:
				break
	
		# connect the rooms
		for a in range(len(self.room_list) - 1):
			self.join_rooms(self.room_list[a], self.room_list[a + 1])
	
		# do the random joins
		for a in range(self.random_connections):
			room_1 = self.room_list[random.randint(0, len(self.room_list) - 1)]
			room_2 = self.room_list[random.randint(0, len(self.room_list) - 1)]
			self.join_rooms(room_1, room_2)
	
		# do the spurs
		for a in range(self.random_spurs):
			room_1 = [random.randint(2, self.width - 2), random.randint(
						2, self.height - 2), 1, 1]
			room_2 = self.room_list[random.randint(0, len(self.room_list) - 1)]
			self.join_rooms(room_1, room_2)
	
		# fill the map
		# paint rooms
		for room_num, room in enumerate(self.room_list):
			for b in range(room[2]):
				for c in range(room[3]):
					self.level[room[1] + c][room[0] + b] = 'floor'
	
		# paint corridors
		for corridor in self.corridor_list:
			x1, y1 = corridor[0]
			x2, y2 = corridor[1]
			for width in range(abs(x1 - x2) + 1):
				for height in range(abs(y1 - y2) + 1):
					self.level[min((min(y1, y2) + height),np.shape(self.level)[0]-1)][
						min((min(x1, x2) + width),np.shape(self.level)[1]-1)] = 'floor'
	
			if len(corridor) == 3:
				x3, y3 = corridor[2]
	
				for width in range(abs(x2 - x3) + 1):
					for height in range(abs(y2 - y3) + 1):
						self.level[min(y2, y3) + height][
							min(x2, x3) + width] = 'floor'
	
		# paint the walls
		for row in range(1, self.height - 1):
			for col in range(1, self.width - 1):
				if self.level[row][col] == 'floor':
					if self.level[row - 1][col - 1] == 'stone':
						self.level[row - 1][col - 1] = 'wall'
	
					if self.level[row - 1][col] == 'stone':
						self.level[row - 1][col] = 'wall'
	
					if self.level[row - 1][col + 1] == 'stone':
						self.level[row - 1][col + 1] = 'wall'
	
					if self.level[row][col - 1] == 'stone':
						self.level[row][col - 1] = 'wall'
	
					if self.level[row][col + 1] == 'stone':
						self.level[row][col + 1] = 'wall'
	
					if self.level[row + 1][col - 1] == 'stone':
						self.level[row + 1][col - 1] = 'wall'
	
					if self.level[row + 1][col] == 'stone':
						self.level[row + 1][col] = 'wall'
	
					if self.level[row + 1][col + 1] == 'stone':
						self.level[row + 1][col + 1] = 'wall'
	
	def gen_tiles_level(self):	
		# Draw
		for row_num, row in enumerate(self.level):
			tmp_tiles = []
	
			for col_num, col in enumerate(row):
				if col == 'stone':
					tmp_tiles.append(0)
				if col == 'floor':
					tmp_tiles.append(0)
				if col == 'wall':
					tmp_tiles.append(1)
			self.tiles_level.append(tmp_tiles)
	
		# print('Room List: ', self.room_list)
		# print('\nCorridor List: ', self.corridor_list)
	
		# [print(row) for row in self.tiles_level]
		# print(self.tiles_level)
		self.tiles_level = np.asarray(self.tiles_level)
		plt.imsave("test_img.jpg",self.tiles_level)
		c = 2
		w_h = []
		# Horizontal walls
		for idx,row in enumerate(self.tiles_level):
			# print("***")
			data = np.where(row == 1)[0]
			if data.size > 0:
					a = list(groupSequence(list(data)))
					for item in a:
						# if len(item) > 2:
						w_h.append([min(item)*c,-idx*c,max(item)*c,-idx*c])
		# print(self.wall_coordinates)
		w_h = np.asarray(w_h)
		# Vertical walls
		w_v = []
		for idx,row in enumerate(self.tiles_level.T):
			# print("***")
			data = np.where(row == 1)[0]
			if data.size > 0:
					# print(data)
					a = list(groupSequence(list(data)))
					for item in a:
						# if len(item) >= 2:
						w_v.append([idx*c,-min(item)*c,idx*c,-max(item)*c])
		# print(self.wall_coordinates)
		w_v = np.asarray(w_v)
		w = np.vstack((w_h,w_v))
		w[:,[0,2]] = w[:,[0,2]] - w[:,[0,2]].mean() # Center horizontally
		w[:,[1,3]] = w[:,[1,3]] - w[:,[1,3]].mean() # Center vertically
		

		# Center vertically
		self.wall_coordinates = w.tolist()
		
		# w_h[:,[0,2]] = w_h[:,[0,2]] - w_h[:,[0,2]].mean()
		# w_v[:,[0,2]] = w_v[:,[0,2]] - w_v[:,[0,2]].mean()

		# w_h[:,[0,2]] = w_h[:,[0,2]] - w_h[:,[0,2]].mean()
		# w_h[:,[1,3]] = w_h[:,[1,3]] - w_h[:,[1,3]].mean()
		# w_h[:,[1,3]] = w_h[:,[1,3]] - w_v[:,[0,2]].mean()

		# w_v[:,[0,2]] = w_v[:,[0,2]] - w_v[:,[0,2]].mean()
		# w_v[:,[1,3]] = w_v[:,[1,3]] - w_v[:,[1,3]].mean()

		# print(self.wall_coordinates)
		# # Extrct vertical walls
		# print("*****************************")
		# for row in self.tiles_level.T:
		# 	print("***")
		# 	data = np.where(row == 1)[0]
		# 	if data.size > 0:
		# 		print(data)
		# 		a = groupSequence(list(data))
		# 		# print(list(np.min(list(a)),np.max(list(a))))

		# 	# if data[0].size > 0:
		# 	# 	for k, g in itertools.groupby(enumerate(data), lambda i,x:i-x):
		# 	# 		print(map(itemgetter(1), g))

	def world2opencv_coords(self,x,y):
		return(int(np.floor((x-self.x_min)*(self.im_size/(self.x_dim)))),int(np.floor((self.y_max-y)*(self.im_size/self.y_dim))))    

	def create_image(self):
		self.env_min, self.env_max = np.min(self.wall_coordinates), np.max(self.wall_coordinates)
		self.x_coords, self.y_coords = np.concatenate([line[0:-1:2] for line in self.wall_coordinates]).ravel().tolist(), np.concatenate([line[1:-1:2] for line in self.wall_coordinates]).ravel().tolist()
		self.x_min, self.x_max, self.y_min, self.y_max = np.min(self.x_coords), np.max(self.x_coords), np.min(self.y_coords), np.max(self.y_coords)
		self.x_dim, self.y_dim = self.x_max-self.x_min, self.y_max-self.y_min	


	def draw_line(self,row):
		num_segments = int(len(row)/2-1)
		for i in range(num_segments):
			start_point = np.clip(self.world2opencv_coords(row[i*2],row[i*2+1]),0,self.im_size-1)
			end_point = np.clip(self.world2opencv_coords(row[i*2+2],row[i*2+3]),0,self.im_size-1)
			self.env_top_view = cv2.line(self.env_top_view, tuple(start_point), tuple(end_point),(255,255,255),1)

	def add_border(self):
		self.draw_line([self.x_min,self.y_min,self.x_min,self.y_max])
		self.draw_line([self.x_min,self.y_max,self.x_max,self.y_max])
		self.draw_line([self.x_max,self.y_max,self.x_max,self.y_min])
		self.draw_line([self.x_max,self.y_min,self.x_min,self.y_min])

	def draw_env(self,dir):
		self.create_image()
		self.env_top_view = np.zeros((self.im_size,self.im_size,3),np.uint8)
		self.add_border()

		for line in self.wall_coordinates:
			self.draw_line(line)
		self.env_top_view = cv2.resize(self.env_top_view,(self.im_size,self.im_size),interpolation = cv2.INTER_AREA) # This is to make sure a wall is small compared to the total arena
		cv2.imwrite(dir,self.env_top_view)

if __name__ == '__main__':
    num_tries = 10
    
    for i in range(args.num_envs):
        tried = 0
        done = False
        while tried < num_tries and done == False:
            try: 
                #gen = Generator(max_rooms=0, width=64, height=64, min_room_xy=5,
               # 				max_room_xy=10, rooms_overlap=False, imsize=100)
                
                gen = Generator()
                gen.gen_level()
                gen.gen_tiles_level()
                img_dir = "{}/python/figures/navigation_2D/mazes/rand_env_{}.png".format(BASE_PATH, str(i))

                gen.draw_env(img_dir)
                done = True
            except:
                tried += 1
