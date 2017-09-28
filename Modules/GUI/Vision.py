#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import numpy as np

class Vision_Functions():
	def __init__(self):
		self.set_functions_dict()

	def set_functions_dict(self):
		self.func_dict = {}
		self.func_dict['Escala de cinza'] = lambda frame: cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
		self.func_dict['Detectar azul'] = lambda frame: self.color_detect(frame, ['azul', 'ciano'])
		self.func_dict['Detectar verde'] = lambda frame: self.color_detect(frame, ['verde'])
		self.func_dict['Detectar vermelho'] = lambda frame: self.color_detect(frame, ['vermelho1', 'vermelho2'])
		self.func_dict['Detectar laranja'] = lambda frame: self.color_detect(frame, ['laranja'])
		self.func_dict['Detectar amarelo'] = lambda frame: self.color_detect(frame, ['amarelo'])
		self.func_dict['Detectar rosa'] = lambda frame: self.color_detect(frame, ['rosa'])
		self.func_dict['Detectar roxo'] = lambda frame: self.color_detect(frame, ['roxo'])


	def get_functions_nicks(self):
		return self.func_dict.keys()

	def run(self, img, nick):
		return self.func_dict[nick](img)

	def color_detect(self, im, color_list):
		hsv = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)
		colors = { 'vermelho1' : range(11),
					'laranja' : range(11,21),
					'amarelo' : range(21,46),
					'verde' : range(46,81),
					'ciano' : range(81,101),
					'azul' : range(101,121),
					'roxo' : range(121,146),
					'rosa' : range(146,170),
					'vermelho2' : range(170,180)}
		if len(color_list) == 1:
			color = color_list[0]
			lower_threshold = np.array([colors[color][0], 50, 50])
			upper_threshold = np.array([colors[color][-1], 255, 255])
			mask = cv2.inRange(hsv, lower_threshold, upper_threshold)
			return cv2.bitwise_and(im,im, mask= mask)
		else:
			im_list = []
			for color in color_list:
				lower_threshold = np.array([colors[color][0], 50, 50])
				upper_threshold = np.array([colors[color][-1], 255, 255])
				mask = cv2.inRange(hsv, lower_threshold, upper_threshold)
				im_i = cv2.bitwise_and(im,im, mask= mask)
				im_list.append(im_i)
			res = im_list[0]
			for im_i in im_list[1:]:
				res = cv2.add(res,im_i)
			return res

if __name__ == '__main__':
	pass