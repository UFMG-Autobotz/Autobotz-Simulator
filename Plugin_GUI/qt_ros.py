#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Float32

import sys
import numpy as np

from PyQt4.QtCore import *
from PyQt4.QtGui import *
from general_utils import get_yaml_dict

class Controller():
	def __init__(self, topics_file, node_name = "joint_plugin_controller"):
		self.specs = get_yaml_dict(topics_file)
		self.init_topics(node_name)
		self.create_window()

	def init_topics(self, node_name):
		rospy.init_node(node_name, anonymous=True)
		self.topics = []
		self.pubs = []
		self.keep_publish = []
		for key in sorted(self.specs.keys()):
			self.topics.append(self.specs[key])
			self.pubs.append([])
			for topic_group in self.topics[-1]['topicos']:
				self.pubs[-1].append([])
				for topic_name in topic_group:
					self.pubs[-1][-1].append(rospy.Publisher(topic_name, Float32, queue_size=1))

	def Line(self, orient, tipo): # HLine, VLine / Plain, Raised, Sunken 
		linha = QFrame()
		linha.setFrameShape( [QFrame.HLine, QFrame.VLine][orient] )
		linha.setFrameShadow( [QFrame.Plain, QFrame.Raised, QFrame.Sunken][tipo] ) 
		return linha

	def create_window(self):
		self.app = QApplication(sys.argv)
		self.win = QWidget()
		self.bh = QHBoxLayout()
		self.current_values = np.zeros((len(self.topics), 1))
		self.results = []
		self.sliders = []
		self.pub_buttons = []
		self.radio_buttons = []
		self.radio_buttons_groups = []

		for k, topic in enumerate(self.topics):
			topic_vbox = QVBoxLayout()

			label = QLabel(topic['nome'])
			label.setAlignment(Qt.AlignCenter)

			n_sliders = len(topic['sliders'])
			diff = n_sliders - self.current_values.shape[1]
			if diff > 0:
				self.current_values = np.column_stack( ( self.current_values, np.zeros( ( len(self.topics), diff ) ) ) )

			self.results.append([])
			self.sliders.append([])
			self.pub_buttons.append([])
			self.radio_buttons.append([])
			self.radio_buttons_groups.append([])
			self.keep_publish.append([])

			sliders_hbox = QHBoxLayout()
			for i in xrange(n_sliders):
				value_vbox = QVBoxLayout()

				slider_label = QLabel(topic['sliders'][i])
				slider_label.setAlignment(Qt.AlignCenter)

				self.results[-1].append(QLineEdit())
				self.results[-1][-1].setText(str(0))
				self.results[-1][-1].setReadOnly(True)

				self.sliders[-1].append(QSlider(Qt.Vertical))
				_min = float(topic['range'][i][0])
				_max = float(topic['range'][i][1])
				ticks = float(topic['ticks'])
				slider_info = [_min, _max, ticks]

				if topic['tipo'][i] == 'int' and ticks > _max-_min:
					self.sliders[-1][-1].setMaximum(_max-_min)
				else:
					self.sliders[-1][-1].setMaximum(ticks)

				self.sliders[-1][-1].setMinimum(0)
				self.sliders[-1][-1].setValue(0)
				self.sliders[-1][-1].setTickPosition(QSlider.TicksBelow)
				self.sliders[-1][-1].setTickInterval(1)
				self.sliders[-1][-1].valueChanged.connect( lambda _, topic=k, slider=i, tipo=topic['tipo'][i], info=slider_info: self.change_value(topic, slider, tipo, info) )

				self.pub_buttons[-1].append(QPushButton('Publish'))
				self.pub_buttons[-1][-1].clicked.connect( lambda _, topic=k, slider=i: self.publish(topic, slider) )

				self.keep_publish[-1].append(False)

				radio_hbox = QHBoxLayout()
				self.radio_buttons_groups[-1].append(QButtonGroup())

				self.radio_buttons[-1].append([QRadioButton("Once"), QRadioButton("Stream")])
				self.radio_buttons[-1][-1][0].setChecked(True)
				
				self.radio_buttons_groups[-1][-1].addButton(self.radio_buttons[-1][-1][0])
				self.radio_buttons_groups[-1][-1].addButton(self.radio_buttons[-1][-1][1])
				radio_hbox.addWidget(self.radio_buttons[-1][-1][0])
				radio_hbox.addWidget(self.radio_buttons[-1][-1][1])
				 # isChecked()

				value_vbox.addWidget(slider_label)
				value_vbox.addWidget(self.results[-1][-1])
				value_vbox.addWidget(self.pub_buttons[-1][-1])
				value_vbox.addLayout(radio_hbox)
				value_vbox.addWidget(self.sliders[-1][-1])
				sliders_hbox.addLayout(value_vbox)
			topic_vbox.addWidget(label)
			topic_vbox.addLayout(sliders_hbox)
			self.bh.addLayout(topic_vbox)
			if k < len(self.topics):
				self.bh.addWidget(self.Line(1,2))


		# self.text = QLineEdit()
		# self.text.editingFinished.connect(self.enterPress)

		self.win.setLayout(self.bh)
		self.win.setWindowTitle("Template Controller")

	# def enterPress(self, topic, slider):
	# 	print 'enter', topic, slider
	# 	pub_str = str('enter '+self.text.text())
	# 	self.pub.publish(pub_str)

	def change_value(self, topic, slider, tipo, info):
		_min, _max, ticks = info
		slider_value = float(self.sliders[ topic ][ slider ].value())

		if tipo == 'int':
			if ticks > _max-_min:
				value = int(_min+slider_value)
			else:
				value = int(_min + (_max-_min)*slider_value/ticks)
		else:
			value = round(_min + (_max-_min)*slider_value/ticks, 3)

		self.current_values[ topic, slider ] = value
		self.results[ topic ][ slider ].setText(str(value))

		if self.radio_buttons[topic][slider][1].isChecked() and self.keep_publish[topic][slider] == True:
			self.publish(topic, slider)
		else:
			self.keep_publish[topic][slider] = False

	def publish(self, topic, slider):
		# print 'publish', topic, slider
		if self.radio_buttons[topic][slider][1].isChecked():
			self.keep_publish[topic][slider] = True
		for pub in self.pubs[ topic ][ slider ]:
			pub.publish( self.current_values[ topic, slider ] )

	def run(self):
		self.win.show()
		sys.exit(self.app.exec_())

if __name__ == '__main__':
	try:
		topics_file = 'topics_teste.yaml'
		c = Controller(topics_file)
		c.run()

	except rospy.ROSInterruptException:
		pass