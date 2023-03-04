#!/usr/bin/env python

import rospy

from matplotlib.pyplot import plot
from numpy import reshape

from ant_colony.msg import SOME_MSG

def update(data):
    for p1, p2, val in zip(zip(data.x1, data.y1), zip(data.x2, data.y2), data.value):
        plot(x, y)

    
def plotter():
    rospy.init_node("plotter")
    rospy.Subscriber("plot_data", PlotPheromones, update)
    rospy.spin()

if __name__ == "__main__":
    plotter()
