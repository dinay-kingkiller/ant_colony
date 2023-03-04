#!/usr/bin/env python

import rospy

from matplotlib.pyplot import plot
from numpy import reshape

def update(data):
    pheromones = reshape(data.pheromones, (VertexCount, VertexCount))
    for blah in blahblah:
        plot(x, y)

    
def plotter():
    rospy.init_node("plotter")
    rospy.Subscriber(SOMETOPIC, SOME_MSG, update)
    rospy.spin()

if __name__ == "__main__":
    plotter()
