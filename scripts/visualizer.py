 #!/usr/bin/env python

# BSD 3-Clause License
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 
# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
# 
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
# 
# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

##
# \brief
#

from math import factorial
import rospy

from matplotlib import pyplot
from matplotlib.animation import FuncAnimation
from matplotlib.collections import LineCollection

from ant_colony.msg import PheromoneMap

class Visualizer:
    def __init__(self, size_x, size_y):
        self.edge_s = set()
        self.edge_l = list()
        self.lines = LineCollection(self.edge_l)
        self.index = dict()
        self.strength = []
        self.figure, self.ax = pyplot.subplots()
        self.ax.set_xlim(0, size_x)
        self.ax.set_ylim(0, size_y)
        self.ax.add_collection(self.lines)
    def init_func(self):
        self.lines.set_segments([])
        self.lines.set_linewidths([])
        return self.lines,
    def animate(self, frame):
        rospy.loginfo("Animate Call")
        return self.lines,
    def listen(self, msg):
        rospy.loginfo("Listen Call")
        for x0, x1, y0, y1, value in zip(msg.from_x, msg.to_x, msg.from_y, msg.to_y, msg.strength):
            if (x0, y0) == (x1, y1):
                continue
            if (x0, y0, x1, y1) not in self.edge_s:
                self.edge_s.add((x0, y0, x1, y1))
                self.edge_l.append([(x0, y0), (x1, y1)])
                self.strength.append(value)
                self.index[(x0, y0, x1, y1)] = len(self.edge_s) - 1
            self.strength[self.index[(x0, y0, x1, y1)]] = value
        self.lines.set_segments(self.edge_l)
        self.lines.set_linewidths(self.strength)
        rospy.loginfo(len(self.strength))
            
if __name__ == "__main__":
    rospy.init_node("visualizer")
    
    # Setup visualizer
    size_x = rospy.get_param("size_x")
    size_y = rospy.get_param("size_y")
    VertexCount = rospy.get_param("VertexCount")
    visualizer = Visualizer(size_x, size_y)

    # Animate visualizer
    rospy.Subscriber("map_pheromones", PheromoneMap, visualizer.listen)
    animation = FuncAnimation(visualizer.figure,
                              visualizer.animate,
                              init_func=visualizer.init_func)
    pyplot.show(block=True)
