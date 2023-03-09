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

import rospy

from matplotlib import pyplot
from matplotlib.animation import FuncAnimation

from ant_colony.msg import PheromoneMap

class Vizualizer:
    def __init__(self, size_x, size_y):
        # self.figure = pyplot.figure()
        # self.axes = pyplot(xlim = (0, size_x), ylim = (0, size_y)
        self.figure, self.ax = pyplot.subplots()
        self.ax.set_xlim(0, size_x)
        self.ax.set_ylim(0, size_y)
        
    def animate(self):
        for path in self.paths:
            
    def listen(self, msg):
        for from_p, to_p, value in zip(zip(data.from_x, data.from_y),

                                       zip(data.to_x, data.to_y),

                                       data.values):

            pyplot.plot(from_p, to_p, linewidth=value)

            pyplot.show()

if __name__ == "__main__":
    # Setup ROS communication
    rospy.init_node("vizualizer")
    rospy.Subscriber("pheromone_map", PheromoneMap, listen)

    # Setup vizualizer
    size_x = rospy.get_param('size_x')
    size_y = rospy.get_param('size_y')
    vizualizer = Vizualizer(size_x, size_y)

    # Animate vizualizer
    animation = FuncAnimation(vizualizer.figure, vizualizer.animate)
    pyplot.show()
