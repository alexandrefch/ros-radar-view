#!/usr/bin/env python

# ==================================================================================================
# Copyright (C) 2023 University of South Brittany, Lab-STICC UMR 6285 All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# ==================================================================================================

"""
Draw a radar like map, it's simply a 'top-view' of point relative to object.
"""

# ==================================================================================================
#                                          I M P O R T S
# ==================================================================================================

import sys
import time
import rospy
import cv2 as cv
import numpy as np

from sensor_msgs.msg import CompressedImage
from radar_view.msg import RadarObjArray, RadarObj

# ==================================================================================================
#                                             C O D E
# ==================================================================================================

class RadarView():
    """
    Draw a radar like map, it's simply a 'top-view' of point relative to object.
    """

    MARGIN_PERCENT   = 0.2             # Percent of space between radar and border
    COLOR_BACKGROUND = (0,0,0)
    COLOR_GREEN      = (123, 207, 101)
    COLOR_DARK_GREEN = (92, 155, 75)
    COLOR_OBJ        = (255,0,0)

    def __init__(self):
        rospy.init_node("radar_view",anonymous=True)

        try:
            self.in_topic     = rospy.get_param("~input_topic")
            self.out_topic    = rospy.get_param("~output_topic")
            self.map_grad     = rospy.get_param("~map_graduation",10)
            self.map_res      = rospy.get_param("~map_resolution",1024)
            self.map_range    = rospy.get_param("~map_range",80)
            self.frequency    = rospy.get_param("~frequency",5)
            self.is_clockwise = rospy.get_param("~clockwise",False)
        except KeyError as e:
            rospy.logerr("[radar-view] Error on init, missing parameter named {} !".format(str(e)))
            sys.exit()

        self.map_ratio     = self.map_res / (2 * self.map_range) * (1-RadarView.MARGIN_PERCENT)
        self.margin        = int(RadarView.MARGIN_PERCENT/2*self.map_res)
        self.map_h_res     = int(self.map_res/2) # Half resolution
        self.map_publisher = rospy.Publisher(self.out_topic, CompressedImage, queue_size=3)
        self.buffer        = []

        self._generate_background()

        rospy.Subscriber(self.in_topic, RadarObjArray, self._on_receive_point_array)

        self._main_loop()

    def _main_loop(self):
        """
        Main loop allow to configure specific node speed look at param `frequency`
        """
        sleep_time = 1/float(self.frequency)
        while not rospy.is_shutdown():
            t_start = time.time()

            self._update_ui()

            t_duration = time.time() - t_start
            t_sleep    = sleep_time - t_duration
            if t_sleep > 0:
                rospy.sleep(t_sleep)

    def _generate_background(self):
        """
        Generate the background image made of the radar by drawing cicle ang
        axis each 22.5 degres, and finally write mtching degres arround the UI
        """
        # Create empty openCv image
        self.background = np.full(
            (self.map_res, self.map_res, 3),
            (255, 255, 255),
            dtype=np.uint8
        )

        cv.circle(
            self.background,
            (self.map_h_res, self.map_h_res),
            int(self.map_range*(self.map_ratio+0.1)),
            RadarView.COLOR_BACKGROUND,
            -1
        )
        # Draw circle every 10 meter
        radius = self.map_grad
        while radius < self.map_range:
            cv.circle(
                self.background,
                (self.map_h_res, self.map_h_res),
                int(radius*self.map_ratio),
                (90, 143, 76),
                max(1,int(self.map_res/256))
            )
            radius += self.map_grad

        cv.circle(
                self.background,
                (self.map_h_res, self.map_h_res),
                int(self.map_range*self.map_ratio),
                (90, 143, 76),
                max(1,int(self.map_res/256))
            )

        angle = 0.0
        while angle < 360:
            x  = int(np.cos(np.deg2rad((360-angle-90)%360)) * self.map_h_res * (1-RadarView.MARGIN_PERCENT))
            y  = int(np.sin(np.deg2rad((360-angle-90)%360)) * self.map_h_res * (1-RadarView.MARGIN_PERCENT))
            xc = int(np.cos(np.deg2rad((360-angle-90)%360)) * self.map_grad * self.map_ratio )
            yc = int(np.sin(np.deg2rad((360-angle-90)%360)) * self.map_grad * self.map_ratio )

            cv.line(
                self.background,
                (x+self.map_h_res,y+self.map_h_res),
                (xc+self.map_h_res,yc+self.map_h_res),
                RadarView.COLOR_GREEN,
                2
            )

            if angle==int(angle):
                text_size = cv.getTextSize(str(int(angle)),cv.FONT_HERSHEY_SIMPLEX,1,2)[0]

                x     = int(np.cos(np.deg2rad((angle-90)%360)) * self.map_h_res * (1-RadarView.MARGIN_PERCENT+0.1))
                y     = int(np.sin(np.deg2rad((angle-90)%360)) * self.map_h_res * (1-RadarView.MARGIN_PERCENT+0.1))
                x    -= int(text_size[0]/2)
                y    += int(text_size[1]/2)
                label = str(int(angle if self.is_clockwise else (360-angle)%360))
                cv.putText(
                    self.background,
                    label,
                    (x+self.map_h_res,y+self.map_h_res),
                    cv.FONT_HERSHEY_DUPLEX,
                    1,
                    RadarView.COLOR_DARK_GREEN,
                    2
                )
            angle += 22.5

    def _on_receive_point_array(self, array):
        """
        Store in a buffer the objects array until the update
        """
        self.buffer = array.objects

    def _update_ui(self):
        """
        Make a copy of the background and draw all the object with their corresponding
        color and label then publish it on the output_topic
        """
        size = self.map_res
        half_size = int(size/2)

        # Important, made a copy of the backgound to keep it clean
        img = np.copy(self.background)

        for obj in self.buffer:
            dist = np.sqrt(obj.x**2+obj.y**2)
            if dist < self.map_range:
                x = int(obj.x * self.map_ratio + half_size)
                y = int(obj.y * -1 * self.map_ratio + half_size)
                r,g,b = obj.color.r, obj.color.g, obj.color.b
                if r == g == b == 0:
                    r,g,b = RadarView.COLOR_OBJ

                cv.circle(img,(x,y),15,(b,g,r),-1)

                label = obj.label
                if label != '':
                    text_size = cv.getTextSize(label,cv.FONT_HERSHEY_DUPLEX,1.3,2)[0]
                    cv.rectangle(img,(x,y-23),(x+text_size[0],y-23-text_size[1]),(0,0,0),-1)
                    cv.putText(img,label,(x,y-23),cv.FONT_HERSHEY_DUPLEX,1.3,(b,g,r),2)

        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        msg.data = np.array(cv.imencode('.jpg', img)[1]).tobytes()

        self.map_publisher.publish(msg)

if __name__ == "__main__":
    RadarView()
