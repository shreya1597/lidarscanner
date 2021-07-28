#! /usr/bin/env python

import rospy
from math import degrees,cos,sin,sqrt
import numpy
from sensor_msgs.msg import LaserScan
import pdb
import pandas as pd

def callback(msg):
    min_dist = min(msg.ranges)
    index = msg.ranges.index(min_dist)
    ranges = msg.ranges
    full_ranges = ranges + ranges[0:index-1]
    print(len(ranges))
    print(len(full_ranges))

    intensities = msg.intensities
    full_intensities = intensities + intensities[0:index-1]
    print(len(intensities))
    print(len(full_intensities))

    print ('______________________________________')
     
    print(index)
    for i in range(0,360):
        if msg.ranges[i] == min_dist:
           break
    #print('index_of_msg_range %s'%i)
    #print ("Angle_of_closest_object %s" %(i*msg.angle_increment*(180/3.14)))
    angle1 = degrees(i*msg.angle_increment)
    #print ('Angle_clockwise %f' %(angle_clockwise))
    #----------------------------print ("Angle_anticlockwise %f" %degrees(i*msg.angle_increment))
    print"Dist_of_closest_object: %f        " %(min_dist*100), "Angle_of_closest_object: %f         " %degrees(i*msg.angle_increment)
    #print degrees(i*msg.angle_increment)
    #print ('_________________________________________________________________')
    
    #mini_value_intensities = min(msg.intensities)
    #for i in range(0,360):
    #   if msg.intensities == mini_value_intensities:
    #    break
    #index_mini_intensities = msg.intensities.index(mini_value_intensities)
    #print (index_mini_intensities)
    #tolerance == 0.02
    #j = min_dist
    #for j in msg.ranges:
    print('___________________________________________________________________')
    full = len(full_ranges)
    for i in range(index,full,1):
        j = i
        #print(msg.ranges[i])
        #print(i)
        limit = (msg.ranges[i+1] - msg.ranges[i])
        if (full_intensities[i] != 0): 
          # if (abs(limit) >= 0.03):
          #    edge1_dist = msg.ranges[i]
          #    angle2 = ((i)*msg.angle_increment)
          #    print(0)
          #    print(i)
          #    break
           if (full_intensities[i+1] == 0):
              if (full_intensities[i+2] != 0): # and full_ranges[i+2] != 'inf'):
                 limit = (full_ranges[i] - full_ranges[i+2])
                 if (abs(limit) >= 0.03):
                    edge1_dist = full_ranges[i]
                    angle2 = ((i)*msg.angle_increment)
                    #print(msg.ranges[i])
                    #print(msg.ranges[i+1])
                    #print(msg.ranges[i+2])
                    #print('edge1_dist: %f'%(edge1_dist*100))
                    #print('Angle_of_edge1: %f' %degrees((i)*msg.angle_increment))
                    print(1)
                    #print(i)
                    break
              if (full_intensities[i+2] == 0):
                 if (full_intensities[i+3] != 0):
                    limit = (full_ranges[i] - full_ranges[i+3])
                    if (abs(limit) >= 0.03):
                       edge1_dist = full_ranges[i]
                       angle2 = ((i)*msg.angle_increment)
                      # print('edge1_dist: %f'%(edge1_dist*100))
                      # print('Angle_of_edge1: %f' %degrees((i)*msg.angle_increment))
                       print(2)
                      # print(i)
                       break
                 if (full_intensities[i+3] == 0):
                    edge1_dist = full_ranges[i]
                    angle2 = ((i)*msg.angle_increment)
                   # print('edge1_dist: %f'%(edge1_dist*100))
                   # print('Angle_of_edge1: %f' %degrees(i*msg.angle_increment))
                    print(3)
                   # print(i)
                    break
                 elif (abs(limit) >= 0.04):
                      edge1_dist = full_ranges[j]
                      angle2 = degrees(i*msg.angle_increment)
                    #  print('edge2_dist: %f'%(edge1_dist*100))
                    #  print('Angle_of_edge2: %f' %degrees(i*msg.angle_increment))
                      print(4)
                    #  print(i)
                      break
           elif (full_intensities[i+1] != 0 and full_intensities[i+2] != 0):
                limit = (full_ranges[i+2] - full_ranges[i+1])
                if (abs(limit) >= 0.03):
                    edge1_dist = full_ranges[i+1]
                    angle2 = ((i+1)*msg.angle_increment)
                    print('el')
                    break
        elif (full_intensities[i] == 0):
             continue
    #############################################################################################

    j = index
    for j in range(index,-50,-1):
        limit = (full_ranges[j] - full_ranges[j-1])
        if (full_intensities[j] != 0):
           if (full_intensities[j-1] == 0):
              if (full_intensities[j-2] != 0): #and full_intensities[j-2] != 0):
                 limit = (full_ranges[j] - full_ranges[j-2])
                 if (abs(limit) >= 0.03):
                    edge2_dist = full_ranges[j]
                    angle3 = degrees(j*msg.angle_increment)
                   # print('edge2_dist: %f'%(edge2_dist*100))
                   # print('Angle_of_edge2: %f' %degrees(j*msg.angle_increment))
                    print(-1)
                   # print(j)
                    break
              if (full_intensities[j-2] == 0):
                 if (full_intensities[j-3] != 0):
                    limit = (full_ranges[j] - full_ranges[j-3])
                    if (abs(limit) >= 0.03):
                       edge2_dist = full_ranges[j]
                       angle3 = degrees(j*msg.angle_increment)
                    #   print('edge2_dist: %f'%(edge2_dist*100))
                    #   print('Angle_of_edge2: %f' %degrees(j*msg.angle_increment))
                       print(-2)
                    #   print(j)
                       break
                 if (full_intensities[j-3] == 0 and full_intensities[j-4] == 0):
                    edge2_dist = full_ranges[j]
                    angle3 = degrees(j*msg.angle_increment)
                  #  print('edge2_dist: %f'%(edge2_dist*100))
                  #  print('Angle_of_edge2: %f' %degrees(j*msg.angle_increment))
                    print(-3)
                  #  print(i)
                    break
                 elif (abs(limit) >= 0.09):
                      edge2_dist = full_ranges[j]
                      angle3 = degrees(j*msg.angle_increment)
                     # print('edge2_dist: %f'%(edge1_dist*100))
                     # print('Angle_of_edge2: %f' %degrees(j*msg.angle_increment))
                      print(-4)
                     # print(j)
                      break
           elif (full_intensities[j-1] != 0 and full_intensities[j-2] != 0 and full_intensities[j-3] != 0):
                limit1 = (full_ranges[j-2] - full_ranges[j-1])
                if (abs(limit) >= 0.03):
                    edge2_dist = full_ranges[j-1]
                    angle3 = ((j-1)*msg.angle_increment)
                    print('-el')
                    break
        elif (full_intensities[i] == 0):
             continue

    print'edge2_dist: %f          ' %(edge2_dist*100), 'edge1_dist: %f          ' %(edge1_dist*100)
    print'Angle_of_edge2: %f    ' %degrees(j*msg.angle_increment), 'Angle_of_edge1: %f    ' %degrees(i*msg.angle_increment)

    #x0 = abs((abs((min_dist*100))) * (sin((angle1))))
    #y0 = abs((abs((min_dist*100))) * (cos((angle1))))
    #x1 = abs((abs((edge1_dist*100))) * (sin((angle2))))
    #y1 = abs((abs((edge1_dist*100))) * (cos((angle2))))
    x2 = abs(round(edge2_dist*100) * cos(round(angle3)))
    y2 = abs(round(edge2_dist*100) * sin(round(angle3)))
    #print(x0)
    #print(y0)
    #print(x1)
    #print(y1)
    
    a0 = abs(sin((angle1))) 
    b0 = abs(cos((angle1)))
    a1 = abs(sin((angle2))) 
    b1 = abs(cos((angle2)))
    c0 = abs((min_dist*100))
    c1 = abs((edge1_dist*100))

    x0 = round((c0 * a0),4)
    y0 = round((c0 * b0),4)
    x1 = round((c1 * a0),4)
    y1 = round((c1 * b0),4)
    

    s1 = numpy.array((x0,y0))
    s2 = numpy.array((x1,y1))
    s3 = numpy.array((x2,y2))
    #print(s1)
    #print(s2)
    side1 = numpy.linalg.norm(s2 - s1)
    dist = sqrt((x1 - x0) ** 2 + (y1 - y0) ** 2)
    side2 = numpy.linalg.norm(s3 - s1)
    print(side1)
    print(dist)
    #print(side2)
    print ('_________________________________________________________________')

    ranges = 0                   
    full_ranges = 0


rospy.init_node('scan_values')
sub = rospy.Subscriber('/scan', LaserScan, callback)
rospy.spin()
