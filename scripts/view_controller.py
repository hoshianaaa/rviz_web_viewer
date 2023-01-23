#!/usr/bin/python

import math

import rospy
from view_controller_msgs.msg import CameraPlacement
from geometry_msgs.msg import Point, Vector3
from std_msgs.msg import String
import json
from geometry_msgs.msg import *

import json
import os

def read_json_file(file_name):
  if os.path.exists(file_name):
    with open(file_name, 'r') as f:
      json_load = json.load(f)
      return json_load, True
  return None, False

def write_json_file(file_name, dict_data):
  with open(file_name, 'w') as f:
    json.dump(dict_data, f, indent=2)

# https://qiita.com/kitasenjudesign/items/e785e00736161ec238ae
def chokkou(r,theta,phi):
  x = r * math.sin(theta) * math.cos(phi);
  y = r * math.sin(theta) * math.sin(phi);
  z = r * math.cos(theta);
  return (x,y,z)

class ViewController:
  def __init__(self):
    global data
    self.pub = rospy.Publisher("/rviz/camera_placement", CameraPlacement, queue_size = 1)
    rospy.Subscriber("/browser/viewer_data", String, self.callback)
    self.point_pub = rospy.Publisher("/rviz_forcus_point", PointStamped, queue_size = 1)
    self.rviz_view_state_pub = rospy.Publisher("/rviz_view_state", String, queue_size = 1)
    r = rospy.Rate(100)

    while not rospy.is_shutdown():
      msg = String()
      msg.data = json.dumps(data)
#      print(msg)
      self.rviz_view_state_pub.publish(msg)
      r.sleep()

      print("======== while =======")
      print(data["theta"], data["phi"], data["r"])
      print("=========================")

      self.publish(data["theta"], data["phi"], data["r"], data["fx"], data["fy"], data["fz"])
  
  def callback(self,msg):

    global f_name, data

#    print(msg)
    String_data = msg.data
#    print(String_data)
    #json_string = String_data.strip().decode()
    json_dict = json.loads(String_data)
    print(json_dict)

    theta = json_dict["data1"]
    phi = json_dict["data2"]
    r = json_dict["data3"]
    fx = json_dict["data4"]
    fy = json_dict["data5"]
    fz = json_dict["data6"]

#    print(theta,phi,r)

    msg = PointStamped()
    msg.point.x = fx
    msg.point.y = fy
    msg.point.z = fz
    msg.header.frame_id = "base_link"

    self.point_pub.publish(msg)

    print("======== callback =======")
    print(theta, phi, r, fx, fy, fz)
    print("=========================")

#    self.publish( theta, phi, r, fx, fy, fz)

    data["theta"] = theta
    data["phi"] = phi
    data["r"] = r
    data["fx"] = fx
    data["fy"] = fy
    data["fz"] = fz

    write_json_file(f_name, data)


  def publish(self,theta,phi,r, fx, fy, fz):

    cx, cy, cz = chokkou(r, -theta, -phi)

    cp = CameraPlacement()

    p = Point(cx + fx, cy + fy, cz + fz)
    cp.eye.point = p
    cp.eye.header.frame_id = "base_link"

    f = Point(fx, fy, fz)
    cp.focus.point = f
    cp.focus.header.frame_id = "base_link"

    up = Vector3(0, 0, 1)
    cp.up.vector = up
    cp.up.header.frame_id = "base_link"

    cp.time_from_start = rospy.Duration(0.01)

#    print(cp)
    self.pub.publish(cp)

node_name = "rviz_view_controller"
rospy.init_node(node_name)
f_name = os.environ['HOME'] + "/.ros/" + node_name + "-mode.json"
data, read_sucess = read_json_file(f_name)
print(data)

if read_sucess == False:
  print("cannot find file")
  data = {"theta": 0,"phi":0,"r":0,"fx":0,"fy":0,"fz":0}
  write_json_file(f_name, data)


vc = ViewController()
rospy.spin()
