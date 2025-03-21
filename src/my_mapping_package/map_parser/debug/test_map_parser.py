"""
Test for MapObject and MapParser classes
"""
import time 
import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from map_object import MapObject 
from map_parser import MapParser

time_in = time.time()
map_parser = MapParser('Town03', 0)
time_out = time.time()
print("Time MapParser = ", time_out-time_in)

time_in = time.time()
map_object = MapObject('Town03', 0)
time_out = time.time()
print("Time MapObject = ", time_out-time_in)