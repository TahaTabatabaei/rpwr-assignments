import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/tahaos/Code/Projects/rpwr-assignments/03_ros/ws/ros/install/turtle_party'
