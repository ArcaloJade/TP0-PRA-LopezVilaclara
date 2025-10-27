import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/alumno1/Documents/TP0-PRA-LopezVilaclara/ros2_ws/install/tpf_part0'
