import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/jj/project_total/camera_ws/src/semantic_builder/install/semantic_builder'
