import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/ivan/AdR/p2_ws/src/p2_kf_adr/install/p2_kf_adr'
