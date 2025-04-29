import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/ivan/p2_kl_ilv/install/p2_kf_ilv'
