import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/yjy/ican_shoot/ican_shoot/install/ican_referee_system'
