import sys
sys.path.insert(0, '/var/www/flask2')
sys.stdout = sys.stderr

from hello import app as application
