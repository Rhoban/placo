import os
import sys
import requests

if not os.path.exists("module/placo.py"):
    infos = requests.get('https://pypi.org/pypi/placo/json').json()
    print(f"No placo.py file found, downloading version {infos['info']['version']}")
    for entry in infos['urls']:
        name = f"placo-{infos['info']['version']}"
        if entry['url'].endswith('.tar.gz'):
            url = entry['url']
            os.system(f"wget {url} -O sdist.tar.gz >/dev/null")
            os.system(f"tar xvf sdist.tar.gz >/dev/null")
            os.system(f"cp {name}/placo.pyi module/placo.py")
            os.system(f"rm -rf sdist.tar.gz {name}")
        
exit()

sys.path.insert(0, os.path.abspath('./module/'))

import placo
import inspect

os.system("rm -rf placo/")
os.system("mkdir placo/")
methods = """
Methods
=======

"""

summary = """
Summary
=======

.. toctree::

  methods
"""

for name, object in inspect.getmembers(placo):
    filename = None

    # Classes
    if isinstance(object, type):
        contents = f"Class {name}\n"
        contents += "=============\n"

        contents += f".. autoclass:: placo.{name}\n"
        contents += "  :members:\n"
        
        with open(f"placo/{name}.rst", "w") as f:
            f.write(contents)

        summary += f"  {name}\n"
    
    # Function
    elif callable(object):
        methods += f".. autofunction:: placo.{name}\n\n"

with open("placo/methods.rst", "w") as f:
    f.write(methods)

with open("placo/summary.rst", "w") as f:
    f.write(summary)
