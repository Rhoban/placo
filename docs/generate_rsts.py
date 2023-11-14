import os
import sys
import requests

os.makedirs("module", exist_ok=True)

# We retrieve placo.py from the latest sdist placo.pyi file
if not os.path.islink("module/placo.py"):
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

# We create a symbolic link to placo_utils
this_dir = os.path.dirname(__file__)
os.system("rm -rf module/placo_utils")
os.system(f"ln -sf {this_dir}/../python/placo_utils ./module/placo_utils")

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
Code reference
==============

.. toctree::

  methods
  ../placo_utils
"""

for name, object in inspect.getmembers(placo):
    filename = None
    
    # Function
    if callable(object) and not inspect.isclass(object):
        if object.__doc__:
          methods += f".. autofunction:: placo.{name}\n\n"

for category in placo.__groups__:
    filename = category.replace("::", "__")
    category_contents = f"{category}\n"
    category_contents += "="*(len(category)+1) + "\n\n"


    for class_name in placo.__groups__[category]:
        category_contents += f".. autoclass:: placo.{class_name}\n"
        category_contents += "  :members:\n"
        category_contents += "  :undoc-members:\n"

    with open(f"placo/{filename}.rst", "w") as f:
        f.write(category_contents)
    summary += "  " + filename + "\n"

with open("placo/methods.rst", "w") as f:
    f.write(methods)

with open("placo/summary.rst", "w") as f:
    f.write(summary)
