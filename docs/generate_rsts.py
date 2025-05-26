import os
import fnmatch
import sys
import requests

os.makedirs("module", exist_ok=True)

# We retrieve placo.py from the latest sdist placo.pyi file
if not os.path.islink("module/placo.py"):
  infos = requests.get('https://pypi.org/pypi/placo/json').json()
  print(f"No placo.py file found, downloading version {infos['info']['version']}")
  for entry in infos['urls']:
      name = f"placo-{infos['info']['version']}"
      if fnmatch.fnmatch(entry["url"], "*manylinux*.whl"):
          url = entry['url']
          print(name)

          os.system(f"mkdir placo_wheel")
          os.system(f"cd placo_wheel; wget {url} -O wheel.zip >/dev/null")
          os.system(f"cd placo_wheel; unzip wheel.zip >/dev/null")
          os.system(f"cp placo_wheel/cmeel.prefix/lib/*/site-packages/placo.pyi module/placo.py")
          os.system(f"rm -rf wheel.zip placo_wheel")
          break

this_dir = os.path.dirname(__file__)
sys.path.insert(0, this_dir + "/module/")

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
