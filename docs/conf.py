# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

project = 'PlaCo'
copyright = '2023, Rhoban Team'
author = 'Rhoban Team'

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = ['sphinx.ext.autodoc', 'sphinx_copybutton', 'sphinxcontrib.video']

templates_path = ['_templates']
exclude_patterns = ['_build', 'Thumbs.db', '.DS_Store']

# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

html_theme = "pydata_sphinx_theme"

html_theme_options = {
    "show_nav_level": 2,
    "show_toc_level": 2,
    "icon_links": [
        {
            # Label for this link
            "name": "GitHub",
            # URL where the link will redirect
            "url": "https://github.com/rhoban/placo",  # required
            # Icon class (if "type": "fontawesome"), or path to local image (if "type": "local")
            "icon": "fa-brands fa-square-github",
            # The type of image to be used (see below for details)
            "type": "fontawesome",
        }
   ]
}

html_css_files = [
    'placo.css',
]

html_logo = "_static/placo.png"

html_static_path = ['_static']
html_page_context = ""

autoclass_content = 'both'

from docutils import nodes

def example_role(role, rawtext, text, lineno, inliner, options={}, content=[]):
    uri = 'https://github.com/rhoban/placo-examples/blob/master/' + text
    return [nodes.reference('Example code: '+text, 'Example code: '+text, refuri=uri, **options)], []

def setup(app):
    app.add_role("example", example_role)

import os
import sys

sys.path.insert(0, os.path.abspath('./module/'))

import generate_rsts
