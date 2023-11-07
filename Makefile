
# Building Python wheel
all:
	@rm -rf dist/*
	python -m build --sdist --wheel

upload:
	python3 -m twine upload --repository pypi dist/*

clean:
	rm -rf dist 
