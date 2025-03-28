
# Building Python wheel
all:
	@rm -rf dist/*
	bash build_wheel.sh
	bash tweak_sdist.sh

upload:
	python3 -m twine upload --repository pypi dist/*

clean:
	rm -rf dist 
