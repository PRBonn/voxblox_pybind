install:
	pip3 -v install .

uninstall:
	pip3 -v uninstall -y voxblox

clean:
	git clean -xf .

test_install:
	python3 -m unittest tests.test_installation

test:test_install
	python3 -m unittest discover -v tests

docker:
	@echo Building builder docker container
	docker-compose build builder

docker_run:
	@echo Building Running docker
	docker-compose run --rm builder

docker_push:
	@echo Pushing builder image to docker registry
	docker-compose push builder
