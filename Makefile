SUBDIRS = lib src tests examples tools

all:
	@for dir in $(SUBDIRS); do \
		$(MAKE) -C $$dir all || exit $$?; \
	done

lint:
	@for dir in $(SUBDIRS); do \
		$(MAKE) -C $$dir lint; \
	done

clean:
	@for dir in $(SUBDIRS); do \
		$(MAKE) -C $$dir clean; \
	done
	$(MAKE) -C app clean

.PHONY: all clean lint $(SUBDIRS)

.PHONY: start-colima
start-colima:
	colima start --cpu 4 --memory 8 --disk 40

.PHONY: stop-colima
stop-colima:
	colima stop

.PHONY: docker
docker:
	docker build -t kfgl .

.PHONY: linux-console
linux-console:
	docker run -v "${PWD}:/src" -it kfgl

.PHONY: clang-defines
clang-defines:
	clang -dM -E -x c /dev/null

.PHONY: gcc-defines
gcc-defines:
	gcc -dM -E -x c /dev/null
