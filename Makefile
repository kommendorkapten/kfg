SUBDIRS = src tests examples tools

all:
	@for dir in $(SUBDIRS); do \
		$(MAKE) -C $$dir all; \
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
