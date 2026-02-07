SUBDIRS = src tests examples

all:
	@for dir in $(SUBDIRS); do \
		$(MAKE) -C $$dir; \
	done

app:
	$(MAKE) -C app

clean:
	@for dir in $(SUBDIRS); do \
		$(MAKE) -C $$dir clean; \
	done
	$(MAKE) -C app clean

.PHONY: all clean app $(SUBDIRS)
