WIDTH=1920
HEIGHT=1080
PREFIX=pic
PROCDIRS=$(shell find . -maxdepth 1 -name '[0-9]*' -printf '%P\n')
POVS=$(shell find . -maxdepth 1 -name '${PREFIX}*.pov' -printf '%P\n' | sed 's/^${PREFIX}//' | sort -n | sed 's/^/${PREFIX}/')
PNGS=$(POVS:.pov=.png)

.PHONY: all clean distclean


all: $(PNGS)

help:
	@echo Number of frames to render:    $$(echo $(PNGS) | wc -w)
	@echo "Number of available processes: $$(grep "^core id" /proc/cpuinfo | sort -u | wc -l)"
	@echo
	@echo "Start with \"make -j$$(grep "^core id" /proc/cpuinfo | sort -u | wc -l)\"."

%.png: %.pov
	povray "+i$<" "+o$@" +fn8 -v -ga -d +a +q9 "+w${WIDTH}" "+h${HEIGHT}" +WT1
# +UA might cause that sky spheres are not rendered or that png's sometimes contains an alpha channel and sometimes not which confuses mencoder
# +WT1 limits the number of work threads to a single one
#povray "+i$<" "+o$@" +fn8 -v -ga -d +a +q9 +UA "+w${width}" "+h${height}"

clean:
	@rm -f $(PNGS)

distclean:
	@rm -f $(PNGS) $(POVS) pic*.pov-state
	@rm -rf $(PROCDIRS)
