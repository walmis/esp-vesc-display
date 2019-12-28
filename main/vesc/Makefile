REVISION=ceb8b7bf7dbc8b0e9f19884eafc59ed36f4270e8
SITE=https://raw.githubusercontent.com/vedderb/bldc

FILES = buffer.c buffer.h confgenerator.c confgenerator.h datatypes.h packet.c packet.h

all: $(FILES)

$(FILES): %:
	@wget $(SITE)/$(REVISION)/$@ -O- > $@
	sed -i "s/#include \"ch.h\"/typedef uint32_t systime_t;/" $@
	sed -i "s/#include \"conf_general.h\"//" $@
	


clean:
	rm $(FILES)
