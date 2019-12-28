#!/bin/sh

REVISION=ceb8b7bf7dbc8b0e9f19884eafc59ed36f4270e8
SITE=https://raw.githubusercontent.com/vedderb/bldc/

FILES=buffer.c buffer.h confgenerator.c confgenerator.h crc.c crc.h datatypes.h packet.c packet.h

for f in $FILES; do
	wget $(SITE)/$(REVISION)/$f -O- > $f
done

	
