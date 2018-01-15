#!/usr/bin/env python

Ftim = 84e6
delay = 1

Npresc = Nreload = 0
fract = -1

while fract <> 0 or Npresc > 0xffff:
  Nreload = Nreload + 1
  Npresc = delay * Ftim / (Nreload + 1) - 1
  fract = Npresc - int(Npresc)

print "Npresc = %d\nNreload = %d" % (Npresc, Nreload)

