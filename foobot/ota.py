import sys
import time

infile = open(sys.argv[1], "rb")
outfile = open(sys.argv[2], "wb")

for byte in infile.read():
  print "%x" % ord(byte)
  outfile.write(bytearray(byte))
  outfile.flush()
  time.sleep(0.1)

outfile.write(bytearray('\r'))

