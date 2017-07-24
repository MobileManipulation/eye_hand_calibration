# coding: utf-8
import sys
print "XYZ: {} {} {}".format(*sys.argv[1:4])
print "RPY: {} {} {}".format(*sys.argv[6:3:-1])
