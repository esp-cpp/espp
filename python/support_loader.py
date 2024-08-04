try:
    import espp
except ImportError:
    import os
    import sys
    dirpath = os.path.dirname(os.path.realpath(__file__))
    sys.path.append(dirpath + "/../lib/pc")
    import espp
else:
    print("espp imported")

print("Imported espp from: ", espp.__file__)
