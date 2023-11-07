#!/usr/bin/python3

# All length in mm

strutLen = 550
axleLen = 900

MOTORS = 7

displacements = [
        (40, 0, 5),
        (25, 40, 0),
        (5, -45, 10),
        (0, 0, 50),
        (-5, -45, 10),
        (-25, 40, 0),
        (-40, 0, 5),
]

starts = []

for m in range(MOTORS):
    d = displacements[m]
    starts.append(d[1] + (strutLen * strutLen - d[0] * d[0] - d[2] * d[2]) ** 0.5)

print(starts)

for z in range(0, strutLen, 10):
    minX = -99999999
    maxX = 99999999
    for m in range(MOTORS):
        l = - (strutLen * strutLen - d[2] * d[2] - z * z) ** 0.5 - d[0]
        r = (strutLen * strutLen - d[2] * d[2] - z * z) ** 0.5 + d[0]
        minX = max(minX, l)
        maxX = min(maxX, r)

    print("Low + high cuboid: %f x %f x %f" % (maxX - minX, axleLen - strutLen, z))

minZ = -9999999
for m in range(MOTORS):
    bsqr = strutLen * strutLen - ((axleLen - abs(d[1])) / 2) ** 2 - d[0] * d[0] - d[2] * d[2]
    if bsqr > 0:
        b = bsqr ** 0.5
        minZ = max(minZ, b)

for z in range(int(minZ // 10 * 10 + 10), strutLen, 10):
    minX = -99999999
    maxX = 99999999
    for m in range(MOTORS):
        l = - (strutLen * strutLen - d[2] * d[2] - z * z) ** 0.5 - d[0]
        r = (strutLen * strutLen - d[2] * d[2] - z * z) ** 0.5 + d[0]
        minX = max(minX, l)
        maxX = min(maxX, r)

    print("Full cuboid: %f x %f x %f" % (maxX - minX, axleLen, z - minZ))
