#!/usr/bin/python3

RPM = 800.0
Startup = 0.05 # second
Run = 1.0 # second
Stop = 0.05 # second
Microsteps = 256

pulsesPerSecond = RPM / 60 * Microsteps * 200

runDt = int(pulsesPerSecond / 2000000 * 2 ** 32)
startDDT = int(runDt / Startup / 2000000)
stopDDT = 2 ** 32 - int(runDt / Stop / 2000000)

startPulses = 0
runPulses = 0
stopPulses = 0
dt = 0
t = 0
for i in range(int(2000000 * Startup)):
    if t > 2 ** 32:
        t -= 2 ** 32
        startPulses = startPulses + 1
    t += dt
    dt += startDDT

t = 0
dt = runDt
for i in range(int(2000000 * Run)):
    if t > 2 ** 32:
        t -= 2 ** 32
        runPulses = runPulses + 1
    t += dt

t = 0
dt = runDt
for i in range(int(2000000 * Stop)):
    if t > 2 ** 32:
        t -= 2 ** 32
        stopPulses = stopPulses + 1
    t += dt
    dt += stopDDT
    if dt > 2 ** 32:
        dt -= 2 ** 32

print("M %d %d %d %d %d %d %d %d %d %d %d %d" % (
    startPulses, 0, startDDT, 0,
    runPulses, runDt, 0, 0,
    stopPulses, runDt, stopDDT, 0,
))
