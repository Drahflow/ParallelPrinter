#!/usr/bin/python3

RPM = 10.0
Startup = 0.02 # second
Run = 0.05 # second
Stop = 0.02 # second
Microsteps = 256

pulsesPerSecond = RPM / 60 * Microsteps * 200

tickFrequency = 500000
runDt = int(pulsesPerSecond / tickFrequency * 2 ** 32)
startDDT = int(runDt / Startup / tickFrequency)
stopDDT = 2 ** 32 - int(runDt / Stop / tickFrequency)

startPulses = 0
runPulses = 0
stopPulses = 0
dt = 0
t = 0
for i in range(int(tickFrequency * Startup)):
    if t > 2 ** 32:
        t -= 2 ** 32
        startPulses = startPulses + 1
    t += dt
    dt += startDDT

t = 0
dt = runDt
for i in range(int(tickFrequency * Run)):
    if t > 2 ** 32:
        t -= 2 ** 32
        runPulses = runPulses + 1
    t += dt

t = 0
dt = runDt
for i in range(int(tickFrequency * Stop)):
    if t > 2 ** 32:
        t -= 2 ** 32
        stopPulses = stopPulses + 1
    t += dt
    dt += stopDDT
    if dt > 2 ** 32:
        dt -= 2 ** 32

# FIXME: Timer zero-values are wrong
print("config:<whatever> %d 0 %d %d %d %d 0 %d %d %d %d 0 %d %d %d" % (
    startPulses, 0, startDDT, 0,
    runPulses, runDt, 0, 0,
    stopPulses, runDt, stopDDT, 0,
))
