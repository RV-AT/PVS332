onbreak {quit -f}
onerror {quit -f}

vsim -t 1ps -lib xil_defaultlib BLKRAM8_opt

do {wave.do}

view wave
view structure
view signals

do {BLKRAM8.udo}

run -all

quit -force
