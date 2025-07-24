; LightBurn Core 2.0.00-RC-8 @ 24d29f7 Qt6.5.7
; GRBL-LPC device profile, absolute coords
; Bounds: X16 Y14 to X75 Y71
G00 G17 G40 G21 G54
G90
M4
; Cut @ 900 mm/min, 60% power
M8
G0 X16Y14
; Layer C00 Pass 1 of 2
G1 Y71S600F900
X75
Y14
X16
; Layer C00 Pass 2 of 2
Y71
X75
Y14
X16
M9
G1 S0
M5
G90
; return to user-defined finish pos
G0 X0Y0
M2
