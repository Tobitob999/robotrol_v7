; Robotrol Plane G-code Interpreter Test
; Tests: G21, G90/G91, G0, G1, G2, G3, F word, layer Z change, M ignore
G21        ; metric
G90        ; absolute
G92 E0     ; reset extruder (ignored)

; --- Layer 0: Z=0, simple square + arc ---
G0 X0 Y0 Z0 F6000
G1 X30 Y0 F1200
G1 X30 Y30
G1 X0 Y30
G1 X0 Y0

; Arc: full circle center (15,15) r=15, CCW
G2 X0 Y0 I15 J15 F800

; --- Layer 1: Z=5 (W offset) ---
G0 Z5 F3000
G0 X0 Y0

; Relative mode test
G91
G1 X10 Y0 F1200
G1 X0 Y10
G1 X-10 Y0
G1 X0 Y-10
G90

; Back to absolute, small arc CW
G0 X20 Y10
G3 X10 Y20 I-10 J10 F800

; M commands (all ignored)
M104 S200
M109 S200
M106 S255
M107

; End
G0 X0 Y0 Z0 F6000
