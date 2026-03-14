; Tobias - Script/Cursive letterforms
; Plane G-code Interpreter
; -------------------------------------------
; Z=0  : pen down (on surface)
; Z=3  : pen up (lifted)
; x-height = 14mm, caps/ascenders = 22mm
; Total width ~55mm
; Draw: F800  Travel: F3000
; -------------------------------------------
; Arc geometry (all verified):
;   o  : G3 from (15,0)  center (15,7)  r=7  full circle CCW
;   b  : G3 from (23,7)  center (27,7)  r=4  full circle CCW
;   a  : G3 from (47,7)  center (41,7)  r=6  full circle CCW

G21       ; metric mm
G90       ; absolute coordinates

; ============================================================
; T  (x: 0-11)
; ============================================================
; Stem upstroke
G0 X3 Y0 Z3 F3000
G1 Z0 F800
G1 X3 Y22
G1 X5 Y22
G1 Z3
; Crossbar + diagonal exit to o
G0 X0 Y14 Z3
G1 Z0
G1 X11 Y14
G1 X9 Y0
G1 Z3

; ============================================================
; o  (x: 9-23)
; ============================================================
G0 X11 Y0 Z3
G1 Z0
G1 X15 Y0              ; approach bottom of oval
G3 X15 Y0 I0 J7        ; full CCW circle  center=(15,7) r=7
G1 X23 Y0              ; exit stroke right -> b
G1 Z3

; ============================================================
; b  (x: 23-33)
; ============================================================
G0 X23 Y0 Z3
G1 Z0
G1 X23 Y22             ; ascender upstroke
G1 X23 Y7              ; back down to bowl height
G3 X23 Y7 I4 J0        ; full CCW bowl  center=(27,7) r=4
G1 X33 Y0              ; exit diagonal -> i
G1 Z3

; ============================================================
; i  (x: 33-39)
; ============================================================
G0 X33 Y0 Z3
G1 Z0
G1 X34 Y14             ; upstroke
G1 X36 Y14             ; top
G1 X37 Y0              ; downstroke
G1 X39 Y0              ; connecting stroke -> a
G1 Z3
; dot
G0 X34 Y18 Z3
G1 Z0
G1 X36 Y18
G1 Z3

; ============================================================
; a  (x: 39-53)
; ============================================================
G0 X39 Y0 Z3
G1 Z0
G1 X47 Y7              ; entry upstroke to right side of oval
G3 X47 Y7 I-6 J0       ; full CCW loop  center=(41,7) r=6
G1 X47 Y0              ; right side downstroke
G1 X53 Y0              ; exit -> s
G1 Z3

; ============================================================
; s  (x: 53-67)
; ============================================================
G0 X53 Y0 Z3
G1 Z0
; lower body of s (right-leaning entry)
G1 X57 Y2
G1 X60 Y5
G1 X59 Y8
; middle waist
G1 X55 Y10
G1 X53 Y9
G1 X53 Y7
; upper curve
G1 X56 Y8
G1 X60 Y10
G1 X61 Y12
G1 X59 Y14
G1 X56 Y14
G1 X54 Y13
G1 Z3

; ============================================================
; Return home
; ============================================================
G0 X0 Y0 Z3 F3000
