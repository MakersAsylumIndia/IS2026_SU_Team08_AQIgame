"""
AQI Pet Device - 3D printable enclosure
Two-part clamshell for ESP32 + 3.2in 240x320 TFT, KY-040 encoder,
MQ-135 + MQ-2 gas sensors, BME680, OV2640 camera, buttons + power switch.
Units: millimetres. Author tool: CadQuery 2.7
"""
import math
import cadquery as cq

# ---------------------------------------------------------------- parameters
WALL        = 2.4     # shell wall thickness
LID_T       = 2.4     # back lid thickness
TOTAL_D     = 34.0    # external depth (front face -> back face)
FILLET      = 4.0     # outer vertical edge fillet
CLR         = 0.40    # fit clearance (total) between lid and inner cavity

INT_W       = 64.0    # internal cavity width
INT_H       = 126.0   # internal cavity height
EXT_W       = INT_W + 2 * WALL
EXT_H       = INT_H + 2 * WALL

# display (3.2in 240x320 ILI9341 module)
PCB_W, PCB_H        = 56.0, 86.0    # display board outline
WIN_W, WIN_H        = 50.0, 66.0    # visible glass window
DISP_CY             = 14.0          # display centre, Y (upper region)
POCKET_D            = 1.2           # seat recess depth on inner front face

# controls (front face)
ENC_CTR     = (-14.0, -45.0)        # rotary encoder shaft
ENC_HOLE    = 8.0
FBTN1       = ( 8.0, -45.0)         # two front push buttons
FBTN2       = ( 22.0, -45.0)
FBTN_HOLE   = 6.5

# side brightness buttons (right wall)
SBTN_HOLE   = 6.5
SBTN_Y      = (20.0, 2.0)           # two of them, stacked
SBTN_Z      = 11.0                  # depth position from front face

# top power button (round switch)
PWR_HOLE    = 16.0
PWR_Z       = 14.0                  # depth position from front face

# bottom USB slot for ESP32
USB_W, USB_H = 13.0, 7.0
USB_Z        = 10.0

# joining bosses (M3 self-tap)
BOSS_OD     = 7.0
BOSS_PILOT  = 2.5
SCREW_CLR   = 3.4
BOSS_INSET  = 6.5                   # centre inset from inner cavity corner

# back lid sensor / camera layout (lid-face coordinates)
MQ135_CTR   = (-16.0,  30.0)
MQ2_CTR     = ( 16.0,  30.0)
MQ_VENT_R   = 12.5                  # vent field radius
MQ_HOLE     = 1.8                   # individual vent hole dia
BME_CTR     = (0.0,  2.0)
CAM_CTR     = (0.0, -34.0)
CAM_HOLE    = 9.0

BOSS_TOP_Z  = TOTAL_D - LID_T       # boss height plane / lid seat
WALL_DEPTH  = TOTAL_D               # front shell wall length in Z


# ---------------------------------------------------------------- helpers
def rrect_prism(w, h, d, fillet, z0=0.0):
    """Rounded-corner rectangular prism, +Z extrusion from z0."""
    s = (cq.Workplane("XY").workplane(offset=z0)
         .rect(w, h).extrude(d))
    if fillet > 0:
        s = s.edges("|Z").fillet(fillet)
    return s


def polar_points(cx, cy, field_r, hole_r, pitch):
    """Hex-ish polar grid of points inside a circle for a vent field."""
    pts = [(cx, cy)]
    ring = 1
    while ring * pitch <= field_r:
        r = ring * pitch
        n = max(6, int(round(2 * math.pi * r / pitch)))
        for i in range(n):
            a = 2 * math.pi * i / n
            px, py = cx + r * math.cos(a), cy + r * math.sin(a)
            if math.hypot(px - cx, py - cy) <= field_r - hole_r:
                pts.append((px, py))
        ring += 1
    return pts


def boss_centres():
    x = INT_W / 2 - BOSS_INSET
    y = INT_H / 2 - BOSS_INSET
    return [(x, y), (-x, y), (x, -y), (-x, -y)]


# ---------------------------------------------------------------- front shell
def front_shell():
    body = rrect_prism(EXT_W, EXT_H, WALL_DEPTH, FILLET, 0.0)

    # hollow cavity, open at the back (leaves WALL thick front face)
    cav = rrect_prism(INT_W, INT_H, WALL_DEPTH, max(FILLET - WALL, 0.5), WALL)
    body = body.cut(cav)

    # display window through front wall
    body = (body.faces("<Z").workplane(origin=(0, DISP_CY, 0))
            .rect(WIN_W, WIN_H).cutThruAll())

    # display seat pocket on inner front face (board rests here)
    seat = (cq.Workplane("XY").workplane(offset=WALL)
            .center(0, DISP_CY).rect(PCB_W, PCB_H).extrude(POCKET_D))
    body = body.cut(seat)

    # rotary encoder + two front buttons
    for (cx, cy), dia in [(ENC_CTR, ENC_HOLE),
                          (FBTN1, FBTN_HOLE), (FBTN2, FBTN_HOLE)]:
        body = (body.faces("<Z").workplane(origin=(cx, cy, 0))
                .circle(dia / 2).cutThruAll())

    # two side brightness buttons through the right (+X) wall
    for sy in SBTN_Y:
        cutter = (cq.Workplane("YZ").workplane(offset=EXT_W / 2)
                  .center(sy, SBTN_Z).circle(SBTN_HOLE / 2)
                  .extrude(-WALL - 2))
        body = body.cut(cutter)

    # top power switch through the top (+Y) wall
    pwr = (cq.Workplane("XZ").workplane(offset=EXT_H / 2)
           .center(0, PWR_Z).circle(PWR_HOLE / 2).extrude(-WALL - 2))
    body = body.cut(pwr)

    # bottom USB slot through the bottom (-Y) wall
    usb = (cq.Workplane("XZ").workplane(offset=-EXT_H / 2)
           .center(0, USB_Z).rect(USB_W, USB_H).extrude(WALL + 2))
    body = body.cut(usb)

    # corner bosses (front inner face -> lid seat) with pilot holes
    for (bx, by) in boss_centres():
        post = (cq.Workplane("XY").workplane(offset=WALL)
                .center(bx, by).circle(BOSS_OD / 2)
                .extrude(BOSS_TOP_Z - WALL))
        body = body.union(post)
        hole = (cq.Workplane("XY").workplane(offset=BOSS_TOP_Z)
                .center(bx, by).circle(BOSS_PILOT / 2)
                .extrude(-(BOSS_TOP_Z - WALL) + 0.5))
        body = body.cut(hole)

    return body


# ---------------------------------------------------------------- back lid
def back_lid():
    lid_w, lid_h = INT_W - CLR, INT_H - CLR
    lid = rrect_prism(lid_w, lid_h, LID_T, max(FILLET - WALL, 0.5), BOSS_TOP_Z)

    # MQ-135 and MQ-2 protective vent fields
    for (cx, cy) in (MQ135_CTR, MQ2_CTR):
        for (px, py) in polar_points(cx, cy, MQ_VENT_R, MQ_HOLE / 2, 3.4):
            v = (cq.Workplane("XY").workplane(offset=BOSS_TOP_Z - 1)
                 .center(px, py).circle(MQ_HOLE / 2).extrude(LID_T + 2))
            lid = lid.cut(v)

    # BME680 small vent cluster (3x3)
    for ix in (-3.0, 0.0, 3.0):
        for iy in (-3.0, 0.0, 3.0):
            v = (cq.Workplane("XY").workplane(offset=BOSS_TOP_Z - 1)
                 .center(BME_CTR[0] + ix, BME_CTR[1] + iy)
                 .circle(0.75).extrude(LID_T + 2))
            lid = lid.cut(v)

    # camera port
    cam = (cq.Workplane("XY").workplane(offset=BOSS_TOP_Z - 1)
           .center(*CAM_CTR).circle(CAM_HOLE / 2).extrude(LID_T + 2))
    lid = lid.cut(cam)

    # countersunk screw clearance holes at the four corners
    for (bx, by) in boss_centres():
        h = (cq.Workplane("XY").workplane(offset=BOSS_TOP_Z - 1)
             .center(bx, by).circle(SCREW_CLR / 2).extrude(LID_T + 2))
        lid = lid.cut(h)
        cs = (cq.Workplane("XY").workplane(offset=TOTAL_D)
              .center(bx, by).circle(SCREW_CLR / 2 + 1.6)
              .workplane(offset=-1.6).circle(SCREW_CLR / 2)
              .loft(combine=True))
        lid = lid.cut(cs)

    return lid


# ---------------------------------------------------------------- build/export
fs = front_shell()
bl = back_lid()
assembly = fs.union(bl)

cq.exporters.export(fs, "/home/claude/front_shell.step")
cq.exporters.export(fs, "/home/claude/front_shell.stl")
cq.exporters.export(bl, "/home/claude/back_lid.step")
cq.exporters.export(bl, "/home/claude/back_lid.stl")
cq.exporters.export(assembly, "/home/claude/assembly_preview.step")

print("front vol cm3:", round(fs.val().Volume() / 1000, 1))
print("lid   vol cm3:", round(bl.val().Volume() / 1000, 1))
print("external WxHxD:", round(EXT_W, 1), round(EXT_H, 1), round(TOTAL_D, 1))
print("done")
