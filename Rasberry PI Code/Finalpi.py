import pygame
import math
import sys
import socket
import RPi.GPIO as GPIO
import spidev

# ============================================================
#  CONFIGURATION
# ============================================================
COMPUTER_IP   = "192.168.1.10"
COMPUTER_PORT = 5006          # Pi -> C++ (controls)
PI_IP         = "0.0.0.0"
PI_PORT       = 5005          # C++ -> Pi (flight data)

# ============================================================
#  MCP3008 SPI SETUP (throttle potentiometer)
# ============================================================
# Wiring:
#   MCP3008 VDD  -> 3.3V
#   MCP3008 VREF -> 3.3V
#   MCP3008 AGND -> GND
#   MCP3008 DGND -> GND
#   MCP3008 CLK  -> GPIO 11 (SCLK)
#   MCP3008 DOUT -> GPIO 9  (MISO)
#   MCP3008 DIN  -> GPIO 10 (MOSI)
#   MCP3008 CS   -> GPIO 8  (CE0)
#   Pot pin 1    -> 3.3V
#   Pot wiper    -> MCP3008 CH0
#   Pot pin 3    -> GND

THROTTLE_ADC_CHANNEL = 0       # MCP3008 channel the pot wiper is on (0–7)
THROTTLE_INVERT      = True   # Set True if pot is wired backwards (idle reads high)

adc = spidev.SpiDev()
adc.open(0, 0)                 # SPI bus 0, CE0
adc.max_speed_hz = 1_000_000
adc.mode = 0b00                # SPI mode 0,0

def read_mcp3008(channel: int) -> int:
    """Read a single-ended channel (0–7) from the MCP3008.
    Returns a raw 10-bit value: 0–1023."""
    if not 0 <= channel <= 7:
        raise ValueError(f"Channel must be 0–7, got {channel}")
    cmd = [0x01, (0x80 | (channel << 4)), 0x00]
    resp = adc.xfer2(cmd)
    return ((resp[1] & 0x03) << 8) | resp[2]

def read_throttle() -> float:
    """Return throttle position 0.0 (idle) to 1.0 (full thrust)."""
    try:
        raw = read_mcp3008(THROTTLE_ADC_CHANNEL)  # 0–1023
        val = raw / 1023.0
        if THROTTLE_INVERT:
            val = 1.0 - val
        return round(val, 4)
    except Exception as e:
        print(f"[THROTTLE] ADC read error: {e}")
        return 0.0

# ============================================================
#  GPIO
# ============================================================
FLAP_HALF_PIN = 27
FLAP_FULL_PIN = 22

GPIO.setmode(GPIO.BCM)
GPIO.setup(FLAP_HALF_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(FLAP_FULL_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# ============================================================
#  JOYSTICK CONFIGURATION
# ============================================================
YOKE_ROLL_AXIS  = 0
YOKE_PITCH_AXIS = 1
PITCH_INVERT    = False
ROLL_INVERT     = False
YOKE_DEADZONE   = 0.02

RUDDER_AXIS        = 2
RUDDER_LEFT_BRAKE  = 0
RUDDER_RIGHT_BRAKE = 1
RUDDER_INVERT      = False
RUDDER_DEADZONE    = 0.02

# ============================================================
#  DISPLAY
# ============================================================
pygame.init()
VIRTUAL_WIDTH, VIRTUAL_HEIGHT = 1300, 750

try:
    screen = pygame.display.set_mode(
        (VIRTUAL_WIDTH, VIRTUAL_HEIGHT),
        pygame.FULLSCREEN | pygame.SCALED
    )
except pygame.error:
    screen = pygame.display.set_mode(
        (VIRTUAL_WIDTH, VIRTUAL_HEIGHT),
        pygame.SCALED
    )

pygame.display.set_caption("RPi Flight Deck")
clock      = pygame.time.Clock()
pygame.mouse.set_visible(False)

FONT       = pygame.font.SysFont("Arial", 22, bold=True)
UNIT_FONT  = pygame.font.SysFont("Arial", 14, bold=True)
SMALL_FONT = pygame.font.SysFont("Arial", 13, bold=True)

# ============================================================
#  JOYSTICK MANAGER
# ============================================================
pygame.joystick.init()

_joysticks: dict[int, pygame.joystick.JoystickType] = {}

yoke           = None
yoke_name      = "Not connected"
yoke_connected = False

rudder           = None
rudder_name      = "Not connected"
rudder_connected = False


def _refresh_joystick_pool():
    for i in range(pygame.joystick.get_count()):
        if i not in _joysticks:
            j = pygame.joystick.Joystick(i)
            j.init()
            _joysticks[i] = j
            print(f"[JOY] Slot {i}: {j.get_name()}")


def _assign_devices():
    global yoke, yoke_name, yoke_connected
    global rudder, rudder_name, rudder_connected

    YOKE_KEYWORDS   = ["YOKO", "VIRTUAL FLY"]
    RUDDER_KEYWORDS = ["THRUSTMASTER", "TPR", "PENDULAR", "RUDDER", "T-RUDDER"]

    for j in _joysticks.values():
        name_up = j.get_name().upper()

        if not yoke_connected and any(k in name_up for k in YOKE_KEYWORDS):
            yoke           = j
            yoke_name      = j.get_name()
            yoke_connected = True
            print(f"[YOKE] Assigned: {yoke_name}")

        elif not rudder_connected and any(k in name_up for k in RUDDER_KEYWORDS):
            rudder           = j
            rudder_name      = j.get_name()
            rudder_connected = True
            print(f"[RUDDER] Assigned: {rudder_name}")

    if not yoke_connected and _joysticks:
        for j in _joysticks.values():
            if rudder and j.get_instance_id() == rudder.get_instance_id():
                continue
            yoke           = j
            yoke_name      = j.get_name()
            yoke_connected = True
            print(f"[YOKE] Fallback: {yoke_name}")
            break


_refresh_joystick_pool()
_assign_devices()


# ============================================================
#  INPUT HELPERS
# ============================================================
def apply_deadzone(value: float, dz: float) -> float:
    if abs(value) < dz:
        return 0.0
    sign = 1.0 if value > 0 else -1.0
    return sign * (abs(value) - dz) / (1.0 - dz)


def read_yoke() -> tuple[float, float]:
    if not yoke_connected or yoke is None:
        return 0.0, 0.0
    try:
        roll  = apply_deadzone(yoke.get_axis(YOKE_ROLL_AXIS),  YOKE_DEADZONE)
        pitch = apply_deadzone(yoke.get_axis(YOKE_PITCH_AXIS), YOKE_DEADZONE)
        if ROLL_INVERT:  roll  = -roll
        if PITCH_INVERT: pitch = -pitch
        return round(roll, 4), round(pitch, 4)
    except Exception as e:
        print(f"[YOKE] Read error: {e}")
        return 0.0, 0.0


def read_rudder() -> float:
    if not rudder_connected or rudder is None:
        return 0.0
    try:
        val = apply_deadzone(rudder.get_axis(RUDDER_AXIS), RUDDER_DEADZONE)
        if RUDDER_INVERT:
            val = -val
        return round(val, 4)
    except Exception as e:
        print(f"[RUDDER] Read error: {e}")
        return 0.0


def read_toe_brakes() -> tuple[float, float]:
    if not rudder_connected or rudder is None:
        return 0.0, 0.0
    try:
        left  = (1.0 - rudder.get_axis(RUDDER_LEFT_BRAKE))  / 2.0
        right = (1.0 - rudder.get_axis(RUDDER_RIGHT_BRAKE)) / 2.0
        return round(left, 3), round(right, 3)
    except Exception as e:
        print(f"[BRAKE] Read error: {e}")
        return 0.0, 0.0


# ============================================================
#  SMOOTHING
# ============================================================
class SmoothValue:
    def __init__(self, smoothing=0.08, is_heading=False):
        self.current    = 0.0
        self.target     = 0.0
        self.smoothing  = smoothing
        self.is_heading = is_heading

    def update(self, target):
        self.target = target
        if self.is_heading:
            diff = (self.target - self.current + 180) % 360 - 180
            self.current += diff * self.smoothing
        else:
            self.current += (self.target - self.current) * self.smoothing
        return self.current


# ============================================================
#  INSTRUMENTS
# ============================================================
class ArtificialHorizon:
    def __init__(self, x, y, radius):
        self.center  = (x, y)
        self.radius  = radius
        self.pitch   = SmoothValue()
        self.roll    = SmoothValue()
        self.bg_size = 1000
        self.bg      = pygame.Surface((self.bg_size, self.bg_size))
        self.bg.fill((70, 130, 180))
        pygame.draw.rect(self.bg, (139, 69, 19),
                         (0, self.bg_size // 2, self.bg_size, self.bg_size // 2))
        pygame.draw.line(self.bg, (255, 255, 255),
                         (0, self.bg_size // 2),
                         (self.bg_size, self.bg_size // 2), 4)
        fp = pygame.font.SysFont("Arial", 14, bold=True)
        fb = pygame.font.SysFont("Arial", 12, bold=True)
        self.font_pitch = fp
        self.font_bank  = fb
        for p in range(-90, 91, 5):
            if p == 0:
                continue
            y_off    = self.bg_size // 2 - (p * 5)
            is_major = p % 10 == 0
            w = 80 if p % 20 == 0 else (60 if is_major else 25)
            pygame.draw.line(self.bg, (255, 255, 255),
                             (self.bg_size // 2 - w // 2, y_off),
                             (self.bg_size // 2 + w // 2, y_off), 2)
            if is_major:
                txt = fp.render(str(abs(p)), True, (255, 255, 255))
                self.bg.blit(txt, (self.bg_size // 2 + w // 2 + 8,
                                   y_off - txt.get_height() // 2))
                self.bg.blit(txt, (self.bg_size // 2 - w // 2 - 8 - txt.get_width(),
                                   y_off - txt.get_height() // 2))

    def update(self, pitch, roll):
        self.pitch.update(pitch)
        self.roll.update(roll)

    def draw(self, surface):
        pitch_val    = max(-90, min(90, self.pitch.current))
        pitch_offset = pitch_val * 5
        view_rect    = pygame.Rect(0, 0, self.radius * 3, self.radius * 3)
        view_rect.center = (self.bg_size // 2, self.bg_size // 2 - pitch_offset)
        view_rect    = view_rect.clamp(self.bg.get_rect())
        pitched_view = self.bg.subsurface(view_rect)
        rotated      = pygame.transform.rotate(pitched_view, self.roll.current)
        temp_surf    = pygame.Surface((self.radius * 2, self.radius * 2), pygame.SRCALPHA)
        rot_rect     = rotated.get_rect(center=(self.radius, self.radius))
        temp_surf.blit(rotated, rot_rect)
        mask = pygame.Surface((self.radius * 2, self.radius * 2), pygame.SRCALPHA)
        pygame.draw.circle(mask, (255, 255, 255, 255),
                           (self.radius, self.radius), self.radius)
        temp_surf.blit(mask, (0, 0), special_flags=pygame.BLEND_RGBA_MIN)
        surface.blit(temp_surf,
                     (self.center[0] - self.radius, self.center[1] - self.radius))
        pygame.draw.circle(surface, (150, 150, 150), self.center, self.radius, 5)
        for ang_deg in [-60, -45, -30, -20, -10, 0, 10, 20, 30, 45, 60]:
            rad      = math.radians(90 + ang_deg)
            is_major = ang_deg in [0, -30, 30, -60, 60]
            length   = 15 if is_major else 8
            thick    = 3  if is_major else 2
            p_out = (self.center[0] + math.cos(rad) * self.radius,
                     self.center[1] - math.sin(rad) * self.radius)
            p_in  = (self.center[0] + math.cos(rad) * (self.radius - length),
                     self.center[1] - math.sin(rad) * (self.radius - length))
            pygame.draw.line(surface, (255, 255, 255), p_in, p_out, thick)
            if ang_deg != 0:
                td   = self.radius + 14
                tx   = self.center[0] + math.cos(rad) * td
                ty   = self.center[1] - math.sin(rad) * td
                atxt = self.font_bank.render(str(abs(ang_deg)), True, (255, 255, 255))
                surface.blit(atxt, atxt.get_rect(center=(tx, ty)))
        ptr_rad    = math.radians(90 + self.roll.current)
        tip        = (self.center[0] + math.cos(ptr_rad) * (self.radius - 16),
                      self.center[1] - math.sin(ptr_rad) * (self.radius - 16))
        base_left  = (self.center[0] + math.cos(ptr_rad + 0.1) * (self.radius - 30),
                      self.center[1] - math.sin(ptr_rad + 0.1) * (self.radius - 30))
        base_right = (self.center[0] + math.cos(ptr_rad - 0.1) * (self.radius - 30),
                      self.center[1] - math.sin(ptr_rad - 0.1) * (self.radius - 30))
        pygame.draw.polygon(surface, (255, 140, 0), [tip, base_left, base_right])
        pygame.draw.polygon(surface, (0, 0, 0),     [tip, base_left, base_right], 1)
        pygame.draw.line(surface, (255, 255, 0),
                         (self.center[0] - 60, self.center[1]),
                         (self.center[0] - 20, self.center[1]), 4)
        pygame.draw.line(surface, (255, 255, 0),
                         (self.center[0] + 20, self.center[1]),
                         (self.center[0] + 60, self.center[1]), 4)
        pygame.draw.circle(surface, (255, 255, 0), self.center, 5)


class AirspeedIndicator:
    def __init__(self, x, y, radius):
        self.center  = (x, y)
        self.radius  = radius
        self.value   = SmoothValue()
        self.max_val = 700
        self.font    = pygame.font.SysFont("Arial", 14)

    def update(self, target): self.value.update(target)

    def _get_angle(self, speed):
        return 225 - (min(speed / self.max_val, 1.0) * 270)

    def draw(self, surface):
        pygame.draw.circle(surface, (20, 20, 20),    self.center, self.radius)
        pygame.draw.circle(surface, (100, 100, 100), self.center, self.radius, 2)
        for s in range(0, self.max_val + 1, 10):
            angle_rad  = math.radians(self._get_angle(s))
            is_major   = s % 50 == 0
            tick_len   = 15 if is_major else 8
            tick_thick = 3  if is_major else 1
            p1 = (self.center[0] + math.cos(angle_rad) * self.radius,
                  self.center[1] - math.sin(angle_rad) * self.radius)
            p2 = (self.center[0] + math.cos(angle_rad) * (self.radius - tick_len),
                  self.center[1] - math.sin(angle_rad) * (self.radius - tick_len))
            pygame.draw.line(surface, (255, 255, 255), p1, p2, tick_thick)
            if is_major:
                tx  = self.center[0] + math.cos(angle_rad) * (self.radius - 35)
                ty  = self.center[1] - math.sin(angle_rad) * (self.radius - 35)
                txt = self.font.render(str(s), True, (200, 200, 200))
                surface.blit(txt, txt.get_rect(center=(tx, ty)))
        unit_txt = UNIT_FONT.render("KNOTS", True, (255, 255, 255))
        surface.blit(unit_txt,
                     (self.center[0] - unit_txt.get_width() // 2,
                      self.center[1] + 40))
        angle = math.radians(self._get_angle(self.value.current))
        tip   = (self.center[0] + math.cos(angle) * self.radius * 0.85,
                 self.center[1] - math.sin(angle) * self.radius * 0.85)
        pygame.draw.line(surface, (255, 255, 255), self.center, tip, 4)
        pygame.draw.circle(surface, (150, 150, 150), self.center, 8)


class HeadingIndicator:
    def __init__(self, x, y, radius):
        self.center        = (x, y)
        self.radius        = radius
        self.heading       = SmoothValue(is_heading=True)
        self.cardinal_font = pygame.font.SysFont("Arial", 24, bold=True)
        self.num_font      = pygame.font.SysFont("Arial", 16)

    def update(self, heading): self.heading.update(heading)

    def draw(self, surface):
        pygame.draw.circle(surface, (10, 10, 10),    self.center, self.radius)
        pygame.draw.circle(surface, (180, 180, 180), self.center, self.radius, 3)
        rot_offset = math.radians(self.heading.current)
        for deg in range(0, 360, 10):
            angle_rad = math.radians(deg) - rot_offset - math.pi / 2
            is_major  = deg % 30 == 0
            length    = 15 if is_major else 8
            p1 = (self.center[0] + math.cos(angle_rad) * self.radius,
                  self.center[1] + math.sin(angle_rad) * self.radius)
            p2 = (self.center[0] + math.cos(angle_rad) * (self.radius - length),
                  self.center[1] + math.sin(angle_rad) * (self.radius - length))
            pygame.draw.line(surface, (255, 255, 255), p1, p2, 2)
            if is_major:
                ld  = self.radius - 35
                lx  = self.center[0] + math.cos(angle_rad) * ld
                ly  = self.center[1] + math.sin(angle_rad) * ld
                val = {0: "N", 90: "E", 180: "S", 270: "W"}.get(deg, str(deg // 10))
                ts  = self.cardinal_font if not val.isdigit() else self.num_font
                txt = ts.render(val, True, (255, 255, 255))
                surface.blit(txt, txt.get_rect(center=(lx, ly)))
        hdg_val = f"{int(self.heading.current) % 360:03}°"
        hdg_txt = UNIT_FONT.render(hdg_val, True, (255, 255, 0))
        surface.blit(hdg_txt,
                     (self.center[0] - hdg_txt.get_width() // 2,
                      self.center[1] - 40))
        pygame.draw.polygon(surface, (255, 0, 0), [
            (self.center[0],      self.center[1] - self.radius + 5),
            (self.center[0] - 10, self.center[1] - self.radius - 15),
            (self.center[0] + 10, self.center[1] - self.radius - 15),
        ])


class DetailedAltimeter:
    def __init__(self, x, y, radius):
        self.center      = (x, y)
        self.radius      = radius
        self.altitude    = SmoothValue(smoothing=0.1)
        self.tick_font   = pygame.font.SysFont("Arial", 14, bold=True)
        self.label_font  = pygame.font.SysFont("Arial", 11, bold=True)
        self.drum_font   = pygame.font.SysFont("Courier New", 21, bold=True)
        self.legend_font = pygame.font.SysFont("Arial", 10)

    def update(self, alt): self.altitude.update(alt)

    def draw(self, surface):
        alt    = max(0, self.altitude.current)
        cx, cy = self.center
        r      = self.radius

        pygame.draw.circle(surface, (13, 13, 13), self.center, r)
        pygame.draw.circle(surface, (80, 80, 80), self.center, r, 3)

        for i in range(50):
            angle_deg = i / 50 * 360 - 90
            angle_rad = math.radians(angle_deg)
            is_major  = i % 5 == 0
            tick_len  = 18 if is_major else 9
            thick     = 3  if is_major else 1
            p1 = (cx + math.cos(angle_rad) * r,
                  cy + math.sin(angle_rad) * r)
            p2 = (cx + math.cos(angle_rad) * (r - tick_len),
                  cy + math.sin(angle_rad) * (r - tick_len))
            pygame.draw.line(surface, (255, 255, 255), p1, p2, thick)
            if is_major:
                n   = (i // 5) % 10
                tx  = cx + math.cos(angle_rad) * (r - 38)
                ty  = cy + math.sin(angle_rad) * (r - 38)
                txt = self.tick_font.render(str(n), True, (255, 255, 255))
                surface.blit(txt, txt.get_rect(center=(tx, ty)))

        for text, offset in [("ALTITUDE", -68), ("FEET", -55)]:
            t = self.label_font.render(text, True, (170, 170, 170))
            surface.blit(t, t.get_rect(center=(cx, cy + offset)))

        dw, dh = 116, 42
        dx, dy = cx - dw // 2, cy + 10
        pygame.draw.rect(surface, (26, 26, 26),
                         (dx - 4, dy - 4, dw + 8, dh + 8), border_radius=6)
        pygame.draw.rect(surface, (85, 85, 85),
                         (dx - 4, dy - 4, dw + 8, dh + 8), 2, border_radius=6)
        pygame.draw.rect(surface, (8, 8, 8),
                         (dx, dy, dw, dh), border_radius=3)

        col_widths = [int(dw * p) for p in [0.22, 0.22, 0.22, 0.34]]
        th    = int(alt) // 10000
        th1   = (int(alt) % 10000) // 1000
        hund  = (int(alt) % 1000) // 100
        hfrac = (alt % 100) / 100.0
        col_x = dx
        mid_y = dy + dh // 2

        t = self.drum_font.render(str(th), True, (232, 160, 32))
        surface.blit(t, t.get_rect(center=(col_x + col_widths[0] // 2, mid_y)))
        col_x += col_widths[0]
        pygame.draw.line(surface, (51, 51, 51), (col_x, dy + 4), (col_x, dy + dh - 4), 1)

        t = self.drum_font.render(str(th1), True, (255, 255, 255))
        surface.blit(t, t.get_rect(center=(col_x + col_widths[1] // 2, mid_y)))
        col_x += col_widths[1]
        pygame.draw.line(surface, (51, 51, 51), (col_x, dy + 4), (col_x, dy + dh - 4), 1)

        drum_surf = pygame.Surface((col_widths[2], dh), pygame.SRCALPHA)
        scroll_px = int(hfrac * dh)
        t0 = self.drum_font.render(str(hund),            True, (255, 255, 255))
        t1 = self.drum_font.render(str((hund + 1) % 10), True, (255, 255, 255))
        drum_surf.blit(t0, t0.get_rect(center=(col_widths[2] // 2, dh // 2 + scroll_px)))
        drum_surf.blit(t1, t1.get_rect(center=(col_widths[2] // 2, dh // 2 + scroll_px - dh)))
        surface.blit(drum_surf, (col_x, dy))
        col_x += col_widths[2]
        pygame.draw.line(surface, (51, 51, 51), (col_x, dy + 4), (col_x, dy + dh - 4), 1)

        t = self.drum_font.render("00", True, (90, 90, 90))
        surface.blit(t, t.get_rect(center=(col_x + col_widths[3] // 2, mid_y)))

        pygame.draw.line(surface, (0, 0, 0), (dx, dy + 3),      (dx + dw, dy + 3),      3)
        pygame.draw.line(surface, (0, 0, 0), (dx, dy + dh - 3), (dx + dw, dy + dh - 3), 3)

        leg = self.legend_font.render("x10k  x1k  x100     x10", True, (85, 85, 85))
        surface.blit(leg, leg.get_rect(center=(cx, dy + dh + 12)))

        needle_angle = math.radians(alt / 1000 * 360 - 90)
        nl   = r * 0.80
        tail = 36
        nx   = math.cos(needle_angle)
        ny   = math.sin(needle_angle)
        px, py = -ny, nx

        tip   = (cx + nx * nl,          cy + ny * nl)
        sl    = (cx + px * 5 + nx * 20, cy + py * 5 + ny * 20)
        bl    = (cx + px * 6,           cy + py * 6)
        t_end = (cx - nx * tail,        cy - ny * tail)
        br    = (cx - px * 6,           cy - py * 6)
        sr    = (cx - px * 5 + nx * 20, cy - py * 5 + ny * 20)

        pygame.draw.polygon(surface, (255, 255, 255), [tip, sl, bl, t_end, br, sr])
        pygame.draw.polygon(surface, (0, 0, 0),       [tip, sl, bl, t_end, br, sr], 1)
        pygame.draw.circle(surface, (136, 136, 136), self.center, 10)
        pygame.draw.circle(surface, (34, 34, 34),    self.center, 10, 1)
        pygame.draw.circle(surface, (85, 85, 85),    self.center, 4)


class FuelGaugeSlider:
    def __init__(self, x, y, width, height):
        self.center = (x, y)
        self.width  = width
        self.height = height
        self.value  = SmoothValue()
        self.font   = pygame.font.SysFont("Arial", 16, bold=True)

    def update(self, target): self.value.update(target)

    def draw(self, surface):
        val   = max(0, min(self.value.current, 100))
        color = ((255, 0, 0) if val < 15
                 else (255, 255, 0) if val < 30
                 else (0, 255, 0))
        bg_rect = pygame.Rect(self.center[0] - self.width // 2,
                              self.center[1] - self.height // 2,
                              self.width, self.height)
        pygame.draw.rect(surface, (40, 40, 40), bg_rect)
        fill_h    = (val / 100.0) * self.height
        fill_rect = pygame.Rect(self.center[0] - self.width // 2,
                                self.center[1] + self.height // 2 - fill_h,
                                self.width, fill_h)
        pygame.draw.rect(surface, color, fill_rect)
        pygame.draw.rect(surface, (150, 150, 150), bg_rect, 3)
        for lbl, y_off in [("F", -self.height // 2 - 5),
                            ("1/2", -8),
                            ("E", self.height // 2 - 15)]:
            t = self.font.render(lbl, True, (255, 255, 255))
            surface.blit(t, (self.center[0] + self.width // 2 + 8,
                              self.center[1] + y_off))
        for y_off in [-self.height // 2, 0, self.height // 2]:
            pygame.draw.line(surface, (200, 200, 200),
                             (self.center[0] + self.width // 2,
                              self.center[1] + y_off),
                             (self.center[0] + self.width // 2 + 5,
                              self.center[1] + y_off), 2)
        unit_txt = UNIT_FONT.render("FUEL", True, (200, 200, 200))
        surface.blit(unit_txt,
                     (self.center[0] - unit_txt.get_width() // 2,
                      self.center[1] + self.height // 2 + 15))


class ThrottleIndicator:
    """Vertical slider showing current throttle position (0.0 – 1.0)."""
    def __init__(self, x, y, width=30, height=200):
        self.center = (x, y)
        self.width  = width
        self.height = height
        self.value  = SmoothValue(smoothing=0.15)
        self.font   = pygame.font.SysFont("Arial", 16, bold=True)
        self.sfont  = pygame.font.SysFont("Arial", 11, bold=True)

    def update(self, target): self.value.update(target)

    def draw(self, surface):
        val   = max(0.0, min(self.value.current, 1.0))
        # colour: green at full, yellow at mid, red at idle
        r = int(255 * (1.0 - val))
        g = int(255 * val)
        color = (r, g, 0)

        bg_rect = pygame.Rect(self.center[0] - self.width // 2,
                              self.center[1] - self.height // 2,
                              self.width, self.height)
        pygame.draw.rect(surface, (40, 40, 40), bg_rect)

        fill_h    = val * self.height
        fill_rect = pygame.Rect(self.center[0] - self.width // 2,
                                self.center[1] + self.height // 2 - fill_h,
                                self.width, fill_h)
        pygame.draw.rect(surface, color, fill_rect)
        pygame.draw.rect(surface, (150, 150, 150), bg_rect, 3)

        for lbl, y_off in [("TO", -self.height // 2 - 5),
                            ("CL", -8),
                            ("ID", self.height // 2 - 15)]:
            t = self.sfont.render(lbl, True, (180, 180, 180))
            surface.blit(t, (self.center[0] + self.width // 2 + 6,
                              self.center[1] + y_off))

        pct_txt = self.font.render(f"{int(val * 100)}%", True, (220, 220, 220))
        surface.blit(pct_txt,
                     (self.center[0] - pct_txt.get_width() // 2,
                      self.center[1] + self.height // 2 + 6))

        lbl = self.sfont.render("THR", True, (120, 120, 120))
        surface.blit(lbl,
                     (self.center[0] - lbl.get_width() // 2,
                      self.center[1] + self.height // 2 + 24))


class YokeIndicator:
    def __init__(self, x, y, size=90):
        self.center = (x, y)
        self.size   = size
        self.roll   = SmoothValue(smoothing=0.25)
        self.pitch  = SmoothValue(smoothing=0.25)
        self.font   = pygame.font.SysFont("Arial", 11, bold=True)

    def update(self, roll, pitch):
        self.roll.update(roll)
        self.pitch.update(pitch)

    def draw(self, surface):
        cx, cy = self.center
        half   = self.size // 2
        box    = pygame.Rect(cx - half, cy - half, self.size, self.size)
        pygame.draw.rect(surface, (20, 20, 20), box, border_radius=6)
        pygame.draw.rect(surface, (70, 70, 70), box, 1, border_radius=6)
        pygame.draw.line(surface, (45, 45, 45),
                         (cx - half + 4, cy), (cx + half - 4, cy), 1)
        pygame.draw.line(surface, (45, 45, 45),
                         (cx, cy - half + 4), (cx, cy + half - 4), 1)
        dot_x = cx + int(self.roll.current  * (half - 8))
        dot_y = cy + int(self.pitch.current * (half - 8))
        dot_x = max(cx - half + 6, min(cx + half - 6, dot_x))
        dot_y = max(cy - half + 6, min(cy + half - 6, dot_y))
        pygame.draw.line(surface, (0, 200, 100), (cx, cy), (dot_x, cy), 1)
        pygame.draw.line(surface, (0, 200, 100), (cx, cy), (cx, dot_y), 1)
        pygame.draw.circle(surface, (0, 220, 110), (dot_x, dot_y), 5)
        pygame.draw.circle(surface, (0, 150,  80), (dot_x, dot_y), 5, 1)
        pygame.draw.circle(surface, (100, 100, 100), (cx, cy), 2)
        lbl = self.font.render("YOKE", True, (120, 120, 120))
        surface.blit(lbl, (cx - lbl.get_width() // 2, cy + half + 4))
        r_txt = self.font.render(f"R {self.roll.current:+.2f}",  True, (80, 180, 80))
        p_txt = self.font.render(f"P {self.pitch.current:+.2f}", True, (80, 180, 80))
        surface.blit(r_txt, (cx - half, cy + half + 17))
        surface.blit(p_txt, (cx + 4,   cy + half + 17))


class RudderIndicator:
    def __init__(self, x, y, width=160, height=34):
        self.center = (x, y)
        self.width  = width
        self.height = height
        self.value  = SmoothValue(smoothing=0.20)
        self.font   = pygame.font.SysFont("Arial", 11, bold=True)

    def update(self, val): self.value.update(val)

    def draw(self, surface):
        cx, cy = self.center
        w, h   = self.width, self.height
        val    = max(-1.0, min(1.0, self.value.current))
        half_w = w // 2
        bg     = pygame.Rect(cx - half_w, cy - h // 2, w, h)
        pygame.draw.rect(surface, (20, 20, 20), bg, border_radius=5)
        pygame.draw.rect(surface, (70, 70, 70), bg, 1, border_radius=5)
        pygame.draw.line(surface, (60, 60, 60),
                         (cx, cy - h // 2 + 4),
                         (cx, cy + h // 2 - 4), 1)
        bar_w = int(abs(val) * (half_w - 6))
        if bar_w > 0:
            bar_col  = (0, 200, 220) if val < 0 else (220, 160, 0)
            bar_rect = (pygame.Rect(cx - bar_w - 1, cy - h // 2 + 4, bar_w, h - 8)
                        if val < 0 else
                        pygame.Rect(cx + 1, cy - h // 2 + 4, bar_w, h - 8))
            pygame.draw.rect(surface, bar_col, bar_rect, border_radius=3)
        tick_x = cx + int(val * (half_w - 6))
        pygame.draw.rect(surface, (255, 255, 255),
                         (tick_x - 2, cy - h // 2 + 2, 4, h - 4),
                         border_radius=2)
        l_txt = self.font.render("L", True, (0, 180, 200))
        r_txt = self.font.render("R", True, (200, 140, 0))
        surface.blit(l_txt, (cx - half_w - 14, cy - l_txt.get_height() // 2))
        surface.blit(r_txt, (cx + half_w + 4,  cy - r_txt.get_height() // 2))
        lbl = self.font.render("RUDDER", True, (120, 120, 120))
        surface.blit(lbl, (cx - lbl.get_width() // 2, cy + h // 2 + 4))
        num = self.font.render(f"{val:+.2f}", True, (120, 180, 120))
        surface.blit(num, (cx - num.get_width() // 2, cy + h // 2 + 17))


# ============================================================
#  NETWORKING
# ============================================================
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
try:
    sock.bind((PI_IP, PI_PORT))
    sock.setblocking(False)
    print(f"[NET] Listening on {PI_IP}:{PI_PORT}")
except Exception as e:
    print(f"[NET] Bind error: {e}")

send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
send_sock.setblocking(False)

# ============================================================
#  INSTRUMENT LAYOUT
# ============================================================
horizon      = ArtificialHorizon (650, 250, 150)
heading      = HeadingIndicator  (650, 550, 120)
airspeed     = AirspeedIndicator (250, 250, 120)
altimeter    = DetailedAltimeter (1050, 250, 120)
fuel         = FuelGaugeSlider   (450, 250, 30, 200)
throttle_ind = ThrottleIndicator (200, 550, 30, 200)  
yoke_ind     = YokeIndicator     (100, 490, 110)
rudder_ind   = RudderIndicator   (100, 630, 160, 34)

instruments = [horizon, airspeed, heading, altimeter, fuel, throttle_ind]

data_vals = [0.0, 0.0, 0.0, 0.0, 0.0, 100.0]

# ============================================================
#  MAIN LOOP
# ============================================================
running = True
while running:

    # ── Events ───────────────────────────────────────────────
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        if event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
            running = False
        if event.type == pygame.JOYDEVICEADDED:
            _refresh_joystick_pool()
            _assign_devices()

    # ── 1. Receive telemetry from C++ ─────────────────────────
    while True:
        try:
            message, _ = sock.recvfrom(1024)
            new_data = [float(x) for x in message.decode('utf-8').split(',')]
            if len(new_data) >= 6:
                data_vals = new_data
        except BlockingIOError:
            break
        except Exception as e:
            print(f"[NET] Receive error: {e}")
            break



    # ── 2. Update instruments ─────────────────────────────────
    p, r, h, a, s, f = data_vals[:6]
    s = s * 1.94384  # m/s to knots
    horizon.update(p, r)
    airspeed.update(s)
    heading.update(h)
    altimeter.update(a)
    fuel.update(f)
    # In the main loop, just after throttle_val = read_throttle()


    # ── 3. Read controls ──────────────────────────────────────
    pygame.event.pump()
    yoke_roll, yoke_pitch = read_yoke()
    rudder_val            = read_rudder()
    throttle_val          = read_throttle()       # MCP3008 + 10kΩ pot
    yoke_ind.update(yoke_roll, yoke_pitch)
    rudder_ind.update(rudder_val)
    throttle_ind.update(throttle_val)

    # ── 4. Read GPIO ──────────────────────────────────────────
    left_brake, right_brake = read_toe_brakes()
    brake_active = 1 if (left_brake > 0.1 or right_brake > 0.1) else 0

    flap_full = GPIO.input(FLAP_FULL_PIN) == GPIO.LOW
    flap_half = GPIO.input(FLAP_HALF_PIN) == GPIO.LOW
    flap_val  = 2 if flap_full else (1 if flap_half else 0)

    # ── 5. Draw ───────────────────────────────────────────────
    screen.fill((30, 30, 30))
    for inst in instruments:
        inst.draw(screen)
    yoke_ind.draw(screen)
    rudder_ind.draw(screen)

    # ── 6. Status labels ──────────────────────────────────────
    f_lbl = ("FLAPS: UP"   if flap_val == 0
             else "FLAPS: 1/2" if flap_val == 1
             else "FLAPS: FULL")
    f_col = (0, 255, 0) if flap_val > 0 else (150, 150, 150)
    screen.blit(FONT.render(f_lbl, True, f_col), (50, 50))

    b_lbl = "BRAKES: ON" if brake_active else "BRAKES: OFF"
    b_col = (255, 0, 0)  if brake_active else (150, 150, 150)
    screen.blit(FONT.render(b_lbl, True, b_col), (50, 80))

    yoke_badge = SMALL_FONT.render(
        f"YOKE: {yoke_name[:22]}" if yoke_connected else "YOKE: NOT FOUND",
        True, (0, 180, 80) if yoke_connected else (180, 40, 40))
    screen.blit(yoke_badge, (10, 10))

    rud_badge = SMALL_FONT.render(
        f"RUDDER: {rudder_name[:22]}" if rudder_connected else "RUDDER: NOT FOUND",
        True, (0, 180, 80) if rudder_connected else (180, 40, 40))
    screen.blit(rud_badge, (10, 30))

    # ── 7. Transmit inputs to PC ──────────────────────────────
    # Format: aileron, elevator, rudder, throttle, brake, flaps
    out_msg = (f"{yoke_roll:.4f},{yoke_pitch:.4f},{rudder_val:.4f},"
               f"{throttle_val:.4f},{brake_active},{flap_val}")
    try:
        send_sock.sendto(out_msg.encode('utf-8'), (COMPUTER_IP, COMPUTER_PORT))
    except Exception:
        pass

    # ── 8. Render display ─────────────────────────────────────
    pygame.display.flip()
    clock.tick(60)

# ============================================================
#  EXIT ROUTINE
# ============================================================
adc.close()
GPIO.cleanup()
pygame.quit()
sys.exit()