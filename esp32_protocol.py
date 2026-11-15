"""Jetson <-> ESP32 line protocol, DRAFT v0 (electrical has not reviewed it). No ROS imports.

One ASCII line per frame, ending in '\n':
  request (Jetson -> ESP32):  <seq>,<VERB>[,<arg>...]*<CK>
  reply   (ESP32 -> Jetson):  <seq>,OK[,<value>...]*<CK>    or    <seq>,ERR,<reason>*<CK>
  event   (ESP32, unsolicited): 0,EVT,<name>[,<value>...]*<CK>
<CK> is two uppercase hex digits: the XOR of every character before the '*'.
seq runs 1..65535 and wraps; 0 is reserved for events. A frame is at most MAX_FRAME bytes.
A port is an opaque token [A-Za-z0-9_]+ that the ESP32 firmware maps to a pin or channel.

Verbs (args -> reply values):
  PING                              -> firmware version string
  HB                                -> (nothing). Feeds the watchdog.
  STOP [port...]                    -> (nothing). Stop these motors now; none listed = all motors.
  MOT  port,permille [port,permille...]   speed, -1000..1000 (1000 = full speed). Closed loop on the ESP32.
  TURN port,degrees,permille [...]  turn the output shaft RELATIVE to now by `degrees` at |permille| speed
  GOTO port,degrees,permille [...]  turn to ABSOLUTE `degrees` (0 = the encoder's zero) at |permille| speed
  SRV  port,degrees [port,degrees...]     servo to this absolute angle
  DOUT port,0|1                     -> (nothing)
  DIN  port                         -> 0 or 1
  ENC  port [port...]               -> cumulative encoder ticks, one value per port
  ENCZ port [port...]               -> (nothing). Zero the encoder count.
  RNG  port                         -> distance in millimeters

Firmware requirements (these are what make the Jetson side safe):
  1. Silently drop any frame with a bad checksum.
  2. If no VALID frame arrives for watchdog_timeout (0.5 s), stop every motor and send EVT,watchdog.
  3. If a request repeats the previous seq, resend the previous reply; do NOT run the command again.
     (The Jetson retries on a lost reply, and TURN must not turn twice.)
  4. Refuse the whole frame with ERR (unknown_port, wrong_port_kind, bad_args, range) and change nothing.
"""
import math
import re
from dataclasses import dataclass

MAX_FRAME = 120
EVENT_SEQ = 0
MAX_SEQ = 65535
_TOKEN = re.compile(r'^[A-Za-z0-9_]+$')
_ARG = re.compile(r'^[^,*\r\n]*$')


class ProtocolError(ValueError):
  """A frame could not be built or parsed."""


@dataclass(frozen=True)
class Frame:
  seq: int
  verb: str
  args: tuple[str, ...] = ()


def check_token(token: str) -> str:
  """Return token if it is a legal port name, else raise ProtocolError."""
  if not isinstance(token, str) or not _TOKEN.match(token):
    raise ProtocolError(f'bad port name {token!r}: use letters, digits and underscore only')
  return token


def checksum(body: str) -> int:
  value = 0
  for byte in body.encode('ascii'):
    value ^= byte
  return value


def format_number(value) -> str:
  """Shortest text for a number: 90.0 -> '90', 0.5 -> '0.5', -0.001 -> '0'."""
  if isinstance(value, bool):
    return '1' if value else '0'
  if isinstance(value, int):
    return str(value)
  if not math.isfinite(value):
    raise ProtocolError(f'cannot send non-finite number {value!r}')
  text = f'{value:.2f}'.rstrip('0').rstrip('.')
  return '0' if text in ('', '-0') else text


def encode(seq: int, verb: str, *args) -> bytes:
  """Build one frame. Args may be str, int or float."""
  if not 0 <= seq <= MAX_SEQ:
    raise ProtocolError(f'seq {seq} out of range')
  if not verb.isalpha() or not verb.isupper():
    raise ProtocolError(f'bad verb {verb!r}')
  parts = [str(seq), verb]
  for arg in args:
    text = arg if isinstance(arg, str) else format_number(arg)
    if not _ARG.match(text):
      raise ProtocolError(f'argument {text!r} contains a comma, asterisk or newline')
    parts.append(text)
  body = ','.join(parts)
  frame = f'{body}*{checksum(body):02X}\n'.encode('ascii')
  if len(frame) > MAX_FRAME:
    raise ProtocolError(f'frame is {len(frame)} bytes; the limit is {MAX_FRAME}')
  return frame


def decode(line) -> Frame:
  """Parse one received line (bytes or str). Raises ProtocolError if it is damaged."""
  try:
    text = (line.decode('ascii') if isinstance(line, (bytes, bytearray)) else line).strip()
  except UnicodeDecodeError as exc:
    raise ProtocolError('not ASCII') from exc
  body, star, ck = text.rpartition('*')
  if not star or len(ck) != 2:
    raise ProtocolError('missing checksum')
  try:
    expected = int(ck, 16)
  except ValueError as exc:
    raise ProtocolError('bad checksum digits') from exc
  if checksum(body) != expected:
    raise ProtocolError('checksum mismatch')
  fields = body.split(',')
  if len(fields) < 2:
    raise ProtocolError('frame needs a seq and a verb')
  try:
    seq = int(fields[0])
  except ValueError as exc:
    raise ProtocolError('seq is not a number') from exc
  return Frame(seq, fields[1], tuple(fields[2:]))
