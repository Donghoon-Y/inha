"""Stand-alone FUNcube-1 (AO-73) telemetry parser.

No dependency on satellites.telemetry.funcube or construct.
Reads a .kiss file, parses each 256-byte payload bit-by-bit, emits CSV.

Frame layout (FC1 / AO-73):
  byte  0       header: satid (2 bits) + frametype (6 bits)
  bytes 1..55   realtime telemetry (BitStruct: eps/bob/rf/pa/ants/sw, total 440 bits)
  bytes 56..255 payload, 200 bytes (interpretation depends on frametype prefix)
"""
import sys, csv, json, os
from collections.abc import Mapping

# ─── KISS framing ────────────────────────────────────────────────────────────
FEND, FESC, TFEND, TFESC = 0xC0, 0xDB, 0xDC, 0xDD


def kiss_frames(buf: bytes):
    out, cur, in_frame = [], bytearray(), False
    i = 0
    while i < len(buf):
        b = buf[i]
        if b == FEND:
            if in_frame and cur:
                out.append(bytes(cur))
            cur, in_frame = bytearray(), True
        elif b == FESC and i + 1 < len(buf):
            nxt = buf[i + 1]
            if nxt == TFEND: cur.append(FEND); i += 1
            elif nxt == TFESC: cur.append(FESC); i += 1
            else: cur.append(b)
        else:
            if in_frame: cur.append(b)
        i += 1
    return out


# ─── BitReader: pull N bits at a time, MSB-first ─────────────────────────────
class BitReader:
    def __init__(self, data: bytes):
        self.data = data
        self.bitpos = 0

    def read(self, n: int) -> int:
        """Return next n bits as unsigned int (MSB-first)."""
        v = 0
        for _ in range(n):
            byte_idx = self.bitpos >> 3
            bit_off  = 7 - (self.bitpos & 7)
            v = (v << 1) | ((self.data[byte_idx] >> bit_off) & 1)
            self.bitpos += 1
        return v

    def flag(self) -> bool:
        return bool(self.read(1))

    def repeat(self, n_items: int, bits_each: int, decode=None):
        out = []
        for _ in range(n_items):
            v = self.read(bits_each)
            out.append(decode(v) if decode else v)
        return out


# ─── Enums ───────────────────────────────────────────────────────────────────
SATID = {0: 'FC1EM', 1: 'FC2', 2: 'FC1FM', 3: 'extended'}

FRAMETYPE_FC1 = {
    0: 'WO1', 1: 'WO2', 2: 'WO3', 3: 'WO4', 4: 'WO5', 5: 'WO6',
    6: 'WO7', 7: 'WO8', 8: 'WO9', 9: 'WO10', 10: 'WO11', 11: 'WO12',
    12: 'HR1', 13: 'FM1', 14: 'FM2', 15: 'FM3',
    16: 'HR2', 17: 'FM4', 18: 'FM5', 19: 'FM6',
    20: 'HR3', 21: 'FM7', 22: 'FM8', 23: 'FM9',
}


# ─── Adapters (raw int → physical unit) ──────────────────────────────────────
def temp_xp(x): return -0.2073 * x + 158.239
def temp_xm(x): return -0.2083 * x + 159.227
def temp_yp(x): return -0.2076 * x + 158.656
def temp_ym(x): return -0.2087 * x + 159.045
def v3v3(x):    return 4 * x       # mV
def v5v(x):     return 6 * x       # mV
def rf_temp(x): return -0.857 * x + 193.672
def rx_curr(x): return 0.636 * x
def tx_curr(x): return 1.272 * x
def pwr(x):     return 5e-3 * (x ** 2.0629)
def pa_curr(x): return 0.5496 * x + 2.5435


# ─── Per-block decoders ──────────────────────────────────────────────────────
def parse_eps(br: BitReader) -> dict:
    return {
        'photovoltage':       br.repeat(3, 16),
        'photocurrent':       br.read(16),
        'batteryvoltage':     br.read(16),
        'systemcurrent':      br.read(16),
        'rebootcount':        br.read(16),
        'softwareerrors':     br.read(16),
        'boostconvertertemp': br.repeat(3, 8),
        'batterytemp':        br.read(8),
        'latchupcount5v':     br.read(8),
        'latchupcount3v3':    br.read(8),
        'resetcause':         br.read(8),
        'MPPTmode':           br.read(8),
    }


def parse_bob(br: BitReader) -> dict:
    return {
        'sunsensor':   br.repeat(3, 10),
        'paneltempX+': temp_xp(br.read(10)),
        'paneltempX-': temp_xm(br.read(10)),
        'paneltempY+': temp_yp(br.read(10)),
        'paneltempY-': temp_ym(br.read(10)),
        '3v3voltage':  v3v3(br.read(10)),
        '3v3current':  br.read(10),
        '5voltage':    v5v(br.read(10)),
    }


def parse_rf(br: BitReader) -> dict:
    return {
        'rxdoppler':    br.read(8),
        'rxrssi':       br.read(8),
        'temp':         rf_temp(br.read(8)),
        'rxcurrent':    rx_curr(br.read(8)),
        'tx3v3current': rx_curr(br.read(8)),
        'tx5vcurrent':  tx_curr(br.read(8)),
    }


def parse_pa(br: BitReader) -> dict:
    return {
        'revpwr':    pwr(br.read(8)),
        'fwdpwr':    pwr(br.read(8)),
        'boardtemp': br.read(8),
        'boardcurr': pa_curr(br.read(8)),
    }


def parse_ants(br: BitReader) -> dict:
    return {
        'temp':       br.repeat(2, 8),
        'deployment': [br.flag() for _ in range(4)],
    }


def parse_sw(br: BitReader) -> dict:
    return {
        'seqnumber':      br.read(24),
        'dtmfcmdcount':   br.read(6),
        'dtmflastcmd':    br.read(5),
        'dtmfcmdsuccess': br.flag(),
        'datavalid':      [br.flag() for _ in range(7)],
        'eclipse':        br.flag(),
        'safemode':       br.flag(),
        'hwabf':          br.flag(),
        'swabf':          br.flag(),
        'deploymentwait': br.flag(),
    }


def parse_realtime_fc1(payload54: bytes) -> dict:
    br = BitReader(payload54)
    return {
        'eps':  parse_eps(br),
        'bob':  parse_bob(br),
        'rf':   parse_rf(br),
        'pa':   parse_pa(br),
        'ants': parse_ants(br),
        'sw':   parse_sw(br),
    }


def parse_header(byte: int):
    satid_raw = (byte >> 6) & 0x3
    ftype_raw = byte & 0x3F
    return SATID.get(satid_raw, f'sat{satid_raw}'), \
           FRAMETYPE_FC1.get(ftype_raw, f'ft{ftype_raw}')


def parse_frame(payload256: bytes) -> dict:
    """Parse a single 256-byte FUNcube payload (header + realtime + 200B payload)."""
    if len(payload256) != 256:
        return {'error': f'unexpected length {len(payload256)}'}
    satid, ftype = parse_header(payload256[0])
    realtime = parse_realtime_fc1(payload256[1:56])  # 55 bytes (440 bits)
    return {'satid': satid, 'frametype': ftype, 'realtime': realtime}


# ─── Serialization helpers ───────────────────────────────────────────────────
def flatten(d, prefix=''):
    flat = {}
    if isinstance(d, Mapping):
        for k, v in d.items():
            flat.update(flatten(v, f'{prefix}.{k}' if prefix else k))
    elif isinstance(d, list):
        for i, v in enumerate(d):
            flat.update(flatten(v, f'{prefix}[{i}]'))
    else:
        flat[prefix] = d
    return flat


def load_config(config_path: str):
    with open(config_path, 'r', encoding='utf-8') as f:
        cfg = json.load(f)
    base = os.path.dirname(os.path.abspath(config_path))
    kiss_in = os.path.normpath(os.path.join(base, cfg['kiss_in']))
    csv_out = os.path.normpath(os.path.join(base, cfg['csv_out']))
    return kiss_in, csv_out


def main():
    config_path = sys.argv[1] if len(sys.argv) >= 2 else \
        os.path.join(os.path.dirname(os.path.abspath(__file__)), 'config.json')
    kiss_path, out_csv = load_config(config_path)
    if not out_csv.lower().endswith('.csv'):
        out_csv += '.csv'
    os.makedirs(os.path.dirname(out_csv), exist_ok=True)
    raw = open(kiss_path, 'rb').read()
    payloads = [f[1:] for f in kiss_frames(raw)
                if f and f[0] == 0x00 and len(f) == 257]

    records = []
    for idx, p in enumerate(payloads):
        rec = {'frame_index': idx, **parse_frame(p)}
        records.append(rec)

    fixed = ['frame_index', 'satid', 'frametype']
    flats, keys = [], set()
    for r in records:
        flat = {c: r.get(c) for c in fixed}
        if isinstance(r.get('realtime'), dict):
            flat.update(flatten(r['realtime']))
        if 'error' in r:
            flat['error'] = r['error']
        flats.append(flat)
        keys.update(flat.keys())

    keys = fixed + sorted(k for k in keys if k not in fixed)
    with open(out_csv, 'w', newline='', encoding='utf-8') as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        for row in flats:
            w.writerow({k: row.get(k, '') for k in keys})

    print(f'parsed {len(records)} frames -> {out_csv}')
    print('frametype counts:',
          dict(sorted({r.get('frametype','?'): sum(1 for x in records if x.get('frametype')==r.get('frametype'))
                       for r in records}.items())))


if __name__ == '__main__':
    main()
