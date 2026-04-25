"""
Generate printable QR code images for all navigation commands.

Usage:
    pip install qrcode[pil]
    python tools/generate_qr_codes.py --out qr_codes/

Output: one PNG per command (e.g. TURN_LEFT.png) at 300 dpi, suitable for
printing on A4 and mounting on walls/stands for the demo course.
"""

import argparse
import os

COMMANDS = [
    'TURN_LEFT',
    'TURN_RIGHT',
    'STOP',
    'GO',
    'SPEED_UP',
    'SPEED_DOWN',
    'U_TURN',
    'AND_TURN_LEFT',
    'AND_TURN_RIGHT',
    'AND_STOP',
    'AND_GO',
    'AND_SPEED_UP',
    'AND_SPEED_DOWN',
    'AND_U_TURN',
]


def generate(out_dir: str, box_size: int = 10, border: int = 4):
    try:
        import qrcode
    except ImportError:
        raise SystemExit('Install qrcode first:  pip install "qrcode[pil]"')

    os.makedirs(out_dir, exist_ok=True)
    for cmd in COMMANDS:
        qr = qrcode.QRCode(
            version=1,
            error_correction=qrcode.constants.ERROR_CORRECT_H,
            box_size=box_size,
            border=border,
        )
        qr.add_data(cmd)
        qr.make(fit=True)
        img = qr.make_image(fill_color='black', back_color='white')
        path = os.path.join(out_dir, f'{cmd}.png')
        img.save(path)
        print(f'Saved: {path}')
    print(f'\n{len(COMMANDS)} QR codes written to {out_dir}/')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Generate QR code PNGs for all navigation commands.')
    parser.add_argument('--out',      default='qr_codes',  help='Output directory')
    parser.add_argument('--box-size', default=10, type=int, help='Pixels per QR box')
    parser.add_argument('--border',   default=4,  type=int, help='QR quiet zone (boxes)')
    args = parser.parse_args()
    generate(args.out, args.box_size, args.border)
