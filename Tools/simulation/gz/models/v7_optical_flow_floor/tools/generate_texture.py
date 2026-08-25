#!/usr/bin/env python3
"""Generate the deterministic multi-scale optical-flow floor texture."""

from pathlib import Path
import random

from PIL import Image, ImageDraw


SIZE = 1024
OUTPUT = Path(__file__).resolve().parents[1] / "materials" / "textures" / "optical_flow_floor.png"


def main() -> None:
    rng = random.Random(0x483F10)
    image = Image.new("L", (SIZE, SIZE), 210)
    draw = ImageDraw.Draw(image)

    # Irregular 32 px cells provide stable texture without checkerboard ambiguity.
    cell = 32
    for y in range(0, SIZE, cell):
        for x in range(0, SIZE, cell):
            shade = rng.choice((25, 45, 70, 150, 190, 225))
            draw.rectangle((x, y, x + cell - 1, y + cell - 1), fill=shade)

    # Features at two smaller scales keep flow observable close to the floor.
    for width, count in ((8, 1800), (3, 3200)):
        for _ in range(count):
            x = rng.randrange(0, SIZE - width)
            y = rng.randrange(0, SIZE - width)
            shade = rng.choice((8, 245))
            draw.rectangle((x, y, x + width, y + width), fill=shade)

    # A sparse set of oriented bars breaks rotational and translational symmetry.
    for _ in range(260):
        x = rng.randrange(0, SIZE - 48)
        y = rng.randrange(0, SIZE - 12)
        shade = rng.choice((0, 255))
        draw.rectangle((x, y, x + rng.randrange(18, 48), y + rng.randrange(3, 12)), fill=shade)

    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    image.convert("RGB").save(OUTPUT, optimize=True)
    print(OUTPUT)


if __name__ == "__main__":
    main()
