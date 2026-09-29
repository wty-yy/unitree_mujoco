import numpy as np
from PIL import Image
from pathlib import Path

TERRAIN_SIZE = (10.0, 10.0)
HORIZONTAL_SCALE = 0.02
VERTICAL_SCALE = 0.00025
DOWNSAMPLED_SCALE = 0.05
NOISE_RANGE = (0.0, 0.002)
NOISE_STEP = 0.0005
SEED = 50

OUTPUT_PATH = Path(__file__).resolve().parent.parent / "unitree_robots" / "go2" / "assets" / "rough_flat_hfield.png"


def random_uniform_heights(size, horizontal_scale, vertical_scale, downsampled_scale, noise_range, noise_step, seed):
    rng = np.random.default_rng(seed)
    width_pixels = int(size[0] / horizontal_scale)
    length_pixels = int(size[1] / horizontal_scale)
    width_downsampled = int(size[0] / downsampled_scale)
    length_downsampled = int(size[1] / downsampled_scale)
    height_min = int(noise_range[0] / vertical_scale)
    height_max = int(noise_range[1] / vertical_scale)
    height_step = int(noise_step / vertical_scale)

    height_range = np.arange(height_min, height_max + height_step, height_step)
    coarse = rng.choice(height_range, size=(width_downsampled, length_downsampled)).astype(np.float32)
    fine = Image.fromarray(coarse, mode="F").resize((width_pixels, length_pixels), Image.BICUBIC)
    heights = np.rint(np.asarray(fine, dtype=np.float32)) * vertical_scale
    return np.clip(heights, noise_range[0], noise_range[1])


def main():
    heights = random_uniform_heights(
        TERRAIN_SIZE,
        HORIZONTAL_SCALE,
        VERTICAL_SCALE,
        DOWNSAMPLED_SCALE,
        NOISE_RANGE,
        NOISE_STEP,
        SEED,
    )
    image = np.rint(heights / NOISE_RANGE[1] * 255.0).clip(0, 255).astype(np.uint8)

    OUTPUT_PATH.parent.mkdir(parents=True, exist_ok=True)
    Image.fromarray(image, mode="L").save(OUTPUT_PATH)

    print(f"terrain size:      {TERRAIN_SIZE[0]} x {TERRAIN_SIZE[1]} m")
    print(f"image resolution:  {image.shape[1]} x {image.shape[0]} px ({HORIZONTAL_SCALE} m/px)")
    print(f"noise range:       {NOISE_RANGE[0]} ~ {NOISE_RANGE[1]} m")
    print(f"noise step:        {NOISE_STEP} m")
    print(f"downsampled scale: {DOWNSAMPLED_SCALE} m")
    print(f"height in image:   {heights.min():.4f} ~ {heights.max():.4f} m")
    print(f"hfield elevation_z must be {NOISE_RANGE[1]} m in rough_flat.xml")
    print(f"written:           {OUTPUT_PATH}")


if __name__ == "__main__":
    main()
