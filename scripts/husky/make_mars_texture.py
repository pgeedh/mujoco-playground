"""Builds the Mars ground texture from a real NASA photo.

Source: NASA/JPL-Caltech/MSSS, PIA16018 "Gravel-Covered Martian Surface" (Curiosity's Mars Descent
Imager, minutes after landing). NASA media are free to use with credit. The crop is flat-fielded
(removes the lander's shadow bands and vignette), tinted toward Mars' butterscotch, and mirrored 2x2
so it tiles without seams."""
import numpy as np
from PIL import Image
from scipy.ndimage import gaussian_filter

im = Image.open("scripts/husky/data/PIA16018.jpg").convert("RGB")
crop = np.asarray(im.crop((700, 480, 1260, 1040)).resize((512, 512), Image.LANCZOS)).astype(float) / 255
lum = crop.mean(axis=2, keepdims=True)
flat = crop / (gaussian_filter(lum, (50, 50, 0)) + 0.02) * 0.40      # remove low-frequency lighting
detail = flat - gaussian_filter(flat, (8, 8, 0))                       # fine gravel contrast
out = np.clip((flat + 0.7 * detail) * np.array([1.10, 0.88, 0.70]), 0, 1)

# Make it tile without a mirror pattern: cross-fade the image with a half-shifted copy of itself
# using a window that is 0 at the borders (the shifted copy is continuous across the seam).
n = out.shape[0]
w1 = np.sin(np.linspace(0, np.pi, n)) ** 2
w = (w1[:, None] * w1[None, :])[:, :, None]
shifted = np.roll(np.roll(out, n // 2, axis=0), n // 2, axis=1)
tile = out * w + shifted * (1 - w)
Image.fromarray((tile * 255).astype("uint8")).save("web/assets/scenes/rover_mars/assets/mars_ground.png", optimize=True)
print("texture", tile.shape, "mean", tile.mean(axis=(0, 1)).round(2))
