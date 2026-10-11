# Build-guide PDF tooling

Regenerates `../Pluvio_M5_Build_Guide.pdf` from `../INSTRUCTABLE.md`. Runs in the
cloud workspace (needs Python `markdown` + Playwright/Chromium), mirroring the
marketing video pipeline convention.

- `build_pdf.py` — reads the Instructable markdown, strips internal notes, points
  images at a local `img/` dir (cover = a real deployed photo, parts render,
  an assembled-unit photo under Step 3), writes `instructable.html`.
- `render.cjs` — Chromium prints the HTML to A4 PDF:
  `node render.cjs <input.html> <output.pdf>`

Cover photo: `photos/IMG_1336.jpg` · assembly photo: `photos/IMG_0996.jpg` ·
parts render: `hardware/parts_overview.png`.
