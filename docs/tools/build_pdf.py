import markdown, re, pathlib

SRC = pathlib.Path("/mnt/user-data/uploads/pluvio/products/pluvio-grow/open-build/docs/_INSTRUCTABLE_for_stage.md")
raw = SRC.read_text()

# ---- 1) produce a CLEANED markdown (repo-relative paths) to write back to the repo ----
md = raw
# drop the stale DRAFT blockquote
md = re.sub(r"(?ms)^> \*\*DRAFT for Instructables.*?(?:\n\n|\Z)", "", md, count=1)
# intro hero render -> real deployed prototype photo
md = md.replace("![Pluvio Grow — the finished build](../hardware/hero.png)",
                "![Pluvio Grow — 13-plant build, deployed](../photos/IMG_1336.jpg)")
# drop the intro placeholder caption (the cover already shows the finished build)
md = md.replace("\n*(Photo: the prototype on the shelf. Photo: quality-control cat inspecting it.)*\n", "\n")
# drop the rejected cartoon wiring image; wiring is covered by the written steps
md = md.replace("\n![Wiring diagram](../hardware/wiring/wiring_diagram.png)\n", "\n")
# put a real assembled-unit photo where it helps: the assembly step
md = md.replace(
    "with the top-hat grommets. *(Photos per the exploded view.)*",
    "with the top-hat grommets.\n\n![The assembled carousel — 13 weighed metering vessels](../photos/IMG_0996.jpg)")
pathlib.Path("/home/claude/oss/INSTRUCTABLE_clean.md").write_text(md)

# ---- 2) build the PDF HTML from that cleaned markdown, pointing images at local img/ ----
body_md = md
body_md = body_md.replace("../photos/IMG_1336.jpg", "img/cover.jpg")      # (not used in body; cover is separate)
body_md = body_md.replace("![Pluvio Grow — 13-plant build, deployed](img/cover.jpg)", "")  # intro hero shown on cover
body_md = body_md.replace("../photos/IMG_0996.jpg", "img/finished.jpg")
body_md = body_md.replace("../hardware/parts_overview.png", "img/parts.png")
body = markdown.markdown(body_md, extensions=["tables", "fenced_code", "sane_lists"])

CSS = """
@page { size: A4; margin: 16mm 15mm; }
@import url('https://fonts.googleapis.com/css2?family=Space+Grotesk:wght@400;500;600;700&family=Fraunces:ital,wght@1,500;0,600&family=JetBrains+Mono:wght@500;700&display=swap');
:root{--bg:#f4f1ec;--ink:#1a1a1a;--soft:#6b6864;--orange:#c46f02;--water:#4a90c2;--rule:#e3ddd2;}
*{box-sizing:border-box}
body{background:var(--bg);color:var(--ink);font-family:'Space Grotesk',system-ui,sans-serif;font-size:10.5pt;line-height:1.5;margin:0;-webkit-print-color-adjust:exact;print-color-adjust:exact;}
h1,h2,h3{font-family:'Fraunces',Georgia,serif;font-weight:600;color:var(--ink);line-height:1.15;}
h1{font-size:26pt;margin:0 0 2pt;}
h2{font-size:16pt;margin:20pt 0 6pt;border-top:2px solid var(--rule);padding-top:10pt;}
h3{font-size:12.5pt;margin:12pt 0 4pt;}
a{color:var(--orange);text-decoration:none;}
code{font-family:'JetBrains Mono',monospace;font-size:9pt;background:#eae4d8;padding:1px 4px;border-radius:3px;}
pre{background:#2a251e;color:#f4f1ec;padding:10px 12px;border-radius:8px;font-family:'JetBrains Mono',monospace;font-size:8.5pt;overflow-x:auto;}
pre code{background:none;color:inherit;padding:0;}
img{max-width:100%;border-radius:10px;display:block;margin:10pt auto;}
table{width:100%;border-collapse:collapse;margin:10pt 0;font-size:9pt;}
th{background:var(--orange);color:#fff;text-align:left;padding:6px 8px;font-family:'JetBrains Mono',monospace;font-size:8pt;letter-spacing:.04em;}
td{padding:6px 8px;border-bottom:1px solid var(--rule);vertical-align:top;}
tr:nth-child(even) td{background:rgba(255,255,255,.4);}
blockquote{background:rgba(196,111,2,.07);border-left:3px solid var(--orange);margin:10pt 0;padding:8pt 12pt;border-radius:0 8px 8px 0;font-size:9.5pt;color:#4a3f30;}
blockquote strong{color:var(--orange);}
hr{border:none;border-top:2px solid var(--rule);margin:16pt 0;}
ul,ol{padding-left:18px;}
li{margin:3pt 0;}
.cover{page-break-after:always;text-align:center;padding-top:6mm;}
.cover img{width:86%;margin:6mm auto;border-radius:14px;box-shadow:0 8px 24px rgba(0,0,0,.18);}
.cover .tag{font-family:'JetBrains Mono',monospace;color:var(--orange);letter-spacing:.14em;text-transform:uppercase;font-size:10pt;}
.cover .eyebrow{font-family:'JetBrains Mono',monospace;color:var(--soft);letter-spacing:.2em;text-transform:uppercase;font-size:9pt;margin-top:2mm;}
.cover h1{font-size:30pt;margin-top:3mm;}
"""
cover = """<div class="cover">
<div class="eyebrow">Open Source · Build it yourself</div>
<h1>Pluvio Grow — M5 Build</h1>
<div class="tag">Rain, with a brain.</div>
<img src="img/cover.jpg">
<div style="font-family:'JetBrains Mono',monospace;color:#6b6864;font-size:9pt;">A 13-plant, weighed-dose plant waterer from M5Stack parts<br>pluviogrow.com · a product of Powell Electronic Inc.</div>
</div>"""
html = f"<!doctype html><html><head><meta charset='utf-8'><style>{CSS}</style></head><body>{cover}{body}</body></html>"
pathlib.Path("/home/claude/oss/pdfbuild/instructable.html").write_text(html)
print("html built", len(html), "bytes")
