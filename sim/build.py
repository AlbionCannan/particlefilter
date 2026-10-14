# Builds the single-file simulator by inlining the verified scripts into the page.
import pathlib
here = pathlib.Path(__file__).parent
html = (here / "index_template.html").read_text()
for marker, name in [("/*__MCL_PORT__*/", "mcl_port.js"), ("/*__SIM_CORE__*/", "sim_core.js")]:
    src = (here / name).read_text()
    assert "</script" not in src
    assert marker in html
    html = html.replace(marker, src)
out = here / "vex_mcl_simulator.html"
out.write_text(html)
print(f"wrote {out} ({len(html):,} bytes)")