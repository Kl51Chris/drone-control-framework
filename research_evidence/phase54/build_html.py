"""Convert the Phase 5.4 Markdown report into one self-contained local HTML file.

Usage: .venv/bin/python research_evidence/phase54/build_html.py <report.md> <report.html>
Supports the subset used in the report: headings, paragraphs, bullet/numbered lists,
pipe tables, images (embedded as base64), links, **bold**, `code`, fenced code.
"""
import base64
import html
import re
import sys
from pathlib import Path

CSS = """
:root{--bg:#ffffff;--fg:#1d2327;--muted:#5d666d;--line:#e2e5e8;--card:#f6f7f8;--accent:#00866A;--bad:#C0392B}
@media (prefers-color-scheme:dark){:root:not([data-theme=light]){--bg:#15191c;--fg:#e7eaec;--muted:#a3abb1;--line:#2c3338;--card:#1d2226;--accent:#3fb894;--bad:#ff7a6b}}
:root[data-theme=dark]{--bg:#15191c;--fg:#e7eaec;--muted:#a3abb1;--line:#2c3338;--card:#1d2226;--accent:#3fb894;--bad:#ff7a6b}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--fg);font:15px/1.55 system-ui,-apple-system,Segoe UI,sans-serif}
main{max-width:980px;margin:0 auto;padding:24px 16px 64px}h1{font-size:1.7rem;margin:.2em 0 .6em}
h2{font-size:1.25rem;margin:2em 0 .6em;padding-top:.6em;border-top:1px solid var(--line)}h3{font-size:1.05rem;margin:1.4em 0 .4em}
a{color:var(--accent)}code{font:13px ui-monospace,Menlo,monospace;background:var(--card);padding:1px 4px;border-radius:4px}
pre{background:var(--card);padding:12px;border-radius:6px;overflow-x:auto}pre code{background:none;padding:0}
.tbl{overflow-x:auto;margin:.8em 0}table{border-collapse:collapse;font-size:13px;min-width:100%}
th,td{border-bottom:1px solid var(--line);padding:5px 8px;text-align:left;vertical-align:top}th{color:var(--muted);font-weight:600}
figure{margin:1.2em 0}img{max-width:100%;height:auto;background:#fff;border:1px solid var(--line);border-radius:6px}
nav{background:var(--card);padding:10px 14px;border-radius:6px;font-size:14px}nav a{margin-right:12px;display:inline-block}
"""


def inline(text, base):
    text = html.escape(text, quote=False)
    text = re.sub(r"`([^`]+)`", r"<code>\1</code>", text)
    text = re.sub(r"\*\*([^*]+)\*\*", r"<strong>\1</strong>", text)
    text = re.sub(r"\[([^\]]+)\]\(([^)]+)\)", lambda m: f'<a href="{m.group(2)}">{m.group(1)}</a>', text)
    return text


def image(alt, src, base):
    path = (base / src).resolve()
    data = base64.b64encode(path.read_bytes()).decode()
    return f'<figure><img alt="{html.escape(alt)}" src="data:image/png;base64,{data}"></figure>'


def convert(md, base):
    out, lines, i, toc = [], md.splitlines(), 0, []
    while i < len(lines):
        line = lines[i]
        if line.startswith("```"):
            j = i + 1
            while j < len(lines) and not lines[j].startswith("```"):
                j += 1
            out.append("<pre><code>" + html.escape("\n".join(lines[i + 1:j])) + "</code></pre>")
            i = j + 1
            continue
        m = re.match(r"^(#{1,3}) (.*)", line)
        if m:
            level, title = len(m.group(1)), m.group(2)
            anchor = re.sub(r"[^a-z0-9]+", "-", title.lower()).strip("-")
            if level == 2:
                toc.append((anchor, title))
            out.append(f'<h{level} id="{anchor}">{inline(title, base)}</h{level}>')
            i += 1
            continue
        m = re.match(r"^!\[([^\]]*)\]\(([^)]+)\)", line)
        if m:
            out.append(image(m.group(1), m.group(2), base))
            i += 1
            continue
        if line.startswith("|"):
            rows = []
            while i < len(lines) and lines[i].startswith("|"):
                rows.append([c.strip() for c in lines[i].strip().strip("|").split("|")])
                i += 1
            head, body = rows[0], [r for r in rows[1:] if not all(re.fullmatch(r":?-+:?", c) for c in r)]
            out.append('<div class="tbl"><table><thead><tr>' + "".join(f"<th>{inline(c, base)}</th>" for c in head)
                       + "</tr></thead><tbody>" + "".join("<tr>" + "".join(f"<td>{inline(c, base)}</td>" for c in r)
                                                         + "</tr>" for r in body) + "</tbody></table></div>")
            continue
        if re.match(r"^(\s*[-*] |\s*\d+\. )", line):
            ordered = bool(re.match(r"^\s*\d+\. ", line))
            items = []
            while i < len(lines) and re.match(r"^(\s*[-*] |\s*\d+\. |\s{2,}\S)", lines[i]):
                if re.match(r"^(\s*[-*] |\s*\d+\. )", lines[i]):
                    items.append(re.sub(r"^(\s*[-*] |\s*\d+\. )", "", lines[i]))
                else:
                    items[-1] += " " + lines[i].strip()
                i += 1
            tag = "ol" if ordered else "ul"
            out.append(f"<{tag}>" + "".join(f"<li>{inline(x, base)}</li>" for x in items) + f"</{tag}>")
            continue
        if line.strip():
            para = [line]
            i += 1
            while i < len(lines) and lines[i].strip() and not re.match(r"^(#|\||!\[|```|\s*[-*] |\s*\d+\. )", lines[i]):
                para.append(lines[i]); i += 1
            out.append(f"<p>{inline(' '.join(para), base)}</p>")
            continue
        i += 1
    nav = "<nav>" + "".join(f'<a href="#{a}">{html.escape(t)}</a>' for a, t in toc) + "</nav>"
    return nav, "\n".join(out)


def main():
    src, dst = Path(sys.argv[1]), Path(sys.argv[2])
    nav, body = convert(src.read_text(), src.parent)
    title = "Phase 5.4 Stage A"
    dst.write_text(f"<!doctype html><html lang=en><head><meta charset=utf-8>"
                   f"<meta name=viewport content='width=device-width,initial-scale=1'><title>{title}</title>"
                   f"<style>{CSS}</style></head><body><main>{nav}{body}</main></body></html>")
    print("wrote", dst, f"{dst.stat().st_size / 1e6:.1f} MB")


if __name__ == "__main__":
    main()
