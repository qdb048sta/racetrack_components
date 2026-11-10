#!/usr/bin/env python3
"""Render canonical equation Markdown to SVG-safe GitHub Markdown and PDF.
Requires pdflatex, reportlab, pymupdf and pillow. Original equation source is
preserved in canonical Markdown, equation_manifest.json and equations.tex.
GitHub assets are self-contained SVG with outlined glyphs and white backgrounds;
high-resolution PNG intermediates used by ReportLab stay under --work.
"""
from __future__ import annotations
import argparse, hashlib, html, json, math, os, re, shutil, subprocess, textwrap
from dataclasses import dataclass
from datetime import date
from pathlib import Path
import fitz
from PIL import Image as PILImage, ImageDraw
from reportlab.lib import colors
from reportlab.lib.pagesizes import A4
from reportlab.lib.styles import ParagraphStyle
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.platypus import (BaseDocTemplate, Flowable, Frame, Image, KeepTogether, LongTable,
    PageBreak, PageTemplate, Paragraph, Spacer, TableStyle, XPreformatted)
from reportlab.platypus.tableofcontents import TableOfContents

TICK=chr(96)
TOKEN_RE=re.compile(r"@@EQ:([a-f0-9]{16})@@")
INLINE_RE=re.compile(r"(?<!\\)\$(?!\$)(.+?)(?<!\\)\$")
WARNINGS=[]

@dataclass
class Equation:
    key:str
    source:str
    display:bool
    path:Path|None=None
    github_path:Path|None=None
    width:float=0
    height:float=0
    depth:float=0
    padding:float=0
    @property
    def token(self): return f"@@EQ:{self.key}@@"

def protect_math(source):
    equations={}
    def add(expr,display):
        expr=expr.strip()
        key=hashlib.sha256(("D:" if display else "I:").encode()+expr.encode()).hexdigest()[:16]
        equations.setdefault(key,Equation(key,expr,display))
        return equations[key].token
    out=[];block=[];code=False;display=False
    for line in source.splitlines():
        if line.lstrip().startswith(TICK*3):
            code=not code;out.append(line);continue
        if code: out.append(line);continue
        if display:
            if "$$" in line:
                a,b=line.split("$$",1);block.append(a)
                out.append(add("\n".join(block),True));block=[];display=False
                if b.strip():out.append(INLINE_RE.sub(lambda m:add(m.group(1),False),b))
            else:block.append(line)
            continue
        if "$$" in line:
            a,b=line.split("$$",1)
            if a.strip():out.append(INLINE_RE.sub(lambda m:add(m.group(1),False),a))
            if "$$" in b:
                expr,c=b.split("$$",1);out.append(add(expr,True))
                if c.strip():out.append(INLINE_RE.sub(lambda m:add(m.group(1),False),c))
            else:block=[b];display=True
        else:
            parts=re.split("("+TICK+"[^"+TICK+"]*"+TICK+")",line)
            out.append("".join(p if p.startswith(TICK) else
                INLINE_RE.sub(lambda m:add(m.group(1),False),p) for p in parts))
    if display:raise ValueError("Unclosed display equation")
    return "\n".join(out)+"\n",equations

def render_equations(equations,assets,work,tex_format,dpi):
    assets.mkdir(parents=True,exist_ok=True);work.mkdir(parents=True,exist_ok=True)
    raster_dir=work/"equation_png";raster_dir.mkdir(parents=True,exist_ok=True)
    ordered=list(equations.values())
    if not ordered:return
    lines=[r"\documentclass{article}",r"\usepackage{amsmath,amssymb,bm}",
        r"\pdfmapfile{+cm.map}",r"\pdfmapfile{+cmextra.map}",r"\pdfmapfile{+symbols.map}",
        r"\pdfhorigin=0pt",r"\pdfvorigin=0pt",r"\begin{document}"]
    for i,eq in enumerate(ordered):
        eq.padding=3.0 if eq.display else .55
        style=r"\displaystyle " if eq.display else r"\textstyle "
        expr=re.sub(r"\\label\{[^}]*\}","",eq.source)
        expr=re.sub(r"\\tag\{([^}]*)\}",lambda m:r"\qquad\text{("+m.group(1)+")}",expr)
        lines += [r"\setbox0=\hbox{$"+style+expr+"$}",
            rf"\typeout{{EQMETA:{i}:\the\wd0:\the\ht0:\the\dp0}}",
            rf"\pdfpagewidth=\dimexpr\wd0+{2*eq.padding}pt\relax",
            rf"\pdfpageheight=\dimexpr\ht0+\dp0+{2*eq.padding}pt\relax",
            rf"\shipout\vbox{{\offinterlineskip\kern{eq.padding}pt"
            rf"\hbox{{\kern{eq.padding}pt\box0\kern{eq.padding}pt}}\kern{eq.padding}pt}}"]
    lines.append(r"\end{document}")
    tex=work/"equations.tex";tex.write_text("\n".join(lines))
    env=os.environ.copy()
    if Path("/usr/share/texlive/texmf-dist").exists():
        env["TEXMF"]="{/usr/share/texlive/texmf-dist,/usr/share/texmf}"
    cmd=["pdflatex","-interaction=nonstopmode","-halt-on-error","-no-shell-escape",
        f"-output-directory={work}"]
    if tex_format:cmd.append(f"-fmt={Path(tex_format).resolve()}")
    cmd.append(str(tex))
    p=subprocess.run(cmd,env=env,capture_output=True,text=True)
    (work/"compiler_output.txt").write_text(p.stdout+p.stderr)
    if p.returncode:raise RuntimeError("Equation TeX failed:\n"+"\n".join(p.stdout.splitlines()[-35:]))
    log=(work/"equations.log").read_text(errors="replace")
    meta={int(m[0]):tuple(float(x) for x in m[1:]) for m in
        re.findall(r"EQMETA:(\d+):([\d.]+)pt:([\d.]+)pt:([\d.]+)pt",log)}
    pdf=fitz.open(work/"equations.pdf")
    if len(pdf)!=len(ordered):raise RuntimeError(f"Formula page count {len(pdf)} != {len(ordered)}")
    manifest=[]
    for i,eq in enumerate(ordered):
        page=pdf[i];eq.path=raster_dir/f"eq_{eq.key}.png"
        eq.github_path=assets/f"eq_{eq.key}.svg"
        eq.width,eq.height=page.rect.width,page.rect.height
        eq.depth=meta[i][2]*72/72.27 if i in meta else 2
        page.get_pixmap(matrix=fitz.Matrix(dpi/72,dpi/72),alpha=False).save(str(eq.path))
        svg=page.get_svg_image(text_as_path=True)
        svg=re.sub(r"(<svg\b[^>]*>)",r'\1\n<rect width="100%" height="100%" fill="#ffffff"/>',svg,count=1)
        eq.github_path.write_text(svg,encoding="utf-8")
        manifest.append(dict(id=eq.key,source=eq.source,display=eq.display,
            filename=eq.github_path.name,width_pt=eq.width,height_pt=eq.height,
            depth_pt=eq.depth,padding_pt=eq.padding))
    (assets/"equation_manifest.json").write_text(json.dumps(manifest,indent=2))
    shutil.copyfile(tex,assets/"equations.tex")
    current={entry["filename"] for entry in manifest}
    for old in assets.iterdir():
        if re.fullmatch(r"eq_[a-f0-9]{16}\.svg",old.name) and old.name not in current:
            old.unlink()

def github_markdown(protected,equations,readme,canonical):
    def repl(m):
        eq=equations[m.group(1)]
        rel=os.path.relpath(eq.github_path,readme.parent).replace(os.sep,"/")
        alt=html.escape(re.sub(r"\s+"," ",eq.source),quote=True).replace("|","&#124;")
        w=min(960,max(1,round(eq.width*(1.55 if eq.display else 1.45))))
        tag=f'<img src="{rel}" width="{w}" alt="{alt}"'
        return '<p align="center">'+tag+"></p>" if eq.display else tag+' align="middle">'
    note=("<!-- Generated from canonical equation source by tools/render_vehicle_reference.py -->\n\n"
        f"Editable equation source: [{canonical.name}]"
        f"({os.path.relpath(canonical,readme.parent).replace(os.sep,'/')}). "
        "Equations are self-contained SVG images so they also display inside GitHub tables. "
        "Each image has a white background for legibility in light and dark themes.\n\n")
    readme.parent.mkdir(parents=True,exist_ok=True)
    readme.write_text(note+TOKEN_RE.sub(repl,protected))

def setup_fonts():
    root=Path("/usr/share/fonts/truetype/dejavu")
    for name,file in [("Body","DejaVuSerif.ttf"),("Body-Bold","DejaVuSerif-Bold.ttf"),
        ("Body-Italic","DejaVuSerif-Italic.ttf"),("Body-BoldItalic","DejaVuSerif-BoldItalic.ttf"),
        ("Sans","DejaVuSans.ttf"),("Sans-Bold","DejaVuSans-Bold.ttf"),("Mono","DejaVuSansMono.ttf")]:
        pdfmetrics.registerFont(TTFont(name,str(root/file)))
    pdfmetrics.registerFontFamily("Body",normal="Body",bold="Body-Bold",
        italic="Body-Italic",boldItalic="Body-BoldItalic")
    pdfmetrics.registerFontFamily("Sans",normal="Sans",bold="Sans-Bold")

def inline_markup(text,equations,size=9.5):
    holds=[]
    def keep(value):
        key=f"ZZZSAFE{len(holds)}ZZZ";holds.append(value);return key
    def eq_repl(m):
        eq=equations[m.group(1)];scale=size/10*1.02
        return keep(f'<img src="{html.escape(str(eq.path))}" width="{eq.width*scale:.3f}" '
            f'height="{eq.height*scale:.3f}" valign="{-(eq.depth+eq.padding)*scale:.3f}"/>')
    text=TOKEN_RE.sub(eq_repl,text)
    text=re.sub(TICK+"([^"+TICK+"]*)"+TICK,lambda m:keep(
        f'<font name="Mono" size="{max(6.8,size-1)}">'+html.escape(m.group(1))+"</font>"),text)
    text=re.sub(r"\[([^\]]+)\]\(([^)]+)\)",lambda m:keep(
        '<link href="'+html.escape(m.group(2),quote=True)+'" color="#12677b">'+
        inline_markup(m.group(1),equations,size)+"</link>"),text)
    text=html.escape(text)
    text=re.sub(r"\*\*(.+?)\*\*",r"<b>\1</b>",text)
    text=re.sub(r"(?<!\*)\*([^*]+)\*(?!\*)",r"<i>\1</i>",text)
    text=text.replace(r"\_","_").replace(r"\|","|")
    for i,value in enumerate(holds):text=text.replace(f"ZZZSAFE{i}ZZZ",value)
    return text

class EquationFlowable(Flowable):
    def __init__(self,eq,max_width):
        super().__init__();self.eq=eq;self.scale=min(1.12,max_width/eq.width)
        self.width=eq.width*self.scale;self.height=eq.height*self.scale+6
        self.hAlign="CENTER";self.spaceBefore=3;self.spaceAfter=7
        if self.scale<.70:WARNINGS.append(f"Wide equation {eq.key}, scale={self.scale:.3f}: {eq.source[:100]}")
    def draw(self):
        self.canv.drawImage(str(self.eq.path),0,3,width=self.width,height=self.height-6,
            preserveAspectRatio=True,mask="auto")

class ReferenceDoc(BaseDocTemplate):
    def __init__(self,filename,title,**kwargs):
        super().__init__(filename,**kwargs);self.title=title;self.author="Case Study"
        f=Frame(self.leftMargin,self.bottomMargin,self.width,self.height,
            leftPadding=0,rightPadding=0,topPadding=0,bottomPadding=0)
        self.addPageTemplates(PageTemplate(id="normal",frames=f,onPage=self.page_header))
    def page_header(self,c,d):
        c.saveState()
        if d.page>1:
            c.setFont("Sans",7);c.setFillColor(colors.HexColor("#537187"))
            c.drawString(self.leftMargin,A4[1]-29,"CASE STUDY / VEHICLE AND TIRE MODELS")
            c.setStrokeColor(colors.HexColor("#d5e1e8"))
            c.line(self.leftMargin,A4[1]-36,A4[0]-self.rightMargin,A4[1]-36)
        c.setFont("Sans",6.7);c.setFillColor(colors.HexColor("#537187"))
        c.drawString(self.leftMargin,26,"Source equations and benchmark adaptations are distinguished throughout")
        c.drawRightString(A4[0]-self.rightMargin,26,str(d.page));c.restoreState()
    def afterFlowable(self,f):
        if isinstance(f,Paragraph) and getattr(f,"_toc_level",None) is not None:
            level=f._toc_level;key=f._bookmark_key
            self.canv.bookmarkPage(key);self.canv.addOutlineEntry(f.getPlainText(),key,level,False)
            self.notify("TOCEntry",(level,f.getPlainText(),self.page,key))

def style_sheet():
    navy=colors.HexColor("#173a55")
    base=dict(fontName="Body",fontSize=9.5,leading=14,
        textColor=colors.HexColor("#192d3d"),spaceAfter=6,
        allowWidows=0,allowOrphans=0,autoLeading="max",splitLongWords=True)
    s={
        "body":ParagraphStyle("body",**base),
        "table":ParagraphStyle("table",**{**base,"fontSize":8.1,"leading":11.2,
            "spaceAfter":0,"allowWidows":1,"allowOrphans":1}),
        "table_head":ParagraphStyle("table_head",**{**base,"fontName":"Sans-Bold",
            "fontSize":8.0,"leading":11,"spaceAfter":0}),
        "bullet":ParagraphStyle("bullet",**{**base,"leftIndent":14,
            "firstLineIndent":-10,"spaceAfter":4}),
        "quote":ParagraphStyle("quote",**{**base,"leftIndent":14,
            "textColor":colors.HexColor("#425e72")}),
        "code":ParagraphStyle("code",fontName="Mono",fontSize=7.2,leading=10,
            backColor=colors.HexColor("#f2f5f7"),borderColor=colors.HexColor("#dbe3e8"),
            borderWidth=.4,borderPadding=7,spaceBefore=5,spaceAfter=9),
        "title":ParagraphStyle("title",fontName="Sans-Bold",fontSize=27,
            leading=34,textColor=navy,spaceAfter=18),
        "subtitle":ParagraphStyle("subtitle",fontName="Body",fontSize=13,
            leading=20,textColor=colors.HexColor("#496b80"),spaceAfter=20)}
    for level,size,lead in [(1,18,24),(2,14.1,19),(3,11.7,17),(4,10.5,15)]:
        s[f"h{level}"]=ParagraphStyle(f"h{level}",fontName="Sans-Bold",fontSize=size,
            leading=lead,textColor=navy,spaceBefore=14 if level<=2 else 9,
            spaceAfter=7,keepWithNext=True)
    return s

def split_table_row(line):
    line=line.strip()
    if line.startswith("|"):line=line[1:]
    if line.endswith("|"):line=line[:-1]
    return [c.strip().replace(r"\|","|") for c in re.split(r"(?<!\\)\|",line)]

def table_flowable(rows,equations,styles,width):
    count=max(map(len,rows));rows=[r+[""]*(count-len(r)) for r in rows]
    if count>=5:widths=[width*.25]+[width*.75/(count-1)]*(count-1)
    else:
        weights=[]
        for j in range(count):
            lens=[len(TOKEN_RE.sub("formula",r[j])) for r in rows]
            weights.append(math.sqrt(min(120,max(18,sum(lens)/len(lens)))))
        widths=[width*w/sum(weights) for w in weights]
    cells=[]
    for i,row in enumerate(rows):
        st=styles["table_head"] if i==0 else styles["table"]
        cells.append([Paragraph(inline_markup(c,equations,st.fontSize),st) for c in row])
    t=LongTable(cells,colWidths=widths,repeatRows=1,hAlign="LEFT",
        splitByRow=1,splitInRow=0,spaceBefore=5,spaceAfter=10)
    t.setStyle(TableStyle([
        ("BACKGROUND",(0,0),(-1,0),colors.HexColor("#e9f1f5")),
        ("ROWBACKGROUNDS",(0,1),(-1,-1),[colors.white,colors.HexColor("#f7f9fa")]),
        ("VALIGN",(0,0),(-1,-1),"TOP"),("TOPPADDING",(0,0),(-1,-1),6),
        ("BOTTOMPADDING",(0,0),(-1,-1),6),("LEFTPADDING",(0,0),(-1,-1),6),
        ("RIGHTPADDING",(0,0),(-1,-1),6),
        ("LINEBELOW",(0,0),(-1,0),.7,colors.HexColor("#9eb7c5")),
        ("LINEBELOW",(0,1),(-1,-1),.25,colors.HexColor("#d8e2e8"))]))
    return t

def markdown_story(protected,equations,styles,width,source_parent):
    lines=protected.splitlines();story=[];i=0;bookmark=0
    while i<len(lines):
        line=lines[i].strip()
        if not line:i+=1;continue
        if line.startswith("<!--"):
            while i<len(lines) and "-->" not in lines[i]:i+=1
            i+=1;continue
        if line.startswith(TICK*3):
            block=[];i+=1
            while i<len(lines) and not lines[i].lstrip().startswith(TICK*3):
                block.extend(textwrap.wrap(lines[i],width=108,replace_whitespace=False,
                    drop_whitespace=False,break_long_words=True,break_on_hyphens=False) or [""])
                i+=1
            i+=1;story.append(XPreformatted(html.escape("\n".join(block)),styles["code"]));continue
        match=TOKEN_RE.fullmatch(line)
        if match and equations[match.group(1)].display:
            story.append(EquationFlowable(equations[match.group(1)],width));i+=1;continue
        h=re.match(r"^(#{1,6})\s+(.+)",line)
        if h:
            depth=len(h.group(1));text=h.group(2)
            if depth==1 and not story:i+=1;continue
            n=min(4,max(1,depth));st=styles[f"h{n}"]
            p=Paragraph(inline_markup(text,equations,st.fontSize),st)
            if depth<=3:
                p._toc_level=max(0,depth-2);p._bookmark_key=f"section-{bookmark}";bookmark+=1
            story.append(p);i+=1;continue
        if i+1<len(lines) and "|" in line and re.match(
                r"^\s*\|?\s*:?-{3,}:?\s*(\|\s*:?-{3,}:?\s*)+\|?\s*$",lines[i+1]):
            rows=[split_table_row(lines[i])];i+=2
            while i<len(lines) and "|" in lines[i] and lines[i].strip():
                rows.append(split_table_row(lines[i]));i+=1
            table=table_flowable(rows,equations,styles,width)
            if len(rows)<=7:
                group=[table]
                if story and isinstance(story[-1],Paragraph) and getattr(story[-1].style,"keepWithNext",False):
                    group.insert(0,story.pop())
                story.append(KeepTogether(group))
            else:story.append(table)
            continue
        if re.fullmatch(r"[-*_]{3,}",line):story.append(Spacer(1,6));i+=1;continue
        pic=re.fullmatch(r"!\[([^\]]*)\]\(([^)]+)\)",line)
        if pic:
            path=(source_parent/pic.group(2)).resolve()
            if path.is_file():
                with PILImage.open(path) as im:w,h=im.size
                scale=min(width/w,340/h)
                story.append(Image(str(path),width=w*scale,height=h*scale))
                if pic.group(1):story.append(Paragraph(html.escape(pic.group(1)),styles["table"]))
            else:
                WARNINGS.append(f"Image not found: {path}")
                story.append(Paragraph(html.escape(pic.group(1)),styles["quote"]))
            i+=1;continue
        bullet=re.match(r"^([-*+]|\d+\.)\s+(.+)",line)
        if bullet:
            label="•" if bullet.group(1) in ("-","*","+") else bullet.group(1)
            text=bullet.group(2);i+=1
            while i<len(lines) and lines[i].startswith("  ") and lines[i].strip():
                text+=" "+lines[i].strip();i+=1
            story.append(Paragraph(html.escape(label)+" "+inline_markup(text,equations),
                styles["bullet"]));continue
        if line.startswith(">"):
            story.append(Paragraph(inline_markup(line.lstrip("> "),equations),styles["quote"]))
            i+=1;continue
        block=[line];i+=1
        while i<len(lines) and lines[i].strip():
            nxt=lines[i].strip()
            if (nxt.startswith(("#",TICK*3,">","|")) or TOKEN_RE.fullmatch(nxt)
                or re.match(r"^([-*+]|\d+\.)\s+",nxt) or nxt.startswith("![")):break
            if i+1<len(lines) and re.match(r"^\s*\|?\s*:?-{3,}",lines[i+1]):break
            block.append(nxt);i+=1
        story.append(Paragraph(inline_markup(" ".join(block),equations),styles["body"]))
    return story

def render_pdf(protected,equations,canonical,output,title,subtitle,date_string,qa_dir):
    setup_fonts();styles=style_sheet();output.parent.mkdir(parents=True,exist_ok=True)
    doc=ReferenceDoc(str(output),title=title,pagesize=A4,leftMargin=49,rightMargin=49,
        topMargin=51,bottomMargin=45,pageCompression=1)
    story=[Spacer(1,75),
        Paragraph("CASE STUDY",ParagraphStyle("eyebrow",fontName="Sans-Bold",fontSize=10,
            leading=14,textColor=colors.HexColor("#14808c"),spaceAfter=18)),
        Paragraph(html.escape(title),styles["title"]),
        Paragraph(html.escape(subtitle),styles["subtitle"]),
        Paragraph(html.escape(date_string),styles["body"]),Spacer(1,20),
        Paragraph("Source audit • Complete selected equations • Reproducible comparison",styles["quote"]),
        PageBreak(),Paragraph("Contents",styles["h1"])]
    toc=TableOfContents()
    toc.levelStyles=[
        ParagraphStyle("toc0",fontName="Sans-Bold",fontSize=9.5,leading=14,
            textColor=colors.HexColor("#173a55"),spaceBefore=7,firstLineIndent=0,leftIndent=0),
        ParagraphStyle("toc1",fontName="Body",fontSize=8.5,leading=12,
            leftIndent=13,firstLineIndent=0,spaceBefore=3)]
    story += [toc,PageBreak()]
    story += markdown_story(protected,equations,styles,doc.width,canonical.parent)
    doc.multiBuild(story)
    pdf=fitz.open(output);qa_dir.mkdir(parents=True,exist_ok=True);thumbs=[]
    for old in qa_dir.iterdir():
        if re.fullmatch(r"(?:page-\d{3}|contact-\d+)\.png",old.name):old.unlink()
    for n,page in enumerate(pdf):
        path=qa_dir/f"page-{n+1:03d}.png"
        page.get_pixmap(matrix=fitz.Matrix(1.1,1.1),alpha=False).save(str(path))
        im=PILImage.open(path).convert("RGB");im.thumbnail((238,337))
        tile=PILImage.new("RGB",(258,365),"#e3e8ed")
        tile.paste(im,((258-im.width)//2,10))
        ImageDraw.Draw(tile).text((10,346),str(n+1),fill="#173a55");thumbs.append(tile)
    for start in range(0,len(thumbs),20):
        chunk=thumbs[start:start+20];rows=math.ceil(len(chunk)/4)
        contact=PILImage.new("RGB",(4*258,rows*365),"#e3e8ed")
        for j,im in enumerate(chunk):contact.paste(im,((j%4)*258,(j//4)*365))
        contact.save(qa_dir/f"contact-{start//20+1}.png")
    return len(pdf)

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument("canonical",type=Path);ap.add_argument("--readme",type=Path)
    ap.add_argument("--pdf",type=Path);ap.add_argument("--assets",type=Path,required=True)
    ap.add_argument("--work",type=Path);ap.add_argument("--qa-dir",type=Path)
    ap.add_argument("--tex-format");ap.add_argument("--dpi",type=float,default=288)
    ap.add_argument("--title",default="Vehicle and Tire Models")
    ap.add_argument("--subtitle",default="Source audit, complete equations, and Norisring comparison")
    ap.add_argument("--date",default=date.today().isoformat())
    a=ap.parse_args();a.canonical=a.canonical.resolve();a.assets=a.assets.resolve()
    protected,equations=protect_math(a.canonical.read_text())
    work=(a.work or (a.canonical.parent/".render_work")).resolve()
    render_equations(equations,a.assets,work,a.tex_format,a.dpi)
    if a.readme:github_markdown(protected,equations,a.readme.resolve(),a.canonical)
    pages=None
    if a.pdf:
        pages=render_pdf(protected,equations,a.canonical,a.pdf.resolve(),a.title,a.subtitle,
            a.date,(a.qa_dir or (work/"qa")).resolve())
    summary=dict(equations=len(equations),display_equations=sum(e.display for e in equations.values()),
        inline_equations=sum(not e.display for e in equations.values()),pages=pages,
        pdf=str(a.pdf) if a.pdf else None,readme=str(a.readme) if a.readme else None,warnings=WARNINGS)
    (work/"render_summary.json").write_text(json.dumps(summary,indent=2))
    print(json.dumps(summary,indent=2))

if __name__=="__main__":main()


