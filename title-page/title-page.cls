\NeedsTeXFormat{LaTeX2e}
\ProvidesClass{title-page}[2025-02-25 Template for title page]

% 如果不需要奇偶面不同的话切换下面两行的注释顺序，同样要修改fancyhdr的选项
% \LoadClass[10pt, a4paper, fontset=none, oneside]{ctexbook}
\LoadClass[10pt, a4paper, fontset=none]{ctexbook}

%%%%%%%%%%%%%%%%导入宏包%%%%%%%%%%%%%%%%%%%%
% 参考https://tex.stackexchange.com/questions/59702/suggest-a-nice-font-family-for-my-basic-latex-template-text-and-math


% \RequirePackage{syntonly} % 编译LaTeX文件后不生成DVI或者PDF文档，只排查错误，要生成文档可以注释\syntaxonly命令
% \syntaxonly
\RequirePackage{geometry} % 设置页面参数
\RequirePackage{hyperref} % 超链接
\RequirePackage{epigraph} % 排版引用
% 字体
\RequirePackage[T1]{fontenc} % 改善文档中西欧语言的显示效果
\RequirePackage{lmodern} % lmodern字体
\RequirePackage{libertine} % Linux Libertine 字体系列(衬线字体)
\RequirePackage{tikz} % 绘图


%%%%%%%%%%%%%%%%导入宏包%%%%%%%%%%%%%%%%%%%%



%%%%%%%%%%%%%%%%设置字体%%%%%%%%%%%%%%%%%%%%


% 中文字体，具体设置参考ctex宏包
\setCJKmainfont{FZSSJW.TTF}[ % 设置中文主字体为方正书宋 
    Path=../fonts/,
    BoldFont=FZHTJW.TTF, % 设置粗体为方正黑体
    ItalicFont=FZKTJW.TTF, % 设置斜体为方正楷体
]
\setCJKsansfont{FZHTJW.TTF}[Path=../fonts/] % 设置无衬线字体为方正黑体
\setCJKmonofont{FZFSJW.TTF}[Path=../fonts/] % 设置等宽字体为方正仿宋

% 英文字体，具体设置参考fontspec宏包
% \setmainfont{CaskaydiaCoveNerdFont}[ % 设置英文主字体为CaskaydiaCove Nerd Font
%     Path=./fonts/, % 指定字体文件所在的目录
%     Extension=.ttf, % 字体文件后缀
%     UprightFont=*-regular, % 正常字体
%     BoldFont=*-Bold, % 加粗
%     ItalicFont=*-Italic, % 斜体
%     BoldItalicFont=*-BoldItalic, % 粗斜体
% ]
% \setsansfont{CaskaydiaCoveNerdFont}[ % 设置英文无衬线字体为CaskaydiaCove Nerd Font
%     Path=./fonts/, % 指定字体文件所在的目录
%     Extension=.ttf, % 字体文件后缀
%     UprightFont=*-regular, % 正常字体
%     BoldFont=*-Bold, % 加粗
%     ItalicFont=*-Italic, % 斜体
%     BoldItalicFont=*-BoldItalic, % 粗斜体
% ]
\setmonofont{CaskaydiaCoveNerdFontMono}[ % 设置英文等宽字体为CaskaydiaCove Nerd Font Mono
    Path=../fonts/, % 指定字体文件所在的目录
    Extension=.ttf, % 字体文件后缀
    UprightFont=*-regular, % 正常字体
    BoldFont=*-Bold, % 加粗
    ItalicFont=*-Italic, % 斜体
    BoldItalicFont=*-BoldItalic, % 粗斜体
]

% 具体设置参考unicode-math宏包
% \setmathfont{TeX Gyre DejaVu Math} % DejaVu风格数学Unicode字体

\linespread{1.25} % 修改行距


%%%%%%%%%%%%%%%%设置字体%%%%%%%%%%%%%%%%%%%%



%%%%%%%%%%%%%%%%页面设置%%%%%%%%%%%%%%%%%%%%


% 具体设置参考geometry宏包
\geometry{
    a4paper,
    inner=1in, % 靠近书脊一侧的边距
    outer=1.25in, % 远离书脊一侧的边距
    top=72pt, % 上边距
    marginparsep=7pt, % 正文区域和边注区域的间距
    marginparwidth=72pt, % 边注区域的宽度
    textheight=700pt, % 正文区域的高度
    footskip=25pt, % 页脚区域和正文区域的间距
}


%%%%%%%%%%%%%%%%页面设置%%%%%%%%%%%%%%%%%%%%



%%%%%%%%%%%%%%%%封面设计%%%%%%%%%%%%%%%%%%%%
% 参考https://tex.stackexchange.com/questions/85904/showcase-of-beautiful-title-page-done-in-tex


% title page
\usetikzlibrary{calc}
\tikzset{3D/.cd,
  x/.store in=\xx, x=0,
  y/.store in=\yy, y=0,
  z/.store in=\zz, z=0
}

\tikzdeclarecoordinatesystem{3D}{%
  \tikzset{3D/.cd,#1}%
  \pgfpoint{sin(\yy)*(\xx)}{-((\xx)/75)^2+(\zz)/100*(\xx)}%
}
\renewcommand\epigraphflush{flushright}
\renewcommand\epigraphsize{\normalsize}
\setlength\epigraphwidth{0.7\textwidth}
\definecolor{titlepagecolor}{cmyk}{1,.60,0,.40}
\DeclareFixedFont{\titlefont}{T1}{ppl}{b}{it}{0.5in}
\makeatletter                       
\def\printauthor{%                  
    {\large \@author}}              
\makeatother
\author{%
    \href{chen-huaneng.github.io}{\textcolor{black}{Chen Huaneng (Abel)}} \\  %%%% Your Name
    Xiamen University \\    %%%% Department Name
    \texttt{huanengchen@foxmail.com}\vspace{5pt} \\ %%% Your Email Address
    }
\newcommand\titlepagedecoration{%
    \begin{tikzpicture}[line join=round, very thin]
    \def\e{1260}
    \foreach \x [evaluate={\i=mod(\x+90,360); \j=int((\i<180)*2-1); \t=3;
        \sc=\x/\e; \n=int((\e-\x)/15+5); \X=\x/\e;}] in {10,25,...,\e} {

        \path [shift={(3D cs:x=\x-\t,y={3*sin(\x-\t)})}, yslant=cos(\x)/5]
        (-\X/2, 0)   coordinate (A')  ( \X/2, 0)   coordinate (B')
        ( \X/2,2*\X) coordinate (C')  (-\X/2,2*\X) coordinate (D');

        \path [shift={(3D cs:x=\x,y=3*sin \x)}, yslant=cos(\x)/5]
        (-\X/2, 0)   coordinate (A) ( \X/2, 0)   coordinate (B)
        ( \X/2,2*\X) coordinate (C) (-\X/2,2*\X) coordinate (D);

        \filldraw [black!90] (B) -- (B') -- (C') -- (C)  -- cycle;
        \filldraw [black!80] (A) -- (A') -- (D') -- (D)  -- cycle;
        \filldraw [black!70] (C) -- (D)  -- (D') -- (C') -- cycle;
        \filldraw [black]    (A) -- (B)  -- (C)  -- (D)  -- cycle;

        \node [text=white, shift={($(C)!0.5!(D)$)}, anchor=north,
        yslant=cos(\x)/5, font=\sf, scale=\sc*1.5] at (0,-.33*\X) {\n};
    }
    %
    \foreach \i [evaluate={\x=\i*30-10; \X=1; \n=int(5-\i); \xsl=\x/180}]
        in {1,...,4} {

        \path [shift={(3D cs:x=\x+\e,y=-3*\x/90)}, yslant=cos \e/5, xslant=\xsl]
        (-\X/2, 0)           coordinate (A) ( \X/2, 0)           coordinate (B)
        ( \X/2, \X*2-\x/360) coordinate (C) (-\X/2, \X*2-\x/360) coordinate (D);

        \path [shift={(3D cs:x=\x+\e,y=-3*\x/90)}, shift={(5/50,5/50-\i*2/50)},
        yslant=cos \e/5, xslant=\xsl]
        (-\X/2, 0)           coordinate (A') ( \X/2, 0)           coordinate (B')
        ( \X/2, \X*2-\x/330) coordinate (C') (-\X/2, \X*2-\x/330) coordinate (D');

        \filldraw [black!70] (C) -- (D)  -- (D') -- (C') -- cycle;
        \filldraw [black!70] (A) -- (B)  -- (B') -- (A') -- cycle;
        \filldraw [black!90] (B) -- (B') -- (C') -- (C)  -- cycle;
        \filldraw [black]    (A) -- (B)  -- (C)  -- (D)  -- cycle;

        \node [text=white, shift={($(C)!0.5!(D)$)}, anchor=north, xslant=\xsl,
        yslant=cos \e/5, font=\sf, scale=1.5] at (0,-.33*\X) {\n};
    }
    \end{tikzpicture}
}


%%%%%%%%%%%%%%%%封面设计%%%%%%%%%%%%%%%%%%%%



%%%%%%%%%%%%%%%%超链接设计%%%%%%%%%%%%%%%%%%


\hypersetup{
    bookmarksopen=true, % 启用书签
    colorlinks=true, % 启用颜色
    linkcolor=red, % 内部链接的颜色
    citecolor=orange, % 引用链接的颜色
    urlcolor=cyan, % 外部链接的颜色
}


%%%%%%%%%%%%%%%%超链接设计%%%%%%%%%%%%%%%%%%
