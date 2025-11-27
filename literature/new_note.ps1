<#
.SYNOPSIS
创建论文笔记文件夹，命名格式为[年份]_[第一作者姓氏]_[标题(或缩写)]

.DESCRIPTION
该脚本用于自动化创建论文笔记文件夹，通过分析用户提供的BibTeX条目提取必要信息，
创建标准化命名的文件夹，生成references.bib文件，并自动填充LaTeX笔记模板中的相关字段。
如果标题过长（超过30个字符），则使用标题首字母缩写。
#>

# 提示用户输入信息
Write-Host "=== 论文笔记创建 ===" -ForegroundColor Cyan

# 获取BibTeX条目
$bibtex = ""
Write-Host "`n请粘贴论文的BibTeX条目（按两次回车结束输入）" -ForegroundColor Yellow
while ($true) {
    $line = Read-Host
    if ([string]::IsNullOrWhiteSpace($line)) {
        if ($bibtex.Trim() -ne "") {
            break
        }
    }
    $bibtex += $line + "`n"
}
$bibtex = $bibtex.Trim()

# 解析BibTeX
$bibtexPattern = '@(?<entrytype>\w+){(?<citationkey>[^,]+),\s*(?<fields>(.|\n)*)}'
$bibtexMatch = [regex]::Match($bibtex, $bibtexPattern)
if (-not $bibtexMatch.Success) {
    Write-Host "BibTeX格式不正确，请检查输入内容" -ForegroundColor Red
    exit
}

# 提取字段
$fields = @{}
$bibtexMatch.Groups['fields'].Value -split ',' | ForEach-Object {
    $fieldLine = $_.Trim()
    if ($fieldLine -match '^\s*(\w+)\s*=\s*[{"]?([^}"]*)[}"]?\s*$') {
        $fieldName = $matches[1].Trim().ToLower()
        $fieldValue = $matches[2].Trim()
        $fields[$fieldName] = $fieldValue
    }
}

# 从BibTeX中提取必要信息
$title = if ($fields.ContainsKey('title')) { $fields['title'] } else { "" }
$year = if ($fields.ContainsKey('year')) { $fields['year'] } else { "" }
$journal = if ($fields.ContainsKey('journal')) { $fields['journal'] } else { "" }
$doi = if ($fields.ContainsKey('doi')) { $fields['doi'] } else { "" }
$authorFull = if ($fields.ContainsKey('author')) { $fields['author'] } else { "" }

# 处理作者信息 - 仅用于文件夹命名
$firstAuthorSurname = ""
if (-not [string]::IsNullOrWhiteSpace($authorFull)) {
    # 提取第一作者
    $firstAuthor = $authorFull -split 'and' | Select-Object -First 1
    $firstAuthor = $firstAuthor.Trim()
    
    # 处理"Last, First"或"First Last"格式
    if ($firstAuthor -match ',') {
        $authorParts = $firstAuthor -split ','
        $firstAuthorSurname = $authorParts[0].Trim()
    } else {
        $authorParts = $firstAuthor -split '\s+'
        $firstAuthorSurname = $authorParts[-1].Trim()  # 假设姓氏在最后
    }
}

# 验证必要信息
if ([string]::IsNullOrWhiteSpace($title) -or [string]::IsNullOrWhiteSpace($year) -or [string]::IsNullOrWhiteSpace($firstAuthorSurname)) {
    Write-Host "BibTeX中缺少必要的信息（标题、年份或作者），请检查输入内容" -ForegroundColor Red
    
    # 提示用户手动输入缺失的信息
    if ([string]::IsNullOrWhiteSpace($title)) {
        do {
            $title = Read-Host "请输入论文标题"
        } while ([string]::IsNullOrWhiteSpace($title))
    }
    
    if ([string]::IsNullOrWhiteSpace($year)) {
        do {
            $year = Read-Host "请输入论文发表年份"
        } while (-not ($year -match '^\d{4}$'))
    }
    
    if ([string]::IsNullOrWhiteSpace($firstAuthorSurname)) {
        do {
            $firstAuthorSurname = Read-Host "请输入第一作者姓氏"
        } while ([string]::IsNullOrWhiteSpace($firstAuthorSurname))
    }
}

# 函数：获取标题首字母缩写
function Get-TitleAcronym {
    param(
        [string]$titleText
    )
    
    # 移除可能的括号内容和特殊符号
    $cleanTitle = $titleText -replace '\([^)]*\)', '' -replace '[^\w\s\-]', ''
    
    # 分割单词（包括处理连字符）
    $words = $cleanTitle -split '[\s\-]+' | Where-Object { $_ -match '\w' }
    
    # 获取每个单词的首字母并连接
    $acronym = ""
    foreach ($word in $words) {
        if ($word.Length -gt 0) {
            $acronym += $word.Substring(0, 1).ToUpper()
        }
    }
    
    return $acronym
}

# 处理标题 - 如果太长则使用首字母缩写
$maxTitleLength = 15
$titleForFolder = ""
if ($title.Length -gt $maxTitleLength) {
    $titleForFolder = Get-TitleAcronym -titleText $title
    Write-Host "标题过长，使用缩写: $titleForFolder" -ForegroundColor Yellow
} else {
    $titleForFolder = $title -replace '\s+', '_' -replace '[^a-zA-Z0-9_]', ''
    $titleForFolder = $titleForFolder.Trim('_')
}

# 处理字符串，移除空格和特殊字符
$authorClean = $firstAuthorSurname -replace '[^a-zA-Z0-9]', ''
$authorClean = $authorClean.Substring(0, [Math]::Min(10, $authorClean.Length))  # 限制作者部分长度

# 构建文件夹名称
$folderName = "$year" + "_" + "$authorClean" + "_" + "$titleForFolder"

# 如果文件夹名仍然太长，进一步缩短
$maxFolderLength = 100
if ($folderName.Length -gt $maxFolderLength) {
    $folderName = $folderName.Substring(0, $maxFolderLength)
    Write-Host "文件夹名称过长，已截断: $folderName" -ForegroundColor Yellow
}

# 创建文件夹
try {
    New-Item -ItemType Directory -Path $folderName -ErrorAction Stop | Out-Null
    Write-Host "✅ 已创建文件夹: $folderName" -ForegroundColor Green
} catch {
    Write-Host "创建文件夹失败：$_" -ForegroundColor Red
    exit
}

# 创建references.bib文件
$referencesPath = Join-Path $folderName "references.bib"
try {
    Set-Content -Path $referencesPath -Value $bibtex -Encoding UTF8
    Write-Host "✅ 已创建BibTeX文件: references.bib" -ForegroundColor Green
} catch {
    Write-Host "创建BibTeX文件失败：$_" -ForegroundColor Red
}

# 源LaTeX文件路径
$sourceTexPath = "E:\ChenHuaneng\Notes\literature\note-quickstart.tex"

# 目标文件路径（与文件夹同名）
$targetTexPath = Join-Path $folderName "$folderName.tex"

try {
    # 复制文件
    Copy-Item -Path $sourceTexPath -Destination $targetTexPath -ErrorAction Stop

    # 读取LaTeX文件内容
    $texContent = Get-Content -Path $targetTexPath -Raw
    
    # 替换标题
    $texContent = $texContent -replace '\\title\{.*?\}', "\title{$title}"
    
    # 填充其他字段 - 使用完整的作者列表
    $texContent = $texContent -replace '\\setjournal\{.*?\}', "\setjournal{$journal}"
    $texContent = $texContent -replace '\\setpaperauthor\{.*?\}', "\setpaperauthor{$authorFull}"
    $texContent = $texContent -replace '\\setpaperdate\{.*?\}', "\setpaperdate{$year}"
    $texContent = $texContent -replace '\\setpaperdoi\{.*?\}', "\setpaperdoi{$doi}"
    
    # 保存更新后的内容
    Set-Content -Path $targetTexPath -Value $texContent -Encoding UTF8
    Write-Host "✅ 已创建并配置LaTeX笔记文件: $folderName.tex" -ForegroundColor Green
} catch {
    Write-Host "处理LaTeX模板失败：$_" -ForegroundColor Red
}

Write-Host "`n🎉 脚本执行完毕 - 请检查 $folderName 文件夹" -ForegroundColor Cyan
