<#
.SYNOPSIS
创建论文笔记文件夹，命名格式为[年份]_[第一作者姓氏]_[标题]

.DESCRIPTION
该脚本用于自动化创建论文笔记文件夹，采用标准化命名，便于与Zotero中的文献关联，并可复制指定的LaTeX笔记模板，同时自动填充论文标题。
#>

# 提示用户输入信息
Write-Host "=== 论文笔记创建 ===" -ForegroundColor Cyan

# 获取年份（确保4位数字）
do {
    $year = Read-Host "请输入论文发表年份"
    if (-not ($year -match '^\d{4}$')) {
        Write-Host "年份格式不正确，请输入4位数字（例如：2023）" -ForegroundColor Red
    }
} while (-not ($year -match '^\d{4}$'))

# 获取第一作者姓氏
do {
    $author = Read-Host "请输入第一作者姓氏"
    if ([string]::IsNullOrWhiteSpace($author)) {
        Write-Host "作者姓氏不能为空" -ForegroundColor Red
    }
} while ([string]::IsNullOrWhiteSpace($author))

# 处理作者姓氏，移除空格和特殊字符
$authorClean = $author -replace '[^a-zA-Z0-9]', ''

# 获取论文标题
do {
    $title = Read-Host "请输入论文标题"
    if ([string]::IsNullOrWhiteSpace($title)) {
        Write-Host "论文标题不能为空" -ForegroundColor Red
    }
} while ([string]::IsNullOrWhiteSpace($title))

# 处理标题，替换空格为下划线，移除特殊字符（用于文件夹命名）
$titleClean = $title -replace '\s+', '_' -replace '[^a-zA-Z0-9_]', ''

# 构建文件夹名称
$folderName = "$year" + "_" + "$authorClean" + "_" + "$titleClean"

# 创建文件夹
try {
    New-Item -ItemType Directory -Path $folderName -ErrorAction Stop | Out-Null
} catch {
    Write-Host "创建文件夹失败：$_" -ForegroundColor Red
    exit
}

# 源LaTeX文件路径
$sourceTexPath = "E:\ChenHuaneng\Notes\literature\note-quickstart.tex"

# 目标文件路径（与文件夹同名）
$targetTexPath = Join-Path $folderName "$folderName.tex"

try {
    # 复制文件
    Copy-Item -Path $sourceTexPath -Destination $targetTexPath -ErrorAction Stop

    # 读取LaTeX文件内容并替换标题
    $texContent = Get-Content -Path $targetTexPath -Raw
    $updatedContent = $texContent -replace '\\title{}', "\title{$title}"
    
    # 保存更新后的内容
    Set-Content -Path $targetTexPath -Value $updatedContent -Encoding UTF8
} catch {
    Write-Host "处理LaTeX模板失败：$_" -ForegroundColor Red
}

Write-Host "🎉脚本执行完毕" -ForegroundColor Cyan
