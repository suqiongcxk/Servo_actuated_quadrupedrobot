[CmdletBinding()]
param(
    [string]$RepoUrl,
    [string]$Message = ('Update quadruped firmware ' + (Get-Date -Format 'yyyy-MM-dd HH:mm:ss')),
    [string]$Branch,
    [switch]$CheckOnly
)

$ErrorActionPreference = 'Stop'
Set-StrictMode -Version Latest

function Invoke-Git {
    param([Parameter(ValueFromRemainingArguments = $true)][string[]]$GitArgs)
    & git @GitArgs
    if ($LASTEXITCODE -ne 0) { throw ('Git failed: ' + ($GitArgs -join ' ')) }
}

Push-Location -LiteralPath $PSScriptRoot
try {
    if (-not (Get-Command git -ErrorAction SilentlyContinue)) {
        throw 'Install Git for Windows first, then run this script again.'
    }
    if (-not (Test-Path -LiteralPath '.git')) {
        throw 'This folder is not initialized. Run: git init -b main'
    }
    if ($CheckOnly) {
        Invoke-Git status --short
        Write-Host 'Check complete. No commit or push was performed.'
        return
    }
    $conflicts = @(Invoke-Git diff --name-only --diff-filter=U)
    if ($conflicts.Count -gt 0 -or (Test-Path '.git/MERGE_HEAD') -or
        (Test-Path '.git/rebase-merge') -or (Test-Path '.git/rebase-apply')) {
        throw 'Finish or abort the existing merge/rebase before uploading.'
    }
    $currentBranch = Invoke-Git symbolic-ref --quiet --short HEAD
    if (-not $currentBranch) { throw 'Check out a local branch before uploading.' }
    $origin = & git config --get remote.origin.url
    if (-not $RepoUrl) { $RepoUrl = $origin }
    if (-not $RepoUrl) { $RepoUrl = Read-Host 'Paste the GitHub repository URL' }
    $RepoUrl = $RepoUrl.Trim()
    if ($RepoUrl -notmatch '^(https://github\.com/|git@github\.com:)[A-Za-z0-9_.-]+/[A-Za-z0-9_.-]+/?$') {
        throw 'Use a GitHub HTTPS or SSH repository URL, without a token in the URL.'
    }
    if ($origin -and $origin -ne $RepoUrl) {
        throw 'origin already points elsewhere. Verify it and use git remote set-url origin explicitly.'
    }
    if (-not $origin) { Invoke-Git remote add origin $RepoUrl }

    # Verify access before making a commit. Git Credential Manager handles login.
    $remoteRefs = @(Invoke-Git ls-remote --symref origin HEAD 'refs/heads/*')
    if (-not $Branch) {
        foreach ($line in $remoteRefs) {
            if ($line -match '^ref: refs/heads/(\S+)\s+HEAD$') { $Branch = $Matches[1]; break }
        }
        if (-not $Branch) { $Branch = 'main' }
    }
    Invoke-Git check-ref-format --branch $Branch
    if (-not (& git config --get user.name)) {
        $authorName = Read-Host 'Git commit author name'
        if (-not $authorName) { throw 'An author name is required.' }
        Invoke-Git config --local user.name $authorName
    }
    if (-not (& git config --get user.email)) {
        $authorEmail = Read-Host 'Git commit author email (GitHub noreply address is supported)'
        if (-not $authorEmail) { throw 'An author email is required.' }
        Invoke-Git config --local user.email $authorEmail
    }

    # Abort rather than upload unexpected paths staged by another tool.
    $scope = '^(F103C8T6/|F103C8T6后/|H723zhukong/|\.gitignore$|\.gitattributes$|README\.md$|upload-github\.(ps1|bat)$)'
    foreach ($path in @(Invoke-Git -c core.quotepath=false diff --cached --name-only)) {
        if ($path -notmatch $scope) { throw ('Unexpected staged path: ' + $path) }
    }
    Invoke-Git add -A -- F103C8T6 F103C8T6后 H723zhukong .gitignore .gitattributes README.md upload-github.ps1 upload-github.bat
    & git diff --cached --quiet
    $diffExit = $LASTEXITCODE
    if ($diffExit -eq 1) { Invoke-Git commit -m $Message }
    elseif ($diffExit -ne 0) { throw 'Unable to inspect staged changes.' }

    Invoke-Git fetch origin
    $remoteBranchExists = $false
    foreach ($line in $remoteRefs) {
        if ($line -match ('\srefs/heads/' + [regex]::Escape($Branch) + '$')) {
            $remoteBranchExists = $true
        }
    }
    if ($remoteBranchExists) {
        # Preserve any existing GitHub README/history; never force-push.
        Invoke-Git merge --no-edit --allow-unrelated-histories ('refs/remotes/origin/' + $Branch)
    }
    Invoke-Git push -u origin ('HEAD:refs/heads/' + $Branch)
    Write-Host ('Uploaded to ' + $RepoUrl + ' [' + $Branch + ']') -ForegroundColor Green
}
catch {
    Write-Host $_.Exception.Message -ForegroundColor Red
    Write-Host 'Upload stopped. No force-push was used. Resolve the reported issue and rerun.'
    exit 1
}
finally { Pop-Location }
