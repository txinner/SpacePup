param(
    [Parameter(Mandatory=$true)][string]$Jdk,
    [Parameter(Mandatory=$true)][string]$AndroidJar,
    [Parameter(Mandatory=$true)][string]$ArCoreJar,
    [Parameter(Mandatory=$true)][string]$OpenCvJar,
    [Parameter(Mandatory=$true)][string]$OpenCvDll
)
$ErrorActionPreference='Stop'
$project=Split-Path $PSScriptRoot -Parent
$out=Join-Path $project 'build/replay'
New-Item -ItemType Directory -Force $out | Out-Null
$classpath=@($out,$AndroidJar,$ArCoreJar,$OpenCvJar) -join [IO.Path]::PathSeparator
$source=Join-Path $project 'app/src/main/java/cn/spacepup/app'
$files=@('Rigid','Vision','Space','DeadReckoner','MapRefresh') | ForEach-Object {Join-Path $source "$_.java"}
& (Join-Path $Jdk 'bin/javac.exe') -encoding UTF-8 -cp $classpath -d $out @files (Join-Path $PSScriptRoot 'Replay.java')
if($LASTEXITCODE -ne 0){exit $LASTEXITCODE}
$env:OPENCV_OPENCL_RUNTIME='disabled'
& (Join-Path $Jdk 'bin/java.exe') "-Djava.io.tmpdir=$out" -cp $classpath cn.spacepup.app.Replay (Resolve-Path -LiteralPath $OpenCvDll).Path
exit $LASTEXITCODE

