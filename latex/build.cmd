@echo off
setlocal
cd /d "%~dp0"
where latexmk >nul 2>nul
if errorlevel 1 (
  echo latexmk not found. Install TeX Live or MiKTeX and enable PATH.
  exit /b 1
)
latexmk -xelatex -interaction=nonstopmode -halt-on-error -outdir=build main.tex
if errorlevel 1 exit /b 1
echo PDF ready: %~dp0build\main.pdf
endlocal
