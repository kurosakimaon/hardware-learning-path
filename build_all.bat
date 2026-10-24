@echo off
setlocal enabledelayedexpansion
chcp 65001 >nul
cd /d "%~dp0"

where gcc >nul 2>nul
if errorlevel 1 (
  if exist "C:\msys64\ucrt64\bin\gcc.exe" (
    set "PATH=C:\msys64\ucrt64\bin;%PATH%"
  ) else (
    echo 错误：找不到 gcc。请安装 MSYS2 UCRT64，或把 gcc 加入 PATH。
    exit /b 1
  )
)

if not exist "build\bin" mkdir "build\bin"

set /a count=0
for /r "%CD%" %%F in (*.c) do (
  echo %%F | findstr /I /C:"\_ref_tmp\" /C:"\.review_out\" /C:"\.review_build\" /C:"\build\" >nul
  if errorlevel 1 (
    set /a count+=1
    echo [!count!] %%~nxF
    gcc -std=c17 -O2 -Wall -Wextra -Werror "%%F" -o "build\bin\%%~nF.exe"
    if errorlevel 1 (
      echo 编译失败：%%F
      exit /b 1
    )
  )
)

echo.
echo 编译完成：!count! 个程序 -^> build\bin\
endlocal
