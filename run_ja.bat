@echo off
setlocal enabledelayedexpansion

echo.
echo 再生するデータを選択してください。
echo.

set count=0

for %%F in ("%~dp0replay\*.csv") do (
    set /a count+=1
    set "file[!count!]=%%F"
    echo !count!: %%~nxF
)

echo.

if %count%==0 (
    echo 再生データが見つかりません。
    pause
    exit /b
)

set /p choice=番号を入力してEnterキーを押してください: 

if not defined file[%choice%] (
    echo.
    echo 正しい番号を入力してください。
    pause
    exit /b
)

echo.
echo 選択されたファイル: !file[%choice%]!
echo.

"%~dp0x64\Release\RnR.exe" --lang ja --file "!file[%choice%]!" --play