@echo off
setlocal
set "CSC=%WINDIR%\Microsoft.NET\Framework64\v4.0.30319\csc.exe"
if not exist "%CSC%" set "CSC=%WINDIR%\Microsoft.NET\Framework\v4.0.30319\csc.exe"
if not exist "%CSC%" (
  echo Microsoft .NET Framework C# compiler was not found.
  pause
  exit /b 1
)
"%CSC%" /nologo /target:winexe /platform:anycpu /optimize+ /out:JDY31PidTuner.ToggleMotion.exe /reference:System.dll /reference:System.Drawing.dll /reference:System.Windows.Forms.dll /reference:System.Management.dll Program.cs
if errorlevel 1 (
  pause
  exit /b 1
)
echo Built JDY31PidTuner.ToggleMotion.exe successfully.
pause
