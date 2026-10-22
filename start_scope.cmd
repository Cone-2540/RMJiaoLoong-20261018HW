@echo off
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0scripts\start_scope.ps1"
if errorlevel 1 pause
