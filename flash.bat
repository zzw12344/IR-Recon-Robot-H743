@echo off
chcp 65001 >nul
echo ==========================================
echo  CarH743 烧录（pyOCD + ST-Link）
echo ==========================================
"C:\Users\a1736\AppData\Local\Programs\Python\Python313\Scripts\pyocd.exe" flash -t stm32h743xx "%~dp0MDK-ARM\STM32H743\STM32H743.axf"
exit /b %ERRORLEVEL%
