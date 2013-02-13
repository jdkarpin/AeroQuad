set filename=%1%
if "%filename%" == "" set filename=objSTM32\AeroQuad32\AeroQuadMain.bin
:loop
	..\tools\dfu-util.exe --reset --alt 1 --download %filename% || goto :loop
pause
