echo off

set bin=%~dp0x64\Release\LocalExpansionStereo.exe
set datasetroot=%~dp0data

"%bin%" -targetDir "D:\ruichen\RIS_STD_final_outputs\sgbm_outputs\sphere_1000\outputs_fabric_pattern_07_2k_png\D2.0_X0" -outputDir "D:\ruichen\RIS_STD_final_outputs\sgbm_outputs\sphere_1000\outputs_fabric_pattern_07_2k_png\D2.0_X0" -mode MiddV2 -ndisp 256 

echo createobject("scripting.filesystemobject").deletefile(wscript.scriptfullname) >%temp%\VBScriptWait.vbs& echo wscript.sleep 10000 >>%temp%\VBScriptWait.vbs& start /wait %temp%\VBScriptWait.vbs