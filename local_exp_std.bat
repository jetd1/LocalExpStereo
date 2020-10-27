echo off

set bin=%~dp0x64\Release\LocalExpansionStereo.exe
set datasetroot=%~dp0data

"%bin%" -targetDir "D:\ruichen\RIS_STD_final_outputs\sgbm_outputs\plane\outputs_coral_ground_02_2k_png\D2.0_X0" -outputDir "D:\ruichen\RIS_STD_final_outputs\sgbm_outputs\plane\outputs_coral_ground_02_2k_png\D2.0_X0" -mode MiddV2 -ndisp 256
1 / 3 MiddV2
"%bin%" -targetDir "D:\ruichen\RIS_STD_final_outputs\sgbm_outputs\plane\outputs_coral_ground_02_2k_png\D2.0_X0" -outputDir "D:\ruichen\RIS_STD_final_outputs\sgbm_outputs\plane\outputs_coral_ground_02_2k_png\D2.0_X0" -mode Census -ndisp 256
1 / 3 Census
"%bin%" -targetDir "D:\ruichen\RIS_STD_final_outputs\sgbm_outputs\plane\outputs_grey_roof_tiles_02_2k_png\D2.0_X0" -outputDir "D:\ruichen\RIS_STD_final_outputs\sgbm_outputs\plane\outputs_grey_roof_tiles_02_2k_png\D2.0_X0" -mode MiddV2 -ndisp 256
2 / 3 MiddV2
"%bin%" -targetDir "D:\ruichen\RIS_STD_final_outputs\sgbm_outputs\plane\outputs_grey_roof_tiles_02_2k_png\D2.0_X0" -outputDir "D:\ruichen\RIS_STD_final_outputs\sgbm_outputs\plane\outputs_grey_roof_tiles_02_2k_png\D2.0_X0" -mode Census -ndisp 256
2 / 3 Census
"%bin%" -targetDir "D:\ruichen\RIS_STD_final_outputs\sgbm_outputs\sphere_1000\outputs_fabric_pattern_07_2k_png\D2.0_X0" -outputDir "D:\ruichen\RIS_STD_final_outputs\sgbm_outputs\sphere_1000\outputs_fabric_pattern_07_2k_png\D2.0_X0" -mode MiddV2 -ndisp 256
3 / 3 MiddV2
"%bin%" -targetDir "D:\ruichen\RIS_STD_final_outputs\sgbm_outputs\sphere_1000\outputs_fabric_pattern_07_2k_png\D2.0_X0" -outputDir "D:\ruichen\RIS_STD_final_outputs\sgbm_outputs\sphere_1000\outputs_fabric_pattern_07_2k_png\D2.0_X0" -mode Census -ndisp 256
3 / 3 Census
