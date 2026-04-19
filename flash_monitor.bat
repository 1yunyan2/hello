@echo off
echo ========================================
echo  Echo2 烧录 + 监视脚本
echo  端口: COM5  波特率: 115200
echo ========================================
call D:\ESP-IDF\.espressif\v5.3.4\esp-idf\export.bat
cd /d d:\new\Echopals\Echo2
idf.py -p COM5 flash monitor
