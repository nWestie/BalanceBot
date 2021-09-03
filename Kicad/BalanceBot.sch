EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A 11000 8500
encoding utf-8
Sheet 1 1
Title "BalanceBot"
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L Device:Battery 12V
U 1 1 612AE4EA
P 2850 3600
F 0 "12V" H 2958 3646 50  0001 L CNN
F 1 "Battery" H 2958 3600 50  0000 L CNN
F 2 "" V 2850 3660 100 0000 C CNN
F 3 "~" V 2850 3660 50  0001 C CNN
	1    2850 3600
	1    0    0    -1  
$EndComp
$Comp
L Sensor_Motion:MPU-6050 U?
U 1 1 612B0BBF
P 7150 2050
F 0 "U?" H 7150 1261 50  0001 C CNN
F 1 "MPU-6050" H 7150 1261 50  0000 C CNN
F 2 "Sensor_Motion:InvenSense_QFN-24_4x4mm_P0.5mm" H 7150 1250 50  0001 C CNN
F 3 "https://store.invensense.com/datasheets/invensense/MPU-6050_DataSheet_V3%204.pdf" H 7150 1900 50  0001 C CNN
	1    7150 2050
	1    0    0    -1  
$EndComp
$Comp
L MCU_Module:Arduino_Nano_v3.x A?
U 1 1 612B5B39
P 4050 2050
F 0 "A?" H 4050 961 50  0000 C CNN
F 1 "Arduino_Nano_v3.x" H 4050 870 50  0000 C CNN
F 2 "Module:Arduino_Nano" H 4050 2050 50  0001 C CIN
F 3 "http://www.mouser.com/pdfdocs/Gravitech_Arduino_Nano3_0.pdf" H 4050 2050 50  0001 C CNN
	1    4050 2050
	1    0    0    -1  
$EndComp
$Comp
L My~Symbols:HC-06 U?
U 1 1 612B8175
P 6950 4000
F 0 "U?" H 7628 4101 50  0001 L CNN
F 1 "HC-06" H 7628 4055 50  0000 L CNN
F 2 "" H 7400 4000 50  0001 C CNN
F 3 "" H 7400 4000 50  0001 C CNN
	1    6950 4000
	1    0    0    -1  
$EndComp
$EndSCHEMATC
