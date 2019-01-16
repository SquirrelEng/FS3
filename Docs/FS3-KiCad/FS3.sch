EESchema Schematic File Version 4
LIBS:FS3-cache
EELAYER 26 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L arduinomicropro:ArduinoMicroPro U3
U 1 1 5B4B9DAB
P 5750 4200
F 0 "U3" H 5750 4950 60  0000 C CNN
F 1 "ArduinoMicroPro" V 5750 4200 60  0000 C CNN
F 2 "" H 5600 4950 60  0001 C CNN
F 3 "" H 5600 4950 60  0001 C CNN
	1    5750 4200
	1    0    0    -1  
$EndComp
$Comp
L adafruitsi5351:AdafruitSI5351 U1
U 1 1 5B4BA03C
P 3000 2050
F 0 "U1" H 3000 2550 60  0000 C CNN
F 1 "AdafruitSI5351" V 3100 2100 60  0000 C CNN
F 2 "" H 2850 2000 60  0001 C CNN
F 3 "" H 2850 2000 60  0001 C CNN
	1    3000 2050
	0    -1   -1   0   
$EndComp
$Comp
L ds18s20:DS18S20 U4
U 1 1 5B4BA1C9
P 8750 4700
F 0 "U4" H 8750 4950 60  0000 C CNN
F 1 "DS18S20" H 8750 4850 60  0000 C CNN
F 2 "" H 8750 4700 60  0001 C CNN
F 3 "" H 8750 4700 60  0001 C CNN
	1    8750 4700
	1    0    0    -1  
$EndComp
$Comp
L transistors:2N7000 Q2
U 1 1 5B4BA274
P 4550 4350
F 0 "Q2" V 4900 4300 50  0000 L CNN
F 1 "2N7000" V 4800 4200 50  0000 L CNN
F 2 "TO_SOT_Packages_THT:TO-92_Molded_Narrow" H 4750 4275 50  0001 L CIN
F 3 "" H 4550 4350 50  0001 L CNN
	1    4550 4350
	0    -1   -1   0   
$EndComp
$Comp
L transistors:2N7000 Q1
U 1 1 5B4BA2D3
P 2950 3200
F 0 "Q1" V 3200 3250 50  0000 L CNN
F 1 "2N7000" V 3200 2900 50  0000 L CNN
F 2 "TO_SOT_Packages_THT:TO-92_Molded_Narrow" H 3150 3125 50  0001 L CIN
F 3 "" H 2950 3200 50  0001 L CNN
	1    2950 3200
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R1
U 1 1 5B4BA320
P 7200 4150
F 0 "R1" V 7280 4150 50  0000 C CNN
F 1 "10M" V 7200 4150 50  0000 C CNN
F 2 "" V 7130 4150 50  0001 C CNN
F 3 "" H 7200 4150 50  0001 C CNN
	1    7200 4150
	1    0    0    -1  
$EndComp
$Comp
L Device:R R2
U 1 1 5B4BA359
P 7200 4600
F 0 "R2" V 7280 4600 50  0000 C CNN
F 1 "2M" V 7200 4600 50  0000 C CNN
F 2 "" V 7130 4600 50  0001 C CNN
F 3 "" H 7200 4600 50  0001 C CNN
	1    7200 4600
	1    0    0    -1  
$EndComp
$Comp
L Device:R R3
U 1 1 5B4BA3A8
P 8750 5550
F 0 "R3" V 8830 5550 50  0000 C CNN
F 1 "4.7K" V 8750 5550 50  0000 C CNN
F 2 "" V 8680 5550 50  0001 C CNN
F 3 "" H 8750 5550 50  0001 C CNN
	1    8750 5550
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR5
U 1 1 5B4BA586
P 6550 3750
F 0 "#PWR5" H 6550 3500 50  0001 C CNN
F 1 "GND" H 6550 3600 50  0000 C CNN
F 2 "" H 6550 3750 50  0001 C CNN
F 3 "" H 6550 3750 50  0001 C CNN
	1    6550 3750
	1    0    0    -1  
$EndComp
$Comp
L genericgps:GPS U2
U 1 1 5B4BA69D
P 3400 3750
F 0 "U2" H 3400 4100 60  0000 C CNN
F 1 "uBlox 7m" V 3500 3800 60  0000 C CNN
F 2 "" H 3200 3700 60  0001 C CNN
F 3 "" H 3200 3700 60  0001 C CNN
	1    3400 3750
	-1   0    0    1   
$EndComp
Text GLabel 5700 1900 0    60   Input ~ 0
Vcc
$Comp
L Device:Battery_Cell BT1
U 1 1 5B4BAC37
P 6000 2200
F 0 "BT1" H 6100 2300 50  0000 L CNN
F 1 "3.3V LIFePO 700mah" H 6100 2200 50  0000 L CNN
F 2 "" V 6000 2260 50  0001 C CNN
F 3 "" V 6000 2260 50  0001 C CNN
	1    6000 2200
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR4
U 1 1 5B4BAC88
P 6000 2300
F 0 "#PWR4" H 6000 2050 50  0001 C CNN
F 1 "GND" H 6000 2150 50  0000 C CNN
F 2 "" H 6000 2300 50  0001 C CNN
F 3 "" H 6000 2300 50  0001 C CNN
	1    6000 2300
	1    0    0    -1  
$EndComp
Text GLabel 3800 3950 2    60   Input ~ 0
VCC
Text GLabel 2650 2500 3    60   Input ~ 0
VCC
$Comp
L power:GND #PWR6
U 1 1 5B4BB62C
P 7200 4750
F 0 "#PWR6" H 7200 4500 50  0001 C CNN
F 1 "GND" H 7200 4600 50  0000 C CNN
F 2 "" H 7200 4750 50  0001 C CNN
F 3 "" H 7200 4750 50  0001 C CNN
	1    7200 4750
	1    0    0    -1  
$EndComp
Text GLabel 7400 3950 2    60   Input ~ 0
VCC
$Comp
L power:GND #PWR3
U 1 1 5B4BB7C8
P 4850 4250
F 0 "#PWR3" H 4850 4000 50  0001 C CNN
F 1 "GND" H 4850 4100 50  0000 C CNN
F 2 "" H 4850 4250 50  0001 C CNN
F 3 "" H 4850 4250 50  0001 C CNN
	1    4850 4250
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR1
U 1 1 5B4BC0FB
P 3250 3100
F 0 "#PWR1" H 3250 2850 50  0001 C CNN
F 1 "GND" H 3250 2950 50  0000 C CNN
F 2 "" H 3250 3100 50  0001 C CNN
F 3 "" H 3250 3100 50  0001 C CNN
	1    3250 3100
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR2
U 1 1 5B4BC1F5
P 4250 1600
F 0 "#PWR2" H 4250 1350 50  0001 C CNN
F 1 "GND" H 4250 1450 50  0000 C CNN
F 2 "" H 4250 1600 50  0001 C CNN
F 3 "" H 4250 1600 50  0001 C CNN
	1    4250 1600
	1    0    0    -1  
$EndComp
Text GLabel 5600 5200 3    60   Input ~ 0
SDA
Text GLabel 5700 5200 3    60   Input ~ 0
SCL
Text GLabel 2850 2500 3    60   Input ~ 0
SDA
Text GLabel 2950 2500 3    60   Input ~ 0
SCL
$Comp
L power:GND #PWR8
U 1 1 5B4BC825
P 8600 5000
F 0 "#PWR8" H 8600 4750 50  0001 C CNN
F 1 "GND" H 8600 4850 50  0000 C CNN
F 2 "" H 8600 5000 50  0001 C CNN
F 3 "" H 8600 5000 50  0001 C CNN
	1    8600 5000
	1    0    0    -1  
$EndComp
Text GLabel 9050 5700 2    60   Input ~ 0
VCC
NoConn ~ 3150 2500
NoConn ~ 3250 2500
NoConn ~ 6300 3850
NoConn ~ 6300 4050
NoConn ~ 6300 4150
NoConn ~ 6300 4250
NoConn ~ 6300 4450
NoConn ~ 5200 3850
NoConn ~ 5200 3950
NoConn ~ 5200 4050
NoConn ~ 5200 4150
NoConn ~ 5200 4250
NoConn ~ 5200 4350
NoConn ~ 5200 4450
NoConn ~ 5200 4550
NoConn ~ 5200 4650
NoConn ~ 5200 4750
NoConn ~ 5800 5200
NoConn ~ 5900 5200
NoConn ~ 6300 3650
Text Notes 8150 7500 2    60   ~ 0
Flying Squirrel #3
Text Notes 8750 6700 2    60   ~ 0
FS#3 schematic by D. Gibson KJ6FO
NoConn ~ 5200 3650
Text Notes 5050 3550 2    60   ~ 0
** Note. Disconnect TX/RX line\nwhen programming Arduino.
$Comp
L ant_dipole:ANT_Dipole ANT1
U 1 1 5B4BCC0E
P 4200 1600
F 0 "ANT1" H 4200 1900 60  0000 C CNN
F 1 "ANT_Dipole" H 4250 2000 60  0000 C CNN
F 2 "" H 4300 1600 60  0001 C CNN
F 3 "" H 4300 1600 60  0001 C CNN
	1    4200 1600
	1    0    0    -1  
$EndComp
$Comp
L ds18s20:DS18S20 U5
U 1 1 5BD0F549
P 7950 4700
F 0 "U5" H 7950 4950 60  0000 C CNN
F 1 "DS18S20" H 7950 4850 60  0000 C CNN
F 2 "" H 7950 4700 60  0001 C CNN
F 3 "" H 7950 4700 60  0001 C CNN
	1    7950 4700
	1    0    0    -1  
$EndComp
Wire Wire Line
	6300 3750 6550 3750
Wire Wire Line
	3800 3750 5200 3750
Wire Wire Line
	4750 4250 4850 4250
Wire Wire Line
	3800 3850 4200 3850
Wire Wire Line
	4200 3850 4200 4250
Wire Wire Line
	4200 4250 4350 4250
Wire Wire Line
	6300 4750 6500 4750
Wire Wire Line
	6500 4750 6500 5550
Wire Wire Line
	6500 5550 4550 5550
Wire Wire Line
	4550 5550 4550 4550
Wire Wire Line
	2750 2500 2750 3100
Wire Wire Line
	3150 3100 3250 3100
Wire Wire Line
	2950 3400 2950 5750
Wire Wire Line
	2950 5750 6650 5750
Wire Wire Line
	6650 5750 6650 4650
Wire Wire Line
	6650 4650 6300 4650
Wire Wire Line
	3050 2500 3050 2600
Wire Wire Line
	3050 2600 4150 2600
Wire Wire Line
	4150 2600 4150 1600
Wire Wire Line
	6300 4550 6800 4550
Wire Wire Line
	6800 4550 6800 5350
Wire Wire Line
	6000 2000 6000 1900
Wire Wire Line
	6300 3950 7200 3950
Wire Wire Line
	7200 4000 7200 3950
Connection ~ 7200 3950
Wire Wire Line
	6300 4350 7200 4350
Connection ~ 7200 4350
Wire Wire Line
	7200 4300 7200 4350
Wire Wire Line
	6800 5350 7950 5350
Wire Wire Line
	8750 5000 8750 5350
Wire Wire Line
	8750 5700 8900 5700
Wire Wire Line
	8900 5000 8900 5700
Connection ~ 8900 5700
Connection ~ 8750 5350
Wire Wire Line
	6000 1900 5700 1900
$Comp
L power:GND #PWR?
U 1 1 5BD0F700
P 7800 5050
F 0 "#PWR?" H 7800 4800 50  0001 C CNN
F 1 "GND" H 7800 4900 50  0000 C CNN
F 2 "" H 7800 5050 50  0001 C CNN
F 3 "" H 7800 5050 50  0001 C CNN
	1    7800 5050
	1    0    0    -1  
$EndComp
Text GLabel 8200 5000 2    60   Input ~ 0
VCC
Wire Wire Line
	7800 5000 7800 5050
Wire Wire Line
	8100 5000 8200 5000
Wire Wire Line
	7950 5000 7950 5350
Connection ~ 7950 5350
Wire Wire Line
	7200 3950 7400 3950
Wire Wire Line
	7200 4350 7200 4450
Wire Wire Line
	8900 5700 9050 5700
Wire Wire Line
	8750 5350 8750 5400
Wire Wire Line
	7950 5350 8750 5350
Wire Wire Line
	5200 4750 5050 4750
Wire Wire Line
	5050 4750 5050 3650
Wire Wire Line
	5050 3650 3800 3650
$EndSCHEMATC
