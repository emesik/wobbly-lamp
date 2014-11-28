EESchema Schematic File Version 2
LIBS:power
LIBS:device
LIBS:transistors
LIBS:conn
LIBS:linear
LIBS:regul
LIBS:74xx
LIBS:cmos4000
LIBS:adc-dac
LIBS:memory
LIBS:xilinx
LIBS:special
LIBS:microcontrollers
LIBS:dsp
LIBS:microchip
LIBS:analog_switches
LIBS:motorola
LIBS:texas
LIBS:intel
LIBS:audio
LIBS:interface
LIBS:digital-audio
LIBS:philips
LIBS:display
LIBS:cypress
LIBS:siliconi
LIBS:opto
LIBS:atmel
LIBS:contrib
LIBS:valves
LIBS:wobbly-lamp-cache
EELAYER 25 0
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
Text GLabel 4700 3850 2    39   Input ~ 0
X
Text GLabel 4700 3750 2    39   Input ~ 0
Y
Text GLabel 4700 3650 2    39   Input ~ 0
Z
Text GLabel 2150 4600 0    39   Output ~ 0
gS
Text GLabel 4700 4750 2    39   Input ~ 0
0g
$Comp
L +12V #PWR01
U 1 1 546B78C4
P 2300 1000
F 0 "#PWR01" H 2300 950 20  0001 C CNN
F 1 "+12V" H 2300 1100 30  0000 C CNN
F 2 "" H 2300 1000 60  0000 C CNN
F 3 "" H 2300 1000 60  0000 C CNN
	1    2300 1000
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR02
U 1 1 546B78E9
P 3000 1500
F 0 "#PWR02" H 3000 1500 30  0001 C CNN
F 1 "GND" H 3000 1430 30  0001 C CNN
F 2 "" H 3000 1500 60  0000 C CNN
F 3 "" H 3000 1500 60  0000 C CNN
	1    3000 1500
	1    0    0    -1  
$EndComp
$Comp
L INDUCTOR_SMALL L1
U 1 1 546B7AF8
P 1400 3150
F 0 "L1" H 1400 3250 50  0000 C CNN
F 1 "10u" H 1400 3100 50  0000 C CNN
F 2 "SMD_Packages:SMD-0805" H 1400 3150 60  0001 C CNN
F 3 "" H 1400 3150 60  0000 C CNN
	1    1400 3150
	-1   0    0    1   
$EndComp
$Comp
L C C5
U 1 1 546B7B39
P 3150 2400
F 0 "C5" H 3150 2500 40  0000 L CNN
F 1 "100n" H 3156 2315 40  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 3188 2250 30  0001 C CNN
F 3 "" H 3150 2400 60  0000 C CNN
	1    3150 2400
	1    0    0    -1  
$EndComp
$Comp
L C C4
U 1 1 546B7B82
P 2250 3550
F 0 "C4" H 2250 3650 40  0000 L CNN
F 1 "100n" H 2256 3465 40  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 2288 3400 30  0001 C CNN
F 3 "" H 2250 3550 60  0000 C CNN
	1    2250 3550
	1    0    0    -1  
$EndComp
$Comp
L C C3
U 1 1 546B7BED
P 1750 3350
F 0 "C3" H 1750 3450 40  0000 L CNN
F 1 "100n" H 1756 3265 40  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 1788 3200 30  0001 C CNN
F 3 "" H 1750 3350 60  0000 C CNN
	1    1750 3350
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR03
U 1 1 546B7C50
P 2250 3850
F 0 "#PWR03" H 2250 3850 30  0001 C CNN
F 1 "GND" H 2250 3780 30  0001 C CNN
F 2 "" H 2250 3850 60  0000 C CNN
F 3 "" H 2250 3850 60  0000 C CNN
	1    2250 3850
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR04
U 1 1 546B7CBD
P 1750 3850
F 0 "#PWR04" H 1750 3850 30  0001 C CNN
F 1 "GND" H 1750 3780 30  0001 C CNN
F 2 "" H 1750 3850 60  0000 C CNN
F 3 "" H 1750 3850 60  0000 C CNN
	1    1750 3850
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR05
U 1 1 546B7CE9
P 2450 3550
F 0 "#PWR05" H 2450 3550 30  0001 C CNN
F 1 "GND" H 2450 3480 30  0001 C CNN
F 2 "" H 2450 3550 60  0000 C CNN
F 3 "" H 2450 3550 60  0000 C CNN
	1    2450 3550
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR06
U 1 1 546B7D16
P 3150 2700
F 0 "#PWR06" H 3150 2700 30  0001 C CNN
F 1 "GND" H 3150 2630 30  0001 C CNN
F 2 "" H 3150 2700 60  0000 C CNN
F 3 "" H 3150 2700 60  0000 C CNN
	1    3150 2700
	1    0    0    -1  
$EndComp
$Comp
L R R1
U 1 1 546B7E92
P 2450 2600
F 0 "R1" V 2530 2600 40  0000 C CNN
F 1 "22k" V 2457 2601 40  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 2380 2600 30  0001 C CNN
F 3 "" H 2450 2600 30  0000 C CNN
	1    2450 2600
	1    0    0    -1  
$EndComp
$Comp
L CONN_02X05 ISP1
U 1 1 546B807E
P 9150 5300
F 0 "ISP1" H 9150 5600 50  0000 C CNN
F 1 "IDC10" H 9150 5000 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_2x05" H 9150 4100 60  0001 C CNN
F 3 "" H 9150 4100 60  0000 C CNN
	1    9150 5300
	1    0    0    -1  
$EndComp
Text GLabel 2300 2950 0    39   Input ~ 0
RST
Text GLabel 8650 5100 0    39   Output ~ 0
MOSI
Text GLabel 8650 5300 0    39   Output ~ 0
RST
Text GLabel 8650 5400 0    39   Output ~ 0
SCK
Text GLabel 8650 5500 0    39   Input ~ 0
MISO
$Comp
L GND #PWR07
U 1 1 546B86C3
P 9550 5650
F 0 "#PWR07" H 9550 5650 30  0001 C CNN
F 1 "GND" H 9550 5580 30  0001 C CNN
F 2 "" H 9550 5650 60  0000 C CNN
F 3 "" H 9550 5650 60  0000 C CNN
	1    9550 5650
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR08
U 1 1 546B88EA
P 3450 5750
F 0 "#PWR08" H 3450 5750 30  0001 C CNN
F 1 "GND" H 3450 5680 30  0001 C CNN
F 2 "" H 3450 5750 60  0000 C CNN
F 3 "" H 3450 5750 60  0000 C CNN
	1    3450 5750
	1    0    0    -1  
$EndComp
$Comp
L CONN_01X05 PL1
U 1 1 546B89E7
P 6500 1200
F 0 "PL1" H 6500 1500 50  0000 C CNN
F 1 "PIN_5" V 6600 1200 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x05" H 6500 1200 60  0001 C CNN
F 3 "" H 6500 1200 60  0000 C CNN
	1    6500 1200
	1    0    0    -1  
$EndComp
$Comp
L CONN_01X05 PR1
U 1 1 546B8A20
P 7600 1200
F 0 "PR1" H 7600 1500 50  0000 C CNN
F 1 "PIN_5" V 7700 1200 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x05" H 7600 1200 60  0001 C CNN
F 3 "" H 7600 1200 60  0000 C CNN
	1    7600 1200
	1    0    0    -1  
$EndComp
Text GLabel 6100 1000 0    39   Output ~ 0
X
Text GLabel 6100 1100 0    39   Output ~ 0
Y
Text GLabel 6100 1200 0    39   Output ~ 0
Z
Text GLabel 6100 1400 0    39   Output ~ 0
0g
$Comp
L GND #PWR09
U 1 1 546B8D2B
P 7000 1650
F 0 "#PWR09" H 7000 1650 30  0001 C CNN
F 1 "GND" H 7000 1580 30  0001 C CNN
F 2 "" H 7000 1650 60  0000 C CNN
F 3 "" H 7000 1650 60  0000 C CNN
	1    7000 1650
	1    0    0    -1  
$EndComp
Text GLabel 7200 1300 0    39   Input ~ 0
gS
$Comp
L R R2
U 1 1 546B8EDE
P 5850 1300
F 0 "R2" V 5930 1300 40  0000 C CNN
F 1 "22k" V 5857 1301 40  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 5780 1300 30  0001 C CNN
F 3 "" H 5850 1300 30  0000 C CNN
	1    5850 1300
	0    1    1    0   
$EndComp
Text GLabel 4700 3450 2    39   Input ~ 0
SCK
Text GLabel 4700 3350 2    39   Output ~ 0
MISO
Text GLabel 4700 3250 2    39   Input ~ 0
MOSI
$Comp
L MOSFET_N Q1
U 1 1 546B9812
P 6650 3050
F 0 "Q1" H 6660 3220 60  0000 R CNN
F 1 "IRLML0030TRPbF" V 6850 3150 60  0000 R CNN
F 2 "Transistors_SMD:SOT23_FET-D+GS" H 6650 3050 60  0001 C CNN
F 3 "" H 6650 3050 60  0000 C CNN
	1    6650 3050
	1    0    0    -1  
$EndComp
$Comp
L MOSFET_N Q2
U 1 1 546B9A81
P 6650 4150
F 0 "Q2" H 6660 4320 60  0000 R CNN
F 1 "IRLML0030TRPbF" V 6850 4250 60  0000 R CNN
F 2 "Transistors_SMD:SOT23_FET-D+GS" H 6650 4150 60  0001 C CNN
F 3 "" H 6650 4150 60  0000 C CNN
	1    6650 4150
	1    0    0    -1  
$EndComp
$Comp
L MOSFET_N Q3
U 1 1 546B9D28
P 6650 5250
F 0 "Q3" H 6660 5420 60  0000 R CNN
F 1 "IRLML0030TRPbF" V 6850 5350 60  0000 R CNN
F 2 "Transistors_SMD:SOT23_FET-D+GS" H 6650 5250 60  0001 C CNN
F 3 "" H 6650 5250 60  0000 C CNN
	1    6650 5250
	1    0    0    -1  
$EndComp
$Comp
L R R3
U 1 1 546BA21E
P 5700 3050
F 0 "R3" V 5780 3050 40  0000 C CNN
F 1 "1k" V 5707 3051 40  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 5630 3050 30  0001 C CNN
F 3 "" H 5700 3050 30  0000 C CNN
	1    5700 3050
	0    1    1    0   
$EndComp
$Comp
L R R4
U 1 1 546BA358
P 5700 4150
F 0 "R4" V 5780 4150 40  0000 C CNN
F 1 "1k" V 5707 4151 40  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 5630 4150 30  0001 C CNN
F 3 "" H 5700 4150 30  0000 C CNN
	1    5700 4150
	0    1    1    0   
$EndComp
$Comp
L R R5
U 1 1 546BA39E
P 5700 5250
F 0 "R5" V 5780 5250 40  0000 C CNN
F 1 "1k" V 5707 5251 40  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 5630 5250 30  0001 C CNN
F 3 "" H 5700 5250 30  0000 C CNN
	1    5700 5250
	0    1    1    0   
$EndComp
Text GLabel 6850 2750 2    39   Output ~ 0
red
Text GLabel 6850 3850 2    39   Output ~ 0
green
Text GLabel 6850 4950 2    39   Output ~ 0
blue
$Comp
L GND #PWR010
U 1 1 546BAAC5
P 6450 3650
F 0 "#PWR010" H 6450 3650 30  0001 C CNN
F 1 "GND" H 6450 3580 30  0001 C CNN
F 2 "" H 6450 3650 60  0000 C CNN
F 3 "" H 6450 3650 60  0000 C CNN
	1    6450 3650
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR011
U 1 1 546BAAEA
P 6450 4750
F 0 "#PWR011" H 6450 4750 30  0001 C CNN
F 1 "GND" H 6450 4680 30  0001 C CNN
F 2 "" H 6450 4750 60  0000 C CNN
F 3 "" H 6450 4750 60  0000 C CNN
	1    6450 4750
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR012
U 1 1 546BAB08
P 6450 5850
F 0 "#PWR012" H 6450 5850 30  0001 C CNN
F 1 "GND" H 6450 5780 30  0001 C CNN
F 2 "" H 6450 5850 60  0000 C CNN
F 3 "" H 6450 5850 60  0000 C CNN
	1    6450 5850
	1    0    0    -1  
$EndComp
$Comp
L CONN_01X02 PWR1
U 1 1 546BB212
P 1100 1150
F 0 "PWR1" H 1100 1300 50  0000 C CNN
F 1 "CONN_01X02" V 1200 1150 50  0000 C CNN
F 2 "Connect:AK300-2" H 1100 1150 60  0001 C CNN
F 3 "" H 1100 1150 60  0000 C CNN
	1    1100 1150
	-1   0    0    -1  
$EndComp
$Comp
L +12V #PWR013
U 1 1 546BB3CE
P 1450 1050
F 0 "#PWR013" H 1450 1000 20  0001 C CNN
F 1 "+12V" H 1450 1150 30  0000 C CNN
F 2 "" H 1450 1050 60  0000 C CNN
F 3 "" H 1450 1050 60  0000 C CNN
	1    1450 1050
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR014
U 1 1 546BB44F
P 1450 1300
F 0 "#PWR014" H 1450 1300 30  0001 C CNN
F 1 "GND" H 1450 1230 30  0001 C CNN
F 2 "" H 1450 1300 60  0000 C CNN
F 3 "" H 1450 1300 60  0000 C CNN
	1    1450 1300
	1    0    0    -1  
$EndComp
NoConn ~ 2550 3850
NoConn ~ 4450 4950
NoConn ~ 7400 1400
NoConn ~ 8900 5200
NoConn ~ 4450 2950
NoConn ~ 4450 4150
NoConn ~ 4450 4350
NoConn ~ 4450 4850
$Comp
L CONN_01X04 P_RGB1
U 1 1 546BC0A9
P 10350 1250
F 0 "P_RGB1" H 10350 1500 50  0000 C CNN
F 1 "CONN_01X04" V 10450 1250 50  0000 C CNN
F 2 "Connect:AK300-4" H 10350 1250 60  0001 C CNN
F 3 "" H 10350 1250 60  0000 C CNN
	1    10350 1250
	1    0    0    -1  
$EndComp
Text GLabel 9950 1200 0    39   Input ~ 0
red
Text GLabel 9950 1300 0    39   Input ~ 0
green
Text GLabel 9950 1400 0    39   Input ~ 0
blue
$Comp
L R R6
U 1 1 546BCC52
P 6050 3300
F 0 "R6" V 6130 3300 40  0000 C CNN
F 1 "10k" V 6057 3301 40  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 5980 3300 30  0001 C CNN
F 3 "" H 6050 3300 30  0000 C CNN
	1    6050 3300
	1    0    0    -1  
$EndComp
$Comp
L R R7
U 1 1 546BCD9C
P 6050 4400
F 0 "R7" V 6130 4400 40  0000 C CNN
F 1 "10k" V 6057 4401 40  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 5980 4400 30  0001 C CNN
F 3 "" H 6050 4400 30  0000 C CNN
	1    6050 4400
	1    0    0    -1  
$EndComp
$Comp
L R R8
U 1 1 546BCDE5
P 6050 5500
F 0 "R8" V 6130 5500 40  0000 C CNN
F 1 "10k" V 6057 5501 40  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 5980 5500 30  0001 C CNN
F 3 "" H 6050 5500 30  0000 C CNN
	1    6050 5500
	1    0    0    -1  
$EndComp
$Comp
L +12V #PWR015
U 1 1 546D3F67
P 9950 1100
F 0 "#PWR015" H 9950 1050 20  0001 C CNN
F 1 "+12V" H 9950 1200 30  0000 C CNN
F 2 "" H 9950 1100 60  0000 C CNN
F 3 "" H 9950 1100 60  0000 C CNN
	1    9950 1100
	1    0    0    -1  
$EndComp
$Comp
L ATMEGA8-AI IC1
U 1 1 546FA741
P 3450 3950
F 0 "IC1" H 2700 5150 40  0000 L BNN
F 1 "ATMEGA8-AI" H 3950 2400 40  0000 L BNN
F 2 "SMD_Packages:TQFP-32" H 3450 3950 30  0000 C CIN
F 3 "" H 3450 3950 60  0000 C CNN
	1    3450 3950
	1    0    0    -1  
$EndComp
Wire Wire Line
	4450 3650 4700 3650
Wire Wire Line
	4450 3750 4700 3750
Wire Wire Line
	4450 3850 4700 3850
Wire Wire Line
	4450 4750 4700 4750
Wire Wire Line
	3000 1300 3000 1500
Wire Wire Line
	2500 1400 3500 1400
Connection ~ 3000 1400
Wire Wire Line
	3400 1000 3750 1000
Connection ~ 3500 1000
Wire Wire Line
	1650 3150 2550 3150
Wire Wire Line
	2450 3550 2450 3350
Wire Wire Line
	2450 3350 2550 3350
Wire Wire Line
	3150 2600 3150 2700
Wire Wire Line
	2450 2850 2450 2950
Wire Wire Line
	2300 2950 2550 2950
Wire Wire Line
	2450 2150 2450 2350
Connection ~ 2450 2950
Wire Wire Line
	8900 5100 8650 5100
Wire Wire Line
	8900 5300 8650 5300
Wire Wire Line
	8900 5400 8650 5400
Wire Wire Line
	8900 5500 8650 5500
Wire Wire Line
	9550 5000 9550 5100
Wire Wire Line
	9550 5100 9400 5100
Wire Wire Line
	9550 5200 9550 5650
Wire Wire Line
	9550 5500 9400 5500
Wire Wire Line
	9550 5400 9400 5400
Connection ~ 9550 5500
Wire Wire Line
	9550 5300 9400 5300
Connection ~ 9550 5400
Wire Wire Line
	9550 5200 9400 5200
Connection ~ 9550 5300
Wire Wire Line
	6300 1000 6100 1000
Wire Wire Line
	6300 1100 6100 1100
Wire Wire Line
	6300 1200 6100 1200
Wire Wire Line
	6300 1400 6100 1400
Wire Wire Line
	7400 1200 7000 1200
Wire Wire Line
	7000 1200 7000 1650
Wire Wire Line
	7400 1300 7200 1300
Wire Wire Line
	6100 1300 6300 1300
Wire Wire Line
	5550 1200 5550 1300
Wire Wire Line
	5550 1300 5600 1300
Wire Wire Line
	4700 3350 4450 3350
Wire Wire Line
	4450 3450 4700 3450
Wire Wire Line
	5950 3050 6450 3050
Wire Wire Line
	5950 4150 6450 4150
Wire Wire Line
	5950 5250 6450 5250
Wire Wire Line
	6850 2750 6750 2750
Wire Wire Line
	6750 2750 6750 2850
Wire Wire Line
	6850 3850 6750 3850
Wire Wire Line
	6750 3850 6750 3950
Wire Wire Line
	6850 4950 6750 4950
Wire Wire Line
	6750 4950 6750 5050
Wire Wire Line
	6750 5800 6750 5450
Wire Wire Line
	6750 4700 6750 4350
Wire Wire Line
	6750 3600 6750 3250
Wire Wire Line
	1450 1050 1450 1100
Wire Wire Line
	1450 1100 1300 1100
Wire Wire Line
	1450 1300 1450 1200
Wire Wire Line
	1450 1200 1300 1200
Wire Wire Line
	10150 1100 9950 1100
Wire Wire Line
	10150 1200 9950 1200
Wire Wire Line
	10150 1300 9950 1300
Wire Wire Line
	9950 1400 10150 1400
Wire Wire Line
	2300 1000 2600 1000
Connection ~ 2500 1000
Wire Wire Line
	4450 3250 4700 3250
Wire Wire Line
	4450 3050 5450 3050
Wire Wire Line
	4600 3150 4600 3250
Connection ~ 4600 3250
Wire Wire Line
	4600 3150 5200 3150
Wire Wire Line
	5300 3100 4550 3100
Wire Wire Line
	4550 3100 4550 3150
Wire Wire Line
	4550 3150 4450 3150
Connection ~ 6050 3050
Wire Wire Line
	6050 3550 6050 3600
Wire Wire Line
	6050 3600 6750 3600
Wire Wire Line
	6450 3600 6450 3650
Connection ~ 6450 3600
Wire Wire Line
	5300 3100 5300 4150
Wire Wire Line
	5300 4150 5450 4150
Connection ~ 6050 4150
Wire Wire Line
	6050 4650 6050 4700
Wire Wire Line
	6050 4700 6750 4700
Wire Wire Line
	6450 4700 6450 4750
Connection ~ 6450 4700
Connection ~ 6050 5250
Wire Wire Line
	6050 5750 6050 5800
Wire Wire Line
	6050 5800 6750 5800
Wire Wire Line
	6450 5800 6450 5850
Connection ~ 6450 5800
Wire Wire Line
	5450 5250 5200 5250
Wire Wire Line
	5200 5250 5200 3150
Wire Wire Line
	3400 2050 3400 2650
Wire Wire Line
	3500 2150 3500 2650
NoConn ~ 4450 4550
NoConn ~ 4450 4250
NoConn ~ 4450 5150
NoConn ~ 4450 5250
Wire Wire Line
	3400 5550 3400 5600
Wire Wire Line
	3400 5600 3500 5600
Wire Wire Line
	3450 5600 3450 5750
Wire Wire Line
	3500 5600 3500 5550
Connection ~ 3450 5600
Wire Wire Line
	4950 2900 4900 2900
Wire Wire Line
	4900 2900 4900 3050
Connection ~ 4900 3050
Wire Wire Line
	5350 3350 5300 3350
Connection ~ 5300 3350
Wire Wire Line
	5350 4950 5200 4950
Connection ~ 5200 4950
NoConn ~ 4450 4650
Text GLabel 4950 2900 2    60   Output ~ 0
PWM_red
Text GLabel 5350 3350 2    60   Output ~ 0
PWM_green
Text GLabel 5350 4950 2    60   Output ~ 0
PWM_blue
$Comp
L +3,3V #PWR016
U 1 1 5471BFF3
P 7100 1100
F 0 "#PWR016" H 7100 1060 30  0001 C CNN
F 1 "+3,3V" H 7100 1210 30  0000 C CNN
F 2 "" H 7100 1100 60  0000 C CNN
F 3 "" H 7100 1100 60  0000 C CNN
	1    7100 1100
	1    0    0    -1  
$EndComp
Wire Wire Line
	7100 1100 7400 1100
$Comp
L +3,3V #PWR017
U 1 1 5471C1A1
P 5550 1200
F 0 "#PWR017" H 5550 1160 30  0001 C CNN
F 1 "+3,3V" H 5550 1310 30  0000 C CNN
F 2 "" H 5550 1200 60  0000 C CNN
F 3 "" H 5550 1200 60  0000 C CNN
	1    5550 1200
	1    0    0    -1  
$EndComp
Connection ~ 1750 3150
Wire Wire Line
	1750 3550 1750 3850
Wire Wire Line
	2450 2150 3500 2150
Connection ~ 3400 2150
Wire Wire Line
	3150 2150 3150 2200
Connection ~ 3150 2150
Wire Wire Line
	1100 3000 1100 3150
Wire Wire Line
	1100 3150 1150 3150
Wire Wire Line
	2550 3250 2250 3250
Wire Wire Line
	2250 3250 2250 3350
Wire Wire Line
	2250 3850 2250 3750
Wire Wire Line
	2450 4600 2150 4600
$Comp
L LD1117S33TR U1
U 1 1 54749FD9
P 3000 1050
F 0 "U1" H 3000 1300 40  0000 C CNN
F 1 "LD1117S33TR" H 3000 1250 40  0000 C CNN
F 2 "SMD_Packages:SOT-223" H 3000 1150 40  0000 C CNN
F 3 "" H 3000 1050 60  0000 C CNN
	1    3000 1050
	1    0    0    -1  
$EndComp
NoConn ~ 7400 1000
$Comp
L +3,3V #PWR018
U 1 1 5474A3EA
P 3750 1000
F 0 "#PWR018" H 3750 960 30  0001 C CNN
F 1 "+3,3V" H 3750 1110 30  0000 C CNN
F 2 "" H 3750 1000 60  0000 C CNN
F 3 "" H 3750 1000 60  0000 C CNN
	1    3750 1000
	1    0    0    -1  
$EndComp
$Comp
L +3,3V #PWR019
U 1 1 5474A514
P 3400 2050
F 0 "#PWR019" H 3400 2010 30  0001 C CNN
F 1 "+3,3V" H 3400 2160 30  0000 C CNN
F 2 "" H 3400 2050 60  0000 C CNN
F 3 "" H 3400 2050 60  0000 C CNN
	1    3400 2050
	1    0    0    -1  
$EndComp
$Comp
L +3,3V #PWR020
U 1 1 5474A553
P 1100 3000
F 0 "#PWR020" H 1100 2960 30  0001 C CNN
F 1 "+3,3V" H 1100 3110 30  0000 C CNN
F 2 "" H 1100 3000 60  0000 C CNN
F 3 "" H 1100 3000 60  0000 C CNN
	1    1100 3000
	1    0    0    -1  
$EndComp
$Comp
L R R9
U 1 1 5471D903
P 2450 4200
F 0 "R9" V 2530 4200 40  0000 C CNN
F 1 "10k" V 2457 4201 40  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 2380 4200 30  0001 C CNN
F 3 "" H 2450 4200 30  0000 C CNN
	1    2450 4200
	1    0    0    -1  
$EndComp
Wire Wire Line
	2450 4600 2450 4450
$Comp
L +3,3V #PWR021
U 1 1 5474ACDD
P 9550 5000
F 0 "#PWR021" H 9550 4960 30  0001 C CNN
F 1 "+3,3V" H 9550 5110 30  0000 C CNN
F 2 "" H 9550 5000 60  0000 C CNN
F 3 "" H 9550 5000 60  0000 C CNN
	1    9550 5000
	1    0    0    -1  
$EndComp
$Comp
L CP2 C1
U 1 1 5474B742
P 2500 1200
F 0 "C1" H 2500 1300 40  0000 L CNN
F 1 "10u" H 2506 1115 40  0000 L CNN
F 2 "Capacitors_ThroughHole:Capacitor5x6RM2.5" H 2538 1050 30  0001 C CNN
F 3 "" H 2500 1200 60  0000 C CNN
	1    2500 1200
	1    0    0    -1  
$EndComp
$Comp
L CP2 C2
U 1 1 5474B848
P 3500 1200
F 0 "C2" H 3500 1300 40  0000 L CNN
F 1 "10u" H 3506 1115 40  0000 L CNN
F 2 "Capacitors_ThroughHole:Capacitor5x6RM2.5" H 3538 1050 30  0001 C CNN
F 3 "" H 3500 1200 60  0000 C CNN
	1    3500 1200
	1    0    0    -1  
$EndComp
Wire Wire Line
	2450 3950 2450 3650
Wire Wire Line
	2450 3650 2550 3650
$Comp
L JUMPER JP1
U 1 1 5475137D
P 5150 5900
F 0 "JP1" H 5150 6050 60  0000 C CNN
F 1 "WOB_SW" H 5150 5820 40  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x02" H 5150 5900 60  0001 C CNN
F 3 "" H 5150 5900 60  0000 C CNN
	1    5150 5900
	1    0    0    -1  
$EndComp
Wire Wire Line
	4850 5900 4800 5900
Wire Wire Line
	4800 5050 4800 6000
Wire Wire Line
	4800 5050 4450 5050
$Comp
L +3,3V #PWR022
U 1 1 547516A7
P 5500 5700
F 0 "#PWR022" H 5500 5660 30  0001 C CNN
F 1 "+3,3V" H 5500 5810 30  0000 C CNN
F 2 "" H 5500 5700 60  0000 C CNN
F 3 "" H 5500 5700 60  0000 C CNN
	1    5500 5700
	1    0    0    -1  
$EndComp
Wire Wire Line
	5500 5700 5500 5900
Wire Wire Line
	5500 5900 5450 5900
$Comp
L R R10
U 1 1 5475179B
P 4800 6250
F 0 "R10" V 4880 6250 40  0000 C CNN
F 1 "22k" V 4807 6251 40  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 4730 6250 30  0001 C CNN
F 3 "" H 4800 6250 30  0000 C CNN
	1    4800 6250
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR023
U 1 1 547517D5
P 4800 6600
F 0 "#PWR023" H 4800 6600 30  0001 C CNN
F 1 "GND" H 4800 6530 30  0001 C CNN
F 2 "" H 4800 6600 60  0000 C CNN
F 3 "" H 4800 6600 60  0000 C CNN
	1    4800 6600
	1    0    0    -1  
$EndComp
Wire Wire Line
	4800 6600 4800 6500
Connection ~ 4800 5900
Wire Wire Line
	4450 3950 4700 3950
Wire Wire Line
	4450 4050 4700 4050
Text GLabel 4700 3950 2    60   Input ~ 0
pot1
Text GLabel 4700 4050 2    60   Input ~ 0
pot2
$Comp
L CONN_01X03 POT2
U 1 1 54751FF2
P 9100 3850
F 0 "POT2" H 9100 4050 50  0000 C CNN
F 1 "CONN_01X03" V 9200 3850 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x03" H 9100 3850 60  0001 C CNN
F 3 "" H 9100 3850 60  0000 C CNN
	1    9100 3850
	1    0    0    -1  
$EndComp
$Comp
L +3,3V #PWR024
U 1 1 547520CD
P 9100 2200
F 0 "#PWR024" H 9100 2160 30  0001 C CNN
F 1 "+3,3V" H 9100 2310 30  0000 C CNN
F 2 "" H 9100 2200 60  0000 C CNN
F 3 "" H 9100 2200 60  0000 C CNN
	1    9100 2200
	1    0    0    -1  
$EndComp
$Comp
L +3,3V #PWR025
U 1 1 547521C0
P 8750 3650
F 0 "#PWR025" H 8750 3610 30  0001 C CNN
F 1 "+3,3V" H 8750 3760 30  0000 C CNN
F 2 "" H 8750 3650 60  0000 C CNN
F 3 "" H 8750 3650 60  0000 C CNN
	1    8750 3650
	1    0    0    -1  
$EndComp
Wire Wire Line
	8750 3650 8750 3750
Wire Wire Line
	8750 3750 8900 3750
$Comp
L GND #PWR026
U 1 1 5475232E
P 9100 3100
F 0 "#PWR026" H 9100 3100 30  0001 C CNN
F 1 "GND" H 9100 3030 30  0001 C CNN
F 2 "" H 9100 3100 60  0000 C CNN
F 3 "" H 9100 3100 60  0000 C CNN
	1    9100 3100
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR027
U 1 1 547523FC
P 8750 4050
F 0 "#PWR027" H 8750 4050 30  0001 C CNN
F 1 "GND" H 8750 3980 30  0001 C CNN
F 2 "" H 8750 4050 60  0000 C CNN
F 3 "" H 8750 4050 60  0000 C CNN
	1    8750 4050
	1    0    0    -1  
$EndComp
Wire Wire Line
	8750 4050 8750 3950
Wire Wire Line
	8750 3950 8900 3950
Wire Wire Line
	8900 3850 8750 3850
Wire Wire Line
	8950 2650 8800 2650
Text GLabel 8800 2650 0    60   Output ~ 0
pot1
Text GLabel 8750 3850 0    60   Output ~ 0
pot2
$Comp
L POT RV1
U 1 1 54752ED3
P 9100 2650
F 0 "RV1" H 9100 2550 50  0000 C CNN
F 1 "22k" H 9100 2650 50  0000 C CNN
F 2 "Potentiometers:Potentiometer_VishaySpectrol-Econtrim-Type36T" H 9100 2650 60  0001 C CNN
F 3 "" H 9100 2650 60  0000 C CNN
	1    9100 2650
	0    -1   1    0   
$EndComp
Wire Wire Line
	9100 2200 9100 2400
Wire Wire Line
	9100 2900 9100 3100
$EndSCHEMATC