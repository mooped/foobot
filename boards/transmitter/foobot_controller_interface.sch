EESchema Schematic File Version 2  date 20/03/2015 20:59:18
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
LIBS:minimus
LIBS:foobot_controller_interface-cache
EELAYER 27 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title ""
Date "20 mar 2015"
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L 74LS165 U2
U 1 1 548B3434
P 4600 2400
F 0 "U2" H 4750 2350 60  0000 C CNN
F 1 "74LS165" H 4750 2150 60  0000 C CNN
F 2 "~" H 4600 2400 60  0000 C CNN
F 3 "~" H 4600 2400 60  0000 C CNN
	1    4600 2400
	1    0    0    -1  
$EndComp
$Comp
L CONN_7 P3
U 1 1 548B343A
P 950 3450
F 0 "P3" V 920 3450 60  0000 C CNN
F 1 "NES" V 1020 3450 60  0000 C CNN
F 2 "" H 950 3450 60  0000 C CNN
F 3 "" H 950 3450 60  0000 C CNN
	1    950  3450
	-1   0    0    1   
$EndComp
Text Label 2550 2200 2    60   ~ 0
MS_UP
Text Label 2550 2100 2    60   ~ 0
MS_DOWN
Text Label 2550 2000 2    60   ~ 0
MS_LEFT
Text Label 2550 1900 2    60   ~ 0
MS_RIGHT
$Comp
L GND #PWR01
U 1 1 548B3444
P 1400 3950
F 0 "#PWR01" H 1400 3950 30  0001 C CNN
F 1 "GND" H 1400 3880 30  0001 C CNN
F 2 "" H 1400 3950 60  0000 C CNN
F 3 "" H 1400 3950 60  0000 C CNN
	1    1400 3950
	1    0    0    -1  
$EndComp
Text Label 2550 2500 2    60   ~ 0
MS_1
Text Label 2550 2600 2    60   ~ 0
MS_2
Text Label 2550 2900 2    60   ~ 0
SHIFT
Text Label 1600 3550 0    60   ~ 0
LATCH
$Comp
L VCC #PWR02
U 1 1 548B344E
P 1400 3050
F 0 "#PWR02" H 1400 3150 30  0001 C CNN
F 1 "VCC" H 1400 3150 30  0000 C CNN
F 2 "" H 1400 3050 60  0000 C CNN
F 3 "" H 1400 3050 60  0000 C CNN
	1    1400 3050
	1    0    0    -1  
$EndComp
Text Label 5400 1900 0    60   ~ 0
DATA
Text Label 1600 3450 0    60   ~ 0
NES_DATA
$Comp
L R_PACK8_COMMON RP1
U 1 1 548B3459
P 3200 1500
F 0 "RP1" H 3200 1950 40  0000 C CNN
F 1 "R_PACK8_COMMON" V 3350 1500 40  0000 C CNN
F 2 "~" H 3200 1500 60  0000 C CNN
F 3 "~" H 3200 1500 60  0000 C CNN
	1    3200 1500
	0    1    -1   0   
$EndComp
Text Label 2550 1800 2    60   ~ 0
NES_DATA
Text Label 2550 2750 2    60   ~ 0
INVERSE_LATCH
Text Label 1600 3650 0    60   ~ 0
SHIFT
$Comp
L GND #PWR03
U 1 1 548B3462
P 2550 3100
F 0 "#PWR03" H 2550 3100 30  0001 C CNN
F 1 "GND" H 2550 3030 30  0001 C CNN
F 2 "" H 2550 3100 60  0000 C CNN
F 3 "" H 2550 3100 60  0000 C CNN
	1    2550 3100
	1    0    0    -1  
$EndComp
NoConn ~ 1300 3350
$Comp
L CONN_8 P2
U 1 1 548B346A
P 900 2250
F 0 "P2" V 850 2250 60  0000 C CNN
F 1 "SMS" V 950 2250 60  0000 C CNN
F 2 "" H 900 2250 60  0000 C CNN
F 3 "" H 900 2250 60  0000 C CNN
	1    900  2250
	-1   0    0    -1  
$EndComp
$Comp
L GND #PWR04
U 1 1 548B3470
P 1400 2700
F 0 "#PWR04" H 1400 2700 30  0001 C CNN
F 1 "GND" H 1400 2630 30  0001 C CNN
F 2 "" H 1400 2700 60  0000 C CNN
F 3 "" H 1400 2700 60  0000 C CNN
	1    1400 2700
	1    0    0    -1  
$EndComp
$Comp
L VCC #PWR05
U 1 1 548B3476
P 3750 1600
F 0 "#PWR05" H 3750 1700 30  0001 C CNN
F 1 "VCC" H 3750 1700 30  0000 C CNN
F 2 "" H 3750 1600 60  0000 C CNN
F 3 "" H 3750 1600 60  0000 C CNN
	1    3750 1600
	1    0    0    -1  
$EndComp
Text Label 1450 2300 0    60   ~ 0
MS_UP
Text Label 1450 2400 0    60   ~ 0
MS_DOWN
Text Label 1450 2500 0    60   ~ 0
MS_LEFT
Text Label 1450 2600 0    60   ~ 0
MS_RIGHT
Text Label 1450 2200 0    60   ~ 0
MS_1
Text Label 1450 1900 0    60   ~ 0
MS_2
$Comp
L CONN_1 P11
U 1 1 548B4969
P 2400 2300
F 0 "P11" H 2480 2300 40  0000 L CNN
F 1 "AUX" H 2400 2355 30  0001 C CNN
F 2 "" H 2400 2300 60  0000 C CNN
F 3 "" H 2400 2300 60  0000 C CNN
	1    2400 2300
	-1   0    0    1   
$EndComp
$Comp
L CONN_1 P12
U 1 1 548B4974
P 2400 2400
F 0 "P12" H 2480 2400 40  0000 L CNN
F 1 "AUX" H 2400 2455 30  0001 C CNN
F 2 "" H 2400 2400 60  0000 C CNN
F 3 "" H 2400 2400 60  0000 C CNN
	1    2400 2400
	-1   0    0    1   
$EndComp
$Comp
L CONN_1 P15
U 1 1 548DA2E6
P 5550 2000
F 0 "P15" H 5630 2000 40  0000 L CNN
F 1 "¬Q7" H 5550 2055 30  0001 C CNN
F 2 "" H 5550 2000 60  0000 C CNN
F 3 "" H 5550 2000 60  0000 C CNN
	1    5550 2000
	1    0    0    -1  
$EndComp
Text Label 7400 2450 2    60   ~ 0
LATCH
Text Label 8600 2150 0    60   ~ 0
INVERSE_LATCH
$Comp
L VCC #PWR06
U 1 1 548DBEB5
P 8400 1450
F 0 "#PWR06" H 8400 1550 30  0001 C CNN
F 1 "VCC" H 8400 1550 30  0000 C CNN
F 2 "" H 8400 1450 60  0000 C CNN
F 3 "" H 8400 1450 60  0000 C CNN
	1    8400 1450
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR07
U 1 1 548DBEBB
P 8400 3100
F 0 "#PWR07" H 8400 3100 30  0001 C CNN
F 1 "GND" H 8400 3030 30  0001 C CNN
F 2 "" H 8400 3100 60  0000 C CNN
F 3 "" H 8400 3100 60  0000 C CNN
	1    8400 3100
	1    0    0    -1  
$EndComp
$Comp
L NPN Q2
U 1 1 548DBEC1
P 8300 2450
F 0 "Q2" H 8300 2300 50  0000 R CNN
F 1 "NPN" H 8300 2600 50  0000 R CNN
F 2 "~" H 8300 2450 60  0000 C CNN
F 3 "~" H 8300 2450 60  0000 C CNN
	1    8300 2450
	1    0    0    -1  
$EndComp
$Comp
L R R4
U 1 1 548DBEC7
P 8400 1800
F 0 "R4" V 8480 1800 40  0000 C CNN
F 1 "6.7k" V 8407 1801 40  0000 C CNN
F 2 "~" V 8330 1800 30  0000 C CNN
F 3 "~" H 8400 1800 30  0000 C CNN
	1    8400 1800
	1    0    0    -1  
$EndComp
$Comp
L R R1
U 1 1 548DBECD
P 7750 2450
F 0 "R1" V 7830 2450 40  0000 C CNN
F 1 "10k" V 7757 2451 40  0000 C CNN
F 2 "~" V 7680 2450 30  0000 C CNN
F 3 "~" H 7750 2450 30  0000 C CNN
	1    7750 2450
	0    -1   -1   0   
$EndComp
$Comp
L CONN_1 P39
U 1 1 548DC170
P 9600 2150
F 0 "P39" H 9680 2150 40  0000 L CNN
F 1 "¬LATCH" H 9600 2205 30  0001 C CNN
F 2 "" H 9600 2150 60  0000 C CNN
F 3 "" H 9600 2150 60  0000 C CNN
	1    9600 2150
	1    0    0    -1  
$EndComp
Text Label 1350 850  0    60   ~ 0
VCC
Text Label 1350 950  0    60   ~ 0
LATCH
Text Label 1350 1050 0    60   ~ 0
SHIFT
Text Label 1350 1150 0    60   ~ 0
DATA
$Comp
L GND #PWR08
U 1 1 548DC350
P 1450 1450
F 0 "#PWR08" H 1450 1450 30  0001 C CNN
F 1 "GND" H 1450 1380 30  0001 C CNN
F 2 "" H 1450 1450 60  0000 C CNN
F 3 "" H 1450 1450 60  0000 C CNN
	1    1450 1450
	1    0    0    -1  
$EndComp
$Comp
L CONN_6 P35
U 1 1 548DC356
P 900 1100
F 0 "P35" V 850 1100 60  0000 C CNN
F 1 "CHANNEL_X" V 950 1100 60  0000 C CNN
F 2 "" H 900 1100 60  0000 C CNN
F 3 "" H 900 1100 60  0000 C CNN
	1    900  1100
	-1   0    0    -1  
$EndComp
$Comp
L CONN_1 P36
U 1 1 548DC372
P 1500 1250
F 0 "P36" H 1580 1250 40  0000 L CNN
F 1 "CONN_1" H 1500 1305 30  0001 C CNN
F 2 "" H 1500 1250 60  0000 C CNN
F 3 "" H 1500 1250 60  0000 C CNN
	1    1500 1250
	1    0    0    -1  
$EndComp
$Comp
L PWR_FLAG #FLG09
U 1 1 548DCF82
P 10750 6700
F 0 "#FLG09" H 10750 6795 30  0001 C CNN
F 1 "PWR_FLAG" H 10750 6880 30  0000 C CNN
F 2 "" H 10750 6700 60  0000 C CNN
F 3 "" H 10750 6700 60  0000 C CNN
	1    10750 6700
	1    0    0    -1  
$EndComp
NoConn ~ 1300 3250
$Comp
L CONN_1 P10
U 1 1 548B48DD
P 1550 2000
F 0 "P10" H 1630 2000 40  0000 L CNN
F 1 "AUX_IN" H 1550 2055 30  0001 C CNN
F 2 "" H 1550 2000 60  0000 C CNN
F 3 "" H 1550 2000 60  0000 C CNN
	1    1550 2000
	1    0    0    -1  
$EndComp
$Comp
L ATTINY13A-SS IC1
U 1 1 5495A1A5
P 2350 5400
F 0 "IC1" H 2450 5450 60  0000 C CNN
F 1 "ATTINY13A-SS" H 3650 4650 60  0000 C CNN
F 2 "SO8" H 2450 4650 60  0001 C CNN
F 3 "" H 2350 5400 60  0000 C CNN
	1    2350 5400
	1    0    0    -1  
$EndComp
$Comp
L C C2
U 1 1 5495A1B4
P 4250 5750
F 0 "C2" H 4250 5850 40  0000 L CNN
F 1 "C" H 4256 5665 40  0000 L CNN
F 2 "~" H 4288 5600 30  0000 C CNN
F 3 "~" H 4250 5750 60  0000 C CNN
	1    4250 5750
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR010
U 1 1 5495A3F4
P 4250 6200
F 0 "#PWR010" H 4250 6200 30  0001 C CNN
F 1 "GND" H 4250 6130 30  0001 C CNN
F 2 "" H 4250 6200 60  0000 C CNN
F 3 "" H 4250 6200 60  0000 C CNN
	1    4250 6200
	1    0    0    -1  
$EndComp
$Comp
L VCC #PWR011
U 1 1 5495A403
P 4250 5050
F 0 "#PWR011" H 4250 5150 30  0001 C CNN
F 1 "VCC" H 4250 5150 30  0000 C CNN
F 2 "" H 4250 5050 60  0000 C CNN
F 3 "" H 4250 5050 60  0000 C CNN
	1    4250 5050
	1    0    0    -1  
$EndComp
$Comp
L R R3
U 1 1 5495A5C9
P 3250 5150
F 0 "R3" V 3330 5150 40  0000 C CNN
F 1 "10k" V 3257 5151 40  0000 C CNN
F 2 "~" V 3180 5150 30  0000 C CNN
F 3 "~" H 3250 5150 30  0000 C CNN
	1    3250 5150
	0    -1   -1   0   
$EndComp
Text Label 3900 6650 0    60   ~ 0
SHIFT
$Comp
L GND #PWR012
U 1 1 5495AA8B
P 4000 7050
F 0 "#PWR012" H 4000 7050 30  0001 C CNN
F 1 "GND" H 4000 6980 30  0001 C CNN
F 2 "" H 4000 7050 60  0000 C CNN
F 3 "" H 4000 7050 60  0000 C CNN
	1    4000 7050
	1    0    0    -1  
$EndComp
$Comp
L CONN_6 P8
U 1 1 5495AA91
P 3450 6700
F 0 "P8" V 3400 6700 60  0000 C CNN
F 1 "CHANNEL_2" V 3500 6700 60  0000 C CNN
F 2 "" H 3450 6700 60  0000 C CNN
F 3 "" H 3450 6700 60  0000 C CNN
	1    3450 6700
	-1   0    0    -1  
$EndComp
$Comp
L CONN_3 K2
U 1 1 5495AB41
P 5250 6000
F 0 "K2" V 5200 6000 50  0000 C CNN
F 1 "TX" V 5300 6000 40  0000 C CNN
F 2 "" H 5250 6000 60  0000 C CNN
F 3 "" H 5250 6000 60  0000 C CNN
	1    5250 6000
	1    0    0    -1  
$EndComp
$Comp
L VCC #PWR013
U 1 1 5495AB47
P 4800 5750
F 0 "#PWR013" H 4800 5850 30  0001 C CNN
F 1 "VCC" H 4800 5850 30  0000 C CNN
F 2 "" H 4800 5750 60  0000 C CNN
F 3 "" H 4800 5750 60  0000 C CNN
	1    4800 5750
	1    0    0    -1  
$EndComp
Text Label 4700 5900 2    60   ~ 0
ATAD
$Comp
L GND #PWR014
U 1 1 5495AB4E
P 4800 6200
F 0 "#PWR014" H 4800 6200 30  0001 C CNN
F 1 "GND" H 4800 6130 30  0001 C CNN
F 2 "" H 4800 6200 60  0000 C CNN
F 3 "" H 4800 6200 60  0000 C CNN
	1    4800 6200
	1    0    0    -1  
$EndComp
$Comp
L CONN_2 P9
U 1 1 5495AB59
P 5250 5250
F 0 "P9" V 5200 5250 40  0000 C CNN
F 1 "MCU_PWR" V 5300 5250 40  0000 C CNN
F 2 "" H 5250 5250 60  0000 C CNN
F 3 "" H 5250 5250 60  0000 C CNN
	1    5250 5250
	1    0    0    -1  
$EndComp
$Comp
L VCC #PWR015
U 1 1 5495AB61
P 4800 5050
F 0 "#PWR015" H 4800 5150 30  0001 C CNN
F 1 "VCC" H 4800 5150 30  0000 C CNN
F 2 "" H 4800 5050 60  0000 C CNN
F 3 "" H 4800 5050 60  0000 C CNN
	1    4800 5050
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR016
U 1 1 5495ACC3
P 4800 5450
F 0 "#PWR016" H 4800 5450 30  0001 C CNN
F 1 "GND" H 4800 5380 30  0001 C CNN
F 2 "" H 4800 5450 60  0000 C CNN
F 3 "" H 4800 5450 60  0000 C CNN
	1    4800 5450
	1    0    0    -1  
$EndComp
Text Label 3900 6550 0    60   ~ 0
LATCH
Text Label 2900 6450 0    60   ~ 0
VCC
Text Label 2900 6550 0    60   ~ 0
LATCH
Text Label 2900 6650 0    60   ~ 0
SHIFT
Text Label 2900 6750 0    60   ~ 0
DATA_1
$Comp
L GND #PWR017
U 1 1 5495B536
P 3000 7050
F 0 "#PWR017" H 3000 7050 30  0001 C CNN
F 1 "GND" H 3000 6980 30  0001 C CNN
F 2 "" H 3000 7050 60  0000 C CNN
F 3 "" H 3000 7050 60  0000 C CNN
	1    3000 7050
	1    0    0    -1  
$EndComp
$Comp
L CONN_6 P6
U 1 1 5495B53C
P 2450 6700
F 0 "P6" V 2400 6700 60  0000 C CNN
F 1 "CHANNEL_1" V 2500 6700 60  0000 C CNN
F 2 "" H 2450 6700 60  0000 C CNN
F 3 "" H 2450 6700 60  0000 C CNN
	1    2450 6700
	-1   0    0    -1  
$EndComp
Text Label 3900 6450 0    60   ~ 0
VCC
Text Label 3900 6750 0    60   ~ 0
DATA_2
$Comp
L CONN_2 P4
U 1 1 5495BFD0
P 900 6100
F 0 "P4" V 850 6100 40  0000 C CNN
F 1 "RESET" V 950 6100 40  0000 C CNN
F 2 "" H 900 6100 60  0000 C CNN
F 3 "" H 900 6100 60  0000 C CNN
	1    900  6100
	-1   0    0    1   
$EndComp
$Comp
L GND #PWR018
U 1 1 5495C0B3
P 2050 6300
F 0 "#PWR018" H 2050 6300 30  0001 C CNN
F 1 "GND" H 2050 6230 30  0001 C CNN
F 2 "" H 2050 6300 60  0000 C CNN
F 3 "" H 2050 6300 60  0000 C CNN
	1    2050 6300
	1    0    0    -1  
$EndComp
Text Label 1250 5500 2    60   ~ 0
LATCH
Text Label 1250 5600 2    60   ~ 0
SHIFT
Text Label 1250 5800 2    60   ~ 0
DATA_1
Text Label 1250 5900 2    60   ~ 0
DATA_2
$Comp
L CONN_6 P5
U 1 1 5495CE5A
P 1700 5050
F 0 "P5" V 1650 5050 60  0000 C CNN
F 1 "ISP" V 1750 5050 60  0000 C CNN
F 2 "" H 1700 5050 60  0000 C CNN
F 3 "" H 1700 5050 60  0000 C CNN
	1    1700 5050
	0    -1   -1   0   
$EndComp
Text Label 1250 5300 2    60   ~ 0
VCC
Wire Wire Line
	1300 3750 1400 3750
Wire Wire Line
	5300 1900 5400 1900
Wire Wire Line
	1400 3750 1400 3950
Wire Wire Line
	1600 3450 1300 3450
Wire Wire Line
	1300 3550 1600 3550
Wire Wire Line
	3550 1700 3750 1700
Wire Wire Line
	2550 3000 2550 3100
Wire Wire Line
	3750 1700 3750 1600
Wire Wire Line
	1450 1900 1250 1900
Wire Wire Line
	1250 2200 1450 2200
Wire Wire Line
	1450 2300 1250 2300
Wire Wire Line
	1250 2400 1450 2400
Wire Wire Line
	1450 2500 1250 2500
Wire Wire Line
	1250 2600 1450 2600
Wire Wire Line
	1400 2700 1400 2100
Wire Wire Line
	1400 2100 1250 2100
Wire Wire Line
	1400 2000 1250 2000
Wire Wire Line
	5400 2000 5300 2000
Wire Wire Line
	8400 2650 8400 3100
Wire Wire Line
	7400 2450 7500 2450
Wire Wire Line
	8000 2450 8100 2450
Wire Wire Line
	8400 2050 8400 2250
Wire Wire Line
	8400 1550 8400 1450
Wire Wire Line
	8400 2150 9450 2150
Connection ~ 8400 2150
Wire Wire Line
	1250 1350 1450 1350
Wire Wire Line
	1450 1350 1450 1450
Wire Wire Line
	1350 1150 1250 1150
Wire Wire Line
	1350 1050 1250 1050
Wire Wire Line
	1350 950  1250 950 
Wire Wire Line
	1350 850  1250 850 
Wire Wire Line
	1350 1250 1250 1250
Wire Wire Line
	1600 3650 1300 3650
Wire Wire Line
	1400 3050 1400 3150
Wire Wire Line
	1400 3150 1300 3150
Wire Wire Line
	3900 1800 2550 1800
Wire Wire Line
	2550 1900 3900 1900
Wire Wire Line
	3900 2000 2550 2000
Wire Wire Line
	2550 2100 3900 2100
Wire Wire Line
	3900 2200 2550 2200
Wire Wire Line
	2550 2300 3900 2300
Wire Wire Line
	3900 2400 2550 2400
Wire Wire Line
	2550 2500 3900 2500
Wire Wire Line
	3900 2600 2550 2600
Wire Wire Line
	2550 2750 3900 2750
Wire Wire Line
	3900 2900 2550 2900
Wire Wire Line
	2550 3000 3900 3000
Wire Wire Line
	3450 1700 3450 2300
Connection ~ 3450 2300
Wire Wire Line
	3350 1700 3350 2200
Connection ~ 3350 2200
Wire Wire Line
	3250 1700 3250 2400
Connection ~ 3250 2400
Wire Wire Line
	3150 1700 3150 2100
Connection ~ 3150 2100
Wire Wire Line
	3050 1700 3050 2500
Connection ~ 3050 2500
Wire Wire Line
	2950 1700 2950 2000
Connection ~ 2950 2000
Wire Wire Line
	2850 1700 2850 2600
Connection ~ 2850 2600
Wire Wire Line
	2750 1700 2750 1900
Connection ~ 2750 1900
Wire Wire Line
	4150 5500 4250 5500
Wire Wire Line
	4250 5050 4250 5550
Wire Wire Line
	4150 6000 4250 6000
Wire Wire Line
	4250 5950 4250 6200
Connection ~ 4250 6000
Connection ~ 4250 5500
Connection ~ 4250 5150
Wire Wire Line
	3500 5150 4250 5150
Wire Wire Line
	3000 5150 2050 5150
Wire Wire Line
	2050 5150 2050 6000
Wire Wire Line
	3800 6950 4000 6950
Wire Wire Line
	4000 6950 4000 7050
Wire Wire Line
	3900 6450 3800 6450
Wire Wire Line
	4900 6000 4800 6000
Wire Wire Line
	4900 6100 4800 6100
Wire Wire Line
	4800 6100 4800 6200
Wire Wire Line
	4800 6000 4800 5750
Wire Wire Line
	4900 5900 4700 5900
Wire Wire Line
	4800 5050 4800 5150
Wire Wire Line
	4800 5150 4900 5150
Wire Wire Line
	4900 5350 4800 5350
Wire Wire Line
	4800 5350 4800 5450
Wire Wire Line
	2800 6950 3000 6950
Wire Wire Line
	3000 6950 3000 7050
Wire Wire Line
	2900 6750 2800 6750
Wire Wire Line
	2900 6650 2800 6650
Wire Wire Line
	2900 6550 2800 6550
Wire Wire Line
	2900 6450 2800 6450
Wire Wire Line
	2900 6850 2800 6850
Wire Wire Line
	3900 6550 3800 6550
Wire Wire Line
	3900 6650 3800 6650
Wire Wire Line
	3900 6750 3800 6750
Wire Wire Line
	1250 6000 2150 6000
Wire Wire Line
	1250 5800 2150 5800
Wire Wire Line
	1250 5700 2150 5700
Wire Wire Line
	1250 5600 2150 5600
Wire Wire Line
	1250 5500 2150 5500
Connection ~ 2050 6000
Wire Wire Line
	1250 6200 2050 6200
Wire Wire Line
	2050 6200 2050 6300
Wire Wire Line
	1450 5400 1450 5600
Connection ~ 1450 5600
Wire Wire Line
	1750 5400 1750 5500
Connection ~ 1750 5500
Wire Wire Line
	1550 5400 1550 5300
Wire Wire Line
	1550 5300 1250 5300
Wire Wire Line
	1650 5400 1650 5700
Connection ~ 1650 5700
Wire Wire Line
	1850 5400 1850 6000
Connection ~ 1850 6000
Wire Wire Line
	1950 5400 1950 6200
Connection ~ 1950 6200
Text Label 1750 6000 2    60   ~ 0
RESET
Text Label 1250 5700 2    60   ~ 0
ATAD
Wire Wire Line
	1250 5900 2150 5900
Wire Wire Line
	3900 6850 3800 6850
$Comp
L GND #PWR019
U 1 1 5495DF93
P 10750 6800
F 0 "#PWR019" H 10750 6800 30  0001 C CNN
F 1 "GND" H 10750 6730 30  0001 C CNN
F 2 "" H 10750 6800 60  0000 C CNN
F 3 "" H 10750 6800 60  0000 C CNN
	1    10750 6800
	1    0    0    -1  
$EndComp
Wire Wire Line
	10750 6700 10750 6800
NoConn ~ 3900 6850
NoConn ~ 2900 6850
Wire Notes Line
	700  700  9850 700 
Wire Notes Line
	9850 700  9850 4150
Wire Notes Line
	9850 4150 700  4150
Wire Notes Line
	700  4150 700  700 
Wire Notes Line
	700  4650 5500 4650
Wire Notes Line
	5500 4650 5500 7350
Wire Notes Line
	700  7350 5500 7350
Wire Notes Line
	700  4650 700  7350
Text Notes 700  4650 0    60   ~ 0
Microcontroller Board
Text Notes 700  700  0    60   ~ 0
Controller Interface Board
$EndSCHEMATC
