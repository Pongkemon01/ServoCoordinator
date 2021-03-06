EESchema Schematic File Version 4
LIBS:coordinator-cache
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 5 6
Title "6DOF Parallel Robot"
Date "2019-September-26"
Rev "2.1.0"
Comp "Kasetsart University"
Comment1 "Faculty of Engineering"
Comment2 "Department of Computer Engineering"
Comment3 "Asst.Prof.Akrapong Patchararungruang, Ph.D."
Comment4 ""
$EndDescr
$Comp
L zeabus:Nucleo-144-zio CON1
U 1 1 5D83F23B
P 5500 3600
F 0 "CON1" H 5950 6000 50  0000 C CNN
F 1 "Nucleo-144-zio" H 5500 3550 50  0000 C CNB
F 2 "zeabus:Nucleo-144-zio" H 4300 1050 50  0001 C CNN
F 3 "" H 4300 1050 50  0001 C CNN
	1    5500 3600
	1    0    0    -1  
$EndComp
Text GLabel 6550 2450 2    50   Input ~ 0
M1_ENCA
Text GLabel 6550 2650 2    50   Input ~ 0
M1_ENCB
Text GLabel 4450 2100 0    50   Input ~ 0
M2_ENCA
Text GLabel 4450 2600 0    50   Input ~ 0
M2_ENCB
Text GLabel 4450 2700 0    50   Input ~ 0
M3_ENCA
Text GLabel 4450 2800 0    50   Input ~ 0
M3_ENCB
Text GLabel 4450 5800 0    50   Input ~ 0
M4_ENCA
Text GLabel 4450 5900 0    50   Input ~ 0
M4_ENCB
Text GLabel 4450 1400 0    50   Input ~ 0
M5_ENCA
Text GLabel 8850 3950 0    50   Input ~ 0
M5_ENCB
Text GLabel 4450 4100 0    50   Input ~ 0
M6_ENCA
Text GLabel 4450 4200 0    50   Input ~ 0
M6_ENCB
Text GLabel 6550 2050 2    50   Output ~ 0
M1_PULSE
Text GLabel 4450 3800 0    50   Output ~ 0
M1_DIR
Text GLabel 4450 3000 0    50   Output ~ 0
M2_PULSE
Text GLabel 4450 3900 0    50   Output ~ 0
M2_DIR
Text GLabel 4450 3100 0    50   Output ~ 0
M3_PULSE
Text GLabel 4450 4000 0    50   Output ~ 0
M3_DIR
Text GLabel 4450 3600 0    50   Output ~ 0
M4_PULSE
Text GLabel 4450 4300 0    50   Output ~ 0
M4_DIR
Text GLabel 4450 1800 0    50   Output ~ 0
M5_PULSE
Text GLabel 4450 4400 0    50   Output ~ 0
M5_DIR
Text GLabel 6550 4050 2    50   Output ~ 0
M6_PULSE
Text GLabel 4450 4500 0    50   Output ~ 0
M6_DIR
Text GLabel 6550 1550 2    50   Input ~ 0
M1_ALARM_N
Text GLabel 6550 2350 2    50   Input ~ 0
M1_COMPLETE_N
Text GLabel 6550 4750 2    50   Input ~ 0
M1_HOME_N
Text GLabel 6550 1650 2    50   Input ~ 0
M2_ALARM_N
Text GLabel 6550 2550 2    50   Input ~ 0
M2_COMPLETE_N
Text GLabel 6550 4850 2    50   Input ~ 0
M2_HOME_N
Text GLabel 6550 1850 2    50   Input ~ 0
M3_ALARM_N
Text GLabel 6550 2750 2    50   Input ~ 0
M3_COMPLETE_N
Text GLabel 6550 4950 2    50   Input ~ 0
M3_HOME_N
Text GLabel 6550 1950 2    50   Input ~ 0
M4_ALARM_N
Text GLabel 6550 2850 2    50   Input ~ 0
M4_COMPLETE_N
Text GLabel 6550 5050 2    50   Input ~ 0
M4_HOME_N
Text GLabel 6550 2150 2    50   Input ~ 0
M5_ALARM_N
Text GLabel 6550 2950 2    50   Input ~ 0
M5_COMPLETE_N
Text GLabel 6550 5150 2    50   Input ~ 0
M5_HOME_N
Text GLabel 6550 2250 2    50   Input ~ 0
M6_ALARM_N
Text GLabel 6550 3050 2    50   Input ~ 0
M6_COMPLETE_N
Text GLabel 6550 5250 2    50   Input ~ 0
M6_HOME_N
Text GLabel 6550 3250 2    50   Output ~ 0
M1_SRV_ON
Text GLabel 4450 2300 0    50   Output ~ 0
M1_ALRM_RES
Text GLabel 6550 3850 2    50   Output ~ 0
M1_DEV_CLR
Text GLabel 6550 3350 2    50   Output ~ 0
M2_SRV_ON
Text GLabel 4450 2400 0    50   Output ~ 0
M2_ALRM_RES
Text GLabel 6550 3950 2    50   Output ~ 0
M2_DEV_CLR
Text GLabel 6550 3450 2    50   Output ~ 0
M3_SRV_ON
Text GLabel 4450 2500 0    50   Output ~ 0
M3_ALRM_RES
Text GLabel 6550 4150 2    50   Output ~ 0
M3_DEV_CLR
Text GLabel 6550 3550 2    50   Output ~ 0
M4_SRV_ON
Text GLabel 4450 2900 0    50   Output ~ 0
M4_ALRM_RES
Text GLabel 6550 4250 2    50   Output ~ 0
M4_DEV_CLR
Text GLabel 6550 3650 2    50   Output ~ 0
M5_SRV_ON
Text GLabel 4450 3400 0    50   Output ~ 0
M5_ALRM_RES
Text GLabel 6550 4350 2    50   Output ~ 0
M5_DEV_CLR
Text GLabel 6550 3750 2    50   Output ~ 0
M6_SRV_ON
Text GLabel 4450 3500 0    50   Output ~ 0
M6_ALRM_RES
Text GLabel 6550 4450 2    50   Output ~ 0
M6_DEV_CLR
NoConn ~ 6550 1750
NoConn ~ 4450 2000
NoConn ~ 5500 950 
Wire Wire Line
	5100 6550 5100 6600
Wire Wire Line
	5100 6600 5200 6600
Wire Wire Line
	6000 6600 6000 6550
Wire Wire Line
	5900 6550 5900 6600
Connection ~ 5900 6600
Wire Wire Line
	5900 6600 6000 6600
Wire Wire Line
	5800 6550 5800 6600
Connection ~ 5800 6600
Wire Wire Line
	5800 6600 5900 6600
Wire Wire Line
	5700 6550 5700 6600
Connection ~ 5700 6600
Wire Wire Line
	5700 6600 5800 6600
Wire Wire Line
	5600 6550 5600 6600
Connection ~ 5600 6600
Wire Wire Line
	5600 6600 5700 6600
Wire Wire Line
	5500 6550 5500 6600
Connection ~ 5500 6600
Wire Wire Line
	5500 6600 5600 6600
Wire Wire Line
	5400 6550 5400 6600
Connection ~ 5400 6600
Wire Wire Line
	5400 6600 5500 6600
Wire Wire Line
	5300 6550 5300 6600
Connection ~ 5300 6600
Wire Wire Line
	5300 6600 5400 6600
Wire Wire Line
	5200 6550 5200 6600
Connection ~ 5200 6600
Wire Wire Line
	5200 6600 5300 6600
$Comp
L power:GND #PWR0203
U 1 1 5D85D0B6
P 5600 6600
F 0 "#PWR0203" H 5600 6350 50  0001 C CNN
F 1 "GND" H 5605 6427 50  0000 C CNN
F 2 "" H 5600 6600 50  0001 C CNN
F 3 "" H 5600 6600 50  0001 C CNN
	1    5600 6600
	1    0    0    -1  
$EndComp
NoConn ~ 5300 950 
NoConn ~ 5400 950 
NoConn ~ 5600 950 
NoConn ~ 4450 1900
$Comp
L Device:LED_Dual_CACA D3
U 1 1 5D860A1D
P 1800 4850
F 0 "D3" H 1800 5275 50  0000 C CNN
F 1 "LED_Dual_CACA" H 1800 5184 50  0000 C CNN
F 2 "zeabus:LED-Inlux-IN-S85DATRG" H 1830 4850 50  0001 C CNN
F 3 "~" H 1830 4850 50  0001 C CNN
	1    1800 4850
	1    0    0    -1  
$EndComp
$Comp
L Device:LED_Dual_CACA D4
U 1 1 5D863B28
P 1800 5550
F 0 "D4" H 1800 5975 50  0000 C CNN
F 1 "LED_Dual_CACA" H 1800 5884 50  0000 C CNN
F 2 "zeabus:LED-Inlux-IN-S85DATRG" H 1830 5550 50  0001 C CNN
F 3 "~" H 1830 5550 50  0001 C CNN
	1    1800 5550
	1    0    0    -1  
$EndComp
$Comp
L Device:LED_Dual_CACA D5
U 1 1 5D86493E
P 1800 6250
F 0 "D5" H 1800 6675 50  0000 C CNN
F 1 "LED_Dual_CACA" H 1800 6584 50  0000 C CNN
F 2 "zeabus:LED-Inlux-IN-S85DATRG" H 1830 6250 50  0001 C CNN
F 3 "~" H 1830 6250 50  0001 C CNN
	1    1800 6250
	1    0    0    -1  
$EndComp
$Comp
L Device:LED_Dual_CACA D6
U 1 1 5D865295
P 1800 6950
F 0 "D6" H 1800 7375 50  0000 C CNN
F 1 "LED_Dual_CACA" H 1800 7284 50  0000 C CNN
F 2 "zeabus:LED-Inlux-IN-S85DATRG" H 1830 6950 50  0001 C CNN
F 3 "~" H 1830 6950 50  0001 C CNN
	1    1800 6950
	1    0    0    -1  
$EndComp
$Comp
L Device:LED_Dual_CACA D2
U 1 1 5D865DAE
P 1800 4150
F 0 "D2" H 1800 4575 50  0000 C CNN
F 1 "LED_Dual_CACA" H 1800 4484 50  0000 C CNN
F 2 "zeabus:LED-Inlux-IN-S85DATRG" H 1830 4150 50  0001 C CNN
F 3 "~" H 1830 4150 50  0001 C CNN
	1    1800 4150
	1    0    0    -1  
$EndComp
$Comp
L Device:LED_Dual_CACA D1
U 1 1 5D866516
P 1800 3450
F 0 "D1" H 1800 3875 50  0000 C CNN
F 1 "LED_Dual_CACA" H 1800 3784 50  0000 C CNN
F 2 "zeabus:LED-Inlux-IN-S85DATRG" H 1830 3450 50  0001 C CNN
F 3 "~" H 1830 3450 50  0001 C CNN
	1    1800 3450
	1    0    0    -1  
$EndComp
Wire Wire Line
	1500 3350 1400 3350
Wire Wire Line
	1400 3350 1400 3550
Wire Wire Line
	1400 7050 1500 7050
Wire Wire Line
	1500 6850 1400 6850
Connection ~ 1400 6850
Wire Wire Line
	1400 6850 1400 7050
Wire Wire Line
	1500 6350 1400 6350
Connection ~ 1400 6350
Wire Wire Line
	1400 6350 1400 6850
Wire Wire Line
	1400 6150 1500 6150
Connection ~ 1400 6150
Wire Wire Line
	1400 6150 1400 6350
Wire Wire Line
	1500 5650 1400 5650
Connection ~ 1400 5650
Wire Wire Line
	1400 5650 1400 6150
Wire Wire Line
	1400 5450 1500 5450
Connection ~ 1400 5450
Wire Wire Line
	1400 5450 1400 5650
Wire Wire Line
	1500 4950 1400 4950
Connection ~ 1400 4950
Wire Wire Line
	1400 4950 1400 5450
Wire Wire Line
	1400 4750 1500 4750
Connection ~ 1400 4750
Wire Wire Line
	1400 4750 1400 4950
Wire Wire Line
	1500 4250 1400 4250
Connection ~ 1400 4250
Wire Wire Line
	1400 4250 1400 4750
Wire Wire Line
	1500 4050 1400 4050
Connection ~ 1400 4050
Wire Wire Line
	1400 4050 1400 4250
Wire Wire Line
	1500 3550 1400 3550
Connection ~ 1400 3550
Wire Wire Line
	1400 3550 1400 4050
$Comp
L power:GND #PWR0204
U 1 1 5D86C483
P 1400 7050
F 0 "#PWR0204" H 1400 6800 50  0001 C CNN
F 1 "GND" H 1405 6877 50  0000 C CNN
F 2 "" H 1400 7050 50  0001 C CNN
F 3 "" H 1400 7050 50  0001 C CNN
	1    1400 7050
	1    0    0    -1  
$EndComp
Connection ~ 1400 7050
$Comp
L Device:R_US R?
U 1 1 5D870C1D
P 2250 3350
AR Path="/5D7E7AFD/5D870C1D" Ref="R?"  Part="1" 
AR Path="/5D7E8877/5D870C1D" Ref="R97"  Part="1" 
F 0 "R97" V 2150 3250 50  0000 C CNN
F 1 "330" V 2150 3450 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 2290 3340 50  0001 C CNN
F 3 "~" H 2250 3350 50  0001 C CNN
	1    2250 3350
	0    1    1    0   
$EndComp
$Comp
L Device:R_US R?
U 1 1 5D87155F
P 2250 3550
AR Path="/5D7E7AFD/5D87155F" Ref="R?"  Part="1" 
AR Path="/5D7E8877/5D87155F" Ref="R98"  Part="1" 
F 0 "R98" V 2350 3450 50  0000 C CNN
F 1 "330" V 2350 3650 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 2290 3540 50  0001 C CNN
F 3 "~" H 2250 3550 50  0001 C CNN
	1    2250 3550
	0    1    1    0   
$EndComp
$Comp
L Device:R_US R?
U 1 1 5D874AF1
P 2250 4050
AR Path="/5D7E7AFD/5D874AF1" Ref="R?"  Part="1" 
AR Path="/5D7E8877/5D874AF1" Ref="R99"  Part="1" 
F 0 "R99" V 2150 3950 50  0000 C CNN
F 1 "330" V 2150 4150 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 2290 4040 50  0001 C CNN
F 3 "~" H 2250 4050 50  0001 C CNN
	1    2250 4050
	0    1    1    0   
$EndComp
$Comp
L Device:R_US R?
U 1 1 5D874AF7
P 2250 4250
AR Path="/5D7E7AFD/5D874AF7" Ref="R?"  Part="1" 
AR Path="/5D7E8877/5D874AF7" Ref="R100"  Part="1" 
F 0 "R100" V 2350 4150 50  0000 C CNN
F 1 "330" V 2350 4350 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 2290 4240 50  0001 C CNN
F 3 "~" H 2250 4250 50  0001 C CNN
	1    2250 4250
	0    1    1    0   
$EndComp
$Comp
L Device:R_US R?
U 1 1 5D876AB7
P 2250 4750
AR Path="/5D7E7AFD/5D876AB7" Ref="R?"  Part="1" 
AR Path="/5D7E8877/5D876AB7" Ref="R101"  Part="1" 
F 0 "R101" V 2150 4650 50  0000 C CNN
F 1 "330" V 2150 4850 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 2290 4740 50  0001 C CNN
F 3 "~" H 2250 4750 50  0001 C CNN
	1    2250 4750
	0    1    1    0   
$EndComp
$Comp
L Device:R_US R?
U 1 1 5D876ABD
P 2250 4950
AR Path="/5D7E7AFD/5D876ABD" Ref="R?"  Part="1" 
AR Path="/5D7E8877/5D876ABD" Ref="R102"  Part="1" 
F 0 "R102" V 2350 4850 50  0000 C CNN
F 1 "330" V 2350 5050 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 2290 4940 50  0001 C CNN
F 3 "~" H 2250 4950 50  0001 C CNN
	1    2250 4950
	0    1    1    0   
$EndComp
$Comp
L Device:R_US R?
U 1 1 5D87814D
P 2250 5450
AR Path="/5D7E7AFD/5D87814D" Ref="R?"  Part="1" 
AR Path="/5D7E8877/5D87814D" Ref="R103"  Part="1" 
F 0 "R103" V 2150 5350 50  0000 C CNN
F 1 "330" V 2150 5550 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 2290 5440 50  0001 C CNN
F 3 "~" H 2250 5450 50  0001 C CNN
	1    2250 5450
	0    1    1    0   
$EndComp
$Comp
L Device:R_US R?
U 1 1 5D878153
P 2250 5650
AR Path="/5D7E7AFD/5D878153" Ref="R?"  Part="1" 
AR Path="/5D7E8877/5D878153" Ref="R104"  Part="1" 
F 0 "R104" V 2350 5550 50  0000 C CNN
F 1 "330" V 2350 5750 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 2290 5640 50  0001 C CNN
F 3 "~" H 2250 5650 50  0001 C CNN
	1    2250 5650
	0    1    1    0   
$EndComp
$Comp
L Device:R_US R?
U 1 1 5D87A113
P 2250 6150
AR Path="/5D7E7AFD/5D87A113" Ref="R?"  Part="1" 
AR Path="/5D7E8877/5D87A113" Ref="R105"  Part="1" 
F 0 "R105" V 2150 6050 50  0000 C CNN
F 1 "330" V 2150 6250 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 2290 6140 50  0001 C CNN
F 3 "~" H 2250 6150 50  0001 C CNN
	1    2250 6150
	0    1    1    0   
$EndComp
$Comp
L Device:R_US R?
U 1 1 5D87A119
P 2250 6350
AR Path="/5D7E7AFD/5D87A119" Ref="R?"  Part="1" 
AR Path="/5D7E8877/5D87A119" Ref="R106"  Part="1" 
F 0 "R106" V 2350 6250 50  0000 C CNN
F 1 "330" V 2350 6450 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 2290 6340 50  0001 C CNN
F 3 "~" H 2250 6350 50  0001 C CNN
	1    2250 6350
	0    1    1    0   
$EndComp
$Comp
L Device:R_US R?
U 1 1 5D87B381
P 2250 6850
AR Path="/5D7E7AFD/5D87B381" Ref="R?"  Part="1" 
AR Path="/5D7E8877/5D87B381" Ref="R107"  Part="1" 
F 0 "R107" V 2150 6750 50  0000 C CNN
F 1 "330" V 2150 6950 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 2290 6840 50  0001 C CNN
F 3 "~" H 2250 6850 50  0001 C CNN
	1    2250 6850
	0    1    1    0   
$EndComp
$Comp
L Device:R_US R?
U 1 1 5D87B387
P 2250 7050
AR Path="/5D7E7AFD/5D87B387" Ref="R?"  Part="1" 
AR Path="/5D7E8877/5D87B387" Ref="R108"  Part="1" 
F 0 "R108" V 2350 6950 50  0000 C CNN
F 1 "330" V 2350 7150 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 2290 7040 50  0001 C CNN
F 3 "~" H 2250 7050 50  0001 C CNN
	1    2250 7050
	0    1    1    0   
$EndComp
Wire Wire Line
	2400 5450 2650 5450
Wire Wire Line
	2650 5450 2650 5500
Wire Wire Line
	2650 5500 4450 5500
Wire Wire Line
	2400 5650 2650 5650
Wire Wire Line
	2650 5650 2650 5600
Wire Wire Line
	2650 5600 4450 5600
Wire Wire Line
	2400 4950 2700 4950
Wire Wire Line
	2700 4950 2700 5400
Wire Wire Line
	2700 5400 4450 5400
Wire Wire Line
	2400 4750 2800 4750
Wire Wire Line
	2800 4750 2800 5300
Wire Wire Line
	2800 5300 4450 5300
Wire Wire Line
	2400 4250 2900 4250
Wire Wire Line
	2900 4250 2900 5200
Wire Wire Line
	2900 5200 4450 5200
Wire Wire Line
	2400 4050 3000 4050
Wire Wire Line
	3000 4050 3000 5100
Wire Wire Line
	3000 5100 4450 5100
Wire Wire Line
	2400 3550 3100 3550
Wire Wire Line
	3100 3550 3100 5000
Wire Wire Line
	3100 5000 4450 5000
Wire Wire Line
	2400 3350 3200 3350
Wire Wire Line
	3200 3350 3200 4900
Wire Wire Line
	3200 4900 4450 4900
Wire Wire Line
	2400 6150 2700 6150
Wire Wire Line
	2700 6150 2700 5700
Wire Wire Line
	2700 5700 4450 5700
Wire Wire Line
	2400 6350 2800 6350
Wire Wire Line
	2800 6350 2800 6000
Wire Wire Line
	2800 6000 4450 6000
Wire Wire Line
	2400 6850 2900 6850
Wire Wire Line
	2900 6850 2900 6100
Wire Wire Line
	2900 6100 4450 6100
Wire Wire Line
	6900 6250 7550 6250
Wire Wire Line
	7550 6250 7550 4550
Wire Wire Line
	7550 4550 6550 4550
NoConn ~ 6550 5550
NoConn ~ 6550 5650
Text GLabel 4450 3200 0    50   Output ~ 0
I2C_SCL
Text GLabel 4450 3300 0    50   BiDi ~ 0
I2C_SDA
Text GLabel 4450 1600 0    50   Input ~ 0
EXT_INT1
Text GLabel 4450 1700 0    50   Input ~ 0
EXT_INT2
Text GLabel 4450 1500 0    50   Input ~ 0
EMERGENCY_N
$Comp
L Connector:Conn_01x01_Male J2
U 1 1 5D8962D2
P 9300 3950
F 0 "J2" H 9272 3974 50  0000 R CNN
F 1 "PA1" H 9272 3883 50  0000 R CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x01_P2.54mm_Vertical" H 9300 3950 50  0001 C CNN
F 3 "~" H 9300 3950 50  0001 C CNN
	1    9300 3950
	-1   0    0    -1  
$EndComp
Wire Wire Line
	8850 3950 9100 3950
Text Notes 8350 4150 0    50   Italic 10
Extra connection to Morpho pin CN11-30
NoConn ~ 4450 4600
NoConn ~ 4450 4700
$Comp
L zeabus:PDQE10 U16
U 1 1 5D8A75BA
P 8650 2950
F 0 "U16" H 9100 3215 50  0000 C CNN
F 1 "PDQE10" H 9100 3124 50  0000 C CNN
F 2 "zeabus:PDQE10" H 8750 3050 50  0001 C CNN
F 3 "" H 8750 3050 50  0001 C CNN
	1    8650 2950
	1    0    0    -1  
$EndComp
NoConn ~ 8650 3050
$Comp
L power:+24V #PWR0205
U 1 1 5D8B015D
P 8500 2950
F 0 "#PWR0205" H 8500 2800 50  0001 C CNN
F 1 "+24V" H 8515 3123 50  0000 C CNN
F 2 "" H 8500 2950 50  0001 C CNN
F 3 "" H 8500 2950 50  0001 C CNN
	1    8500 2950
	1    0    0    -1  
$EndComp
$Comp
L power:GNDPWR #PWR0206
U 1 1 5D8B0F13
P 8500 3150
F 0 "#PWR0206" H 8500 2950 50  0001 C CNN
F 1 "GNDPWR" H 8504 2996 50  0000 C CNN
F 2 "" H 8500 3100 50  0001 C CNN
F 3 "" H 8500 3100 50  0001 C CNN
	1    8500 3150
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0207
U 1 1 5D8B1F9B
P 9700 3150
F 0 "#PWR0207" H 9700 2900 50  0001 C CNN
F 1 "GND" H 9705 2977 50  0000 C CNN
F 2 "" H 9700 3150 50  0001 C CNN
F 3 "" H 9700 3150 50  0001 C CNN
	1    9700 3150
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR0208
U 1 1 5D8B279D
P 9700 2950
F 0 "#PWR0208" H 9700 2800 50  0001 C CNN
F 1 "+5V" H 9715 3123 50  0000 C CNN
F 2 "" H 9700 2950 50  0001 C CNN
F 3 "" H 9700 2950 50  0001 C CNN
	1    9700 2950
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x01_Male J1
U 1 1 5D8B317A
P 10050 2950
F 0 "J1" H 10100 3150 50  0000 R CNN
F 1 "E5V" H 10100 3050 50  0000 R CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x01_P2.54mm_Vertical" H 10050 2950 50  0001 C CNN
F 3 "~" H 10050 2950 50  0001 C CNN
	1    10050 2950
	-1   0    0    -1  
$EndComp
Wire Wire Line
	8500 2950 8650 2950
Wire Wire Line
	8650 3150 8500 3150
Wire Wire Line
	9550 2950 9700 2950
Wire Wire Line
	9700 2950 9850 2950
Connection ~ 9700 2950
Wire Wire Line
	9550 3150 9700 3150
Text Notes 8400 3500 0    50   Italic 10
Extra connection to Morpho pin CN11-6
$Comp
L Device:CP_Small C1
U 1 1 5DA7D311
P 8500 3050
F 0 "C1" H 8300 3100 50  0000 L CNN
F 1 "10uF 25V" H 8050 3000 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 8500 3050 50  0001 C CNN
F 3 "~" H 8500 3050 50  0001 C CNN
	1    8500 3050
	1    0    0    -1  
$EndComp
$Comp
L Device:CP_Small C2
U 1 1 5DA7EA90
P 9700 3050
F 0 "C2" H 9788 3096 50  0000 L CNN
F 1 "10uF 25V" H 9788 3005 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 9700 3050 50  0001 C CNN
F 3 "~" H 9700 3050 50  0001 C CNN
	1    9700 3050
	1    0    0    -1  
$EndComp
Connection ~ 9700 3150
Connection ~ 8500 2950
Connection ~ 8500 3150
$Comp
L power:+3V3 #PWR0209
U 1 1 5DA85BEA
P 5700 950
F 0 "#PWR0209" H 5700 800 50  0001 C CNN
F 1 "+3V3" H 5715 1123 50  0000 C CNN
F 2 "" H 5700 950 50  0001 C CNN
F 3 "" H 5700 950 50  0001 C CNN
	1    5700 950 
	1    0    0    -1  
$EndComp
Wire Wire Line
	6900 7050 6900 6250
Wire Wire Line
	2400 7050 6900 7050
$EndSCHEMATC
