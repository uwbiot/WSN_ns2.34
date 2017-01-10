
# default system pamaters

#Phy/WirelessCPhy set RXThresh_ 4.80696e-05
#Phy/WirelessCPhy set RXThresh_ 3.71409e-09
#75m
Phy/WirelessCPhy set RXThresh_ 3.41828e-08
#150m
Phy/WirelessCPhy set CSThresh_ 2.81838e-09
#Phy/WirelessCPhy set CSThresh_ 3.61828e-08

set opt(radioBW)	        2.5e5 ; # 250 kbps
set opt(RTSThreshold) 		20000
set opt(CSThresh) 			[Phy/WirelessCPhy set CSThresh_]
set opt(RXThresh) 			[Phy/WirelessCPhy set RXThresh_]
set opt(CWMin) 				31 ;# 7; 2^3-1 802.15.4 spec
set opt(SlotTime) 			0.000320 ;# 20 symbols @ 62.5 ksymbol rate
set opt(SIFS) 				0.000192 ;# turnaround time; 192us 12 symbols @ 62.5 ks; we also use this as T_ack (interval between DATA and ACK) though in the standard it's a random number between aTurnaroundTime and (aTurnaroundTime + aUnitBackoffPeriod)
set opt(PreambleLength) 	40 ;# 32 bit; SHR header
set opt(PLCPHeaderLength) 	8 ;# 8 bits; length field

Mac/802_11 set RTSThreshold_		$opt(RTSThreshold) 
Mac/802_11 set CSThresh_			$opt(CSThresh)
Mac/802_11 set CWMin_				$opt(CWMin)
Mac/802_11 set SlotTime_			$opt(SlotTime)
Mac/802_11 set SIFS_				$opt(SIFS)
Mac/802_11 set PreambleLength_		$opt(PreambleLength)
Mac/802_11 set PLCPHeaderLength_	$opt(PLCPHeaderLength)
Mac/802_11 set PLCPDataRate_		$opt(radioBW)    
Mac/802_11 set LongRetryLimit_		0; # no retransmission in orig XMAC
Mac/802_11 set ShortRetryLimit_		0; # no retransmission in orig XMAC
Mac/802_11 set dataRate_			$opt(radioBW); 
Mac/802_11 set basicRate_			$opt(radioBW);

#Mac/RILMac set ACK                  true
Mac/RILMac set RTSThreshold_		$opt(RTSThreshold) 
Mac/RILMac set CSThresh_			$opt(CSThresh)
Mac/RILMac set CWMin_				$opt(CWMin)
Mac/RILMac set SlotTime_			$opt(SlotTime)
Mac/RILMac set SIFS_				$opt(SIFS)
Mac/RILMac set PreambleLength_		$opt(PreambleLength)
Mac/RILMac set PLCPHeaderLength_	$opt(PLCPHeaderLength)
Mac/RILMac set PLCPDataRate_		$opt(radioBW)    
Mac/RILMac set LongRetryLimit_		0; # no retransmission in orig XMAC
Mac/RILMac set ShortRetryLimit_		0; # no retransmission in orig XMAC
Mac/RILMac set dataRate_			$opt(radioBW); 
Mac/RILMac set basicRate_			$opt(radioBW);
#Mac/RILMac set DutyCycle            1
#Mac/RILMac set OnRadioTime          0.1


