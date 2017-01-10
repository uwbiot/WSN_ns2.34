#ifndef ns_mac_sensor_h
#define ns_mac_sensor_h

#include "timer-handler.h"
#include "packet.h"
#include "mac.h"
#include "wireless-chargingphy.h"
class Mac_Sensor;

class MAC_Sensor_Config
{
public:
	MAC_Sensor_Config()
	{
		/* Configuration for Standard 802.15.4 MAC */
		data_rate = 250000;
		backoff_slot = 0.00032;
		CCA = 0.000128;
		cw_min = 7;
		cw_max = 127;
		header_len = 3;
		ack_len = 3;
	}
	double data_rate;
	double backoff_slot;
	double CCA;
	unsigned int cw_min;
	unsigned int cw_max;
	int header_len;				//The length (in bytes) of MAC header
	int ack_len;					//The length (in bytes) of ACK
};

class Mac_SensorTimer: public Handler {
public:
	Mac_SensorTimer(Mac_Sensor* m) : mac(m) {
	  busy_ = 0;
	}
	virtual void handle(Event *e) = 0;
	virtual void restart(double time);
	virtual void start(double time);
	virtual void stop(void);
    virtual void forcestop(void);
	inline int busy(void) { return busy_; }
	inline double expire(void) {
		return ((stime + rtime) - Scheduler::instance().clock());
	}

protected:
	Mac_Sensor	*mac;
	int		busy_;
	Event		intr;
	double		stime;
	double		rtime;
	double		slottime;
};

// Timer to use for backoff
class Mac_SensorWaitTimer: public Mac_SensorTimer {
public: Mac_SensorWaitTimer(Mac_Sensor *m) : Mac_SensorTimer(m) {}
	void handle(Event *e);
};

//  Timer to use for finishing sending of packets
class Mac_SensorSendTimer: public Mac_SensorTimer {
public:
	Mac_SensorSendTimer(Mac_Sensor *m) : Mac_SensorTimer(m) {}
	void handle(Event *e);
};

// Timer to use for finishing reception of packets
class Mac_SensorRecvTimer: public Mac_SensorTimer {
public:
	Mac_SensorRecvTimer(Mac_Sensor *m) : Mac_SensorTimer(m) {}
	void handle(Event *e);
};

// Timer to use for ack backoff
class Mac_SensorACKTimer: public Mac_SensorTimer {
public:
	Mac_SensorACKTimer(Mac_Sensor *m) : Mac_SensorTimer(m) {}
	void handle(Event *e);
};

// timers to implement xmac
class Mac_SensorDutyTimer: public Mac_SensorTimer {
public:
    Mac_SensorDutyTimer(Mac_Sensor *m) : Mac_SensorTimer(m), lastStatus_(1), sleepInterval_(0){}
	void            handle(Event *e);
    void            setDutyCycle(double sleepInterval){sleepInterval_ = sleepInterval;}
    inline double   interval(){return sleepInterval_;}
private:
    int lastStatus_;
    double sleepInterval_;
};

class Mac_SensorOnRadioTimer: public Mac_SensorTimer {
public:
	Mac_SensorOnRadioTimer(Mac_Sensor *m) : Mac_SensorTimer(m), onperiod_(0.1), recvCnt(0),lastRecvCnt(0) {}
	void            handle(Event *e);
    inline double   interval(){return onperiod_;}
    void            setOnInterval(double interval){onperiod_ = interval;minimalOntime_ = interval;}
    void            setNodeDuty(double duty){nodeDuty_ = duty;}
    void            increaseInterval();
private:
    double onperiod_;
    double nodeDuty_;
    double minimalOntime_;
    int recvCnt;
    int lastRecvCnt;
};

class Mac_SensorPreembleTimer: public Mac_SensorTimer {
public:
	Mac_SensorPreembleTimer(Mac_Sensor *m) : Mac_SensorTimer(m) {}
	void handle(Event *e);
};


class Mac_Sensor : public Mac {
public:
	Mac_Sensor();
	void recv(Packet *p, Handler *h);
	void send(Packet *p, Handler *h, u_int16_t type);

	void waitHandler(void);
	void sendHandler(void);
	void recvHandler(void);
	void ackHandler(void);

	void dutyHandler(void);
	void onRadioHandler(void);
    void preembleHandler(bool resend);

	double txtime(Packet *p);

	inline void inc_cw() {
		cw_ = (cw_ << 1) + 1;
		if(cw_ > config.cw_max)
			cw_ = config.cw_max;
	}
	inline void reset_cw() { cw_ = config.cw_min; }

	inline bool ACK() { return ACK_; }

private:
	    MAC_Sensor_Config config;
	    Packet*	        pktRx_;
	    Packet*	        pktTx_;
        Packet*         pktTxBackup_;
        MacState        rx_state_;      // incoming state (MAC_RECV or MAC_IDLE)
	    MacState        tx_state_;      // outgoing state
        int             tx_active_;
        int	ACK_;					//ACK on/off flag
        unsigned int cw_;			//contention window
        bool acked_;
        int resendCount_;
        double DutyCycle_;
        double OnRadioTime_;
        bool resending_;
        bool emptyEng_;

	    Handler * 	txHandler_;
	    Mac_SensorWaitTimer waitTimer;
	    Mac_SensorSendTimer sendTimer;
	    Mac_SensorRecvTimer recvTimer;
	    Mac_SensorACKTimer  ackTimer;

        Mac_SensorDutyTimer dutyTimer;
        Mac_SensorOnRadioTimer onRadioTimer;
        Mac_SensorPreembleTimer preembleTimer;

        inline WirelessChargingPhy* newnetif(){return dynamic_cast<WirelessChargingPhy*>(netif_);}
};

#endif
