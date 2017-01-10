#ifndef ns_rilmac_h
#define ns_rilmac_h


#include <vector>
#include <list>

#include "timer-handler.h"
#include "packet.h"
#include "mac.h"
#include "wireless-chargingphy.h"
#include "god.h"



class RILMac;

enum MacFrameSubType {
  MAC_RIL_RRTS = 1,
  MAC_RIL_ACK = 2,
  MAC_RIL_RESPONSE = 3,
  MAC_RIL_DATA = 4,
  MAC_RIL_ONDEMAND = 5,
};


struct neighborEntry {
  int id;
  double theta;         // average data arrival interval
  double delta;         // data arrival variance
  double pretx;         // previous tx time
  double wakeup;        // next wakeup time
  double rxbound;       // default rxbound time
  double prearrival;    // previous data arrival time
  double alive;
  int dupsize;
  neighborEntry(int i):id(i),theta(-1),delta(-1),pretx(0),wakeup(-1),rxbound(-1),prearrival(0),dupsize(0){}
};

class RILMac_Config
{
public:
  RILMac_Config() {
    /* Configuration for Standard 802.15.4 MAC */
    data_rate = 250000;
    backoff_slot = 0.00032;
    CCA = 0.000128;
    cw_min = 7;
    cw_max = 127;
    header_len = 16;
    ack_len = 16;
  }
  double data_rate;
  double backoff_slot;
  double CCA;
  unsigned int cw_min;
  unsigned int cw_max;
  int header_len;                 // The length (in bytes) of MAC header
  int ack_len;                    // The length (in bytes) of ACK
};

class RILMacTimer: public Handler {
public:
	RILMacTimer(RILMac* m) : mac(m) {
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
	RILMac*   mac;
	int		    busy_;
	Event		  intr;
	double		stime;
	double		rtime;
	double		slottime;
};

// Timer to use for backoff
class RILMacWaitTimer: public RILMacTimer {
public: RILMacWaitTimer(RILMac *m) : RILMacTimer(m) {}
	void handle(Event *e);
};

//  Timer to use for finishing sending of packets
class RILMacSendTimer: public RILMacTimer {
public:
	RILMacSendTimer(RILMac *m) : RILMacTimer(m) {}
	void handle(Event *e);
};

// Timer to use for finishing reception of packets
class RILMacRecvTimer: public RILMacTimer {
public:
	RILMacRecvTimer(RILMac *m) : RILMacTimer(m) {}
	void handle(Event *e);
};

// Timer to use for waiting for ack backoff
class RILMacACKTimer: public RILMacTimer {
public:
	RILMacACKTimer(RILMac *m) : RILMacTimer(m) {}
	void handle(Event *e);
};

// Timer to do duty cycling
class RILMacWakeupTimer: public RILMacTimer {
public:
  RILMacWakeupTimer(RILMac *m) : RILMacTimer(m){}
	void handle(Event *e);
};

// Timer to control radio on/off
class RILMacRadioTimer: public RILMacTimer {
public:
	RILMacRadioTimer(RILMac *m) : RILMacTimer(m), onperiod_(0.1){}
	void            handle(Event *e);
  inline double   interval(){return onperiod_;}
  void            setOnInterval(double interval){onperiod_ = interval;}
private:
    double onperiod_;
};

class RILMac : public Mac {
public:
	RILMac();
	void recv(Packet *p, Handler *h);
	void send(Packet *p, Handler *h, u_int16_t type);

	void waitHandler(void);
	void sendHandler(void);
	void recvHandler(void);
	void ackHandler(void);
	void wakeupHandler(void);
	void radioHandler(void);

	double txtime(Packet *p);

	inline void inc_cw() {
		cw_ = (cw_ << 1) + 1;
		if(cw_ > config_.cw_max) cw_ = config_.cw_max;
	}
	inline void reset_cw() { cw_ = config_.cw_min; sender_cw_ = cw_;}

private:
  RILMac_Config   config_;
  Packet*	        pktRx_;         // recv packet
  Packet*	        pktTx_;         // send packet (not from mac)
  Packet*         pktRRTS_;       // mac beacon packet
  Packet*         pktCTRL_;       // mac ack packet
  MacState        rx_state_;      // incoming state (MAC_RECV or MAC_IDLE)
  MacState        tx_state_;      // outgoing state
  int             tx_active_;     // transmission active flag
  unsigned int    cw_;			      // contention window
  //double          OnRadioTime_;
  double             EstimationType_;
  std::list<neighborEntry> txTable;  // store sender's information
  std::list<neighborEntry> rxTable;  // store receiver's information
  std::list< Packet* > macqueue;  // mac buffer
  u_int16_t retryCount_;          // retry count for one data packet
  u_int16_t thresh_;              // data retransmission threshold
  double mu_;                      // relative delay ratio
  double nextrx_wakeup_;          // scheduled next receive wakeup time
  double nexttx_wakeup_;          // scheduled next transmission wakeup time
  double last_alive_;             // last time a data message is received

  bool pending_rxdata_;           // flag of pending data to be received
  bool pending_txdata_;           // flag of pending data to be sent
  bool more_rxdata_;

  unsigned int sender_cw_;
  //double max_txtime_;
  double last_send_beacon_; double last_receiver_alive_;
  int beaconRetry_;
  int needtoclear_;
  Handler* 	txHandler_;
  RILMacWaitTimer waitTimer;      // CCA and contension backoff timer
  RILMacSendTimer sendTimer;      // pkt transmission timer
  RILMacRecvTimer recvTimer;      // pkt receiption timer
  RILMacACKTimer  ackTimer;       // ack waiting and data retranmission timer
  RILMacWakeupTimer wakeTimer;    // control when to get awake
  RILMacRadioTimer onRadioTimer;  // turn off radio timer

  inline WirelessChargingPhy* newnetif(){return dynamic_cast<WirelessChargingPhy*>(netif_);}

  u_int16_t hdr_mactype(char* hdr, uint16_t type=0);
  u_int16_t hdr_submactype(char* hdr, uint16_t type=0);
  u_int16_t hdr_morebit(char *hdr, int16_t value=-1);
  u_int16_t hdr_backoff(char* hdr, int16_t value = -1);
  u_int32_t hdr_nextwakeup(char* hdr, int32_t value = -1);
  u_int16_t hdr_theta(char* hdr, int16_t value = -1);
  u_int16_t hdr_delta(char* hdr, int16_t value = -1);
  u_int32_t hdr_bound(char *hdr, int32_t value = -1);
  void computeThetaDelta(int receiver, double parameter=0.0);
  bool ifNoOngoingTransaction();
  void dumpStatus();
  double collisionDetection(bool);
  void checktoSendBeacon(double);
  double updateMySenderInfo(int src, double nextt);
  void   updateMyReceiverInfo(int src, double rxbound, double nextt);
  double reschedule(double current);
  double getReceiverWakeupTime(int dst);
  void addBackupSchedule(int receiver, double rxbound);
  void clearReceiverInfo(int src);
  double getBoundTime(int dst);
  double getAliveTime(int dst);
  double updateAliveTime(int dst,double t);
  void tempToSendData(double backofftime, int src, bool mustSend);
  void handleWakeup();

  void generateBeacon(MacFrameSubType type, int dst);
  void sendBeacon(int dst);
  bool sendAck(int, int);
  void RetransmitDATA();

  void recvDATA(Packet* p);
  void recvOndemand(Packet* p);
  void recvACK(Packet* p);
  void recvBeacon(Packet* p);
};

#endif







