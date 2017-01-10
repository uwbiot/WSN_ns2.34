/* 
 * Copyright (c) 2009 Rice University.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. All advertising materials mentioning features or use of this software
 *    must display the following acknowledgement:
 * 4. Neither the name of the University nor of the Laboratory may be used
 *    to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE UNIVERSITY AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE UNIVERSITY OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL 
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * Header file for RI-MAC, created by Yanjun in the Monarch group of 
 * Rice University, 2009
 */
#ifndef ns_rimac_h
#define ns_rimac_h

#include <mac-802_11.h>

// meta beacon (beacon generated when waking up for data transmission
#define YJ_META_BEACON
#ifdef YJ_META_BEACON
// BEACON on demand
#define YJ_BEACON_ON_DEMAND
#endif

typedef enum {YJ_ACTION_BACKOFF_TX_DATA, YJ_ACTION_BACKOFF_TX_RRTS, YJ_ACTION_DEFER_TX_RRTS, YJ_ACTION_DEFER_TX_DATA, YJ_ACTION_DEFER_TX_CTRL, YJ_ACTION_NONE} YJAction;
class RIMAC;
class DiscardTimer : public MacTimer {
public:
	DiscardTimer(Mac802_11 *m) : MacTimer(m) {}

	void	start(double);
	void	handle(Event *e);
};
/* ======================================================================
   The actual RIMAC class.
   ====================================================================== */
class RIMAC : public Mac802_11{
	friend class DeferTimer;
	friend class BackoffTimer;
	friend class IFTimer;
	friend class RxTimer;
	friend class TxTimer;
public:
	RIMAC();
	void		recv(Packet *p, Handler *h);
	inline int	hdr_dst(char* hdr, int dst = -2);
	inline int	hdr_src(char* hdr, int src = -2);
	inline int	hdr_type(char* hdr, u_int16_t type = 0);
	
	inline int bss_id() { return bss_id_; }
	
	// Added by Sushmita to support event tracing
	void trace_event(char *, Packet *);
	EventTrace *et_;

protected:
	virtual void	recvHandler(void);
	virtual void	sendHandler(void);
	virtual void	txHandler(void);

private:
	int		command(int argc, const char*const* argv);

	/*
	 * Called by the timers.
	 */
	void		recv_timer(void);
	void		send_timer(void);
	int		check_pktCTRL();
	int		check_pktTx(bool ignoremedium = false);

	/*
	 * Packet Transmission Functions.
	 */
	void	send(Packet *p, Handler *h);
	void	sendACK(int dst);
	void	sendDATA(Packet *p);
	void	RetransmitDATA();

	/*
	 * Packet Reception Functions.
	 */
	void	recvRTS(Packet *p);
	void	recvCTS(Packet *p);
	void	recvACK(Packet *p);
	void	recvDATA(Packet *p);

	void		capture(Packet *p);
	void		collision(Packet *p);
	void		discard(Packet *p, const char* why);

	/*
	 * Debugging Functions.
	 */
	void		trace_pkt(Packet *p);
	void		dump(char* fname);

	inline int initialized() {	
		return (cache_ && logtarget_
                        && Mac::initialized());
	}

	inline void mac_log(Packet *p) {
                logtarget_->recv(p, (Handler*) 0);
        }

	double txtime(Packet *p);
	double txtime(double psz, double drt);
	double txtime(int bytes) { /* clobber inherited txtime() */ abort(); return 0;}

	inline void transmit(Packet *p, double timeout);
	inline void postBackoff(int pri);


	inline void inc_cw() {
		cw_ = (cw_ << 1) + 1;
		if(cw_ > phymib_.getCWMax())
			cw_ = phymib_.getCWMax();
	}
	inline void rst_cw() { cw_ = phymib_.getCWMin(); }

	inline double sec(double t) { return(t *= 1.0e-6); }
	inline u_int16_t usec(double t) {
		u_int16_t us = (u_int16_t)floor((t *= 1e6) + 0.5);
		return us;
	}

protected:
	PHY_MIB         phymib_;
        MAC_MIB         macmib_;

       /* the macaddr of my AP in BSS mode; for IBSS mode
        * this is set to a reserved value IBSS_ID - the
        * MAC_BROADCAST reserved value can be used for this
        * purpose
        */
       int     bss_id_;
       enum    {IBSS_ID=MAC_BROADCAST};


private:
	double		basicRate_;
 	double		dataRate_;
	
	/*
	 * Mac Timers
	 */
	IFTimer		mhIF_;		// interface timer
	RxTimer		mhRecv_;		// incoming packets
	TxTimer		mhSend_;		// outgoing packets

	DeferTimer	mhDefer_;	// defer timer
	BackoffTimer	mhBackoff_;	// backoff timer

	/* ============================================================
	   Internal MAC State
	   ============================================================ */

	MacState	rx_state_;	// incoming state (MAC_RECV or MAC_IDLE)
	MacState	tx_state_;	// outgoint state
	int		tx_active_;	// transmitter is ACTIVE

	Packet          *eotPacket_;    // copy for eot callback

	Packet		*pktRTS_;	// outgoing RTS packet
	Packet		*pktCTRL_;	// outgoing non-RTS packet

	u_int32_t	cw_;		// Contention Window
	u_int32_t	ssrc_;		// STA Short Retry Count
	u_int32_t	slrc_;		// STA Long Retry Count

	int		min_frame_len_;

//	NsObject*	logtarget_;
	NsObject*       EOTtarget_;     // given a copy of packet at TX end




	/* ============================================================
	   Duplicate Detection state
	   ============================================================ */
	u_int16_t	sta_seqno_;	// next seqno that I'll use
	int		cache_node_count_;
	Host		*cache_;

public:
	// override existing funtions
	bool attemptToSleep(bool forced = false);
	bool wakeup();
	bool is_transmission_attempt_allowed();
	void handleWakeup();
	void idleGuardTimerHandler();
	IdleGuardTimer mhIdle_;
	virtual void	backoffHandler(void);
	virtual void	deferHandler(void);
	void		rx_resume(void);
	// override the function in MAC_802_11
	virtual void QedPktWaitTimerHandler();
	double last_receiver_active_time_;
	bool if_incoming_data_;

	DiscardTimer mhDiscard_;
	virtual void discardTimerHandler();

	// specific to RIMAC
	bool checkPendingBEACON();
	bool ifNoOngoingTransaction(); // whether there is any pending transaction
	bool if_contending_senders_; // whether there are contending senders
	int sender_cw_; // backoff window for senders
	YJAction back_off_timer_action_;
	YJAction defer_timer_action_;
	void setRxState(MacState newState, bool force = false);
	void setTxState(MacState newState, bool force = false);
	void setSleepFlag();
	void unsetSleepFlag();
	void setWaitingFlag();
	void unsetWaitingFlag();
	Packet	*pktRRTS_;	// outgoing RRTS packet
	void sendBeacon(int dst);
	void recvBeacon(Packet *p);
	void generateBeacon();
	int	check_pktRRTS(bool ignoremedium = false);
	int base_beacon_frame_length_; // first beacon after wakeup, this is increased if beacon on request /ack function is added 
	// including preamble (4B), start of frame (1B), length (1B), frame control(2B), RIMAC specific (2B), sender's addres (2B), FCS (2B) , optional field (2B)
	void restoreRIMACStateAfterWakeup(); 
	// retransmission counter for BEACON: continuous failed transmissions
	int beacon_retransmission_counter_;
#ifdef YJ_META_BEACON
	void assignIntendedReceiverToBEACON(Packet *beacon, Packet *data);
#endif


	SINRMonitor sinrMonitor;
	virtual void sinrMonitorHandler();
	double CSThresh_;
	double RXThresh_;

	// if the given pkt's PLCP header is being received 
	bool isPLCPHdrDone(Packet *p);
	double rrts_rcvd_time;
	bool rrts_rcvd_flag;
	Trace*	logtarget_;
	double hello_upper_bound;

	void start_hello();
	MACHelloTimer* hellotimer_;
	void direct_trace(char* fmt, ...);


};
#endif 

