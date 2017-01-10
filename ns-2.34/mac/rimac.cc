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
 * RI-MAC implemention. Created by Yanjun in the Monarch group 
 * of Rice University, 2009
 */

#include "delay.h"
#include "connector.h"
#include "packet.h"
#include "random.h"
#include "mobilenode.h"

#include "arp.h"
#include "ll.h"
#include "mac.h"
#include "mac-timers.h"
#include "wireless-phy.h"
#include "rimac.h"
#include "cmu-trace.h"

#include "agent.h"
#include "basetrace.h"

#include "dsr/hdr_sr.h"
#include <iostream>
#include <set>
using namespace std;
#define TIME_NOW (Scheduler::instance().clock())

inline void
RIMAC::transmit(Packet *p, double timeout)
{
	tx_active_ = 1;
	
	if (EOTtarget_) {
		assert (eotPacket_ == NULL);
		eotPacket_ = p->copy();
	}

	/*
	 * If I'm transmitting without doing CS, such as when
	 * sending an ACK, any incoming packet will be "missed"
	 * and hence, must be discarded.
	 */
	if(rx_state_ != MAC_IDLE) {
		//assert(dh->dh_fc.fc_type == MAC_Type_Control);
		//assert(dh->dh_fc.fc_subtype == MAC_Subtype_ACK);
		assert(pktRx_);
		if( pktRx_ ){
			struct hdr_cmn *ch = HDR_CMN(pktRx_);
			ch->error() = 1;        /* force packet discard */
		}
	}

	/*
	 * pass the packet on the "interface" which will in turn
	 * place the packet on the channel.
	 *
	 * NOTE: a handler is passed along so that the Network
	 *       Interface can distinguish between incoming and
	 *       outgoing packets.
	 */
	downtarget_->recv(p->copy(), this);	
	mhSend_.start(timeout);
	mhIF_.start(txtime(p));
}

void RIMAC::setRxState(MacState newState, bool force)
{
	// ignore any state update if it's sleep state now
	if( rx_state_ == MAC_SLEEP ){
		if( newState != MAC_IDLE )
			fprintf(stdout, "### Warning: rimac %d ignore newstate %d in setRxState because node is sleeping\n at %f\n", index_, newState, TIME_NOW);

		return;
	}

	if( force ){
		rx_state_ = newState;
		return;
	}

	rx_state_ = newState;

	// try to transmit the beacon
	if(checkPendingBEACON()) return;

	if( NULL == pktRRTS_ && NULL == pktTx_ && ifNoOngoingTransaction() )
		attemptToSleep();

}

bool RIMAC::ifNoOngoingTransaction(){
	return (rx_state_ == MAC_IDLE && tx_state_ == MAC_IDLE && NULL == pktCTRL_ &&
			false == mhBackoff_.busy() && false == mhDefer_.busy() &&
			false == mhSend_.busy() && false == mhIF_.busy() && false == mhIdle_.busy()); 
}

void RIMAC::setTxState(MacState newState, bool force)
{
	// ignore any state update if it's sleep state now
	if( tx_state_ == MAC_SLEEP ) return;

	if( force ){
		tx_state_ = newState;
		return;
	}

	tx_state_ = newState;

	// if no pending transaction, try to transmit the beacon
	if(checkPendingBEACON()) return;

	//if( newState == MAC_IDLE && if_want_sleep_ && rx_state_ == MAC_IDLE && attemptToSleep())
	//	return;
	if( NULL == pktRRTS_ && NULL == pktTx_ && ifNoOngoingTransaction() )
		attemptToSleep();
}


/* ======================================================================
   TCL Hooks for the simulator
   ====================================================================== */
static class RIMACClass : public TclClass {
public:
	RIMACClass() : TclClass("Mac/RIMAC") {}
	TclObject* create(int, const char*const*) {
	return (new RIMAC());

}
} class_RIMAC;


/* ======================================================================
   Mac Class Functions
   ====================================================================== */
RIMAC::RIMAC() : 
	Mac802_11(), phymib_(this), macmib_(this), mhIF_(this), 
	mhRecv_(this), mhSend_(this), 
	mhDefer_(this), mhBackoff_(this)
	, mhIdle_(this)
	, mhDiscard_(this)
	, sinrMonitor(this)
{
	
	tx_state_ = rx_state_ = MAC_IDLE;
	tx_active_ = 0;
	eotPacket_ = NULL;
	pktRTS_ = 0;
	pktCTRL_ = 0;		
	cw_ = phymib_.getCWMin();
	ssrc_ = slrc_ = 0;
	// Added by Sushmita
        et_ = new EventTrace();
	
	sta_seqno_ = 1;
	cache_ = 0;
	cache_node_count_ = 0;
	
	// chk if basic/data rates are set
	// otherwise use bandwidth_ as default;
	
	Tcl& tcl = Tcl::instance();
	tcl.evalf("Mac/802_11 set basicRate_");
	if (strcmp(tcl.result(), "0") != 0) 
		bind_bw("basicRate_", &basicRate_);
	else
		basicRate_ = bandwidth_;

	tcl.evalf("Mac/802_11 set dataRate_");
	if (strcmp(tcl.result(), "0") != 0) 
		bind_bw("dataRate_", &dataRate_);
	else
		dataRate_ = bandwidth_;

	EOTtarget_ = 0;
	bss_id_ = IBSS_ID;


	bind("CSThresh_", &CSThresh_);
	bind("RXThresh_", &RXThresh_);
	bind("DutyCycleLength_", &duty_cycle_length_);
	tx_state_ = rx_state_ = MAC_SLEEP;

	pktRRTS_ = 0;

	// in sleep state initially
	unsetSleepFlag();
	last_radio_state_update_time_ = 0.;
	duty_cycle_stat_started_ = false;
	bind("base_beacon_frame_length", &base_beacon_frame_length_);
	if_contending_senders_ = false;
	sender_cw_ = phymib_.getCWMin();
	back_off_timer_action_ = YJ_ACTION_NONE;

	last_receiver_active_time_ = 0.;
}


int
RIMAC::command(int argc, const char*const* argv)
{
	if (argc == 2) {
		if (strcmp(argv[1], "start-neighbor") == 0) {
			if( index_ ){
				start_neighbourdetect();
				start_hello();
			}
			return TCL_OK;
		} else if (strcmp(argv[1], "updateDutyCycle") == 0) {
			//printf("node %d updateDutyCycle at %f\n", index_, TIME_NOW);
			// handle the corner where a node never sleep/wakeup after START_MEASUREMENT_THRESHOLD
			if( duty_cycle_stat_started_ == false ){
				last_radio_state_update_time_ = START_MEASUREMENT_THRESHOLD;
			}
			// dutycycle counter
			if( tx_state_ != MAC_SLEEP ){
				God::instance()->total_active_time_ += TIME_NOW - last_radio_state_update_time_;
				God::instance()->duty_cycle_active_[index_] += TIME_NOW - last_radio_state_update_time_;
			} else {
				God::instance()->total_sleep_time_ += TIME_NOW - last_radio_state_update_time_;
				God::instance()->duty_cycle_sleep_[index_] += TIME_NOW - last_radio_state_update_time_;
			}

			last_radio_state_update_time_ = TIME_NOW;

			return TCL_OK;
		}
	} else if (argc == 3) {
		if (strcmp(argv[1], "eot-target") == 0) {
			EOTtarget_ = (NsObject*) TclObject::lookup(argv[2]);
			if (EOTtarget_ == 0)
				return TCL_ERROR;
			return TCL_OK;
		} else if (strcmp(argv[1], "get-ifq") == 0) {
			ifq_ = (CMUPriQueue *) TclObject::lookup(argv[2]);
			if(ifq_ == 0) return TCL_ERROR;
			return TCL_OK;
		} else if (strcmp(argv[1], "bss_id") == 0) {
			bss_id_ = atoi(argv[2]);
			return TCL_OK;
		} else if (strcmp(argv[1], "log-target") == 0) { 
			logtarget_ = (Trace*) TclObject::lookup(argv[2]);
			//logtarget_ = (NsObject*) TclObject::lookup(argv[2]);
			if(logtarget_ == 0)
				return TCL_ERROR;
			return TCL_OK;
		} else if(strcmp(argv[1], "nodes") == 0) {
			if(cache_) return TCL_ERROR;
			cache_node_count_ = atoi(argv[2]);
			cache_ = new Host[cache_node_count_ + 1];
			assert(cache_);
			bzero(cache_, sizeof(Host) * (cache_node_count_+1 ));
			return TCL_OK;
		} else if(strcmp(argv[1], "eventtrace") == 0) {
			// command added to support event tracing by Sushmita
                        et_ = (EventTrace *)TclObject::lookup(argv[2]);
                        return (TCL_OK);
                }
	}
	return Mac::command(argc, argv);
}

// Added by Sushmita to support event tracing
void RIMAC::trace_event(char *eventtype, Packet *p) 
{
        if (et_ == NULL) return;
        char *wrk = et_->buffer();
        char *nwrk = et_->nbuffer();
	
        //char *src_nodeaddr =
	//       Address::instance().print_nodeaddr(iph->saddr());
        //char *dst_nodeaddr =
        //      Address::instance().print_nodeaddr(iph->daddr());
	
        struct hdr_mac802_11* dh = HDR_MAC802_11(p);
	
        //struct hdr_cmn *ch = HDR_CMN(p);
	
	if(wrk != 0) {
		sprintf(wrk, "E -t "TIME_FORMAT" %s %2x ",
			et_->round(Scheduler::instance().clock()),
                        eventtype,
                        //ETHER_ADDR(dh->dh_sa)
                        ETHER_ADDR(dh->dh_ta)
                        );
        }
        if(nwrk != 0) {
                sprintf(nwrk, "E -t "TIME_FORMAT" %s %2x ",
                        et_->round(Scheduler::instance().clock()),
                        eventtype,
                        //ETHER_ADDR(dh->dh_sa)
                        ETHER_ADDR(dh->dh_ta)
                        );
        }
        et_->dump();
}

/* ======================================================================
   Debugging Routines
   ====================================================================== */
void
RIMAC::trace_pkt(Packet *p) 
{
	struct hdr_cmn *ch = HDR_CMN(p);
	struct hdr_mac802_11* dh = HDR_MAC802_11(p);
	u_int16_t *t = (u_int16_t*) &dh->dh_fc;

	fprintf(stderr, "\t[ %2x %2x %2x %2x ] %x %s %d\n",
		*t, dh->dh_duration,
		 ETHER_ADDR(dh->dh_ra), ETHER_ADDR(dh->dh_ta),
		index_, packet_info.name(ch->ptype()), ch->size());
}

void
RIMAC::dump(char *fname)
{
	fprintf(stderr,
		"\n%s --- (INDEX: %d, time: %2.9f)\n",
		fname, index_, Scheduler::instance().clock());

	fprintf(stderr,
		"\ttx_state_: %x, rx_state_: %x, idle: %d\n",
		tx_state_, rx_state_, is_transmission_attempt_allowed());

	fprintf(stderr,
		"\tpktTx_: %lx, pktRx_: %lx, pktRTS_: %lx, pktCTRL_: %lx, callback: %lx\n",
		(long) pktTx_, (long) pktRx_, (long) pktRTS_,
		(long) pktCTRL_, (long) callback_);

	fprintf(stderr,
		"\tDefer: %d, Backoff: %d, Recv: %d, Timer: %d ",
		mhDefer_.busy(), mhBackoff_.busy(), 
		mhRecv_.busy(), mhSend_.busy());
	fprintf(stderr,
		"\tBackoff Expire: %f\n",
		mhBackoff_.expire());
}


/* ======================================================================
   Packet Headers Routines
   ====================================================================== */
inline int
RIMAC::hdr_dst(char* hdr, int dst )
{
	struct hdr_mac802_11 *dh = (struct hdr_mac802_11*) hdr;
	
       if (dst > -2) {
               if ((bss_id() == ((int)IBSS_ID)) || (addr() == bss_id())) {
                       /* if I'm AP (2nd condition above!), the dh_3a
                        * is already set by the MAC whilst fwding; if
                        * locally originated pkt, it might make sense
                        * to set the dh_3a to myself here! don't know
                        * how to distinguish between the two here - and
                        * the info is not critical to the dst station
                        * anyway!
                        */
                       STORE4BYTE(&dst, (dh->dh_ra));
               } else {
                       /* in BSS mode, the AP forwards everything;
                        * therefore, the real dest goes in the 3rd
                        * address, and the AP address goes in the
                        * destination address
                        */
                       STORE4BYTE(&bss_id_, (dh->dh_ra));
                       STORE4BYTE(&dst, (dh->dh_3a));
               }
       }


       return (u_int32_t)ETHER_ADDR(dh->dh_ra);
}

inline int 
RIMAC::hdr_src(char* hdr, int src )
{
	struct hdr_mac802_11 *dh = (struct hdr_mac802_11*) hdr;
        if(src > -2)
               STORE4BYTE(&src, (dh->dh_ta));
        return ETHER_ADDR(dh->dh_ta);
}

inline int 
RIMAC::hdr_type(char* hdr, u_int16_t type)
{
	struct hdr_mac802_11 *dh = (struct hdr_mac802_11*) hdr;
	if(type)
		STORE2BYTE(&type,(dh->dh_body));
	return GET2BYTE(dh->dh_body);
}


/* ======================================================================
   Misc Routines
   ====================================================================== */
void
RIMAC::discard(Packet *p, const char* why)
{
	hdr_mac802_11* mh = HDR_MAC802_11(p);
	hdr_cmn *ch = HDR_CMN(p);

	/* if the rcvd pkt contains errors, a real MAC layer couldn't
	   necessarily read any data from it, so we just toss it now */
	if(ch->error() != 0) {
		Packet::free(p);
		return;
	}

	switch(mh->dh_fc.fc_type) {
	case MAC_Type_Management:
		drop(p, why);
		return;
	case MAC_Type_Control:
		switch(mh->dh_fc.fc_subtype) {
		case MAC_Subtype_RRTS:
			Packet::free(p);
			return;
		case MAC_Subtype_RTS:
			 if((u_int32_t)ETHER_ADDR(mh->dh_ta) ==  (u_int32_t)index_) {
				drop(p, why);
				return;
			}
			/* fall through - if necessary */
		case MAC_Subtype_ACK:
			Packet::free(p);
			return;
		default:
			fprintf(stderr, "discard: invalid MAC Control subtype at node %d at %f\n", index_, TIME_NOW);
			exit(1);
		}
		break;
	case MAC_Type_Data:
		switch(mh->dh_fc.fc_subtype) {
		case MAC_Subtype_Data:
			if((u_int32_t)ETHER_ADDR(mh->dh_ra) == \
                           (u_int32_t)index_ ||
                          (u_int32_t)ETHER_ADDR(mh->dh_ta) == \
                           (u_int32_t)index_ ||
                          (u_int32_t)ETHER_ADDR(mh->dh_ra) == MAC_BROADCAST) {
                                drop(p,why);
                                return;
			}
			break;
		default:
			fprintf(stderr, "invalid MAC Data subtype\n");
			exit(1);
		}
		break;
	default:
		fprintf(stderr, "invalid MAC type (%x)\n", mh->dh_fc.fc_type);
		trace_pkt(p);
		exit(1);
	}
	Packet::free(p);
}

void
RIMAC::capture(Packet *p)
{

	Packet::free(p);
}


bool RIMAC::isPLCPHdrDone(Packet *p){
	struct hdr_cmn *ch = HDR_CMN(p);
	if( ch->arrival_time_ + 8.*phymib_.getPLCPhdrLen()/ phymib_.getPLCPDataRate() >= NOW ){
		return true;
	} else {
		return false;
	}
}


void
RIMAC::collision(Packet *p)
{
	switch(rx_state_) {
	case MAC_RECV:
		setRxState(MAC_COLL);
		/* fall through */
	case MAC_COLL:
		assert(pktRx_);
		assert(mhRecv_.busy());
		/*
		 *  Since a collision has occurred, figure out
		 *  which packet that caused the collision will
		 *  "last" the longest.  Make this packet,
		 *  pktRx_ and reset the Recv Timer if necessary.
		 */
		if( pktRx_){
			/* sanity 
			if(!mhRecv_.busy()){
				cerr << "weird here2" << endl;
				exit(2);
			}*/

			mhRecv_.stop();
			discard(pktRx_, DROP_MAC_COLLISION);
			pktRx_ = 0;
		} 
		discard(p, DROP_MAC_COLLISION);
		rx_resume();
		break;
	default:
		assert(0);
	}
}

/* return true if backoff timer is started for BEACON*/
bool RIMAC::checkPendingBEACON(){
	if(pktRRTS_ && ifNoOngoingTransaction() ){
#ifdef YJ_BEACON_ON_DEMAND
		// if this is a beacon_on_demand, use max backoff window to avoid interfering with
		// other flows
		if( ((struct rrts_frame*)pktRRTS_->access(hdr_mac::offset_))->on_demand )
			mhBackoff_.start(0, true, phymib_.getCWMax() * phymib_.getSlotTime() );
		else
#endif
			mhBackoff_.start(cw_, true, phymib_.getDIFS());

		back_off_timer_action_ = YJ_ACTION_BACKOFF_TX_RRTS;
		return true;
	} else {
		return false;
	}
}

void
RIMAC::rx_resume()
{
	assert(pktRx_ == 0);
	assert(mhRecv_.busy() == 0);


	if(sinrMonitor.channelClear(NOW, CSThresh_)){
		setRxState(MAC_IDLE);
	} else {
		setRxState(MAC_COLL);
		sinrMonitor.start(sinrMonitor.head_->finish_time_ - NOW);
	}
}


/* ======================================================================
   Timer Handler Routines
   ====================================================================== */
void
RIMAC::backoffHandler()
{
	if( back_off_timer_action_ == YJ_ACTION_BACKOFF_TX_RRTS ){
		// sanity
		if( pktRRTS_ == 0 ){
			fprintf(stderr, "rimac %d backoffHandler called without a BEACON to transmit at %f\n", index_, TIME_NOW);
			exit(1);
		}
		check_pktRRTS();
	} else if( back_off_timer_action_ == YJ_ACTION_BACKOFF_TX_DATA ){
		// sanity
		if( pktTx_ == 0 ){
			fprintf(stderr, "rimac %d backoffHandler called without a DATA to transmit at %f\n", index_, TIME_NOW);
			exit(1);
		}
		check_pktTx();
	} else {
		fprintf(stderr, "rimac %d backoffHandler called without an action at %f\n", index_, TIME_NOW);
		exit(1);
	}
}

void
RIMAC::deferHandler()
{
	switch (defer_timer_action_){
		case YJ_ACTION_DEFER_TX_CTRL:
			check_pktCTRL();
			break;
		case YJ_ACTION_DEFER_TX_RRTS:
			check_pktRRTS();
			break;
		case YJ_ACTION_DEFER_TX_DATA:
			check_pktTx(true);
			break;
		default:
			// sanity
			fprintf(stderr, "rimac %d deferHandler called without any packet to transmit at %f\n", index_, TIME_NOW);
			exit(1);
	}
}

void
RIMAC::recvHandler()
{
	recv_timer();
}

void
RIMAC::sendHandler()
{
	send_timer();
}

void RIMAC::sinrMonitorHandler()
{
	/* sanity check
	if( rx_state_ != MAC_COLL ){
		cerr << "RIMAC::sinrMonitorHandler unexpected state" << endl;
		exit(1);
	}*/
	if(sinrMonitor.channelClear(NOW, CSThresh_)){
		setRxState(MAC_IDLE);
	} else {
		setRxState(MAC_COLL);
		sinrMonitor.start(sinrMonitor.head_->finish_time_ - NOW);
	}
}

void
RIMAC::txHandler()
{
	if (EOTtarget_) {
		assert(eotPacket_);
		EOTtarget_->recv(eotPacket_, (Handler *) 0);
		eotPacket_ = NULL;
	}
	tx_active_ = 0;
}


/* ======================================================================
   The "real" Timer Handler Routines
   ====================================================================== */
void
RIMAC::send_timer()
{
//	fprintf(stdout, "rimac %d send_timer called at %f\n\n", index_, TIME_NOW);
	switch(tx_state_) {
	/*
	 * Sent DATA, but did not receive an ACK packet.
	 */
	case MAC_SEND:
		RetransmitDATA();
		break;
	/*
	 * Sent an ACK, and now ready to resume transmission.
	 */
	case MAC_ACK:
		// if idle timer (collision detection) is busy, ignore the rest; as a packet might being received 
		if( mhIdle_.busy() ){
			return;
		}

		Packet::free(pktCTRL_); 
		pktCTRL_ = 0;
		// no other incoming packet
		unsetWaitingFlag();
		if_contending_senders_ = false;

			setTxState(MAC_IDLE, true); 
			if( NULL == pktTx_ && ifNoOngoingTransaction() ){
				attemptToSleep(true); // go to sleep immediately
				return;
			}

		break;
	case MAC_IDLE:
		break;
	case MAC_RIMACBEACON:
		// if idle timer (collision detection) is busy, ignore the rest; as a packet might being received 
		if( mhIdle_.busy() ){
			return;
		}
		Packet::free(pktRRTS_); pktRRTS_ = 0;
		// no other incoming packet
		unsetWaitingFlag();
		if_contending_senders_ = false;
		setTxState(MAC_IDLE, true); 
		if( NULL == pktTx_ && ifNoOngoingTransaction() ){
			attemptToSleep(true); // go to sleep immediately
			return;
		}

		break;
	default:
		assert(0);
	}

	setTxState(MAC_IDLE); 
}


/* ======================================================================
   Outgoing Packet Routines
   ====================================================================== */
int
RIMAC::check_pktCTRL()
{
	struct hdr_mac802_11 *mh = 0;
	double timeout = 0;

	if(pktCTRL_ == 0)
		return -1;
	if(tx_state_ == MAC_ACK)
		return -1;

	mh = HDR_MAC802_11(pktCTRL_);

	hdr_cmn* ch = HDR_CMN(pktCTRL_);
	int beacon_size = base_beacon_frame_length_;
	// if contention window is not 0, include one more byte to the beacon
	if( sender_cw_ ){
		ch->size() += 1;
	}
#ifdef YJ_META_BEACON
	// include one more byte to the beacon in order to hold dst info
	if(pktTx_){
		ch->size() += 2;
	}
#endif
							  
	switch(mh->dh_fc.fc_subtype) {
	case MAC_Subtype_ACK:		
		setTxState(MAC_ACK);
		// this extended timeout to allow another incoming packet
		setWaitingFlag();
		timeout = txtime(beacon_size, basicRate_)
			+ DSSS_MaxPropagationDelay                      
			+ phymib_.getSIFS()
			+ DSSS_MaxPropagationDelay;

		// let potential senders to use this backoff window
		((struct ack_frame*) mh)->cw = sender_cw_;
		
#ifdef YJ_META_BEACON
		// assign intended receiver to the BEACON
		if( pktTx_ ){
			assignIntendedReceiverToBEACON(pktCTRL_, pktTx_);
		}
#endif

		if(mh->dh_fc.fc_retry){
			timeout += sender_cw_ * phymib_.getSlotTime(); 
		} 
		break;
	default:
		fprintf(stderr, "check_pktCTRL:Invalid MAC Control subtype\n");
		exit(1);
	}
	transmit(pktCTRL_, timeout);
	return 0;
}

int
RIMAC::check_pktTx(bool ignoremedium)
{
	struct hdr_mac802_11 *mh = 0;
	double timeout = 0;
	
	assert(mhBackoff_.busy() == 0);

	if(pktTx_ == 0)
		return -1;

	mh = HDR_MAC802_11(pktTx_);

	switch(mh->dh_fc.fc_subtype) {
	case MAC_Subtype_Data:
		if(false == ignoremedium && !is_transmission_attempt_allowed()) {
			// nothing to do, wait for next BEACON/ACK
			back_off_timer_action_ = YJ_ACTION_NONE;
			return 0;
		}
		setTxState(MAC_SEND);
		if((u_int32_t)ETHER_ADDR(mh->dh_ra) != MAC_BROADCAST){
			// in case of collision, start retry immediately after the beacon
			// rather than going through retransmission function
			// yj mar 30, 2008; beacon triggers retransmission rather than send timeout
			timeout = phymib_.getSlotTime() * phymib_.getCWMax() 
				+ txtime(YJ_MAX_FRAME_SIZE,basicRate_)
			   + DSSS_MaxPropagationDelay              
			   + phymib_.getSIFS()
			   + txtime(phymib_.getACKlen(), basicRate_)
			   + DSSS_MaxPropagationDelay;             
		}
		else
			timeout = txtime(pktTx_);
		break;
	default:
		fprintf(stderr, "check_pktTx:Invalid MAC Control subtype\n");
		exit(1);
	}
	transmit(pktTx_, timeout);
	return 0;
}
void
RIMAC::sendACK(int dst)
{
	Packet *p = Packet::alloc();
	hdr_cmn* ch = HDR_CMN(p);
	struct ack_frame *af = (struct ack_frame*)p->access(hdr_mac::offset_);

	assert(pktCTRL_ == 0);

	ch->uid() = 0;
	ch->ptype() = PT_MAC;
	ch->size() = phymib_.getACKlen();
	ch->iface() = -2;
	ch->error() = 0;
	
	bzero(af, MAC_HDR_LEN);

	af->af_fc.fc_protocol_version = MAC_ProtocolVersion;
 	af->af_fc.fc_type	= MAC_Type_Control;
 	af->af_fc.fc_subtype	= MAC_Subtype_ACK;
 	af->af_fc.fc_to_ds	= 0;
 	af->af_fc.fc_from_ds	= 0;
 	af->af_fc.fc_more_frag	= 0;
 	af->af_fc.fc_retry	= 0;
 	af->af_fc.fc_pwr_mgt	= 0;
 	af->af_fc.fc_more_data	= 0;
 	af->af_fc.fc_wep	= 0;
 	af->af_fc.fc_order	= 0;

	STORE4BYTE(&dst, (af->af_ra));
	// Because this ACK also servers as a BEACON, sender address is needed
	STORE4BYTE(&index_, (af->af_ta));
	HDR_MAC802_11(p)->dh_fc.fc_retry = if_contending_senders_;

	/* store ack tx time */
 	ch->txtime() = txtime(ch->size(), basicRate_);
	
	/* calculate ack duration */
 	af->af_duration = 0;	
	
	pktCTRL_ = p;
}

void
RIMAC::sendDATA(Packet *p)
{
	hdr_cmn* ch = HDR_CMN(p);
	struct hdr_mac802_11* dh = HDR_MAC802_11(p);

	assert(pktTx_ == 0);

	/*
	 * Update the MAC header
	 */
	ch->size() += phymib_.getHdrLen11();

	dh->dh_fc.fc_protocol_version = MAC_ProtocolVersion;
	dh->dh_fc.fc_type       = MAC_Type_Data;
	dh->dh_fc.fc_subtype    = MAC_Subtype_Data;
	
	dh->dh_fc.fc_to_ds      = 0;
	dh->dh_fc.fc_from_ds    = 0;
	dh->dh_fc.fc_more_frag  = 0;
	dh->dh_fc.fc_retry      = 0;
	dh->dh_fc.fc_pwr_mgt    = 0;
	dh->dh_fc.fc_more_data  = 0;
	dh->dh_fc.fc_wep        = 0;
	dh->dh_fc.fc_order      = 0;

	/* store data tx time */
 	ch->txtime() = txtime(ch->size(), dataRate_);

	if((u_int32_t)ETHER_ADDR(dh->dh_ra) != MAC_BROADCAST) {
		/* store data tx time for unicast packets */
		ch->txtime() = txtime(ch->size(), dataRate_);
		
		dh->dh_duration = usec(txtime(phymib_.getACKlen(), basicRate_)
				       + phymib_.getSIFS());



	} else {
		/* store data tx time for broadcast packets (see 9.6) */
		ch->txtime() = txtime(ch->size(), basicRate_);
		
		dh->dh_duration = 0;
	}
	pktTx_ = p;
}

/* ======================================================================
   Retransmission Routines
   ====================================================================== */

void
RIMAC::RetransmitDATA()
{
	struct hdr_cmn *ch;
	struct hdr_mac802_11 *mh;
	u_int32_t *rcount, thresh;
	assert(mhBackoff_.busy() == 0);

	assert(pktTx_);
	assert(pktRTS_ == 0);

	ch = HDR_CMN(pktTx_);
	mh = HDR_MAC802_11(pktTx_);

	/*
	 *  Broadcast packets don't get ACKed and therefore
	 *  are never retransmitted.
	 */
	if((u_int32_t)ETHER_ADDR(mh->dh_ra) == MAC_BROADCAST) {
		Packet::free(pktTx_); 
		pktTx_ = 0;
		if(callback_) {
			Handler *h = callback_;
			callback_ = 0;
			h->handle((Event*) 0);
		} 

		/*
		 * Backoff at end of TX.
		 */
		rst_cw();
		return;
	}


	macmib_.ACKFailureCount++;

	rcount = &slrc_;
	thresh = macmib_.getLongRetryLimit();

	(*rcount)++;

	if(*rcount >= thresh) {
		if( mhDiscard_.busy() ){
			mhDiscard_.stop();
		}

		/* IEEE Spec section 9.2.3.5 says this should be greater than
		   or equal */
		macmib_.FailedCount++;
		/* tell the callback the send operation failed 
		   before discarding the packet */
		hdr_cmn *ch = HDR_CMN(pktTx_);
		if (ch->xmit_failure_) {
			ch->size() -= phymib_.getHdrLen11();
			ch->xmit_reason_ = XMIT_REASON_ACK;
			ch->xmit_failure_(pktTx_->copy(), ch->xmit_failure_data_);
		}

		*rcount = 0;
		rst_cw();
		discard(pktTx_, DROP_MAC_RETRY_COUNT_EXCEEDED); 
		pktTx_ = 0;
		if(callback_) {
			Handler *h = callback_;
			callback_ = 0;
			h->handle((Event*) 0);
		} 
	}
	else {
		struct hdr_mac802_11 *dh;
		dh = HDR_MAC802_11(pktTx_);
		dh->dh_fc.fc_retry = 1;

		if( mhDiscard_.busy() ){
			mhDiscard_.stop();
		}
		mhDiscard_.start(2*duty_cycle_length_);

	}
}

/* ======================================================================
   Incoming Packet Routines
   ====================================================================== */
void
RIMAC::send(Packet *p, Handler *h)
{
	struct hdr_mac802_11* dh = HDR_MAC802_11(p);

	int dst = ETHER_ADDR(dh->dh_ra);
	callback_ = h;
	sendDATA(p);

	/*
	 * Assign the data packet a sequence number.
	 */
	dh->dh_scontrol = sta_seqno_++;


	if( mhDiscard_.busy() ){
		mhDiscard_.stop();
	}
	mhDiscard_.start(2*duty_cycle_length_);

	myneighbor_list_item *pn = getNeighbor(dst);
	// if this node has no knowledge on the neighbor yet, do nothing and keep listening
	if( pn == NULL ) {
		goto metabeacon;
	}
	// if the intended receipient is still awake
	if( TIME_NOW <= pn->transmission_bound_time_ && ifNoOngoingTransaction() ){
		// stop any pending timers
		if( mhDefer_.busy() ) {
			mhDefer_.stop();
			defer_timer_action_ = YJ_ACTION_NONE;
		}
		if( mhBackoff_.busy() ) {
			mhBackoff_.stop();
			back_off_timer_action_ = YJ_ACTION_NONE;
		}
		if( pn->if_collision_ ){
			mhBackoff_.start(pn->cw_, true, phymib_.getDIFS());
			back_off_timer_action_ = YJ_ACTION_BACKOFF_TX_DATA;
		} else {
			mhDefer_.start(phymib_.getSIFS());
			defer_timer_action_ = YJ_ACTION_DEFER_TX_DATA;
			tx_active_ = 1;
		}
		return;
	} 

metabeacon:

#ifdef YJ_META_BEACON
	// try to transmit a beacon: for fast turn around and collision avoidance
	if(checkPendingBEACON()) return;
#endif

	return;

}

void
RIMAC::recv(Packet *p, Handler *h)
{
	struct hdr_cmn *hdr = HDR_CMN(p);
	/*
	 * Sanity Check
	 */
	assert(initialized()) ;

	/*
	 *  Handle outgoing packets.
	 */
	if(hdr->direction() == hdr_cmn::DOWN) {

		if( mhQedPktWaiter_.busy() ){
			mhQedPktWaiter_.stop();
		}

		// if radio is sleeping, wakeup
		if( tx_state_ == MAC_SLEEP ){

#ifdef YJ_META_BEACON
			// generate a BEACON, and put intended receiver into it
			if( NULL == pktRRTS_ )
				generateBeacon();
			// set sender_cw_, cw for beacon and etc..
			restoreRIMACStateAfterWakeup();
#endif

			wakeup();


			if(false == sinrMonitor.channelClear(TIME_NOW, CSThresh_)){
				// detected busy medium, stay awake to get possible incoming packet
				setRxState(MAC_COLL); 
				// start timer of sinrMonitor
				if( !sinrMonitor.busy() )
					sinrMonitor.start(sinrMonitor.head_->finish_time_ - NOW);
			}

		}

		send(p, h);
		return;
	}
	/*
	 *  Handle incoming packets.
	 *
	 *  We just received the 1st bit of a packet on the network
	 *  interface.
	 *
	 */


	if( hdr->below_cs_threshold_ ){
		sinrMonitor.add(txtime(p), p, NOW);
		Packet::free(p);
		if(false == sinrMonitor.channelClear(NOW, CSThresh_)){
			if( mhBackoff_.busy() ) {
				mhBackoff_.stop();
				back_off_timer_action_ = YJ_ACTION_NONE;
			}
		}

		return;
	}

	// if node is sleeping
	if( tx_state_ == MAC_SLEEP ){
		sinrMonitor.add(txtime(p), p, NOW);
		Packet::free(p);
		return;
	}

	/*
	 *  If the interface is currently in transmit mode, then
	 *  it probably won't even see this packet.  However, the
	 *  "air" around me is BUSY so I need to let the packet
	 *  proceed.  Just set the error flag in the common header
	 *  to that the packet gets thrown away.
	 */
	if(tx_active_ && hdr->error() == 0) {
		hdr->error() = 1;
	}

	hdr->arrival_time_ = NOW;

	// cancel send timer for RRTS if receiving signal strength is greater than RXThresh_
	if( tx_active_ == false && ( (sinrMonitor.getTotal(TIME_NOW)+p->txinfo_.RxPr) > RXThresh_) ){
		if( mhSend_.busy() ){
			if(tx_state_ == MAC_ACK || tx_state_ == MAC_RIMACBEACON ){
				// start timer to detect whether collision has happened
				// a successfully arrived DATA frame will cancel this timer
				if( false == mhIdle_.busy() )
					mhIdle_.start(txtime(YJ_MAX_FRAME_SIZE, basicRate_));
			} 
		} 
	}


	if( mhBackoff_.busy() ) {
		mhBackoff_.stop();
		back_off_timer_action_ = YJ_ACTION_NONE;
	}

	if(rx_state_ == MAC_IDLE) {
		setRxState(MAC_RECV);
		pktRx_ = p;
		/*
		 * Schedule the reception of this packet, in
		 * txtime seconds.
		 */
		mhRecv_.start(txtime(p));
		sinrMonitor.add(txtime(pktRx_), pktRx_, NOW);
	} else {

		if( sinrMonitor.busy() ){
			sinrMonitor.stop();
		}

#ifdef MAC_WITH_CAPTURE // if capture effect is enabled
		/*
		 *  If the power of the incoming packet is smaller than the
		 *  power of the packet currently being received by at least
                 *  the capture threshold, then we ignore the new packet.
		 */
		double signalpw = sinrMonitor.getTotal(NOW);
		// use accumulative energy instead of p's energy
		if(pktRx_ && pktRx_->txinfo_.RxPr / (signalpw - pktRx_->txinfo_.RxPr + p->txinfo_.RxPr) >= p->txinfo_.CPThresh) {
			sinrMonitor.add(txtime(p), p, NOW);
			// keep original rx state
			capture(p);
		} 
		/* signalpw is less than CSThresh_ when it's at the end of the previous pkt receipt */
		else if(signalpw < CSThresh_ || p->txinfo_.RxPr / signalpw >= p->txinfo_.CPThresh){
			sinrMonitor.add(txtime(p), p, NOW);
			if( pktRx_ ){
				capture(pktRx_); 
				mhRecv_.stop();
			} 
			setRxState(MAC_RECV);
			pktRx_ = p;
			mhRecv_.start(txtime(pktRx_));
		}

		else
#endif //MAC_WITH_CAPTURE
		{
			sinrMonitor.add(txtime(p), p,NOW);
			collision(p);
		}
	}
}

void
RIMAC::recv_timer()
{
	//u_int32_t src; 
	hdr_cmn *ch = HDR_CMN(pktRx_);
	hdr_mac802_11 *mh = HDR_MAC802_11(pktRx_);
	u_int32_t dst = ETHER_ADDR(mh->dh_ra);
	u_int32_t src = ETHER_ADDR(mh->dh_ta);
	
	u_int8_t  type = mh->dh_fc.fc_type;
	u_int8_t  subtype = mh->dh_fc.fc_subtype;

	assert(pktRx_);
	assert(rx_state_ == MAC_RECV || rx_state_ == MAC_COLL);
	
        /*
         *  If the interface is in TRANSMIT mode when this packet
         *  "arrives", then I would never have seen it and should
         *  do a silent discard without adjusting the NAV.
         */
        if(tx_active_) {
                Packet::free(pktRx_);
                goto done;
        }

	/*
	 * Handle collisions.
	 */
	if(rx_state_ == MAC_COLL) {
		fprintf(stderr, "RIMAC:recv_timer collided pkts should already have been dropped\n");
		exit(1);
		discard(pktRx_, DROP_MAC_COLLISION);		

		goto done;
	}

	/*
	 * Check to see if this packet was received with enough
	 * bit errors that the current level of FEC still could not
	 * fix all of the problems - ie; after FEC, the checksum still
	 * failed.
	 */
	if( ch->error() ) {
		Packet::free(pktRx_);
		goto done;
	}

	/* tap out - */
	if (tap_ && type == MAC_Type_Data &&
		MAC_Subtype_Data == subtype ) 
	tap_->tap(pktRx_);
	/*
	 * Adaptive Fidelity Algorithm Support - neighborhood infomation 
	 * collection
	 *
	 * Hacking: Before filter the packet, log the neighbor node
	 * I can hear the packet, the src is my neighbor
	 */

	// ACK packet for other node is processed as a BEACON still be passed through
	if( dst != (u_int32_t) index_ && subtype == MAC_Subtype_ACK && type == MAC_Type_Control ){
		// change ACK to BEACON
		subtype = mh->dh_fc.fc_subtype = MAC_Subtype_RRTS;
		// set destination to broadcast address
		int tmpaddr = MAC_BROADCAST;
		STORE4BYTE(&tmpaddr, (mh->dh_ra));
		// reset dst
		dst = MAC_BROADCAST;
	}

	// if this is not a frame triggered by my BEACON/ACK, stop idle timer
	if(dst != (u_int32_t)index_ || dst == MAC_BROADCAST) {
		if( mhIdle_.busy() ){
			mhIdle_.stop();
			// clear state of TX (possibly skipped in send_timer) when mhSend_ expires earlier
			if(false == mhSend_.busy()){
				// no incoming pkt
				if( tx_state_ == MAC_ACK ){
					Packet::free(pktCTRL_); pktCTRL_ = 0;
				} else {
					Packet::free(pktRRTS_); pktRRTS_ = 0;
				}

				setTxState(MAC_IDLE, true);

				// no other incoming packet
				unsetWaitingFlag();
				// no reset in ADMI
				if_contending_senders_ = false;

			} else {
				// if a beacon is receivd and I have a pending data, cancel pending mhSend_ to allow
				// immediate data transmission
				if( pktTx_ && subtype == MAC_Subtype_RRTS && type == MAC_Type_Control
					&& src == (u_int32_t) ETHER_ADDR(HDR_MAC802_11(pktTx_)->dh_ra)){
					if( tx_state_ == MAC_ACK ){
						Packet::free(pktCTRL_); pktCTRL_ = 0;
					} else {
						Packet::free(pktRRTS_); pktRRTS_ = 0;
					}
					mhSend_.stop();
					setTxState(MAC_IDLE, true);
					//if_contending_senders_ = false;
				}
			}
		} 
		// yj mar 30, 2008; beacon triggers retransmission rather than send timeout
		else {
			if( pktTx_ && mhSend_.busy() ){
				// clear tx state for next tx after the beacon
				mhSend_.stop();
				setTxState(MAC_IDLE, true);
			}
		}
	}
	/*
	 * Address Filtering
	 */
	if(dst != (u_int32_t)index_ && dst != MAC_BROADCAST) {
		/*
		 *  We don't want to log this event, so we just free
		 *  the packet instead of calling the drop routine.
		 */
		discard(pktRx_, "---");
		goto done;
	}


	switch(type) {

	case MAC_Type_Management:
		discard(pktRx_, DROP_MAC_PACKET_ERROR);
		goto done;
	case MAC_Type_Control:
		switch(subtype) {
		case MAC_Subtype_ACK:
			recvACK(pktRx_);
			break;
		case MAC_Subtype_RRTS:
			recvBeacon(pktRx_);
			break;
		default:
			fprintf(stderr,"recvTimer1:Invalid MAC Control Subtype %x\n",
				subtype);
			exit(1);
		}
		break;
	case MAC_Type_Data:
		switch(subtype) {
		case MAC_Subtype_Data:
			recvDATA(pktRx_);
			break;
		default:
			fprintf(stderr, "recv_timer2:Invalid MAC Data Subtype %x\n",
				subtype);
			exit(1);
		}
		break;
	default:
		fprintf(stderr, "recv_timer3:Invalid MAC Type %x\n", subtype);
		exit(1);
	}
 done:
	pktRx_ = 0;
	rx_resume();
}

/*
 * txtime()	- pluck the precomputed tx time from the packet header
 */
double
RIMAC::txtime(Packet *p)
{
	struct hdr_cmn *ch = HDR_CMN(p);
	double t = ch->txtime();
	if (t < 0.0) {
		drop(p, "XXX");
 		exit(1);
	}
	return t;
}

 
/*
 * txtime()	- calculate tx time for packet of size "psz" bytes 
 *		  at rate "drt" bps
 */
double
RIMAC::txtime(double psz, double drt)
{
	double dsz = psz - phymib_.getPLCPhdrLen();
        int plcp_hdr = phymib_.getPLCPhdrLen() << 3;	
	int datalen = (int)dsz << 3;
	double t = (((double)plcp_hdr)/phymib_.getPLCPDataRate())
                                       + (((double)datalen)/drt);
	return(t);
}



void
RIMAC::recvDATA(Packet *p)
{
	struct hdr_mac802_11 *dh = HDR_MAC802_11(p);
	u_int32_t dst, src, size;
	struct hdr_cmn *ch = HDR_CMN(p);

	dst = ETHER_ADDR(dh->dh_ra);
	src = ETHER_ADDR(dh->dh_ta);
	size = ch->size();
	/*
	 * Adjust the MAC packet size - ie; strip
	 * off the mac header
	 */
	ch->size() -= phymib_.getHdrLen11();
	ch->num_forwards() += 1;

	// cancel pending idle timer after having successfully received a packet
	if( mhIdle_.busy() ){
		mhIdle_.stop();

		if( tx_state_ == MAC_ACK ){
			Packet::free(pktCTRL_); pktCTRL_ = 0;
		} else {
			Packet::free(pktRRTS_); pktRRTS_ = 0;
		}

		if(mhSend_.busy()) mhSend_.stop();

		setTxState(MAC_IDLE, true);

		// no other incoming packet
		unsetWaitingFlag();
	}

	if_incoming_data_ = true;

	if(dst != MAC_BROADCAST) {
		// reset retransmission counter for beacon
		beacon_retransmission_counter_ = 0;
		rst_cw();
		sendACK(src);
		mhDefer_.start(phymib_.getSIFS());
		defer_timer_action_ = YJ_ACTION_DEFER_TX_CTRL;
		tx_active_ = 1;
	}
	
	/* ============================================================
	   Make/update an entry in our sequence number cache.
	   ============================================================ */

	/* Changed by Debojyoti Dutta. This upper loop of if{}else was 
	   suggested by Joerg Diederich <dieder@ibr.cs.tu-bs.de>. 
	   Changed on 19th Oct'2000 */

        if(dst != MAC_BROADCAST) {
                if (src < (u_int32_t) cache_node_count_) {
                        Host *h = &cache_[src];

                        if(h->seqno && h->seqno == dh->dh_scontrol) {
                                discard(p, DROP_MAC_DUPLICATE);
                                return;
                        }
                        h->seqno = dh->dh_scontrol;
                } else {
			static int count = 0;
			if (++count <= 10) {
				printf ("MAC_802_11: accessing MAC cache_ array out of range (src %u, dst %u, size %d)!\n", src, dst, cache_node_count_);
				if (count == 10)
					printf ("[suppressing additional MAC cache_ warnings]\n");
			};
		};
	}

	/*
	 *  Pass the packet up to the link-layer.
	 *  XXX - we could schedule an event to account
	 *  for this processing delay.
	 */
	uptarget_->recv(p, (Handler*) 0);
}


void
RIMAC::recvACK(Packet *p)
{	
	if(tx_state_ != MAC_SEND
			) {
		discard(p, DROP_MAC_INVALID_STATE);
		return;
	}
	assert(pktTx_);
	mhSend_.stop();

	/*
	 * The successful reception of this ACK packet implies
	 * that our DATA transmission was successful.  Hence,
	 * we can reset the Short/Long Retry Count and the CW.
	 *
	 * need to check the size of the packet we sent that's being
	 * ACK'd, not the size of the ACK packet.
	 */
	slrc_ = 0;
	rst_cw();

	
	/*
	 * Backoff before sending again.
	 */
	assert(mhBackoff_.busy() == 0);


	if( mhDiscard_.busy() ){
		mhDiscard_.stop();
	}

	int receiver = ETHER_ADDR(HDR_MAC802_11(pktTx_)->dh_ra);
	recv_beacon(receiver);

	myneighbor_list_item *pn = getNeighbor(receiver);
	pn->if_collision_ = HDR_MAC802_11(p)->dh_fc.fc_retry;
	pn->cw_ = ((struct ack_frame*)p->access(hdr_mac::offset_))->cw;
	if( pn->if_collision_ ){
		// if contending sender mode
		pn->transmission_bound_time_ = TIME_NOW + phymib_.getSlotTime() * pn->cw_ + 0.00001;
	} else {
		// receiver will remain awake for a while (one slot for now, this node needs sifs to switch to transmission mode)
		pn->transmission_bound_time_ = TIME_NOW + 0.00001; // use some margin to prevent rounding problem of floating points
	}
	last_receiver_active_time_ = pn->transmission_bound_time_;
	
#ifdef YJ_BEACON_ON_DEMAND
	if(pktRRTS_ == NULL){
		if( index_ == ((struct rrts_frame*)p->access(hdr_mac::offset_))->intended_receiver ) {
			fprintf(stdout, "rimac %d generate a BEACON on demand for neighbor %d at %f\n", index_, receiver, TIME_NOW);
			generateBeacon();
			// set the on_demand flag so that max backoff window is used to avoid interfering with
			// other flows to that node
			((struct rrts_frame*)pktRRTS_->access(hdr_mac::offset_))->on_demand = 1;
		}
	}
#endif

	// no true for setTxState to involve checkPendingBEACON
	setRxState(MAC_IDLE, true);
	setTxState(MAC_IDLE);

	Packet::free(pktTx_); 
	pktTx_ = 0;
	if(callback_) {
		Handler *h = callback_;
		callback_ = 0;
		h->handle((Event*) 0);
	} 

	mac_log(p);
}


void RIMAC::start_hello(){
	hellotimer_ = new MACHelloTimer(this);
	// start at a random time between 1. and 9.
	hellotimer_->start(index_, Random::uniform(1.,9.));
}





/*output to trace file regardless of the verbose level */
void RIMAC::direct_trace(char* fmt, ...)
{
  va_list ap;
  
  if (!logtarget_) return;

  va_start(ap, fmt);
  vsprintf(logtarget_->pt_->buffer(), fmt, ap);
  logtarget_->pt_->dump();
  va_end(ap);
}

bool RIMAC::is_transmission_attempt_allowed(){
	// states other than tx_state
	bool otherviolation = rx_state_ != MAC_IDLE;
	if( otherviolation ){
violated:
		return false;
	}

	if( tx_state_ != MAC_IDLE ){
		goto violated;
	}
	return true;
}

// power state control
bool RIMAC::attemptToSleep(bool forced) 
{
	if( forced == false ){
		setSleepFlag();

		// RIMAC does not go to sleep if there is a pending data packet
		// check if any timer is active
		if(pktTx_ || mhSend_.busy() || mhRecv_.busy() || mhDefer_.busy() ||
				mhBackoff_.busy() || mhIF_.busy() || mhIdle_.busy() ||
				rx_state_ != MAC_IDLE || tx_state_ != MAC_IDLE ){
			printf("rimac %d cannot sleep because pktTx_ %d mhSend_ %d mhRecv_ %d mhDefer_ %d mhBackoff_ %d mhIF_ %d mhIdle_ %d rx_state_ != MAC_IDLE %d tx_state_ != MAC_IDLE %d\n", index_, pktTx_ != 0, mhSend_.busy(),mhRecv_.busy(),mhDefer_.busy(),mhBackoff_.busy(),mhIF_.busy(),mhIdle_.busy(), rx_state_ != MAC_IDLE, tx_state_ != MAC_IDLE );
			return false;
		}
		// though only a sender need this dwell time, we let all nodes have this dwell
		// time to simplify code (in a realy system, this dwell time is also necessary
		// to account for processing delays at the sender side
		double dwelltime = last_receiver_active_time_ - TIME_NOW;
		if( ! mhQedPktWaiter_.busy() && dwelltime > 0){
			mhQedPktWaiter_.start(dwelltime);
			tx_state_ = rx_state_ = MAC_IDLE;
			return true;
		} else {
			// go to sleep immediately
			unsetSleepFlag();
		}
	}

	unsetSleepFlag();
	if( mhQedPktWaiter_.busy() ){
		// overheard irrelevant flows
		mhQedPktWaiter_.stop();
	}
	if( sinrMonitor.busy() ){
		sinrMonitor.stop();
	}
	if( mhRecv_.busy() ){
		mhRecv_.stop();
		pktRx_ = NULL;
	}
	if( mhIdle_.busy() ){
		mhIdle_.stop();
	}

	// schedule next wakeup
	// only start wakeup timer if it's not running;
	// if wakeup timer is not cancelled when a packet is rcvd from upper layer
	// wakeup timer may still be running, in that case, do not schedule a new wakeup time
	if( false == hellotimer_->busy() )
		hellotimer_->start(index_, duty_cycle_length_ * Random::uniform(0.5,1.5));

	// dutycycle counter
	if( tx_state_ != MAC_SLEEP ){

		if(duty_cycle_stat_started_){
			God::instance()->total_active_time_ += TIME_NOW - last_radio_state_update_time_;
			God::instance()->duty_cycle_active_[index_] += TIME_NOW - last_radio_state_update_time_;
		} else if( TIME_NOW >= START_MEASUREMENT_THRESHOLD ){
			duty_cycle_stat_started_ = true;
			God::instance()->total_active_time_ += TIME_NOW - START_MEASUREMENT_THRESHOLD;
			God::instance()->duty_cycle_active_[index_] += TIME_NOW - START_MEASUREMENT_THRESHOLD;
		}
		last_radio_state_update_time_ = TIME_NOW;
	}

	if( mhDiscard_.busy() ){
		mhDiscard_.stop();
	}

	// TODO cancel or wait all pending transactions
	// go to sleep, turn off radio
	tx_state_ = rx_state_ = MAC_SLEEP;

	((WirelessPhy *)netif_)->node_sleep();
	return true;
}

bool RIMAC::wakeup()
{
	// dutycycle counter
	if( tx_state_ == MAC_SLEEP ){
		if(duty_cycle_stat_started_){
			God::instance()->total_sleep_time_ += TIME_NOW - last_radio_state_update_time_;
			God::instance()->duty_cycle_sleep_[index_] += TIME_NOW - last_radio_state_update_time_;
		} else if( TIME_NOW >= START_MEASUREMENT_THRESHOLD ){
			duty_cycle_stat_started_ = true;
			God::instance()->total_sleep_time_ += TIME_NOW - START_MEASUREMENT_THRESHOLD;
			God::instance()->duty_cycle_sleep_[index_] += TIME_NOW - START_MEASUREMENT_THRESHOLD;
		}
		last_radio_state_update_time_ = TIME_NOW;
	}
	//wakeup from sleep. turn on radio
	//this is the only place tx_state_ and rx_state_ can be changed from sleep to other state
	tx_state_ = rx_state_ = MAC_IDLE;

	((WirelessPhy *) netif_)->node_wakeup();

	return true;
}

// this function is called at scheduled wakeup time + wakeup delay (0.5~2ms) + 
// CCA delay (128us); no seperate timer used to simulate these delays as they
// can be easily simultated when calculate next wakeup time, and setting wakeup
// penalty properly
void RIMAC::handleWakeup(){

	// schedule next wakeup regardless of current ratio state
	hellotimer_->start(index_, duty_cycle_length_* Random::uniform(0.5,1.5));
	// allocate a beacon
	generateBeacon();

	if( tx_state_ != MAC_SLEEP ){
		// sanity check
		if( pktTx_ == 0 && pktRRTS_ == 0 && is_waiting_for_pkt_ == false && tx_state_ != MAC_PREAMBLE && rx_state_ != MAC_RECV ){
			fprintf(stderr, "### Warning: rimac %d is not sleeping when handleWakeup called at %f; trying to put the node into sleep\n", index_, TIME_NOW);
			// put this node to sleep
			if( false == attemptToSleep() ){
				fprintf(stderr, "### Warning: rimac %d failed to put the node into sleep\n", index_);
				exit(1);
			} 
		}
		checkPendingBEACON();
		return;
	}

	// set sender_cw_, cw for beacon and etc..
	restoreRIMACStateAfterWakeup();

	// wake up radio, delay and energy dissipation has been taken care of by
	// scheduling wakeuptime and wake penalty
	wakeup();

	// check if there is an preamble
	if(sinrMonitor.channelClear(TIME_NOW, CSThresh_)){
		// start transmitting beacon after CCA check delay; 128us based on SCP paper
		mhDefer_.start(128e-6);
		defer_timer_action_ = YJ_ACTION_DEFER_TX_RRTS;
		tx_active_ = 1;
	} else {
		setRxState(MAC_COLL); 
		// start timer of sinrMonitor
		if( !sinrMonitor.busy() ){
			// set the flag to prevent the node from sleeping
			unsetSleepFlag();
			sinrMonitor.start(sinrMonitor.head_->finish_time_ - NOW);
		} else {
			// sanity check
			fprintf(stderr, "### Error: sinrMonitor still busy when wake up\n");
			exit(1);
		}
	}
}
void RIMAC::generateBeacon(){
	// want to transmit a beacon, stop dwell timer
	if( mhQedPktWaiter_.busy() ){
		mhQedPktWaiter_.stop();
	}
	if( pktRRTS_ ){
		// generate a new BEACON, as backoff winder may have been updated
		Packet::free(pktRRTS_); pktRRTS_ = 0;
	}
	sendBeacon(MAC_BROADCAST);
}

void
RIMAC::sendBeacon(int dst)
{
	Packet *p = Packet::alloc();
	hdr_cmn* ch = HDR_CMN(p);
	struct rrts_frame *rf = (struct rrts_frame*)p->access(hdr_mac::offset_);
	
	assert(pktTx_);
	assert(pktRTS_ == 0);

	ch->uid() = 0;
	ch->ptype() = PT_MAC;
	ch->size() = base_beacon_frame_length_;
	ch->iface() = -2;
	ch->error() = 0;

	bzero(rf, MAC_HDR_LEN);

	rf->rf_fc.fc_protocol_version = MAC_ProtocolVersion;
 	rf->rf_fc.fc_type	= MAC_Type_Control;
 	rf->rf_fc.fc_subtype	= MAC_Subtype_RRTS;
 	rf->rf_fc.fc_to_ds	= 0;
 	rf->rf_fc.fc_from_ds	= 0;
 	rf->rf_fc.fc_more_frag	= 0;
 	rf->rf_fc.fc_retry	= 0;
 	rf->rf_fc.fc_pwr_mgt	= 0;
 	rf->rf_fc.fc_more_data	= 0;
 	rf->rf_fc.fc_wep	= 0;
 	rf->rf_fc.fc_order	= 0;

	STORE4BYTE(&dst, (rf->rf_ra));
	STORE4BYTE(&index_, (rf->rf_ta));
	rf->rf_fc.fc_retry = if_contending_senders_;
	
	/* store rts tx time */
 	ch->txtime() = txtime(ch->size(), basicRate_);
	
	// TODO reserve time for CTRL frame for potential sender? NAV is not used for now
	rf->rf_duration = 0;
	pktRRTS_ = p;
}

int RIMAC::check_pktRRTS(bool ignoremedium)
{
	struct hdr_mac802_11 *mh;
	double timeout = -1.;

	assert(mhBackoff_.busy() == 0);

	if(pktRRTS_ == 0)
 		return -1;
	mh = HDR_MAC802_11(pktRRTS_);

	hdr_cmn* ch = HDR_CMN(pktRRTS_);
	int beacon_size = base_beacon_frame_length_;
	// if contention window is not 0, include one more byte to the beacon
	if( sender_cw_ ){
		ch->size() += 1;
	}
#ifdef YJ_META_BEACON
	// include one more byte to the beacon in order to hold dst info
	if(pktTx_){
		ch->size() += 2;
	}
#endif

	

 	switch(mh->dh_fc.fc_subtype) {
	case MAC_Subtype_RRTS:
		if(ignoremedium == false && (rx_state_ != MAC_IDLE || tx_state_ != MAC_IDLE) ) {
#ifdef YJ_BEACON_ON_DEMAND
			// if this is a beacon_on_demand, use max backoff window to avoid interfering with
			// other flows
			if( ((struct rrts_frame*)pktRRTS_->access(hdr_mac::offset_))->on_demand )
				mhBackoff_.start(0, true, phymib_.getCWMax() * phymib_.getSlotTime() );
			else
#endif
				mhBackoff_.start(cw_, true);

			back_off_timer_action_ = YJ_ACTION_BACKOFF_TX_RRTS;
			return 0;
		}
		setTxState(MAC_RIMACBEACON);
		timeout = txtime(beacon_size, basicRate_)
			+ DSSS_MaxPropagationDelay                      
			+ phymib_.getSIFS()
			+ DSSS_MaxPropagationDelay;
		break;
	default:
		fprintf(stderr, "check_pktRRTS:Invalid MAC Control subtype\n");
		exit(1);
	}

	// let potential senders to use this backoff window
	((struct rrts_frame*) mh)->cw = sender_cw_;


#ifdef YJ_META_BEACON
	// assign intended receiver to the BEACON
	if( pktTx_ ){
		assignIntendedReceiverToBEACON(pktRRTS_, pktTx_);
	}
#endif

	if (mh->dh_fc.fc_retry){
		timeout += sender_cw_ * phymib_.getSlotTime(); 
	} 
	setWaitingFlag();
	transmit(pktRRTS_, timeout);
	return 0;
}

void RIMAC::recvBeacon(Packet *p)
{
	struct rrts_frame *rf = (struct rrts_frame*)p->access(hdr_mac::offset_);
	int sender = ETHER_ADDR(rf->rf_ta);
	bool if_collision = HDR_MAC802_11(p)->dh_fc.fc_retry; 
	
	recv_beacon(sender);
	
	bool match = pktTx_ && sender == ETHER_ADDR(HDR_MAC802_11(pktTx_)->dh_ra);


	if( match ){
		if( mhDiscard_.busy() ){
			mhDiscard_.stop();
			mhDiscard_.start(2 * duty_cycle_length_);
		}
		myneighbor_list_item *pn = getNeighbor(ETHER_ADDR(HDR_MAC802_11(pktTx_)->dh_ra));
		pn->if_collision_ = if_collision;
		pn->cw_ = rf->cw;
		if( if_collision == false ){
			// first RRTS after wakeup
			// receiver will remain awake for a while (one slot for now, this node needs sifs to switch to transmission mode)
			pn->transmission_bound_time_ = TIME_NOW + 0.00001; // use some margin to prevent rounding problem of floating points


			// stop conflicting timers
			if( mhBackoff_.busy() ) {
				mhBackoff_.stop();
				back_off_timer_action_ = YJ_ACTION_NONE;
			}
			if( mhDefer_.busy() ) {
				mhDefer_.stop();
				defer_timer_action_ = YJ_ACTION_NONE;
			}

			mhDefer_.start(phymib_.getSIFS());
			defer_timer_action_ = YJ_ACTION_DEFER_TX_DATA;
			tx_active_ = 1;

		} else {
			// collision observed at the receiver, use backoff before DATA transmission
			// receiver will remain awake for a while (one slot for now, this node needs sifs to switch to transmission mode)
			pn->transmission_bound_time_ = TIME_NOW + phymib_.getSlotTime()*pn->cw_ + 0.00001;

			// stop conflicting timers
			if( mhBackoff_.busy() ) {
				mhBackoff_.stop();
				back_off_timer_action_ = YJ_ACTION_NONE;
			}
			if( mhDefer_.busy() ) {
				mhDefer_.stop();
				defer_timer_action_ = YJ_ACTION_NONE;
			}
			// start backoff
			mhBackoff_.start(pn->cw_, true, phymib_.getDIFS());
			back_off_timer_action_ = YJ_ACTION_BACKOFF_TX_DATA;
		}

	last_receiver_active_time_ = pn->transmission_bound_time_;

#ifdef YJ_BEACON_ON_DEMAND
		if(pktRRTS_ == NULL){
			if( index_ == rf->intended_receiver ) {
				generateBeacon();
				// set the on_demand flag so that max backoff window is used to avoid interfering with
				// other flows to that node
				((struct rrts_frame*)pktRRTS_->access(hdr_mac::offset_))->on_demand = 1;
			}
		}
#endif

	}

	mac_log(p);
	setRxState(MAC_IDLE);
	return;
}
void RIMAC::setSleepFlag(){
	if_want_sleep_ = true;
}
void RIMAC::unsetSleepFlag(){
	if_want_sleep_ = false;
}
void RIMAC::setWaitingFlag(){
	is_waiting_for_pkt_ = true;
}
void RIMAC::unsetWaitingFlag(){
	is_waiting_for_pkt_ = false;
}

void RIMAC::idleGuardTimerHandler(){
	// collision happend, generate another BEACON

	if_incoming_data_ = true;

	// if mhSend_ expires earlier, clear the residual state
	if( tx_state_ == MAC_ACK ){
		Packet::free(pktCTRL_); pktCTRL_ = 0;
	} else {
		Packet::free(pktRRTS_); pktRRTS_ = 0;
	}
	// reset tx state to allow further transmission
	setTxState(MAC_IDLE, true);
	if( mhSend_.busy() ){
		mhSend_.stop();
	}

	if( if_contending_senders_ == false ){
		if_contending_senders_ = true;
	} else 
	{
		sender_cw_ = (sender_cw_ << 1) + 1;
		if( sender_cw_ > (int) phymib_.getCWMax()){
			fprintf(stdout, "rimac %d beacon max backoff window has been reached, give up beacon for this cycle at %f\n", index_, TIME_NOW);
			sender_cw_ = phymib_.getCWMax();
		}
	}

	// retransmission limit check
	beacon_retransmission_counter_++;
	if( beacon_retransmission_counter_ >= 5 ){ // for bw to at least to reach 255
		fprintf(stdout, "rimac %d beacon retransmission limit has been reached, give up beacon for this cycle at %f\n", index_, TIME_NOW);
		if( ifNoOngoingTransaction() && NULL == pktTx_ )
			attemptToSleep();

		return;
	}

	inc_cw();
	generateBeacon();
	checkPendingBEACON();
}

void RIMAC::restoreRIMACStateAfterWakeup(){
	// assume no contending senders at the beginning (the most common case)
	if_contending_senders_ = false;
	sender_cw_ = phymib_.getCWMin();
	// reset retransmission counter for beacon
	beacon_retransmission_counter_ = 0;
	// reset contention window for BEACON
	rst_cw();
	if_incoming_data_ = false;
}

#ifdef YJ_META_BEACON
void RIMAC::assignIntendedReceiverToBEACON(Packet *beacon, Packet *data){
	struct rrts_frame *rf = (struct rrts_frame*)beacon->access(hdr_mac::offset_);
	rf->intended_receiver = ETHER_ADDR(HDR_MAC802_11(data)->dh_ra);
}
#endif

void RIMAC::QedPktWaitTimerHandler(){
	// sanity check
	if( tx_state_ == MAC_SLEEP ){
		fprintf(stderr, "ERROR!!! bmac %d QedPktWaitTimerHandler: node already in sleep at %f\n", index_, TIME_NOW);
		exit(1);
	}
	attemptToSleep(true);
}

void DiscardTimer::start(double time)
{
	Scheduler &s = Scheduler::instance();
	assert(busy_ == 0);
	busy_ = 1;
	paused_ = 0;
	stime = s.clock();
	rtime = time;
	assert(rtime >= 0.0);

	s.schedule(this, &intr, rtime);
}


void DiscardTimer::handle(Event *)
{       
	busy_ = 0;
	paused_ = 0;
	stime = 0.0;
	rtime = 0.0;

	mac->discardTimerHandler();
}

void RIMAC::discardTimerHandler(){
	// sanity
	if( pktTx_ == NULL ){
		fprintf(stderr, "rimac %d discardTimerHandler: no pending data at %f \n", index_, TIME_NOW);
		exit(1);
	}

	if( ifNoOngoingTransaction() ){
		RetransmitDATA();
	} else {
		// check later
		mhDiscard_.start(0.5 * duty_cycle_length_);
	}
}
