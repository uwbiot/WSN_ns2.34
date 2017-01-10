/* -*-	Mode:C++; c-basic-offset:8; tab-width:8; indent-tabs-mode:t -*- 
 *
 * Copyright (c) 1996 Regents of the University of California.
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
 *	This product includes software developed by the Computer Systems
 *	Engineering Group at Lawrence Berkeley Laboratory and the Daedalus
 *	research group at UC Berkeley.
 * 4. Neither the name of the University nor of the Laboratory may be used
 *    to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * $Header: /cvsroot/nsnam/ns-2/mac/wireless-phy.cc,v 1.28 2007/09/04 04:32:18 tom_henderson Exp $
 *
 * Ported from CMU/Monarch's code, nov'98 -Padma Haldar.
 * wireless-phy.cc
 */

#include <math.h>

#include <packet.h>
#include <mobilenode.h>
#include <phy.h>
#include <propagation.h>
#include <modulation.h>
#include <omni-antenna.h>
#include <wireless-chargingphy.h>
#include <packet.h>
#include <ip.h>
#include <agent.h>
#include <trace.h>
#include <sys/param.h>  /* for MIN/MAX */
#include <god.h>
//#include "diffusion/diff_header.h"

/* ======================================================================
   WirelessChargingPhy Interface
   ====================================================================== */
static class WirelessChargingPhyClass: public TclClass {
public:
        WirelessChargingPhyClass() : TclClass("Phy/WirelessCPhy") {}
        TclObject* create(int, const char*const*) {
                return (new WirelessChargingPhy);
        }
}class_WirelessChargingPhy;


WirelessChargingPhy::WirelessChargingPhy() : WirelessPhy(), radioStatus_(RADIOON), last_radioOn_time_(NOW)
{
	//Pt_consume_ = 0.0588;
	//Pr_consume_ = 0.0616;
	//Pt_consume_ = 0.588/2;
	//Pr_consume_ = 0.616/2;
	//P_idle_ = 0.000042;
}

int
WirelessChargingPhy::getRadioStatus(){
	return radioStatus_;
}

void
WirelessChargingPhy::turnOnRadio(){
//if (!God::instance()->isSink(index_)) {
	double radioOnTime = NOW - last_radioOn_time_;
	last_radioOn_time_ = NOW;
	if (radioStatus_ == RADIOON) {
		if (radioOnTime > 0) {
			//em()->DecrTxEnergy(radioOnTime, Pt_consume_);
			God::instance()->addDutyOnTime(index_, radioOnTime);
		}
	}
	radioStatus_ = RADIOON;
	em()->DecrTxEnergy(radioOnTime, P_idle_);
//}
}

void
WirelessChargingPhy::turnOffRadio(){
//if (!God::instance()->isSink(index_)) {
	double radioOnTime = NOW - last_radioOn_time_;
	if (radioStatus_ == RADIOON) {
		last_radioOn_time_ = NOW;
		if (radioOnTime > 0) {
			//em()->DecrTxEnergy(radioOnTime, Pt_consume_);
			God::instance()->addDutyOnTime(index_, radioOnTime);
		}
		radioStatus_ = RADIOOFF;
	}
	em()->DecrTxEnergy(radioOnTime, P_idle_);
//}
}

 
void 
WirelessChargingPhy::sendDown(Packet *p)
{
	/*
	 * Sanity Check
	 */
	assert(initialized());
	
	if (em()) {
			//node is off here...
			if (radioStatus_ != RADIOON ) {
				printf("node %d radio is already off, eng %f!\n", index_, em()->energy());
				Packet::free(p);
				return;
			}


	}
	/*
	 * Decrease node's energy
	 */
	if(em()) {
		if (em()->energy() > 0) {

		    double txtime = hdr_cmn::access(p)->txtime();
		    double start_time = MAX(channel_idle_time_, NOW);
		    double end_time = MAX(channel_idle_time_, NOW+txtime);
		    double actual_txtime = end_time-start_time;

		    if (start_time > update_energy_time_) {
			    //em()->DecrIdleEnergy(start_time - 
				//		 update_energy_time_, P_idle_);
			    update_energy_time_ = start_time;
		    }

		    /* It turns out that MAC sends packet even though, it's
		       receiving some packets.
		    
		    if (txtime-actual_txtime > 0.000001) {
			    fprintf(stderr,"Something may be wrong at MAC\n");
			    fprintf(stderr,"act_tx = %lf, tx = %lf\n", actual_txtime, txtime);
		    }
		    */

		   // Sanity check
		   double temp = MAX(NOW,last_send_time_);

		   /*
		   if (NOW < last_send_time_) {
			   fprintf(stderr,"Argggg !! Overlapping transmission. NOW %lf last %lf temp %lf\n", NOW, last_send_time_, temp);
		   }
		   */
		   
		   double begin_adjust_time = MIN(channel_idle_time_, temp);
		   double finish_adjust_time = MIN(channel_idle_time_, NOW+txtime);
		   double gap_adjust_time = finish_adjust_time - begin_adjust_time;
		   if (gap_adjust_time < 0.0) {
			   fprintf(stderr,"What the heck ! negative gap time.\n");
		   }

		   if ((gap_adjust_time > 0.0) && (status_ == RECV)) {
			   //em()->DecrTxEnergy(gap_adjust_time,
				//	      Pt_consume_-Pr_consume_);
		   }

			//em()->DecrTxEnergy(actual_txtime+0.1,Pt_consume_);
		   status_ = IDLE;

		   last_send_time_ = NOW+txtime;
		   channel_idle_time_ = end_time;
		   update_energy_time_ = end_time;


		   //if (em()->energy() <= 0) {
		//	   em()->setenergy(0);
		//	   ((MobileNode*)node())->log_energy(0);
		 //  }

		} else {

			// log node energy
			//if (em()->energy() > 0) {
			//	((MobileNode *)node_)->log_energy(1);
			//} 
//
			Packet::free(p);
			return;
		}
	}

	/*
	 *  Stamp the packet with the interface arguments
	 */
	p->txinfo_.stamp((MobileNode*)node(), ant_->copy(), Pt_, lambda_);
	
	// Send the packet
	channel_->recv(p, this);
}

int 
WirelessChargingPhy::sendUp(Packet *p)
{
	/*
	 * Sanity Check
	 */
	assert(initialized());

	PacketStamp s;
	double Pr;
	int pkt_recvd = 0;

	Pr = p->txinfo_.getTxPr();
	
	// if the node is in sleeping mode, drop the packet simply
	if (em()) {
			if (radioStatus_ != RADIOON){
				pkt_recvd = 0;
				goto DONE;
			}
			
	}
	// if the energy goes to ZERO, drop the packet simply
	if (em()) {
		if (em()->energy() <= 0) {
			pkt_recvd = 0;
			goto DONE;
		}
	}

	if(propagation_) {
		s.stamp((MobileNode*)node(), ant_, 0, lambda_);
		Pr = propagation_->Pr(&p->txinfo_, &s, this);
		if (Pr < CSThresh_) {
			pkt_recvd = 0;
			goto DONE;
		}
		if (Pr < RXThresh_) {
			/*
			 * We can detect, but not successfully receive
			 * this packet.
			 */
			hdr_cmn *hdr = HDR_CMN(p);
			hdr->error() = 1;
#if DEBUG > 3
			printf("SM %f.9 _%d_ drop pkt from %d low POWER %e/%e\n",
			       Scheduler::instance().clock(), node()->index(),
			       p->txinfo_.getNode()->index(),
			       Pr,RXThresh);
#endif
		}
	}
	if(modulation_) {
		hdr_cmn *hdr = HDR_CMN(p);
		hdr->error() = modulation_->BitError(Pr);
	}
	
	/*
	 * The MAC layer must be notified of the packet reception
	 * now - ie; when the first bit has been detected - so that
	 * it can properly do Collision Avoidance / Detection.
	 */
	pkt_recvd = 1;

DONE:
	p->txinfo_.getAntenna()->release();

	/* WILD HACK: The following two variables are a wild hack.
	   They will go away in the next release...
	   They're used by the mac-802_11 object to determine
	   capture.  This will be moved into the net-if family of 
	   objects in the future. */
	p->txinfo_.RxPr = Pr;
	p->txinfo_.CPThresh = CPThresh_;

	/*
	 * Decrease energy if packet successfully received
	 */
	if(pkt_recvd && em()) {

		double rcvtime = hdr_cmn::access(p)->txtime();
		// no way to reach here if the energy level < 0
		
		double start_time = MAX(channel_idle_time_, NOW);
		double end_time = MAX(channel_idle_time_, NOW+rcvtime);
		double actual_rcvtime = end_time-start_time;

		if (start_time > update_energy_time_) {
			//em()->DecrIdleEnergy(start_time-update_energy_time_,
			//		     P_idle_);
			update_energy_time_ = start_time;
		}
/*
  if (end_time > channel_idle_time_) {
  status_ = RECV;
  }
*/
		channel_idle_time_ = end_time;
		update_energy_time_ = end_time;

		status_ = IDLE;

		/*
		  hdr_diff *dfh = HDR_DIFF(p);
		  printf("Node %d receives (%d, %d, %d) energy %lf.\n",
		  node()->address(), dfh->sender_id.addr_, 
		  dfh->sender_id.port_, dfh->pk_num, node()->energy());
		*/
#if 0
		// log node energy
		if (em()->energy() > 0) {
		((MobileNode *)node_)->log_energy(1);
        	} 

		if (em()->energy() <= 0) {  
			// saying node died
			em()->setenergy(0);
			((MobileNode*)node())->log_energy(0);
		}
#endif
	}
	
	return pkt_recvd;
}
