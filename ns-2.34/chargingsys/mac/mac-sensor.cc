#include "mac-sensor.h"
#include "random.h"
#include "cmu-trace.h"
#include "time.h"


static class Mac_SensorClass : public TclClass {
public:
	Mac_SensorClass() : TclClass("Mac/Sensor") {}
	TclObject* create(int, const char*const*) {
		return new Mac_Sensor();
	}
} class_Mac_Sensor;


Mac_Sensor::Mac_Sensor() : Mac(), config(), 
            waitTimer(this), sendTimer(this), recvTimer(this), ackTimer(this),
            dutyTimer(this), onRadioTimer(this), preembleTimer(this)
{
	rx_state_ = tx_state_ = MAC_IDLE;
	tx_active_ = 0;
	cw_ = config.cw_min;
    acked_ = false;
    resendCount_ = 0;
    pktTxBackup_ = 0;
    resending_ = false;
    emptyEng_ = false;
	bind_bool("ACK", &ACK_);
    bind("DutyCycle", &DutyCycle_);
    bind("OnRadioTime", &OnRadioTime_);
    // the sink node will not run duty cycling
    assert(DutyCycle_>=0);
    if (!God::instance()->isSink(index_)) {
        dutyTimer.setDutyCycle(DutyCycle_);
        if (dutyTimer.interval()>0) dutyTimer.restart(dutyTimer.interval());
        onRadioTimer.setOnInterval(OnRadioTime_);
        onRadioTimer.setNodeDuty(DutyCycle_);
        printf("duty %f, ontime %f\n", DutyCycle_,OnRadioTime_);
    }
}

void Mac_Sensor::recv(Packet *p, Handler *h) {

	struct hdr_cmn *hdr = HDR_CMN(p);
    if (emptyEng_) {
        if (hdr->direction() == hdr_cmn::UP) Packet::free(p);
        else h->handle(p);
        God::instance()->logDutyOntime();
        time_t rawtime;
        time (&rawtime );
        God::instance()->ctrace()->log("node %d endtime: ns2 %f system %s \n", index_, NOW, ctime(&rawtime));
        exit(0);
        return;
    }
    if (newnetif()->em()->energy()<1) {
		tx_active_ = 0;
		tx_state_ = MAC_IDLE;
        rx_state_ = MAC_IDLE;
		reset_cw();
        if (hdr->direction() == hdr_cmn::UP) Packet::free(p);
        else h->handle(p);
	    pktTx_ = 0;
		txHandler_ = 0;
        onRadioTimer.forcestop();
        waitTimer.forcestop();
        sendTimer.forcestop();
        recvTimer.forcestop();
        ackTimer.forcestop();
        dutyTimer.forcestop();
        onRadioTimer.forcestop();
        preembleTimer.forcestop();
        emptyEng_ = true;
        printf("node %d no energy, directly return\n", index_);
        return;
    }


	if (hdr->direction() == hdr_cmn::DOWN) {
        if (resending_!=true) {
            send(p,h,MF_DATA);
        }
        else {
            printf("super error!!!!!!!!!node %d: %d %d %d %d\n",
                    index_, resending_, tx_state_, tx_active_, rx_state_);
        }
		return;
	}

    // now, we are processing received packets
	// If we are transmitting, then set the error bit in the packet so that it will be thrown away
	if(tx_active_ && hdr->error() == 0) {
		hdr->error() = 1;
    }
	// check to see if we're already receiving a different packet
	if (rx_state_ == MAC_IDLE)
	{
		rx_state_ = MAC_RECV;
		pktRx_ = p;
		// schedule reception of the packet
		recvTimer.start(txtime(p));
	}
	else
	{
		//We are receiving a different packet, so decide whether this packet's power is high enough to cause collision
		if (pktRx_->txinfo_.RxPr / p->txinfo_.RxPr >= p->txinfo_.CPThresh)
		{
			//power too low, ignore the packet
			Packet::free(p);
		}
		else
		{
			//power is high enough to cause collision
			rx_state_ = MAC_COLL;

			/*
			 * look at the length of each packet and update the timer if necessary
			 */
			if (txtime(p) > recvTimer.expire())
			{
				recvTimer.stop();
				Packet::free(pktRx_);
				pktRx_ = p;
				recvTimer.start(txtime(pktRx_));
			}
			else {
				Packet::free(p);
            }
		}
	}
}

double Mac_Sensor::txtime(Packet *p)
 {
	 struct hdr_cmn *ch = HDR_CMN(p);
	 double t = ch->txtime();
	 if (t < 0.0)
	 	t = 0.0;
	 return t;
 }

/* Send Data or ACK Packet*/
void Mac_Sensor::send(Packet *p, Handler *h, u_int16_t type)
{
	hdr_cmn* ch = HDR_CMN(p);
	hdr_type((char*) HDR_MAC(p), type); // set mac frame type

	//Send Data
	if(type == MF_DATA) {
        assert(pktTx_==0);
		pktTx_ = p;
		txHandler_ = h;

		ch->txtime() = (8.0 * (ch->size() + config.header_len)) / config.data_rate;
		if (rx_state_ == MAC_IDLE && tx_state_ == MAC_IDLE) {
			// idle, try to send after CCA
			waitTimer.restart(config.CCA);
		} else {
			// busy, begin backoff now
			double backoff = (Random::random() % cw_) * config.backoff_slot;
			waitTimer.restart(backoff + config.CCA);
			inc_cw();
		}
	}
	//Send ACK immediately
	else if(type == MF_ACK)
	{
		ch->size() = config.ack_len;
		ch->txtime() = (8.0 * ch->size()) / config.data_rate;
        //if (tx_state_  != MAC_IDLE) {
        //    printf("node %d err...mac is not idle when sending ack\n", index_);
        //    printf("node %d %d %d %d\n",index_, rx_state_, tx_state_, tx_active_);
        //}
		tx_state_ = MAC_ACK;
		tx_active_ = 1;
		sendTimer.restart(ch->txtime());
		downtarget_->recv(p, h);
	}
	else {
		printf("%s\tUnknown MAC frame type!\n",__FUNCTION__);
        // then, should we free this packet?
    }
}

void Mac_Sensor::recvHandler()
{
	hdr_cmn *ch = HDR_CMN(pktRx_);
	Packet* p = pktRx_;
	MacState state = rx_state_;
	int dst = hdr_dst((char*)HDR_MAC(p));

	//Reset channel
	pktRx_ = 0;
	rx_state_ = MAC_IDLE;

	if (tx_active_) {
		// We are currently sending, so discard packet
		Packet::free(p);
	} else if (state == MAC_COLL) {
		// Collision, so discard the packet
		Packet::free(p);
	} else if (dst != index_ && (u_int32_t)dst != MAC_BROADCAST) {
		// address filtering, then how to do snooping
		Packet::free(p);
	} else if (ch->error()) {
		// packet has errors, so discard it
		Packet::free(p);
	} else {
		u_int16_t type = hdr_type((char*)HDR_MAC(p));
		if(type == MF_DATA) {
			uptarget_->recv(p, (Handler*) 0);
			//Send back ACK immediately if it is not broadcast
			if(ACK() && (u_int32_t)dst != MAC_BROADCAST)
			{
                onRadioTimer.increaseInterval();
				//Get sender's MAC address
				int src = hdr_src((char*)HDR_MAC(p));

				//Compose ACK packet
				Packet *pktACK = Packet::alloc();
				char *mh = (char*) HDR_MAC(pktACK);
				hdr_dst(mh, src);
				hdr_src(mh, index_);
                hdr_cmn::access(pktACK)->direction() = hdr_cmn::DOWN;
				send(pktACK,(Handler*) 0,MF_ACK);
			}
            // receive done, start to turn off radio
            if (dutyTimer.interval() > 0 && !God::instance()->isSink(index_)) {
                onRadioTimer.restart(onRadioTimer.interval());
            }
		}
		else if(type == MF_ACK) {
			// receive ack from receiver
            acked_ = true;
            //preembleHandler(false);
			Packet::free(p);
		}
		else {
			printf("%s\tUnknown MAC frame type!\n",__FUNCTION__);
            // should free it?
        }
	}
}

void Mac_Sensor::waitHandler()
{
	if(rx_state_ == MAC_IDLE && tx_state_ == MAC_IDLE) {
		//Still idle, so start to transmit
		tx_active_ = 1;
        tx_state_ = MAC_SEND;
        if (ACK() && (u_int32_t)hdr_dst((char*)HDR_MAC(pktTx_)) != MAC_BROADCAST) {
            acked_ = false;
            pktTxBackup_ = pktTx_->copy();// backup for resending
        }
        else {
            acked_ = true;
        }
		sendTimer.restart(HDR_CMN(pktTx_)->txtime());
        newnetif()->turnOnRadio();// turn on radio before sending
		downtarget_->recv(pktTx_, txHandler_);
	}
	else
	{
		//channel busy, increase contention window
		double backoff = (Random::random() % cw_) * config.backoff_slot;
		waitTimer.restart(backoff + config.CCA);
		inc_cw();
	}
}
// this means send is finished, either signal up layer or check ack
void Mac_Sensor::sendHandler() {
    //if (tx_active_!=1) {printf("node %d error...... tx_active_ is not 1 when senddone\n", index_);}
    if (tx_state_ == MAC_SEND && acked_ != true) {
        // need to wait for ack, though the packet has been sent out
        tx_active_ = 0;
        tx_state_ = MAC_IDLE;
		reset_cw();
		ackTimer.restart((8.0 * (config.ack_len+20)) / config.data_rate);
    }
	else if(tx_state_ == MAC_SEND) {
		tx_active_ = 0;
		tx_state_ = MAC_IDLE;
		reset_cw();
        // senddone start to turn off radio
        if (dutyTimer.interval() > 0 && !God::instance()->isSink(index_)) {
            onRadioTimer.restart(onRadioTimer.interval());
        }
	    Packet *p = pktTx_;
	    pktTx_ = 0;
		Handler *h = txHandler_;
		txHandler_ = 0;
		h->handle(p);
	}
	else if(tx_state_  == MAC_ACK)
	{
		tx_state_ = MAC_IDLE;
		tx_active_ = 0;
        // send ack done, turn off radio
        if (dutyTimer.interval() > 0 && !God::instance()->isSink(index_)) {
            onRadioTimer.restart(onRadioTimer.interval());
        }
	}
    // after send done, prepare to close radio

}

void Mac_Sensor::ackHandler() {
    if (acked_!=true) {
        //resend
        // actually, we should send data out immediately, but simulation is too fast, slow it down
        double backoff = 0.1+(Random::random() % cw_) * config.backoff_slot;
        preembleTimer.restart(backoff);
        resending_ = true;
    }
    else {
        acked_ = false;
//        resending_ = false;
        preembleHandler(false);      
    }
}

void Mac_Sensor::dutyHandler()
{
    newnetif()->turnOnRadio();
    if (dutyTimer.interval()>0 && !God::instance()->isSink(index_)) {
        dutyTimer.restart(dutyTimer.interval());
        // duty cycle fired, turn on and turn off later
        onRadioTimer.restart(onRadioTimer.interval());
    }
}

void Mac_Sensor::onRadioHandler() {
    if (God::instance()->isSink(index_)) return;
    if (tx_state_ == MAC_IDLE && tx_active_ == 0 && rx_state_ == MAC_IDLE && resending_!=true) {
        newnetif()->turnOffRadio();
    }
    else {
        onRadioTimer.restart(onRadioTimer.interval());
    }
}

void Mac_Sensor::preembleHandler(bool resend) {
    if (resend!=true || resendCount_ > 20) {
        if (preembleTimer.busy()) {preembleTimer.stop();}
		tx_state_ = MAC_IDLE;
		tx_active_ = 0;
        resending_ = false;
		reset_cw();
        if (pktTxBackup_!=0) {Packet::free(pktTxBackup_); pktTxBackup_ = 0;}
        if (dutyTimer.interval()>0 && !God::instance()->isSink(index_)) {onRadioTimer.restart(onRadioTimer.interval());}
        resendCount_ = 0;
	    Packet *p = pktTx_;
	    pktTx_ = 0;
		Handler *h = txHandler_;
		txHandler_ = 0;
		h->handle(p);
    }
    else {
        if (txHandler_ != 0 && pktTxBackup_!= 0) {
            assert(pktTxBackup_!=0);
            resending_ = true;
            if (pktTxBackup_!=0) {
                pktTx_ = pktTxBackup_->copy();
                Packet::free(pktTxBackup_);
                pktTxBackup_ = 0;
            }
            double backoff = 0.1+(Random::random() % cw_) * config.backoff_slot;
            waitTimer.restart(backoff + config.CCA);
            resendCount_++;
        }
        //else {
            //printf("error!!! node %d txHandler_ %x pktTxBackup_ %x\n", index_,txHandler_,pktTxBackup_);
        //}
    }
}

/*************************************************
 **********************Timers**********************
 *************************************************/
void Mac_SensorTimer::restart(double time)
{
	if (busy_)
		stop();
	start(time);
}



void Mac_SensorTimer::start(double time)
{
	//Scheduler &s = Scheduler::instance();

	assert(busy_ == 0);

	busy_ = 1;
	stime = Scheduler::instance().clock();
	rtime = time;
	assert(rtime >= 0.0);

	Scheduler::instance().schedule(this, &intr, rtime);
}

void Mac_SensorTimer::stop(void)
{
//	Scheduler &s = Scheduler::instance();

	assert(busy_);
	Scheduler::instance().cancel(&intr);
	busy_ = 0;
	stime = rtime = 0.0;
}

void Mac_SensorTimer::forcestop() {
    if (busy_) {
        stop();
    }
}

void Mac_SensorWaitTimer::handle(Event *e)
{
	busy_ = 0;
	stime = rtime = 0.0;

	mac->waitHandler();
}

void Mac_SensorSendTimer::handle(Event *e)
{
	busy_ = 0;
	stime = rtime = 0.0;

	mac->sendHandler();
}

void Mac_SensorRecvTimer::handle(Event *e)
{
	busy_ = 0;
	stime = rtime = 0.0;

	mac->recvHandler();
}

void Mac_SensorACKTimer::handle(Event *e)
{
	busy_ = 0;
	stime = rtime = 0.0;

	mac->ackHandler();
}

void Mac_SensorDutyTimer::handle(Event *e) {
    busy_ = 0;
    stime = rtime = 0.0;
    mac->dutyHandler();
}

void Mac_SensorOnRadioTimer::handle(Event *e) {
    busy_ = 0;
    stime = rtime = 0.0;
    mac->onRadioHandler();
#if 0
    if (recvCnt<lastRecvCnt) {
        lastRecvCnt = recvCnt;
    }
    onperiod_ = onperiod_-0.05;
    if (onperiod_ < minimalOntime_) onperiod_ = minimalOntime_;
    recvCnt = 0;
#endif
}

void Mac_SensorPreembleTimer::handle(Event *e) {
    busy_ = 0;
    stime = rtime = 0.0;
    mac->preembleHandler(true);
}

void Mac_SensorOnRadioTimer::increaseInterval(){
#if 1
    recvCnt++;
    if (recvCnt>2*(lastRecvCnt+1)) {
        onperiod_ += 0.01;
        if (onperiod_>nodeDuty_) onperiod_ = onperiod_/2;
        lastRecvCnt = recvCnt+1;
    }
#endif
}











