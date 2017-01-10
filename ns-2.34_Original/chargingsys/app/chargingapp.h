/*
 * nrrapplication.h
 *
 *  Created on: Aug 19, 2009
 *      Author: yang
 */

#ifndef CHARGINGAPP_H_
#define CHARGINGAPP_H_

#include "chargingcommon.h"

// nrrapp timer class
class ChargingAppTimer : public TimerHandler {
public:
	ChargingAppTimer(ChargingApp* app, int tp) : appinstance_(app), type(tp), continueRunning(true){}
	~ChargingAppTimer(){
        appinstance_ = 0;
    }
	void stoptimer() {continueRunning = false;}
protected:
	void expire(Event*);
private:
	ChargingApp* appinstance_;
	int type;
	bool continueRunning;
};

// NRRApp should be an application which could be attached to agents
class ChargingApp : public Application{
public:
	ChargingApp(int nodeid):dataT_(0), chargeT_(0), myID(nodeid), packetseq(0), efficiency_(0){
        bind("DataRate", &dataInterval_);
    }
	~ChargingApp(){
	    if (dataT_) dataT_->force_cancel();
	    delete dataT_;
        dataT_ = 0;
	    if (chargeT_) chargeT_->force_cancel();
	    delete chargeT_;
        chargeT_ = 0;
    }
	void 	timeout(int timertype);								// timeout function called by nrr timer
	void 	process_data(int size, AppData* data);	// Process incoming data
protected:
	int 	command(int argc, const char*const* argv);
	void 	start();
	void 	stop();
private:
	// function members
	void 	recvMsg(CAppMessageT_* msg);				// recv data in msg format
	void 	sendMsg(CAppMessageT_* msg);
    void    dumpMsg(CAppMessageT_* msg, bool yes);
    void    produceMsg(CAppMessageT_* msg, int msgtype);
    bool    chargeNode(int targetid, double time, double efficiency);
    double  readEng();
	ChargingAppTimer* dataT_;			// timer for data transmitting
    ChargingAppTimer* chargeT_;
    int     myID;
    int     packetseq;
    int32_t dataInterval_;
    double  efficiency_;
};

#endif /* NRRAPPLICATION_H_ */
