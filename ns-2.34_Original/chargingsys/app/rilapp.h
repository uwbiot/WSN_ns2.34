/*
 * rilapplication.h
 *
 *  Created on: Aug 19, 2009
 *      Author: yang
 */

#ifndef RILAPP_H_
#define RILAPP_H_

#include "rilcommon.h"

class RILAppTimer : public TimerHandler {
public:
	RILAppTimer(RILApp* app, int tp) : appinstance_(app), type(tp), continueRunning(true){}
	~RILAppTimer(){
        appinstance_ = 0;
    }
	void stoptimer() {continueRunning = false;}
protected:
	void expire(Event*);
private:
	RILApp* appinstance_;
	int type;
	bool continueRunning;
};

class RILApp : public Application{
public:
	RILApp(int nodeid):dataT_(0), myID(nodeid), packetseq(0){
        bind("DataRate", &dataInterval_);
    }
	~RILApp(){
	    if (dataT_) dataT_->force_cancel();
	    delete dataT_;
        dataT_ = 0;
    }
	void 	timeout(int timertype);								// timeout function called by nrr timer
	void 	process_data(int size, AppData* data);	// Process incoming data
protected:
	int 	command(int argc, const char*const* argv);
	void 	start();
	void 	stop();
private:
	// function members
	void 	recvMsg(RILAppMessageT_* msg);				// recv data in msg format
	void 	sendMsg(RILAppMessageT_* msg);
    void    dumpMsg(RILAppMessageT_* msg, bool yes);
    void    produceMsg(RILAppMessageT_* msg, int msgtype);
    double  next_interval();
	RILAppTimer* dataT_;			// timer for data transmitting
    int     myID;
    int     packetseq;
    double dataInterval_;
};

#endif /* NRRAPPLICATION_H_ */



