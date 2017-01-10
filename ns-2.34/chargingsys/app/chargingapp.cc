/*
 * nrrapplication.cc
 *
 *  Created on: Aug 19, 2009
 *      Author: yang
 */

#include "chargingapp.h"
#include "charging-energy-model.h"
#include "charger.h"

static class ChargingAppClass : public TclClass {
public:
	ChargingAppClass() : TclClass("Application/ChargingApp") {}
	TclObject* create(int argc, const char*const* argv) {
		assert(argc==5);
		return (new ChargingApp(atoi(argv[4])));
	}
}class_ChargingApp;


int ChargingApp::command(int argc, const char*const* argv) {
	//Tcl& tcl = Tcl::instance();
	if (argc == 2) {
		if (strcmp(argv[1], "start") == 0) {
			start();
			return (TCL_OK);
		}
		else if (strcmp(argv[1], "stop") == 0) {
			stop();
			return (TCL_OK);
		}
	}
    else if (argc == 3) {
        if (strcmp(argv[1], "set-efficiency") == 0) {
            efficiency_ = atof(argv[2])*3/1000;
            //printf("efficiency %f\n", efficiency_);
   			return (TCL_OK);
        }
    }
	return (Application::command(argc, argv));
}

bool ChargingApp::chargeNode(int targetid, double time, double eff){
    bool result = false;    
    MobileNode* mb = God::instance()->getNode(targetid);
    if (mb!=0) {
        ChargingEnergyModel* cem = dynamic_cast<ChargingEnergyModel*>(mb->energy_model());
        if (cem!=0) {
            cem->charge(time, eff);
            //printf("start charging node %d for %f seconds\n", targetid, time);
            result = true;
        }
    }
    return result;
}

double ChargingApp::readEng(){
    MobileNode* mb = God::instance()->getNode(myID);
    if (mb!=0) {
        ChargingEnergyModel* cem = dynamic_cast<ChargingEnergyModel*>(mb->energy_model());
        if (cem!=0) {
            return cem->energy();
        }
    }
    return 0.0;
}

void ChargingApp::timeout(int type) {
    static double lefttoCharge_ = 0;
    static int nodetoCharge_ = 0;
    if (type == DTIMER) {
	    CAppMessageT_ msg;
        if (!God::instance()->isSink(myID)) {
	        produceMsg(&msg, DATA_SENSING);
            sendMsg(&msg);
            //printf("node %d send app msg @ %f\n", myID, NOW);
        }
	    dataT_->resched(dataInterval_+Random::uniform()*5);
    }
    else if (type == CTIMER) {
        if (lefttoCharge_ < 1) {
            vector<pair<int, double> > seq = Charger::getChargeSeq();
            if (seq.size() != 0) {
                #if 0
                for (int i = 0; i < seq.size(); i++) {
	                int id = seq[i].first;
	                double ct = seq[i].second;
	                cout << "id: " << id << " ct: " << ct << " eng: "<<Charger::getNodeEng(id)<<endl;
                }
                #endif
                nodetoCharge_ = seq[0].first; lefttoCharge_ = seq[0].second;
//God::instance()->ctrace()->log("charge node %d time %f eng %f @ %f\n", nodetoCharge_, lefttoCharge_, Charger::getNodeEng(nodetoCharge_), NOW);
//                if (lefttoCharge_ < 600) lefttoCharge_ = 0;
            }
            else {
                //printf("empty sequence\n");
        	    chargeT_->resched(600);
                lefttoCharge_ = 0;
                return;
            }
        }
        if (lefttoCharge_ > 3600) lefttoCharge_ = 3600;
        if (lefttoCharge_ > 60) {
            chargeNode(nodetoCharge_, 60, efficiency_);
    	    chargeT_->resched(60);
            God::instance()->moveEng += Charger::doCharge(nodetoCharge_,60);
            lefttoCharge_ -= 60;
        }
        else {
            if (lefttoCharge_ > 0.2) {
                chargeNode(nodetoCharge_, lefttoCharge_, efficiency_);
                God::instance()->moveEng += Charger::doCharge(nodetoCharge_, lefttoCharge_);
        	    chargeT_->resched(lefttoCharge_);
            }
            else chargeT_->resched(600);
            lefttoCharge_ = 0;
        }
    }
}

// process received nrrapp packet
void ChargingApp::process_data(int size, AppData* data) {
	PacketData* p = (PacketData*)data;
	CAppMessageT_* cappmsgp = (CAppMessageT_*)p->data();
    recvMsg(cappmsgp);
	return;
}

void ChargingApp::start() {
	//God::instance()->ctrace()->log("node %d start charging app @ %f\n", myID, NOW);
    if (!God::instance()->isSink(myID)) {
	    dataT_ = new ChargingAppTimer(this, DTIMER);
	    dataT_->sched(dataInterval_+Random::uniform()*5);
    }
    //if (God::instance()->isSink(myID) && God::instance()->enableCharge()) {
    if (myID == God::instance()->sink_num() && God::instance()->enableCharge()) {
	    chargeT_ = new ChargingAppTimer(this, CTIMER);
	    chargeT_->sched(600);
        //God::instance()->ctrace()->log("start chargingtimer %d\n", myID);
    }
}

void ChargingApp::stop() {
	//God::instance()->ctrace()->log("node %d stop charging app @ %f\n", myID, NOW);
	if (dataT_!=0) {
		dataT_->stoptimer();
	}
    if (chargeT_ != 0) {
        chargeT_->stoptimer();
    }
}

void ChargingApp::produceMsg(CAppMessageT_* msg, int msgtype) {
	msg->source = myID;
	msg->msgType = msgtype;
	msg->timestamp = NOW;
	msg->seq = packetseq++;
    double eng = readEng();
    if (msgtype == DATA_SENSING) {
		for (int i=0;i<MAX_DATA_SIZE; i++) {
			msg->data[i] = eng;
		}
		msg->len = MAX_DATA_SIZE;
	}
	return;
}

// send msg based on destination
void ChargingApp::sendMsg(CAppMessageT_* msg) {
	msg->dest = God::instance()->sink_num();
    msg->netType = UCAST;
	PacketData* data = new PacketData(sizeof(CAppMessageT_));
	memcpy(data->data(), msg, sizeof(CAppMessageT_));
	agent_->sendmsg(sizeof(CAppMessageT_), data);
}

void ChargingApp::recvMsg(CAppMessageT_* msg) {
	//dumpMsg(msg, 1);
    if (msg == 0) return;
    if (msg->msgType == DATA_SENSING) {
        double eng = 0;
	    for (int i=0;i<msg->len; i++) {
		    eng+=msg->data[i];
	    }
        eng = eng/msg->len;
        int src = msg->source;
        Charger::setNodeEng(src, eng);
        //God::instance()->ctrace()->log("recv sink %d src %d seq %d eng %f time %f @ %f\n", myID, msg->source, msg->seq, eng, msg->timestamp, NOW);
        double interval = msg->timestamp - God::instance()->lastUpdateTime[src];
        God::instance()->lastUpdateTime[src] = msg->timestamp;
        if (interval > 1) {
            double lasteng = God::instance()->lastUpdateEng[src];
            double workload = (lasteng - eng)/interval;
//            if (lasteng>1) workload = workload*0.5+0.5*(10000-eng)/NOW;
            if (lasteng>1) {God::instance()->lastWorkload[src] = God::instance()->lastWorkload[src]*0.8+workload*0.2;}
            else {God::instance()->lastWorkload[src] = (10000-eng)/NOW;}
            workload = God::instance()->lastWorkload[src];
            God::instance()->lastUpdateEng[src] = eng;
            if (workload > 0) {
                Charger::setWorkload(src, workload);
            }
        }
    }
}

void ChargingApp::dumpMsg(CAppMessageT_* msg, bool yes) {
#if 0
	if (msg!=0 && yes) {
        if (msg->msgType == DATA_SENSING) {
            double eng = 0;
		    for (int i=0;i<msg->len; i++) {
			    eng+=msg->data[i];
		    }
            eng = eng/msg->len;
            int src = msg->source;
            Charger::setNodeEng(src, eng);
            //God::instance()->ctrace()->log("recv sink %d src %d seq %d eng %f time %f @ %f\n", myID, msg->source, msg->seq, eng, msg->timestamp, NOW);
            double interval = msg->timestamp - God::instance()->lastUpdateTime[src];
            God::instance()->lastUpdateTime[src] = msg->timestamp;
            if (interval > 1) {
                double workload = (God::instance()->lastUpdateEng[src] - eng)/interval;
                God::instance()->lastUpdateEng[src] = eng;
                if (workload > 0) {
                    Charger::setWorkload(src, workload);
                }
            }
	    }
	}
#endif
}

/*------------------- NRRApp class stop ----------------------*/

void ChargingAppTimer::expire(Event *) {
	if (continueRunning) {
		appinstance_->timeout(this->type);
	}
}







