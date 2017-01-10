#include "rilmac.h"
#include "random.h"
#include "time.h"


static class RILMacClass : public TclClass {
public:
  RILMacClass() : TclClass("Mac/RILMac") {}
  TclObject* create(int, const char*const*) {
    return new RILMac();
  }
} class_RILMac;


RILMac::RILMac() : Mac(), config_(),
  waitTimer(this), sendTimer(this), recvTimer(this), ackTimer(this),
  wakeTimer(this), onRadioTimer(this)
{
    rx_state_ = tx_state_ = MAC_IDLE;
    tx_active_ = 0;
    sender_cw_ = cw_ = config_.cw_min; retryCount_ = 0; thresh_ = 4;
    pktRx_ = pktTx_ = pktRRTS_ = pktCTRL_ = 0;
    nextrx_wakeup_ = NOW; nexttx_wakeup_ = 999999999;
    last_alive_ = 0; needtoclear_ = -1;
    pending_txdata_ = FALSE;pending_rxdata_ = FALSE; more_rxdata_ = FALSE;
    last_send_beacon_ = NOW; last_receiver_alive_ = NOW;
    beaconRetry_ = 0;

    bind("mu",&mu_);
    bind("EstimationType", &EstimationType_);
    if (mu_<0.01) mu_ = 0.01;
    nextrx_wakeup_ = NOW+NOW*mu_;
    wakeTimer.start(NOW*mu_);
}

/*------------------ misc functions ------------------------*/
// update senders and receivers information
double RILMac::updateMySenderInfo(int src, double tallow){
    list<neighborEntry>::iterator it = txTable.begin();
    neighborEntry* s=NULL;
    double min = 99999999;
    if (tallow<0.000001) tallow = 0.000001;
    while (it!=txTable.end()) {
        if (it->id == src) {
            it->pretx = NOW;
            it->wakeup = NOW+tallow;
            s = &(*it);
        }
        while (it->wakeup < NOW+0.000001) {
          it->wakeup += (it->wakeup - it->pretx)*mu_;
        }
        if (it->wakeup < min) {
            min = it->wakeup;
        }
        it++;
    }
    if (s==NULL) {
        s = new neighborEntry(src);
        s->pretx = NOW;
        s->wakeup = NOW+tallow;
        txTable.push_back(*s);
        if (s->wakeup < min) {min = s->wakeup;}
    }
    if (min < NOW) {
      printf("error_x2!! node %d sender %d wrong scheduled rx time %f @ %f\n", index_, s->id, it->wakeup, NOW);
      exit(-1);
    }
    //printf("node %d updatesender %d rxbound %f nextrx %f pretx %f min %f @ %f\n",index_,src,tallow,s->wakeup,s->pretx,min,NOW);
    return min;
}

void RILMac::updateMyReceiverInfo(int src, double rxbound, double nextt){
  list<neighborEntry>::iterator it = rxTable.begin();
  if (nextt<0.000001) nextt = 0.000001;
  while (it!=rxTable.end()) {
      if (it->id == src) {
          it->pretx = NOW;
          it->wakeup = NOW+nextt;
          it->rxbound = NOW+rxbound;
          return;
      }
      it++;
  }
  neighborEntry *r = new neighborEntry(src);
  r->pretx = NOW;
  r->wakeup = NOW+nextt;
  r->rxbound = NOW+rxbound;
  rxTable.push_back(*r);
  if (r->wakeup < NOW) {
    printf("error_y1!! node %d receiver scheduled wrong wakeup time %f\n", index_, r->wakeup);
    exit(-1);
  }
}

double RILMac::reschedule(double current){
  list<neighborEntry>::iterator it = txTable.begin();
  double min = 99999999;
  while (it!=txTable.end()) {
      while (it->wakeup < NOW+0.000001) {
        it->wakeup += (it->wakeup - it->pretx)*mu_;
      }
      if (it->wakeup < min) {
          min = it->wakeup;
      }
      it++;
  }
  if (min <99999999) {
    if (min < NOW + 0.04) {min = NOW+0.04;}
    return min;
  }
  else {
      double relative = (NOW-last_alive_)*mu_+Random::uniform(0.03,0.2);
      return NOW+relative;
  }
}
 
double RILMac::getReceiverWakeupTime(int dst) {
  list<neighborEntry>::iterator it = rxTable.begin();
  double minschedule = 99999999;
  while (it!=rxTable.end()) {
      if (it->id == dst) {
          if (it->wakeup<0) {minschedule = 99999999.1;break;}
          if (it->wakeup < NOW+0.000001) {
            if (it->wakeup < it->rxbound) {it->wakeup = it->rxbound;}
            while (it->wakeup < NOW+0.000001) {
                it->wakeup += (it->wakeup-it->pretx)*mu_;
            }
          }
          if (it->wakeup < minschedule) {
            minschedule=it->wakeup;
          }
          //printf("node %d receiver %d estimated wakeup %f rxbound %f @ %f\n",index_,dst,it->wakeup,it->rxbound,NOW);
      }
      it++;
  }
  if (minschedule < 99999999) return minschedule;
  else return -1;
}

void RILMac::addBackupSchedule(int receiver, double rxbound) {
  neighborEntry *r = new neighborEntry(receiver);
  r->pretx = NOW;
  r->wakeup = NOW+rxbound;
  r->rxbound = NOW+rxbound;
  r->dupsize = 1;
  list<neighborEntry>::iterator it = rxTable.begin();
  printf("add backup newschedule rxbound %f wakeup %f pretx %f\n",r->rxbound,r->wakeup,r->pretx);
  while (it!=rxTable.end()) {
      if (it->id == receiver) {
        it->dupsize = 2;
        r->theta = it->theta;
        r->delta = it->delta;
        r->prearrival = it->prearrival;
        r->alive = it->alive;
        //printf("oldschedule rxbound %f wakeup %f pretx %f\n",it->rxbound,it->wakeup,it->pretx);
      }
      it++;
  }
  rxTable.push_back(*r);
}
void RILMac::clearReceiverInfo(int src) {
  list<neighborEntry>::iterator it = rxTable.begin();
  while (it!=rxTable.end()) {
      if (it->id == src && it->dupsize > 1) {
        it = rxTable.erase(it);
      }
      else {
        if (it->id == src) {it->dupsize = 0;}
        it++;
      }
  }
}

double RILMac::getAliveTime(int dst) {
  list<neighborEntry>::iterator it = rxTable.begin();
  while (it!=rxTable.end()) {
    if (it->id == dst) {
        return it->alive;
    }
    it++;
  }
  return NOW;
}

double RILMac::updateAliveTime(int dst,double alivet) {
  list<neighborEntry>::iterator it = rxTable.begin();
  while (it!=rxTable.end()) {
    if (it->id == dst) {
        it->alive = NOW+alivet;
    }
    it++;
  }
  return NOW;
}

void RILMac::dumpStatus() {
  printf("node %d tx_active %d rx_ %d tx_ %d pendingtx_ %d morerx_ %d acktimer %d waittimer %d@ %f\n",
  index_, tx_active_,rx_state_,tx_state_,pending_txdata_,more_rxdata_,ackTimer.busy(), waitTimer.busy(), NOW);
}

// set or get mac type, used for all messages
inline u_int16_t RILMac::hdr_mactype(char *hdr, u_int16_t type) {
  struct hdr_mac *dh = (struct hdr_mac*) hdr;
  if (type) dh->ftype_ = (MacFrameType)type;
  return dh->ftype_;
}

// set or get submac type, used for all messages
inline u_int16_t RILMac::hdr_submactype(char *hdr, u_int16_t type) {
  struct hdr_mac *dh = (struct hdr_mac*) hdr;
  if (type) {dh->hdr_type_ &= 0x8000; dh->hdr_type_ |= (0x7fff&type);}
  return dh->hdr_type_&0x7fff;
}

// set or get more bit, used only in data messages
inline u_int16_t RILMac::hdr_morebit(char *hdr, int16_t value) {
  struct hdr_mac *dh = (struct hdr_mac*) hdr;
  if (value > -1) {
        if (value==1) dh->hdr_type_ |= 0x8000;
        else dh->hdr_type_ &= 0x7fff;
  }
  u_int16_t morebit = (dh->hdr_type_&0x8000)>>15UL;
  return morebit;
}

// set or get backoff count, used only in ack messages
inline u_int16_t RILMac::hdr_backoff(char *hdr, int16_t value) {
  struct hdr_mac *dh = (struct hdr_mac*) hdr;
  if (value > -1) {
    dh->padding_ &= 0x0000ffff;
    u_int16_t tempvalue = (u_int16_t)value;
    dh->padding_ |= (tempvalue<<16UL);
  }
  return (dh->padding_&0xffff0000)>>16UL;
}

// set or get nextwakeup time, used only in ack messages
inline u_int32_t RILMac::hdr_nextwakeup(char *hdr, int32_t value) {
  struct hdr_mac *dh = (struct hdr_mac*) hdr;
  if (value > -1) {
    dh->padding_ &= 0xffff0000;
    u_int16_t tempvalue = (u_int16_t)value;
    dh->padding_ |= tempvalue;
  }
  return (dh->padding_&0x0000ffff);
}

// set or get theta, used in data messages
inline u_int16_t RILMac::hdr_theta(char *hdr, int16_t value) {
  struct hdr_mac *dh = (struct hdr_mac*) hdr;
  if (value > -1) {
    dh->padding_ &= 0x0000ffff;
    u_int16_t tempvalue = (u_int16_t)value;
    dh->padding_ |= (tempvalue<<16UL);
  }
  return (dh->padding_&0xffff0000)>>16UL;
}

// set or get delta, used in data messages
inline u_int16_t RILMac::hdr_delta(char *hdr, int16_t value) {
  struct hdr_mac *dh = (struct hdr_mac*) hdr;
  if (value > -1) {
    dh->padding_ &= 0xffff0000;
    u_int16_t tempvalue = (u_int16_t)value;
    dh->padding_ |= tempvalue;
  }
  return (dh->padding_&0x0000ffff);
}

// set or get rxbound, used in data messages
inline u_int32_t RILMac::hdr_bound(char *hdr, int32_t value) {
  struct hdr_mac *dh = (struct hdr_mac*) hdr;
  if (value > -1) {
    dh->padding_ &= 0xffff0000;
    u_int16_t tempvalue = (u_int16_t)value;
    dh->padding_ |= tempvalue;
  }
  return (dh->padding_&0x0000ffff);
}

bool RILMac::ifNoOngoingTransaction(){
    return (tx_active_==0 && tx_state_ == MAC_IDLE && rx_state_ == MAC_IDLE && nexttx_wakeup_>NOW &&
            pending_txdata_==FALSE && more_rxdata_ == FALSE && !ackTimer.busy() && !waitTimer.busy());
}

double RILMac::getBoundTime(int dst) {
  list<neighborEntry>::iterator it = rxTable.begin();
  double rxbound = -1;
  //double k;
  //if (mu_+0.000001 > 0.035) k = 1;
  //else {
  //  k = 2;
  //}
  //double nextt = reschedule(NOW);
  while (it!=rxTable.end()) {
      if (it->id == dst) {
        //if (it->theta>0) rxbound = it->theta-it->delta-(NOW-it->prearrival);
        //else rxbound = (nextt-NOW)(1+mu_);
        rxbound = (it->theta-it->delta)*(1+mu_)-(NOW-it->prearrival);
	//printf("node %d receiver %d rxbound %f @ %f\n",index_,it->id,rxbound,NOW);
        break;
      }
      it++;
  }
  //if (nextrx_wakeup_ < NOW+rxbound && nextrx_wakeup_> NOW && !txTable.empty()) {rxbound = nextrx_wakeup_-NOW;}
  if (rxbound>655) rxbound = 655;
  if (rxbound<0.01) rxbound = 0.01;
  return rxbound;
}

// get data tx time
double RILMac::txtime(Packet *p) {
  struct hdr_cmn *ch = HDR_CMN(p);
  double t = ch->txtime();
  if (t < 0.0)
  t = 0.0;
  return t;
}

void RILMac::computeThetaDelta(int dst, double parameter) {
  list<neighborEntry>::iterator it = rxTable.begin();
  neighborEntry* s=NULL;
  while (it!=rxTable.end()) {
      if (it->id == dst) {
        s = &(*it);
        break;
      }
      it++;
  }
  if (s==NULL) {
    s = new neighborEntry(dst);
    s->prearrival = NOW;
    s->theta = NOW-5;
    s->delta = 0.01;
    s->alive = NOW;
    rxTable.push_back(*s);
    printf("node %d add entry for %d\n",index_,dst);
    return;
  }
  double theta = s->theta;
  double delta = s->delta;

  double temptheta = theta;
  double tempdelta = delta;
  double thetanew = NOW-s->prearrival;
  double deltanew = fabs(thetanew - temptheta);
  if (parameter > 1) {
      double weight1 = pow(parameter,-thetanew/(100*theta))*0.9;//pow(parameter, -thetanew/(5*temptheta));
      double weight2 = 0.9;
      theta = weight1*temptheta+(1-weight1)*thetanew;
      delta = weight2*tempdelta+(1-weight2)*deltanew;
  }
  else {
    theta = (temptheta+thetanew)/2;
    delta = (tempdelta+deltanew)/2;
  }

  if (theta < 0.01) theta = 0.01;
  if (delta > theta) delta = theta/2;
  s->theta = theta;
  s->delta = delta;
  s->prearrival = NOW;
  //printf("node %d theta %f delta %f @ %f\n",index_,theta,delta,NOW);
  if (s->dupsize > 0) {
    list<neighborEntry>::iterator itr = rxTable.begin();
    while (itr!=rxTable.end()) {
        if (itr->id == dst) {
          itr->theta = theta;
          itr->delta = delta;
          itr->prearrival = NOW;
        }
        itr++;
    }
  }
}

double RILMac::collisionDetection(bool detected) {
  double boundtime = 0.005;
  if (detected) {
    sender_cw_ = (sender_cw_ << 1) + 1;
    if( sender_cw_ > config_.cw_max){
      sender_cw_ = config_.cw_max;
    }
  }
  if (pending_rxdata_ || more_rxdata_) {
    boundtime += sender_cw_*config_.backoff_slot + config_.CCA;
  }
  return boundtime;
}

void RILMac::checktoSendBeacon(double delay){
  double nextrx_ = reschedule(NOW);
  // if it is necessary to send beacon now, send it now
  if (nextrx_<NOW+0.000001) {
    printf("node %d nextrx too close %f @ %f\n",index_, nextrx_, NOW);
    generateBeacon(MAC_RIL_RRTS, BCAST_ADDR);
  }
  else {
    if (nexttx_wakeup_ < nextrx_wakeup_ && nexttx_wakeup_> NOW) {
      wakeTimer.restart(nexttx_wakeup_-NOW);
    }
    else {
      if (nexttx_wakeup_<NOW) {
      newnetif()->turnOnRadio();
      pending_txdata_ = TRUE;
      }
      if (!wakeTimer.busy()){
        nextrx_wakeup_ = nextrx_;
        wakeTimer.start(nextrx_wakeup_-NOW);
      }
      else {
        if (nextrx_< nextrx_wakeup_ && nextrx_>NOW ) {
          nextrx_wakeup_ = nextrx_;
          wakeTimer.restart(nextrx_wakeup_-NOW);
        }
      }
    }
    onRadioTimer.restart(delay);
  }
}

/*-------------------- functions related to recv ----------------------*/
// first entry to handle recv packet
void RILMac::recv(Packet *p, Handler *h) {
  struct hdr_cmn *hdr = HDR_CMN(p);
  if (hdr->direction() == hdr_cmn::DOWN) {
    txHandler_ = h;
    macqueue.push_back(p);
    h->handle((Event*) 0);
//    computeThetaDelta(hdr_dst((char*)HDR_MAC(p)), EstimationType_);
    computeThetaDelta(hdr->next_hop_,EstimationType_);
    tempToSendData(0, index_, false);
    return;
  }

  // If we are transmitting, then set the error bit in the packet
  if(tx_active_ && hdr->error() == 0) {
    hdr->error() = 1;
  }

  if (rx_state_ == MAC_IDLE) {
    rx_state_ = MAC_RECV;
    pktRx_ = p;
    recvTimer.start(txtime(p));
  }
  else {
    // decide whether this packet's power is high enough to cause collision
    if (pktRx_->txinfo_.RxPr / p->txinfo_.RxPr >= p->txinfo_.CPThresh) {
      // power too low, ignore the incoming packet
      Packet::free(p);
    }
    else {
      // power is high enough to cause collision
      rx_state_ = MAC_COLL;
      // decide which packets to be left
      if (txtime(p) > recvTimer.expire()) {
        recvTimer.stop();
        Packet::free(pktRx_);
        pktRx_ = p;
        recvTimer.start(txtime(pktRx_)); // start recv timer to process
      }
      else {
        Packet::free(p);
      }
    }
  }
}

// recv timer handler
void RILMac::recvHandler() {
  if (pktRx_==NULL || (rx_state_ != MAC_RECV && rx_state_ != MAC_COLL)) {
    printf("error_y!!! node %d wrong state in recvHandle\n",index_);
    exit(-1);
  }

  Packet* p = pktRx_;
  hdr_cmn *ch = HDR_CMN(p);
  char* mh = (char*)HDR_MAC(p);
  int dst = hdr_dst(mh);
  u_int16_t  type = hdr_mactype(mh);
  u_int16_t  subtype = hdr_submactype(mh);

  if (tx_active_) {
    Packet::free(p);
    pktRx_ = 0;
    rx_state_ = MAC_IDLE;
    // XXX will a beacon be temped to send after any tx event? must guarantee it
    return;
  }
  if (rx_state_ == MAC_COLL) {
    Packet::free(p);
    pktRx_ = 0;
    rx_state_ = MAC_IDLE;
    // two successive receiption cause collision
    double boundtime = collisionDetection(true);
    checktoSendBeacon(boundtime);
    return;
  }
  if (ch->error()) {
    Packet::free(p);
    pktRx_ = 0;
    rx_state_ = MAC_IDLE;
    // error is caused by pre collision with tx or passed csthreshold
    // it should not affect tx or rx
    double boundtime = collisionDetection(false);
    checktoSendBeacon(boundtime);
    return;
  }

  if (dst != index_ && (MacFrameType)type == MF_CONTROL && subtype == MAC_RIL_ACK){
    subtype = MAC_RIL_RRTS;
    hdr_submactype((char*)HDR_MAC(p), subtype);
    dst = MAC_BROADCAST;
    hdr_dst((char*)HDR_MAC(p), dst);
  }

  if (dst != index_ && dst != BCAST_ADDR) {
    Packet::free(p);
    pktRx_ = 0;
    rx_state_ = MAC_IDLE;
    double boundtime = collisionDetection(true);
    checktoSendBeacon(boundtime);
    return;
  }

  switch(type) {
      case MF_CONTROL:
        switch(subtype) {
        case MAC_RIL_ACK:
          recvACK(pktRx_);
          break;
        case MAC_RIL_RRTS:
          recvBeacon(pktRx_);
          break;
        case MAC_RIL_ONDEMAND:
          recvOndemand(pktRx_);
          break;
        default:
          fprintf(stderr,"Invalid MAC Control Subtype %x\n", subtype);
          exit(1);
        }
        break;
      case MF_DATA:
        switch(subtype) {
        case MAC_RIL_DATA:
          recvDATA(pktRx_);
          break;
        default:
          fprintf(stderr, "Invalid MAC Data Subtype %x\n", subtype);
          exit(1);
        }
        break;
      default:
        fprintf(stderr, "Invalid MAC Type %x\n", type);
        exit(1);
  }
  return;
}

//---------------------------- process recv pkt ------------------------//
// process recv data pkt
void RILMac::recvDATA(Packet *p) {
  char* mh = (char*)HDR_MAC(p);
  int dst = hdr_dst(mh);
  int src = hdr_src(mh);
  last_alive_ = NOW;

  if (tx_state_ != MAC_IDLE || rx_state_ != MAC_RECV){
    printf("error_z!!! node %d wrong state in recvDATA \n", index_);
    exit(-1);
  }
  bool sentack = false;
  if(dst != BCAST_ADDR) {
    reset_cw();
    double defaultinterval = hdr_bound(mh)/100.0; 
    double nextrx = updateMySenderInfo(src, defaultinterval);
    if (!wakeTimer.busy()) {nextrx_wakeup_ = nextrx; wakeTimer.restart(nextrx-NOW);}
    else {
	    if (nextrx < nextrx_wakeup_ && (nexttx_wakeup_>nextrx || nexttx_wakeup_<NOW) && nextrx>NOW ) {
            nextrx_wakeup_ = nextrx;
            wakeTimer.restart(nextrx_wakeup_-NOW);
	    }
    }
    nextrx = (nextrx-NOW)*100.0;
    int offset = (int)nextrx;
    if (offset<0) offset = 0;
    more_rxdata_ = hdr_morebit(mh);
    sentack = sendAck(src, offset);
  }
  uptarget_->recv(p->copy(), (Handler*) 0);
  Packet::free(p); pktRx_ = 0; rx_state_ = MAC_IDLE;

  double boundtime = 0.01;
  if(dst != BCAST_ADDR) {
    boundtime = collisionDetection(true);
  }
  else {
    boundtime = collisionDetection(false);
  }
  checktoSendBeacon(boundtime);
}

void RILMac::recvOndemand(Packet *p) {
    printf("not supported - ondemand message\n");
    Packet::free(p);
    pktRx_ = 0;
    rx_state_ = MAC_IDLE;
    exit(-1);
}

// recv an ack pkt
void RILMac::recvACK(Packet *p) {
  if (tx_state_ != MAC_IDLE || rx_state_ != MAC_RECV){
    printf("error_z!!! node %d wrong state in recvDATA \n", index_);
    exit(-1);
  }
  int src = hdr_src((char*)HDR_MAC(p));
  bool more = false;
  if(pktTx_ != NULL) {
    double rxbound = hdr_bound((char*)HDR_MAC(pktTx_))/100.0;
    double nextt = hdr_nextwakeup((char*)HDR_MAC(p))/100.0;
    more = hdr_morebit((char*)HDR_MAC(pktTx_));
    if (needtoclear_ == src) {
      clearReceiverInfo(src);
      printf("node %d clear receiver %d info @ %f\n",index_,src,NOW);
      needtoclear_ = -1;
    }
    updateMyReceiverInfo(src, rxbound,nextt);
    Packet::free(pktTx_); pktTx_ = 0; tx_state_ = MAC_IDLE; tx_active_ = 0;
    if (ackTimer.busy()) ackTimer.stop();
    if (waitTimer.busy()) waitTimer.stop();
    retryCount_ = 0;
    reset_cw();
    if (nexttx_wakeup_ < nextrx_wakeup_ && nexttx_wakeup_ > NOW) {if (wakeTimer.busy()) wakeTimer.stop();}
    nexttx_wakeup_ = 999999999;
    //printf("node %d receive an ack nextt %f @ %f\n",index_, nextt,NOW);
  }
  else {
    printf("node %d recv delayed ack @ %f\n",index_, NOW);
  }

  Packet::free(p); pktRx_ = 0; rx_state_ = MAC_IDLE;

  // try to send immediately if i have more data
  if (more && (pktTx_!= NULL || !macqueue.empty())) {
    tempToSendData(0, src, TRUE);
  }
  else {
    double boundtime = collisionDetection(false);
    checktoSendBeacon(boundtime);
  }
  return;
}

// receive a receivers beacon (an ack to other nodes or a bcast rrts)
void RILMac::recvBeacon(Packet *p) {
  int src = hdr_src((char*)HDR_MAC(p));
  unsigned int sendercw = hdr_backoff((char*)HDR_MAC(p))/2;
  double backofftime = (Random::random()%sendercw+1)*config_.backoff_slot;
  Packet::free(p); pktRx_ = 0; rx_state_ = MAC_IDLE;
  reset_cw();
  //printf("node %d recv beacon from %d @ %f\n",index_,src, NOW);
  if (pktTx_ == NULL && !macqueue.empty()) {pktTx_ = macqueue.front();macqueue.pop_front();}
  if (pktTx_!= NULL) {
    if (hdr_dst((char*)HDR_MAC(pktTx_)) == src) {
      tempToSendData(backofftime, src, TRUE);
    }
    else {
      double boundtime = collisionDetection(true);
      checktoSendBeacon(boundtime);
    }
  }
  else {
    double boundtime = collisionDetection(true);
    checktoSendBeacon(boundtime);
  }

  return;
}


void RILMac::generateBeacon(MacFrameSubType type, int dest ) {
  if(pktRRTS_){
    Packet::free(pktRRTS_); pktRRTS_ = 0;
  }
  sendBeacon(MAC_BROADCAST);
}

// RIL MAC beacon handling functions
void RILMac::sendBeacon(int dst) {
  Packet *p = Packet::alloc();
  hdr_cmn* ch = HDR_CMN(p);
  ch->direction() = hdr_cmn::DOWN;
  ch->size() = config_.header_len;
  ch->uid() = 0;
  ch->ptype() = PT_MAC;
  ch->iface() = -2;
  ch->error() = 0;
  ch->txtime() = (8.0 * ch->size()) / config_.data_rate;

  char *mh = (char*)HDR_MAC(p);
  hdr_mactype(mh, MF_CONTROL);
  hdr_submactype(mh, MAC_RIL_RRTS);
  hdr_backoff(mh, sender_cw_);
  hdr_dst(mh, dst);
  hdr_src(mh, index_);

  pktRRTS_ = p;
  if (tx_state_ == MAC_IDLE && rx_state_ == MAC_IDLE && tx_active_ == 0) {
    tx_state_ = MAC_RIMACBEACON;
    tx_active_ = 1;
    downtarget_->recv(pktRRTS_->copy(), this);
    sendTimer.restart(HDR_CMN(pktRRTS_)->txtime());
    //printf("node %d send beacon @ %f\n",index_, NOW);
    beaconRetry_ = 0;
    last_send_beacon_ = NOW;
  }
  else {
    // XXX how to handle this failed sendbeacon?
    beaconRetry_++;
    //onRadioTimer.restart(0.00001);
    //printf("node %d fail to send beacon nextrx_ %f beaconRetry_ %d @ %f \n",index_,nextrx_wakeup_,beaconRetry_,NOW);
    //dumpStatus();
  }
}

bool RILMac::sendAck(int dst, int wakeupoffset) {
  Packet *p = Packet::alloc();
  if (pktCTRL_!=0) {Packet::free(pktCTRL_);pktCTRL_=0;}
  char *mh = (char*)HDR_MAC(p);
  hdr_mactype(mh, MF_CONTROL);
  hdr_submactype(mh, MAC_RIL_ACK);
  hdr_dst(mh, dst);
  hdr_src(mh, index_);
  hdr_nextwakeup(mh, wakeupoffset);
  hdr_backoff(mh, sender_cw_);

  hdr_cmn* ch = HDR_CMN(p);
  ch->direction() = hdr_cmn::DOWN;
  ch->size() = config_.ack_len;
  ch->uid() = 0;
  ch->ptype() = PT_MAC;
  ch->iface() = -2;
  ch->error() = 0;
  ch->txtime() = (8.0 * ch->size()) / config_.data_rate;
  pktCTRL_ = p;
  if (tx_state_ == MAC_IDLE && rx_state_ == MAC_RECV && tx_active_ == 0) {
    tx_active_ = 1;
    tx_state_ = MAC_ACK;
    last_send_beacon_ = NOW;
    //printf("node %d sendack time %f moredata %d @ %f\n",index_,ch->txtime(),more_rxdata_,NOW);
    sendTimer.restart(ch->txtime());
    downtarget_->recv(pktCTRL_->copy(),  (Handler*)0);
    return true;
  }
  else {
    printf("node %d fail to send ack @ %f\n",index_,NOW);
    dumpStatus();
    // fail to send ack would not cause resend
    exit(-1);
  }
}

// this is the core to send a data pkt
void RILMac::tempToSendData(double backofftime, int src, bool mustSend) {
  if (pktTx_ == NULL) {
    if (macqueue.empty()) {
        printf("node %d no data to send when tempto send\n",index_);
        exit(-1);
    }
    else {
      pktTx_ = macqueue.front();
      macqueue.pop_front();
    }
  }

  hdr_cmn* ch = HDR_CMN(pktTx_);
  ch->txtime() = (8.0 * (ch->size() + config_.header_len)) / config_.data_rate;

  char* mh = (char*)HDR_MAC(pktTx_);
  hdr_mactype(mh, MF_DATA);
  hdr_submactype(mh, MAC_RIL_DATA);
  int dst = hdr_dst(mh);

  double boundtime = 0.01;

  if (dst == src) {
    if (waitTimer.busy()) {
      waitTimer.stop();
    }
    if (ackTimer.busy()) {ackTimer.stop();retryCount_ = 0;}
    double nextrxtime = getBoundTime(dst)*100.0;
    u_int16_t temp_nextrxtime_ = (u_int16_t)nextrxtime;
    hdr_bound(mh, temp_nextrxtime_);
    if (!macqueue.empty()) hdr_morebit(mh, 1);
    else hdr_morebit(mh, 0);
    waitTimer.restart(backofftime+config_.CCA);
    pending_txdata_ = true;
    last_receiver_alive_ = NOW;
  }
  else {
    boundtime = collisionDetection(false);
    double scheduledrx = getReceiverWakeupTime(dst)-0.015;
    // next scheduledrx time is comming
    if (scheduledrx<NOW+0.000001) {
      newnetif()->turnOnRadio();
      pending_txdata_ = TRUE;
      //nexttx_wakeup_ = scheduledrx;
      nexttx_wakeup_ = 0;
      //boundtime = 0.015*3;
      //printf("node %d wait for beacon scheduledrx %f @ %f\n",index_,scheduledrx, NOW);
    }
    else {
      nexttx_wakeup_=scheduledrx;
      //printf("node %d missed scheduledrx %f @ %f\n",index_,scheduledrx, NOW);
    }
    checktoSendBeacon(boundtime);
  }
}

void RILMac::RetransmitDATA() {
  if(pktTx_ == NULL) {
    printf("node %d invalid pktTx_ in RetransmitDATA\n", index_);
    exit(-1);
  }

  char* mh = (char*)HDR_MAC(pktTx_);

  if((u_int32_t)hdr_dst(mh) == MAC_BROADCAST) {
    Packet::free(pktTx_); pktTx_ = 0;
    tx_active_ = 0; tx_state_ = MAC_IDLE;
    retryCount_ = 0;
    reset_cw();
    pending_txdata_ = FALSE;
    if (nexttx_wakeup_ < nextrx_wakeup_ && nexttx_wakeup_ > NOW) {if (wakeTimer.busy()) wakeTimer.stop();}
    nexttx_wakeup_ = 999999999;
    more_rxdata_ = FALSE; pending_rxdata_ = FALSE;
    checktoSendBeacon(false);
    //printf("node %d after sending broadcast data @ %f\n",index_, NOW);
    return;
  }

  retryCount_++;

  if(retryCount_ > thresh_) {
    if (ackTimer.busy()) { ackTimer.stop();}
    if (waitTimer.busy()) { waitTimer.stop();}
    int dst = hdr_dst((char*)HDR_MAC(pktTx_));
    double temp_rxbound = hdr_bound((char*)HDR_MAC(pktTx_))/100.0;
    Packet::free(pktTx_); pktTx_ = 0;
    tx_active_ = 0; tx_state_ = MAC_IDLE;
    more_rxdata_ = FALSE; pending_rxdata_ = FALSE;
    pending_txdata_ = FALSE;
    retryCount_ = 0;
    if (nexttx_wakeup_ < nextrx_wakeup_ && nexttx_wakeup_ > NOW) {if (wakeTimer.busy()) wakeTimer.stop();}
    nexttx_wakeup_ = 999999999;
    printf("node %d reach max retry cw %d @ %f\n", index_,cw_,NOW);
    addBackupSchedule(dst, temp_rxbound); needtoclear_ = dst;
    reset_cw();
    double boundtime = collisionDetection(true);
    checktoSendBeacon(boundtime);
  }
  else {
    ackTimer.restart(config_.CCA+(Random::random() % cw_ +config_.cw_min) * config_.backoff_slot);
  }
}

// backoff timer handler
void RILMac::waitHandler() {
  if(rx_state_ == MAC_IDLE && tx_state_ == MAC_IDLE && tx_active_ == 0) {
      tx_active_ = 1;
      tx_state_ = MAC_SEND;
      sendTimer.restart(HDR_CMN(pktTx_)->txtime());
      downtarget_->recv(pktTx_->copy(), txHandler_);
  }
  else {
    //channel busy, increase contention window
    double backoff = (Random::random() % cw_) * config_.backoff_slot;
    waitTimer.restart(backoff + config_.CCA);
    inc_cw();
  }
}

// senddone timer handler
void RILMac::sendHandler() {
  double boundtime = 0.01;
  if (tx_state_ == MAC_ACK) {
    tx_state_ = MAC_IDLE; tx_active_ = 0;
    Packet::free(pktCTRL_); pktCTRL_ = 0;
    pending_rxdata_ = TRUE;
    if (more_rxdata_) {
      boundtime = collisionDetection(true);
      checktoSendBeacon(boundtime);
    } else {
      if (pktTx_!=NULL || !macqueue.empty()) {
        //printf("node %d tempto forward data after sending ack @ %f\n",index_,NOW);
        tempToSendData(0,index_,false);
      }
      else {
        boundtime = collisionDetection(false);
        checktoSendBeacon(boundtime);
      }
    }
  }
  else if (tx_state_ == MAC_SEND) {
    pending_txdata_ = FALSE;
    tx_state_ = MAC_IDLE; tx_active_ = 0;
    reset_cw();
    RetransmitDATA();
  }
  // send beacon pkt done
  else if (tx_state_ == MAC_RIMACBEACON) {
    Packet::free(pktRRTS_); pktRRTS_ = 0;
    tx_state_ = MAC_IDLE; tx_active_ = 0;
    pending_rxdata_ = TRUE;
    boundtime = collisionDetection(false);
    checktoSendBeacon(boundtime);
  }
  else {
    printf("error_zz!!wrong tx state after senddonwaitTimer.restart(config_.CCA);e %d\n", tx_state_);
    tx_state_ = MAC_IDLE;
    tx_active_ = 0;
    exit(-1);
  }
}

void RILMac::radioHandler() {
#if 0
  double next_wakeup_ = nextrx_wakeup_;
  // if i have pending rx data, check the receiver's wake up time
  if (pktTx_!=NULL && nexttx_wakeup_ > 0) {
    pending_txdata_ = false;
    nexttx_wakeup_ = getReceiverWakeupTime(hdr_dst((char*)(HDR_MAC(pktTx_))))-0.03;
    if (nexttx_wakeup_<=NOW && nexttx_wakeup_>= NOW-0.03) {
      pending_txdata_ = true;
      newnetif()->turnOnRadio();
      //printf("node %d wait for beacon scheduledrx %f @ %f\n",index_,nexttx_wakeup_+0.03, NOW);
    }
    else {
      // it is still too early
      //pending_txdata_ = true;
      //newnetif()->turnOnRadio();
    }
  }
  if (nexttx_wakeup_ < next_wakeup_ && nexttx_wakeup_> NOW) next_wakeup_ = nexttx_wakeup_;

  if (ifNoOngoingTransaction()) {
    newnetif()->turnOffRadio();
    sender_cw_ = config_.cw_min;
    pending_rxdata_ = false;
    pending_txdata_ = false;
    more_rxdata_ = false;
    reset_cw();
//    printf("node %d turnoff radio @ %f\n", index_, NOW);
  }
  else {
    //dumpStatus();
    //more_rxdata_ = false;
  }
  if (next_wakeup_<=NOW) {
    printf("node %d nextwakup is wrong %f @ %f\n", index_, nextrx_wakeup_, NOW);
    nextrx_wakeup_ = NOW+0.000001;
    wakeTimer.restart(0.000001);
  }
  else {
    if (next_wakeup_ < nextrx_wakeup_) {
      // this is due to tx
      wakeTimer.restart(next_wakeup_ - NOW);
      //printf("node %d reset wakuptimer to send nexttx_wakeup_ %f nextrx_wakeup_ %f @ %f\n",index_,nexttx_wakeup_,nextrx_wakeup_,NOW);
    }
    else {
      //printf("node %d reset wakuptimer to recv nexttx_wakeup_ %f nextrx_wakeup_ %f @ %f\n",index_,nexttx_wakeup_,nextrx_wakeup_,NOW);
      if(!wakeTimer.busy()) wakeTimer.start(next_wakeup_ - NOW);
    }
  }
  //printf("node %d next_wakeup_ %f nexttx_ %f nextrx_ %f @ %f\n", index_, next_wakeup_, nexttx_wakeup_, nextrx_wakeup_,NOW);
#endif
  if (ifNoOngoingTransaction()) {
    newnetif()->turnOffRadio();
    sender_cw_ = config_.cw_min;
    pending_rxdata_ = false;
    pending_txdata_ = false;
    more_rxdata_ = false;
    reset_cw();
    //printf("node %d turnoff radio @ %f\n", index_, NOW);
  }
  if (nexttx_wakeup_ < nextrx_wakeup_ && nexttx_wakeup_ > NOW) {
    wakeTimer.restart(nexttx_wakeup_-NOW);
  }
  if (!wakeTimer.busy()) {
    nextrx_wakeup_ = reschedule(NOW);
    wakeTimer.start(nextrx_wakeup_-NOW);
  }

}

// handler of the duty cycle timer
void RILMac::wakeupHandler() {
  newnetif()->turnOnRadio();
  handleWakeup();
}

void RILMac::handleWakeup() {
  bool sendbeacon = false;
  if (nextrx_wakeup_<=NOW+0.000001) {
    nextrx_wakeup_ = reschedule(NOW);
    //printf("node %d wakeup to receive data nextwakup %f @ %f\n",index_, nextrx_wakeup_, NOW);
    generateBeacon(MAC_RIL_RRTS, BCAST_ADDR);
  }
  if (nexttx_wakeup_<= NOW +0.00001 && pktTx_!= NULL) {
    //printf("node %d wakeup to send data @ %f\n",index_,NOW);
    nexttx_wakeup_ = 0;
    tempToSendData(0,index_,false);
  }
  if (nexttx_wakeup_ > NOW+0.000001 && nextrx_wakeup_ > nexttx_wakeup_) {
    wakeTimer.restart(nexttx_wakeup_- NOW);
  }
  else {
    // nexttx_wakeup_< NOW || nextrx_wakeup < nexttx_wakeup_
    if (!wakeTimer.busy()) {
      wakeTimer.restart(nextrx_wakeup_-NOW);
    }
  }
}

void RILMac::ackHandler() {
  if (pktTx_ == NULL) return;
  if (retryCount_ > 0) {
    if (waitTimer.busy()) {
        waitTimer.stop();
    }
    double backoff = config_.CCA+(Random::random() % cw_ + config_.cw_min) * config_.backoff_slot;
    waitTimer.restart(backoff);
  }
}


// ------------------------ all timers -------------------------//
void RILMacTimer::restart(double time) {
  if (busy_)
    stop();
  start(time);
}

void RILMacTimer::start(double time) {
  Scheduler &s = Scheduler::instance();

  assert(busy_ == 0);
  busy_ = 1;
  stime = s.clock();
  rtime = time;
  assert(rtime >= 0.0);

  s.schedule(this, &intr, rtime);
}

void RILMacTimer::stop(void) {
  Scheduler &s = Scheduler::instance();

  assert(busy_);
  s.cancel(&intr);
  busy_ = 0;
  stime = rtime = 0.0;
}

void RILMacTimer::forcestop() {
    if (busy_) {
        stop();
    }
}

void RILMacWaitTimer::handle(Event *e) {
  busy_ = 0;
  stime = rtime = 0.0;
  mac->waitHandler();
}

void RILMacSendTimer::handle(Event *e){
  busy_ = 0;
  stime = rtime = 0.0;
  mac->sendHandler();
}

void RILMacRecvTimer::handle(Event *e){
  busy_ = 0;
  stime = rtime = 0.0;
  mac->recvHandler();
}

void RILMacACKTimer::handle(Event *e){
  busy_ = 0;
  stime = rtime = 0.0;
  mac->ackHandler();
}

void RILMacWakeupTimer::handle(Event *e) {
    busy_ = 0;
    stime = rtime = 0.0;
    mac->wakeupHandler();
}

void RILMacRadioTimer::handle(Event *e) {
    busy_ = 0;
    stime = rtime = 0.0;
    mac->radioHandler();
}












