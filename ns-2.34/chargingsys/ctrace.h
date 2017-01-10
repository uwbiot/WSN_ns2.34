/*
 * nrrapplication.h
 *
 *  Created on: Aug 19, 2009
 *      Author: yang
 */

#ifndef CTRACE_H
#define CTRACE_H

#include <stdio.h>
#include <stdarg.h>
#include <assert.h>

#include "object.h"


class CTRACE : public TclObject{
//class CTRACE {
public:
	CTRACE(const char* filename){
        logp = fopen(filename, "w");
    }
	~CTRACE(){
        if (logp!=0) {fclose(logp); logp = 0;}
    }
    void log( const char* format, ... );
private:
    FILE* logp;
};

#endif /* NRRAPPLICATION_H_ */
