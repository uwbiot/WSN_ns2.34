/*
 * nrrapplication.cc
 *
 *  Created on: Aug 19, 2009
 *      Author: yang
 */

#include "ctrace.h"

static class CTRACEClass : public TclClass {
public:
	CTRACEClass() : TclClass("CTRACE") {}
	TclObject* create(int argc, const char*const* argv) {
		assert(argc==5);
		return (new CTRACE(argv[4]));
	}
}class_CTRACE;


void CTRACE::log( const char* format, ... ) {
    if (logp!=0) {
        va_list args;
        va_start( args, format );
        vfprintf( logp, format, args );
        va_end( args );
        fflush (logp);
    }
}


