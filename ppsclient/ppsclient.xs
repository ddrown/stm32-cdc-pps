#define PERL_NO_GET_CONTEXT
#include "EXTERN.h"
#include "perl.h"
#include "XSUB.h"
#include <linux/pps.h>
#include <sys/ioctl.h>

#include "ppport.h"


MODULE = ppsclient		PACKAGE = ppsclient		
void
pps_wait(fd)
	int fd
	INIT:
		struct pps_fdata fetch_data;
		int status;
	PPCODE:
		fetch_data.timeout.flags = PPS_TIME_INVALID;
		status = ioctl(fd, PPS_FETCH, &fetch_data);
		if(status < 0) {
			XPUSHs(sv_2mortal(newSVnv(0)));
		} else {
			XPUSHs(sv_2mortal(newSVnv(1)));
			XPUSHs(sv_2mortal(newSVnv(fetch_data.info.assert_sequence)));
			XPUSHs(sv_2mortal(newSVnv(fetch_data.info.assert_tu.sec)));
			XPUSHs(sv_2mortal(newSVnv(fetch_data.info.assert_tu.nsec)));
			XPUSHs(sv_2mortal(newSVnv(fetch_data.info.clear_sequence)));
			XPUSHs(sv_2mortal(newSVnv(fetch_data.info.clear_tu.sec)));
			XPUSHs(sv_2mortal(newSVnv(fetch_data.info.clear_tu.nsec)));
		}

void
pps_poll(fd)
	int fd
	INIT:
		struct pps_fdata fetch_data;
		int status;
	PPCODE:
		fetch_data.timeout.flags = fetch_data.timeout.sec = fetch_data.timeout.nsec = 0;
		status = ioctl(fd, PPS_FETCH, &fetch_data);
		if(status < 0) {
			XPUSHs(sv_2mortal(newSVnv(0)));
		} else {
			XPUSHs(sv_2mortal(newSVnv(1)));
			XPUSHs(sv_2mortal(newSVnv(fetch_data.info.assert_sequence)));
			XPUSHs(sv_2mortal(newSVnv(fetch_data.info.assert_tu.sec)));
			XPUSHs(sv_2mortal(newSVnv(fetch_data.info.assert_tu.nsec)));
			XPUSHs(sv_2mortal(newSVnv(fetch_data.info.clear_sequence)));
			XPUSHs(sv_2mortal(newSVnv(fetch_data.info.clear_tu.sec)));
			XPUSHs(sv_2mortal(newSVnv(fetch_data.info.clear_tu.nsec)));
		}
