# Before 'make install' is performed this script should be runnable with
# 'make test'. After 'make install' it should work as 'perl ppsclient.t'

#########################

use strict;
use warnings;

use Test::More tests => 3;
BEGIN { use_ok('ppsclient') };

#########################

is( ppsclient::pps_poll(0), 0, "call pps_poll" );
is( ppsclient::pps_wait(0), 0, "call pps_wait" );

