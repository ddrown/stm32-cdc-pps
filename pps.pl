#!/usr/bin/perl

use strict;
use ppsclient;
use IO::File;

my $pps = IO::File->new("/dev/pps0", "+<");
my $serial = IO::File->new("/dev/ttyACM17", "+<");
my(@tim,@int);
my(%last);
while(<$serial>) {
  $_ =~ s/[\r\n]//gs;
  if(/^TIM /) {
    @tim = split(/ /);
  } elsif(/^INT /) {
    @int = split(/ /);
    my(@data) = ppsclient::pps_poll($pps->fileno);
    if($data[0] != 1) {
      print "status: $data[0]\n";
    } else {
      my(%current) = (
        ms => $tim[1],
        irq => $tim[2],
        capture => $tim[3],
        sent => $int[1],
        sent_us => $int[2],
        sent_ms => $int[3],
        timestamp_s => $data[2],
        timestamp_ns => $data[3]
      );

      $current{cap_to_irq} = ($current{irq} % 65536) - $current{capture};
      if($current{cap_to_irq} < 0) {
        $current{cap_to_irq} += 65536;
      }
      $current{cap_to_sent_cycles} = $current{sent} - $current{irq};
      if($current{cap_to_sent_cycles} < 0) {
        $current{cap_to_sent_cycles} += 2**32;
      }
      $current{cap_to_sent_cycles} += $current{cap_to_irq};
      $current{cap_to_sent_ns} = $current{cap_to_sent_cycles} / 0.072;

      $current{adjusted_ts_ns} = $current{timestamp_ns} - $current{cap_to_sent_ns};
      $current{adjusted_ts_s} = $current{timestamp_s};
      if($current{adjusted_ts_ns} < 0) {
        $current{adjusted_ts_ns} += 1000000000;
        $current{adjusted_ts_s}--;
      }

      printf("%u.%09u %u.%09u %u %u %u\n", $current{timestamp_s}, $current{timestamp_ns}, $current{adjusted_ts_s}, $current{adjusted_ts_ns}, $current{cap_to_irq}, $current{cap_to_sent_cycles}, $current{cap_to_sent_ns});

      %last = %current;
    }
  }
}
