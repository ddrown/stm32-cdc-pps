#!/usr/bin/perl

use strict;
use ppsclient;
use IO::File;
use Getopt::Long;
use FindBin;
use lib "$FindBin::Bin";
use NTP::ntpd::64;

my(%config) = (
  pps => "/dev/pps0",
  shmid => undef
);

GetOptions(
  "pps=s" => \$config{pps},
  "shmid=s" => \$config{shmid}
) or die("Error in command line arguments\n");

if(not defined($config{shmid})) {
  $config{shmid} = 0x4e545030;
} else {
  $config{shmid} = hex($config{shmid});
}

$| = 1;

my $ntpd = NTP::ntpd::64->new(shmkey => $config{shmid});

my $pps = IO::File->new($config{pps}, "+<");
my($last_assert_sequence);
while(<>) {
  $_ =~ s/[\r\n]//gs;
  if(/^\$INT,/) {
    my(@int) = split(/[,*]/);
    my(@data) = ppsclient::pps_poll($pps->fileno);
    if($data[0] != 1) {
      print "status: $data[0]\n";
    } else {
      my(%current) = (
        irq => $int[1],
        capture => $int[2],
        sent => $int[3],
        timestamp_s => $data[2],
        timestamp_ns => $data[3]
      );
      if($last_assert_sequence == $data[1]) {
        print("no change in assert sequence, skipping\n");
        next;
      }
      $last_assert_sequence = $data[1];

      # cycles between pps capture and timer interrupt
      $current{cap_to_irq} = ($current{irq} % 65536) - $current{capture};
      if($current{cap_to_irq} < 0) {
        $current{cap_to_irq} += 65536;
      }
  
      # cycles between pps capture and usb status sent
      $current{cap_to_sent_cycles} = $current{sent} - $current{irq};
      if($current{cap_to_sent_cycles} < 0) {
        $current{cap_to_sent_cycles} += 2**32;
      }
      $current{cap_to_sent_cycles} += $current{cap_to_irq};

      # nanoseconds between pps capture and usb status sent
      $current{cap_to_sent_ns} = $current{cap_to_sent_cycles} / 0.072;

      # adjust pps timestamp for delay between pps capture and usb status sent
      $current{adjusted_ts_ns} = $current{timestamp_ns} - $current{cap_to_sent_ns};
      $current{adjusted_ts_s} = $current{timestamp_s};
      if($current{adjusted_ts_ns} < 0) {
        $current{adjusted_ts_ns} += 1000000000;
        $current{adjusted_ts_s}--;
      }

      # check sanity
      if($current{cap_to_sent_ns} > 100000000) { # 100ms
        print("cap to sent ns over 100ms: ".$current{cap_to_sent_ns}.", skipping\n");
        next;
      }

      my($local) = [$current{adjusted_ts_s}, $current{adjusted_ts_ns}];
      my($remote) = [$current{adjusted_ts_s}, 0];
      # assumption: local clock is within +/-500ms of true time
      if($current{adjusted_ts_ns} > 500000000) {
        $remote->[0]++;
      }

      printf("%u.%09u %u %u %u\n", $current{adjusted_ts_s}, $current{adjusted_ts_ns}, $remote->[0], $current{cap_to_irq}, $current{cap_to_sent_ns});
      $ntpd->send_time($local, $remote);
    }
  }
}
