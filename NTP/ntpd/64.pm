package NTP::ntpd::64;

use strict;
use base qw(NTP::ntpd);

sub bytes {
  return 96;
}

sub time_t {
  return "q";
}

sub time_t_pad {
  return "l";
}

sub structure_pad {
  return "l";
}

sub message_data {
  my($self, $mode, $count, $remote, $local, $leap, $precision, $nsamples, $valid) = @_;

  return($mode, $count,
      $remote->[0], $remote->[1]/1000,
      0, # time_t_pad
      $local->[0], $local->[1]/1000, 
      $leap, $precision, $nsamples, $valid,
      $remote->[1],$local->[1],
      0,0,0,0,0,0,0,0,
      0 # structure_pad
      );
}

1;
