package NTP::ntpd;

use strict;
use IPC::SysV qw(IPC_PRIVATE IPC_RMID S_IRUSR S_IWUSR);

sub new {
  my $class = shift;
  my $self = {@_};
  bless($self,$class);

  $self->_shm_reference();
  $self->_create_format();

  return $self;
}

sub bytes {
  return 80;
}

sub _shm_reference {
  my($self) = @_;

  $self->{shmid} = shmget($self->{shmkey}, $self->bytes(), S_IRUSR | S_IWUSR);
  die "shmget: $!" if ( $self->{shmid} < 0 );
}

sub _create_format {
  my($self) = @_;

  $self->{format} = "ll";
  $self->{format} .= $self->time_t()."l". $self->time_t_pad();
  $self->{format} .= $self->time_t()."l";
  $self->{format} .= "llll" . "ll" . "llllllll" . $self->structure_pad();
}

sub time_t {
  return "l";
}

sub time_t_pad {
  return "";
}

sub structure_pad {
  return "";
}

sub message_data {
  my($self, $mode, $count, $remote, $local, $leap, $precision, $nsamples, $valid) = @_;

  return($mode, $count,
      $remote->[0], $remote->[1]/1000,
      $local->[0], $local->[1]/1000, 
      $leap, $precision, $nsamples, $valid,
      $remote->[1],$local->[1],
      0,0,0,0,0,0,0,0);
}

sub send_time {
  my($self,$local,$remote) = @_;

  my $nsamples = 0;
  my $valid = 1;
  my $precision = -20; # 2^-20 = 953 nanoseconds ~ 1us
  my $leap = 0;
  my $count = 0;
  my $mode = 0;
 
  my $message = pack( $self->{format}, $self->message_data($mode, $count, $remote, $local, $leap, $precision, $nsamples, $valid));

  my $len = length($message);
  die "wrong message length $len != ".$self->bytes() unless($len == $self->bytes());
  shmwrite($self->{shmid}, $message, 0, $len) || die("$!");  
}

1;
