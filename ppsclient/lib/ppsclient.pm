package ppsclient;

use 5.024001;
use strict;
use warnings;

require Exporter;

our @ISA = qw(Exporter);

# Items to export into callers namespace by default. Note: do not export
# names by default without a very good reason. Use EXPORT_OK instead.
# Do not simply export all your public functions/methods/constants.

# This allows declaration	use ppsclient ':all';
# If you do not need this, moving things directly into @EXPORT or @EXPORT_OK
# will save memory.
our %EXPORT_TAGS = ( 'all' => [ qw(
	
) ] );

our @EXPORT_OK = ( @{ $EXPORT_TAGS{'all'} } );

our @EXPORT = qw(
	
);

our $VERSION = '0.01';

require XSLoader;
XSLoader::load('ppsclient', $VERSION);

# Preloaded methods go here.

1;
__END__

=head1 NAME

ppsclient - Perl extension for getting Pulse Per Second data

=head1 SYNOPSIS

  use ppsclient;
  @data = ppsclient::pps_wait($pps->fileno);
  @data = ppsclient::pps_poll($pps->fileno);

=head1 DESCRIPTION

pps_wait - waits for a change in the pps assert or clear data

pps_poll - returns the current pps assert and clear data

both return an array of: status, assert sequence, assert seconds, assert nanoseconds,
clear sequence, clear seconds, clear nanoseconds

=head2 EXPORT

None by default.



=head1 SEE ALSO

ioctl, linuxpps, pps-tools

=head1 AUTHOR

Dan Drown, E<lt>dan-perl@drown.orgE<gt>

=head1 COPYRIGHT AND LICENSE

Copyright (C) 2017 by Dan Drown

This library is free software; you can redistribute it and/or modify
it under the same terms as Perl itself, either Perl version 5.24.1 or,
at your option, any later version of Perl 5 you may have available.


=cut
