package robot;
use strict;

sub new {
    my $self={
	'name' => shift,
	};
    bless($self);
    return $self;
};

sub add {
    my $self=shift;
    $self->{shift}=shift;
    return 0;
}

sub get {
    my $self=shift;
    return $self->{shift};
}
1;
