package symbol_processing;
use strict;

sub new {
    my $self={
	};
    bless($self);
    return $self;
};

sub add {
    my ($self,$name,$module)=@_;
    $self->{$name}=$module;
    return $module;
}

1;
