package wordnet;
use strict;

sub new {
    my ($pathfile)=@_;
    my $self={};
    bless($self);
    $self->load($pathfile);
    return $self;
};

sub load {
    my ($self,$pathfile)=@_;
    if (-d $pathfile) {
	# reading the original Wordnet database
      open (NOUN ,"< $pathfile/data.noun") 
	|| die "can't open $pathfile/data.noun";

      print "reading nouns...\n";

      my $wcount=0;
      my %inconnu;
      while (<NOUN>){
	# Normal data begins with a digit
	if (/^[0-9]/){
	  my @cur_noun=split / /;
	  my $concept = {
			 ID      => $cur_noun[0],
			 ORIGINE => $cur_noun[1],
			 TYPE    => $cur_noun[2],
			 COUNT   => $cur_noun[3]
			};
	  $wcount+=$concept->{COUNT};
	  my $last=0;
	  for (my $i=0;$i<$concept->{COUNT};$i++){
	    $concept->{SYSNET}[$i]=$cur_noun[4+(2*$i)];
	    $last=4+(2*$i);
	    }
	  $last+=2;
	  $concept->{REF_COUNT}=$cur_noun[$last];
	  my $hypocnt=0;

	  $last++;
	  for (my $i=0;$i<$concept->{REF_COUNT};$i++){
	    if ($cur_noun[$last] eq '~'){
	      $concept->{HYPONYM}[$hypocnt++]=$cur_noun[$last+1];
	      $last+=4;
	    }
	    elsif ($cur_noun[$last] eq '@'){
	      $concept->{HYPERNYM}=$cur_noun[$last+1];
	      $last+=4;
	    }
	    else {
	      $inconnu{$cur_noun[$last]} ++;
	      $last+=4;
		}
	  }
	  if ($cur_noun[$last] eq '|'){
	    $concept->{GLOSE}=join ' ', $cur_noun[$last..$#cur_noun];
	  }
	  $data{$concept->{ID}}= $concept;
	}
	# extract the WordNet version number
	elsif (/WordNet/ && /Copyright/){
	  /WordNet ([^ ]+) Copyright/;
	  print STDERR "WORDNET VERSION:$1\n";
	}
      }
      for my $symbol (keys %inconnu){
	print STDERR "INCONNU =>$symbol<= $inconnu{$symbol} fois\n";
      }
      my @nouns = keys %data;
      my $total= @nouns;
      print STDERR "WORDNET SYNSETS LOADED $total\n";
      print STDERR "WORDNET WORDS LOADED $wcount";

      close(NOUN);
    }
    else {
	# restoring a derived hash
	use Data::Dumper;
	$Data::Dumper::Purity = 1;
	open (FILE , "< $pathfile") || return 1;
	undef $/;
	eval <FILE>;
	return 1 if $@;
	close FILE;
    }
    return 0;
}

sub save {
    my ($self,$pathfile)=@_;
    use Data::Dumper;
    $Data::Dumper::Purity = 1;
    open (FILE , "> $pathfile") || return 1;
    print FILE Data::Dumper->Dump([\$self],['*self']);
    close FILE || return 1;;
}

1;
