#!/usr/local/bin/perl -w
require 5.004;
use Tk 800.000;
use Tk::DialogBox;
use Tk::Tree;
use Getopt::Std;
#use OpenGL;
use strict;

# Command line switchs, and default setting
my %options=(
	     b => 0,
	     f => '.rouage_play_room',
	     h => 0,
	     l => 'rouage_play_room.log',
	     v => 'follow',
	     );

# -b  bug-trap mode
# -d: use directory, defaults to current
# -f: config file
# -h  command line options help
# -l: logfile
# -p: playroom to open
# -r: robot to load
# -v: view to use
# -w: robot to load and launch
# -x: script to execute

getopts('bd:f:hl:p:r:v:w:x:',\%options);
die Usage() if ($options{'h'});

my %interface=(
	       'newroom'   => \&newroom,
	       'openroom'   => \&openroom,
	       'closeroom'   => \&closeroom,
	       'newrobot'  => \&newrobot,
	       'openrobot'  => \&openrobot,
	       'callrobot' => \&callrobot,
	       'byerobot' => \&byerobot,
	       'exit'      => sub {exit},
);

# interface creation
my $top          = MainWindow->new();
my $menu_frame   = $top->Menu;
my $button_frame = $top->Frame(-relief => 'raised', -borderwidth => 2);
my $status_frame = $top->Frame(-relief => 'raised', -borderwidth => 2);
my $view=$top->Scrolled('Canvas',-relief=>'sunken',-background=>'white');
my $console=$top->Scrolled('Text',-relief=>'sunken',-background=>'white',-height => 5, -state => 'disabled');
my $entry=$top->Entry(-relief => 'raised', -borderwidth => 2); 

#packing the whole
$button_frame->pack(-anchor => 'nw');
$status_frame->pack(-side => 'bottom',-fill => 'x');
$entry->pack(-side => 'bottom',-fill => 'x')->focus;
$console->pack(-side => 'bottom',-fill => 'x');
$view->pack(-side => 'top',-fill => 'both',-expand => 1);

# configure
$top->title ("Rouage play room");
$top->configure(-menu => $menu_frame);
my $FOLDIMG = $top->Bitmap(-file => Tk->findINC('folder.xbm'));
my $curmenu;
open (CONFIG , "< $options{'f'}") || die "Error opening $options{'f'}\n";
while (<CONFIG>){
    s/[\r\n]//g;  # portable chomp
    SWITCH: for ($_){
	/menu=/ && do {
	    /menu=(.*)/;
	    $curmenu = $menu_frame->cascade(-label => $1);
	    last SWITCH;
	};
	/command=/ && do {
	    /command=([^,]+),([^,]+),?(.*)/;
	    $curmenu->command(
			      -label => $2,
			      -command => $interface{$1},
			      );
	    if ($3) {
		$top->Photo($3 , -file=>$3);
		$button_frame->Button(
				      -text => $2,
				      -command => $interface{$1},
				      -image => $3,
				      )->pack(-side=>'left');
	    }
	    last SWITCH;
	};
	/separator/ && do {
	    $curmenu->separator;
	    last SWITCH;
	};
	/status:/ && do {
	    /status:([^:]+):([^:]+):(.*)/;
	    $top->Photo($2 , -file=>$2);
	    $top->Photo($3 , -file=>$3);
	    $status_frame->Checkbutton(
				       -image            => $3,
				       -selectimage      => $2,
				       -indicatoron      => 0,
				       -state            => 'disabled',
				       )->pack(-side=>'left');
	    last SWITCH;
	};
	/sw -/ && do {
	    /sw -([a-zA-Z]) ?(.*)/;
	    if ($2){
		$options{$1} = $2;
	    }
	    else {
		$options{$1} = 1;
	    }
	    last SWITCH;
	};
	/./ && do {
	};
    } 
}
close (CONFIG);

# and running
MainLoop();
exit 0;


sub Usage{
    return "rouage_play_room.pl 
\t-f: config file, defaults to .rouage_play_room 
\t-d: use directory, defaults to current 
\t-p: playroom to open 
\t-r: robot to load 
\t-w: robot to load and launch (ignore -r)
\t-x: script to execute (requires -w:)
\t-v: view to use 
\t-l: logfile 
\t-b  bug-trap mode 
\t-h  command line options help 
\n"
}


sub newroom{
    require Tk::NoteBook;
    my $dlg=$top->DialogBox(
			    -title => "Room builder",
			    -buttons => [qw/OK Cancel/],
			    );
    my $nb=$dlg->NoteBook()->grid();

    my $p1=$nb->add('plane',-label => 'Floor plane');
    my $p2=$nb->add('code',-label => 'Room code');
    my $p3=$nb->add('view',-label => 'Welcome');

    my $v1;
    my $v2;
    my $v3;
    my $control;
    $v1=$p1->Scrolled('Canvas',
		      -scrollbars => 'se')->grid($v2=$p1->Scrolled('Canvas',
								    -scrollbars=>'se'));
    $v3=$p1->Scrolled('Canvas',
		      -scrollbars => 'se')->grid($control=$p1->Frame());


    $control->Button(-text=>"Floor 1")->grid($control->Button(-text=>"Floor 2"),
					  $control->Button(-text=>"Floor 3"),
					  $control->Button(-text=>"Floor 4"),
					  $control->Button(-text=>"Floor 5"),-sticky=>'nsew');


    $control->Button(-text=>"Wall 1")->grid($control->Button(-text=>"Wall 2"),
					  $control->Button(-text=>"Wall 3"),
					  $control->Button(-text=>"Wall 4"),-sticky=>'nsew');

    $control->Button(-text=>"stairs 1")->grid($control->Button(-text=>"Stairs 2"),
					      $control->Button(-text=>"Stairs 2"),
					      $control->Button(-text=>"Stairs 3"),-sticky=>'nsew');

					  
    $control->Button(-text=>"door 1")->grid($control->Button(-text=>"door 2"),
					    $control->Button(-text=>"door 3"),
					    $control->Button(-text=>"window 1"),
					    $control->Button(-text=>"window 2"),-sticky=>'nsew');
    
    $control->Button(-text=>"texture 1")->grid($control->Button(-text=>"texture 2"),
					   $control->Button(-text=>"texture 3"),
					   $control->Button(-text=>"texture 4"),
					   $control->Button(-text=>"texture 5"),-sticky=>'nsew');

    $control->Button(-text=>"rotate")->grid($control->Button(-text=>"bigger"),
					  $control->Button(-text=>"smaller"),
					  $control->Button(-text=>"Flip vert"),
					  $control->Button(-text=>"Flip horiz"),-sticky=>'nsew');
    
    $control->Button(-text=>"paper 1")->grid($control->Button(-text=>"paper 2"),
					  $control->Button(-text=>"paper 3"),
					  $control->Button(-text=>"paper 4"),
					  $control->Button(-text=>"paper 5"),-sticky=>'nsew');


					  
    $p2->Entry();
    $p3->Canvas();
    my $button = $dlg->Show;
}

sub openroom{
    my @types =(
		["Room definition",'.room','TEXT'],
		);
    my $file = $top->getOpenFile(-filetypes => \@types);    
}

sub closeroom{
    print STDERR "room closed\n";
}

sub newrobot{
    # Options tree
#    my $olddir = `cwd`;
    my $root='robot';

    my $dlg=$top->DialogBox(
			   -title => "Properties",
			   -buttons => [qw/OK Cancel/],
			   );    
    
    my $h = $dlg->Scrolled(qw\Tree -separator / -selectmode extended -width 30
			   -height 20 -indent 35 -scrollbars se
			   -itemtype imagetext \
			   )->grid(qw/-sticky nsew/);


    chdir '../robot';
    scan_module('.',$root,$h);
    chdir '../rouage_play_room';
    my $button = $dlg->Show;
 }   

sub openrobot{
    my @types =(
		["Robot definition",'.bot','TEXT'],
		);
    my $file = $top->getOpenFile(-filetypes => \@types);    
}

sub callrobot{
#    system('chdir robots');
    my @types =(
		["Robot definition",'.bot','TEXT'],
		);
    my $file = $top->getOpenFile(-filetypes => \@types);
    if ($file){
	my $pid=fork();
	system("perl -I $options{'d'} $file") if ($pid);
    }
#    system('cd ..');
}

sub byerobot{
    print STDERR "shutdown...\n";
}

sub scan_module{
    my($entry_path, $text, $h) = @_;
    if ($text ne 'CVS'){
	my $here=$h->add($entry_path,  -text => $text, -image => $FOLDIMG) ;

#       call module::Usage to capture the module option list
	$h->addchild($here,-text => 'name');
	$h->addchild($here,-text => 'verbatim');

	opendir H, $entry_path;
	my(@dirent) = grep ! /^\.\.?$/, sort(readdir H);
	closedir H;
	while ($_ = shift @dirent) {
	    my $file = "$entry_path/$_";
	    if (-d $file) {
		scan_module($file, $_, $h);
	    }    
	}
    }
}
