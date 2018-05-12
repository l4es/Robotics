#!/usr/bin/perl
use strict;
use Expect;

#open (CODE,"| g++ -E -I ../../include - daemonizer.cpp -o sample >& compilation.error") or die;
open (CODE,"> sample.cpp")or die;
while (<DATA>){
    print CODE "$_";
}
close (CODE);

my $ret=system('g++ -I ../../include -o sample daemonizer.cpp sample.cpp >& compilation.error');
die "Compilation errors, check compilation.error file" if ($ret !=0);

chmod 0755,'sample';
unlink "compilation.error" if (-z "compilation.error");

# launch daemon
system("sample");

my $command=Expect->spawn("ps -aux | grep sample") 
    or die ("Can't execute sample, maybe because a compilation error. check compilation.error file\n");

unless ($command->expect(10,-re => 'sample')){
    unlink "sample";
    unlink "sample.cpp";
    die "Daemon not correctly launched\n";
}

unlink "sample";
unlink "sample.cpp";

print STDERR "OK\n";


__DATA__
#include "daemonizer.h" 
#include <stdio.h> 
main(){ 
  Daemonizer *daemonizer=new Daemonizer;
  int ret= daemonizer->launch();
  int pid=daemonizer->get_pid();
  fprintf(stdout,"pid of daemon=%d\n",pid);
  for (int i=0;i<1000;i++){
  }
  return ret;
}
