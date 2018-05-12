use strict;
use CGI;
my $q=new CGI;

print $q->header;
print $q->start_html('rouage factory');
print $q->h1('Rouage factory');
print $q->end_html;

