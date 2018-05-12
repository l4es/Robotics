/*
 *    OpenRDK : OpenSource Robot Development Kit
 *    Copyright (C) 2007  Andrea Censi (<first_name>.<last_name>@dis.uniroma1.it)
 *
 *    This program is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

/* 
 * cpu_time() returns CPU time in seconds.
 *
 * It will be safer to do dummy call right after program starts.
 * Since cpu_time() is double, it returns with a decimal point.
 * It returns 0.0 when error occurs, but is not ordinal.
 * It works on gcc/egcs (Sun,Linux,FreeBSD), Turbo C/LSI-C (MS-DOS).
 * You can get the latest version at address follows.
 * http://www.nn.iij4u.or.jp/~tutimura/c/
 * 2002/ 3/11 (C) tutimura@nn.iij4u.or.jp :-) Feel free to use.
 */

/* How to use (not partial compile)
 *
 *  #include "cpu_time.c"
 *  ...
 *  main() {
 *      cpu_time(); <== dummy call
 *      ...
 *      printf( "time:%.2f(sec)\n", cpu_time() );
 *  }
 */

/* How to use (partial compile)
 *
 * In common headder
 *  double cpu_time( void );
 */


#ifdef TEST
#include <stdio.h>
#define TEST_MESSAGE(s) printf("by %s\n",s)
#else
#define TEST_MESSAGE(s)
#endif /* TEST */

#ifdef __STDC__
double cpu_time( void );
#endif /* __STDC__ */


#if ( (defined(MSDOS) || defined(__MSDOS__) || defined(LSI_C)) && \
      !(defined(unix) || defined(__unix)) )

        /* MS-DOS */
#include <time.h>
#if !defined(CLOCKS_PER_SEC) && defined(CLK_TCK)
#define CLOCKS_PER_SEC CLK_TCK
#endif
double cpu_time( void ) {
    clock_t t;  static clock_t last = (clock_t)-1;
    TEST_MESSAGE("clock()");
    t = clock();
    if (last == (clock_t)-1) last = t;
    return (double)(t-last)/CLOCKS_PER_SEC;
}
        /* MS-DOS */

#else /* unix */

#include <sys/time.h>
#include <sys/resource.h>
#include <unistd.h>
#ifdef RUSAGE_SELF

        /* Sun 5.5, Linux, FreeBSD, DJGPP */
double cpu_time() {
    struct rusage tmp;
    TEST_MESSAGE("getrusage()");
    if ( getrusage( RUSAGE_SELF, &tmp ) ) return 0.0;
    return (double)(tmp.ru_utime.tv_sec)
          +(double)(tmp.ru_utime.tv_usec)/1000000;
}
        /* Sun 5.5, Linux, FreeBSD, DJGPP */
#else
        /* Sun 5.3 */
#include <sys/types.h>
#include <sys/times.h>
double cpu_time() {
    struct tms tmp;
    TEST_MESSAGE("times()");
    if ( times( &tmp ) == -1 ) return 0.0;
    return (double)(tmp.tms_utime)/CLK_TCK;
}
        /* Sun 5.3 */
#endif

#endif /* unix */

#if 0
//#ifdef TEST
int main() {
    int i, j, k;

    cpu_time(); /* dummy */
    for ( i=0; i<25; i++ ) {
        for ( j=0; j<30000; j++ ) {
            k=j*j*j*j;
        }
        j=k;
        printf( "time:%.4f(sec)\n", cpu_time() );
    }
    return 0;
}
#endif /* TEST */


/*
 * This program calculates, like
 *     (double) clock()/CLOCKS_PER_SEC
 * does in ANSI, but in better way.
 */

/* This works on ...
 *
 * SUN
 * gcc 2.5.8 (SunOS 4.1.3) -- warning getrusage()
 * gcc 2.6.3 (SunOS 5.5/5.5.1) -- warning getrusage()
 * egcs 1.1.2 (SunOS 5.5/5.5.1)
 *
 * Linux
 * gcc 2.7.2.3 (Linux 2.0/2.2, glibc2.0.7)
 * egcs 1.0.3 (Linux 2.0/2.2, glibc2.0.7)
 *
 * FreeBSD
 * gcc 2.7.2.1 (FreeBSD 3.2-RELEASE)
 *
 * MS-DOS
 * DJGPP
 * Turbo C 2nd Ver.1.0
 * Turbo C++ Ver.2.0
 * Turbo C++ Ver.4.0J
 * LSI-C86 Ver.3.3c
 */
