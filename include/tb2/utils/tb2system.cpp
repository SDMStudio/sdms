/*
 * ****** System dependent functions.
 */

#include "core/tb2types.hpp"

//Must be included after tb2types.hpp
#include "tb2system.hpp"

#ifdef LONGDOUBLE_PROB
const char* PrintFormatProb = "%Lf";
#else
const char* PrintFormatProb = "%lf";
#endif
/* --------------------------------------------------------------------
// Timer management functions
// -------------------------------------------------------------------- */
#ifndef __WIN32__
#include <unistd.h>
#include <sys/time.h>
#include <sys/resource.h>
#include <sys/times.h>

//double cpuTime()
//{
//    static struct tms buf;
//
//    times(&buf);
//    double res = ((double) (buf.tms_utime+buf.tms_stime+buf.tms_cutime+buf.tms_cstime)) / ((double) sysconf(_SC_CLK_TCK));
//    return (res>0)?res:0;
//}

double cpuTime()
{
    static struct rusage buf;

    getrusage(RUSAGE_SELF, &buf);
    double res = (double)(buf.ru_utime.tv_sec + buf.ru_stime.tv_sec) + (buf.ru_utime.tv_usec + buf.ru_stime.tv_usec) / 1000000.;
    return (res > 0) ? res : 0;
}

void timeOut(int sig)
{
    if (ToulBar2::verbose >= 0) {
        cout << endl
             << "Time limit expired... Aborting..." << endl;
        cout.flush();
    }

    if (ToulBar2::solutionFile != NULL) {
        if (ftruncate(fileno(ToulBar2::solutionFile), ftell(ToulBar2::solutionFile)))
            exit(EXIT_FAILURE);
        fclose(ToulBar2::solutionFile);
    }
    if (ToulBar2::solution_uai_file != NULL) {
        if (ftruncate(fileno(ToulBar2::solution_uai_file), ftell(ToulBar2::solution_uai_file)))
            exit(EXIT_FAILURE);
        fclose(ToulBar2::solution_uai_file);
    }

    if (ToulBar2::timeOut)
        ToulBar2::timeOut();
    else
        exit(0);
}

static struct itimerval thetimer = { { 0, 0 }, { 0, 0 } };

/* set a timer (in seconds) */
void timer(int t)
{
    ToulBar2::interrupted = false;
    signal(SIGVTALRM, timeOut);
    thetimer.it_interval.tv_sec = 0;
    thetimer.it_interval.tv_usec = 0;
    thetimer.it_value.tv_sec = t;
    thetimer.it_value.tv_usec = 0;
    setitimer(ITIMER_VIRTUAL, &thetimer, NULL);
}

/* stop the current timer */
void timerStop()
{
    thetimer.it_value.tv_sec = 0;
    thetimer.it_value.tv_usec = 0;
    setitimer(ITIMER_VIRTUAL, &thetimer, NULL);
    ToulBar2::interrupted = false;
}

#else
void timeOut(int sig) {}
double cpuTime()
{
    return (double)(clock() / CLOCKS_PER_SEC);
}
void timer(int t) {}
void timerStop() {}
#endif

/* Local Variables: */
/* c-basic-offset: 4 */
/* tab-width: 4 */
/* indent-tabs-mode: nil */
/* c-default-style: "k&r" */
/* End: */
