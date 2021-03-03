#include "sys_time.hpp"

const Sys_time get_time()
{
    gettimeofday(&tv, &tz);
    return tv.tv_sec * 1000000 + tv.tv_usec;
}

const double get_delta_t(Sys_time now, Sys_time past)
{
    /* 1s = 10^6 us */
    return (now - past) / (double)1e+6;
}


/* ms is ok. */
/* int main()
{
    Sys_time t1 = get_time();
    cout << t1 << endl;
    int x = 0;
    for (int i = 0; i < 100000; i++)
        x++;
    Sys_time t2 = get_time();
    cout << t2 << endl;

    cout << get_delta_t(t1, t2) << endl;
    return 0;
} */