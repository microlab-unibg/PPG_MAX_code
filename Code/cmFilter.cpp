
#include "cmFilter.h"
#include <cstdio>
#include <cstdlib>


double DigitalFilter::pt1FilterApply(double input, lowpassFilter1p *filter)
{

    double output = filter->pt1PrevInput
            + filter->pt1K * (input - filter->pt1PrevInput);
    filter->pt1PrevInput = output;

    return output;

}


DigitalFilter::dcFilter_t DigitalFilter::dcRemoval(double x, double prev_w, double alpha)
{
    DigitalFilter::dcFilter_t filtered;
    filtered.w = x + alpha * prev_w;
    filtered.result = filtered.w - prev_w;

    return filtered;
}

double DigitalFilter::meanDiffFilter(float M, meanDiffFilter_t *filterValues)
{

    double avg = 0;

    filterValues->sum -= filterValues->values[filterValues->index];
    filterValues->values[filterValues->index] = M;
    filterValues->sum += filterValues->values[filterValues->index];

    filterValues->index++;
    filterValues->index = filterValues->index % MEAN_FILTER_SIZE;

    if (filterValues->count < MEAN_FILTER_SIZE)
        filterValues->count++;

    avg = filterValues->sum / filterValues->count;
    return avg - M;

}

#define NZEROS 4
#define NPOLES 4
#define GAIN   3.775352055e+02

static double xv[NZEROS + 1], yv[NPOLES + 1];

double DigitalFilter::lowPass(double x) {
  xv[0] = xv[1]; xv[1] = xv[2]; xv[2] = xv[3]; xv[3] = xv[4];
  xv[4] = x / GAIN;
  yv[0] = yv[1]; yv[1] = yv[2]; yv[2] = yv[3]; yv[3] = yv[4];
  yv[4] =   (xv[0] + xv[4]) + 4 * (xv[1] + xv[3]) + 6 * xv[2]
                 + ( -0.2469881426 * yv[0]) + (  1.3267482152 * yv[1])
                 + ( -2.7499775953 * yv[2]) + (  2.6278373692 * yv[3]);
  return yv[4];
}



