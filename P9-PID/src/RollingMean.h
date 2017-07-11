#ifndef ROLLINGMEAN_H
#define ROLLINGMEAN_H

/**
 * Compute statistics online as new data is added.
 *
 * See Knuth TAOCP vol 2, 3rd edition, page 232
 * See 'online algorithm' in https://en.wikipedia.org/wiki/Algorithms_for_calculating_variance
 */
class RollingMean
{
public:
    RollingMean() { reset(); }

    ~RollingMean() {}

    inline void reset();

    inline void addData(const double& data);

    long long int numData() const { return _numData; }

    double mean() const { return _mean; }

    double variance() const { return _scaledVariance / ((double)_numData - 1); }

private:
    long long int   _numData;
    double          _mean;
    double          _scaledVariance;
};

//---------------------------------------------------------------------------------------------------------------------
void RollingMean::reset()
//---------------------------------------------------------------------------------------------------------------------
{
    _numData = 0;
    _mean = 0;
    _scaledVariance = 0;
}

//---------------------------------------------------------------------------------------------------------------------
void RollingMean::addData(const double& data)
//---------------------------------------------------------------------------------------------------------------------
{
    _numData++;

    // m(k) = m(k-1) + { x(k) - m(k-1) } / k
    const double delta = data - _mean;
    const double newMean = _mean + delta/ ((double)_numData);

    // s(k) = s(k-1) + { x(k) - m(k-1) } * { x(k) - m(k) }
    double scaledVar = _scaledVariance + delta * (data - newMean);

    _mean = newMean;
    _scaledVariance = scaledVar;
}

#endif // ROLLINGMEAN_H
