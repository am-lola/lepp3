#ifndef LEPP3_UTILS_H
#define LEPP3_UTILS_H

namespace lepp
{

template <typename TOut, typename T>
static TOut rangeToColor(T min, T max, T value)
{
  if (max == min)
    return TOut(1.0f, 1.0f, 1.0f);

  double i = double(value - min) / (double(max - min) + 1.0);
  const double r = sin(2.0 * M_PI * i) * 0.5 + 0.5;
  const double g = sin(2.0 * M_PI * i + 2.0 * M_PI / 3.0) * 0.5 + 0.5;
  const double b = sin(2.0 * M_PI * i + 4.0 * M_PI / 3.0) * 0.5 + 0.5;

  return TOut(r, g, b);
}

} // namespace lepp

#endif // LEPP3_UTILS_H
