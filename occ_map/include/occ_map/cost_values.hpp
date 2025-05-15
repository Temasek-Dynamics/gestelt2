#ifndef OCC_MAP__COST_VALUES_HPP_
#define OCC_MAP__COST_VALUES_HPP_
/** Provides a mapping for often used cost values */
namespace occ_map
{

static constexpr int NO_INFORMATION = 255;
static constexpr int LETHAL_OBSTACLE = 254;
static constexpr int INSCRIBED_INFLATED_OBSTACLE = 253;
static constexpr int MAX_NON_OBSTACLE = 252;
static constexpr int FREE_SPACE = 0;
}  // namespace occ_map

#endif  // OCC_MAP__COST_VALUES_HPP_
