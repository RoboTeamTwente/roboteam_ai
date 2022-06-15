# Views #

## General ##
The views are to their associated POD types what `std::string_view` is to `std::string`.

Essentially it's just a wrapper around a pointer that provides some additional functions.

For example the `RobotView` structure provides a `hasBall` function, which is called as such:

```cpp
RobotView rbt = ...; // get the robotview (from world)
rbt.hasBall(); // true or false
// access members of Robot directly by using the dereference operator.
rbt->getId(); // number of robot
```

Same thing for the `WorldDataView` and `BallView`, although `BallView` does not provide any additional functions.

## Advice ##
No advice for this one either. No performance is to be gained here.

If I had to give some I'd use `std::reference_wrapper` instead of a `T const*`, but that's all.